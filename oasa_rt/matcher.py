"""Matches live OASA vehicles to scheduled GTFS trips.

The telematics API reports vehicles per *route code* with no trip identifier, so each
vehicle is assigned to a scheduled trip by comparing where it is along the route with
where each candidate trip should be at that moment.

Positions along a trip are measured on the GTFS shape (shapes.txt). Trips whose shape is
missing or does not fit their stops fall back to straight lines between consecutive stops.
"""

import bisect
import math
from collections import Counter
from dataclasses import dataclass

from .telematics import parse_cs_date

EARTH_RADIUS = 6371000.0
LAT0 = math.radians(37.98)          # local equirectangular projection centred on Athens
MAX_OFF_ROUTE_M = 250               # farther than this from the trip geometry -> not on this trip
STOP_SNAP_M = 150                   # a stop farther than this from its shape -> shape unusable
TERMINAL_RADIUS_M = 100             # "waiting at the first stop"
MIN_DELAY, MAX_DELAY = -10 * 60, 60 * 60
STICKY_BONUS = 180                  # prefer keeping last cycle's assignment (seconds of cost)
WRONG_HEADING_COST = 300            # vehicle heading opposes the direction of the route there
MIN_PATTERN_SIMILARITY = 0.6


def _xy(lat, lon):
    return (math.radians(lon) * EARTH_RADIUS * math.cos(LAT0), math.radians(lat) * EARTH_RADIUS)


class Polyline:
    def __init__(self, points_xy):
        self.xy = points_xy
        self.cum = [0.0]
        self.bearing = []
        for a, b in zip(points_xy, points_xy[1:]):
            self.cum.append(self.cum[-1] + math.dist(a, b))
            self.bearing.append(math.degrees(math.atan2(b[0] - a[0], b[1] - a[1])) % 360)

    @property
    def length(self):
        return self.cum[-1]

    def candidates(self, x, y, max_dist, heading=None):
        """Every separate pass of the line within `max_dist` of (x, y).

        Returns [(distance_along, distance_off, wrong_heading)], one per run of consecutive
        nearby segments, so a loop that passes the same street twice yields two candidates.
        """
        result, run = [], None
        for i in range(len(self.xy) - 1):
            (ax, ay), (bx, by) = self.xy[i], self.xy[i + 1]
            dx, dy = bx - ax, by - ay
            length2 = dx * dx + dy * dy
            t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / length2))
            d = math.hypot(x - (ax + t * dx), y - (ay + t * dy))
            if d > max_dist:
                run = None
                continue
            wrong = heading is not None and length2 > 0 and \
                abs((heading - self.bearing[i] + 180) % 360 - 180) > 90
            cand = (self.cum[i] + t * (self.cum[i + 1] - self.cum[i]), d, wrong)
            if run is None:
                run = len(result)
                result.append(cand)
            elif d < result[run][1]:
                result[run] = cand
        return result


@dataclass
class Geometry:
    line: Polyline
    stop_along: list                # distance along `line` of every stop of the pattern
    from_shape: bool

    def progress(self, along):
        """Fractional stop index (2.5 = halfway between the 3rd and 4th stop)."""
        a = self.stop_along
        seg = min(max(bisect.bisect_right(a, along) - 1, 0), len(a) - 2)
        span = a[seg + 1] - a[seg]
        t = 0.0 if span <= 0 else max(0.0, min(1.0, (along - a[seg]) / span))
        return seg + t


@dataclass
class Match:
    vehicle_id: str
    line: str                       # line number, e.g. "040"
    route_id: str                   # GTFS route_id (of the trip once matched)
    route_code: str
    lat: float
    lon: float
    bearing: float
    timestamp: object               # aware datetime of the GPS fix
    trip: object = None             # static.Trip or None
    service_day: object = None
    delay: int = 0                  # seconds, positive = late
    next_index: int = 0             # index in trip.stop_times of the next stop
    waiting_at_start: bool = False


class RouteMapper:
    """Maps a live telematics route code to the GTFS shape ids it corresponds to.

    Route codes in the live API and shape ids in the GTFS export usually coincide but not
    always (e.g. line 040 runs on 3922/3923/3924 while the GTFS uses 5512/5513/5535), so the
    mapping is made by comparing the stop sequences.
    """

    def __init__(self, gtfs, telematics):
        self.gtfs = gtfs
        self.tel = telematics
        self._cache = {}

    def shapes_for(self, line, route_code):
        key = (line, route_code)
        if key in self._cache:
            return self._cache[key]
        by_shape = {}
        for trip in self.gtfs.trips_for_line(line):
            by_shape.setdefault(trip.shape_id, Counter())[trip.pattern] += 1
        if route_code in by_shape:
            result = {route_code}
        else:
            live = {s["StopCode"] for s in self.tel.route_stops(route_code)}
            result = set()
            for shape_id, patterns in by_shape.items():
                stops = set(patterns.most_common(1)[0][0])
                if live and len(live & stops) / max(len(live), len(stops)) >= MIN_PATTERN_SIMILARITY:
                    result.add(shape_id)
        self._cache[key] = result
        return result


class Matcher:
    def __init__(self, gtfs, mapper, use_shapes=True):
        self.gtfs = gtfs
        self.mapper = mapper
        self.use_shapes = use_shapes
        self.previous = {}          # vehicle_id -> (trip_id, progress)
        self._shape_lines = {}
        self._geometry = {}

    def geometry(self, trip):
        key = (trip.shape_id, trip.pattern)
        if key not in self._geometry:
            self._geometry[key] = self._build_geometry(trip)
        return self._geometry[key]

    def _build_geometry(self, trip):
        stops_xy = [_xy(*self.gtfs.stops[s][:2]) if s in self.gtfs.stops else None for s in trip.pattern]
        if any(p is None for p in stops_xy):
            return None
        if self.use_shapes and trip.shape_id in self.gtfs.shapes:
            line = self._shape_lines.get(trip.shape_id)
            if line is None:
                line = self._shape_lines[trip.shape_id] = Polyline(
                    [_xy(lat, lon) for lat, lon in self.gtfs.shapes[trip.shape_id]])
            along = self._snap_stops(line, stops_xy)
            if along is not None:
                return Geometry(line, along, True)
        line = Polyline(stops_xy)
        return Geometry(line, list(line.cum), False)

    @staticmethod
    def _snap_stops(line, stops_xy):
        """Position of each stop along the shape, in order (Viterbi over each stop's candidates)."""
        layers = []
        for x, y in stops_xy:
            cands = [(along, d) for along, d, _ in line.candidates(x, y, STOP_SNAP_M)]
            if not cands:
                return None
            layers.append(cands)
        cost = [d for _, d in layers[0]]
        back = []
        for prev, cur in zip(layers, layers[1:]):
            new_cost, pointers = [], []
            for along, d in cur:
                options = [(cost[k], k) for k, (p_along, _) in enumerate(prev) if p_along <= along + 1e-6]
                c, k = min(options) if options else (math.inf, None)
                new_cost.append(c + d)
                pointers.append(k)
            cost = new_cost
            back.append(pointers)
        best = min(range(len(cost)), key=cost.__getitem__)
        if math.isinf(cost[best]):
            return None
        path = [best]
        for pointers in reversed(back):
            path.append(pointers[path[-1]])
        path.reverse()
        return [layers[i][k][0] for i, k in enumerate(path)]

    def match_line(self, line, vehicles):
        """vehicles: getBusLocation rows of every route of one line number, each tagged with
        the LINE_CODE it was polled under."""
        matches, candidates = {}, []
        for v in vehicles:
            vid = v["VEH_NO"]
            m = Match(vid, line, self.gtfs.route_id_for(v["LINE_CODE"], line), v["ROUTE_CODE"],
                      float(v["CS_LAT"]), float(v["CS_LNG"]),
                      float(v.get("VEH_HEADING") or 0), parse_cs_date(v["CS_DATE"]))
            matches[vid] = m
            pos = _xy(m.lat, m.lon)
            heading = m.bearing or None     # 0 means "unknown" more often than "due north"
            shapes = self.mapper.shapes_for(line, m.route_code)
            prev_trip, prev_progress = self.previous.get(vid, (None, 0.0))
            on_line = {}                    # Polyline id -> candidates, shared by trips on one shape
            for trip, day, midnight in self.gtfs.candidate_trips(line, m.timestamp):
                if trip.shape_id not in shapes:
                    continue
                geo = self.geometry(trip)
                if geo is None:
                    continue
                if id(geo.line) not in on_line:
                    on_line[id(geo.line)] = geo.line.candidates(*pos, MAX_OFF_ROUTE_M, heading)
                sticky = trip.trip_id == prev_trip
                secs = (m.timestamp - midnight).total_seconds()
                best = None
                for along, _, wrong_heading in on_line[id(geo.line)]:
                    progress = geo.progress(along)
                    if sticky and progress < prev_progress - 1:
                        continue            # buses do not drive backwards along their route
                    seg = int(progress)
                    if seg >= len(trip.stop_times) - 1:
                        seg = len(trip.stop_times) - 2
                    t = progress - seg
                    stop, nxt = trip.stop_times[seg], trip.stop_times[seg + 1]
                    waiting = (seg == 0 and secs < trip.start
                               and math.dist(pos, _xy(*self.gtfs.stops[stop[1]][:2])) <= TERMINAL_RADIUS_M)
                    if waiting:
                        delay, cost = 0, (trip.start - secs) * 0.5
                    else:
                        delay = secs - (stop[3] + t * (nxt[2] - stop[3]))
                        if not MIN_DELAY <= delay <= MAX_DELAY:
                            continue
                        # Buses run late far more often than early.
                        cost = delay if delay >= 0 else -3 * delay
                    if wrong_heading:
                        cost += WRONG_HEADING_COST
                    if sticky:
                        cost -= STICKY_BONUS
                    if best is None or cost < best[0]:
                        best = (cost, int(round(delay)), seg + 1, waiting, progress)
                if best:
                    cost, delay, next_index, waiting, progress = best
                    candidates.append((cost, vid, trip, day, delay, next_index, waiting, progress))

        # Greedy one-to-one assignment, cheapest first.
        used_trips, progress_of = set(), {}
        for cost, vid, trip, day, delay, next_index, waiting, progress in sorted(candidates, key=lambda c: c[0]):
            m = matches[vid]
            key = (trip.trip_id, day)
            if m.trip is not None or key in used_trips:
                continue
            used_trips.add(key)
            m.trip, m.service_day, m.delay, m.next_index, m.waiting_at_start = trip, day, delay, next_index, waiting
            m.route_id = trip.route_id
            progress_of[vid] = progress

        for vid, m in matches.items():
            if m.trip is not None:
                self.previous[vid] = (m.trip.trip_id, progress_of[vid])
            else:
                self.previous.pop(vid, None)
        return list(matches.values())
