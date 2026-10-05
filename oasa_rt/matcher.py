"""Matches live OASA vehicles to scheduled GTFS trips.

The telematics API reports vehicles per *route code* with no trip identifier, so each
vehicle is assigned to a scheduled trip by comparing where it is along the route with
where each candidate trip should be at that moment.

Positions along a trip are measured on the GTFS shape (shapes.txt). Trips whose shape is
missing or does not fit their stops fall back to straight lines between consecutive stops.
"""

import bisect
import math
import statistics
from collections import Counter
from dataclasses import dataclass, field, replace
from datetime import datetime

from .telematics import ATHENS, parse_cs_date

EARTH_RADIUS = 6371000.0
LAT0 = math.radians(37.98)          # local equirectangular projection centred on Athens
MAX_OFF_ROUTE_M = 250               # farther than this from the trip geometry -> not on this trip
STOP_SNAP_M = 150                   # a stop farther than this from its shape -> shape unusable
TERMINAL_RADIUS_M = 100             # "waiting at the first stop"
MIN_DELAY, MAX_DELAY = -10 * 60, 60 * 60
STICKY_BONUS = 180                  # prefer keeping last cycle's assignment (seconds of cost)
WRONG_HEADING_COST = 300            # vehicle heading opposes the direction of the route there
MIN_PATTERN_SIMILARITY = 0.6
INFEASIBLE, UNMATCHED = 1e9, 1e6    # Hungarian assignment: impossible pair / vehicle left unmatched
HISTORY_S = 15 * 60                 # MemoryMatcher: positions considered when scoring a trip
INCONSISTENT_FIX_COST = 3600        # a past position that does not fit the trip at all
RUN_GAP_S = 10 * 60                 # longer silence -> a new run
ANCHOR_BONUS = 1800                 # departure-anchored trip; outweighs any position-based cost
DEPARTED_M = 150                    # this far from the first stop counts as departed
ANCHOR_EARLY_S, ANCHOR_LATE_S = 5 * 60, 15 * 60
# HMMMatcher, in units of -log probability; one unit = HMM_TAU seconds of position cost.
HMM_TAU = 300.0
HMM_SWITCH_MIDRUN = 9.0             # changing trip in the middle of a run
HMM_SWITCH_TERMINAL = 1.0           # changing trip at a first stop or on a new route code
HMM_NONE = 12.0                     # per observation, "running no scheduled trip"
HMM_MISSING = 15.0                  # a tracked trip that no position fits any more
HMM_PRUNE = 30.0
HMM_LATE_FREE = 10 * 60             # lateness up to this is unremarkable and costs nothing
HMM_EARLY_FREE = 5 * 60             # en route, running this much ahead of the timetable is free too
HMM_DEPARTURE_TAU = 60.0            # at an observed departure, each minute off schedule = 1 unit


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


@dataclass
class Candidate:
    cost: float             # what the assignment minimises
    raw_cost: float         # position-based cost before stickiness
    vid: str
    trip: object
    day: object
    delay: int
    next_index: int
    waiting: bool
    progress: float
    sticky: bool


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
        matches = {}
        for v in vehicles:
            m = Match(v["VEH_NO"], line, self.gtfs.route_id_for(v["LINE_CODE"], line), v["ROUTE_CODE"],
                      float(v["CS_LAT"]), float(v["CS_LNG"]),
                      float(v.get("VEH_HEADING") or 0), parse_cs_date(v["CS_DATE"]))
            matches[m.vehicle_id] = m
        candidates = [c for m in matches.values() for c in self._candidates(line, m)]

        progress_of = {}
        for c in self._assign(candidates):
            m = matches[c.vid]
            m.trip, m.service_day, m.delay, m.next_index, m.waiting_at_start = (
                c.trip, c.day, c.delay, c.next_index, c.waiting)
            m.route_id = c.trip.route_id
            progress_of[c.vid] = c.progress

        for vid, m in matches.items():
            if m.trip is not None:
                self.previous[vid] = (m.trip.trip_id, progress_of[vid])
            else:
                self.previous.pop(vid, None)
        return list(matches.values())

    def _candidates(self, line, m):
        """Every trip the vehicle could be running, scored from its current position."""
        pos = _xy(m.lat, m.lon)
        heading = m.bearing or None     # 0 means "unknown" more often than "due north"
        shapes = self.mapper.shapes_for(line, m.route_code)
        prev_trip, prev_progress = self.previous.get(m.vehicle_id, (None, 0.0))
        on_line = {}                    # Polyline id -> candidates, shared by trips on one shape
        out = []
        for trip, day, midnight in self.gtfs.candidate_trips(line, m.timestamp):
            if trip.shape_id not in shapes:
                continue
            geo = self.geometry(trip)
            if geo is None:
                continue
            if id(geo.line) not in on_line:
                on_line[id(geo.line)] = geo.line.candidates(*pos, MAX_OFF_ROUTE_M, heading)
            sticky = trip.trip_id == prev_trip
            best = self._score(trip, midnight, geo, on_line[id(geo.line)], m.timestamp, pos,
                               prev_progress if sticky else None)
            if best:
                raw, delay, next_index, waiting, progress = best
                out.append(Candidate(raw - (STICKY_BONUS if sticky else 0), raw, m.vehicle_id, trip, day,
                                     delay, next_index, waiting, progress, sticky))
        return out

    def _score(self, trip, midnight, geo, positions, timestamp, pos, prev_progress=None):
        """Cheapest way the vehicle at `pos` at `timestamp` can be running `trip`:
        (cost, delay, next_index, waiting, progress), or None if no position fits."""
        secs = (timestamp - midnight).total_seconds()
        best = None
        for along, _, wrong_heading in positions:
            progress = geo.progress(along)
            if prev_progress is not None and progress < prev_progress - 1:
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
            if best is None or cost < best[0]:
                best = (cost, int(round(delay)), seg + 1, waiting, progress)
        return best

    @staticmethod
    def _assign(candidates):
        """Greedy one-to-one assignment, cheapest pair first."""
        chosen, used_trips, used_vehicles = [], set(), set()
        for c in sorted(candidates, key=lambda c: c.cost):
            key = (c.trip.trip_id, c.day)
            if c.vid in used_vehicles or key in used_trips:
                continue
            used_trips.add(key)
            used_vehicles.add(c.vid)
            chosen.append(c)
        return chosen


class HungarianMatcher(Matcher):
    """Same costs, but the one-to-one assignment minimises the total cost of a line at once,
    so one vehicle cannot take the only trip another vehicle could be running."""

    def _unmatched_cost(self, vid):
        return UNMATCHED

    def _assign(self, candidates):
        import numpy as np
        from scipy.optimize import linear_sum_assignment

        if not candidates:
            return []
        vehicles = sorted({c.vid for c in candidates})
        trips = sorted({(c.trip.trip_id, c.day) for c in candidates})
        vi = {v: i for i, v in enumerate(vehicles)}
        ti = {t: j for j, t in enumerate(trips)}
        n, k = len(vehicles), len(trips)
        # Columns: one per trip, plus one "unmatched" column per vehicle. UNMATCHED dwarfs every
        # real cost, so as many vehicles as possible are matched, then the total cost is minimised.
        cost = np.full((n, k + n), INFEASIBLE)
        for i, v in enumerate(vehicles):
            cost[i, k + i] = self._unmatched_cost(v)
        by_cell = {}
        for c in candidates:
            i, j = vi[c.vid], ti[(c.trip.trip_id, c.day)]
            if c.cost < cost[i, j]:
                cost[i, j] = c.cost
                by_cell[(i, j)] = c
        rows, cols = linear_sum_assignment(cost)
        return [by_cell[(i, j)] for i, j in zip(rows, cols) if (i, j) in by_cell]


@dataclass
class _Track:
    route_code: str
    fixes: list = field(default_factory=list)   # (timestamp, xy, heading) of the current run
    last_at_terminal: object = None              # timestamp, while the departure is pending
    anchor: tuple = None                         # (trip_id, service_day) fixed at departure


class MemoryMatcher(HungarianMatcher):
    """Hungarian assignment with memory.

    * A vehicle seen leaving its first stop is anchored to the trip scheduled to depart then,
      for the rest of the run: overtaking on the road cannot swap two trips any more.
    * Otherwise a trip is scored over the vehicle's last HISTORY_S seconds of positions (median
      cost), not only the current one, so a single ambiguous position cannot flip it.
    """

    def __init__(self, gtfs, mapper, use_shapes=True):
        super().__init__(gtfs, mapper, use_shapes)
        self.tracks = {}            # vehicle_id -> _Track
        self._positions = {}        # (vehicle_id, timestamp, Polyline id) -> candidates

    def _candidates(self, line, m):
        base = super()._candidates(line, m)
        track = self._update_track(m, base)
        if not base:
            return base
        out = []
        for c in base:
            midnight = datetime(c.day.year, c.day.month, c.day.day, tzinfo=ATHENS)
            geo = self.geometry(c.trip)
            costs = [c.raw_cost]
            for ts, pos, heading in track.fixes[:-1]:
                key = (m.vehicle_id, ts, id(geo.line))
                if key not in self._positions:
                    self._positions[key] = geo.line.candidates(*pos, MAX_OFF_ROUTE_M, heading)
                scored = self._score(c.trip, midnight, geo, self._positions[key], ts, pos)
                costs.append(scored[0] if scored else INCONSISTENT_FIX_COST)
            cost = statistics.median(costs)
            if c.sticky:
                cost -= STICKY_BONUS
            if track.anchor == (c.trip.trip_id, c.day):
                cost -= ANCHOR_BONUS
            out.append(replace(c, cost=cost))
        return out

    def _update_track(self, m, base):
        """Keep the vehicle's recent positions for this run, and detect its departure."""
        track = self.tracks.get(m.vehicle_id)
        pos = _xy(m.lat, m.lon)
        if (track is None or track.route_code != m.route_code
                or (track.fixes and (m.timestamp - track.fixes[-1][0]).total_seconds() > RUN_GAP_S)):
            track = self.tracks[m.vehicle_id] = _Track(m.route_code)
        if not track.fixes or track.fixes[-1][0] != m.timestamp:
            track.fixes.append((m.timestamp, pos, m.bearing or None))
        track.fixes = [f for f in track.fixes if (m.timestamp - f[0]).total_seconds() <= HISTORY_S]

        near = _first_stop_distance(self.gtfs, pos, base)
        if near is not None and near <= TERMINAL_RADIUS_M:
            # At the first stop: a new run is about to start, so forget the previous one.
            track.last_at_terminal, track.anchor = m.timestamp, None
            track.fixes = track.fixes[-1:]
        elif track.last_at_terminal is not None and near is not None and near > DEPARTED_M:
            departure = track.last_at_terminal + (m.timestamp - track.last_at_terminal) / 2
            track.anchor = self._anchor_trip(base, departure)
            track.last_at_terminal = None
        return track

    @staticmethod
    def _anchor_trip(base, departure):
        """The trip scheduled to leave closest to the observed departure (late is likelier)."""
        best = None
        for c in base:
            midnight = datetime(c.day.year, c.day.month, c.day.day, tzinfo=ATHENS)
            late = (departure - midnight).total_seconds() - c.trip.start
            if not -ANCHOR_EARLY_S <= late <= ANCHOR_LATE_S:
                continue
            score = late if late >= 0 else -3 * late
            if best is None or score < best[0]:
                best = (score, (c.trip.trip_id, c.day))
        return best[1] if best else None


def _first_stop_distance(gtfs, pos, candidates):
    """Distance (m) from `pos` to the nearest first stop of the candidate trips, or None."""
    stops = {c.trip.stop_times[0][1] for c in candidates}
    dists = [math.dist(pos, _xy(*gtfs.stops[s][:2])) for s in stops if s in gtfs.stops]
    return min(dists) if dists else None


@dataclass
class _HMMState:
    route_code: str
    timestamp: object
    scores: dict            # (trip_id, service_day) -> -log score of the best path ending there
    none: float             # same, for "running no scheduled trip"
    last_at_terminal: object = None   # last fix at a first stop, while the departure is pending


class HMMMatcher(HungarianMatcher):
    """Online Viterbi per vehicle over "which trip is it running", then Hungarian per line.

    Emissions come from the position-based cost of each trip at each new GPS fix. Staying on a
    trip is free; changing trip costs HMM_SWITCH_MIDRUN in the middle of a run and only
    HMM_SWITCH_TERMINAL at a first stop or when the vehicle's route code changes. A vehicle's
    trip therefore follows the evidence of its whole run, not just its latest position.
    """

    def __init__(self, gtfs, mapper, use_shapes=True, late_free=HMM_LATE_FREE, early_free=HMM_EARLY_FREE):
        super().__init__(gtfs, mapper, use_shapes)
        self.late_free, self.early_free = late_free, early_free
        self.states = {}
        self._none_cost = {}

    def _unmatched_cost(self, vid):
        return self._none_cost.get(vid, UNMATCHED)

    def _candidates(self, line, m):
        base = super()._candidates(line, m)
        by_key = {(c.trip.trip_id, c.day): c for c in base}
        prev = self.states.get(m.vehicle_id)
        new_run = (prev is None or prev.route_code != m.route_code
                   or (m.timestamp - prev.timestamp).total_seconds() > RUN_GAP_S)
        if prev is not None and not new_run and prev.timestamp == m.timestamp:
            state = prev                                # same GPS fix as last cycle: nothing new
        else:
            state = self._update(m, base, by_key, None if new_run else prev)
        self.states[m.vehicle_id] = state
        self._none_cost[m.vehicle_id] = state.none * HMM_TAU
        return [replace(c, cost=state.scores[key] * HMM_TAU - (STICKY_BONUS if c.sticky else 0))
                for key, c in by_key.items() if key in state.scores]

    def _update(self, m, base, by_key, prev):
        """One Viterbi step for the vehicle's new GPS fix (prev=None starts a new run)."""
        near = _first_stop_distance(self.gtfs, _xy(m.lat, m.lon), base)
        at_first_stop = near is not None and near <= TERMINAL_RADIUS_M
        last_at_terminal = prev.last_at_terminal if prev else None
        departure = None
        if at_first_stop:
            last_at_terminal = m.timestamp
        elif last_at_terminal is not None and near is not None and near > DEPARTED_M:
            departure = last_at_terminal + (m.timestamp - last_at_terminal) / 2
            last_at_terminal = None
        changing_allowed = prev is None or at_first_stop or departure is not None
        switch = HMM_SWITCH_TERMINAL if changing_allowed else HMM_SWITCH_MIDRUN
        prev_scores, prev_none = (prev.scores, prev.none) if prev else ({}, 0.0)
        enter = min([prev_none, *prev_scores.values()]) + switch
        scores = {}
        for key, c in by_key.items():
            stay = prev_scores.get(key)
            scores[key] = self._emission(c, departure) + (enter if stay is None else min(stay, enter))
        for key, stay in prev_scores.items():
            if key not in scores:
                scores[key] = HMM_MISSING + min(stay, enter)
        none = HMM_NONE + min(prev_none, enter)
        lowest = min([none, *scores.values()])
        scores = {k: v - lowest for k, v in scores.items() if v - lowest <= HMM_PRUNE}
        return _HMMState(m.route_code, m.timestamp, scores, none - lowest, last_at_terminal)

    def _emission(self, c, departure=None):
        """-log likelihood of the current observation under trip c.

        On the road, buses run late relative to the timetable all the time, so moderate lateness
        is free and being early stays expensive. At an observed departure from the first stop,
        though, the departure time pins the trip down: every minute off its schedule counts."""
        cost = c.raw_cost
        if not c.waiting:
            # raw_cost is delay (late) or 3 x |delay| (early); forgive the unremarkable band.
            cost -= min(c.delay, self.late_free) if c.delay >= 0 else 3 * min(-c.delay, self.early_free)
        units = cost / HMM_TAU
        if departure is not None:
            midnight = datetime(c.day.year, c.day.month, c.day.day, tzinfo=ATHENS)
            late = (departure - midnight).total_seconds() - c.trip.start
            units += (late if late >= 0 else -3 * late) / HMM_DEPARTURE_TAU
        return units
