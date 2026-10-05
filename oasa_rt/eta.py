"""Arrival-time prediction from recently observed stop-to-stop travel times.

The baseline prediction ("scheduled time + current delay") assumes a bus keeps its current
delay, i.e. that every segment ahead takes as long as the timetable says. In peak traffic it
does not. Here each segment ahead takes as long as buses actually needed for it recently;
segments are keyed by their two stop ids, so lines sharing a road share observations.
"""

import bisect
import statistics
from collections import defaultdict
from dataclasses import dataclass, field
from datetime import datetime

from .matcher import MAX_OFF_ROUTE_M, _xy
from .telematics import ATHENS

WINDOW_S = 45 * 60      # observations considered: those that ended in the last WINDOW_S
MIN_SAMPLES = 2         # fewer than this -> fall back to the timetable for that segment
AVAILABLE_AFTER_S = 30  # a traversal becomes known one polling cycle after it ends
MAX_FIX_GAP_S = 180     # no passage is interpolated across a longer gap between fixes
RUN_GAP_S = 600         # longer silence starts a new run
LOOP_RESTART_M = 500    # moving back this far along the route = a circular route restarting
DEPARTURE_MARGIN_M = 50 # leaving the first stop, not idling at it


class RecentSegmentTimes:
    def __init__(self, window_s=WINDOW_S, min_samples=MIN_SAMPLES):
        self.window_s = window_s
        self.min_samples = min_samples
        self._ends = defaultdict(list)       # (from_stop, to_stop) -> sorted end timestamps
        self._durations = defaultdict(list)  # same order as _ends

    def add(self, from_stop, to_stop, start_ts, end_ts):
        key = (from_stop, to_stop)
        i = bisect.bisect(self._ends[key], end_ts)
        self._ends[key].insert(i, end_ts)
        self._durations[key].insert(i, end_ts - start_ts)

    def __len__(self):
        return sum(len(ends) for ends in self._ends.values())

    def prune(self, now):
        """Forget traversals too old to matter any more (keeps a long-running feed bounded)."""
        cutoff = now - self.window_s
        for key, ends in self._ends.items():
            i = bisect.bisect_left(ends, cutoff)
            if i:
                del ends[:i]
                del self._durations[key][:i]

    def segment(self, from_stop, to_stop, now):
        """Median travel time (s) of the segment over the window before `now`, or None."""
        key = (from_stop, to_stop)
        ends = self._ends.get(key)
        if not ends:
            return None
        lo = bisect.bisect_left(ends, now - self.window_s)
        hi = bisect.bisect_left(ends, now - AVAILABLE_AFTER_S)
        if hi - lo < self.min_samples:
            return None
        return statistics.median(self._durations[key][lo:hi])


def predict_arrivals(trip, day, delay, next_index, waiting, now, model=None):
    """Yield (stop_index, stop_id, predicted arrival timestamp) for the stops still ahead.

    With model=None every segment takes its scheduled time, which is exactly the GTFS-RT
    convention of propagating the current delay ("scheduled time + delay").
    """
    midnight = datetime(day.year, day.month, day.day, tzinfo=ATHENS).timestamp()
    st = trip.stop_times
    if waiting:
        t = max(now, midnight + st[0][3])
        yield 0, st[0][1], t
        first, done = 1, 0.0
    else:
        prev, nxt = st[next_index - 1], st[next_index]
        span = nxt[2] - prev[3]
        # Share of the current segment already covered, as implied by the delay.
        done = min(1.0, max(0.0, (now - delay - midnight - prev[3]) / span)) if span > 0 else 1.0
        t, first = now, next_index
    for k in range(first, len(st)):
        a, b = st[k - 1], st[k]
        seconds = model.segment(a[1], b[1], now) if model else None
        if seconds is None:
            seconds = b[2] - a[3]
        if k == first and not waiting:
            seconds *= 1.0 - done
        t += seconds
        yield k, b[1], t


def route_geometry(gtfs, mapper, matcher):
    """(geometry, stop ids) of the most common stop pattern of a live route code, cached."""
    cache = {}

    def get(line, route_code):
        if (line, route_code) not in cache:
            shapes = mapper.shapes_for(line, route_code)
            trips = [t for t in gtfs.trips_for_line(line) if t.shape_id in shapes]
            result = None
            if trips:
                counts = defaultdict(int)
                for t in trips:
                    counts[t.pattern] += 1
                pattern = max(counts, key=counts.get)
                geo = matcher.geometry(next(t for t in trips if t.pattern == pattern))
                result = (geo, list(pattern)) if geo else None
            cache[(line, route_code)] = result
        return cache[(line, route_code)]
    return get


@dataclass
class _Run:
    route_code: str
    line: str
    geo: object
    stop_ids: list
    last_ts: float
    along: float = None
    passed: dict = field(default_factory=dict)  # stop index -> passage timestamp


class PassageTracker:
    """Follows each vehicle along its route from its GPS fixes and detects when it passes
    each stop, interpolating between consecutive fixes. Stop-to-stop traversals are added to
    `segments` (a RecentSegmentTimes) when given. Used live and to build replay ground truth."""

    def __init__(self, geometry_of, segments=None):
        self.geometry_of = geometry_of
        self.segments = segments
        self.runs = {}

    def observe(self, veh, line, route_code, ts, lat, lon, heading=None):
        run = self.runs.get(veh)
        if run is not None and ts <= run.last_ts:
            return                                  # same GPS fix as before
        if run is None or run.route_code != route_code or ts - run.last_ts > RUN_GAP_S:
            found = self.geometry_of(line, route_code)
            if found is None:
                self.runs.pop(veh, None)
                return
            run = self.runs[veh] = _Run(route_code, line, found[0], found[1], ts)
        cands = run.geo.line.candidates(*_xy(lat, lon), MAX_OFF_ROUTE_M, heading or None)
        if not cands:
            return                                  # off the route for now
        prev = run.along
        along = min((c[0] for c in cands), key=lambda a: abs(a - prev) if prev is not None else a)
        if prev is not None and along < prev - LOOP_RESTART_M:
            run.passed, prev = {}, None             # a circular route starting over
        self.on_position(veh, route_code, ts, along)
        if prev is not None and ts - run.last_ts <= MAX_FIX_GAP_S:
            for k, (sid, pos) in enumerate(zip(run.stop_ids, run.geo.stop_along)):
                threshold = pos + (DEPARTURE_MARGIN_M if k == 0 else 0)
                if k in run.passed or not prev < threshold <= along:
                    continue
                t = run.last_ts + (threshold - prev) / (along - prev) * (ts - run.last_ts)
                run.passed[k] = t
                self.on_passage(veh, run.line, route_code, k, sid, t)
                if self.segments is not None and k - 1 in run.passed and t > run.passed[k - 1]:
                    self.segments.add(run.stop_ids[k - 1], sid, run.passed[k - 1], t)
        run.along, run.last_ts = along, ts

    def on_position(self, veh, route_code, ts, along):
        pass

    def on_passage(self, veh, line, route_code, stop_index, stop_id, ts):
        pass
