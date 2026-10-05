"""Arrival-time prediction from recently observed stop-to-stop travel times.

The baseline prediction ("scheduled time + current delay") assumes a bus keeps its current
delay, i.e. that every segment ahead takes as long as the timetable says. In peak traffic it
does not. Here each segment ahead takes as long as buses actually needed for it recently;
segments are keyed by their two stop ids, so lines sharing a road share observations.
"""

import bisect
import statistics
from collections import defaultdict
from datetime import datetime

from .telematics import ATHENS

WINDOW_S = 45 * 60      # observations considered: those that ended in the last WINDOW_S
MIN_SAMPLES = 2         # fewer than this -> fall back to the timetable for that segment
AVAILABLE_AFTER_S = 30  # a traversal becomes known one polling cycle after it ends


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
