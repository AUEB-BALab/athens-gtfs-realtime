"""Replays a recording made with oasa_rt.record through several matchers and scores them
against what the buses actually did.

    python -m oasa_rt.replay data/record-2026-10-05.sqlite

Ground truth comes from the recorded GPS fixes themselves: each vehicle's run is tracked along
its route and the moment it passes every stop is interpolated between consecutive fixes.
That gives, independently of any matcher:
  * actual stop passages, to score arrival predictions (ours, OASA's own, schedule only);
  * observed departures from the first stop, whose closest scheduled trip is a good proxy for
    the trip a vehicle was really running;
  * overtakes: two vehicles on the same route swapping order.
"""

import argparse
import bisect
import sqlite3
import statistics
from collections import defaultdict
from datetime import datetime

from .eta import RecentSegmentTimes, predict_arrivals
from .matcher import MAX_OFF_ROUTE_M, HMMMatcher, HungarianMatcher, Matcher, MemoryMatcher, RouteMapper, _xy
from .static import StaticGTFS
from .telematics import ATHENS, Telematics

HORIZONS = [(0, 5), (5, 10), (10, 20), (20, 30)]    # minutes ahead
MAX_FIX_GAP_S = 180          # no passage is interpolated across a longer gap between fixes
RUN_GAP_S = 600
OVERTAKE_MARGIN_M = 150      # both before and after the swap, to ignore GPS noise


def cs_date(ts):
    return datetime.fromtimestamp(ts, ATHENS).strftime("%b %d %Y %I:%M:%S:000%p")


def load(path):
    db = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    rows = db.execute("""
        SELECT m.cycle, m.line, m.veh, m.route_code, f.ts, f.lat, f.lon, f.heading, f.line_code
        FROM match m JOIN fix f ON f.veh = m.veh AND f.ts = m.fix_ts
        ORDER BY m.cycle, m.line, m.veh""").fetchall()
    cycles = defaultdict(lambda: defaultdict(list))
    fixes = defaultdict(dict)
    for cycle, line, veh, route, ts, lat, lon, heading, line_code in rows:
        cycles[cycle][line].append({"VEH_NO": veh, "CS_DATE": cs_date(ts), "CS_LAT": lat, "CS_LNG": lon,
                                    "ROUTE_CODE": route, "VEH_HEADING": heading, "LINE_CODE": line_code})
        fixes[veh][ts] = (line, route, lat, lon, heading)
    eta = db.execute("SELECT polled, stop_id, veh, minutes FROM oasa_eta").fetchall()
    return cycles, fixes, eta


# ---------------------------------------------------------------- ground truth from GPS

class Truth:
    def __init__(self, fixes, gtfs, mapper, geometry_of):
        self.passages = defaultdict(list)   # (veh, stop_id) -> sorted passage timestamps
        self.departures = []                # (veh, line, route_code, departure_ts)
        self.along = defaultdict(dict)      # veh -> {ts: (route_code, distance along route)}
        self.segments = RecentSegmentTimes() # stop-to-stop traversals, for arrival predictions
        for veh, by_ts in fixes.items():
            run = []
            for ts in sorted(by_ts):
                line, route, lat, lon, heading = by_ts[ts]
                if run and (route != run[-1][2] or ts - run[-1][0] > RUN_GAP_S):
                    self._run(veh, run, gtfs, mapper, geometry_of)
                    run = []
                run.append((ts, line, route, lat, lon, heading))
            if run:
                self._run(veh, run, gtfs, mapper, geometry_of)
        for times in self.passages.values():
            times.sort()

    def _run(self, veh, run, gtfs, mapper, geometry_of):
        line, route = run[0][1], run[0][2]
        found = geometry_of(line, route)
        if found is None:
            return
        geo, stop_ids = found
        track, prev = [], None
        for ts, _, _, lat, lon, heading in run:
            cands = geo.line.candidates(*_xy(lat, lon), MAX_OFF_ROUTE_M, heading or None)
            if not cands:
                continue
            along = min((c[0] for c in cands), key=lambda a: abs(a - prev) if prev is not None else a)
            if prev is not None and along < prev - 500:
                self._passages(veh, line, route, track, geo, stop_ids)   # circular route restarted
                track = []
            track.append((ts, along))
            self.along[veh][ts] = (route, along)
            prev = along
        self._passages(veh, line, route, track, geo, stop_ids)

    def _passages(self, veh, line, route, track, geo, stop_ids):
        seen = {}
        for k, (sid, pos) in enumerate(zip(stop_ids, geo.stop_along)):
            threshold = pos + (50 if k == 0 else 0)     # leaving the first stop, not idling at it
            for (t0, a0), (t1, a1) in zip(track, track[1:]):
                if a0 < threshold <= a1 and t1 - t0 <= MAX_FIX_GAP_S:
                    t = t0 + (threshold - a0) / (a1 - a0) * (t1 - t0)
                    self.passages[(veh, sid)].append(t)
                    seen[k] = (sid, t)
                    if k - 1 in seen and t > seen[k - 1][1]:
                        self.segments.add(seen[k - 1][0], sid, seen[k - 1][1], t)
                    if k == 0:
                        self.departures.append((veh, line, route, t))
                    break

    def passage_after(self, veh, stop_id, after):
        times = self.passages.get((veh, stop_id), [])
        i = bisect.bisect_left(times, after)
        return times[i] if i < len(times) else None


def route_geometry(gtfs, mapper, matcher):
    """(geometry, stop ids) of the most common stop pattern of a live route code, cached."""
    cache = {}

    def get(line, route):
        if (line, route) not in cache:
            shapes = mapper.shapes_for(line, route)
            trips = [t for t in gtfs.trips_for_line(line) if t.shape_id in shapes]
            result = None
            if trips:
                counts = defaultdict(int)
                for t in trips:
                    counts[t.pattern] += 1
                pattern = max(counts, key=counts.get)
                trip = next(t for t in trips if t.pattern == pattern)
                geo = matcher.geometry(trip)
                result = (geo, list(pattern)) if geo else None
            cache[(line, route)] = result
        return cache[(line, route)]
    return get


# ---------------------------------------------------------------- scoring

def midnight_of(day):
    return datetime(day.year, day.month, day.day, tzinfo=ATHENS).timestamp()


def summarise(errors):
    """errors: list of (horizon_s, error_s) -> per-horizon N, median |e|, p90 |e|, median e."""
    out = []
    for lo, hi in HORIZONS:
        sel = sorted(abs(e) for h, e in errors if lo * 60 < h <= hi * 60)
        signed = [e for h, e in errors if lo * 60 < h <= hi * 60]
        if sel:
            out.append((f"{lo}-{hi}", len(sel), statistics.median(sel) / 60,
                        sel[int(0.9 * (len(sel) - 1))] / 60, statistics.median(signed) / 60))
        else:
            out.append((f"{lo}-{hi}", 0, None, None, None))
    return out


def print_table(title, rows):
    print(f"\n{title}")
    print(f"  {'minutes ahead':>13} {'N':>7} {'median |err|':>13} {'p90 |err|':>10} {'median err':>11}")
    for h, n, med, p90, bias in rows:
        if n:
            print(f"  {h:>13} {n:>7} {med:>11.1f} m {p90:>8.1f} m {bias:>+9.1f} m")
        else:
            print(f"  {h:>13} {n:>7}")


class CachedSegments:
    """RecentSegmentTimes with a per-(segment, moment) cache: all vehicles in a cycle share 'now'."""

    def __init__(self, model):
        self.model, self.cache = model, {}

    def segment(self, a, b, now):
        key = (a, b, now)
        if key not in self.cache:
            self.cache[key] = self.model.segment(a, b, now)
        return self.cache[key]


def prediction_errors(results, truth, trips_by_id, model=None, schedule_only=False):
    """Errors of predicted arrivals for every downstream stop the vehicle actually reached.
    model=None: scheduled time + current delay. schedule_only: the timetable, ignoring delay."""
    errors = []
    for (cycle, veh), (trip_id, day, delay, next_index, waiting, _) in results.items():
        if trip_id is None:
            continue
        trip = trips_by_id[trip_id]
        base = midnight_of(day)
        if schedule_only:
            predictions = ((sid, base + (dep if waiting else arr))
                           for _, sid, arr, dep in trip.stop_times[next_index - (1 if waiting else 0):])
        else:
            predictions = ((sid, t) for _, sid, t in
                           predict_arrivals(trip, day, delay, next_index, waiting, cycle, model))
        for stop_id, predicted in predictions:
            actual = truth.passage_after(veh, stop_id, cycle)
            if actual is None or actual - cycle > HORIZONS[-1][1] * 60:
                continue
            errors.append((actual - cycle, predicted - actual))
    return errors


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("db")
    ap.add_argument("--gtfs", default="data/osy_gtfs.zip")
    ap.add_argument("--identity-only", action="store_true", help="skip the arrival-time evaluation")
    ap.add_argument("--blocks", action="store_true",
                    help="only check whether vehicles follow the GTFS vehicle blocks")
    args = ap.parse_args()

    cycles, fixes, eta = load(args.db)
    lines = sorted({line for by_line in cycles.values() for line in by_line})
    if args.blocks:
        return block_report(args.gtfs, cycles, fixes)
    gtfs = StaticGTFS(args.gtfs, lines)
    mapper = RouteMapper(gtfs, Telematics())
    methods = {"greedy (now)": Matcher(gtfs, mapper),
               "hungarian": HungarianMatcher(gtfs, mapper),
               "memory": MemoryMatcher(gtfs, mapper),
               "hmm (late only)": HMMMatcher(gtfs, mapper, early_free=0),
               "hmm": HMMMatcher(gtfs, mapper)}
    trips_by_id = gtfs.trips
    print(f"{len(cycles)} cycles, {sum(len(v) for v in fixes.values())} GPS fixes, "
          f"{len(fixes)} vehicles, {len(lines)} lines")

    # Replay: every method sees exactly what the live recorder saw, cycle by cycle.
    results = {name: {} for name in methods}
    anchored = set()
    for cycle in sorted(cycles):
        for line, vehicles in cycles[cycle].items():
            for name, matcher in methods.items():
                for m in matcher.match_line(line, vehicles):
                    results[name][(cycle, m.vehicle_id)] = (
                        m.trip.trip_id if m.trip else None, m.service_day, m.delay, m.next_index,
                        m.waiting_at_start, m.route_code)
                    track = getattr(matcher, "tracks", {}).get(m.vehicle_id)
                    if m.trip and track and track.anchor == (m.trip.trip_id, m.service_day):
                        anchored.add((cycle, m.vehicle_id))

    truth = Truth(fixes, gtfs, mapper, route_geometry(gtfs, mapper, methods["greedy (now)"]))
    n_pass = sum(len(v) for v in truth.passages.values())
    print(f"ground truth: {n_pass} stop passages, {len(truth.departures)} observed departures")

    # ---- identity: coverage, stability, agreement, proxy truth from observed departures
    print("\nTRIP IDENTITY")
    proxy = departure_proxy(truth, gtfs, mapper, cycles, results)
    names = list(methods)
    for name in names:
        res = results[name]
        matched = sum(1 for r in res.values() if r[0])
        switches = mid_run_switches(res)
        agree = [res[k][0] == trip for k, trip in proxy.items() if k in res]
        print(f"  {name:<18} matched {100 * matched / len(res):5.1f}%   mid-run trip changes {switches:4d}   "
              f"agrees with departure-based trip {100 * sum(agree) / max(1, len(agree)):5.1f}% (n={len(agree)})")
    mem = results["memory"]
    print(f"  memory: {100 * len(anchored) / max(1, sum(1 for r in mem.values() if r[0])):.1f}% of its matches come from an observed departure")
    print("  mid-run trip changes by line: " + ", ".join(
        f"{line} " + "/".join(str(n) for n in counts)
        for line, counts in switches_by_line(results, names, cycles)))
    a = results[names[0]]
    for other in names[1:]:
        b = results[other]
        differ = sum(1 for k in a if a[k][0] != b.get(k, (None,))[0])
        print(f"  greedy vs {other} disagree on {100 * differ / len(a):.1f}% of vehicle-cycles")

    print("\nOVERTAKES (same route, order swapped by >150 m, sustained)")
    events = overtakes(truth, cycles)
    print(f"  {len(events)} found")
    for name in names:
        kept = swapped = other = 0
        for v1, v2, before, after in events:
            r = results[name]
            t1b, t2b = r.get((before, v1), (None,))[0], r.get((before, v2), (None,))[0]
            t1a, t2a = r.get((after, v1), (None,))[0], r.get((after, v2), (None,))[0]
            if t1b and t2b and t1b == t2a and t2b == t1a:
                swapped += 1
            elif t1b and t2b and t1b == t1a and t2b == t2a:
                kept += 1
            else:
                other += 1
        print(f"  {name:<18} kept their trips {kept:3d}   swapped trips {swapped:3d}   other {other:3d}")

    if args.identity_only:
        return
    # ---- arrival predictions vs actual passages
    print("\nARRIVAL PREDICTIONS vs ACTUAL PASSAGES (from GPS)")
    model = CachedSegments(truth.segments)
    for name in names:
        print_table(f"{name}: scheduled time + current delay", summarise(prediction_errors(results[name], truth, trips_by_id)))
    print_table("hmm's trips: recent observed segment times", summarise(
        prediction_errors(results["hmm"], truth, trips_by_id, model=model)))
    print_table("schedule only (hmm's trips, no delay) = what Google Maps shows today",
                summarise(prediction_errors(results["hmm"], truth, trips_by_id, schedule_only=True)))
    oasa = []
    for polled, stop_id, veh, minutes in eta:
        actual = truth.passage_after(veh, stop_id, polled - 60)
        if actual is not None and actual - polled <= HORIZONS[-1][1] * 60:
            oasa.append((actual - polled, polled + minutes * 60 - actual))
    print_table("OASA's own prediction (getStopArrivals)", summarise(oasa))
    paired(eta, truth, results["hmm"], trips_by_id, model)


def block_of(trip):
    """OASA encodes the vehicle block in the trip id: {route}_{service}_{block}_{HHMM}."""
    return (trip.service_id, trip.trip_id.split("_")[3])


def identify_departure(gtfs, mapper, line, route, dep):
    """Trips of this route that could have left at `dep` (5 min early to 15 min late), best first,
    as (score, minutes late, trip)."""
    day = datetime.fromtimestamp(dep, ATHENS).date()
    shapes = mapper.shapes_for(line, route)
    out = []
    for trip in gtfs.trips_for_line(line):
        if trip.shape_id in shapes and gtfs.service_active(trip.service_id, day):
            late = dep - midnight_of(day) - trip.start
            if -300 <= late <= 900:
                out.append((late if late >= 0 else -3 * late, late / 60, trip))
    return sorted(out, key=lambda c: c[0])


def block_report(gtfs_path, cycles, fixes):
    """Do vehicles run consecutive trips of the same GTFS vehicle block?"""
    import csv, io, zipfile
    with zipfile.ZipFile(gtfs_path) as zf:
        names = {r["route_short_name"] for r in csv.DictReader(
            io.TextIOWrapper(zf.open("routes.txt"), encoding="utf-8-sig"))}
    gtfs = StaticGTFS(gtfs_path, names)           # whole network: blocks span several lines
    mapper = RouteMapper(gtfs, Telematics())
    truth = Truth(fixes, gtfs, mapper, route_geometry(gtfs, mapper, Matcher(gtfs, mapper)))
    blocks = defaultdict(list)
    for trip in gtfs.trips.values():
        if len(trip.trip_id.split("_")) == 5:
            blocks[block_of(trip)].append(trip)
    for trips in blocks.values():
        trips.sort(key=lambda t: t.start)

    by_veh, unidentified, lateness = defaultdict(list), 0, []
    for veh, line, route, dep in truth.departures:
        cands = identify_departure(gtfs, mapper, line, route, dep)
        if not cands:
            unidentified += 1
            continue
        margin = cands[1][0] - cands[0][0] if len(cands) > 1 else float("inf")
        by_veh[veh].append((dep, line, cands[0][2], margin))
        lateness.append(cands[0][1])
    print(f"{len(truth.departures)} observed departures; {unidentified} match no scheduled departure "
          f"within 5 min early .. 15 min late; departure delay median {statistics.median(lateness):+.1f} min")

    pairs, offsets = [], []
    for veh, deps in by_veh.items():
        deps.sort(key=lambda d: d[0])
        for a, b in zip(deps, deps[1:]):
            if b[0] - a[0] > 3 * 3600:
                continue
            nxt = next((t for t in blocks.get(block_of(a[2]), []) if t.start > a[2].start), None)
            day = datetime.fromtimestamp(b[0], ATHENS).date()
            route = next(r for v, l, r, d in truth.departures if v == veh and d == b[0])
            # Could the block's next trip have been the one we saw leave at all?
            possible = nxt is not None and any(t is nxt for _, _, t in identify_departure(gtfs, mapper, b[1], route, b[0]))
            if nxt is not None:
                offsets.append((b[0] - midnight_of(day) - nxt.start) / 60)
            pairs.append((block_of(a[2]) == block_of(b[2]), nxt is b[2], possible, min(a[3], b[3]) >= 180, a[1] != b[1]))
    for label, sel in (("all pairs", pairs), ("unambiguous departures only (next-best trip >= 3 min worse)",
                                              [p for p in pairs if p[3]])):
        n = len(sel)
        if not n:
            continue
        print(f"\n{label}: {n} consecutive trip pairs of the same vehicle")
        print(f"  same vehicle block:                         {100 * sum(p[0] for p in sel) / n:5.1f}%")
        print(f"  exactly the next trip of that block:        {100 * sum(p[1] for p in sel) / n:5.1f}%")
        print(f"  block's next trip was even a possible match: {100 * sum(p[2] for p in sel) / n:5.1f}%")
        print(f"  pairs that change line:                     {sum(p[4] for p in sel)}")
    if len(offsets) >= 10:
        print("\nobserved departure minus the block's next scheduled departure, deciles (min): "
              + ", ".join(f"{x:+.0f}" for x in statistics.quantiles(offsets, n=10)))


def paired(eta, truth, results, trips_by_id, model):
    """Same vehicle, same stop, same moment: OASA's prediction vs ours, on identical samples."""
    by_veh = defaultdict(list)
    for (cycle, veh), r in results.items():
        by_veh[veh].append((cycle, r))
    for v in by_veh.values():
        v.sort(key=lambda x: x[0])
    ours, recent, theirs = [], [], []
    for polled, stop_id, veh, minutes in eta:
        actual = truth.passage_after(veh, stop_id, polled - 60)
        if actual is None or actual - polled > HORIZONS[-1][1] * 60:
            continue
        hist = by_veh.get(veh, [])
        i = bisect.bisect_right([c for c, _ in hist], polled) - 1
        if i < 0 or polled - hist[i][0] > 60:
            continue
        cycle, (trip_id, day, delay, next_index, waiting, _) = hist[i]
        if not trip_id:
            continue
        trip = trips_by_id[trip_id]
        plain = {sid: t for _, sid, t in predict_arrivals(trip, day, delay, next_index, waiting, cycle)}
        if stop_id not in plain:
            continue
        learned = {sid: t for _, sid, t in predict_arrivals(trip, day, delay, next_index, waiting, cycle, model)}
        ours.append((actual - polled, plain[stop_id] - actual))
        recent.append((actual - polled, learned[stop_id] - actual))
        theirs.append((actual - polled, polled + minutes * 60 - actual))
    print_table("PAIRED, identical samples: OASA", summarise(theirs))
    print_table("PAIRED, identical samples: ours, scheduled time + delay", summarise(ours))
    print_table("PAIRED, identical samples: ours, recent segment times", summarise(recent))


def departure_proxy(truth, gtfs, mapper, cycles, results):
    """For runs whose departure we observed: the scheduled trip leaving closest to it, applied
    to that vehicle's cycles until it leaves the route. Used as a proxy for the true trip."""
    proxy = {}
    cycle_list = sorted(cycles)
    for veh, line, route, dep in truth.departures:
        best = None
        shapes = mapper.shapes_for(line, route)
        for trip in gtfs.trips_for_line(line):
            if trip.shape_id not in shapes:
                continue
            day = datetime.fromtimestamp(dep, ATHENS).date()
            if not gtfs.service_active(trip.service_id, day):
                continue
            late = dep - midnight_of(day) - trip.start
            if -300 <= late <= 900:
                score = late if late >= 0 else -3 * late
                if best is None or score < best[0]:
                    best = (score, trip.trip_id, trip.end - trip.start)
        if best is None:
            continue
        _, trip_id, duration = best
        for cycle in cycle_list[bisect.bisect_right(cycle_list, dep + 60):]:
            if cycle > dep + duration + 3600:
                break
            r = results["greedy (now)"].get((cycle, veh))
            if r is None or r[5] != route:
                break
            proxy[(cycle, veh)] = trip_id
    return proxy


def switches_by_line(results, names, cycles):
    line_of = {}
    for by_line in cycles.values():
        for line, vehicles in by_line.items():
            for v in vehicles:
                line_of[v["VEH_NO"]] = line
    counts = defaultdict(lambda: [0] * len(names))
    for i, name in enumerate(names):
        by_veh = defaultdict(list)
        for (cycle, veh), r in results[name].items():
            by_veh[veh].append((cycle, r))
        for veh, rows in by_veh.items():
            rows.sort(key=lambda x: x[0])
            for (_, a), (_, b) in zip(rows, rows[1:]):
                if a[5] == b[5] and a[0] and b[0] and a[0] != b[0] and not a[4] and not b[4]:
                    counts[line_of.get(veh, "?")][i] += 1
    return sorted(counts.items(), key=lambda kv: -kv[1][0])


def mid_run_switches(res):
    by_veh = defaultdict(list)
    for (cycle, veh), r in res.items():
        by_veh[veh].append((cycle, r))
    n = 0
    for rows in by_veh.values():
        rows.sort(key=lambda x: x[0])
        for (_, a), (_, b) in zip(rows, rows[1:]):
            # same route code, both matched, not at the terminal, but a different trip
            if a[5] == b[5] and a[0] and b[0] and a[0] != b[0] and not a[4] and not b[4]:
                n += 1
    return n


def overtakes(truth, cycles):
    events = []
    for cycle_a, cycle_b in zip(sorted(cycles), sorted(cycles)[1:]):
        pos_a, pos_b = positions_at(truth, cycles, cycle_a), positions_at(truth, cycles, cycle_b)
        for route, va in pos_a.items():
            vb = pos_b.get(route, {})
            ids = sorted(set(va) & set(vb))
            for i, x in enumerate(ids):
                for y in ids[i + 1:]:
                    before, after = va[x] - va[y], vb[x] - vb[y]
                    if abs(before) > OVERTAKE_MARGIN_M and abs(after) > OVERTAKE_MARGIN_M and before * after < 0:
                        events.append((x, y, cycle_a, cycle_b))
    return events


def positions_at(truth, cycles, cycle):
    out = defaultdict(dict)
    for vehicles in cycles[cycle].values():
        for v in vehicles:
            ts = datetime.strptime(v["CS_DATE"], "%b %d %Y %I:%M:%S:000%p").replace(tzinfo=ATHENS).timestamp()
            hit = truth.along.get(v["VEH_NO"], {}).get(ts)
            if hit:
                out[hit[0]][v["VEH_NO"]] = hit[1]
    return out


if __name__ == "__main__":
    main()
