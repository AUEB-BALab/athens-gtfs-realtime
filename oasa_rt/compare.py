"""Sanity check: compare our TripUpdate ETAs with OASA's own getStopArrivals predictions.

    python -m oasa_rt.compare --lines 040,550 --ahead 4

For every matched vehicle, picks the stop `ahead` stops downstream and compares
  schedule-only ETA     (what Google Maps shows today, static GTFS only),
  our ETA               (scheduled time + matched delay), once measuring progress on
                        shapes.txt and once on straight lines between stops,
with OASA's ETA (telematics prediction for that vehicle at that stop).
"""

import argparse
import statistics
from datetime import datetime, timedelta

from .__main__ import poll, resolve_lines
from .matcher import Matcher, RouteMapper
from .static import StaticGTFS
from .telematics import ATHENS, Telematics


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gtfs", default="data/osy_gtfs.zip")
    ap.add_argument("--lines", required=True)
    ap.add_argument("--ahead", type=int, default=4)
    args = ap.parse_args()

    tel = Telematics()
    lines, _ = resolve_lines(tel, args.lines)
    gtfs = StaticGTFS(args.gtfs, lines)
    mapper = RouteMapper(gtfs, tel)
    methods = {"shapes": Matcher(gtfs, mapper, use_shapes=True),
               "straight": Matcher(gtfs, mapper, use_shapes=False)}
    now = datetime.now(ATHENS)
    arrivals = {}

    def oasa_eta(stop_id, vehicle_id):
        if stop_id not in arrivals:
            arrivals[stop_id] = tel.stop_arrivals(stop_id)
        return next((now + timedelta(minutes=int(a["btime2"]))
                     for a in arrivals[stop_id] if a["veh_code"] == vehicle_id), None)

    def mins(t):
        return (t - now).total_seconds() / 60

    errors = {"schedule": [], "shapes": [], "straight": []}
    matched = {name: 0 for name in methods}
    vehicles_total = 0
    print(f"{'vehicle':>7} {'method':<8} {'trip':<26} {'stop':>7}  sched   ours   OASA  (min from now)")
    for line, route_codes in lines.items():
        vehicles = poll(tel, route_codes, now)
        vehicles_total += len(vehicles)
        results = {name: {m.vehicle_id: m for m in matcher.match_line(line, vehicles)}
                   for name, matcher in methods.items()}
        for vid in sorted(results["shapes"]):
            rows = []
            for name in methods:
                m = results[name][vid]
                if not m.trip:
                    rows.append((name, None))
                    continue
                matched[name] += 1
                if m.waiting_at_start:
                    continue
                k = min(m.next_index + args.ahead, len(m.trip.stop_times) - 1)
                _, stop_id, arr, _ = m.trip.stop_times[k]
                midnight = datetime(m.service_day.year, m.service_day.month, m.service_day.day, tzinfo=ATHENS)
                sched = midnight + timedelta(seconds=arr)
                rows.append((name, (m, stop_id, sched, sched + timedelta(seconds=m.delay),
                                    oasa_eta(stop_id, vid))))
            for name, row in rows:
                if row is None:
                    print(f"{vid:>7} {name:<8} line {line} route {results[name][vid].route_code}: no trip match")
                    continue
                m, stop_id, sched, ours, oasa = row
                ref = f"{mins(oasa):6.1f}" if oasa else "    --"
                print(f"{vid:>7} {name:<8} {m.trip.trip_id:<26} {stop_id:>7} {mins(sched):6.1f} {mins(ours):6.1f} {ref}")
            # Score only vehicles that both methods matched and OASA predicts, so the samples are comparable.
            scored = [row for _, row in rows if row and row[4]]
            if len(scored) == len(methods):
                errors["schedule"].append(abs(mins(scored[0][2]) - mins(scored[0][4])))
                for (name, _), row in zip(rows, scored):
                    errors[name].append(abs(mins(row[3]) - mins(row[4])))

    shape_geos = sum(1 for g in methods["shapes"]._geometry.values() if g and g.from_shape)
    print(f"\n{vehicles_total} vehicles; matched to a trip: "
          + ", ".join(f"{name} {n}" for name, n in matched.items()))
    print(f"stop patterns measured on shapes.txt: {shape_geos}/{len(methods['shapes']._geometry)}")
    if errors["shapes"]:
        print(f"{len(errors['shapes'])} vehicles compared against OASA's prediction:")
        for name, errs in errors.items():
            print(f"  {name:<9}: median |error| {statistics.median(errs):.1f} min, "
                  f"mean {statistics.mean(errs):.1f}, max {max(errs):.1f}")


if __name__ == "__main__":
    main()
