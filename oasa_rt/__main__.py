"""Proof-of-concept GTFS-Realtime producer for OASA buses and trolleys.

    python -m oasa_rt --lines 040,550,Α1 --once
    python -m oasa_rt --lines 040,550 --interval 30 --serve 8080
"""

import argparse
import functools
import http.server
import os
import statistics
import sys
import threading
import time
from datetime import datetime

from .feed import build_feeds, write_feeds
from .matcher import Matcher, RouteMapper
from .static import StaticGTFS, download_gtfs, refresh_gtfs
from .telematics import ATHENS, Telematics, parse_cs_date

# Latin letters that users type for Greek line names (e.g. "A1" for "Α1").
_GREEK = str.maketrans("ABEZHIKMNOPTYX", "ΑΒΕΖΗΙΚΜΝΟΡΤΥΧ")
STALE_AFTER = 5 * 60
EXPIRY_WARNING_DAYS = 7


def normalise_line(name):
    return name.strip().upper().translate(_GREEK)


def serve(directory, port):
    handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=directory)
    server = http.server.ThreadingHTTPServer(("", port), handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"serving {directory} on http://localhost:{port}/vehicle_positions.pb and /trip_updates.pb")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gtfs", default="data/osy_gtfs.zip", help="static GTFS zip (downloaded if missing)")
    ap.add_argument("--lines", required=True, help="comma-separated line numbers, e.g. 040,550,Α1")
    ap.add_argument("--out", default="out")
    ap.add_argument("--interval", type=int, default=30, help="seconds between polls")
    ap.add_argument("--once", action="store_true", help="poll once, write feeds and exit")
    ap.add_argument("--serve", type=int, metavar="PORT", help="also serve the feeds over HTTP")
    ap.add_argument("--min-request-interval", type=float, default=0.25,
                    help="minimum seconds between API requests (be gentle with OASA's server)")
    ap.add_argument("--gtfs-check-hours", type=float, default=6,
                    help="how often to look for a new static GTFS on data.gov.gr (0 = never)")
    args = ap.parse_args()

    tel = Telematics(min_interval=args.min_request_interval)
    lines, missing = resolve_lines(tel, args.lines)
    if missing:
        print(f"unknown lines: {', '.join(sorted(missing))}", file=sys.stderr)
    if not lines:
        return 1

    if not os.path.exists(args.gtfs):
        print(f"downloading static GTFS to {args.gtfs} ...")
        download_gtfs(args.gtfs)
    gtfs, matcher = load_static(args.gtfs, lines, tel)
    if args.serve:
        os.makedirs(args.out, exist_ok=True)
        serve(args.out, args.serve)

    if args.gtfs_check_hours > 0:
        next_check = time.monotonic()
    else:
        next_check = float("inf")
        warn_expiry(gtfs)
    while True:
        cycle_start = time.monotonic()
        if cycle_start >= next_check:
            next_check = cycle_start + args.gtfs_check_hours * 3600
            try:
                if refresh_gtfs(args.gtfs):
                    print("a new static GTFS was published; reloading", file=sys.stderr)
                    gtfs, matcher = load_static(args.gtfs, lines, tel)
            except Exception as exc:  # keep serving with the current feed
                print(f"static GTFS update check failed: {exc!r}", file=sys.stderr)
            warn_expiry(gtfs)
        now = datetime.now(ATHENS)
        tel.requests = 0
        matches = []
        for line, route_codes in lines.items():
            matches += matcher.match_line(line, poll(tel, route_codes, now))
        # Stamp the feed when polling finished, so consecutive headers are evenly spaced.
        vehicles_msg, trips_msg = build_feeds(matches, datetime.now(ATHENS))
        write_feeds(args.out, vehicles_msg, trips_msg)
        _report(now, matches, tel.requests, time.monotonic() - cycle_start)
        if args.once:
            return 0
        time.sleep(max(1.0, args.interval - (time.monotonic() - cycle_start)))


def load_static(path, lines, tel):
    t0 = time.monotonic()
    gtfs = StaticGTFS(path, lines)
    print(f"loaded {len(gtfs.trips)} trips for {len(lines)} lines in {time.monotonic() - t0:.1f}s")
    for line in lines:
        if not gtfs.trips_for_line(line):
            print(f"note: line {line} has no trips in the static GTFS", file=sys.stderr)
    return gtfs, Matcher(gtfs, RouteMapper(gtfs, tel))


def warn_expiry(gtfs):
    end = gtfs.feed_end_date()
    if end is None:
        return
    days_left = (end - datetime.now(ATHENS).date()).days
    if days_left < 0:
        print(f"WARNING: static GTFS expired on {end}; no trips can be matched until OASA publishes a new one",
              file=sys.stderr)
    elif days_left <= EXPIRY_WARNING_DAYS:
        print(f"WARNING: static GTFS expires on {end} ({days_left} days left)", file=sys.stderr)


def resolve_lines(tel, spec):
    """{line number: {route_code: line_code}} for a comma-separated list of line numbers.

    One line number can have several LineCodes (variants, night services); all are polled.
    """
    wanted = {normalise_line(x) for x in spec.split(",") if x.strip()}
    lines = {}
    for l in tel.lines():
        if l["LineID"] in wanted:
            for r in tel.routes(l["LineCode"]):
                lines.setdefault(l["LineID"], {})[r["RouteCode"]] = l["LineCode"]
    return lines, wanted - set(lines)


def poll(tel, route_codes, now):
    """Fresh getBusLocation rows for {route_code: line_code}, tagged with their LINE_CODE."""
    vehicles = []
    for route_code, line_code in route_codes.items():
        try:
            rows = tel.bus_locations(route_code)
        except Exception as exc:  # one failing route should not drop the whole feed
            print(f"getBusLocation {route_code}: {exc}", file=sys.stderr)
            continue
        for v in rows:
            v["LINE_CODE"] = line_code
        vehicles += rows
    return [v for v in vehicles if _age(v, now) <= STALE_AFTER]


def _age(vehicle, now):
    try:
        return (now - parse_cs_date(vehicle["CS_DATE"])).total_seconds()
    except ValueError:
        return float("inf")


def _report(now, matches, requests, elapsed):
    matched = [m for m in matches if m.trip]
    delays = [m.delay / 60 for m in matched if not m.waiting_at_start]
    summary = f"median delay {statistics.median(delays):+.1f} min" if delays else "no delays"
    print(f"[{now:%H:%M:%S}] {len(matches)} vehicles, {len(matched)} matched to trips, {summary} "
          f"({requests} API requests, {elapsed:.1f}s)")
    for m in sorted(matches, key=lambda m: (m.line, m.route_code)):
        if m.trip:
            stop = m.trip.stop_times[0 if m.waiting_at_start else m.next_index]
            state = "waiting" if m.waiting_at_start else f"{m.delay / 60:+5.1f} min"
            print(f"   {m.line:>4} veh {m.vehicle_id:>6} route {m.route_code:>5} -> {m.trip.trip_id:<28} {state:>9}"
                  f"  next stop {stop[1]}")
        else:
            print(f"   {m.line:>4} veh {m.vehicle_id:>6} route {m.route_code:>5} -> (no trip match)")


if __name__ == "__main__":
    sys.exit(main())
