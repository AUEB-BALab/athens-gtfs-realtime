"""Proof-of-concept GTFS-Realtime producer for OASA buses and trolleys.

    python -m oasa_rt --lines 040,550,Α1 --once
    python -m oasa_rt --lines 040,550 --interval 30 --serve 8080
"""

import argparse
import functools
import http.server
import os
import shutil
import statistics
import sys
import threading
import time
from datetime import datetime

from .eta import PassageTracker, RecentSegmentTimes, route_geometry
from .feed import build_feeds, write_feeds, write_static, write_trip_info
from .matcher import HMMMatcher, HungarianMatcher, Matcher, MemoryMatcher, RouteMapper
from .static import StaticGTFS, download_gtfs, refresh_gtfs
from .telematics import ATHENS, Telematics, parse_cs_date

# Latin letters that users type for Greek line names (e.g. "A1" for "Α1").
_GREEK = str.maketrans("ABEZHIKMNOPTYX", "ΑΒΕΖΗΙΚΜΝΟΡΤΥΧ")
STALE_AFTER = 5 * 60
EXPIRY_WARNING_DAYS = 7
MATCHERS = {"greedy": Matcher, "hungarian": HungarianMatcher, "memory": MemoryMatcher, "hmm": HMMMatcher}


def normalise_line(name):
    return name.strip().upper().translate(_GREEK)


VIEWER = os.path.join(os.path.dirname(__file__), "viewer.html")


class FeedEvents:
    """Wakes the map viewer's /events streams whenever a new feed has been written."""

    def __init__(self):
        self._cond = threading.Condition()
        self.version = 0

    def publish(self):
        with self._cond:
            self.version += 1
            self._cond.notify_all()

    def wait(self, seen, timeout):
        with self._cond:
            self._cond.wait_for(lambda: self.version != seen, timeout)
            return self.version


class _ViewerHandler(http.server.SimpleHTTPRequestHandler):
    events = None   # set by serve()

    def log_message(self, format, *args):
        pass  # the viewer polls every few seconds; per-request logs would drown everything else

    def do_GET(self):
        if self.path.split("?")[0] != "/events":
            return super().do_GET()
        # Server-sent events: one "feed" message per new feed, comments as keep-alives.
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        seen = self.events.version
        try:
            while True:
                version = self.events.wait(seen, timeout=15)
                self.wfile.write(b"data: feed\n\n" if version != seen else b": keep-alive\n\n")
                self.wfile.flush()
                seen = version
        except (BrokenPipeError, ConnectionResetError):
            pass


def serve(directory, port):
    shutil.copyfile(VIEWER, os.path.join(directory, "index.html"))
    _ViewerHandler.events = FeedEvents()
    handler = functools.partial(_ViewerHandler, directory=directory)
    server = http.server.ThreadingHTTPServer(("", port), handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"map: http://localhost:{port}/   feeds: /vehicle_positions.pb and /trip_updates.pb")
    return _ViewerHandler.events


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
    ap.add_argument("--matcher", choices=MATCHERS, default="memory",
                    help="vehicle-to-trip matching (see README for the evaluation)")
    ap.add_argument("--eta", choices=["recent", "schedule"], default="recent",
                    help="arrival predictions from recently observed stop-to-stop times, "
                         "or scheduled time + current delay")
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
    gtfs, matcher = load_static(args.gtfs, lines, tel, MATCHERS[args.matcher])
    write_static(args.out, gtfs)
    # Stop passages seen live feed the recent segment times used for arrival predictions.
    # Until a segment has been driven a couple of times, its scheduled time is used.
    segments = RecentSegmentTimes() if args.eta == "recent" else None
    tracker = PassageTracker(route_geometry(gtfs, matcher.mapper, matcher), segments)
    events = None
    if args.serve:
        os.makedirs(args.out, exist_ok=True)
        events = serve(args.out, args.serve)

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
                    gtfs, matcher = load_static(args.gtfs, lines, tel, MATCHERS[args.matcher])
                    write_static(args.out, gtfs)
                    tracker = PassageTracker(route_geometry(gtfs, matcher.mapper, matcher), segments)
            except Exception as exc:  # keep serving with the current feed
                print(f"static GTFS update check failed: {exc!r}", file=sys.stderr)
            warn_expiry(gtfs)
        now = datetime.now(ATHENS)
        tel.requests = 0
        matches = []
        for line, route_codes in lines.items():
            vehicles = poll(tel, route_codes, now)
            for v in vehicles:
                tracker.observe(v["VEH_NO"], line, v["ROUTE_CODE"], parse_cs_date(v["CS_DATE"]).timestamp(),
                                float(v["CS_LAT"]), float(v["CS_LNG"]), float(v.get("VEH_HEADING") or 0))
            matches += matcher.match_line(line, vehicles)
        if segments:
            segments.prune(now.timestamp())
        # Stamp the feed when polling finished, so consecutive headers are evenly spaced.
        vehicles_msg, trips_msg = build_feeds(matches, datetime.now(ATHENS), segments)
        write_feeds(args.out, vehicles_msg, trips_msg)
        write_trip_info(args.out, matches, gtfs)
        if events:
            events.publish()
        _report(now, matches, tel.requests, time.monotonic() - cycle_start, segments)
        if args.once:
            return 0
        time.sleep(max(1.0, args.interval - (time.monotonic() - cycle_start)))


def load_static(path, lines, tel, matcher_class=MemoryMatcher):
    t0 = time.monotonic()
    gtfs = StaticGTFS(path, lines)
    print(f"loaded {len(gtfs.trips)} trips for {len(lines)} lines in {time.monotonic() - t0:.1f}s")
    for line in lines:
        if not gtfs.trips_for_line(line):
            print(f"note: line {line} has no trips in the static GTFS", file=sys.stderr)
    return gtfs, matcher_class(gtfs, RouteMapper(gtfs, tel))


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


def _report(now, matches, requests, elapsed, segments=None):
    matched = [m for m in matches if m.trip]
    delays = [m.delay / 60 for m in matched if not m.waiting_at_start]
    summary = f"median delay {statistics.median(delays):+.1f} min" if delays else "no delays"
    print(f"[{now:%H:%M:%S}] {len(matches)} vehicles, {len(matched)} matched to trips, {summary} "
          f"({requests} API requests, {elapsed:.1f}s)"
          + (f", {len(segments)} stop-to-stop traversals observed in the last 45 min" if segments is not None else ""))
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
