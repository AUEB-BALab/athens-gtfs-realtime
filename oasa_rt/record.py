"""Records raw GPS fixes, our trip matches and OASA's own arrival predictions to SQLite,
for later evaluation against actual stop passages.

    python -m oasa_rt.record --lines 040,550 --start 2026-10-05T07:00 --end 2026-10-05T10:00

Routes that had no vehicles are re-polled every --empty-interval seconds instead of every
cycle, and getStopArrivals is sampled round-robin over every --stop-step'th stop at a fixed
rate, to keep the load on OASA's server low.
"""

import argparse
import os
import sqlite3
import subprocess
import sys
import time
from datetime import datetime

from .__main__ import STALE_AFTER, _age, resolve_lines
from .matcher import Matcher, RouteMapper
from .static import StaticGTFS, download_gtfs, service_date_str
from .telematics import ATHENS, Telematics

SCHEMA = """
CREATE TABLE IF NOT EXISTS fix(veh TEXT, ts REAL, fetched REAL, line TEXT, line_code TEXT,
    route_code TEXT, lat REAL, lon REAL, heading REAL, PRIMARY KEY(veh, ts));
CREATE TABLE IF NOT EXISTS match(cycle REAL, veh TEXT, line TEXT, route_code TEXT, fix_ts REAL,
    trip_id TEXT, service_date TEXT, delay INTEGER, next_index INTEGER, waiting INTEGER);
CREATE TABLE IF NOT EXISTS oasa_eta(polled REAL, stop_id TEXT, veh TEXT, route_code TEXT, minutes INTEGER);
CREATE TABLE IF NOT EXISTS cycle(started REAL, routes_polled INTEGER, requests INTEGER,
    vehicles INTEGER, matched INTEGER, seconds REAL);
CREATE INDEX IF NOT EXISTS match_veh ON match(veh, cycle);
CREATE INDEX IF NOT EXISTS eta_stop ON oasa_eta(stop_id, veh, polled);
"""


def parse_local(value):
    return datetime.fromisoformat(value).replace(tzinfo=ATHENS)


def keep_awake():
    """Prevent idle and (on AC power) system sleep for as long as this process lives (macOS)."""
    try:
        subprocess.Popen(["caffeinate", "-i", "-s", "-w", str(os.getpid())])
    except FileNotFoundError:
        pass


def log(msg):
    print(f"[{datetime.now(ATHENS):%Y-%m-%d %H:%M:%S}] {msg}", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lines", required=True)
    ap.add_argument("--gtfs", default="data/osy_gtfs.zip")
    ap.add_argument("--db", default="data/record.sqlite")
    ap.add_argument("--start", type=parse_local, help="Athens local time, e.g. 2026-10-05T07:00")
    ap.add_argument("--end", type=parse_local, required=True)
    ap.add_argument("--interval", type=float, default=30)
    ap.add_argument("--empty-interval", type=float, default=300)
    ap.add_argument("--arrivals-rate", type=float, default=0.5, help="getStopArrivals requests per second")
    ap.add_argument("--stop-step", type=int, default=4)
    args = ap.parse_args()

    if datetime.now(ATHENS) >= args.end:
        log(f"end time {args.end:%Y-%m-%d %H:%M} already passed, nothing to do")
        return 0
    keep_awake()
    if args.start and datetime.now(ATHENS) < args.start:
        log(f"waiting until {args.start:%Y-%m-%d %H:%M}")
        while datetime.now(ATHENS) < args.start:
            time.sleep(min(60.0, (args.start - datetime.now(ATHENS)).total_seconds() + 0.1))

    tel = Telematics()
    lines, missing = resolve_lines(tel, args.lines)
    if missing:
        log(f"unknown lines: {', '.join(sorted(missing))}")
    if not os.path.exists(args.gtfs):
        download_gtfs(args.gtfs)
    gtfs = StaticGTFS(args.gtfs, lines)
    matcher = Matcher(gtfs, RouteMapper(gtfs, tel))
    stops = []
    for route_codes in lines.values():
        for rc in route_codes:
            stops += [s["StopCode"] for s in tel.route_stops(rc)][::args.stop_step]
    stops = list(dict.fromkeys(stops))
    log(f"{len(lines)} lines, {sum(map(len, lines.values()))} routes, {len(gtfs.trips)} trips, "
        f"{len(stops)} sampled stops")

    db = sqlite3.connect(args.db)
    db.executescript(SCHEMA)
    next_poll = {rc: 0.0 for route_codes in lines.values() for rc in route_codes}
    stop_i = 0
    while datetime.now(ATHENS) < args.end:
        started = time.monotonic()
        try:
            stop_i = _cycle(args, tel, db, lines, matcher, next_poll, stops, stop_i, started)
        except Exception as exc:  # keep recording through transient network/API failures
            log(f"cycle failed: {exc!r}")
            time.sleep(5)
    log("done")
    return 0


def _cycle(args, tel, db, lines, matcher, next_poll, stops, stop_i, started):
    now = datetime.now(ATHENS)
    tel.requests = 0
    polled = vehicles = matched = 0
    for line, route_codes in lines.items():
        rows = []
        for rc, lc in route_codes.items():
            if next_poll[rc] > started:
                continue
            polled += 1
            try:
                got = tel.bus_locations(rc)
            except Exception as exc:
                log(f"getBusLocation {rc}: {exc!r}")
                next_poll[rc] = started + args.interval
                continue
            next_poll[rc] = started + (args.interval if got else args.empty_interval)
            for v in got:
                v["LINE_CODE"] = lc
            rows += got
        rows = [v for v in rows if _age(v, now) <= STALE_AFTER]
        if not rows:
            continue
        fetched = time.time()
        results = matcher.match_line(line, rows)
        db.executemany("INSERT OR IGNORE INTO fix VALUES (?,?,?,?,?,?,?,?,?)", [
            (m.vehicle_id, m.timestamp.timestamp(), fetched, line, v["LINE_CODE"], m.route_code,
             m.lat, m.lon, m.bearing) for m, v in zip(results, rows)])
        db.executemany("INSERT INTO match VALUES (?,?,?,?,?,?,?,?,?,?)", [
            (now.timestamp(), m.vehicle_id, line, m.route_code, m.timestamp.timestamp(),
             m.trip.trip_id if m.trip else None,
             service_date_str(m.service_day) if m.trip else None,
             m.delay if m.trip else None, m.next_index if m.trip else None, int(m.waiting_at_start))
            for m in results])
        vehicles += len(results)
        matched += sum(1 for m in results if m.trip)
    db.execute("INSERT INTO cycle VALUES (?,?,?,?,?,?)",
               (now.timestamp(), polled, tel.requests, vehicles, matched, time.monotonic() - started))
    db.commit()
    log(f"{polled} routes polled, {vehicles} vehicles, {matched} matched, {tel.requests} requests")

    # Spend the rest of the cycle sampling OASA's arrival predictions.
    while stops and time.monotonic() < started + args.interval - 1 and datetime.now(ATHENS) < args.end:
        t0 = time.monotonic()
        stop_id = stops[stop_i % len(stops)]
        stop_i += 1
        try:
            arrivals = tel.stop_arrivals(stop_id)
        except Exception as exc:
            log(f"getStopArrivals {stop_id}: {exc!r}")
            arrivals = []
        polled_at = time.time()
        db.executemany("INSERT INTO oasa_eta VALUES (?,?,?,?,?)", [
            (polled_at, stop_id, a["veh_code"], a["route_code"], int(a["btime2"])) for a in arrivals])
        db.commit()
        time.sleep(max(0.0, 1 / args.arrivals_rate - (time.monotonic() - t0)))
    time.sleep(max(0.0, started + args.interval - time.monotonic()))
    return stop_i


if __name__ == "__main__":
    sys.exit(main())
