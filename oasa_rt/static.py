"""Loads the subset of OASA's static GTFS (osy_gtfs.zip) needed for realtime matching."""

import csv
import io
import json
import os
import urllib.request
import zipfile
from dataclasses import dataclass, field
from datetime import date, datetime, timedelta

from .telematics import ATHENS, USER_AGENT

# Official OSY (bus/trolley) feed published by OASA on data.gov.gr.
OSY_GTFS_URL = ("https://data.gov.gr/dataset/fb049bb1-aea6-4443-95fa-8b941dd6a057/resource/"
                "119db488-16ea-4c76-b560-41c472872390/download/osy_gtfs.zip")
REQUIRED_FILES = {"stops.txt", "routes.txt", "trips.txt", "stop_times.txt", "calendar.txt"}


def remote_version(url=OSY_GTFS_URL):
    """ETag / Last-Modified / size of the published feed, without downloading it."""
    req = urllib.request.Request(url, method="HEAD", headers={"User-Agent": USER_AGENT})
    with urllib.request.urlopen(req, timeout=30) as resp:
        return {"etag": resp.headers.get("ETag"), "last_modified": resp.headers.get("Last-Modified"),
                "size": resp.headers.get("Content-Length")}


def _version_path(path):
    return path + ".version.json"


def download_gtfs(path, url=OSY_GTFS_URL):
    """Download the feed atomically, refusing files that are not a usable GTFS zip."""
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    version = remote_version(url)
    tmp = path + ".tmp"
    urllib.request.urlretrieve(url, tmp)
    try:
        with zipfile.ZipFile(tmp) as zf:
            missing = REQUIRED_FILES - set(zf.namelist())
            if missing or zf.testzip() is not None:
                raise ValueError(f"downloaded GTFS is unusable (missing {sorted(missing)})")
    except Exception:
        os.remove(tmp)
        raise
    os.replace(tmp, path)
    with open(_version_path(path), "w") as f:
        json.dump(version, f)


def refresh_gtfs(path, url=OSY_GTFS_URL):
    """Download the feed again if the published version differs from the local one.

    Returns True when the local file was replaced.
    """
    remote = remote_version(url)
    try:
        with open(_version_path(path)) as f:
            local = json.load(f)
    except FileNotFoundError:
        local = None
    if local is None and os.path.exists(path) and remote["size"] == str(os.path.getsize(path)):
        # A copy fetched before versions were recorded; assume it is current.
        with open(_version_path(path), "w") as f:
            json.dump(remote, f)
        return False
    if local == remote and os.path.exists(path):
        return False
    download_gtfs(path, url)
    return True


def _hms(value):
    h, m, s = value.split(":")
    return int(h) * 3600 + int(m) * 60 + int(s)


@dataclass
class Trip:
    trip_id: str
    route_id: str
    service_id: str
    direction_id: str
    shape_id: str
    # (stop_sequence, stop_id, arrival_secs, departure_secs), ordered by stop_sequence
    stop_times: list = field(default_factory=list)

    @property
    def pattern(self):
        return tuple(st[1] for st in self.stop_times)

    @property
    def start(self):
        return self.stop_times[0][3]

    @property
    def end(self):
        return self.stop_times[-1][2]


class StaticGTFS:
    """Static data for the given line numbers (route_short_name), across all their route_ids.

    OASA keeps several route_ids per line number (variants, night services), and the
    LineCode a vehicle runs under live is not always the route_id its trips have in the GTFS
    (e.g. the X96 night service runs live as 1153 but its trips are under 892).
    """

    def __init__(self, path, line_names):
        self.line_names = set(line_names)
        self.routes = {}           # route_id -> route_short_name, for the selected lines
        self.stops = {}            # stop_id -> (lat, lon, name)
        self.trips = {}            # trip_id -> Trip
        self.shapes = {}           # shape_id -> [(lat, lon), ...] in sequence order
        self.calendar = {}         # service_id -> (weekday flags, start, end)
        self.exceptions = {}       # (service_id, date) -> exception_type
        self._load(path)

    def _open(self, zf, name):
        return io.TextIOWrapper(zf.open(name), encoding="utf-8-sig", newline="")

    def _load(self, path):
        with zipfile.ZipFile(path) as zf:
            for r in csv.DictReader(self._open(zf, "stops.txt")):
                self.stops[r["stop_id"]] = (float(r["stop_lat"]), float(r["stop_lon"]), r["stop_name"])
            for r in csv.DictReader(self._open(zf, "calendar.txt")):
                days = [r[d] == "1" for d in
                        ("monday", "tuesday", "wednesday", "thursday", "friday", "saturday", "sunday")]
                self.calendar[r["service_id"]] = (
                    days, datetime.strptime(r["start_date"], "%Y%m%d").date(),
                    datetime.strptime(r["end_date"], "%Y%m%d").date())
            if "calendar_dates.txt" in zf.namelist():
                for r in csv.DictReader(self._open(zf, "calendar_dates.txt")):
                    d = datetime.strptime(r["date"], "%Y%m%d").date()
                    self.exceptions[(r["service_id"], d)] = r["exception_type"]
            for r in csv.DictReader(self._open(zf, "routes.txt")):
                if r["route_short_name"] in self.line_names:
                    self.routes[r["route_id"]] = r["route_short_name"]
            for r in csv.DictReader(self._open(zf, "trips.txt")):
                if r["route_id"] in self.routes:
                    self.trips[r["trip_id"]] = Trip(r["trip_id"], r["route_id"], r["service_id"],
                                                    r.get("direction_id", ""), r.get("shape_id", ""))
            # stop_times.txt is ~225 MB; stream it and keep only rows of the selected trips.
            reader = csv.reader(self._open(zf, "stop_times.txt"))
            header = next(reader)
            i_trip, i_arr, i_dep, i_stop, i_seq = (header.index(c) for c in (
                "trip_id", "arrival_time", "departure_time", "stop_id", "stop_sequence"))
            for row in reader:
                trip = self.trips.get(row[i_trip])
                if trip is not None:
                    trip.stop_times.append((int(row[i_seq]), row[i_stop], _hms(row[i_arr]), _hms(row[i_dep])))
            if "shapes.txt" in zf.namelist():
                wanted = {t.shape_id for t in self.trips.values()}
                points = {}
                for r in csv.DictReader(self._open(zf, "shapes.txt")):
                    if r["shape_id"] in wanted:
                        points.setdefault(r["shape_id"], []).append(
                            (int(r["shape_pt_sequence"]), float(r["shape_pt_lat"]), float(r["shape_pt_lon"])))
                self.shapes = {k: [(lat, lon) for _, lat, lon in sorted(v)] for k, v in points.items()}
        for trip in self.trips.values():
            trip.stop_times.sort()
        self.trips = {k: t for k, t in self.trips.items() if len(t.stop_times) >= 2}
        self.by_line = {}
        for trip in self.trips.values():
            self.by_line.setdefault(self.routes[trip.route_id], []).append(trip)

    def service_active(self, service_id, day):
        exc = self.exceptions.get((service_id, day))
        if exc is not None:
            return exc == "1"
        cal = self.calendar.get(service_id)
        return bool(cal and cal[1] <= day <= cal[2] and cal[0][day.weekday()])

    def feed_end_date(self):
        return max((c[2] for c in self.calendar.values()), default=None)

    def trips_for_line(self, line_name):
        return self.by_line.get(line_name, [])

    def route_id_for(self, line_code, line_name):
        """GTFS route_id to report for a vehicle whose trip is unknown."""
        if line_code in self.routes:
            return line_code
        trips = self.trips_for_line(line_name)
        return trips[0].route_id if trips else line_code

    def candidate_trips(self, line_name, now, before=15 * 60, after=60 * 60):
        """Trips of a line whose scheduled run (padded) covers `now`.

        Yields (trip, service_date, service_midnight) for today's and yesterday's service days,
        since GTFS times past 24:00 belong to the previous service day.
        """
        today = now.astimezone(ATHENS).date()
        for day in (today, today - timedelta(days=1)):
            midnight = datetime(day.year, day.month, day.day, tzinfo=ATHENS)
            secs = (now - midnight).total_seconds()
            for trip in self.trips_for_line(line_name):
                if not self.service_active(trip.service_id, day):
                    continue
                if trip.start - before <= secs <= trip.end + after:
                    yield trip, day, midnight


def service_date_str(day: date):
    return day.strftime("%Y%m%d")
