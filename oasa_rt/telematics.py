"""Minimal client for the (undocumented) OASA Telematics API.

Endpoints are described in the community docs:
https://oasa-telematics-api.readthedocs.io/en/latest/
"""

import json
import re
import threading
import time
import urllib.parse
import urllib.request
from datetime import datetime
from zoneinfo import ZoneInfo

BASE_URL = "https://telematics.oasa.gr/api/"
ATHENS = ZoneInfo("Europe/Athens")
USER_AGENT = "oasa-gtfs-rt-poc/0.1 (volunteer GTFS-Realtime experiment)"

_MONTHS = {m: i for i, m in enumerate(
    ["Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"], 1)}
# e.g. "Oct  4 2026 10:15:38:000PM"
_CS_DATE = re.compile(r"(\w{3})\s+(\d{1,2})\s+(\d{4})\s+(\d{1,2}):(\d{2}):(\d{2}):\d+\s*([AP]M)")


def parse_cs_date(value):
    """Parse the vehicle timestamp format used by getBusLocation into an aware datetime."""
    m = _CS_DATE.match(value.strip())
    if not m:
        raise ValueError(f"unrecognised CS_DATE: {value!r}")
    mon, day, year, hour, minute, sec, ampm = m.groups()
    hour = int(hour) % 12 + (12 if ampm == "PM" else 0)
    return datetime(int(year), _MONTHS[mon], int(day), hour, int(minute), int(sec), tzinfo=ATHENS)


class Telematics:
    """Throttled client: never sends more than one request per `min_interval` seconds."""

    def __init__(self, min_interval=0.25, timeout=20):
        self.min_interval = min_interval
        self.timeout = timeout
        self._last = 0.0
        self._lock = threading.Lock()
        self.requests = 0

    def call(self, act, *params):
        query = {"act": act}
        for i, p in enumerate(params, 1):
            query[f"p{i}"] = p
        with self._lock:
            wait = self._last + self.min_interval - time.monotonic()
            if wait > 0:
                time.sleep(wait)
            self._last = time.monotonic()
            self.requests += 1
        req = urllib.request.Request(
            BASE_URL + "?" + urllib.parse.urlencode(query), headers={"User-Agent": USER_AGENT})
        with urllib.request.urlopen(req, timeout=self.timeout) as resp:
            body = resp.read().decode("utf-8").strip()
        # The API answers "no data" with an empty JSON string or null.
        if body in ("", '""', "null"):
            return []
        data = json.loads(body)
        if isinstance(data, dict) and "error" in data:
            raise RuntimeError(f"{act}: {data['error']}")
        return data

    def lines(self):
        return self.call("webGetLines")

    def routes(self, line_code):
        return self.call("webGetRoutes", line_code)

    def route_stops(self, route_code):
        return self.call("webGetStops", route_code)

    def bus_locations(self, route_code):
        return self.call("getBusLocation", route_code)

    def stop_arrivals(self, stop_code):
        return self.call("getStopArrivals", stop_code)
