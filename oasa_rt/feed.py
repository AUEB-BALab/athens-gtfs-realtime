"""Builds GTFS-Realtime VehiclePositions and TripUpdates feeds from matched vehicles."""

import json
import os

from google.protobuf import json_format
from google.transit import gtfs_realtime_pb2 as rt

from .static import service_date_str


def _message(now):
    msg = rt.FeedMessage()
    msg.header.gtfs_realtime_version = "2.0"
    msg.header.incrementality = rt.FeedHeader.FULL_DATASET
    msg.header.timestamp = int(now.timestamp())
    return msg


def _trip_descriptor(desc, m):
    desc.trip_id = m.trip.trip_id
    desc.route_id = m.trip.route_id
    desc.start_date = service_date_str(m.service_day)
    desc.schedule_relationship = rt.TripDescriptor.SCHEDULED


def build_feeds(matches, now):
    # The header must not be older than any entity; OASA's GPS fixes can be a few seconds
    # newer than the start of the polling cycle.
    newest = max([now] + [m.timestamp for m in matches])
    vehicles, trip_updates = _message(newest), _message(newest)
    for m in matches:
        ent = vehicles.entity.add()
        ent.id = f"vehicle-{m.vehicle_id}"
        vp = ent.vehicle
        vp.vehicle.id = m.vehicle_id
        vp.vehicle.label = m.vehicle_id
        vp.position.latitude = m.lat
        vp.position.longitude = m.lon
        vp.position.bearing = m.bearing
        vp.timestamp = int(m.timestamp.timestamp())
        if m.trip is None:
            # Partial descriptor: the line is known even when the exact trip is not.
            vp.trip.route_id = m.route_id
            continue
        _trip_descriptor(vp.trip, m)
        if m.waiting_at_start:
            stop = m.trip.stop_times[0]
            vp.current_status = rt.VehiclePosition.STOPPED_AT
        else:
            stop = m.trip.stop_times[m.next_index]
            vp.current_status = rt.VehiclePosition.IN_TRANSIT_TO
        vp.current_stop_sequence = stop[0]
        vp.stop_id = stop[1]

        ent = trip_updates.entity.add()
        ent.id = f"trip-{m.trip.trip_id}-{service_date_str(m.service_day)}"
        tu = ent.trip_update
        _trip_descriptor(tu.trip, m)
        tu.vehicle.id = m.vehicle_id
        tu.timestamp = int(m.timestamp.timestamp())
        stu = tu.stop_time_update.add()
        stu.stop_sequence = stop[0]
        stu.stop_id = stop[1]
        stu.schedule_relationship = rt.TripUpdate.StopTimeUpdate.SCHEDULED
        # A single delay propagates to all downstream stops (GTFS-RT spec).
        if m.waiting_at_start:
            stu.departure.delay = m.delay
        else:
            stu.arrival.delay = m.delay
    return vehicles, trip_updates


def _write_atomic(path, data):
    tmp = path + ".tmp"
    with open(tmp, "wb") as f:
        f.write(data)
    os.replace(tmp, path)


def write_feeds(out_dir, vehicles, trip_updates):
    os.makedirs(out_dir, exist_ok=True)
    for name, msg in (("vehicle_positions", vehicles), ("trip_updates", trip_updates)):
        _write_atomic(os.path.join(out_dir, name + ".pb"), msg.SerializeToString())
        _write_atomic(os.path.join(out_dir, name + ".json"), json.dumps(
            json_format.MessageToDict(msg), ensure_ascii=False, indent=1).encode("utf-8"))


def write_names(out_dir, gtfs):
    """Line and stop names for the map viewer, which otherwise only sees ids in the feed."""
    os.makedirs(out_dir, exist_ok=True)
    stop_ids = {st[1] for trip in gtfs.trips.values() for st in trip.stop_times}
    names = {
        "routes": {rid: [short, gtfs.route_long_names.get(rid, "")] for rid, short in gtfs.routes.items()},
        "stops": {sid: gtfs.stops[sid][2] for sid in stop_ids if sid in gtfs.stops},
    }
    _write_atomic(os.path.join(out_dir, "names.json"), json.dumps(names, ensure_ascii=False).encode("utf-8"))
