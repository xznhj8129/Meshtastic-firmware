#!/usr/bin/env python3
"""Two-node MAVLink-over-Meshtastic bench rig without an autopilot.

A pymavlink "vehicle" talks to the air node's UART; a pymavlink "GCS" talks to the ground
node's UDP endpoint. Every message either side receives is written to events.jsonl with its
full field set. Nothing is gated on what arrives: commands and the parameter request go out
on a fixed clock, and summary.json only counts what was recorded.

usage: bench_rig.py <ground-node-ip> [duration_s]
"""

# ═══════════════════════ CONFIG ═══════════════════════
AIR_UART = "/dev/ttyUSB0"  # USB-UART wired to the air node's GPIO47(RX)/GPIO48(TX)
AIR_BAUD = 57600  # must match serial.baud on the air node
GROUND_PORT = 14550  # ground node MAVLink UDP server
DEFAULT_DURATION_S = 180

VEHICLE_SYSID, VEHICLE_COMPID = 1, 1
GCS_SYSID, GCS_COMPID = 255, 190

# Vehicle telemetry, Hz. Deliberately more than a check-in profile so state replacement engages.
VEHICLE_RATES_HZ = {
    "HEARTBEAT": 1.0,
    "GLOBAL_POSITION_INT": 1.0,
    "SYS_STATUS": 0.5,
    "VFR_HUD": 1.0,
    "ATTITUDE": 2.0,
}
GCS_HEARTBEAT_HZ = 1.0
COMMAND_INTERVAL_S = 5.0  # one COMMAND_LONG per interval, index carried in param1
COMMANDS_START_S = 10.0
PARAM_LIST_AT_S = 90.0  # GCS sends PARAM_REQUEST_LIST once; 0 disables
PARAM_COUNT = 20
PARAM_SEND_HZ = 5.0  # vehicle paces its PARAM_VALUE burst

HOME_LAT, HOME_LON = 36.530440, -83.216383  # Sneedville, Tennessee
REPORT_ROOT = "mavlink_tests/reports"
# ══════════════════════════════════════════════════════

import json
import os
import statistics
import sys
import time

from pymavlink import mavutil

ground_ip = sys.argv[1]
duration = float(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_DURATION_S
report_dir = os.path.join(REPORT_ROOT, time.strftime("bench-%Y%m%d-%H%M%S"))
os.makedirs(report_dir)
events = open(os.path.join(report_dir, "events.jsonl"), "w")

vehicle = mavutil.mavlink_connection(AIR_UART, baud=AIR_BAUD, source_system=VEHICLE_SYSID,
                                     source_component=VEHICLE_COMPID, dialect="common", force_mavlink2=True)
gcs = mavutil.mavlink_connection(f"udpout:{ground_ip}:{GROUND_PORT}", source_system=GCS_SYSID,
                                 source_component=GCS_COMPID, dialect="common", force_mavlink2=True)

t0 = time.time()


def record(kind, **fields):
    events.write(json.dumps({"t": round(time.time() - t0, 3), "kind": kind, **fields}) + "\n")


def record_rx(side, msg):
    record("rx", side=side, type=msg.get_type(), sysid=msg.get_srcSystem(), compid=msg.get_srcComponent(),
           seq=msg.get_seq(), fields={k: v for k, v in msg.to_dict().items() if k != "mavpackettype"})


def vehicle_telemetry(name, t):
    alt = min(100.0, t * 2.0)
    lat = HOME_LAT + t * 10.0 / 111320.0
    if name == "HEARTBEAT":
        vehicle.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR, mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                                   mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0,
                                   mavutil.mavlink.MAV_STATE_ACTIVE)
    elif name == "GLOBAL_POSITION_INT":
        vehicle.mav.global_position_int_send(int(t * 1000), int(lat * 1e7), int(HOME_LON * 1e7), int(alt * 1000),
                                             int(alt * 1000), 1000, 0, 0, 0)
    elif name == "SYS_STATUS":
        vehicle.mav.sys_status_send(0, 0, 0, 400, 12600 - int(t * 5), 800, max(20, 100 - int(t / 10)), 0, 0, 0, 0,
                                    0, 0)
    elif name == "VFR_HUD":
        vehicle.mav.vfr_hud_send(10.0, 10.0, 0, 50, alt, 0.0)
    elif name == "ATTITUDE":
        vehicle.mav.attitude_send(int(t * 1000), 0.01, -0.02, 0.0, 0.0, 0.0, 0.0)


def summarize():
    sent = {}
    acks = {}
    counts = {"vehicle": {}, "gcs": {}}
    params_received = set()
    with open(os.path.join(report_dir, "events.jsonl")) as f:
        for line in f:
            e = json.loads(line)
            if e["kind"] == "gcs_command_sent":
                sent[e["index"]] = e["t"]
            elif e["kind"] == "rx":
                counts[e["side"]][e["type"]] = counts[e["side"]].get(e["type"], 0) + 1
                if e["side"] == "gcs" and e["type"] == "COMMAND_ACK":
                    acks.setdefault(e["fields"]["result_param2"], e["t"])
                if e["side"] == "gcs" and e["type"] == "PARAM_VALUE":
                    params_received.add(e["fields"]["param_index"])
    latencies = [acks[i] - sent[i] for i in sent if i in acks]
    summary = {
        "config": {"duration_s": duration, "vehicle_rates_hz": VEHICLE_RATES_HZ, "command_interval_s": COMMAND_INTERVAL_S,
                   "param_count": PARAM_COUNT, "air_uart": AIR_UART, "ground": f"{ground_ip}:{GROUND_PORT}"},
        "commands_sent": len(sent),
        "command_acks_received": len([i for i in sent if i in acks]),
        "missing_ack_indices": sorted(i for i in sent if i not in acks),
        "ack_round_trip_s": {"min": min(latencies), "median": statistics.median(latencies), "max": max(latencies)}
        if latencies else None,
        "param_values_received": len(params_received),
        "rx_counts": counts,
    }
    with open(os.path.join(report_dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2))


last = {name: 0.0 for name in VEHICLE_RATES_HZ}
last_gcs_hb = 0.0
next_command_t = COMMANDS_START_S
command_index = 0
param_list_sent = False
param_queue = []
last_param_send = 0.0

print(f"bench rig -> {report_dir}  vehicle={AIR_UART}@{AIR_BAUD}  gcs=udp:{ground_ip}:{GROUND_PORT}  {duration}s",
      flush=True)
record("config", air_uart=AIR_UART, ground=f"{ground_ip}:{GROUND_PORT}", duration_s=duration)

while True:
    now = time.time()
    t = now - t0
    if t >= duration:
        break

    for name, hz in VEHICLE_RATES_HZ.items():
        if hz > 0 and now - last[name] >= 1.0 / hz:
            last[name] = now
            vehicle_telemetry(name, t)

    if now - last_gcs_hb >= 1.0 / GCS_HEARTBEAT_HZ:
        last_gcs_hb = now
        gcs.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                               mavutil.mavlink.MAV_STATE_ACTIVE)

    if t >= next_command_t:
        next_command_t += COMMAND_INTERVAL_S
        command_index += 1
        gcs.mav.command_long_send(VEHICLE_SYSID, VEHICLE_COMPID, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                  float(command_index), 0, 0, 0, 0, 0, 0)
        record("gcs_command_sent", index=command_index)

    if PARAM_LIST_AT_S and not param_list_sent and t >= PARAM_LIST_AT_S:
        param_list_sent = True
        gcs.mav.param_request_list_send(VEHICLE_SYSID, VEHICLE_COMPID)
        record("gcs_param_request_list_sent")

    if param_queue and now - last_param_send >= 1.0 / PARAM_SEND_HZ:
        last_param_send = now
        index = param_queue.pop(0)
        vehicle.mav.param_value_send(f"BENCH_P{index:02d}".encode(), float(index),
                                     mavutil.mavlink.MAV_PARAM_TYPE_REAL32, PARAM_COUNT, index)
        record("vehicle_param_value_sent", index=index)

    while (msg := vehicle.recv_match(blocking=False)) is not None:
        if msg.get_type() == "BAD_DATA":
            record("bad_data", side="vehicle", data=bytes(msg.get_msgbuf()).hex())
            continue
        record_rx("vehicle", msg)
        if msg.get_type() == "COMMAND_LONG" and msg.target_system == VEHICLE_SYSID:
            vehicle.mav.command_ack_send(msg.command, mavutil.mavlink.MAV_RESULT_ACCEPTED, 0, int(msg.param1),
                                         msg.get_srcSystem(), msg.get_srcComponent())
            record("vehicle_ack_sent", index=int(msg.param1))
        elif msg.get_type() == "PARAM_REQUEST_LIST" and msg.target_system == VEHICLE_SYSID:
            param_queue = list(range(PARAM_COUNT))

    while (msg := gcs.recv_match(blocking=False)) is not None:
        if msg.get_type() == "BAD_DATA":
            record("bad_data", side="gcs", data=bytes(msg.get_msgbuf()).hex())
            continue
        record_rx("gcs", msg)

    time.sleep(0.01)

events.close()
summarize()
