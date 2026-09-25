#!/usr/bin/env python3
"""INAV SITL on the JSBSim bench, flying over the Meshtastic MAVLink bridge.

INAV's MAVLink port (SITL TCP) is bridged with socat onto the air node's UART; a pymavlink GCS
talks to the ground node's UDP endpoint. INAV starts in high-latency mode. The quad takes off,
holds, flies out and returns home under INAV's own navigation, so HIGH_LATENCY2 carries changing
state. The GCS heartbeats, sends a CONTROL_HIGH_LATENCY(1) probe on a fixed clock, and switches
high latency off for a window mid-flight to put INAV's full stream on the link.

Nothing is gated on what arrives. events.jsonl holds every message the GCS receives with all
fields, every probe sent, and the bench's ground truth once a second; summary.json only counts.

usage: inav_hl_bench.py <ground-node-ip> [SITL.elf]
"""

# ═══════════════════════ CONFIG ═══════════════════════
AIR_UART = "/dev/ttyUSB0"  # USB-UART wired to the air node's GPIO47(RX)/GPIO48(TX)
AIR_BAUD = 57600  # must match serial.baud on the air node
GROUND_PORT = 14550  # ground node MAVLink UDP server
BENCH_DIR = "/media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/inav_development/inav-jsbsim-bench"
DEFAULT_SITL = "/media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/inav_development/inav/cmake/build_SITL/bin/SITL.elf"
INAV_CLI = ["set mavlink_port1_high_latency = ON"]

GCS_SYSID, GCS_COMPID = 255, 190
VEHICLE_SYSID, VEHICLE_COMPID = 1, 1
GCS_HEARTBEAT_HZ = 1.0
PROBE_INTERVAL_S = 15.0  # CONTROL_HIGH_LATENCY(1): idempotent, INAV always ACKs it
PROBE_START_S = 10.0

# Flight timeline, seconds from start
ARM_AT_S = 15.0
CLIMB_S = 5.0
FLY_OUT_AT_S = 30.0
FLY_OUT_S = 20.0
HL_OFF_AT_S = 60.0  # CONTROL_HIGH_LATENCY(0): INAV's full stream onto the link
HL_ON_AT_S = 90.0  # CONTROL_HIGH_LATENCY(1)
RTH_AT_S = 100.0
DURATION_S = 200.0
REPORT_ROOT = "mavlink_tests/reports"
# ══════════════════════════════════════════════════════

import json
import os
import subprocess
import sys
import time

sys.path.insert(0, BENCH_DIR)
from inavbench import ArmingError, Bench
from pymavlink import mavutil

ground_ip = sys.argv[1]
sitl = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_SITL
report_dir = os.path.join(REPORT_ROOT, time.strftime("inav-hl-%Y%m%d-%H%M%S"))
os.makedirs(report_dir)
events = open(os.path.join(report_dir, "events.jsonl"), "w")
t0 = time.time()


def record(kind, **fields):
    events.write(json.dumps({"t": round(time.time() - t0, 3), "kind": kind, **fields}) + "\n")


def send_hl(gcs, on):
    gcs.mav.command_long_send(VEHICLE_SYSID, VEHICLE_COMPID, mavutil.mavlink.MAV_CMD_CONTROL_HIGH_LATENCY, 0,
                              1.0 if on else 0.0, 0, 0, 0, 0, 0, 0)


record("config", air_uart=AIR_UART, ground=f"{ground_ip}:{GROUND_PORT}", sitl=sitl, inav_cli=INAV_CLI)
with Bench("quad", sitl=sitl, cli=INAV_CLI, log=os.path.join(report_dir, "flight.csv")) as b:
    bridge = subprocess.Popen(["socat", "-d", f"TCP:127.0.0.1:{b.gcs_mavlink_port}",
                               f"{AIR_UART},b{AIR_BAUD},raw,echo=0"], stderr=open(os.path.join(report_dir, "socat.log"), "w"))
    gcs = mavutil.mavlink_connection(f"udpout:{ground_ip}:{GROUND_PORT}", source_system=GCS_SYSID,
                                     source_component=GCS_COMPID, dialect="common", force_mavlink2=True)
    print(f"inav hl bench -> {report_dir}  SITL MAVLink tcp:{b.gcs_mavlink_port} <-> {AIR_UART}@{AIR_BAUD}, "
          f"GCS udp:{ground_ip}:{GROUND_PORT}", flush=True)
    done = set()
    last_hb = last_truth = 0.0
    next_probe = PROBE_START_S
    probes = 0
    try:
        while (t := time.time() - t0) < DURATION_S:
            b.run(0.05)
            now = time.time()
            if now - last_hb >= 1.0 / GCS_HEARTBEAT_HZ:
                last_hb = now
                gcs.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                       mavutil.mavlink.MAV_STATE_ACTIVE)
            if now - last_truth >= 1.0:
                last_truth = now
                tr = b.truth()
                record("truth", lat=tr["lat"], lon=tr["lon"], agl_m=round(tr["agl_m"], 2), yaw=round(tr["yaw"], 1),
                       armed=b.fc.armed, modes=sorted(b.active_modes()))
            if t >= next_probe:
                next_probe += PROBE_INTERVAL_S
                # The probe itself re-enables high latency, so it must stay quiet while that is switched off.
                if not HL_OFF_AT_S <= t < HL_ON_AT_S:
                    probes += 1
                    send_hl(gcs, True)
                    record("probe_sent", index=probes)

            if t >= ARM_AT_S and "arm" not in done:
                done.add("arm")
                try:
                    b.mode("ANGLE")
                    b.arm()
                    b.sticks(throttle=0.62)
                    record("flight", step="armed, climbing")
                except ArmingError as e:
                    record("flight", step="arm refused", blockers=str(e))
            if t >= ARM_AT_S + CLIMB_S and "hold" not in done and b.fc.armed:
                done.add("hold")
                b.sticks(throttle=0.5)
                b.mode("NAV POSHOLD")
                record("flight", step="poshold")
            if t >= FLY_OUT_AT_S and "out" not in done and b.fc.armed:
                done.add("out")
                b.sticks(pitch=-0.6)  # as in the bench's quad_flight example: flies south
                record("flight", step="flying out")
            if t >= FLY_OUT_AT_S + FLY_OUT_S and "stop" not in done and "out" in done:
                done.add("stop")
                b.sticks(pitch=0.0)
                record("flight", step="holding")
            if t >= HL_OFF_AT_S and "hl_off" not in done:
                done.add("hl_off")
                send_hl(gcs, False)
                record("hl_command_sent", on=False)
            if t >= HL_ON_AT_S and "hl_on" not in done:
                done.add("hl_on")
                send_hl(gcs, True)
                record("hl_command_sent", on=True)
            if t >= RTH_AT_S and "rth" not in done and b.fc.armed:
                done.add("rth")
                b.mode("NAV RTH")
                record("flight", step="rth")

            while (msg := gcs.recv_match(blocking=False)) is not None:
                if msg.get_type() == "BAD_DATA":
                    record("bad_data", data=bytes(msg.get_msgbuf()).hex())
                    continue
                record("rx", type=msg.get_type(), sysid=msg.get_srcSystem(), compid=msg.get_srcComponent(),
                       seq=msg.get_seq(), fields={k: v for k, v in msg.to_dict().items() if k != "mavpackettype"})
    finally:
        bridge.terminate()
        events.close()

counts = {}
acks = []
hl2 = []
with open(os.path.join(report_dir, "events.jsonl")) as f:
    for line in f:
        e = json.loads(line)
        if e["kind"] == "rx":
            counts[e["type"]] = counts.get(e["type"], 0) + 1
            if e["type"] == "COMMAND_ACK":
                acks.append((e["t"], e["fields"]["command"], e["fields"]["result"]))
            if e["type"] == "HIGH_LATENCY2":
                hl2.append(e["t"])
summary = {
    "probes_sent": probes,
    "command_acks": len(acks),
    "command_ack_detail": acks,
    "hl2_received": len(hl2),
    "hl2_inter_arrival_s": [round(b2 - a2, 2) for a2, b2 in zip(hl2, hl2[1:])],
    "rx_counts": counts,
}
with open(os.path.join(report_dir, "summary.json"), "w") as f:
    json.dump(summary, f, indent=2)
print(json.dumps(summary, indent=2))
