#!/usr/bin/env python3
"""Measure how reliably a one-shot MAVLink request survives the mesh.

Periodic streams repeat until a copy gets through, so they make a lossy link look healthy.
A single command has no second chance. This probe fires MAV_CMD_REQUEST_MESSAGE repeatedly
at short spacing and counts replies per attempt, so one-shot delivery can be measured
directly instead of inferred from stream traffic.

Cross-check the result against PX4's ulog: if `vehicle_command_ack` contains no command 512,
the requests never reached PX4 and the loss is on the ground-to-air path.

Usage:

    set -a; . mavlink_tests/px4_sitl_mesh_test.env; set +a
    MAVLINK20=1 mavlink_tests/.venv-px4-mesh-test/bin/python mavlink_tests/direct_send_probe.py \
        --px4-dir "$PX4_DIR" --air-uart "$(readlink -f "$AIR_UART")" --ground-host "$GROUND_HOST"
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import Counter
from pathlib import Path

from px4_sitl_mesh_test import MavlinkUdpEndpoint, Px4Session
from pymavlink import mavutil


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--px4-dir", required=True)
    parser.add_argument("--air-uart", required=True)
    parser.add_argument("--air-baud", type=int, default=57600)
    parser.add_argument("--ground-host", required=True)
    parser.add_argument("--ground-port", type=int, default=14550)
    parser.add_argument("--local-port", type=int, default=14600)
    parser.add_argument("--target-sysid", type=int, default=1)
    parser.add_argument("--target-compid", type=int, default=1)
    parser.add_argument("--max-rate-bps", type=int, default=1000)
    parser.add_argument("--build-timeout", type=int, default=1200)
    parser.add_argument("--attempts", type=int, default=10)
    parser.add_argument("--spacing", type=float, default=2.0)
    parser.add_argument("--drain", type=float, default=6.0)
    parser.add_argument("--log", default="mavlink_tests/reports/direct-send-probe-px4.log")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    log_path = Path(args.log)
    log_path.parent.mkdir(parents=True, exist_ok=True)

    session = Px4Session(
        Path(args.px4_dir),
        args.air_uart,
        args.air_baud,
        args.max_rate_bps,
        args.build_timeout,
        log_path,
    )
    print("starting PX4 ...", flush=True)
    session.start()
    print("PX4 up, serial instance configured", flush=True)

    endpoint = MavlinkUdpEndpoint(args.ground_host, args.ground_port, args.local_port)
    counts: Counter[str] = Counter()
    version_latencies: list[float] = []
    ack_latencies: list[float] = []

    # Register with the ground node's UDP server and wait for the vehicle to appear.
    for _ in range(6):
        endpoint.send_heartbeat()
        for message in endpoint.receive(0.5):
            counts[message.get_type()] += 1
        if counts.get("HEARTBEAT"):
            break
    print(f"registered; pre-probe counts={dict(counts)}", flush=True)

    started = time.monotonic()
    try:
        for attempt in range(1, args.attempts + 1):
            sent_at = time.monotonic()
            version_before = counts.get("AUTOPILOT_VERSION", 0)
            ack_before = counts.get("COMMAND_ACK", 0)
            endpoint.send_request_autopilot_version(args.target_sysid, args.target_compid)

            deadline = sent_at + args.spacing
            while time.monotonic() < deadline:
                remaining = max(0.0, deadline - time.monotonic())
                for message in endpoint.receive(min(0.3, remaining)):
                    kind = message.get_type()
                    counts[kind] += 1
                    if kind == "AUTOPILOT_VERSION":
                        version_latencies.append(time.monotonic() - sent_at)
                    elif kind == "COMMAND_ACK":
                        ack_latencies.append(time.monotonic() - sent_at)
                        counts[f"ACK_cmd_{int(message.command)}"] += 1

            print(
                f"  attempt {attempt:2d}: "
                f"AUTOPILOT_VERSION={counts.get('AUTOPILOT_VERSION', 0) - version_before} "
                f"COMMAND_ACK={counts.get('COMMAND_ACK', 0) - ack_before}",
                flush=True,
            )
            endpoint.send_heartbeat()

        drain_end = time.monotonic() + args.drain
        while time.monotonic() < drain_end:
            for message in endpoint.receive(0.3):
                kind = message.get_type()
                counts[kind] += 1
                if kind == "COMMAND_ACK":
                    counts[f"ACK_cmd_{int(message.command)}"] += 1
    finally:
        elapsed = time.monotonic() - started
        requested = mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION
        print("\n=== PROBE RESULT ===")
        print(f"probe window: {elapsed:.1f}s over {args.attempts} attempts at {args.spacing}s spacing")
        print(f"requested message id: {requested}")
        print(f"AUTOPILOT_VERSION replies: {counts.get('AUTOPILOT_VERSION', 0)} / {args.attempts}")
        print(f"COMMAND_ACK replies:       {counts.get('COMMAND_ACK', 0)} / {args.attempts}")
        if version_latencies:
            print(
                f"AUTOPILOT_VERSION latency: min {min(version_latencies):.2f}s "
                f"max {max(version_latencies):.2f}s"
            )
        if ack_latencies:
            print(f"COMMAND_ACK latency:       min {min(ack_latencies):.2f}s max {max(ack_latencies):.2f}s")
        print(f"all message counts: {dict(sorted(counts.items()))}")
        print(
            "\nCross-check PX4's newest ulog for command 512 in vehicle_command_ack. "
            "Absent means the requests never reached PX4."
        )
        endpoint.close()
        session.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
