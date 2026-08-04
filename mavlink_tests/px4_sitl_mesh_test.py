#!/usr/bin/env python3
"""Hands-off PX4 SIH -> UART -> Meshtastic mesh -> UDP integration test."""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
import socket
import sys
import time
from collections import Counter
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import pexpect
from pymavlink import mavutil


class HarnessError(RuntimeError):
    """Base class for a controlled harness failure."""


class PreflightError(HarnessError):
    """A required local resource or configuration is unavailable."""


class TestFailure(HarnessError):
    """The end-to-end MAVLink test did not meet its acceptance criteria."""


@dataclass
class TestResult:
    passed: bool
    started_at: str
    finished_at: str
    px4_dir: str
    air_uart: str
    air_baud: int
    ground_host: str
    ground_port: int
    local_port: int
    expected_sysid: int
    vehicle_sysid: int | None = None
    vehicle_compid: int | None = None
    heartbeat_latency_s: float | None = None
    high_latency2_latency_s: float | None = None
    command_ack_latency_s: float | None = None
    autopilot_version_latency_s: float | None = None
    high_latency2: dict[str, Any] | None = None
    command_ack_result: int | None = None
    flight_sw_version: int | None = None
    message_counts: dict[str, int] | None = None
    report_dir: str | None = None
    error: str | None = None


class UdpWriter:
    """File-like object used by pymavlink for datagram transmission."""

    def __init__(self, sock: socket.socket, destination: tuple[str, int]) -> None:
        self.sock = sock
        self.destination = destination

    def write(self, data: bytes) -> int:
        return self.sock.sendto(data, self.destination)


class MavlinkUdpEndpoint:
    """Single bound UDP socket that both registers with and receives from the ground node."""

    def __init__(self, ground_host: str, ground_port: int, local_port: int) -> None:
        self.destination = (socket.gethostbyname(ground_host), ground_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", local_port))
        self.writer = UdpWriter(self.sock, self.destination)
        self.tx = mavutil.mavlink.MAVLink(
            self.writer,
            srcSystem=255,
            srcComponent=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
        )
        self.rx = mavutil.mavlink.MAVLink(None)
        self.rx.robust_parsing = True

    def close(self) -> None:
        self.sock.close()

    def send_heartbeat(self) -> None:
        self.tx.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
            3,
        )

    def send_request_autopilot_version(self, target_system: int, target_component: int) -> None:
        self.tx.command_long_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    def receive(self, timeout_s: float) -> list[Any]:
        self.sock.settimeout(max(0.0, timeout_s))
        try:
            data, _source = self.sock.recvfrom(4096)
        except socket.timeout:
            return []
        parsed = self.rx.parse_buffer(data)
        return list(parsed or [])


class Px4Session:
    """Interactive PX4 SIH process controlled through its pxh shell."""

    PROMPT = re.compile(r"pxh>\s*")

    def __init__(
        self,
        px4_dir: Path,
        air_uart: str,
        air_baud: int,
        max_rate_bps: int,
        build_timeout_s: int,
        log_path: Path,
    ) -> None:
        self.px4_dir = px4_dir
        self.air_uart = air_uart
        self.air_baud = air_baud
        self.max_rate_bps = max_rate_bps
        self.build_timeout_s = build_timeout_s
        self.log_path = log_path
        self.child: pexpect.spawn | None = None
        self.log_file: Any = None

    def start(self) -> None:
        env = os.environ.copy()
        env.setdefault("PX4_SIM_SPEED_FACTOR", "1")
        self.log_file = self.log_path.open("w", encoding="utf-8")
        self.child = pexpect.spawn(
            "make",
            ["px4_sitl_sih", "sihsim_quadx"],
            cwd=str(self.px4_dir),
            env=env,
            encoding="utf-8",
            codec_errors="replace",
            timeout=self.build_timeout_s,
            maxread=65536,
            searchwindowsize=65536,
            preexec_fn=os.setsid,
        )
        self.child.logfile = self.log_file
        try:
            self.child.expect(self.PROMPT)
        except (pexpect.TIMEOUT, pexpect.EOF) as exc:
            raise TestFailure(f"PX4 did not reach the pxh prompt; inspect {self.log_path}") from exc

        # Remove PX4's ordinary localhost GCS route. The verifier binds a different port,
        # but stopping this instance guarantees that all observed traffic uses the radio path.
        try:
            self.run_command("mavlink stop -u 18570", timeout_s=10)
        except HarnessError:
            pass

        start_output = self.run_command(
            f"mavlink start -d {self.air_uart} -b {self.air_baud} "
            f"-m custom -r {self.max_rate_bps} -Z"
        )
        self._reject_command_error("mavlink start", start_output)

        for stream, rate in (("HEARTBEAT", "1"), ("HIGH_LATENCY2", "0.5")):
            output = self.run_command(f"mavlink stream -d {self.air_uart} -s {stream} -r {rate}")
            self._reject_command_error(f"configure {stream}", output)

        status = self.run_command("mavlink status streams")
        if self.air_uart not in status:
            raise TestFailure(
                f"PX4 MAVLink status does not show serial device {self.air_uart}; inspect {self.log_path}"
            )
        if "HIGH_LATENCY2" not in status or "HEARTBEAT" not in status:
            raise TestFailure(
                f"PX4 MAVLink status does not show the required streams; inspect {self.log_path}"
            )

    def run_command(self, command: str, timeout_s: int = 30) -> str:
        if not self.child:
            raise RuntimeError("PX4 process is not running")
        self.child.sendline(command)
        try:
            self.child.expect(self.PROMPT, timeout=timeout_s)
        except (pexpect.TIMEOUT, pexpect.EOF) as exc:
            raise TestFailure(
                f"PX4 shell did not complete command: {command!r}; inspect {self.log_path}"
            ) from exc
        return self.child.before or ""

    @staticmethod
    def _reject_command_error(label: str, output: str) -> None:
        lowered = output.lower()
        markers = (
            "failed to start",
            "no such file",
            "permission denied",
            "already running",
            "error [mavlink]",
        )
        if any(marker in lowered for marker in markers):
            raise TestFailure(f"PX4 failed to {label}: {output.strip()}")

    def stop(self) -> None:
        child = self.child
        if not child:
            return
        try:
            if child.isalive():
                try:
                    self.run_command(f"mavlink stop -d {self.air_uart}", timeout_s=10)
                except HarnessError:
                    pass
                child.sendline("shutdown")
                try:
                    child.expect(pexpect.EOF, timeout=15)
                except (pexpect.TIMEOUT, pexpect.EOF):
                    pass
            if child.isalive():
                try:
                    os.killpg(child.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                time.sleep(1)
            if child.isalive():
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        finally:
            child.close(force=True)
            self.child = None
            if self.log_file:
                self.log_file.close()
                self.log_file = None


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def ensure_udp_port_free(port: int, label: str) -> None:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.bind(("0.0.0.0", port))
    except OSError as exc:
        raise PreflightError(
            f"UDP port {port} is already in use ({label}). Close QGroundControl or the process holding the port."
        ) from exc
    finally:
        probe.close()


def validate_args(args: argparse.Namespace) -> None:
    px4_dir = Path(args.px4_dir).expanduser().resolve()
    if not (px4_dir / "Makefile").is_file():
        raise PreflightError(f"Not a PX4-Autopilot checkout: {px4_dir}")
    args.px4_dir = px4_dir

    if any(character.isspace() for character in args.air_uart):
        raise PreflightError("AIR_UART must not contain whitespace")
    air_uart = Path(args.air_uart)
    if not air_uart.exists():
        raise PreflightError(f"Air UART device does not exist: {air_uart}")
    if not os.access(air_uart, os.R_OK | os.W_OK):
        raise PreflightError(
            f"Air UART is not readable and writable: {air_uart}. Check dialout permissions."
        )

    try:
        socket.gethostbyname(args.ground_host)
    except socket.gaierror as exc:
        raise PreflightError(f"Cannot resolve ground node host: {args.ground_host}") from exc

    ensure_udp_port_free(14550, "PX4/QGC bypass port")
    ensure_udp_port_free(args.local_port, "test GCS receive port")


def high_latency2_snapshot(message: Any) -> dict[str, Any]:
    return {
        "sysid": message.get_srcSystem(),
        "compid": message.get_srcComponent(),
        "latitude": int(message.latitude),
        "longitude": int(message.longitude),
        "altitude_m": int(message.altitude),
        "heading_2deg": int(message.heading),
        "battery_percent": int(message.battery),
        "custom_mode": int(message.custom_mode),
    }


def run_link_test(
    endpoint: MavlinkUdpEndpoint,
    expected_sysid: int,
    timeout_s: int,
    command_attempts: int,
) -> dict[str, Any]:
    started = time.monotonic()
    deadline = started + timeout_s
    next_gcs_heartbeat = 0.0
    heartbeat: Any | None = None
    high_latency2: Any | None = None
    command_ack: Any | None = None
    autopilot_version: Any | None = None
    command_sent_at: float | None = None
    next_command_retry = 0.0
    attempts = 0
    counts: Counter[str] = Counter()
    latencies: dict[str, float] = {}

    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_gcs_heartbeat:
            endpoint.send_heartbeat()
            next_gcs_heartbeat = now + 5.0

        for message in endpoint.receive(min(0.5, max(0.0, deadline - now))):
            message_type = message.get_type()
            if message_type == "BAD_DATA":
                continue
            counts[message_type] += 1

            source_system = message.get_srcSystem()
            if message_type == "HEARTBEAT":
                if expected_sysid and source_system != expected_sysid:
                    continue
                if int(message.autopilot) == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                    continue
                if heartbeat is None:
                    heartbeat = message
                    latencies["heartbeat"] = time.monotonic() - started

            elif message_type == "HIGH_LATENCY2":
                if heartbeat is not None and source_system != heartbeat.get_srcSystem():
                    continue
                if expected_sysid and source_system != expected_sysid:
                    continue
                if high_latency2 is None:
                    high_latency2 = message
                    latencies["high_latency2"] = time.monotonic() - started

            elif message_type == "COMMAND_ACK":
                if (
                    int(message.command) == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE
                    and (heartbeat is None or source_system == heartbeat.get_srcSystem())
                ):
                    command_ack = message
                    if command_sent_at is not None:
                        latencies["command_ack"] = time.monotonic() - command_sent_at

            elif message_type == "AUTOPILOT_VERSION":
                if heartbeat is None or source_system == heartbeat.get_srcSystem():
                    autopilot_version = message
                    if command_sent_at is not None:
                        latencies["autopilot_version"] = time.monotonic() - command_sent_at

        if heartbeat is not None and high_latency2 is not None:
            now = time.monotonic()
            if command_ack is None or autopilot_version is None:
                if attempts < command_attempts and now >= next_command_retry:
                    endpoint.send_request_autopilot_version(
                        heartbeat.get_srcSystem(),
                        heartbeat.get_srcComponent(),
                    )
                    if command_sent_at is None:
                        command_sent_at = now
                    attempts += 1
                    next_command_retry = now + 10.0

        if (
            heartbeat is not None
            and high_latency2 is not None
            and command_ack is not None
            and autopilot_version is not None
        ):
            ack_result = int(command_ack.result)
            accepted_results = {
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
            }
            if ack_result not in accepted_results:
                raise TestFailure(
                    f"PX4 returned COMMAND_ACK result {ack_result} for MAV_CMD_REQUEST_MESSAGE"
                )

            snapshot = high_latency2_snapshot(high_latency2)
            if snapshot["latitude"] == 0 and snapshot["longitude"] == 0:
                raise TestFailure("HIGH_LATENCY2 arrived but contains no valid SIH position")
            if snapshot["battery_percent"] < 0:
                raise TestFailure("HIGH_LATENCY2 arrived but battery percentage is unknown")

            return {
                "vehicle_sysid": heartbeat.get_srcSystem(),
                "vehicle_compid": heartbeat.get_srcComponent(),
                "heartbeat_latency_s": latencies.get("heartbeat"),
                "high_latency2_latency_s": latencies.get("high_latency2"),
                "command_ack_latency_s": latencies.get("command_ack"),
                "autopilot_version_latency_s": latencies.get("autopilot_version"),
                "high_latency2": snapshot,
                "command_ack_result": ack_result,
                "flight_sw_version": int(autopilot_version.flight_sw_version),
                "message_counts": dict(sorted(counts.items())),
            }

    missing = []
    if heartbeat is None:
        missing.append("vehicle HEARTBEAT")
    if high_latency2 is None:
        missing.append("HIGH_LATENCY2")
    if command_ack is None:
        missing.append("COMMAND_ACK")
    if autopilot_version is None:
        missing.append("AUTOPILOT_VERSION")
    raise TestFailure(
        "Timed out waiting for " + ", ".join(missing) + f"; message counts: {dict(counts)}"
    )


def write_result(path: Path, result: TestResult) -> None:
    path.write_text(json.dumps(asdict(result), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Launch PX4 SIH, attach a serial MAVLink instance to the air Meshtastic node, "
            "and verify the complete UART/LoRa/UDP path."
        )
    )
    parser.add_argument("--px4-dir", required=True)
    parser.add_argument("--air-uart", required=True)
    parser.add_argument("--air-baud", type=int, default=115200)
    parser.add_argument("--ground-host", required=True)
    parser.add_argument("--ground-port", type=int, default=14550)
    parser.add_argument("--local-port", type=int, default=14600)
    parser.add_argument("--expected-sysid", type=int, default=1)
    parser.add_argument("--max-rate-bps", type=int, default=1000)
    parser.add_argument("--build-timeout", type=int, default=1200)
    parser.add_argument("--test-timeout", type=int, default=240)
    parser.add_argument("--command-attempts", type=int, default=3)
    parser.add_argument("--report-root", default="mavlink_tests/reports")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    started_at = utc_now()
    result = TestResult(
        passed=False,
        started_at=started_at,
        finished_at=started_at,
        px4_dir=str(args.px4_dir),
        air_uart=args.air_uart,
        air_baud=args.air_baud,
        ground_host=args.ground_host,
        ground_port=args.ground_port,
        local_port=args.local_port,
        expected_sysid=args.expected_sysid,
    )

    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    report_root = Path(args.report_root).expanduser()
    if not report_root.is_absolute():
        report_root = Path.cwd() / report_root
    report_dir = report_root / f"px4-sitl-mesh-{timestamp}"
    report_dir.mkdir(parents=True, exist_ok=False)
    result.report_dir = str(report_dir)
    result_path = report_dir / "result.json"
    px4_log = report_dir / "px4.log"

    px4: Px4Session | None = None
    endpoint: MavlinkUdpEndpoint | None = None

    try:
        validate_args(args)
        result.px4_dir = str(args.px4_dir)

        px4 = Px4Session(
            args.px4_dir,
            args.air_uart,
            args.air_baud,
            args.max_rate_bps,
            args.build_timeout,
            px4_log,
        )
        px4.start()

        endpoint = MavlinkUdpEndpoint(
            args.ground_host,
            args.ground_port,
            args.local_port,
        )
        observations = run_link_test(
            endpoint,
            args.expected_sysid,
            args.test_timeout,
            args.command_attempts,
        )
        for key, value in observations.items():
            setattr(result, key, value)

        result.passed = True
        print("PASS: PX4 SIH UART -> Meshtastic mesh -> UDP")
        print(json.dumps(observations, indent=2, sort_keys=True))
        return 0

    except PreflightError as exc:
        result.error = str(exc)
        print(f"PREFLIGHT FAILED: {exc}", file=sys.stderr)
        return 2
    except HarnessError as exc:
        result.error = str(exc)
        print(f"TEST FAILED: {exc}", file=sys.stderr)
        return 3
    except KeyboardInterrupt:
        result.error = "Interrupted"
        print("INTERRUPTED", file=sys.stderr)
        return 130
    except Exception as exc:
        result.error = f"{type(exc).__name__}: {exc}"
        print(f"HARNESS ERROR: {result.error}", file=sys.stderr)
        return 4
    finally:
        if endpoint is not None:
            endpoint.close()
        if px4 is not None:
            px4.stop()
        result.finished_at = utc_now()
        write_result(result_path, result)
        print(f"Report: {result_path}")


if __name__ == "__main__":
    raise SystemExit(main())
