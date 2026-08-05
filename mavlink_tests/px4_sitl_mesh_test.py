#!/usr/bin/env python3
"""Capture PX4 SIH <-> Meshtastic MAVLink traffic without judging it."""

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
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import pexpect
from pymavlink import mavutil


class HarnessError(RuntimeError):
    pass


class PreflightError(HarnessError):
    pass


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def json_safe(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, bytes):
        return {"hex": value.hex(), "length": len(value)}
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return repr(value)


class EventLog:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.started = time.monotonic()
        self.file = path.open("w", encoding="utf-8")
        self.counts: Counter[str] = Counter()

    def write(self, event: str, **fields: Any) -> None:
        self.counts[event] += 1
        row = {
            "time_utc": utc_now(),
            "elapsed_s": round(time.monotonic() - self.started, 6),
            "event": event,
            **{k: json_safe(v) for k, v in fields.items()},
        }
        self.file.write(json.dumps(row, sort_keys=True) + "\n")
        self.file.flush()

    def close(self) -> None:
        self.file.close()


class UdpWriter:
    def __init__(self, sock: socket.socket, destination: tuple[str, int], log: EventLog) -> None:
        self.sock = sock
        self.destination = destination
        self.log = log
        self.datagrams = 0
        self.bytes = 0

    def write(self, data: bytes) -> int:
        written = self.sock.sendto(data, self.destination)
        self.datagrams += 1
        self.bytes += written
        self.log.write(
            "udp_tx",
            destination_host=self.destination[0],
            destination_port=self.destination[1],
            length=written,
            raw_hex=data[:written].hex(),
        )
        return written


class MavlinkUdpEndpoint:
    def __init__(self, host: str, port: int, local_port: int, log: EventLog) -> None:
        self.destination = (socket.gethostbyname(host), port)
        self.log = log
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", local_port))
        self.writer = UdpWriter(self.sock, self.destination, log)
        self.tx = mavutil.mavlink.MAVLink(
            self.writer,
            srcSystem=255,
            srcComponent=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
        )
        self.rx = mavutil.mavlink.MAVLink(None)
        self.rx.robust_parsing = True
        self.rx_datagrams = 0
        self.rx_bytes = 0
        self.rx_messages: Counter[str] = Counter()
        self.rx_sources: Counter[str] = Counter()
        self.tx_messages: Counter[str] = Counter()

    def close(self) -> None:
        self.sock.close()

    def log_tx_message(self, message_type: str, **fields: Any) -> None:
        self.tx_messages[message_type] += 1
        self.log.write("mavlink_tx_message", message_type=message_type, **fields)

    def send_heartbeat(self) -> None:
        self.log_tx_message(
            "HEARTBEAT",
            source_system=255,
            source_component=int(mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER),
            type=int(mavutil.mavlink.MAV_TYPE_GCS),
            autopilot=int(mavutil.mavlink.MAV_AUTOPILOT_INVALID),
        )
        self.tx.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
            3,
        )

    def send_control_high_latency(self, target_system: int, target_component: int, enable: bool) -> None:
        """Assert the high-latency link. PX4 only honours this in IRIDIUM mode."""
        self.log_tx_message(
            "COMMAND_LONG",
            command=int(mavutil.mavlink.MAV_CMD_CONTROL_HIGH_LATENCY),
            target_system=target_system,
            target_component=target_component,
            confirmation=0,
            param1_enable=1 if enable else 0,
        )
        self.tx.command_long_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_CMD_CONTROL_HIGH_LATENCY,
            0,
            1 if enable else 0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    def send_request_version(self, target_system: int, target_component: int) -> None:
        self.log_tx_message(
            "COMMAND_LONG",
            command=int(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE),
            target_system=target_system,
            target_component=target_component,
            confirmation=0,
            param1_message_id=int(mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION),
        )
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

    def send_control(self, target_system: int, target_component: int) -> None:
        self.log_tx_message(
            "COMMAND_LONG",
            command=int(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL),
            target_system=target_system,
            target_component=target_component,
            confirmation=0,
            param1_message_id=int(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT),
            param2_interval_us=1_000_000,
        )
        self.tx.command_long_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,
            1_000_000,
            0,
            0,
            0,
            0,
            0,
        )

    def receive_once(self, timeout_s: float) -> None:
        self.sock.settimeout(max(0.0, timeout_s))
        try:
            data, source = self.sock.recvfrom(65535)
        except socket.timeout:
            return

        self.rx_datagrams += 1
        self.rx_bytes += len(data)
        self.log.write(
            "udp_rx",
            source_host=source[0],
            source_port=source[1],
            length=len(data),
            raw_hex=data.hex(),
        )

        try:
            messages = list(self.rx.parse_buffer(data) or [])
        except Exception as exc:
            self.log.write(
                "mavlink_parse_exception",
                exception_type=type(exc).__name__,
                exception=str(exc),
                raw_hex=data.hex(),
            )
            return

        if not messages:
            self.log.write("mavlink_rx_unparsed_datagram", raw_hex=data.hex())
            return

        for message in messages:
            message_type = message.get_type()
            sysid = int(message.get_srcSystem())
            compid = int(message.get_srcComponent())
            self.rx_messages[message_type] += 1
            self.rx_sources[f"{sysid}:{compid}"] += 1
            try:
                decoded = message.to_dict()
            except Exception as exc:
                decoded = {"decode_exception": f"{type(exc).__name__}: {exc}"}
            try:
                sequence = int(message.get_header().seq)
            except Exception:
                sequence = None
            self.log.write(
                "mavlink_rx_message",
                message_type=message_type,
                source_system=sysid,
                source_component=compid,
                sequence=sequence,
                decoded=decoded,
            )


class Px4Session:
    PROMPT = re.compile(r"pxh>\s*")

    def __init__(
        self,
        px4_dir: Path,
        air_uart: str,
        air_baud: int,
        max_rate_bps: int,
        build_timeout: int,
        console_path: Path,
        log: EventLog,
        px4_mode: str,
    ) -> None:
        self.px4_dir = px4_dir
        self.air_uart = air_uart
        self.air_baud = air_baud
        self.max_rate_bps = max_rate_bps
        self.build_timeout = build_timeout
        self.console_path = console_path
        self.log = log
        self.px4_mode = px4_mode
        self.child: pexpect.spawn | None = None
        self.console: Any = None

    def run_command(self, command: str, timeout: int = 30) -> str:
        if not self.child:
            raise HarnessError("PX4 process is not running")
        self.child.sendline(command)
        try:
            # This exact echo match is only pxh transport synchronization, not a test assertion.
            self.child.expect_exact(command, timeout=timeout)
            self.child.expect(self.PROMPT, timeout=timeout)
        except (pexpect.TIMEOUT, pexpect.EOF) as exc:
            raise HarnessError(f"PX4 shell did not complete {command!r}; inspect {self.console_path}") from exc
        return self.child.before or ""

    def run_and_log(self, command: str, timeout: int = 30, tolerate: bool = False) -> str:
        try:
            output = self.run_command(command, timeout)
        except HarnessError as exc:
            self.log.write("px4_shell_error", command=command, error=str(exc), tolerated=tolerate)
            if tolerate:
                return ""
            raise
        self.log.write("px4_shell_output", command=command, output=output)
        return output

    def dump_status(self, stage: str) -> None:
        for command in ("mavlink status", "mavlink status streams"):
            try:
                output = self.run_command(command, 15)
                self.log.write("px4_status_dump", stage=stage, command=command, output=output)
            except HarnessError as exc:
                self.log.write(
                    "px4_shell_error",
                    stage=stage,
                    command=command,
                    error=str(exc),
                    tolerated=True,
                )

    def start(self) -> None:
        env = os.environ.copy()
        env.setdefault("PX4_SIM_SPEED_FACTOR", "1")
        self.console = self.console_path.open("w", encoding="utf-8")
        self.log.write(
            "px4_launch",
            cwd=str(self.px4_dir),
            argv=["make", "px4_sitl_sih", "sihsim_quadx"],
        )
        self.child = pexpect.spawn(
            "make",
            ["px4_sitl_sih", "sihsim_quadx"],
            cwd=str(self.px4_dir),
            env=env,
            encoding="utf-8",
            codec_errors="replace",
            timeout=self.build_timeout,
            maxread=65536,
            searchwindowsize=65536,
        )
        self.child.logfile = self.console
        try:
            self.child.expect(self.PROMPT)
        except (pexpect.TIMEOUT, pexpect.EOF) as exc:
            raise HarnessError(f"PX4 did not reach pxh; inspect {self.console_path}") from exc

        self.run_and_log("mavlink stop -u 18570", 10, tolerate=True)
        # `-m custom` does NOT mean "only what I configure". PX4 adds HEARTBEAT and STATUSTEXT
        # unconditionally for every mode except IRIDIUM, and HEARTBEAT is a fixed-rate stream that
        # cannot be disabled with `-r 0`. IRIDIUM is the only mode that carries HIGH_LATENCY2
        # alone, which is what this link is sized for.
        #
        # Caveat when reading a capture: IRIDIUM transmission is gated on GCS connection state.
        # PX4 stops transmitting once it believes a GCS is connected and resumes when the link is
        # considered lost, unless commanded on with MAV_CMD_CONTROL_HIGH_LATENCY. A quiet capture
        # is therefore not automatically a transport failure.
        self.run_and_log(
            f"mavlink start -d {self.air_uart} -b {self.air_baud} "
            f"-m {self.px4_mode} -r {self.max_rate_bps} -Z"
        )
        self.run_and_log(f"mavlink stream -d {self.air_uart} -s HIGH_LATENCY2 -r 0.5")
        if self.px4_mode != "iridium":
            # Every non-IRIDIUM mode adds STATUSTEXT at 20 Hz and 66 bytes. HEARTBEAT is also
            # added and is a constant-rate stream that cannot be turned off.
            self.run_and_log(f"mavlink stream -d {self.air_uart} -s STATUSTEXT -r 0")
        self.dump_status("startup")

    def stop(self) -> None:
        child = self.child
        if not child:
            return
        try:
            if child.isalive():
                self.run_and_log(f"mavlink stop -d {self.air_uart}", 10, tolerate=True)
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
            if self.console:
                self.console.close()
                self.console = None


def validate_args(args: argparse.Namespace) -> None:
    args.px4_dir = Path(args.px4_dir).expanduser().resolve()
    if not (args.px4_dir / "Makefile").is_file():
        raise PreflightError(f"Not a PX4-Autopilot checkout: {args.px4_dir}")
    if any(c.isspace() for c in args.air_uart):
        raise PreflightError("AIR_UART must not contain whitespace")
    air_uart = Path(args.air_uart)
    if not air_uart.exists():
        raise PreflightError(f"Air UART does not exist: {air_uart}")
    if not os.access(air_uart, os.R_OK | os.W_OK):
        raise PreflightError(f"Air UART is not readable and writable: {air_uart}")
    try:
        socket.gethostbyname(args.ground_host)
    except socket.gaierror as exc:
        raise PreflightError(f"Cannot resolve ground host: {args.ground_host}") from exc

    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.bind(("0.0.0.0", args.local_port))
    except OSError as exc:
        raise PreflightError(f"UDP port {args.local_port} is already in use") from exc
    finally:
        probe.close()

    for name in ("warmup", "probe_interval", "control_offset", "drain"):
        if getattr(args, name) < 0:
            raise PreflightError(f"--{name.replace('_', '-')} must be non-negative")
    if args.probe_attempts < 0:
        raise PreflightError("--probe-attempts must be non-negative")


def run_capture(endpoint: MavlinkUdpEndpoint, log: EventLog, args: argparse.Namespace) -> None:
    started = time.monotonic()
    request_times = [args.warmup + i * args.probe_interval for i in range(args.probe_attempts)]
    control_times = [t + args.control_offset for t in request_times]
    stop_at = max(request_times + control_times, default=args.warmup) + args.drain
    request_index = 0
    control_index = 0
    next_heartbeat = 0.0

    log.write(
        "capture_schedule",
        target_system=args.target_sysid,
        target_component=args.target_compid,
        request_times_s=request_times,
        control_times_s=control_times,
        drain_s=args.drain,
        capture_duration_s=stop_at,
    )

    while True:
        elapsed = time.monotonic() - started
        if elapsed >= stop_at:
            break

        # No GCS heartbeat. PX4 disables IRIDIUM transmission whenever it believes a GCS is
        # connected, and that branch ignores the commanded flag, so heart-beating over the only
        # link silences the vehicle. A satellite GCS does not heartbeat; it asserts the link with
        # MAV_CMD_CONTROL_HIGH_LATENCY instead. This also keeps the ground node's UDP client
        # registration alive, which the heartbeat used to provide.
        if elapsed >= next_heartbeat:
            if args.px4_mode == "iridium":
                log.write("probe_send", probe="control_high_latency_enable")
                endpoint.send_control_high_latency(args.target_sysid, args.target_compid, True)
            else:
                # Non-IRIDIUM has no transmit gating, so behave like an ordinary GCS.
                endpoint.send_heartbeat()
            next_heartbeat += 5.0

        while request_index < len(request_times) and elapsed >= request_times[request_index]:
            log.write("probe_send", probe="request_autopilot_version", index=request_index + 1)
            endpoint.send_request_version(args.target_sysid, args.target_compid)
            request_index += 1

        while control_index < len(control_times) and elapsed >= control_times[control_index]:
            log.write("probe_send", probe="set_heartbeat_interval", index=control_index + 1)
            endpoint.send_control(args.target_sysid, args.target_compid)
            control_index += 1

        due = [stop_at, next_heartbeat]
        if request_index < len(request_times):
            due.append(request_times[request_index])
        if control_index < len(control_times):
            due.append(control_times[control_index])
        timeout = min(0.25, max(0.0, min(due) - (time.monotonic() - started)))
        endpoint.receive_once(timeout)

    while True:
        count = endpoint.rx_datagrams
        endpoint.receive_once(0.05)
        if endpoint.rx_datagrams == count:
            break


def write_summary(
    path: Path,
    args: argparse.Namespace,
    log: EventLog,
    endpoint: MavlinkUdpEndpoint | None,
    started_at: str,
    error: str | None,
) -> None:
    summary: dict[str, Any] = {
        "format": "meshtastic-mavlink-raw-capture-v1",
        "started_at": started_at,
        "finished_at": utc_now(),
        "completed_without_harness_error": error is None,
        "fatal_error": error,
        "configuration": {
            "px4_dir": str(args.px4_dir),
            "air_uart": args.air_uart,
            "air_baud": args.air_baud,
            "ground_host": args.ground_host,
            "ground_port": args.ground_port,
            "local_port": args.local_port,
            "target_sysid": args.target_sysid,
            "target_compid": args.target_compid,
            "max_rate_bps": args.max_rate_bps,
            "warmup_s": args.warmup,
            "probe_attempts": args.probe_attempts,
            "probe_interval_s": args.probe_interval,
            "control_offset_s": args.control_offset,
            "drain_s": args.drain,
        },
        "artifacts": {"events_jsonl": "events.jsonl", "px4_console_log": "px4.log"},
        "event_counts": dict(sorted(log.counts.items())),
    }
    if endpoint:
        summary.update(
            {
                "udp": {
                    "tx_datagrams": endpoint.writer.datagrams,
                    "tx_bytes": endpoint.writer.bytes,
                    "rx_datagrams": endpoint.rx_datagrams,
                    "rx_bytes": endpoint.rx_bytes,
                },
                "sent_message_counts": dict(sorted(endpoint.tx_messages.items())),
                "received_message_counts": dict(sorted(endpoint.rx_messages.items())),
                "received_source_counts": dict(sorted(endpoint.rx_sources.items())),
            }
        )
    path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Dump the PX4 SIH UART/LoRa/UDP path without generating protocol verdicts."
    )
    parser.add_argument("--px4-dir", required=True)
    parser.add_argument("--air-uart", required=True)
    parser.add_argument("--air-baud", type=int, default=57600)
    parser.add_argument("--ground-host", required=True)
    parser.add_argument("--ground-port", type=int, default=14550)
    parser.add_argument("--local-port", type=int, default=14600)
    parser.add_argument("--target-sysid", type=int, default=1)
    parser.add_argument("--target-compid", type=int, default=1)
    parser.add_argument("--max-rate-bps", type=int, default=1000)
    parser.add_argument("--px4-mode", default="iridium")
    parser.add_argument("--build-timeout", type=int, default=1200)
    parser.add_argument("--warmup", type=float, default=5.0)
    parser.add_argument("--probe-attempts", type=int, default=10)
    parser.add_argument("--probe-interval", type=float, default=3.0)
    parser.add_argument("--control-offset", type=float, default=1.0)
    parser.add_argument("--drain", type=float, default=8.0)
    parser.add_argument("--report-root", default="mavlink_tests/reports")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    started_at = utc_now()
    report_root = Path(args.report_root).expanduser()
    if not report_root.is_absolute():
        report_root = Path.cwd() / report_root
    report_dir = report_root / f"px4-sitl-mesh-capture-{datetime.now().strftime('%Y%m%d-%H%M%S')}"
    report_dir.mkdir(parents=True, exist_ok=False)

    events_path = report_dir / "events.jsonl"
    summary_path = report_dir / "capture.json"
    px4_path = report_dir / "px4.log"
    log = EventLog(events_path)
    endpoint: MavlinkUdpEndpoint | None = None
    px4: Px4Session | None = None
    error: str | None = None
    exit_code = 0

    log.write("capture_created", report_dir=str(report_dir), argv=sys.argv)
    try:
        validate_args(args)
        log.write(
            "configuration_resolved",
            px4_dir=str(args.px4_dir),
            air_uart=args.air_uart,
            ground_host=socket.gethostbyname(args.ground_host),
        )
        px4 = Px4Session(
            args.px4_dir,
            args.air_uart,
            args.air_baud,
            args.max_rate_bps,
            args.build_timeout,
            px4_path,
            log,
            args.px4_mode,
        )
        px4.start()
        endpoint = MavlinkUdpEndpoint(args.ground_host, args.ground_port, args.local_port, log)
        log.write(
            "udp_endpoint_open",
            local_port=args.local_port,
            destination_host=endpoint.destination[0],
            destination_port=endpoint.destination[1],
        )
        run_capture(endpoint, log, args)
        px4.dump_status("after_capture")
        log.write("capture_complete")
    except KeyboardInterrupt:
        error = "Interrupted"
        exit_code = 130
        log.write("capture_interrupted")
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        exit_code = 2
        log.write("capture_error", exception_type=type(exc).__name__, exception=str(exc))
    finally:
        if endpoint:
            endpoint.close()
            log.write("udp_endpoint_closed")
        if px4:
            px4.stop()
            log.write("px4_stopped")
        write_summary(summary_path, args, log, endpoint, started_at, error)
        log.close()

    print(f"Capture summary: {summary_path}")
    print(f"Raw event log:   {events_path}")
    print(f"PX4 console log: {px4_path}")
    if error:
        print(f"Capture machinery error: {error}", file=sys.stderr)
    else:
        print("Capture completed. No protocol verdict was generated.")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
