# PX4 SITL Meshtastic Hardware Test

This harness drives PX4 SIH through a physical UART-connected Meshtastic air node and verifies the complete UART, LoRa, and UDP path with an automated pymavlink endpoint.

## Topology

```text
PX4 SIH on the Linux host
        |
USB-UART adapter at the configured baud
        |
air Meshtastic node UART
        |
Meshtastic LoRa mesh
        |
ground Meshtastic node, remote UDP 14550
        |
automated pymavlink verifier, local UDP 14600
```

The harness itself is the GCS endpoint. PX4's ordinary localhost GCS MAVLink instance on local UDP 18570 is stopped before verification so the result cannot use a direct PX4-to-host path.

## Important build behavior

The harness runs:

```bash
make px4_sitl_sih sihsim_quadx
```

This is an incremental PX4 make invocation. It does not clean the PX4 tree, PlatformIO tree, firmware objects, or Python environment.

Do not pre-clean or separately rebuild PX4. Do not delete `.pio/build`. Do not run the optional native suite as part of this hardware test.

Harness-only changes under `mavlink_tests/` require no Meshtastic reflash. The current ACK-localization phase also changes `src/modules/Mavlink/`, so flash both nodes once with an ordinary incremental target build. Do not clean first.

Typical local sequence:

```bash
pio run -e heltec-wsl-v3
pio run -e heltec-wsl-v3 -t upload --upload-port <node-port>
```

Repeat only the upload command for the second node as appropriate. Use the existing local method and ports; do not use a tool that rewrites project configuration or invalidates the cache.

## Hardware contract

### Air node

- current `mavlink` firmware with ACK tracing;
- Serial module enabled in MAVLink mode;
- RX/TX pins configured;
- baud matching `AIR_BAUD`;
- private secondary channel named exactly `serial`.

Wire a 3.3 V USB-UART adapter:

```text
adapter TX -> air node RX
adapter RX -> air node TX
adapter GND -> air node GND
```

Use `/dev/serial/by-id/...` when adapters expose unique serial values. Use `/dev/serial/by-path/...` when duplicate serial values make `by-id` ambiguous. Do not use 5 V UART logic.

### Ground node

- current `mavlink` firmware with ACK tracing;
- Serial module enabled in MAVLink mode;
- RX and TX unset so it operates as a UDP-only endpoint;
- Wi-Fi connected and reachable from the test host;
- same private secondary channel named exactly `serial`.

### Host

- PX4-Autopilot checkout with normal development dependencies;
- Python 3 with `venv` support;
- permission to open the USB-UART device.

## One-time local configuration

```bash
cp mavlink_tests/px4_sitl_mesh_test.env.example \
   mavlink_tests/px4_sitl_mesh_test.env
```

Set the actual machine-specific values:

```bash
PX4_DIR="/path/to/PX4-Autopilot"
AIR_UART="/dev/serial/by-path/your-adapter"
GROUND_HOST="ground-node-address"
AIR_BAUD=57600
```

The local configuration, virtual environment, and generated reports are ignored by Git.

## Run

From the firmware repository root:

```bash
bash mavlink_tests/run_px4_sitl_mesh_test.sh
```

The runner reuses its existing Python virtual environment and the existing PX4 build cache.

## Automated procedure

The harness:

1. Verifies the PX4 checkout, serial device, ground-node address, and verifier UDP port 14600.
2. Starts PX4 SIH with the incremental make target.
3. Waits for the tested PX4 `pxh>` prompt behavior.
4. Stops PX4's ordinary localhost GCS instance on local UDP 18570.
5. Starts a custom serial MAVLink instance on the physical USB-UART device.
6. Enables only:
   - `HEARTBEAT` at 1 Hz;
   - `HIGH_LATENCY2` at 0.5 Hz.
7. Sends a verifier heartbeat to the ground node every five seconds.
8. Records the first PX4 autopilot heartbeat and valid `HIGH_LATENCY2` snapshot.
9. Sends `MAV_CMD_REQUEST_MESSAGE(AUTOPILOT_VERSION)` through the mesh.
10. Records the returned `AUTOPILOT_VERSION` and any matching `COMMAND_ACK`.
11. Sends a harmless control command:

```text
MAV_CMD_SET_MESSAGE_INTERVAL
message = HEARTBEAT
interval = 1 second
```

This requests the interval already configured by the harness. In the tested PX4 source it remains on the direct ACK path and does not arm, move, land, or materially reconfigure the vehicle.

12. Waits only for the focused ACK grace period, then shuts down and writes the report.

## Truthful result model

The JSON report separates these milestones:

```text
transport_pass
telemetry_pass
command_delivery_pass
response_return_pass
command_ack_pass
control_command_ack_pass
strict_pass
```

A later ACK failure no longer erases earlier evidence. Vehicle identity, latencies, `HIGH_LATENCY2`, firmware version, command attempts, and message counts remain populated whenever observed.

For the first hardware result, the intended truthful representation is:

```text
transport_pass = true
telemetry_pass = true
command_delivery_pass = true
response_return_pass = true
command_ack_pass = false
strict_pass = false
```

The process still exits nonzero when the strict checks fail, but `result.json` shows exactly what passed.

## Firmware ACK trace

The node logs emit markers only when message ID 77 is involved:

```text
MAVLink COMMAND_ACK local ingress
MAVLink COMMAND_ACK transport queued
MAVLink COMMAND_ACK mesh transmission committed
MAVLink COMMAND_ACK mesh reassembled
MAVLink COMMAND_ACK local endpoint delivered
```

The air-node sequence localizes PX4-to-mesh behavior. The ground-node sequence localizes mesh-to-UDP behavior. Drop markers identify queue or endpoint loss.

No broad packet logging was added.

## Output

Each run creates:

```text
mavlink_tests/reports/px4-sitl-mesh-YYYYMMDD-HHMMSS/
    px4.log
    result.json
```

Exit status:

```text
0   strict pass
2   preflight or local configuration failure
3   partial result or strict test failure
4   unexpected harness defect
130 interrupted
```

For the next run, retain:

- `result.json`;
- the ACK-related lines from the air-node log;
- the ACK-related lines from the ground-node log.

The two ACK milestone values and the first missing firmware marker identify the next boundary without another broad investigation.
