# PX4 SITL Meshtastic Hardware Test

This is the first fully automated physical test of the MAVLink mesh router.

## Topology

```text
PX4 SIH on the Linux host
        |
USB-UART adapter at 115200 baud
        |
air Meshtastic node UART
        |
Meshtastic LoRa mesh
        |
ground Meshtastic node, UDP 14550
        |
automated pymavlink verifier on UDP 14600
```

The test deliberately does not use QGroundControl. PX4's ordinary localhost GCS MAVLink instance is stopped before verification, so a passing result cannot use the direct UDP path.

## Hardware contract

### Air node

- current `mavlink` firmware branch;
- Serial module enabled in MAVLink mode;
- RX/TX pins configured;
- baud set to `115200` unless the test environment says otherwise;
- private secondary channel named exactly `serial`.

Wire a 3.3 V USB-UART adapter:

```text
adapter TX -> air node RX
adapter RX -> air node TX
adapter GND -> air node GND
```

Use the stable `/dev/serial/by-id/...` path for the adapter. Do not use 5 V UART logic.

### Ground node

- current `mavlink` firmware branch;
- Serial module enabled in MAVLink mode;
- RX and TX left unset so it operates as a UDP-only endpoint;
- Wi-Fi connected and reachable from the test host;
- same private secondary channel named exactly `serial`.

### Host

- PX4-Autopilot checkout with the normal development dependencies;
- Python 3 with `venv` support;
- QGroundControl closed;
- permission to open the USB-UART device.

## One-time local configuration

```bash
cp mavlink_tests/px4_sitl_mesh_test.env.example \
   mavlink_tests/px4_sitl_mesh_test.env
```

Edit only the required machine-specific values:

```bash
PX4_DIR="/path/to/PX4-Autopilot"
AIR_UART="/dev/serial/by-id/usb-your-adapter"
GROUND_HOST="192.168.0.232"
```

The configuration file, virtual environment, logs, and reports are ignored by Git.

## Run

From the firmware repository root:

```bash
bash mavlink_tests/run_px4_sitl_mesh_test.sh
```

The runner creates its own Python virtual environment and installs only `pexpect` and `pymavlink`.

## Automated procedure

The harness performs these steps without interaction:

1. Verifies the PX4 checkout, serial-device permissions, ground-node address, and UDP ports.
2. Refuses to run while anything is listening on UDP 14550, preventing a QGC bypass.
3. Builds and starts `make px4_sitl_sih sihsim_quadx`.
4. Waits for the PX4 `pxh>` shell.
5. Stops the ordinary PX4 GCS instance on local UDP 18570.
6. Starts a custom serial MAVLink instance on the USB-UART device with hardware flow control disabled.
7. Enables only:
   - `HEARTBEAT` at 1 Hz;
   - `HIGH_LATENCY2` at 0.5 Hz.
8. Sends a GCS heartbeat to the ground node every five seconds.
9. Waits for the PX4 autopilot heartbeat from the expected `sysid`.
10. Requires a valid `HIGH_LATENCY2` message with nonzero SIH position and known battery percentage.
11. Sends targeted `MAV_CMD_REQUEST_MESSAGE(AUTOPILOT_VERSION)` through the mesh.
12. Requires both `COMMAND_ACK` and `AUTOPILOT_VERSION` to return through the mesh.
13. Stops the serial MAVLink instance, shuts down PX4, and writes a JSON result.

## Pass condition

A pass proves, in one run:

- PX4 SITL can use the air node exactly like a physical UART telemetry port;
- complete MAVLink frames cross the UART/LoRa/UDP path in both directions;
- `HEARTBEAT` and `HIGH_LATENCY2` survive the constrained link;
- targeted command traffic reaches PX4;
- PX4 acknowledgement and requested data return;
- no localhost QGC path was available to fake success.

The native firmware tests separately prove that local telemetry snooping is limited to the locally learned airframe `sysid` and ignores other aircraft.

## Output

Each run creates:

```text
mavlink_tests/reports/px4-sitl-mesh-YYYYMMDD-HHMMSS/
    px4.log
    result.json
```

Exit status:

```text
0   complete pass
2   preflight/configuration failure
3   end-to-end test failure
4   unexpected harness defect
130 interrupted
```

On failure, report `result.json` and the end of `px4.log`.
