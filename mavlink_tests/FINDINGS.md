# PX4 SIH Meshtastic MAVLink evidence

First physical work: 2026-08-04.

## Evidence policy

The old pass/fail harness is retired.

Do not use the old `result.json` Boolean fields as evidence. Several runs silently skipped
command transmission because later phases depended on exact earlier observations. Other fields
claimed command delivery or transport failure by copying the state of a different response
check. That cost hours and produced contradictory diagnoses.

The capture tool now does only this:

1. Starts PX4 SIH and the physical serial MAVLink instance.
2. Records full PX4 shell output in `px4.log`.
3. Sends request and control probes on a fixed clock.
4. Records every outgoing UDP datagram as raw hex.
5. Records every incoming UDP datagram as raw hex.
6. Records every MAVLink message pymavlink decodes, including all fields, source IDs, and sequence.
7. Writes counts and configuration to `capture.json` without a protocol verdict.

The authoritative artifacts for a run are:

```text
capture.json
events.jsonl
px4.log
PX4 uLog or a direct extract from it
air-node log
ground-node log
```

Read those artifacts directly. Do not infer a boundary from a derived Boolean.

## Physical topology

```text
PX4 SIH
  -> physical USB-UART
  -> air Heltec Wireless Stick Lite V3 UART
  -> Meshtastic LoRa
  -> ground Heltec Wireless Stick Lite V3 UDP
  -> pymavlink recorder
```

Known configuration:

- PX4 `v1.18.0-beta1-216`;
- two `heltec-wsl-v3` nodes;
- air UART GPIO 47/48 at 57600 baud;
- PX4 host device `/dev/ttyUSB0` during the recorded runs;
- ground node `192.168.0.244`, UDP 14550;
- recorder local UDP 14600.

## Directly observed facts

The original physical run carried repeated `HEARTBEAT` and `HIGH_LATENCY2` from PX4 to the
recorder. It also carried `MAV_CMD_REQUEST_MESSAGE(AUTOPILOT_VERSION)` toward PX4 and returned
`AUTOPILOT_VERSION` messages to the recorder.

The old harness later produced two runs that transmitted zero request commands while reporting
ACK failures:

- one run blocked command transmission on a latitude/longitude quality assertion;
- one run accepted an unsolicited `AUTOPILOT_VERSION` before sending the request.

Those runs are not ACK evidence.

A corrected three-request run produced two accepted PX4 `vehicle_command_ack` records for
command 512, addressed to system 255 component 190. In that same run, the recorder did not see
the corresponding `COMMAND_ACK` or requested `AUTOPILOT_VERSION` messages.

A later ten-request probe reported no command 512 in the PX4 uLog and no requested responses at
the recorder while periodic streams remained visible. That establishes an end-to-end
one-shot command delivery failure during that capture window. It does not, by itself, identify
which hop lost the frames.

## What remains unknown

The first missing boundary for a ground-to-air command can be anywhere in this chain:

```text
pymavlink UDP transmit
-> ground-node UDP receive and MAVLink parse
-> ground outbound transport queue
-> ground Meshtastic send
-> air Meshtastic receive and reassembly
-> air UART write
-> PX4 serial receive
```

For PX4 responses, the possible chain is reversed. A PX4 uORB ACK record proves ACK creation,
not MAVLink serialization or UART transmission.

No router, PX4, retry, scheduling, queue, or flow-control change is justified until the raw
artifacts show the first missing boundary.

## Rejected claims

The claim that bridge-generated `RADIO_STATUS.txbuf` starved PX4 ACK transmission was not
supported. PX4 reported `no radio status` on the physical serial instance.

The claim that the missing ACK was conclusively localized inside PX4 was also too strong. The
uLog proved ACK creation, but the available artifacts did not prove whether PX4 serialized and
wrote message 77 to the physical UART.

The claim that single frames were already proven lost inside the Meshtastic mesh in both
directions was too strong. The data showed end-to-end one-shot failure, not the exact hop.

## Current capture procedure

Run:

```bash
bash mavlink_tests/run_px4_sitl_mesh_test.sh
```

The run sends both commands independently on a fixed schedule:

```text
MAV_CMD_REQUEST_MESSAGE(AUTOPILOT_VERSION)
MAV_CMD_SET_MESSAGE_INTERVAL(HEARTBEAT, 1 Hz)
```

Neither command waits for heartbeat, valid position, a prior response, or an ACK. The capture
completes after the configured drain interval and exits successfully unless the capture
machinery itself failed.

Interpret `events.jsonl` chronologically beside `px4.log`, the PX4 uLog, and both node logs.
