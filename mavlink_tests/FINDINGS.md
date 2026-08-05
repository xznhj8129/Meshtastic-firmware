# PX4 SITL mesh test findings

First physical run: 2026-08-04.

Tested:

- Meshtastic firmware `2.8.0.69c06a6` on two `heltec-wsl-v3` nodes;
- PX4 `v1.18.0-beta1-216`;
- air-node UART on GPIO 47/48 at 57600 baud;
- physical path `PX4 -> UART -> air node -> LoRa -> ground node -> UDP verifier`.

## Proven working

The physical router path works in both directions.

Observed in the final run:

| Message | Count |
| --- | ---: |
| `HEARTBEAT` | 148 |
| `HIGH_LATENCY2` | 69 |
| `AUTOPILOT_VERSION` | 3 |
| `COMMAND_LONG` | 3 |
| `RADIO_STATUS` | 376 |
| `MISSION_CURRENT` | 142 |
| `PARAM_VALUE` | 3 |

PX4 bound the real serial device at 57600 baud. The verifier sent three targeted `MAV_CMD_REQUEST_MESSAGE(AUTOPILOT_VERSION)` commands through the mesh and received three corresponding `AUTOPILOT_VERSION` responses.

Therefore:

```text
PX4 UART binding: PASS
air-to-ground telemetry: PASS
ground-to-air command delivery: PASS
PX4 response return path: PASS
complete UART + LoRa + UDP transport: PASS
```

`UNKNOWN_410` and `UNKNOWN_411` are newer PX4 dialect messages not named by the pinned pymavlink dialect. They are not evidence of corruption.

## Open: COMMAND_ACK not observed

No `COMMAND_ACK` was observed for `MAV_CMD_REQUEST_MESSAGE`.

The established sequence is:

```text
COMMAND_LONG reaches PX4
AUTOPILOT_VERSION returns
COMMAND_ACK is not observed
```

The first boundary where message ID 77 disappears is not yet known.

Possible boundaries include:

- PX4 does not publish an ACK for this exact command path;
- the serial MAVLink instance does not select or serialize it;
- the ACK does not reach the air-node UART parser;
- the bridge loses it between local parsing and mesh transmission;
- the ground node loses it before UDP delivery;
- valid UDP bytes are not recognized by the verifier dialect.

## Rejected diagnosis

The earlier claim that bridge `RADIO_STATUS.txbuf` starved the ACK is rejected as unsupported.

The PX4 serial instance reported:

```text
no radio status.
tx rate mult: 0.721
```

The multiplier is therefore not evidence that received bridge `RADIO_STATUS` suppressed the ACK. Do not change bridge flow-control behavior based on that theory.

## Harness defects fixed before the first run

These fixes are valid and retained:

1. Removed redundant `preexec_fn=os.setsid`, which failed after `pty.fork()` had already created the child session.
2. Used `mavlink status` for serial-device verification and `mavlink status streams` for stream verification.
3. Consumed the exact PX4 command echo before matching the real `pxh>` prompt.

The prompt handling is brittle but now hardware-tested. Do not casually refactor it.

## Next-phase harness changes

The next-phase harness now:

- preserves partial observations when a strict assertion fails;
- reports separate transport, telemetry, command, response, and ACK milestones;
- removes QGroundControl wording and the irrelevant host-local UDP 14550 check;
- keeps only the verifier's local UDP 14600 availability check;
- adds a harmless independent ACK control using `MAV_CMD_SET_MESSAGE_INTERVAL` for the already-configured 1 Hz heartbeat;
- shortens the focused run after the command response arrives instead of waiting the full former 240-second timeout;
- continues to use incremental PX4 make without cleaning either PX4 or PlatformIO build state.

No embedded router source was changed in this phase. The already-flashed firmware can be reused for the next run.

## Interpretation of the next run

```text
request-message ACK absent
control-command ACK present
    -> command-specific PX4 behavior

both ACKs absent
    -> shared PX4 serial ACK path or downstream message-ID-77 path

ACK observed by verifier
    -> strict path resolved without router changes
```

Only after this result should temporary embedded ACK tracing be added, and only at the first boundary that still needs localization.
