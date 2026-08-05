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

> Resolved by the second round. The boundary is inside PX4, not the mesh.
> See "Round 2: COMMAND_ACK localized to PX4" below. The text in this section
> is retained as the state of knowledge after the first run.

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

## Embedded ACK trace

The firmware now adds narrow, event-only tracing for `COMMAND_ACK` without changing routing, queue sizes, scheduling, flow control, build definitions, or dependencies.

The expected trace boundaries are:

```text
MAVLink COMMAND_ACK local ingress
MAVLink COMMAND_ACK transport queued
MAVLink COMMAND_ACK mesh transmission committed
MAVLink COMMAND_ACK mesh reassembled
MAVLink COMMAND_ACK local endpoint delivered
```

Drop-only markers identify outbound queue loss, mesh-send drop, inbound queue loss, or local endpoint stall.

Transport counters now separately record:

- ACK frames queued from a local endpoint;
- ACK frames fully committed to Meshtastic;
- ACK outbound drops;
- ACK frames reassembled from Meshtastic;
- ACK inbound drops;
- ACK frames delivered to the local UART/UDP endpoint.

This trace requires a normal incremental firmware rebuild and reflash of the two nodes. It does **not** require or justify:

- `pio run -t clean`;
- deleting `.pio/build`;
- touching PlatformIO configuration or variants;
- changing dependencies or submodules;
- running native/coverage builds;
- rebuilding PX4 separately.

## Interpretation of the next run

```text
request-message ACK absent
control-command ACK present
    -> command-specific PX4 behavior

both ACKs absent
no air-node local-ingress marker
    -> ACK never reached the air-node UART

local ingress present
queue/send marker missing
    -> air-node transport boundary

mesh send present
ground reassembly missing
    -> Meshtastic transport boundary

ground reassembly present
local delivery missing
    -> ground endpoint-delivery boundary

local delivery present
verifier sees no ACK
    -> UDP/verifier parsing boundary
```

Only the first missing marker should drive the next correction.

# Round 2: COMMAND_ACK localized to PX4

Second round, 2026-08-04, firmware `2.8.0.6d533cc` on both nodes, PX4 `v1.18.0-beta1-216`.

## Conclusion

`COMMAND_ACK` is created by PX4 and never leaves PX4. It does not enter the Meshtastic
bridge, so no firmware change is warranted and no msgid-77 trace marker can ever fire.

| Boundary | Result |
| --- | --- |
| A. command reaches PX4 | **PASS** |
| B. PX4 publishes `vehicle_command_ack` | **PASS** |
| C. serial instance serializes msgid 77 | **FAIL** |
| D–F. air ingress, mesh, ground, verifier | moot; frame never reaches the bridge |

## Evidence

PX4 writes a ulog per run. `reports/px4-sitl-mesh-20260804-220351/px4-ulog-command-ack.txt`
holds the extract. For `MAV_CMD_REQUEST_MESSAGE` (512) PX4 published, in both firmware
versions, twice per run:

```text
command=512  result=0 (ACCEPTED)  target_system=255  target_component=190  from_external=0
```

The ACK exists, is accepted, and is addressed to the verifier. It never arrived.

Every gate in `Mavlink::handleAndGetCurrentCommandAck()` that this record can satisfy does:
`from_external` is 0, 512 is below `VEHICLE_CMD_PX4_INTERNAL_START`, and target component 190
is below `COMPONENT_MODE_EXECUTOR_START` (1000).

The one `COMMAND_ACK` that did reach the verifier can only be the ack for command 211, which
differs in exactly one respect:

| | cmd 211 (arrived) | cmd 512 (never arrived) |
| --- | --- | --- |
| `target_system` | 0 (broadcast) | 255 |
| `target_component` | 1 | 190 |

That points at `is_target_known = _receiver.component_was_seen(target_system, target_component)`
in `mavlink_main.cpp`. A broadcast target passes trivially; the specific 255/190 target
evidently does not, so the ACK is skipped before serialization.

Confidence: the `component_was_seen` attribution is inference from the 211-vs-512 contrast,
not direct instrumentation. Confirming it needs a temporary probe at that branch. The
boundary itself (created by PX4, never transmitted) is directly evidenced.

The uORB records are byte-identical between `69c06a6` and `6d533cc`, so this is unrelated to
the ACK-tracing commit and predates it.

## Corrected earlier claims

1. `RADIO_STATUS.txbuf` starvation: rejected previously, and round 2 supersedes it with a
   cause that involves no flow control at all.
2. A suspected ground-to-air regression in `6d533cc`, called from response counts alone, was
   **wrong**. The ulog shows PX4 received and acked our requests in that run. Message counts
   are not sufficient evidence of delivery; the uORB record is.

## Harness defects found and fixed in round 2

Both had to be fixed before the ACK could be tested at all. Runs `215653` and `220138`
transmitted zero commands and their ACK failures were therefore meaningless.

| # | Defect | Where |
| --- | --- | --- |
| 1 | The `HIGH_LATENCY2` snapshot was latched from the first sample, which arrives ~2 s in, before the SIH estimator has a global position. That pinned latitude/longitude at 0/0, so `telemetry_pass` was false for the whole run and the command phase — gated on it — never executed. 36 later valid samples were discarded. | snapshot handler |
| 2 | Any unprompted `AUTOPILOT_VERSION` satisfied `response_return_pass`. PX4 emits it on its own, so the request was pre-empted and never sent (`request_attempts: 0`). | `AUTOPILOT_VERSION` handler |

## Still unexplained

Run `220351` received 0 `AUTOPILOT_VERSION` in reply to 3 requests, where the earlier
`69c06a6` run received 3, even though PX4 acked 512 twice in both. The return path for the
requested message is a separate open question from the ACK boundary.

## Takeaways for the next round

1. **Stop using multi-minute windows.** Observed latencies are seconds: heartbeat 3.3 s,
   `HIGH_LATENCY2` 1.5 s, and PX4 publishes its ACK in the same window. A run that waits 120 s
   to conclude a message did not arrive wastes minutes per iteration and hid these defects
   behind slow feedback. Size the wait from measured latency plus margin.
2. **Read the ulog first.** It answers "did PX4 receive it" and "did PX4 ack it" directly.
   Both wrong conclusions in this project came from inferring cause from verifier-side message
   counts instead.
3. Milestone flags must not gate transmission on an unrelated assertion. Defect 1 above turned
   a position-quality check into a hard block on the command phase.

## Attribution: ours versus PX4

Almost all the cost so far is ours. Effectively every failed run was a harness defect, not a
hardware or protocol problem: five separate defects, three of which made the harness incapable
of completing a run on any machine, and two of which silently transmitted zero commands while
reporting ACK failures. Two confident wrong diagnoses (`RADIO_STATUS.txbuf` starvation, then a
ground-to-air regression) were also ours, both from reading verifier-side message counts
instead of the ulog.

The `COMMAND_ACK` behaviour itself does look PX4-side: PX4 creates an ACCEPTED ack addressed to
a component it had just received a command from, and does not transmit it.

That story is not airtight, and one observation undercuts it. In run `220351` the requested
`AUTOPILOT_VERSION` also failed to return, three times, while `HEARTBEAT` and `HIGH_LATENCY2`
kept flowing. A pure ack-target gate in PX4 does not explain that, because `AUTOPILOT_VERSION`
does not go through the ack path. What both messages share is that they are one-shot direct
sends rather than configured streams.

Two competing explanations remain, and they assign blame differently:

| Explanation | Predicts | Fits? |
| --- | --- | --- |
| PX4 suppresses the ack on a target check | ack never returns, `AUTOPILOT_VERSION` always returns | fails run `220351` |
| Direct (non-stream) sends are lost on this link | both intermittent | fits run 1 and run `220351` |

The second covers more of the data and would point at the link or the bridge, not PX4. Do not
close this as a PX4 bug yet.

Cheap decisive test, seconds not minutes: issue the request repeatedly in quick succession and
measure how reliably `AUTOPILOT_VERSION` returns. Reliable return with never an ack implicates
PX4. Both flaky implicates the return path, and that is ours.
