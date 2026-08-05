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

The claim that `RADIO_STATUS` was 38% of mesh traffic and the largest contributor to channel
congestion was wrong. `RADIO_STATUS` is generated locally by the bridge and written to the
local endpoint; it never crosses LoRa. The 60 frames counted came from recorder-side totals,
which include locally injected frames. Measured mesh load is 96 frames against 97 LoRa packets,
one frame per packet, none of them `RADIO_STATUS`.

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

# Root cause: PX4 MAVLink mode, not the mesh

Capture `20260804-235621`. One change from the previous capture: `mavlink start … -m custom`
became `-m iridium`.

```text
vehicle_command_ack: 17 records -> {211: 2, 511: 8, 512: 7}
```

15 of 20 transmitted commands reached PX4 and were acknowledged. Every prior capture delivered
0 to 2.

| Measure | `-m custom` | `-m iridium` |
| --- | ---: | ---: |
| mesh frames from air | 96–107 | 7 |
| ground `TX queue is full` | 80–86 | 0 |
| ground txGood delta | +3 to +7 | +31 |
| air LoRa RX from ground | 0–2 | 20 |
| commands reaching PX4 | 0 | 15 |

`-m custom` does not mean "only the streams I configure". PX4 adds `HEARTBEAT` and `STATUSTEXT`
unconditionally for every mode except `IRIDIUM`, and `HEARTBEAT` is a constant-rate stream that
cannot be disabled with `-r 0`. With `MISSION_CURRENT` and the mode messages arriving from their
own components, PX4 offered roughly 2.25 frames/s to a link that could not carry it. `IRIDIUM`
is the only mode that carries `HIGH_LATENCY2` alone and is the mode intended for slow,
packet-metered links.

## Consequence for earlier conclusions

Every congestion finding — ground TX-queue starvation, relay overhead, arbitration asymmetry —
was a true observation of a link carrying roughly 14x the traffic it should have. All symptoms,
not causes. The relay tax (`hop_limit` 3 broadcast in a two-node network, ground node spending
6 of 7 transmissions relaying) is real and still worth fixing, but was only fatal under the
excess load.

## Still open

- `HIGH_LATENCY2` arrived 6 times in ~43 s against 0.5 Hz configured. IRIDIUM gates transmission
  on GCS connection state and goes quiet when it believes a GCS is connected, unless commanded
  on with `MAV_CMD_CONTROL_HIGH_LATENCY`. A quiet capture in this mode is not automatically a
  transport failure.
- No `COMMAND_ACK` reached the verifier despite PX4 creating 15. The original question is now
  testable for the first time without congestion confounding it.

## Frame aggregation: implemented, does not engage

Transport now supports an aggregate container (`VERSION_AGGREGATE = 2`) packing whole frames up
to the payload limit, bounded by the queue depths. Measured under `-m custom`: 107 mesh frames
in 102 packets, **1.05 frames per packet**. It never engages, because frames do not accumulate —
the bridge peeks the moment one is enqueued. Making it effective needs a coalescing hold before
transmit, which is a scheduling change and a latency-versus-airtime decision that was not made.
Oversized frames still use the original single-frame fragment format; the receiver handles both.

# COMMAND_ACK resolved: PX4 suppresses its own ACKs in IRIDIUM mode

Capture `20260805-004817`, both nodes on `128ff0a2d`, PX4 `-m iridium`.

## Evidence

PX4 created 19 acks. **Zero `MAVLink COMMAND_ACK` markers appeared on either node**, so message
id 77 never reached the air node's UART. The frame was never serialized.

`mavlink_main.cpp`, in the ack send path:

```c
if (_mode == MAVLINK_MODE_IRIDIUM) {
    if (command_ack.from_external) {
        // for MAVLINK_MODE_IRIDIUM send only if external
        mavlink_msg_command_ack_send_struct(get_channel(), &msg);
    }
} else {
    mavlink_msg_command_ack_send_struct(get_channel(), &msg);
}
```

Every ack in the uLog carries `from_external=0`:

```text
command=512 from_external=0 target_system=255 target_component=190 result=0
command=511 from_external=0 target_system=255 target_component=190 result=4
(19 records, all from_external=0)
```

PX4 generates these acks itself, so `from_external` is never set, so in IRIDIUM mode every one
is discarded before serialization. This is deliberate: do not spend bytes on acks over a
pay-per-byte satellite link.

The earlier `get_free_tx_buf()` and `component_was_seen()` theories are both dead.
`get_free_tx_buf()` returns the constant `MAVLINK_MAX_PACKET_LEN` on POSIX, so it never gates.

## The bind

| mode | offered load | emits own COMMAND_ACK |
| --- | --- | --- |
| `-m custom` | floods the link; 0 of 20 commands delivered | yes |
| `-m iridium` | usable; 19 of 20 delivered | never |

There is no PX4 mode that both keeps the offered load survivable and returns its own
`COMMAND_ACK`. The two requirements are mutually exclusive in the tested PX4.

## Consequence for the test

`COMMAND_ACK` is not a valid delivery criterion under `-m iridium`. Use instead:

- the PX4 uLog `vehicle_command_ack` records, which prove reception and the result code; and
- the requested-message response, which does return over the mesh.

`MAV_CMD_REQUEST_MESSAGE` is acknowledged `result=0` (ACCEPTED). `MAV_CMD_SET_MESSAGE_INTERVAL`
is acknowledged `result=4` (FAILED), because `HEARTBEAT` is not a configured stream in IRIDIUM
mode, so that control command is not a useful probe in this configuration.

Nothing in the Meshtastic firmware is implicated. No transport change is warranted by this.

# IRIDIUM-compliant GCS: telemetry restored

Capture `20260805-010157`. Harness change only, no reflash.

## The demand

```c
if (_transmitting_enabled && (!vehicle_status.gcs_connection_lost || ...)) {
    _transmitting_enabled = false;
}
```

PX4 disables IRIDIUM transmission whenever it believes a GCS is connected, and that branch
ignores `_transmitting_enabled_commanded`. Sending GCS heartbeats over the only available link
therefore silences the vehicle. A satellite GCS does not heartbeat; PX4 treats "GCS lost" as the
signal that the high-latency link is all that remains.

## The change

The recorder no longer sends `HEARTBEAT`. It asserts the link with
`MAV_CMD_CONTROL_HIGH_LATENCY(enable=1)` on the same 5 s clock instead, which also keeps the
ground node's UDP client registration alive.

## Result

| Measure | heartbeating GCS (004817) | compliant GCS (010157) |
| --- | ---: | ---: |
| `HIGH_LATENCY2` received | 2 | **15** |
| `AUTOPILOT_VERSION` received | 1 | **3** |
| GCS heartbeats sent | 9 | 0 |
| commands acked by PX4 | 19 / 20 | **28 / 29** |
| `COMMAND_ACK` on the wire | 0 | 0 |

`HIGH_LATENCY2` at 15 in ~43 s is roughly 0.35/s against 0.5 Hz configured. PX4 now transmits
steadily instead of oscillating between enabled and disabled. Our `MAV_CMD_CONTROL_HIGH_LATENCY`
probes are themselves acknowledged (`2600` appears 12 times in the uLog), confirming delivery.

## Unchanged, and unchangeable from the GCS

`COMMAND_ACK` is still absent, and every uLog record still carries `from_external=0`. No GCS
behaviour affects that branch. The suppression is structural, as recorded above.

# Open design question: enforcing the link profile

The link's constraints are known to the Meshtastic node, but the autopilot's MAVLink mode is
configured out of band on the PX4 side. Nothing currently enforces it. Measured consequence of
getting it wrong: 0 of 20 commands delivered.

## Sketch

A serial-module sub-option, set through the normal config path so the CLI exposes it. Enum, not
a bool, because non-high-latency operation is expected to be worked out later:

```text
serial.mavlink_link_profile
  UNSET          (0)  current behaviour, bridge asserts nothing
  HIGH_LATENCY   (1)  bridge asserts high-latency operation to the local autopilot
  ...                 room for a normal/constrained profile once one is characterised
```

Under `HIGH_LATENCY` the bridge would periodically emit
`MAV_CMD_CONTROL_HIGH_LATENCY(enable=1)` to the locally attached autopilot, using the sysid and
compid already learned from its heartbeat, **written to the local UART only and never
encapsulated for the mesh**.

This costs one small frame every few seconds on a wire that is not the constrained link.

## The part that does not work, stated plainly

Asserting the command is not sufficient to enforce the profile. PX4 disables IRIDIUM
transmission whenever `gcs_connection_lost` is false, and that branch ignores the commanded
flag. So a third-party GCS that heartbeats normally will silence the vehicle no matter what the
bridge asserts. Fully enforcing it would require the bridge to withhold `HEARTBEAT` frames from
the local autopilot.

That is message filtering, and it contradicts a stated principle of this bridge: it transports
MAVLink, it does not maintain a whitelist or decide which services are permitted. Adding
direction- and type-specific filtering to win a PX4 mode argument is a real architectural
change, not a tweak.

So the honest position: a toggle can *assert* the profile, it cannot *enforce* it without
crossing that line. The choice belongs to whoever owns the architecture.

## Cheaper middle ground

The bridge already counts frames. It could log a rate warning when sustained local ingress
exceeds what the current LoRa preset can carry, naming the offending rate. That is diagnostic
rather than filtering, it would have made the root cause obvious within one run instead of
several days, and it commits to nothing architecturally.
