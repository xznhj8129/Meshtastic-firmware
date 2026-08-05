# Capture 20260804-235621 — root cause found: PX4 MAVLink mode

Firmware `2.8.0.32d28ab` + uncommitted transport aggregation on both nodes. PX4
`v1.18.0-beta1-216`. Identical capture schedule to `20260804-231359`: 5 s warmup, 10 cycles at
3 s, control command 1 s after each request, 8 s drain.

**Single change from the previous capture: `mavlink start … -m custom` became `-m iridium`.**

## Result

```text
vehicle_command_ack: 17 records -> {211: 2, 511: 8, 512: 7}
```

**15 of the 20 transmitted commands reached PX4 and were acknowledged.** Every prior capture
delivered 0–2.

| Measure | `-m custom` (231359) | `-m custom` (234527) | `-m iridium` (235621) |
| --- | ---: | ---: | ---: |
| mesh frames from air | 96 | 107 | **7** |
| ground `TX queue is full` | 86 | 80 | **0** |
| ground txGood delta | +7 | +3 | **+31** |
| air LoRa RX from ground | 2 | 0 | **20** |
| commands reaching PX4 | 0 | 0 | **15** |

Every symptom cleared simultaneously. The queue-full condition did not shrink, it disappeared.
All 20 of the ground node's outbound packets arrived at the air node.

## Why

`-m custom` does not mean "only the streams I configure". In `mavlink_main.cpp`:

```c
case MAVLINK_MODE_CUSTOM:
    //stream nothing
    break;
...
/* add default streams depending on mode */
if (_mode != MAVLINK_MODE_IRIDIUM) {
    configure_stream("HEARTBEAT", 1.0f);
    configure_stream("STATUSTEXT", ...);
    ...
}
```

Every mode except `IRIDIUM` gets `HEARTBEAT` and `STATUSTEXT` added unconditionally, and
`HEARTBEAT` is commented as a constant-rate stream whose rate is never adjusted — it cannot be
disabled with `-r 0`. Together with `MISSION_CURRENT` and the mode messages, which come from
their own components rather than the stream table, PX4 offered roughly 2.25 frames/s to a link
that could not carry it.

`IRIDIUM` is the only mode that carries `HIGH_LATENCY2` alone, and it is the mode intended for
slow, packet-metered links. A LoRa mesh is one.

## What this supersedes

The channel was never saturated by anything the bridge did. Every prior conclusion about mesh
congestion — ground TX queue starvation, relay overhead consuming the ground node's airtime,
arbitration asymmetry — was a true observation of a link being crushed by roughly 14x the
traffic it should have carried. They were symptoms, not causes.

The relay-tax observation (ground node spending 6 of 7 transmissions relaying broadcast
telemetry, `hop_limit` 3 in a two-node network) remains real and worth addressing, but it was
only fatal because of the offered load.

## Still open

- `HIGH_LATENCY2` arrived 6 times in ~43 s against 0.5 Hz configured (~21 expected). IRIDIUM
  gates transmission on GCS connection state and stops when it believes a GCS is connected,
  unless commanded on with `MAV_CMD_CONTROL_HIGH_LATENCY`. Telemetry is now sparser than
  intended. A quiet capture in this mode is not automatically a transport failure.
- **No `COMMAND_ACK` reached the verifier** despite PX4 creating 15. This is the original
  question, and it is now askable for the first time: commands arrive, acks are created, and the
  link is no longer saturated. Boundary C is finally testable rather than confounded.

## Frame aggregation, measured separately

Capture `20260804-234527` carried the transport aggregation change under `-m custom`:

```text
mesh-borne frames 107 / air LoRa packets 102 = 1.05 frames per packet
```

Aggregation is implemented and correct but effectively never engages, because frames do not
accumulate in the outbound queue — the bridge peeks the moment a frame is enqueued, so the pack
loop almost always finds exactly one frame waiting. Making it effective requires a coalescing
hold before transmission, which is a scheduling change and a deliberate latency-versus-airtime
decision. It was not made.
