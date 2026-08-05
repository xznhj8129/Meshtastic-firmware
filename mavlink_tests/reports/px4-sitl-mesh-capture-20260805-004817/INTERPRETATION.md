# Capture 20260805-004817 — coalescing on hardware

Both nodes flashed to `128ff0a2d` (aggregate container + 250 ms coalescing hold). PX4
`-m iridium`. Same schedule as every capture since `231359`.

## Command delivery

```text
vehicle_command_ack: {211: 2, 511: 10, 512: 9}
```

19 of 20 transmitted commands reached PX4 and were acknowledged.

| Measure | `-m custom` | iridium (235621) | + coalescing (004817) |
| --- | ---: | ---: | ---: |
| commands reaching PX4 | 0 | 15 | **19** |
| ground `TX queue is full` | 80–86 | 0 | 0 |
| air LoRa RX from ground | 0–2 | 20 | 24 |
| frames per ground packet | — | 1.45 | 1.21 |

## Coalescing effect: negligible, as predicted

The recorder sent 29 frames both times. Ground packets rose 20 to 24 because delivery improved,
not because packing worsened. The frames-per-packet figure moves within noise of 1.0. At these
offered rates a 250 ms window rarely sees a second frame, so aggregation still has little to
pack. This confirms the earlier prediction rather than contradicting it: coalescing is the right
mechanism for a loaded link, and this link is no longer loaded.

The gain in this round came from the PX4 mode, not from packing.

## The IRIDIUM gating trap fired

`HIGH_LATENCY2` fell to 2, from 6 in the previous capture. GCS heartbeats now reach PX4, so PX4
concludes a GCS is connected and stops transmitting on the high-latency link. Telemetry went
quiet *because delivery started working*. Read cold this looks like a regression; it is the
documented `MAV_CMD_CONTROL_HIGH_LATENCY` behaviour.

## Open

No `COMMAND_ACK` reached the verifier despite PX4 creating 19. Commands now arrive at 95 percent,
the queue never fills, and the channel is quiet, so nothing is confounding this any longer. It is
the next thing to trace, and the first time it can be traced cleanly.
