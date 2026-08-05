# Capture 20260804-231359 — direct interpretation

Firmware `2.8.0.6d533cc` on both nodes (no reflash; commits after `6d533cc` touch only
harness and docs). Repo head at capture time `670d2f72`. PX4 `v1.18.0-beta1-216`.
Air node `!b6c5b4c5`, ground node `!3a180e17` at 192.168.0.244.

Capture window ~43 s: 5 s warmup, 10 cycles at 3 s, control command 1 s after each request,
8 s drain. Every command transmitted on a fixed clock, gated on nothing.

## Ground-to-air chain, as measured

| Transition | Evidence | Count |
| --- | --- | ---: |
| recorder UDP transmit | `capture.json` `sent_message_counts` | 20 COMMAND_LONG + 9 HEARTBEAT |
| ground UDP ingress | `ground-node.log` `MAVLink UDP client registered`, then outbound activity | reached |
| ground mesh enqueue | `enqueue for send (… fr=0x3a180e17 …)` | **10** |
| **ground radio TX queue** | **`TX queue is full, and there is no lower-priority packet available to evict`** | **86 rejections** |
| ground LoRa transmit | `txGood` 129 → 136 | **7** |
| air LoRa receive from ground | `Lora RX (… fr=0x3a180e17 …)` | **2** |
| PX4 command reception | `px4-ulog-extract.txt` | **0** |

**First missing transition: the ground node's outbound radio TX queue.** It is full and
rejecting packets 86 times in a 43 s window. Only 7 packets were transmitted at all.

## Why the queue is full

The ground node received **100** LoRa packets from the air node in the same window, against 7
transmitted. The air-to-ground telemetry stream is consuming the channel, and the ground node
cannot obtain airtime for outbound commands. Four `Can not send yet, busyRx` deferrals appear
alongside.

Received message mix at the recorder, 156 messages in ~43 s:

| Message | Count | Share |
| --- | ---: | ---: |
| `RADIO_STATUS` | 60 | 38% |
| `HEARTBEAT` | 32 | 21% |
| `MISSION_CURRENT` | 29 | 19% |
| `HIGH_LATENCY2` | 13 | 8% |
| `UNKNOWN_410`/`411` | 17 | 11% |
| `COMMAND_LONG` | 3 | 2% |
| `AUTOPILOT_VERSION` | 1 | <1% |
| `COMMAND_ACK` | 1 | <1% |

`RADIO_STATUS`, generated locally by the bridge every 500 ms, is the single largest contributor
to the traffic occupying the link. Stated as an observation only. No flow-control change is
proposed or made here.

## Air-to-ground one-shot loss, also measured

The COMMAND_ACK trace markers were exercised by PX4's own command 211 acks:

```text
air   MAVLink COMMAND_ACK local ingress             2
air   MAVLink COMMAND_ACK transport queued          2
air   MAVLink COMMAND_ACK mesh transmission committed 2
ground MAVLink COMMAND_ACK mesh reassembled          1
ground MAVLink COMMAND_ACK local endpoint delivered  1
recorder COMMAND_ACK received                        1
```

Two frames left the air node, one arrived. The loss is between air mesh transmission and
ground reassembly. The bridge and transport did their part on both ends; the frame was lost on
the radio.

## What this supersedes

The earlier claim that the boundary was inside PX4 is now definitively wrong for this
condition: the commands never reach PX4 at all, so PX4's ack behaviour was never the active
defect. Earlier runs where 2 of 3 commands arrived were runs where the queue happened to have
room.

`ToPhone queue is full, drop packet` appears 100 times and is **not** relevant — that is the
phone/API queue discarding inbound packets with no phone client attached. It was briefly
mistaken for the radio queue during analysis.

## Observability gaps

- No firmware tracing exists for inbound COMMAND_LONG on the ground node, so "ground UDP
  ingress" and "ground MAVLink frame parse" are inferred from surrounding activity rather than
  observed per frame. 20 frames in produced 10 mesh enqueues; whether the other 10 were merged,
  dropped at the bridge outbound queue, or never parsed is not directly visible.
- The 2 packets that did reach the air node were not correlated to specific commands.

## Next

The channel is saturated. Anything measured about command delivery, ack behaviour, or PX4
serialization while the ground node cannot transmit is measuring congestion, not protocol.
Reduce offered load or measure with the air node quiet before drawing further protocol
conclusions.
