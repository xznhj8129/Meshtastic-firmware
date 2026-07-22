# MAVLink Serial Bridge — Design & Implementation Plan

Status: **DRAFT / for review** — no code written yet.

## 1. Goal

Add MAVLink awareness to Meshtastic's serial (UART) subsystem so a Meshtastic node
can sit inline on a MAVLink serial link and bridge a **low-rate** subset of MAVLink
traffic across the LoRa mesh. The node auto-detects whether it is attached to a
**vehicle (air side / UAV)** or a **ground control station (GCS / ground side)** from
the MAVLink `sysid`/`compid`/`HEARTBEAT`, and behaves accordingly:

- **Air side**: snoop the autopilot's telemetry for the node's *own* use (position,
  battery), and forward a filtered/rate-limited slice of MAVLink to the mesh.
- **Ground side**: forward GCS→vehicle commands to the mesh, and reconstruct
  vehicle→GCS MAVLink coming off the mesh back onto the local UART.
- Both sides: frame-aware fragmentation/reassembly so MAVLink frames survive the
  233-byte mesh payload, plus flow control borrowed from ELRS-MAVLink / mLRS.

## 2. Reality check — what this is and is NOT

Meshtastic is a high-latency (seconds), very-low-throughput (a few hundred bytes
every few seconds on `LONG_FAST`), duty-cycle-limited **mesh**. It is **not** a
point-to-point telemetry radio. So:

- **NOT** an FPV / real-time attitude / RC-control link. No 4 Hz `ATTITUDE`, no
  `RC_CHANNELS`, no video, no manual flight control.
- **IS** a "track + status + occasional command" link: position reports, arm/mode
  and battery status, `STATUSTEXT`, and low-frequency commands (arm, RTL, set mode,
  goto/waypoint, mission upload with care).

This constraint is the single most important design driver. Every decision below
optimizes for *bytes on air*, not latency. The module must be aggressively
rate-limited and default to a conservative whitelist, or it will saturate the mesh
and starve every other node in range.

The ELRS/mLRS techniques we borrow are the *bandwidth-management* parts of those
projects (framing, filtering, `RADIO_STATUS` flow control), not their real-time
transport — their links are ~10–50 kbit/s point-to-point; ours is orders of
magnitude slower and shared.

## 3. Where it plugs in

The existing `SerialModule` (`src/modules/SerialModule.{h,cpp}`) already owns the
secondary UART, its baud/pin config, and a set of line-oriented "modes"
(`NMEA`, `CALTOPO`, `WS85`, `MS_CONFIG`, …) dispatched in `runOnce()`. Its companion
`SerialModuleRadio` (a `SinglePortModule`) handles the mesh side. This is the
natural home.

**Two integration options:**

| | Option A: new SerialConfig mode | Option B: standalone MavlinkModule |
|---|---|---|
| Config | add `Serial_Mode_MAVLINK` | new `MESHTASTIC_EXCLUDE_MAVLINK` module |
| UART ownership | reuses SerialModule's UART setup | duplicates UART setup |
| Coexists w/ other serial modes | mutually exclusive (fine) | needs arbitration for the port |
| Proto churn | 1 enum value | new module config block |

**Recommendation: Option A** — add `Serial_Mode_MAVLINK` and delegate the heavy
lifting to a new self-contained helper `src/modules/Mavlink/MavlinkBridge.{h,cpp}`
that `SerialModule::runOnce()` pumps, plus a `MavlinkRadio` handler on a new portnum.
This keeps UART setup in one place and matches the existing pattern
(`processWXSerial()` for `WS85`).

## 4. Component breakdown

```
                       UART (autopilot or GCS)
                                │
                    ┌───────────▼────────────┐
                    │  MavlinkFrameParser     │  byte-stream → whole MAVLink frames
                    │  (v1 0xFE / v2 0xFD)    │  (LEN-based, transport framing only)
                    └───────────┬────────────┘
                                │ complete frame + (sysid,compid,msgid)
                    ┌───────────▼────────────┐
                    │  RoleDetector           │  HEARTBEAT → AIR (vehicle) / GROUND (GCS)
                    └───────────┬────────────┘
                                │
              ┌─────────────────┼──────────────────┐
              │                 │                   │
     ┌────────▼──────┐  ┌───────▼────────┐  ┌───────▼────────┐
     │ TelemetrySnoop│  │ ForwardPolicy  │  │ FlowControl    │
     │ (air only)    │  │ filter + rate  │  │ RADIO_STATUS   │
     │ pos/batt→mesh │  │ limit per msgid│  │ inject → UART  │
     │ subsystems    │  └───────┬────────┘  └────────────────┘
     └───────────────┘          │
                        ┌───────▼────────┐
                        │ TunnelFramer   │  frame(s) → mesh payload(s)
                        │ frag/reassembly│  w/ 2-byte tunnel header
                        └───────┬────────┘
                                │ MeshPacket (MAVLINK_APP portnum)
                        ┌───────▼────────┐
                        │  MavlinkRadio   │  send/recv on mesh; bound channel/peer
                        │ (SinglePort)    │
                        └────────────────┘
```

### 4.1 MavlinkFrameParser (transport framing, dependency-free)

- State machine over incoming UART bytes. Detects `0xFD` (v2) / `0xFE` (v1) STX,
  reads `LEN`, computes total frame length:
  - v1: `6 + LEN + 2` (STX,LEN,SEQ,SYS,COMP,MSGID + payload + CRC16).
  - v2: `10 + LEN + 2 + (incompat & 0x01 ? 13 : 0)` (adds signature if signed).
- Emits the **exact original bytes** of each complete frame plus a parsed tuple
  `{version, sysid, compid, msgid, seq, payload_ptr, payload_len}`.
- **CRC policy**: transport framing does *not* require MAVLink `CRC_EXTRA` tables
  (those are per-message, per-dialect). We forward bytes verbatim; the receiving
  autopilot/GCS validates CRC as usual. Optionally validate the CRC-16/MCRF4XX over
  the frame body for *sync recovery only* (reject garbage, resync on next STX). We
  do **not** need the full generated MAVLink C library — keeps flash/RAM small.

### 4.2 RoleDetector (UAV vs GCS)

Decode only `HEARTBEAT` (msgid 0) by fixed offsets — no dialect tables needed:

- `HEARTBEAT.type` at payload offset 4 (`MAV_TYPE`), `.autopilot` at offset 5.
- `type == MAV_TYPE_GCS (6)` → this node is on the **GROUND** side.
- any vehicle `type` with `autopilot != MAV_AUTOPILOT_INVALID (8)`, or
  `compid == MAV_COMP_ID_AUTOPILOT1 (1)` → **AIR** side.
- Learn and cache `{sysid, compid} → role`; the dominant local heartbeat wins.
- Config override: `auto` (default) / `force_air` / `force_ground` for setups where
  the heartbeat is ambiguous (e.g. companion computer, MAVProxy on the ground
  emitting vehicle-typed heartbeats).

### 4.3 TelemetrySnoop (air side only — "uses telemetry for its own uses")

When AIR, decode a small fixed set by offset and feed Meshtastic's own subsystems so
the drone's mesh node is live on the map/telemetry **without a separate GPS**:

| MAVLink msg | id | Feeds |
|---|---|---|
| `GLOBAL_POSITION_INT` | 33 | node position (lat/lon/alt) → `meshtastic_Position` |
| `GPS_RAW_INT` | 24 | fallback position + fix/sats when no global pos |
| `SYS_STATUS` | 1 | battery voltage/current/remaining → `DeviceMetrics` |
| `BATTERY_STATUS` | 147 | richer battery telemetry |
| `HEARTBEAT` | 0 | armed/mode → status string / `StatusMessage` |
| `STATUSTEXT` | 253 | surfaced as mesh text / log |

Injection points already exist: position flows through `PositionModule`
(`sendOurPosition`/`meshtastic_Position`); battery/voltage through the telemetry
path (`DeviceMetrics`). We push a synthetic position/metric update rather than
touching the LoRa position broadcast cadence (that stays governed by
`PositionModule`'s normal throttling). This means the *air node* participates in the
mesh as a normal position/telemetry source — huge win for UAV tracking.

### 4.4 ForwardPolicy (filter + per-msgid rate limit)

The gate that keeps us off the air. Default **whitelist** with per-message minimum
intervals (all configurable later; conservative defaults):

| msg | id | air→mesh | ground→mesh | default min interval |
|---|---|---|---|---|
| `HEARTBEAT` | 0 | ✓ (thinned) | ✓ (thinned) | 5 s |
| `GLOBAL_POSITION_INT` | 33 | ✓ | — | 3–10 s (or rely on 4.3) |
| `SYS_STATUS` | 1 | ✓ | — | 15 s |
| `STATUSTEXT` | 253 | ✓ | — | on-change |
| `COMMAND_LONG`/`COMMAND_INT` | 76/75 | ✓(ack) | ✓ | event |
| `COMMAND_ACK` | 77 | ✓ | ✓ | event |
| `MISSION_*` | 38–47 | ✓ | ✓ | event, guarded |
| `SET_MODE` | 11 | — | ✓ | event |
| everything else | | drop | drop | — |

Use `Throttle::isWithinTimespanMs` per (msgid) for rate limiting (rollover-safe,
per house style). Commands/acks/mission are event-driven (no periodic stream) so
they pass immediately but are still subject to a global airtime budget / token
bucket so a mission upload can't monopolize the mesh.

### 4.5 FlowControl (ELRS/mLRS technique)

Two mechanisms, both aimed at stopping the local endpoint from over-producing:

1. **`RADIO_STATUS` (msgid 109) injection** onto the local UART toward the
   autopilot/GCS. ArduPilot/PX4 and QGC/MAVProxy honor `RADIO_STATUS.txbuf` as a
   flow-control signal and back off their stream rates. We synthesize `RADIO_STATUS`
   with `rssi`/`remrssi` from mesh SNR/RSSI and a `txbuf` derived from our tunnel
   queue depth — low `txbuf` ⇒ sender slows down. This is exactly how SiK radios,
   ELRS-MAVLink and mLRS throttle the FC/GCS.
2. **Proactive stream-rate control**: on AIR side, send
   `MAV_CMD_SET_MESSAGE_INTERVAL` (or legacy `REQUEST_DATA_STREAM`) to the autopilot
   to pin the outgoing message rates to something the mesh can actually carry,
   instead of relying on back-pressure alone.

### 4.6 TunnelFramer (chunking / fragmentation & reassembly)

Mesh payload cap is `meshtastic_Constants_DATA_PAYLOAD_LEN = 233`. MAVLink frames
range from ~8 bytes (`HEARTBEAT`) to ~280 bytes (v2 max payload + signature). So:

- **Aggregation**: pack multiple *whole* small frames into one mesh payload when
  they're ready together (amortize per-packet overhead/airtime).
- **Fragmentation**: a frame larger than the payload budget is split with a compact
  tunnel header so the far side can reassemble exactly.

**Tunnel header (2 bytes, prepended to mesh payload):**

```
byte0: 0b W_SSSSSS   W=1 whole-frames-batch / 0=fragment; S=6-bit stream/session seq
byte1: fragment: FFFF_TTTT  F=frag index, T=frag total   (max 15 frags ≈ 3.5 KB, plenty)
       batch:    NNNN_0000  N=frame count in this batch
```

Reassembly buffer keyed by `(fromNode, seq)`, with a short timeout (drop partials —
this is lossy transport, no ARQ; upper-layer MAVLink retries handle
mission/param). Keep it small: one or two in-flight reassembly slots per peer.

### 4.7 MavlinkRadio (mesh send/receive)

- New `SinglePortModule` on **`MAVLINK_APP`** portnum.
- **Addressing**: bind to `Channels::serialChannel` (like `SerialModuleRadio`) and/or
  a configurable **peer node id** so a GCS node DMs its paired UAV node and vice
  versa. DM is preferred (keeps traffic off broadcast, enables PKI); broadcast on a
  dedicated channel is the fallback for one-to-many ("show all my drones on this
  GCS").
- On receive: strip tunnel header, reassemble, write reconstructed MAVLink bytes to
  the local UART verbatim. Ground side hands vehicle telemetry to the GCS; air side
  hands commands to the autopilot.

## 5. Data-flow summary

**Air node** (attached to UAV autopilot):
1. UART in ← autopilot stream. Parse frames.
2. Snoop position/battery → feed local Meshtastic position/telemetry (§4.3).
3. Filter+rate-limit (§4.4) → tunnel-frame (§4.6) → `MAVLINK_APP` DM to paired GCS
   node.
4. Inject `RADIO_STATUS` + set message intervals toward autopilot (§4.5).
5. Mesh in (commands from GCS) → reassemble → write to autopilot UART.

**Ground node** (attached to GCS):
1. UART in ← GCS. Parse frames. Filter commands/mission (§4.4).
2. Tunnel-frame → `MAVLINK_APP` DM to paired UAV node.
3. Mesh in (vehicle telemetry) → reassemble → write to GCS UART; GCS sees a normal
   (slow) MAVLink vehicle.
4. Inject `RADIO_STATUS` toward GCS so it throttles command/param bursts.

## 6. Protobuf / wire-format changes (upstream dependency)

Per repo rules, generated protobuf under `src/mesh/generated/` is **never**
hand-edited; wire changes go to the `meshtastic/protobufs` repo first, then the sync
action regenerates here. Needed there:

1. `portnums.proto`: add `MAVLINK_APP` (pick an unused number in the app range).
2. `module_config.proto`: add `Serial_Mode_MAVLINK = 11` to `SerialConfig.Serial_Mode`.
3. (later) optional MAVLink config fields — role override, peer node id, forward
   profile, `inject_radio_status` — either as new `SerialConfig` fields or a small
   dedicated config block.

**Prototyping without proto changes:** use `PRIVATE_APP (256)` as the portnum and a
spare/unused serial mode value gated behind a build flag, so we can build and
hardware-test the whole pipeline before the upstream proto PR lands. Swap to the
real enum values once merged.

## 7. Files to add / touch

```
protobufs/ (separate repo PR)      portnums.proto, module_config.proto
src/modules/Mavlink/MavlinkBridge.{h,cpp}   parser, role, snoop, policy, flow, framer
src/modules/Mavlink/MavlinkRadio.{h,cpp}    SinglePortModule on MAVLINK_APP
src/modules/Mavlink/MavlinkMsgDefs.h        msgid + field-offset constants we decode
src/modules/SerialModule.cpp                 dispatch Serial_Mode_MAVLINK → bridge pump
src/modules/Modules.cpp                       register (guard MESHTASTIC_EXCLUDE_MAVLINK)
test/test_mavlink/test_main.cpp               framer + frag/reassembly + role unit tests
```

Build size: parser + a handful of hand-decoded messages ≈ small; keep behind
`MESHTASTIC_EXCLUDE_MAVLINK` (default-excluded on RAM-constrained parts like STM32WL
/ nRF52832, matching existing module gating).

## 8. Testing

- **Native unit tests** (`./bin/run-tests.sh`, Unity, `test/test_mavlink/`):
  - Frame parser: v1/v2, signed frames, split-across-reads, garbage resync.
  - Fragment → reassemble round-trip, out-of-order/dropped-fragment handling.
  - Role detection from canned `HEARTBEAT` frames (GCS vs several vehicle types).
  - Forward policy rate-limiting.
- **Hardware-in-the-loop** via `meshtastic/meshtastic-mcp` (two nodes): feed a
  recorded MAVLink stream (mavproxy/SITL) into the air node's UART, confirm the GCS
  node's UART reproduces a byte-faithful (if thinned) MAVLink stream that QGC/MAVProxy
  accepts; verify air node's position appears on the mesh.

## 9. Open questions / decisions for review

1. **Portnum vs PRIVATE_APP** for first cut — recommend PRIVATE_APP behind a flag
   until the upstream proto PR merges.
2. **Pairing model**: configured peer node id (DM, recommended) vs dedicated
   broadcast channel (one GCS ↔ many drones). Support both; default to DM.
3. **How aggressive is the default whitelist / rate limit?** Proposed defaults in
   §4.4 — need field validation against real airtime budgets on `LONG_FAST`.
4. **Signed MAVLink**: forward signature bytes verbatim (we do), but we can't
   re-sign injected `RADIO_STATUS`; acceptable since `RADIO_STATUS` is local-link and
   most setups don't require signing on the local UART.
5. **Full MAVLink lib vs hand-rolled decoders**: recommend hand-rolled for the ~6
   snooped messages to avoid the multi-hundred-KB generated dialect headers on
   embedded targets.
```
