# MAVLink over Meshtastic - Branch Guide

**Branch:** `mavlink` · **Design doc:** [MAVLINK.md](MAVLINK.md) · **Implementation plan:**
[PLAN.md](PLAN.md) · **Session status:** [STATE.md](STATE.md) · **Reviews:** PLAN_REVIEW.md, REVIEW_1.md

## What this is

This branch makes a pair of Meshtastic nodes act as a wireless MAVLink link: it bridges a
MAVLink serial connection across the LoRa mesh, the way a SiK telemetry radio pair would.

- **Air node**: a Meshtastic radio wired by UART to a flight controller (or anything on the
  onboard MAVLink network).
- **Ground node**: a Meshtastic radio feeding a ground control station - over its UART, or over
  WiFi/Ethernet via a built-in UDP server on port 14550.
- **Mesh**: carries the raw MAVLink byte stream between the two, encrypted, inside ordinary
  Meshtastic `SERIAL_APP` packets.

Everything MAVLink 1 and MAVLink 2 can express crosses the bridge: standard, custom-dialect,
and unknown messages, commands and ACKs, mission and parameter traffic, and signed MAVLink 2
frames - all forwarded byte-exact, with no whitelist and no filtering. On top of transport, the
air node passively learns the aircraft's position and battery from the MAVLink stream and
publishes them as its own normal Meshtastic telemetry.

## Why

- LoRa mesh range and relay behavior for free, instead of a dedicated telemetry radio link.
- Meshtastic already provides node addressing, channels, AES encryption, ACK/retransmit, and
  routing - the bridge reuses all of it and adds nothing of its own on top.
- The aircraft shows up on the Meshtastic map: its fused position becomes the air node's
  position broadcast, and the flight battery becomes the air node's battery telemetry.
- The ground node can be completely wire-free: power it, join it to WiFi, point the GCS at it.

This is a **low-throughput** link. Its usefulness depends on configuring sensible MAVLink
message rates at the GCS or autopilot (see Operating notes). It is not for RC manual control,
not for video, and not for anything that needs hard real-time delivery.

## How it works (60-second version)

Following the ExpressLRS MAVLink serial transport model:

1. UART (or UDP) bytes go into a bounded 1024-byte input FIFO.
2. Up to 233 bytes at a time leave as raw chunks in direct `SERIAL_APP` mesh packets to the
   locked peer (broadcast only until a peer is heard). Chunk boundaries ignore frame boundaries.
3. Received chunks go into a 512-byte output FIFO and are fed through the generated MAVLink C
   parser (`mavlink/c_library_v2`, pinned). Only complete frames are written to the local
   endpoint - a lost mesh packet corrupts at most the frames it touched; the parser
   resynchronizes on the next valid frame.
4. Forwarded frames replay their **captured raw wire bytes**, so non-canonical, signed, and
   unknown-dialect frames survive byte-exact (checksums and signatures untouched).
5. The bridge emits a local `RADIO_STATUS` at 2 Hz whose `txbuf` field reports input-FIFO free
   space - the same software flow control SiK radios and ExpressLRS use. The GCS/autopilot sees
   congestion the way it would on any telemetry radio.
6. On the air node, a second decode-only parser pass over the UART stream learns role
   (autopilot heartbeat = AIR, GCS heartbeat = GROUND), then snoops `GLOBAL_POSITION_INT` /
   `GPS_RAW_INT` into the node's local position (delivered through the normal GPS code path, so
   smart-broadcast scheduling applies) and `BATTERY_STATUS` / `SYS_STATUS` into outbound device
   telemetry (never into power-management state).

## Hardware

- Two ESP32 Meshtastic nodes (developed against Heltec Wireless Stick V3 class boards;
  `heltec-v3` / `heltec-wsl-v3` envs). Other arch targets compile with the feature excluded.
- Air node: a UART connection to the flight controller (TX/RX/GND, 3.3 V levels).
- Ground node: nothing but USB power if you use UDP.

## Building

The feature is opt-in. `MESHTASTIC_EXCLUDE_MAVLINK` defaults to 1 (excluded); the stick envs
already enable it (`-DMESHTASTIC_EXCLUDE_MAVLINK=0` in their platformio.ini stanzas). For any
other env, add that flag yourself. The pinned MAVLink headers are fetched automatically as a
PlatformIO lib dep on ESP32 and native envs.

```
pio run -e heltec-v3          # or heltec-wsl-v3, or your env + the flag
pio run -e heltec-v3 -t upload --upload-port /dev/ttyUSB0
```

The UDP server is compiled in automatically on ESP32 targets with networking
(`HAS_NETWORKING && ARCH_ESP32`); there is no separate switch.

## Configuration

Do this once per node. Serial config changes reboot the node.

### 1. Both nodes: mesh prerequisites

- `serial.enabled = true`
- `serial.mode = 11` - there is no upstream enum name yet; nanopb persists the raw value. If
  the Python CLI refuses to set it, write it through the admin/MCP raw config path.
- Create a secondary channel **literally named `serial`** with a shared non-default PSK on both
  nodes. Receive dispatch requires the name match; the default LongFast channel alone will
  silently drop the traffic.
- No peer/pairing settings exist: the first node each side hears a MAVLink chunk from becomes
  its locked peer (logged, reset on reboot). Until then, chunks are broadcast for discovery.

### 2. Air node: UART to the flight controller

- `serial.rxd` / `serial.txd` to free GPIOs wired to the FC telemetry port (cross TX/RX, common
  ground).
- `serial.baud` to match the FC (57600 is the usual MAVLink default).

### 3. Ground node: UDP (recommended) or UART

- **UDP:** leave `serial.rxd` / `serial.txd` unset entirely. Set `network.wifi_enabled = true`
  with `network.wifi_ssid` / `network.wifi_psk` (or `network.eth_enabled = true` on Ethernet
  boards; a WiFi AP hosted by the device also works). When the link comes up the device binds
  UDP 14550 and logs it. Nothing else to configure - there are no UDP settings.
- **UART instead:** set `serial.rxd` / `serial.txd` / `serial.baud` and plug a USB-UART adapter
  into the GCS machine, exactly like the air node.

### 4. GCS

The device is a **wait-for-client** server: it never dials out. The GCS must send the first
datagram so the device learns where to reply:

- **MAVProxy:** `mavproxy.py --master=udp:<ground-node-ip>:14550`
- **QGroundControl:** Comm Links -> add a UDP link with the device IP in the server addresses
  field.
- UART ground nodes: connect to the serial port as usual.

One client at a time; the latest sender takes over; 30 s of silence drops the registration.

### 5. Autopilot message rates (required for a usable link)

A Meshtastic channel carries far less than a 57k serial wire. On ArduPilot, set the telemetry
port rates down before judging the link, e.g. `SRx_EXTRA1 1`, `SRx_EXTRA2 1`, `SRx_EXTRA3 1`,
`SRx_EXT_STAT 1`, `SRx_PARAMS 1`, `SRx_POSITION 2`, `SRx_RAW_SENS 0`, `SRx_RC_CHAN 0` as a
starting point, then watch `RADIO_STATUS.txbuf`: persistent drops below ~80 mean you are
over-driving the link. The bridge never sets rates for you (by design); it reports congestion
via `txbuf`, counts overflow, and drops only when its bounded FIFOs are exhausted.

## What you should see

- GCS receives heartbeats and telemetry; parameter reads and low-rate commands work.
- Device log on boot: `MAVLink serial bridge enabled` (or `... (UDP only)`), then
  `MAVLink bridge locked to peer 0x...` on first traffic, and on network nodes
  `MAVLink UDP server listening on port 14550` plus
  `MAVLink UDP client registered: <ip>:<port>` when the GCS checks in.
- On the air node: `MAVLink autopilot learned: sysid N compid M, role AIR`; the node then
  broadcasts the aircraft position as its own and reports the flight battery as its battery.
- On the ground node: role GROUND learned from the GCS heartbeat (snooping of position/battery
  correctly stays off there).

## Limitations and prototype boundaries

- **Throughput.** This is a telemetry-rate link. Mission downloads and parameter lists work but
  are slow; large log downloads and calibration workflows are impractical.
- **Peer policy is first-sender-wins**, remembered until reboot. One air/ground pair per
  channel; multi-vehicle sysid routing is a documented future phase, not implemented.
- **Best-effort delivery.** Lost mesh packets drop bytes; MAVLink-level protocols (commands,
  missions, params) retry at their own layer, plain telemetry just skips. There is no custom
  reliability layer and none is planned.
- **Security posture.** Chunks ride the named `serial` channel (AES-CTR, not authenticated;
  direct serial-channel packets are exempt from PKI in the current Router). Fine for the bench;
  noted for the upstream design.
- **Not shippable as-is upstream:** serial mode 11 is a raw nanopb value and there is no
  protobuf field for a configured peer node. Both require a `meshtastic/protobufs` PR before
  this can leave the branch.
- Signed MAVLink 2 frames pass through untouched and stay valid; the locally injected
  `RADIO_STATUS` is unsigned - compile with `-DMESHTASTIC_MAVLINK_NO_RADIO_STATUS` on
  signed-only installs.

## Status

Done and committed: transparent transport with byte-exact raw-frame forwarding (incl. signed
and unknown-dialect frames), transactional mesh submission, RADIO_STATUS flow control, position
and battery snooping, UDP server endpoint. Stages 1-3 build-verified on `heltec-v3` with the
feature on and off; the UDP checkpoint compiles-gated but is not build-verified yet.

Pending: build verification of the UDP commit, native unit tests (`test/test_mavlink/`, also
blocked locally by a missing system lib), hardware-in-the-loop validation on two sticks
(one-sided loopback smoke test, then full SITL/GCS), flash/RAM size table, and the upstream
protobufs PR. See STATE.md for the live list.

## Where things live

| Path | What |
| --- | --- |
| `src/modules/Mavlink/MavlinkBridge.{h,cpp}` | FIFOs, raw-frame forwarder, parser, RADIO_STATUS, snoop decoders |
| `src/modules/Mavlink/MavlinkUdpServer.{h,cpp}` | UDP 14550 wait-for-client endpoint |
| `src/modules/SerialModule.{h,cpp}` | mode-11 wiring: UART drain, bridge construction, mesh chunk send, position delivery |
| `src/modules/Telemetry/DeviceTelemetry.cpp` | aircraft battery override of outbound device metrics |
| `src/mesh/MeshService.{h,cpp}` | `sendToMesh` returns `ErrorCode` (transactional chunk commit) |
| `MAVLINK.md` | the design document |
| `STATE.md` | current status and handoff notes |
