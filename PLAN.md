# MAVLink Serial Bridge — Staged Implementation Plan (v2)

Derived from [MAVLINK.md](MAVLINK.md), revised per [PLAN_REVIEW.MD](PLAN_REVIEW.MD) (codex gpt-5.6-sol high). Branch: `mavlink`. HIL hardware: 2× Heltec ESP32-S3 sticks — local `/dev/ttyUSB0`, remote on `pc2.lan` (exact variant `heltec-v3` vs `heltec-wsl-v3` confirmed from the devices before flashing).

## Ground rules

- ExpressLRS transport model: UART → input FIFO → raw chunks over `SERIAL_APP` → output FIFO → MAVLink parser → complete frames to UART. No whitelist, no rate control, no custom framing.
- `MavlinkBridge` is standalone (no `SerialModule` dependency) because `SerialModule` is compiled out on `ARCH_PORTDUINO`; native tests drive the bridge directly.
- **Threading invariant:** every bridge/FIFO method is called only from the main cooperative scheduler (OSThread runOnce + synchronous Router module dispatch). Documented in the header; any future ISR/task caller must add locking first.
- Every stage ends with: `trunk fmt`; `./bin/run-tests.sh -f test_mavlink` (full `./bin/run-tests.sh` before final); clean device build with feature on and off; codex review → `REVIEW_<n>.md`; fixes; checkpoint commit.

## Repo-specific decisions (revised)

### D1. Serial mode 11 — prototype cast, upstream PR is the real fix

`static_cast<meshtastic_ModuleConfig_SerialConfig_Serial_Mode>(11)` as `Serial_Mode_MAVLINK` constant; nanopb `UENUM` fields decode/persist out-of-range values, and generated files stay untouched. **Prototype-only boundary is explicit:** shipping requires a meshtastic/protobufs PR adding `MAVLINK = 11` *and* a peer-node destination field (out of scope for this branch; recorded as the upstream prerequisite). A unit test proves numeric mode 11 round-trips through `meshtastic_ModuleConfig_SerialConfig` encode/decode.

### D2. MAVLink headers + parser ownership

- Pinned zip of `mavlink/c_library_v2` added to the **esp32 base and native envs only** (not global `[env]`, so excluded platforms don't fetch it).
- **Use `mavlink_frame_char_buffer()`**, never `mavlink_frame_char()`: parser state lives in bridge members (`mavlink_message_t` + `mavlink_status_t` per direction — one pair for mesh→UART reconstruction, one pair for UART-side snooping in Stage 3). No per-TU static channel storage in the hot path, no `MAVLINK_COMM_NUM_BUFFERS` correctness dependency, independent state per bridge instance (test isolation).
- `MavlinkBridge.h` includes the MAVLink headers (members need the types); only `MavlinkBridge.cpp` and the test TU call MAVLink functions, so unused inline helpers get dropped elsewhere.

### D3. Peer addressing — locked single peer, no open broadcast parsing

Review must-fix #2: interleaved streams from multiple broadcast sources would corrupt the byte stream, and `Router::send()` clears `want_ack` on broadcasts anyway.

Prototype policy (explicitly non-shippable, replaced by a config field upstream):

- **RX:** the first node we receive a MAVLINK serial chunk from becomes the locked peer (logged); chunks from any other source are rejected and counted. Lock resets on reboot.
- **TX:** direct-message the locked peer once known (gets Meshtastic ACK/retransmit); until a peer is heard, chunks are broadcast on the bound serial channel so the two ends can discover each other.
- `ingestMeshPayload(NodeNum source, ...)` carries the source from Stage 1.
- Channel security: bound channel is the one named `serial` (AES-CTR, not authenticated; direct serial-channel packets are exempted from PKI by `Router`). Acceptable for the bench; noted for the upstream design.

### D4. Feature gating — opt-in for now

`MESHTASTIC_EXCLUDE_MAVLINK` defaults to **1** (excluded) in `configuration.h`; the HIL/native envs enable it with `-DMESHTASTIC_EXCLUDE_MAVLINK=0`. Default-on decisions wait for the Stage 4 flash/RAM measurements.

---

## Stage 1 — Transparent bridge

**Files:** `src/modules/Mavlink/MavlinkBridge.h`, `src/modules/Mavlink/MavlinkBridge.cpp`, `src/modules/SerialModule.h`, `src/modules/SerialModule.cpp`, `src/configuration.h`, `platformio.ini` (+ esp32 base ini), `test/test_mavlink/test_main.cpp`, `test/native-suite-count` (38 → 39).

### MavlinkBridge interface

```cpp
class MavlinkBridge
{
  public:
    explicit MavlinkBridge(Stream *serial);

    void ingestSerialBytes(const uint8_t *data, size_t len);
    // Transactional mesh-chunk handoff: peek copies without consuming; commit removes.
    size_t peekMeshPayload(uint8_t *out, size_t capacity);
    void commitMeshPayload(size_t len);
    void ingestMeshPayload(NodeNum source, const uint8_t *data, size_t len);
    void processOutput(); // bounded work per call: parse output FIFO, write frames to UART

    bool wantsMeshSend(uint32_t now) const; // full chunk ready, or data older than flush interval
    const Stats &getStats() const;

  private:
    bool acceptSource(NodeNum source); // peer locking (D3)
    void writeFrame(const mavlink_message_t &msg); // via pending-frame buffer, short-write safe
    ...
};
```

- `ByteFifo<N>` ring buffer (1024 in / 512 out): push accepts the fitting prefix and drops+counts the remainder; high-water marks tracked.
- **Frame forwarding rule (review must-fix #1):**
  - `MAVLINK_FRAMING_OK` → forward.
  - `MAVLINK_FRAMING_BAD_CRC` → `mavlink_get_msg_entry(msgid)`; entry absent ⇒ unknown-dialect frame, forward as-is (checksum preserved from the wire); entry present ⇒ genuinely corrupt, discard + count.
  - Serialization uses `mavlink_msg_to_send_buffer()` on the received message (writes stored len/checksum/signature — no re-finalize); a unit test asserts byte-exact round-trip for v1, v2, signed, and unknown-msgid frames. If exactness cannot be met for any case, switch to capturing raw frame bytes during parse.
- **UART write safety:** frames go through a pending-frame buffer with write offset; each `processOutput()` pass writes at most `serial->availableForWrite()` (where meaningful) and resumes next pass. `RADIO_STATUS` injection (Stage 2) waits until no partial frame is pending.
- Overflow / reject / framing-error counters with rate-limited `LOG_WARN` (`Throttle`).

### SerialModule integration

- MAVLINK branch of `runOnce()`: non-blocking `available()`/`read()` drain into `ingestSerialBytes`; `processOutput()`; then if `bridge->wantsMeshSend(millis())`:
  1. check `airTime->isTxAllowedChannelUtil()` **once per throttled window** (it logs on denial — don't hammer it every 10 ms) and router TX queue headroom,
  2. `allocDataPacket()` — if pool is exhausted (nullptr), leave bytes in the FIFO,
  3. `peekMeshPayload()` into the packet, send (direct to locked peer, else broadcast), `commitMeshPayload()` after handoff. One chunk per pass.
- Flush policy: send when a full 233-byte chunk is available or oldest queued byte is older than ~100 ms.
- `SerialModuleRadio::handleReceived()` (mode==MAVLINK, not from us): `bridge->ingestMeshPayload(getFrom(&mp), p.payload.bytes, p.payload.size)`.
- Init: bridge constructed at first-time MAVLINK init on the actual UART instance the module configured (passed explicitly — not the global `serialPrint`); ESP32 RX buffer bumped to 1024 for this mode.

### Tests (native, mock Stream with capacity control)

- v1 / v2 / signed v2 byte-exact end-to-end.
- Unknown-dialect msgid (crafted CRC-extra) forwarded, incl. signed-unknown; corrupt known frame discarded and counted.
- Splits: frame across 2 and N chunks; multiple frames per chunk; boundary-exact and mid-frame chunk ends.
- Garbage prefix; lost middle chunk → resync (≥2 clean frames after damage per review note).
- FIFO overflow both sides: prefix-accept semantics, counts, stream continues.
- Peer locking: second source rejected + counted.
- Short/zero-capacity UART writes: frame completes across passes, no interleaving.
- Mode-11 protobuf round-trip (D1).
- peek/commit transactionality: peek without commit leaves FIFO intact.

**Verify:** `./bin/run-tests.sh -f test_mavlink`; `pio run -e heltec-v3` with `MESHTASTIC_EXCLUDE_MAVLINK` 0 and 1 (default). → codex → `REVIEW_1.md` → fix → commit.

---

## Stage 2 — Flow control (RADIO_STATUS)

**Files:** `MavlinkBridge.{h,cpp}`, `test/test_mavlink/test_main.cpp`.

- `serviceFlowControl(now)` (injected time for testability): every 500 ms, and only when there has been *recent* (~10 s) peer or UART activity, emit local `RADIO_STATUS` to UART (never interleaved with a pending partial frame):
  - `txbuf` = free input-FIFO %, ExpressLRS formula.
  - sysid 255 / `MAV_COMP_ID_TELEMETRY_RADIO`.
  - `rssi`/`noise` from last received chunk's `rx_rssi`/`rx_snr` (final-hop values, documented as such), mapped conservatively; `UINT8_MAX` = invalid where unknown (0 is a valid value, per spec); `remrssi`/`remnoise` invalid; `rxerrors` = 16-bit-saturated framing-error count; `fixed` = 0.
  - Compile-time kill switch for signed-only installs.
- Rollover-safe via `Throttle` + injected `now`; mandatory rollover unit test.

**Tests:** txbuf empty/half/full; cadence incl. millis wrap; suppression when idle; no emission mid-frame.

→ codex → `REVIEW_2.md` → fix → commit.

---

## Stage 3 — Local telemetry snooping

**Files:** `MavlinkBridge.{h,cpp}`, `test/test_mavlink/test_main.cpp`, `src/modules/Telemetry/DeviceTelemetry.{h,cpp}` (battery), position hookup per below.

- Snooping runs on the **UART ingest path** with its own `mavlink_frame_char_buffer()` state pair; decode-only, forwarding untouched.
- `HEARTBEAT`: role detect (`MAV_TYPE_GCS` ⇒ GROUND; valid autopilot+vehicle ⇒ AIR) **filtered to the first-learned autopilot sysid/component** so extra components (gimbal, GCS heartbeats on the vehicle bus) don't flap the role. Snooping applies only in AIR role.
- Position (AIR only, throttled ≥1 s): `GLOBAL_POSITION_INT` primary (lat/lon 1e7 pass-through; alt mm→m; vx/vy cm/s→ Meshtastic `ground_speed` units per mesh.proto — verify m/s vs km/h from proto comments at implementation; hdg centideg, `UINT16_MAX`=invalid); `GPS_RAW_INT` supplies fix type/sats (and full fallback only when no recent GLOBAL_POSITION_INT). Delivery mirrors the real GPS path: `nodeDB->setLocalPosition()` **plus** the `PositionModule::handleNewPosition()`-equivalent hook so smart-broadcast scheduling and the NodeDB satellite entry actually engage (exact call verified against GPS code at implementation). No extra mesh packets. Bridge exposes decoded state via callback/accessor so unit tests don't drag in NodeDB.
- Battery: override **only** outbound `DeviceMetrics` in `DeviceTelemetryModule::getDeviceTelemetry()` — never `PowerStatus` (charging/shutdown safety logic must see real hardware). Bridge stores latest level/voltage + timestamp; accessor applies rollover-safe staleness cutoff (~120 s) after which hardware readings resume. `BATTERY_STATUS` id 0 (first seen) preferred; sum only valid cells (`UINT16_MAX` skipped, extension cells included); `battery_remaining == -1` ⇒ level unknown; `SYS_STATUS` only when no recent valid `BATTERY_STATUS`.
- No STATUSTEXT/command/mode conversion into mesh packets.

**Tests:** position decode + conversions + invalid-value handling; GCS-role frames don't update position; battery selection/validity/staleness; heartbeat role + sysid learn; mode-11 config still round-trips.

→ codex → `REVIEW_3.md` → fix → commit.

---

## Stage 4 — Size, flash, HIL

1. **Size:** flash/RAM delta vs `develop` for the stick env with feature on/off; record here. Includes parser+FIFO static RAM and UART buffer.
2. **Board confirm:** read hw model from both running devices (MCP `device_info` locally; ssh to pc2.lan for the remote) → pick `heltec-v3` / `heltec-wsl-v3`. Flash local via MCP; remote: scp factory bin + esptool over ssh. *Flashing and config changes are operator-visible actions — announced in the session log as they happen.*
3. **Config (both nodes):** `serial.enabled=true`, `serial.mode=11` (raw value via admin/MCP if the python CLI refuses), baud 57600, RX/TX pins, and — critical (review must-fix #3) — a **secondary channel literally named `serial`** with a shared non-default PSK on both nodes (receive dispatch requires the name match; default LongFast alone is rejected before `handleReceived`).
4. **One-sided loopback smoke test** (review must-fix #6 — two-sided loopback is an infinite amplifier):
   - Node A: `rxd == txd` (GPIO matrix loopback). Node B: TX pin unconnected (terminal sink).
   - A's periodic RADIO_STATUS loops into its own input FIFO → mesh → B → B's UART TX → dies. Verifies A's UART→mesh and B's mesh→parser→UART paths with real radio traffic.
   - Swap roles by config (no reflash) to cover the reverse direction.
   - Watch: frame counters advance, framing errors ≈0, no FIFO overflow, txbuf sane, no duplicate position/telemetry packets on the mesh, peer lock logged correctly; disable/re-enable one side → clean resync.
5. **Full SITL/GCS validation** (MAVLINK.md §14.2: heartbeats, rate commands, ACKs, params, signed frames, loss recovery, custom dialect) requires physically wiring a USB-UART/FC to a stick's GPIOs — not possible remotely; documented as the explicit follow-up with the user.

→ codex final-diff review → `REVIEW_4.md` → fix → final commit.

---

## Out of scope

Multi-vehicle sysid routing; RC/video; custom reliability layers; message filtering; the protobufs-repo PR (enum + peer field — upstream prerequisite for shipping); non-ESP32 validation.

## Risks / watch items

- mavlink headers vs Xtensa packed-member warnings → pragmas confined to includes if needed.
- `mavlink_msg_to_send_buffer` byte-exactness for signed/non-canonical frames — verified by test before relying on it (fallback: raw capture during parse).
- 512 B output FIFO holds one max signed v2 frame (280 B) + headroom; revisit with HIL data.
- 10 ms poll at 57600 ≈ 58 B/poll; 1024 B RX buffer covers bursts; loopback test confirms.
- CLI may refuse enum 11 → raw admin config write path ready.
