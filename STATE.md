# MAVLink Bridge - Session State / Handoff

**As of:** Stages 1-3 code-complete, all builds green, committed as one checkpoint (this commit).
**Branch:** `mavlink` · **Parent commit:** `444c1691e` "MAVLink serial bridge stage 1: transparent transport"
**Working tree:** clean (Stages 1-3 + this doc committed together).

---

## TL;DR

1. **Stage 1 must-fixes (REVIEW_1.md) are applied**: raw-frame capture replaced re-serialization
   (MF1/MF2); `MeshService::sendToMesh` now returns `ErrorCode` and the FIFO commit is gated on
   `ERRNO_OK` (MF3). Should-fixes folded in: empty-chunk guard before peer lock, `isValidConfig`
   rejects mode 11 when excluded, missing-`serial`-channel warning (rate-limited, first fire not
   suppressed), `framingErrors` stat feeding `rxerrors`, `MAVLINK_COMM_NUM_BUFFERS` ifndef-guarded.
2. **Stage 2 (RADIO_STATUS flow control) complete**: `serviceFlowControl()`, 2 Hz, activity-gated,
   txbuf = free input-FIFO %, SiK-style rssi/noise from final LoRa hop (UINT8_MAX unknown),
   `MESHTASTIC_MAVLINK_NO_RADIO_STATUS` kill switch, never interleaves with a pending frame.
3. **Stage 3 (telemetry snooping) complete**: second `mavlink_frame_char_buffer()` state pair on
   the UART ingest path (decode-only, forwarding untouched). Heartbeat role learn (GCS→GROUND,
   valid autopilot→AIR, first-learned sysid/compid filter, `MESHTASTIC_MAVLINK_ROLE` compile-time
   override). Position: GLOBAL_POSITION_INT primary, GPS_RAW_INT fix/sats + stale fallback
   (GLOBAL_POS_FRESH_MS), delivered at ≤1 Hz via `nodeDB->setLocalPosition()` +
   `positionModule->handleNewPosition()` (LOC_EXTERNAL, km/h ground_speed, deg×1e5 ground_track -
   the firmware's real conventions, verified in GPS.cpp). Battery: BATTERY_STATUS (first-seen id
   sticks, valid-cell sum incl. voltages_ext, -1 = unknown) with SYS_STATUS 10 s-deferred fallback;
   overrides **only** outbound `DeviceMetrics` in `DeviceTelemetryModule::getDeviceTelemetry()`
   with a 120 s staleness cutoff; PowerStatus untouched.
4. **Builds**: `pio run -e heltec-v3` green with feature on (Flash 2306399 B, RAM 130792 B static)
   AND green with `PLATFORMIO_BUILD_FLAGS=-DMESHTASTIC_EXCLUDE_MAVLINK=1` (feature off).
5. **Next**: Stage 4 (size table, board confirm, flash, loopback smoke test, SITL follow-up).
   Codex review of the full diff can be run at any point; per house rules, ask before committing
   or flashing.

## Key design notes (don't re-derive)

- **Forward path replays captured raw wire bytes** (`rawFrame`/`rawLen`); the parser only finds
  boundaries and validates. A byte fed while `parse_state==IDLE` starts a new capture; garbage
  never accumulates. Signed frames: BAD_CRC arrives at CRC2 with `parse_state==SIGNATURE_WAIT`
  (msg invalid) → record `pendingSignedBadCrc`+msgid from `meshRxWorking`; the parser returns OK
  after the 13 signature bytes (no signing context) → apply known/unknown rule then.
  Known-msgid bad CRC → discard + `corruptFrames`; unknown → forward raw.
- `MeshService::sendToMesh` signature change (void→ErrorCode) is source-compatible; ~25 callers
  ignore the return. On non-OK the router has already released the packet - never touch it after.
- Position delivery requires `positionModule` non-null (guarded - `MESHTASTIC_EXCLUDE_GPS` builds).
- Channel check compares `ch->settings.name` (raw field), not the display fallback.
- Snoop gating: heartbeat always processed (role learn); position/battery only when role==AIR and
  from learned autopilot (or any source when role is forced/no autopilot learned).

## Still open from PLAN.md

- **Stage 4**: size table vs develop, board confirm (heltec-v3 vs heltec-wsl-v3 from live devices),
  flash 2 nodes, `serial`-named channel + PSK config, one-sided loopback smoke test, then full
  SITL/GCS validation (needs wiring - explicit user follow-up).
- Native test suite `test/test_mavlink/` - deferred per user; also blocked locally by missing
  `libyaml-cpp-dev` (pre-existing PortduinoGlue.h failure).
- Upstream protobufs PR (Serial_Mode.MAVLINK=11 + peer-node field) - shipping prerequisite,
  out of scope for this branch.
- REVIEW_1.md re-review was interrupted mid-run per user direction ("write the complete code,
  build at the end, then review"); review of the stages 1-3 checkpoint can be re-run any time.

## Toolchain notes

- codex: `codex exec --sandbox read-only --output-last-message <file> "<prompt>"` (background with
  `setsid nohup ... </dev/null` - plain `&` gets killed by shell timeouts; never `pkill -f` a
  pattern your own command line contains).
- `trunk` not installed here; format with
  `clang-format -i --style=file:.trunk/configs/.clang-format` (18.1.3 vs pinned 16 - fine) and
  strip em/en-dashes: `LC_ALL=C sed -i 's/\xe2\x80\x94/-/g; s/\xe2\x80\x93/-/g'` (trunk's
  ascii-dash rule; `trunk check` enforces it).
- Build: `~/.platformio/penv/bin/pio run -e heltec-v3` (~5.5 min clean, ~1 min incremental).
