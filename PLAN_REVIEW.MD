## Verdict

PLAN.md has the right high-level architecture, but it is not implementation-ready. The FIFO-based raw transport, `SERIAL_APP` reuse, passive telemetry snooping, and staged rollout all match MAVLINK.md. However, several current choices either contradict the design or would make the resulting bridge fail key requirements.

The biggest blockers are arbitrary custom-message forwarding, broadcast/source interleaving, parser-state ownership, destructive FIFO dequeue before mesh acceptance, and the two-node UART loopback storm.

### D1–D4 summary

| Decision | Assessment |
|---|---|
| D1 cast value 11 | Acceptable for a private prototype only. Not suitable as the final implementation or upstream submission. |
| D2 pinned `c_library_v2` | Pinning is sound, but the include/parser-state plan is internally inconsistent and common-only parsing does not automatically pass custom dialects. |
| D3 broadcast on serial channel | Unsound for a command-capable bridge and contradicts the design’s direct-peer recommendation. |
| D4 feature gating | Good concept, but the plan lacks the actual target/build-macro matrix and enables the feature before measuring cost. |

## Must-fix issues

### 1. The plan does not transport unknown/custom-dialect messages as required

The design explicitly requires forwarding standard, custom, and unknown messages without filtering in [MAVLINK.md](</media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/MAVLINK.md:17>) and requires an unknown/custom test at line 569. The plan instead admits that common-only parsing rejects genuinely unknown IDs and substitutes a “valid-but-unusual common message” in [PLAN.md](</media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/PLAN.md:98>). That does not test or implement the requirement.

The generated parser deliberately returns `MAVLINK_FRAMING_BAD_CRC` when a message ID is absent from the dialect CRC table; it also preserves the wire checksum specifically so callers may forward such a frame. See the current [official `mavlink_helpers.h`](https://raw.githubusercontent.com/mavlink/c_library_v2/master/mavlink_helpers.h).

The plan should specify:

- Forward `MAVLINK_FRAMING_OK` for known messages.
- On `MAVLINK_FRAMING_BAD_CRC`, query `mavlink_get_msg_entry(msgid)`.
- If the entry is absent, treat it as an unknown dialect frame and forward it without claiming CRC validation.
- If the entry exists, treat it as an actually corrupt known frame and discard/count it.
- Test a genuinely unknown message ID with a custom CRC extra, including a signed unknown frame.

There is also a byte-preservation issue: `mavlink_msg_to_send_buffer()` trims MAVLink 2 trailing-zero payload bytes. It is normally fine for generated outbound frames, but rebuilding a received frame while retaining its original checksum/signature can invalidate a non-canonical frame. The plan should either capture and emit the exact raw bytes for accepted frames or use serialization that preserves the received payload length exactly.

### 2. Broadcast-only transport contradicts the design and breaks with more than one sender

The design recommends a configured direct peer for command-capable operation in [MAVLINK.md](</media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/MAVLINK.md:401>). PLAN.md replaces that with broadcast because adding a peer field is out of scope ([PLAN.md](</media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/PLAN.md:38>)).

That has several concrete consequences:

- Chunks from two broadcasting MAVLink nodes can interleave in one output FIFO and parser, corrupting both byte streams.
- Any node on the channel can inject bytes into the command stream.
- Broadcast ACKs do not “stay as-is”: `Router::send()` explicitly clears `want_ack` for broadcasts in [Router.cpp](</media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/src/mesh/Router.cpp:412>).
- A direct packet on the named `serial` channel still does not automatically get PKI; the Router explicitly exempts serial/gpio channels in [Router.cpp](</media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/src/mesh/Router.cpp:1019>). Therefore “encryption and authentication provided by Meshtastic” is too strong: normal channel encryption is AES-CTR and is not authenticated.

Stage 1 needs one of these explicit architectures:

1. Preferred: add `peer_node` to the upstream protobuf along with `MAVLINK = 11`, send direct, and reject incoming serial chunks whose `from` does not match the configured peer.
2. Prototype-only: lock onto one explicitly configured compile-time/test peer and clearly mark the branch non-shippable.
3. Much more expensive: maintain independent FIFO/parser state per source node.

If authenticated commands are required, decide whether direct serial packets will explicitly request PKI. If so, the 233-byte chunk ceiling must be reduced to accommodate PKI overhead and actual encoded `Data` size.

### 3. The HIL channel configuration is wrong

The plan says the default LongFast channel is sufficient in [PLAN.md](</media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/PLAN.md:144>). It is not.

`SerialModuleRadio` binds default-like serial modes to a channel named literally `serial` in [SerialModule.cpp](</media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/src/modules/SerialModule.cpp:112>). `Channels::getByName("serial")` falls back to the primary channel when no such channel exists ([Channels.cpp](</media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/src/mesh/Channels.cpp:359>)), but receive dispatch still checks that the channel name equals `serial` ([MeshModule.cpp](</media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/src/mesh/MeshModule.cpp:142>)). Consequently, a packet sent on the fallback LongFast primary can be rejected before `handleReceived()`.

HIL setup must create matching private channels named `serial` on both devices, preferably with a non-default PSK.

### 4. Parser ownership and `MAVLINK_COMM_NUM_BUFFERS` are not correctly specified

The class sketch stores `mavlink_message_t` and `mavlink_status_t` in its header ([PLAN.md](</media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/PLAN.md:71>)), while D2 says only the `.cpp` includes `common/mavlink.h`. Both cannot be true without an opaque implementation.

More importantly, `mavlink_frame_char()` does not use the class’s `rxMessage`/`rxStatus` as its parser storage. It parses through internal per-channel static buffers and copies results into those arguments. That causes:

- Parser state shared by all `MavlinkBridge` instances in the same TU.
- Test contamination when one test leaves an incomplete frame.
- A brittle Stage 3 dependency on `MAVLINK_COMM_NUM_BUFFERS == 2`.
- Redundant RAM because the class members are copies in addition to internal channel storage.

Use `mavlink_frame_char_buffer()` instead. It is intended for caller-owned parser buffers. Give each direction its own persistent `mavlink_message_t` plus `mavlink_status_t`. Then:

- `MAVLINK_COMM_NUM_BUFFERS` is no longer part of RX parser correctness.
- Concurrent/test bridge instances have independent state.
- The two snooping directions are explicit.
- The header must either include the MAVLink types or use a genuinely opaque implementation.

The two-channel approach in Stage 3 would otherwise work, but it is inferior to the buffer API for this standalone class.

### 5. FIFO dequeue and mesh submission are not transactional

`buildMeshPayload()` removes bytes before the caller knows that a mesh packet can be allocated or accepted. Current `MeshService::sendToMesh()` returns `void` and explicitly notes that a full Router FIFO causes a drop in [MeshService.cpp](</media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/src/mesh/MeshService.cpp:317>).

Therefore “airtime permits” does not prevent silent stream loss from:

- Packet-pool exhaustion.
- Router TX-queue exhaustion.
- Duty-cycle rejection.
- Encoding/size failure.
- Missing channel.

Revise the interface to peek/copy a chunk and commit the FIFO removal only after packet allocation and successful enqueue, or explicitly account for an unavoidable mesh-submit drop. This probably requires propagating `Router::sendLocal()`’s result rather than hiding it behind the current void API.

Pacing must also check `router->getQueueStatus()`, not only channel utilization. Calling `airTime->isTxAllowedChannelUtil()` every 10 ms while congested will itself produce repeated warnings because that helper logs on denial.

### 6. The proposed two-node `rxd == txd` test creates an infinite packet amplifier

With both nodes looped back as described in [PLAN.md](</media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/PLAN.md:145>):

1. A frame crosses A → B.
2. B writes it to TX.
3. B’s RX reads it again.
4. It crosses B → A.
5. A loops it back again.

Every periodic `RADIO_STATUS` frame circulates forever, while both nodes continue injecting new frames at 2 Hz. Mesh packet IDs change on every pass, so Meshtastic duplicate suppression will not stop it. FIFO overflow and congestion are guaranteed eventually.

Use one-sided loopback and an external sink on the other side, then swap directions. Better, use a USB-UART adapter or SITL endpoint so one injected frame has a terminal consumer. Full SITL/GCS validation should not be deferred beyond the “final” stage: the design explicitly places SITL/MAVProxy validation in Phase 1 ([MAVLINK.md](</media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/MAVLINK.md:623>)).

### 7. Stage 3’s position hook will not fully engage PositionModule scheduling

`NodeDB::setLocalPosition()` only replaces the global local position in [NodeDB.h](</media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/src/mesh/NodeDB.h:481>). The normal GPS path subsequently calls `PositionModule::handleNewPosition()`, which refreshes the local NodeDB satellite entry and evaluates smart broadcast policy in [PositionModule.cpp](</media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/src/modules/PositionModule.cpp:649>).

The plan needs to mirror that flow after an accepted MAVLink position update. Otherwise periodic/smart position broadcast may continue seeing no valid cached self-position.

Also specify exact conversions:

- MAVLink latitude/longitude already use degrees × 1e7.
- Altitude is millimetres MSL and must become integer metres.
- `GLOBAL_POSITION_INT.vx/vy` and `GPS_RAW_INT.vel` are cm/s; Meshtastic currently stores `ground_speed` as km/h.
- MAVLink heading/course is centidegrees; Meshtastic uses degrees × 1e5.
- Handle `UINT16_MAX` heading/course and invalid GPS fix values.
- Define how `GLOBAL_POSITION_INT` remains primary while `GPS_RAW_INT` supplies ancillary fix/satellite data without repeatedly overwriting the fused position.
- Filter updates by the learned autopilot sysid/component so multiple MAVLink components do not make the AIR/GROUND role flap.

### 8. The battery hook must not alter power-management safety state

The plan leaves the hook undecided between `Power` and `DeviceTelemetry`. That decision should be made before implementation.

The design only calls for overriding `DeviceMetrics.battery_level` and `voltage`. The narrow safe integration point is `DeviceTelemetryModule::getDeviceTelemetry()` in [DeviceTelemetry.cpp](</media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/src/modules/Telemetry/DeviceTelemetry.cpp:94>). Altering `PowerStatus` could incorrectly affect charging detection, low-battery shutdown, and other hardware safety decisions.

Specify:

- MAVLink override applies only to outbound `DeviceMetrics`.
- A rollover-safe staleness cutoff restores the local hardware reading.
- `BATTERY_STATUS` battery selection when multiple IDs exist.
- Sum only valid cell voltages; handle `UINT16_MAX`, extension cells, and `battery_remaining == -1`.
- Use `SYS_STATUS` only when a recent valid `BATTERY_STATUS` is unavailable.

### 9. D1 needs an explicit prototype-versus-shipping boundary

The cast to enum value 11 is likely workable on the firmware side because the generated field is a nanopb `UENUM` ([module_config.pb.h](</media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/src/mesh/generated/meshtastic/module_config.pb.h:977>)). It avoids forbidden edits under `src/mesh/generated`.

But it leaves:

- Generated enum maximum/name metadata at 10.
- Host clients without the `MAVLINK` name.
- Unverified CLI/MCP configuration behavior.
- No peer-node configuration needed by the design.
- No canonical protobuf source-of-truth.

Add a Stage 0 protobufs PR for `MAVLINK = 11` and a peer destination field. The cast can remain temporarily for a private branch, with a unit/integration test proving admin decode, persistence, reboot, and readback of numeric value 11.

### 10. Native suite registration is missing

Adding `test/test_mavlink` requires incrementing [test/native-suite-count](</media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/test/native-suite-count:1>) from its current 38. Otherwise the canonical full run returns AMBER even if every test passes.

Add that file to Stage 1’s file list and use `./bin/run-tests.sh -f test_mavlink` as the filtered canonical command.

## Should-fix issues

### Threading

The no-lock conclusion is sound for the current path: `OSThread`s run through the main controller, and Router dispatch calls modules synchronously. Radio interrupts enqueue packets rather than invoking `SerialModuleRadio::handleReceived()` directly.

Still, document a stronger invariant: every FIFO method must be called only from the main scheduler. If any future UART task, ISR, or separate receive thread calls the bridge, the FIFO immediately needs locking. Avoid claiming that “Meshtastic threading is cooperative” globally; other subsystems do use locks and asynchronous callbacks.

### Arduino `Stream` output semantics

`processOutput()` must handle short or blocking writes. A single `Stream::write(buffer, len)` is not guaranteed by the abstraction to accept the whole frame without blocking.

Keep a pending serialized frame plus write offset, consult `availableForWrite()` where supported, and bound bytes written per pass. Native tests should simulate:

- Short writes.
- Zero writable capacity.
- Recovery across several calls.
- No interleaving of `RADIO_STATUS` with a partially written forwarded frame.

The bridge should also receive the actual selected UART object, not rely on the current global `Print *serialPrint`.

### RADIO_STATUS details

The `txbuf` formula and 2 Hz starting cadence are reasonable. Tighten the rest:

- `UINT8_MAX` is the defined unavailable value; zero is a valid device-dependent RSSI/noise value. See the official [`RADIO_STATUS` definition](https://mavlink.io/en/messages/common.html#RADIO_STATUS).
- `rx_rssi`/`rx_snr` describe the final LoRa hop, not necessarily the end-to-end peer on a routed mesh.
- Derive local noise consistently if desired; remote RSSI/noise are unknown unless they are explicitly exchanged.
- Define `rxerrors` and `fixed`, including saturation to 16 bits.
- “Traffic seen” should mean recent activity, not “ever seen since boot.”
- Make the rollover test mandatory by accepting an injected `now` value; “if practical” is too weak for a listed design requirement.

### Feature gating and dependency placement

`MESHTASTIC_EXCLUDE_MAVLINK` is a reasonable guard, but the plan does not list the necessary `configuration.h` changes or define the target matrix precisely. Current SerialModule compilation is architecture-gated in [SerialModule.h](</media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/src/modules/SerialModule.h:11>), while native only builds the bridge for direct tests.

Initially make the feature opt-in for the HIL build, measure it, then decide whether all ESP32 targets should receive the linked code. Measure:

- Flash with and without the common CRC table and selected decoders.
- Static/parser/FIFO RAM.
- Heap allocation when serial MAVLink mode is enabled.
- ESP32 UART RX buffer allocation.
- Stack use during max-length signed-frame serialization.

Pinning a commit ZIP matches existing `platformio.ini` practice, but replace `<pinned-commit>` before implementation and add a PlatformIO include smoke build. Adding it under global `[env] lib_deps` also causes every environment to download it even when the feature is excluded.

### HIL coverage

The final HIL stage is much weaker than MAVLINK.md. It needs at least one reproducible SITL/GCS run covering heartbeat, rate commands, command ACK, parameters, signed frames, packet-loss recovery, and a genuine custom-dialect frame. The one-sided hardware loopback is useful only as an electrical/transport smoke test.

Device flashing and config mutation should also be explicit operator-approval gates.

## Minor notes

- The 1024-byte input and 512-byte output FIFOs are sensible ESP32 starting values. A 512-byte output FIFO can hold one maximum 280-byte signed MAVLink 2 frame plus substantial following data.
- Define whether a FIFO span push accepts the fitting prefix and drops only the remainder; that best matches “reject bytes that do not fit.”
- Resynchronization may occur on a later valid frame, not necessarily the immediately following frame. Tests should send at least two clean frames after a damaged chunk.
- `Stats` should be exposed through a const accessor rather than as a publicly mutable member.
- The plan should preserve source `NodeNum` in `ingestMeshPayload()` from Stage 1, even if multi-vehicle routing is deferred. It is already needed for peer filtering, metadata, and diagnostics.
- The board naming in Stage 4 conflates Wireless Stick V3 with Wireless Stick Lite V3. The repo environments are `heltec-v3` and `heltec-wsl-v3`; confirm the actual hardware before building.

In short: retain the staged structure, but add a Stage 0 for protobuf/destination decisions, redesign parser ownership around `mavlink_frame_char_buffer()`, make dequeue-to-send transactional, replace the two-sided loopback, and restore the genuine unknown/custom-frame requirement.