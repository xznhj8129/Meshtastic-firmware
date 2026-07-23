## Verdict

**Request changes.** The ring buffer, scheduler ownership, UART selection, short-write state, rollover arithmetic, and direct-peer ACK policy are generally sound. Three correctness defects block Stage 1 from being a transparent MAVLink bridge.

## Must-fix

1. **Signed unknown/BAD_CRC frames use uninitialized data and are forwarded incorrectly.**

   [MavlinkBridge.cpp:81](/media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/src/modules/Mavlink/MavlinkBridge.cpp:81) declares `msg` uninitialized and examines it whenever the parser returns `BAD_CRC`.

   The installed parser returns `BAD_CRC` immediately after CRC2 for signed frames, before consuming the 13-byte signature. At that point it has not copied `rxmsg` into `r_message`; that copy happens only after the signature completes ([mavlink_helpers.h:761](/media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/.pio/libdeps/heltec-v3/c_library_v2/mavlink_helpers.h:761), [mavlink_helpers.h:819](/media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/.pio/libdeps/heltec-v3/c_library_v2/mavlink_helpers.h:819)). Consequently, [MavlinkBridge.cpp:89](/media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/src/modules/Mavlink/MavlinkBridge.cpp:89) reads an indeterminate `msg.msgid` and may serialize garbage.

   It gets worse: `msg_received` is reset on every byte. After the signature arrives, the parser returns `OK` because no signing context is configured, forgetting the earlier bad CRC. The bridge can therefore forward a known signed corrupt frame, while a signed unknown frame can produce both a garbage frame and a second frame with the parser-computed CRC-extra-0 checksum rather than the wire checksum.

   This needs persistent raw-frame/BAD_CRC state through the complete signed frame. Raw-byte capture is the safest solution.

2. **`mavlink_msg_to_send_buffer()` is not byte-exact for received MAVLink 2 frames.**

   [MavlinkBridge.cpp:100](/media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/src/modules/Mavlink/MavlinkBridge.cpp:100) relies on the serializer preserving the received bytes. The actual pinned helper trims trailing-zero MAVLink 2 payload bytes at [mavlink_helpers.h:465](/media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/.pio/libdeps/heltec-v3/c_library_v2/mavlink_helpers.h:465), but retains the stored checksum and signature.

   Therefore:

   - MAVLink 1 is byte-exact.
   - Canonically trimmed MAVLink 2 is normally byte-exact.
   - A valid non-canonical MAVLink 2 frame with trailing zeros becomes CRC-invalid.
   - A signed such frame also becomes signature-invalid.

   This violates the explicit transparent/custom/signed transport requirement. Capture and emit the original frame bytes, or provide a serializer that preserves the received length exactly.

3. **The peek/commit handoff is still not transactional across mesh submission.**

   [SerialModule.cpp:455](/media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/src/modules/SerialModule.cpp:455) calls `sendToMesh()` and then unconditionally commits the bytes. But `sendToMesh()` returns `void` and explicitly discards the router result ([MeshService.cpp:317](/media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/src/mesh/MeshService.cpp:317)).

   The prechecks do not cover duty-cycle rejection, disabled/no radio, encoding failure, or final interface enqueue failure; see [Router.cpp:378](/media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/src/mesh/Router.cpp:378). Those paths free/drop the packet while the bridge records the chunk as transmitted.

   Propagate the actual `ErrorCode` through `MeshService::sendToMesh()` or another ownership-safe submission API, and commit only after confirmed acceptance.

## Should-fix

- **Missing `serial` channel fails silently.** `boundChannel` is non-null in MAVLink mode, but `Channels::getByName("serial")` falls back to primary ([Channels.cpp:359](/media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/src/mesh/Channels.cpp:359)). The receiver then rejects that primary-channel packet because its name is not `serial` ([MeshModule.cpp:142](/media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/src/mesh/MeshModule.cpp:142)). Validate/log the missing channel instead of transmitting into a black hole.

- **Peer locking can be captured by an empty or unrelated `SERIAL_APP` packet.** [MavlinkBridge.cpp:48](/media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/src/modules/Mavlink/MavlinkBridge.cpp:48) calls `acceptSource()` before checking `len`, permanently locking until reboot. Reject empty chunks before locking. The broader first-sender policy remains acceptable only under the documented bench/prototype boundary.

- **Feature-off mode 11 behaves asymmetrically instead of being rejected.** `Serial_Mode_MAVLINK` exists even when MAVLink is excluded ([SerialModule.h:14](/media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/src/modules/SerialModule.h:14)). A persisted raw mode 11 on a feature-off build falls through to generic UART→mesh sending, while mesh→UART dispatch matches no mode and drops data. `isValidConfig()` should reject mode 11 when excluded.

- **Dependency placement is broader and narrower than the gate implies.** Every ESP32 environment downloads the MAVLink dependency through [esp32-common.ini:61](/media/anon/WD2TB/DataVault/TechProjects/Software/GitRepos/meshtastic-firmware/variants/esp32/esp32-common.ini:61), including excluded C3/S2 targets. Conversely, enabling the advertised flag on nRF52/RP2040/STM32WL fails because those bases lack the dependency. Their default-excluded builds remain safe.

## Minor

- Parser ownership and locking are correct for the current cooperative scheduler; no mutex is presently required.
- ByteFifo wrap, prefix-accept, peek/drop, high-water tracking, flush timing, and rollover arithmetic are correct.
- Short writes and zero UART capacity are retained across calls correctly. The 500 ms timeout measures lack of progress, not total frame duration, which is appropriate.
- The selected UART matches each existing initialization branch; ESP32 RX buffer sizing occurs before `begin()`. Normal serial configuration changes reboot, so hot mode switching is not currently required.
- Direct chunks set `want_ack=true`; discovery broadcasts correctly do not. Packets from ourselves are excluded before MAVLink ingestion.
- `corruptFrames` only counts known CRC failures; other parser/framing errors are discarded without statistics. That will under-report Stage 2 `rxerrors`.
- `MAVLINK_COMM_NUM_BUFFERS` is unnecessary when exclusively using `mavlink_frame_char_buffer()` and is best removed from the public header to avoid future macro conflicts.

No files were modified. The review used the exact committed blobs despite the existing post-commit edit in `MavlinkBridge.h`.