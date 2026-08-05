# PX4 SITL mesh test — findings

First hardware run, 2026-08-04. Firmware `2.8.0.69c06a6` on both nodes (heltec-wsl-v3),
PX4 `v1.18.0-beta1-216`, air node UART at 57600 on GPIO 47/48.

## Open

### 1. COMMAND_ACK is starved by the bridge's RADIO_STATUS txbuf

| | |
| --- | --- |
| Where | `src/modules/Mavlink/MavlinkBridge.cpp:136`, `MavlinkBridge.h:111` |
| Status | Open — no fix attempted |
| Evidence | `reports/px4-sitl-mesh-20260804-202938/` |

PX4 gates the ack send on `get_free_tx_buf() >= COMMAND_ACK_TOTAL_LEN`
(`mavlink_main.cpp:2994`). `get_free_tx_buf()` is driven by the RADIO_STATUS `txbuf`
the bridge emits every 500 ms (`rs.txbuf = transport.outboundFreePercent()`).

Observed: 376 RADIO_STATUS in one 240 s run, PX4 reporting `tx rate mult: 0.721`.
No `COMMAND_ACK` reached the GCS at all — it is absent from the message counts, which
are tallied before any filtering. PX4 logged neither `Ignore command` nor
`vehicle_command_ack lost`, so the command was accepted and the ack was simply never
transmitted.

`AUTOPILOT_VERSION` still arrived 3×, because `handle_request_message_command` sends it
directly and bypasses that gate. So the mesh path is not at fault — the txbuf reporting
is throttling PX4 hard enough to suppress the ack.

### 2. The documented pass condition cannot be met as specified

`PX4_SITL_MESH_TEST.md` enables only `HEARTBEAT` and `HIGH_LATENCY2`, then requires
`COMMAND_ACK` to return. Given finding 1, those two requirements are in conflict.

### 3. Stale machine-specific values in the doc

| Documented | Actual |
| --- | --- |
| `PX4_DIR` = `other_software/PX4-Autopilot` | does not exist; the live checkout is `px4dev/PX4-Autopilot` |
| `GROUND_HOST` = `192.168.0.232` | `192.168.0.244` (mDNS `_meshtastic._tcp`, `id=!3a180e17`) |
| air baud 115200 | node is configured 57600 |
| "Use the stable `/dev/serial/by-id/...` path" | unusable here — both CP2102 bridges report serial `0001`, so the by-id name collides and resolves to whichever enumerated. Use `/dev/serial/by-path/`. |

## Fixed

All three prevented the harness from ever completing a run, on any machine.

| # | Defect | Where |
| --- | --- | --- |
| 4 | `pexpect.spawn(preexec_fn=os.setsid)` raised `PermissionError: [Errno 1]` every run. `pty.fork()` already makes the child a session leader, so the second `setsid()` is EPERM by definition. `os.killpg` still works without it — the child is its own process-group leader. | `px4_sitl_mesh_test.py:165` |
| 5 | Serial-device assertion read `mavlink status streams`, which prints only the stream rate table. Only `mavlink status` prints `transport protocol: serial (<dev> @<baud>)`. | `px4_sitl_mesh_test.py:189` |
| 6 | `PROMPT = pxh>\s*` matched inside the command's own echo — pxh redraws `pxh>` for every echoed character. `run_command` returned an empty string, so **every** assertion on command output was silently reading nothing. Now consumes the echo with `expect_exact(command)` first. | `px4_sitl_mesh_test.py:206` |

## Verified working

Complete MAVLink frames cross UART → LoRa → UDP in both directions.

| Message | Count |
| --- | --- |
| HEARTBEAT | 148 |
| HIGH_LATENCY2 | 69 |
| AUTOPILOT_VERSION | 3 |
| RADIO_STATUS | 376 |
| MISSION_CURRENT | 142 |
| PARAM_VALUE / COMMAND_LONG | 3 / 3 |

PX4 bound the real UART (`transport protocol: serial (/dev/ttyUSB0 @57600)`), streams
applied exactly as configured (`HEARTBEAT 1.00`, `HIGH_LATENCY2 0.50`), the localhost GCS
instance on 18570 was stopped, and `MAV_CMD_REQUEST_MESSAGE` sent through the mesh reached
PX4 and was answered. `COMMAND_LONG` appears in the counts because the UDP server tees every
locally completed frame back to the client.

`UNKNOWN_410` / `UNKNOWN_411` are PX4 messages newer than the pinned pymavlink dialect, not
corruption.
