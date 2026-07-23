# MAVLink-over-Meshtastic hardware session notes

Date: 2026-07-22 through 2026-07-23  
Branch: `mavlink`

This is the complete handoff record for the build, flashing, configuration, and live hardware work performed in this session. Credentials, channel keys, and machine-specific parent directory paths are intentionally omitted.

## Final state at halt

- All diagnostic commands and listeners started by the agent have stopped.
- INAV Configurator and X-Plane were user processes and were not stopped.
- Current firmware commit: `ce813f2a6`.
- Firmware version on both active WSL V3 nodes: `2.8.0.ce813f2`.
- Ground WSL V3:
  - Host: `pc2.lan`.
  - USB port: `/dev/ttyUSB0`.
  - Node ID: `!3a180e17`.
  - LAN address: `192.168.0.244`.
  - UDP MAVLink endpoint: port 14550.
  - Serial bridge mode 11 enabled.
  - No UART pins configured, intentionally selecting UDP-only operation.
  - Wi-Fi enabled.
  - Primary channel `Airmesh`.
  - Secondary channel `serial`.
- Air WSL V3:
  - Local USB port: `/dev/ttyUSB0`.
  - Current node ID after preferences erase: `!73a05699`.
  - Serial bridge mode 11 enabled.
  - RX GPIO47, TX GPIO48.
  - Baud enum 10, which is 57600 baud.
  - Region US, modem preset MediumFast.
  - Primary channel `Airmesh`.
  - Secondary channel `serial`.
  - USB Meshtastic API remains operational with GPIO47/48.
- The two `serial` channel key fingerprints match exactly:
  - `b1eece8c8b094dd7784841352c41ef6bde087cc1ef0c8c4a8895c5a153c2112a`
- At the last physical test, GPIO48 and GPIO47 on the air node had been jumpered for UART loopback. The FC should be considered disconnected from those pins unless physically confirmed otherwise.
- The loopback test failed: no fake MAVLink frame completed the ground → LoRa → air UART loopback → LoRa → ground path.
- The current fault is therefore not demonstrated to be an INAV fault. The complete two-node bridge still fails before any FC-specific behavior is required.

## Documents and source reviewed

The following project documents were read:

- `MAVLINK_README.md`
- `MAVLINK.md`
- `STATE.md` and related plan/review material as needed during implementation work
- `.github/copilot-instructions.md`
- `CLAUDE.md`
- Repository `AGENTS.md`

The INAV branch documentation was found under the local `inav_development` checkout and read:

- `inav/docs/Mavlink.md`

Relevant INAV behavior:

- INAV supports up to four MAVLink telemetry ports.
- The first MAVLink port uses configured default stream rates.
- Ports 2 through 4 start with heartbeat only at 1 Hz.
- Other stream groups on ports 2 through 4 remain disabled until requested with `REQUEST_DATA_STREAM` or `MAV_CMD_SET_MESSAGE_INTERVAL`.
- A secondary-port liveness test must therefore accept heartbeat alone as valid, or explicitly request low-rate streams.
- `PING` and `TIMESYNC` are supported and should receive replies.
- INAV uses MAVLink 2 by default.
- The tested FC was reported configured with UART4 in MAVLink telemetry mode at 57600 baud.

Relevant Meshtastic bridge requirements from `MAVLINK_README.md`:

- `serial.enabled = true`
- Raw `serial.mode = 11`
- A secondary channel literally named `serial`
- The same non-default PSK on the `serial` channel on both nodes
- Air node must have RX/TX pins and baud configured
- Ground node may have no UART pins and use Wi-Fi UDP port 14550
- Ground UDP is wait-for-client: the GCS must send the first datagram
- The first MAVLink peer heard is locked until reboot
- Before peer discovery, bridge chunks are broadcast
- A local `RADIO_STATUS` is emitted at 2 Hz after bridge activity

## Repository changes and commits

### SerialModule preprocessor fix

An extra preprocessor terminator in `src/modules/SerialModule.cpp` prevented the branch from building correctly.

Committed as:

```text
98a34357e Fix MAVLink SerialModule preprocessor guard
```

### Original T-LoRa V1 support

The original T-LoRa V1 was removed from normal flasher support and uses a 26 MHz ESP32 crystal. Work performed:

- Enabled MAVLink for `tlora-v1`.
- Added the pinned `mavlink/c_library_v2` dependency.
- Added a 26 MHz custom SDK configuration.
- Discovered that the packaged Arduino bootloader still recorded a 40 MHz crystal even when the framework was rebuilt for 26 MHz.
- Added an early runtime correction in `src/main.cpp`:

```cpp
#if defined(ARCH_ESP32) && defined(TLORA_V1)
    rtc_clk_xtal_freq_update(SOC_XTAL_FREQ_26M);
#endif
```

- The correction occurs before power HAL and console initialization.

Committed as:

```text
ce813f2a6 Enable MAVLink on original T-LoRa V1
```

The final commit contains only 15 inserted lines across:

- `src/main.cpp`
- `variants/esp32/tlora_v1/platformio.ini`

### Formatting mishap and correction

`trunk` was unavailable.

Running system `clang-format` against all of `src/main.cpp` mechanically rewrote almost the whole file. The resulting first commit showed roughly 969 insertions and 819 deletions. This was immediately detected from commit statistics.

Recovery:

1. Restored `src/main.cpp` from the parent commit.
2. Reapplied only the intended include and runtime crystal correction.
3. Amended the commit.
4. Verified the final diff was exactly 15 insertions.

Do not run the system `clang-format` blindly on this repository.

## Build results

### Heltec V3

- Built successfully.
- Flashed successfully during the initial local smoke test.
- Meshtastic CLI connected and returned normal device information.

### Wireless Stick Lite V3

PlatformIO environment:

```text
heltec-wsl-v3
```

Final build:

```text
firmware-heltec-wsl-v3-2.8.0.ce813f2.bin
firmware-heltec-wsl-v3-2.8.0.ce813f2.factory.bin
```

Application image:

```text
MD5: 0c629f5710f39bc8975ea893f5797bcd
Size: 2306816 bytes
```

Factory image:

```text
MD5: 4464cdd267d5b2fa5454fe45bde487b3
Size: 2372352 bytes
```

The first WSL V3 build took approximately nine minutes because the custom ESP-IDF/Arduino framework and dependencies were rebuilt. Subsequent direct esptool flashes used the completed artifacts and did not require rebuilding.

### T-LoRa V1

The committed build was named:

```text
firmware-tlora-v1-2.8.0.ce813f2.bin
```

The final committed rebuild was interrupted before its final artifact was produced. Earlier experimental T-LoRa builds did complete and were flashed remotely.

## Flashing procedures used

### WSL V3 factory flash

Direct esptool was faster and more predictable than invoking the PlatformIO upload target, which unexpectedly began recompiling.

```bash
python -m esptool \
  --chip esp32s3 \
  --port /dev/ttyUSB0 \
  --baud 921600 \
  write-flash \
  0x0 \
  .pio/build/heltec-wsl-v3/firmware-heltec-wsl-v3-2.8.0.ce813f2.factory.bin
```

Observed chip:

- ESP32-S3 revision 0.2
- Embedded 8 MB flash
- 40 MHz crystal

The write completed at approximately 880–995 kbit/s and esptool verified the image hash.

### WSL V3 application-only update

USB1 initially contained a `heltec-v3` build even though the physical device was another WSL V3. To preserve its channels and preferences, only the application partition was updated:

```bash
python -m esptool \
  --chip esp32s3 \
  --port /dev/ttyUSB1 \
  --baud 921600 \
  write-flash \
  0x10000 \
  .pio/build/heltec-wsl-v3/firmware-heltec-wsl-v3-2.8.0.ce813f2.bin
```

Afterward it correctly reported:

```text
pioEnv: heltec-wsl-v3
hwModel: HELTEC_WSL_V3
firmwareVersion: 2.8.0.ce813f2
```

### Preferences recovery after UART0 was assigned

Configuring air RX/TX as GPIO44/GPIO43 made the USB Meshtastic API inaccessible because those are the normal WSL V3 UART0 USB bridge pins.

A normal application reflash does not remove Meshtastic preferences.

An attempted full chip erase connected and began erasing, but the device disappeared during reset and esptool never printed a successful chip-erase confirmation. Reflashing the factory image afterward did not clear the bad serial-pin configuration.

The reliable recovery was to erase the exact preferences filesystem partition.

The WSL V3 `default_8MB.csv` partition layout is:

```text
nvs      0x009000  0x005000
otadata  0x00e000  0x002000
app0     0x010000  0x330000
app1     0x340000  0x330000
spiffs   0x670000  0x180000
coredump 0x7f0000  0x010000
```

Exact successful recovery command:

```bash
python -m esptool \
  --chip esp32s3 \
  --port /dev/ttyUSB0 \
  erase-region \
  0x670000 \
  0x180000
```

esptool reported:

```text
Flash memory region erased successfully
```

After first boot, the USB API connected again.

Important consequences of erasing preferences:

- Node identity changed.
- PKI keypair changed.
- Channels and LoRa region were lost.
- Serial settings were lost.
- Wi-Fi settings were lost.
- Cached node records on peers contained stale identities and public keys.

The air node changed from prior IDs such as `!51259eaa` to the current `!73a05699`.

## T-LoRa V1 remote attempt

Remote host:

```text
pc2.lan
```

Stable serial path at the time:

```text
/dev/serial/by-id/usb-1a86_USB_Single_Serial_531C010127-if00
```

The connected original T-LoRa V1 was confirmed by esptool as a 26 MHz ESP32.

The first MAVLink-enabled T-LoRa image flashed successfully and all written hashes verified. Meshtastic CLI then timed out.

Serial diagnosis:

- Firmware output was readable at 74880 baud rather than 115200.
- This was consistent with a 26/40 MHz clock mismatch.
- The framework custom SDK configuration alone did not fix it because the packaged bootloader remained the same 40 MHz binary.
- Official 2.7.15 release archives were inspected, but the original `tlora-v1` target was absent, consistent with its removal from the flasher.

The later runtime crystal correction was committed, but the final committed image was not flashed because the T-LoRa effort was abandoned.

The remote T-LoRa was left with the earlier experimental image whose normal serial API was unusable. Do not assume that device is in a good state.

## WSL V3 initial flash and basic verification

The first active WSL V3 was flashed on local `/dev/ttyUSB0`.

CLI verification returned:

- Firmware `2.8.0.ce813f2`
- Hardware `HELTEC_WSL_V3`
- Node ID at that time `!51259eaa`
- Working Meshtastic API

The image hash verified during flash.

Hermes was asked to send a Telegram completion notification. A detailed payload containing device metadata was rejected by the safety gate. A generic message was then sent successfully:

```text
Firmware flash and verification complete.
```

Hermes confirmed delivery to the configured Telegram home channel.

## Two-local-node mesh verification

At one point both local radios were attached:

- `/dev/ttyUSB0`: WSL V3 air candidate
- `/dev/ttyUSB1`: second WSL V3, initially running the wrong `heltec-v3` build

The USB1 node initially reported:

```text
pioEnv: heltec-v3
hwModel: HELTEC_V3
node: !3a180e17
```

After application-only correction it reported:

```text
pioEnv: heltec-wsl-v3
hwModel: HELTEC_WSL_V3
firmware: 2.8.0.ce813f2
node: !3a180e17
```

Both nodes had:

- Channel name `Airmesh`
- Matching primary channel fingerprint
- Region US
- MediumFast
- 250 kHz bandwidth
- Spread factor 11
- Coding rate 5
- TX enabled
- TX power 30

Direct-message ACK attempts initially failed:

- USB0 → USB1: `MAX_RETRANSMIT`
- USB1 → USB0: `PKI_SEND_FAIL_PUBLIC_KEY`

The PKI error was caused by stale peer keys after factory flashing and key regeneration. Direct messages use peer PKI and are not equivalent to channel-encrypted broadcast traffic.

Channel-encrypted broadcast tests were then performed with a CLI listener on the receiving node.

USB1 → USB0 succeeded:

```text
transport: LoRa
SNR: 11 dB
RSSI: -18 dBm
```

USB0 → USB1 succeeded:

```text
transport: LoRa
SNR: 10.5 dB
RSSI: -17 dBm
```

This proved both physical radios could transmit and receive on the shared mesh channel.

## Ground/GCS node configuration on pc2

The ground WSL V3 was moved to `pc2.lan` and appeared as `/dev/ttyUSB0`.

Initial state:

```text
serial.enabled: false
serial.mode: 0
serial.rxd: 0
serial.txd: 0
serial.baud: 0
wifi_enabled: true
```

It had only the primary `Airmesh` channel and no channel named `serial`.

This made MAVLink operation impossible despite valid firmware.

### Creating the serial channel

Attempt 1:

```bash
meshtastic --ch-add-url "<primary channel URL>"
```

Result:

```text
Warning: Invalid URL
```

Attempt 2:

```bash
meshtastic --ch-add serial
meshtastic --ch-set psk "<base64 PSK>" --ch-index 1
```

Result:

```text
expected bytes, str found
```

Working procedure:

1. Add the channel.
2. Read the existing primary PSK without printing it.
3. Decode base64 to raw bytes.
4. Convert raw bytes to hex.
5. Pass it with a `0x` prefix.

Conceptually:

```bash
meshtastic --ch-add serial
psk_hex="$(printf '%s' "$primary_psk" | base64 -d | xxd -p -c 256)"
meshtastic --ch-set psk "0x$psk_hex" --ch-index 1
```

The ground node was then configured:

```bash
meshtastic \
  --set serial.enabled true \
  --set serial.mode 11
```

Readback:

```text
serial.enabled: True
serial.mode: 11
serial.rxd: 0
serial.txd: 0
network.wifi_enabled: True
```

The missing pins are intentional for UDP-only ground operation.

The primary and `serial` channel key fingerprints were verified identical.

### Ground IP discovery

`meshtastic.local` resolved to:

```text
192.168.0.244
```

mDNS advertised the Meshtastic service.

### Ground UDP test

A fake GCS heartbeat was sent with pymavlink to:

```text
udpout:192.168.0.244:14550
```

The bridge immediately returned `RADIO_STATUS` at 2 Hz.

Typical response:

```text
txbuf: 98 initially, then 100
rssi/remrssi/noise/remnoise: 255 when unknown
rxerrors: 0
fixed: 0
```

Tests observed:

- 20 `RADIO_STATUS` messages in approximately 12 seconds.
- 60 in 30 seconds.
- 90 in 45 seconds.
- 110 in 55 seconds.

This proves:

- Ground Wi-Fi is up.
- UDP 14550 is bound.
- Wait-for-client registration works.
- UDP bytes enter the ground `MavlinkBridge`.
- The local flow-control output path works.

It does not prove that bridge chunks enter the LoRa mesh.

## Air-node configuration history

### Initial state

Before configuration, the air WSL V3 reported:

```text
serial.enabled: False
serial.mode: 0
serial.rxd: 0
serial.txd: 0
serial.baud: 0
```

It also lacked a secondary `serial` channel.

### First configuration: GPIO44/GPIO43

The WSL V3 board labels identify:

- U0RXD = GPIO44
- U0TXD = GPIO43

The air node was configured:

```text
serial.enabled: true
serial.mode: 11
serial.rxd: 44
serial.txd: 43
serial.baud: 10 (57600)
```

The shared `serial` channel was also added.

After reboot, USB Meshtastic API requests timed out. This is expected from assigning the USB-UART pins to the bridge and is why GPIO43/44 are unsuitable when USB debugging must remain connected.

An external FC and the onboard CP2102 should not simultaneously drive these pins.

### Remote recovery attempt

The air node remained reachable over LoRa, so a remote admin update through pc2 was attempted:

```text
set serial.rxd = 47
set serial.txd = 48
```

The air node responded:

```text
ADMIN_PUBLIC_KEY_UNAUTHORIZED
```

The remote change was not applied.

A Bluetooth scan was started as an alternate recovery path but was manually aborted.

### Final configuration: GPIO47/GPIO48

After preferences-partition recovery, channels and settings were rebuilt from the ground node.

Final verified readback:

```text
serial.enabled: True
serial.mode: 11
serial.rxd: 47
serial.txd: 48
serial.baud: 10
lora.region: 1
lora.modem_preset: 4
```

Meaning:

- RX GPIO47
- TX GPIO48
- 57600 baud
- Mode 11
- US region
- MediumFast

USB API remained usable after reboot.

## INAV/X-Plane live setup observed

Local processes showed:

- INAV Configurator 10 `mavlink_multiport2` build running.
- X-Plane 11 running.
- FC enumerated as `/dev/ttyACM0`.
- X-Plane held the FC serial device.

The FC was identified as:

```text
INAV SpeedyBee F405 Wing
```

Because X-Plane held the serial connection for HITL, the session did not open a second MSP connection and did not disturb the running simulator.

The user reported:

- INAV 10 `mavlink_multiport2`
- UART4 configured as MAVLink
- 57600 baud
- FC connected to X-Plane HITL and powered

## End-to-end MAVLink tests with FC connected

The GCS test client used:

```text
source system: 255
source component: 190
destination UDP: 192.168.0.244:14550
```

### Basic heartbeat tests

Repeated GCS heartbeats were sent for windows of 30 to 45 seconds.

Observed:

- Ground `RADIO_STATUS` continued at 2 Hz.
- No INAV `HEARTBEAT`.
- No other non-local MAVLink frame.

### Secondary-port-aware test

Because INAV ports 2 through 4 default to heartbeat-only, the test explicitly sent:

- GCS `HEARTBEAT`
- `PING`
- `TIMESYNC`
- `REQUEST_DATA_STREAM` at 1 Hz for:
  - `EXTENDED_STATUS`
  - `POSITION`
  - `EXTRA1`

Observed over 55 seconds:

```text
RADIO_STATUS: 110
Non-local messages: 0
```

No heartbeat, PING response, TIMESYNC response, or requested stream data arrived.

### Air mesh liveness during FC tests

The pc2 ground node continued hearing the air WSL V3 at approximately:

```text
SNR: 10 dB
Hops: 0
```

This proved the air MCU and LoRa radio were not hung despite ambiguous LED behavior.

The steady white LED was therefore not evidence of an MCU crash.

## Manual channel and LED observation

The user sent a normal manual chat message on the channel named `serial` from the ground node using the Meshtastic app.

The air node’s light reacted.

This proves:

- The channel itself existed on both nodes.
- A normal Meshtastic packet on that channel reached the air radio.

It does not prove:

- `SERIAL_APP` bridge chunks were sent.
- `SERIAL_APP` packets were dispatched into `MavlinkBridge`.
- UART bytes moved on GPIO47/48.

Normal chat traffic and raw MAVLink `SERIAL_APP` traffic use different port numbers.

## GPIO47/GPIO48 loopback test

To remove INAV and wiring from the test, the FC was disconnected and the user jumpered:

```text
GPIO48 TX → GPIO47 RX
```

Expected route:

```text
PC fake MAVLink
→ pc2 UDP 14550
→ ground bridge
→ LoRa serial channel
→ air bridge
→ GPIO48 TX
→ jumper
→ GPIO47 RX
→ air bridge
→ LoRa serial channel
→ ground bridge
→ PC UDP
```

The fake source used:

```text
system 255
component 190
```

It sent:

- GCS heartbeat
- PING with unique sequence numbers

Fifteen probe pairs were sent over 60 seconds.

Observed:

```text
Ground RADIO_STATUS: 118
Returned heartbeat/PING: 0
Loopback success: false
```

This is the most important current result:

- The failure reproduces without INAV.
- The failure reproduces with direct GPIO47/GPIO48 loopback.
- Therefore the FC and INAV cannot be the only cause.

## USB log observations during loopback

The air node USB API and serial log were monitored while ten more GCS heartbeat/PING pairs were sent.

The air node showed normal:

- Device metadata
- Channel configuration
- Mesh telemetry
- Current node uptime

It did not show the expected bridge events:

```text
MAVLink bridge locked to peer ...
MAVLink autopilot learned ...
MAVLink input FIFO overflow ...
MAVLink output FIFO overflow ...
```

The air node did receive ordinary LoRa telemetry traffic.

The absence of `locked to peer` strongly suggests that raw ground `SERIAL_APP` chunks were not reaching `MavlinkBridge::ingestMeshPayload()` during this test. Logging capture limitations should still be considered, but the failed loopback independently confirms no completed data path.

## Source inspection at the final stopping point

The following bridge code paths were inspected:

### UDP ingress

`MavlinkUdpServer::poll()`:

- Binds UDP 14550 when the network is up.
- Learns the latest client endpoint.
- Reads complete UDP datagrams.
- Calls:

```cpp
mavlinkBridge->ingestSerialBytes(buf, n, now);
```

The returned local `RADIO_STATUS` proves this path is active.

### Bridge input FIFO

`MavlinkBridge::ingestSerialBytes()`:

- Marks the bridge active.
- Snoops local MAVLink.
- Pushes bytes into the input FIFO.

`wantsMeshSend()` becomes true when:

- FIFO reaches the maximum chunk size, or
- The flush interval expires with any queued bytes.

### Mesh transmission

`SerialModuleRadio::sendMavlinkChunk()`:

- Checks bridge FIFO.
- Checks channel utilization.
- Checks router queue space.
- Allocates a `SERIAL_APP` packet.
- Broadcasts before peer discovery.
- Uses the channel literally named `serial`.
- Commits FIFO bytes only after `sendToMesh()` succeeds.

### Mesh receive

`SerialModuleRadio::handleReceived()` in mode 11 calls:

```cpp
mavlinkBridge->ingestMeshPayload(...)
```

for packets not originating from the same node.

The first received non-empty chunk should call:

```cpp
MavlinkBridge::acceptSource()
```

and log:

```text
MAVLink bridge locked to peer 0x...
```

That log was not observed.

### UART selection

On ESP32/ESP32-S3 with explicit RX/TX pins, the serial module uses:

```cpp
Serial2.begin(baud, SERIAL_8N1, rxd, txd);
```

The MAVLink bridge uses `Serial2`.

Therefore GPIO47/48 should be independent of the CP2102 USB console and suitable for simultaneous USB debugging.

### Flow-control behavior

`RADIO_STATUS` is not emitted on a completely idle bridge.

`serviceFlowControl()` requires:

- `everActive == true`
- Recent activity within the activity window

Therefore receiving no `RADIO_STATUS` directly from an air node with no FC attached was expected and was not proof of failure.

## Current leading fault boundary

Proven working:

- Both firmware images build.
- Both nodes boot.
- Both Meshtastic APIs work when UART0 is not reassigned.
- Both LoRa radios communicate in both directions using ordinary channel traffic.
- Primary mesh settings match.
- Secondary `serial` channel names and keys match.
- Ground Wi-Fi and UDP 14550 work.
- Ground UDP ingress activates `MavlinkBridge` and local `RADIO_STATUS`.
- Air RX/TX/mode/baud settings persist correctly.
- Air node remains alive and visible over LoRa.

Not proven working:

- Ground `MavlinkBridge` input FIFO becoming a transmitted `SERIAL_APP` mesh packet.
- Air `SerialModuleRadio` receiving and dispatching that raw chunk.
- Air peer lock.
- GPIO48 → GPIO47 local loopback through the bridge.
- Any INAV MAVLink frame reaching the ground GCS.

The next investigation should begin at ground `SerialModuleRadio::sendMavlinkChunk()` and packet dispatch, not at INAV.

## Recommended next test with minimal scope

Add temporary debug logs containing relevant variables only:

1. In `MavlinkUdpServer::poll()`:
   - received datagram byte count
2. In `MavlinkBridge::ingestSerialBytes()`:
   - input FIFO size after push
3. In `SerialModuleRadio::sendMavlinkChunk()`:
   - payload length
   - selected channel index/name
   - destination node
   - `sendToMesh()` result
4. In `SerialModuleRadio::handleReceived()`:
   - packet source
   - port number
   - channel index/name
   - payload size
5. In `MavlinkBridge::ingestMeshPayload()`:
   - source
   - accepted payload length
   - locked peer

Then repeat only the GPIO47/GPIO48 loopback test. Do not involve INAV until the fake heartbeat returns.

Remove or reduce temporary logs after the fault is found.

## Useful command patterns

### Read serial config

```bash
meshtastic \
  --port /dev/ttyUSB0 \
  --get serial.enabled \
  --get serial.mode \
  --get serial.rxd \
  --get serial.txd \
  --get serial.baud
```

### Configure air

```bash
meshtastic \
  --port /dev/ttyUSB0 \
  --set lora.region 1 \
  --set lora.modem_preset 4 \
  --set serial.rxd 47 \
  --set serial.txd 48 \
  --set serial.baud 10 \
  --set serial.mode 11 \
  --set serial.enabled true
```

### Configure UDP-only ground

```bash
meshtastic \
  --port /dev/ttyUSB0 \
  --set serial.enabled true \
  --set serial.mode 11
```

Leave RX and TX zero on ground.

### Verify the ground node

```bash
ssh pc2.lan \
  'meshtastic --port /dev/ttyUSB0 --get serial.enabled --get serial.mode'
```

### Pymavlink ground endpoint

```python
from pymavlink import mavutil

m = mavutil.mavlink_connection(
    "udpout:192.168.0.244:14550",
    source_system=255,
    source_component=190,
)
```

The GCS must send the first datagram before replies can arrive.

## Security and privacy notes

- Wi-Fi credentials were visible in one verbose CLI output during the session but are not recorded here.
- Channel PSKs and channel URLs are not recorded here.
- Only non-reversible SHA-256 fingerprints are retained for comparison.
- A full preferences erase rotates the node PKI keypair and invalidates cached peer keys.

## Git state before adding this file

Immediately before creating `NOTES.md`:

- Working tree was clean.
- Latest commits were:

```text
ce813f2a6 Enable MAVLink on original T-LoRa V1
98a34357e Fix MAVLink SerialModule preprocessor guard
```

`NOTES.md` itself is new and uncommitted unless a later commit explicitly includes it.
