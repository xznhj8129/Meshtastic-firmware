MAVLink Serial Bridge for Meshtastic

Status: Draft design
Scope: Firmware implementation plan
Primary reference: ExpressLRS MAVLink serial transport

1. Goal

Add MAVLink awareness to Meshtastic's serial subsystem so two Meshtastic nodes can bridge a MAVLink serial connection across the mesh.

Typical arrangement:

- Air node: connected by UART to a flight controller or other onboard MAVLink network.
- Ground node: connected by UART or USB serial to a ground control station.
- Mesh: transports the MAVLink byte stream between the two nodes.

The bridge must:

1. Transport MAVLink 1 and MAVLink 2 traffic in both directions.
2. Pass standard, custom-dialect, and unknown MAVLink messages without filtering.
3. Preserve MAVLink framing and checksums.
4. Allow the GCS and autopilot to control message rates normally.
5. Expose useful onboard MAVLink telemetry, especially position and battery state, to Meshtastic itself.
6. Use software flow control based on bridge buffer occupancy, following the ExpressLRS MAVLink implementation.

This is a low-throughput MAVLink link. Its usefulness depends on configuring suitable MAVLink message rates at the GCS or autopilot.

2. Core design principles

2.1 The bridge transports MAVLink; it does not redefine it

The bridge does not maintain a message whitelist and does not decide which MAVLink services are permitted.

Every MAVLink frame that can traverse the configured link is eligible for transport, including:

- Standard common-dialect messages
- ArduPilot, PX4, INAV, and vendor-specific messages
- Custom dialect messages
- Commands and acknowledgements
- Mission and parameter protocol traffic
- Camera, gimbal, payload, and component traffic
- Signed MAVLink 2 frames

The bridge may decode selected known messages for Meshtastic's own local use, but decoding is not a condition for forwarding.

2.2 Message rates belong to the MAVLink endpoints

The GCS, autopilot, or user configuration determines the stream rates.

The Meshtastic bridge does not:

- Maintain per-message rate limits
- Send unsolicited "MAV_CMD_SET_MESSAGE_INTERVAL" commands
- Create telemetry profiles
- Suppress messages by ID
- Implement special mission or parameter policies

If the selected MAVLink stream exceeds the available mesh capacity, the bridge reports congestion through "RADIO_STATUS.txbuf", records overflow, and drops excess input only when its bounded buffer is exhausted.

2.3 Follow the ExpressLRS transport model

The implementation should adapt the proven ExpressLRS MAVLink serial design:

1. Read UART bytes into an input FIFO.
2. Remove as many bytes as fit in the next mesh payload.
3. Send those bytes as one Meshtastic packet.
4. Append received mesh payload bytes to an output FIFO.
5. Feed output bytes through the generated MAVLink C parser.
6. When a complete valid MAVLink frame is parsed, serialize it and write it to the UART.
7. Generate local "RADIO_STATUS" messages with "txbuf" derived from input FIFO free space.

The mesh transport carries raw MAVLink byte chunks. Meshtastic packet boundaries do not need to match MAVLink frame boundaries.

3. Non-goals

The first implementation does not provide:

- RC control
- Real-time manual flight control
- Video transport
- A new reliable transaction layer above Meshtastic
- A custom MAVLink fragmentation protocol
- Message whitelisting
- Message-specific transport queues
- Mission-specific bridge logic
- Parameter-specific bridge logic
- Automatic message-rate configuration
- A separate battery model for the Meshtastic node
- A new pairing protocol independent of Meshtastic node addressing

The bridge is only as fast and reliable as the configured Meshtastic link.

4. Integration with Meshtastic

4.1 Serial mode

Add a new serial mode:

MAVLINK = 11;

The existing "SerialModule" continues to own:

- UART selection
- RX and TX pins
- Baud rate
- Serial initialization
- Serial module enable state
- Mesh channel binding

When the configured mode is "MAVLINK", "SerialModule" delegates byte handling to "MavlinkBridge".

4.2 Mesh port number

The first implementation should reuse "SERIAL_APP".

Both endpoints are explicitly configured for MAVLink serial mode, so the serial application port already identifies the relevant traffic and avoids an unnecessary protobuf and dispatch layer.

A dedicated "MAVLINK_APP" port can be added later if a concrete requirement appears, such as:

- Client applications needing to identify MAVLink traffic separately
- Simultaneous independent serial applications
- Different routing or forwarding rules for MAVLink traffic

It is not required for the bridge itself.

4.3 Radio handler

A separate "MavlinkRadio" module is not required.

"SerialModuleRadio" can dispatch received "SERIAL_APP" payloads to "MavlinkBridge" when serial mode is "MAVLINK".

Likewise, "MavlinkBridge" can ask the existing serial radio object to transmit raw chunks.

4.4 UDP server endpoint

When networking is enabled and up, either Ethernet or WiFi, the device opens a UDP server on port 14550 that serves as an additional local MAVLink connection point alongside the UART.

Typical arrangement: the ground node is a Meshtastic stick with no wiring; the GCS reaches it over the LAN.

The scheme is "wait for client", not "connect to provided client address":

1. The device binds UDP port 14550 and waits.
2. The first datagram received registers the client (source IP and port).
3. Every later datagram refreshes the registration; a new sender re-points the registration. One client at a time; latest sender wins.
4. The client registration expires after 30 s of silence, after which egress to UDP stops.

Transport rules:

- Ingress: datagram payload bytes enter the same bridge input FIFO as UART bytes. Mesh transport is unchanged; datagram boundaries do not need to match MAVLink frame boundaries.
- Egress: every completed MAVLink frame written to the local UART is also sent to the registered client as one UDP datagram (one frame per datagram). Locally generated RADIO_STATUS frames are included.
- Both endpoints are active simultaneously. When no UART RX/TX pins are configured, the device runs as a UDP-only endpoint: frame bytes are discarded to a null stream and the UDP client is the only local consumer.
- Snooping applies to UDP-ingress bytes exactly as to UART bytes; a GCS heartbeat arriving over UDP correctly marks the node GROUND.

Implementation notes:

- On ESP32, WiFiUDP is lwIP-based and works over WiFi STA, WiFi AP, and ESP-IDF Ethernet (USE_WS5500, CH390) alike. Network-up detection mirrors MQTT's isConnectedToNetwork(), extended to accept WiFi AP mode.
- Non-ESP32 networking targets are out of scope for the first implementation.
- The GCS must actively send to be discovered. QGroundControl: add the device IP as a server address on the UDP link. MAVProxy: --master=udp:<device-ip>:14550.
- The socket lifecycle is managed lazily from the main scheduler: opened when the network is up, closed when it goes down. No ISR or task touches the socket.

Non-goals for this iteration: TCP transport, multiple simultaneous clients, broadcast or multicast discovery beacons, and any client keepalive beyond the receive timeout.

5. MAVLink implementation

5.1 Generated C headers

Include the generated MAVLink C headers, initially using the common dialect:

#define MAVLINK_COMM_NUM_BUFFERS 1
#include "common/mavlink.h"

Use the normal generated API:

- "mavlink_frame_char()"
- "mavlink_msg_to_send_buffer()"
- "mavlink_msg_heartbeat_decode()"
- "mavlink_msg_global_position_int_decode()"
- "mavlink_msg_gps_raw_int_decode()"
- "mavlink_msg_sys_status_decode()"
- "mavlink_msg_battery_status_decode()"
- "mavlink_msg_radio_status_encode()"

This provides correct MAVLink 1 and MAVLink 2 parsing, checksum handling, signatures, field layout, and serialization.

No hand-maintained message offsets or CRC tables are required.

5.2 Parser behavior

The receiving side feeds bytes one at a time to "mavlink_frame_char()".

When it returns "MAVLINK_FRAMING_OK":

1. The complete "mavlink_message_t" is available.
2. Selected messages may be decoded for local Meshtastic state.
3. The message is serialized using "mavlink_msg_to_send_buffer()".
4. The resulting bytes are written to the local UART.

If a mesh packet is lost, the parser discards the incomplete frame and resynchronizes when a later valid MAVLink frame is encountered.

No separate fragment reassembly is needed.

5.3 Signed MAVLink

Signed MAVLink 2 frames are handled by the generated parser and serializer.

The bridge does not inspect, alter, or re-sign transported messages.

Locally generated messages, such as "RADIO_STATUS", are separate MAVLink frames produced by the bridge. Sites requiring strict signed-only local UART traffic may disable local message injection.

6. Data transport

6.1 UART to mesh

UART bytes are pushed into a bounded input FIFO.

When the radio side is ready to send:

1. Determine the maximum payload available in a Meshtastic "SERIAL_APP" packet.
2. Remove up to that many bytes from the input FIFO.
3. Place those bytes directly in the mesh payload.
4. Send to the configured destination or according to the active routing policy.

No MAVLink frame parsing is required before transmission.

A frame may be split across multiple mesh packets, and several small frames may share one mesh packet.

6.2 Mesh to UART

Bytes from each received mesh payload are appended to a bounded output FIFO.

The bridge drains the output FIFO through "mavlink_frame_char()".

Only complete valid MAVLink frames are written to UART. This prevents a lost or reordered mesh packet from writing arbitrary partial MAVLink data into the local endpoint.

6.3 FIFO sizes

Initial sizes should follow the same general scale as ExpressLRS and be adjusted through hardware testing:

static constexpr size_t MAVLINK_INPUT_FIFO_SIZE = 1024;
static constexpr size_t MAVLINK_OUTPUT_FIFO_SIZE = 512;

The exact values should be compile-time constants and may vary by target class.

Required behavior:

- Never allocate based on untrusted packet length.
- Never block the serial thread waiting for radio capacity.
- Count dropped input and output bytes.
- Expose FIFO high-water marks through logs or debug telemetry.
- Use locking or atomic FIFO operations where serial and radio callbacks may run concurrently.

6.4 Overflow

If a FIFO is full:

- Reject bytes that do not fit.
- Increment an overflow counter.
- Log a rate-limited warning.
- Lower "RADIO_STATUS.txbuf" as the input FIFO fills.

The bridge does not attempt to inspect message IDs and choose which messages to discard.

The operational correction for persistent overflow is to reduce MAVLink message rates or select a faster Meshtastic radio configuration.

7. Flow control

7.1 "RADIO_STATUS"

Following ExpressLRS, the bridge periodically emits a local "RADIO_STATUS" message onto the attached UART.

The most important field is:

txbuf = percentage of free space remaining in the UART-to-mesh input FIFO

Example:

const uint8_t txbuf =
    ((MAVLINK_INPUT_FIFO_SIZE - inputFifo.size()) * 100U) /
    MAVLINK_INPUT_FIFO_SIZE;

The bridge uses:

- A configurable source system ID, defaulting to a conventional GCS or radio value.
- "MAV_COMP_ID_TELEMETRY_RADIO" as the source component.
- Mesh-derived RSSI, SNR, or link-quality fields where reasonable.
- Zero or unknown values where no meaningful mapping exists.

The update rate does not need to match ExpressLRS's 100 Hz. Meshtastic queue occupancy changes much more slowly. An initial rate around 2 to 10 Hz is sufficient and should be validated in testing.

7.2 No automatic stream control

The bridge does not issue rate-setting commands.

The GCS or autopilot remains responsible for:

- "MAV_CMD_SET_MESSAGE_INTERVAL"
- "REQUEST_DATA_STREAM", where still applicable
- Autopilot serial stream-rate parameters
- Disabling unneeded telemetry
- Selecting suitable mission or parameter operations for the available link

8. Local Meshtastic telemetry snooping

The bridge parses incoming UART MAVLink frames for transport reconstruction and may also decode selected messages to update the Meshtastic node's local state.

This is passive observation. It does not alter forwarding.

8.1 Position

Primary source:

- "GLOBAL_POSITION_INT"

Fallback source:

- "GPS_RAW_INT"

The bridge updates Meshtastic's local position state with:

- Latitude
- Longitude
- Altitude
- Ground speed, when available
- Heading, when available
- Fix type and satellites, when available

The existing Meshtastic "PositionModule" remains responsible for:

- Position broadcast timing
- Smart position scheduling
- Channel-utilization limits
- Precision reduction
- Stationary behavior
- Mesh packet creation

The bridge must not transmit an extra position packet for every MAVLink position frame.

8.2 Battery

The onboard Meshtastic node and aircraft use the same electrical system. MAVLink flight-battery data is therefore the authoritative battery data for the Meshtastic node.

Sources:

- "BATTERY_STATUS"
- "SYS_STATUS" as a fallback

Update:

- "DeviceMetrics.battery_level"
- "DeviceMetrics.voltage"

Local Meshtastic values such as uptime, channel utilization, and transmitted airtime remain locally generated.

If a future installation has a separate radio battery, that can be handled as an optional extension. It is not part of the baseline design.

8.3 Heartbeat

Decode "HEARTBEAT" to learn:

- Source system ID
- Source component ID
- MAVLink type
- Autopilot type
- Armed state
- Flight mode, where useful

The bridge may use heartbeat information to determine whether the local UART is attached to an aircraft or GCS for snooping purposes.

Transport behavior remains identical in either case.

8.4 No duplicate converted traffic

Do not convert the following into separate Meshtastic packets by default:

- "STATUSTEXT"
- Flight mode
- Armed state
- Commands
- Command acknowledgements

Those messages already cross the bridge as MAVLink.

Local display or logging support may consume them without creating extra radio traffic.

9. Role and system discovery

9.1 Role

Role is not part of the transport path.

The bridge can maintain a small local role state:

- "UNKNOWN"
- "AIR"
- "GROUND"

Suggested automatic detection:

- A heartbeat with "type == MAV_TYPE_GCS" indicates ground.
- A heartbeat with a valid autopilot and vehicle type indicates air.

A configuration override may be provided:

- "AUTO"
- "FORCE_AIR"
- "FORCE_GROUND"

This setting affects local snooping only.

9.2 MAVLink system to Meshtastic node association

For multi-vehicle use, the ground node can learn:

MAVLink system ID -> Meshtastic source node ID

The mapping is learned from valid MAVLink frames received from each mesh node, especially heartbeats.

Important MAVLink addressing rule:

- Header "sysid" identifies the source system.
- Broadcast targeting is represented by "target_system == 0" in messages that contain a target field.
- "sysid == 0" is not used as a normal broadcast source identity.

The first implementation may use one configured Meshtastic destination node. Dynamic multi-vehicle routing can be added after the one-to-one bridge is stable.

No separate pairing protocol is needed beyond Meshtastic channels, node IDs, and direct messaging.

10. Meshtastic packet behavior

10.1 Destination

For the initial implementation, support a configured destination node ID.

Recommended default:

- Direct message to the peer node
- Bound to the configured serial channel
- Encryption and authentication provided by Meshtastic

Broadcast may be useful for passive telemetry observation, but command-capable operation should normally use direct addressing.

10.2 Acknowledgements

Use the existing "SerialModuleRadio" packet behavior unless testing shows it unsuitable.

Meshtastic acknowledgements, duplicate handling, routing, and encryption remain transport-layer concerns below MAVLink.

The MAVLink bridge does not add its own acknowledgement or retransmission protocol.

10.3 Ordering and loss

The bridge assumes Meshtastic normally delivers packets from a peer in usable order.

When bytes are lost or reordered:

- The current MAVLink frame may fail parsing.
- "mavlink_frame_char()" resynchronizes on a later valid frame.
- MAVLink services such as commands, missions, and parameters retain their own application-level request, acknowledgement, and retry behavior.

No custom fragment protocol is added.

11. Proposed class structure

11.1 "MavlinkBridge"

Responsibilities:

- Own input and output FIFOs
- Accept UART bytes
- Produce mesh payload chunks
- Accept mesh payload chunks
- Parse received MAVLink
- Write completed frames to UART
- Generate "RADIO_STATUS"
- Decode local position, battery, and heartbeat information
- Track counters and high-water marks
- Optionally learn MAVLink system-to-node associations

Suggested interface:

class MavlinkBridge
{
  public:
    explicit MavlinkBridge(Stream &serial);

    void ingestSerialBytes(const uint8_t *data, size_t length);
    size_t buildMeshPayload(uint8_t *output, size_t capacity);

    void ingestMeshPayload(
        NodeNum source,
        const uint8_t *data,
        size_t length);

    void processOutput();
    void serviceFlowControl();

  private:
    void handleParsedMessage(
        NodeNum source,
        const mavlink_message_t &message);

    void updatePosition(const mavlink_message_t &message);
    void updateBattery(const mavlink_message_t &message);
    void updateHeartbeat(
        NodeNum source,
        const mavlink_message_t &message);

    FIFO<MAVLINK_INPUT_FIFO_SIZE> inputFifo;
    FIFO<MAVLINK_OUTPUT_FIFO_SIZE> outputFifo;

    mavlink_status_t parserStatus {};
    Stream &serial;
};

The exact FIFO and locking types should follow Meshtastic's existing conventions.

11.2 "SerialModule"

Changes:

- Construct "MavlinkBridge" when mode is "MAVLINK".
- Read available UART bytes without waiting for a timeout-delimited packet.
- Push them into the bridge input FIFO.
- Ask the bridge for outgoing mesh chunks.
- Forward incoming "SERIAL_APP" payloads to the bridge.
- Periodically call output parsing and flow-control service functions.

The existing timeout-oriented "readBytes()" behavior used by generic serial mode should not define MAVLink packet boundaries.

11.3 "SerialModuleRadio"

Changes:

- When serial mode is "MAVLINK", send the payload supplied by "MavlinkBridge".
- On receive, pass raw decoded payload bytes and source node ID to "MavlinkBridge".
- Preserve existing serial channel binding and destination handling.

12. Files to add or modify

protobufs repository:
  meshtastic/module_config.proto
    add Serial_Mode.MAVLINK = 11

firmware repository:
  src/modules/Mavlink/MavlinkBridge.h
  src/modules/Mavlink/MavlinkBridge.cpp
  src/modules/Mavlink/MavlinkUdpServer.h
  src/modules/Mavlink/MavlinkUdpServer.cpp
  src/modules/SerialModule.h
  src/modules/SerialModule.cpp
  src/modules/SerialModuleRadio implementation, if separated
  src/modules/Modules.cpp, only if new construction or build gating is required
  test/test_mavlink/test_main.cpp

No custom message-definition header is required.

No dedicated tunnel-framing class is required.

No forward-policy class is required.

No mission or parameter handler is required.

13. Build configuration

Gate the feature with:

MESHTASTIC_EXCLUDE_MAVLINK

The MAVLink bridge also depends on the existing serial module.

Targets with tight flash or RAM budgets may exclude it by default.

The build must include:

- Generated MAVLink common-dialect headers
- MAVLink parser state
- FIFO storage
- Local snooping code

Actual linked flash and RAM usage should be measured before deciding which targets include it by default.

14. Testing

14.1 Native unit tests

Test the parser and transport with generated MAVLink frames.

Required cases:

- MAVLink 1 frame
- MAVLink 2 frame
- Signed MAVLink 2 frame
- Frame split across two mesh payloads
- Frame split across several mesh payloads
- Several frames in one mesh payload
- Payload ending exactly at a frame boundary
- Payload ending in the middle of a frame
- Garbage before a valid frame
- Lost middle chunk followed by parser resynchronization
- Custom or unknown message transported without filtering
- Input FIFO overflow
- Output FIFO overflow
- "RADIO_STATUS.txbuf" at empty, half-full, and full FIFO states
- Position decoding
- Battery decoding
- Heartbeat role and sysid learning
- Millisecond timer rollover for periodic flow-control output

14.2 Hardware-in-the-loop

Use two Meshtastic nodes:

SITL or flight controller
    <-> air node UART
    <-> Meshtastic mesh
    <-> ground node UART
    <-> MAVProxy, QGroundControl, or Mission Planner

Verify:

1. Heartbeats arrive at the GCS.
2. GCS message-rate commands reach the autopilot.
3. Reduced telemetry rates prevent persistent FIFO overflow.
4. Commands and acknowledgements work.
5. Parameter reads work at low rates.
6. Mission transfer behavior is acceptable for the configured link.
7. Unknown and custom messages pass.
8. Signed MAVLink frames remain valid.
9. Air-node position appears through normal Meshtastic position broadcasts.
10. Aircraft battery appears as the Meshtastic node battery.
11. Parser recovery occurs after intentional mesh packet loss.
12. No duplicate Meshtastic text or telemetry packets are created from MAVLink messages.

14.3 Instrumentation

Expose debug counters:

- UART bytes received
- Mesh bytes transmitted
- Mesh bytes received
- Valid MAVLink frames written to UART
- Parser framing errors
- Input FIFO overflow bytes
- Output FIFO overflow bytes
- Input FIFO high-water mark
- Output FIFO high-water mark
- Current "txbuf"
- Learned system IDs and source nodes

Counters should be available through logs first. A protobuf status interface can be added only if needed.

15. Implementation sequence

Phase 1: transparent bridge

1. Add serial mode 11.
2. Add MAVLink C headers.
3. Implement input and output FIFOs.
4. Send raw FIFO chunks through "SERIAL_APP".
5. Parse received bytes and write complete frames to UART.
6. Add counters and overflow logging.
7. Validate with SITL and MAVProxy.

Phase 2: flow control

1. Generate local "RADIO_STATUS".
2. Derive "txbuf" from input FIFO free space.
3. Add conservative RSSI and SNR mapping.
4. Verify GCS and autopilot behavior under congestion.

Phase 3: local Meshtastic state

1. Decode heartbeat.
2. Decode position.
3. Decode battery.
4. Feed existing Meshtastic local state.
5. Confirm existing Meshtastic scheduling handles broadcasts without duplication.

Phase 4: multi-vehicle routing

Only after one-to-one operation is stable:

1. Learn "sysid -> Meshtastic node ID".
2. Route targeted GCS messages according to "target_system".
3. Handle "target_system == 0" broadcasts.
4. Detect duplicate sysids.
5. Add explicit mapping configuration where automatic discovery is ambiguous.

Phase 5: UDP server endpoint

1. Open UDP 14550 when networking is up; manage the socket from the main scheduler.
2. Register the client from incoming datagrams with a 30 s timeout.
3. Feed datagram bytes into the bridge input FIFO.
4. Tee every completed local frame to the client as one datagram.
5. Allow UDP-only operation when no UART pins are configured.
6. Validate with a GCS over WiFi against a two-node bench link.

16. Open decisions

The implementation still needs concrete choices for:

1. Input and output FIFO sizes by hardware target.
2. Default peer-node configuration and whether broadcast is allowed.
3. "RADIO_STATUS" emission frequency.
4. Source system ID for locally generated "RADIO_STATUS".
5. Exact mapping of Meshtastic RSSI and SNR into MAVLink radio fields.
6. Whether local "RADIO_STATUS" injection can be disabled for signed-only installations.
7. Whether multi-vehicle routing belongs in the first release or a later phase.

These decisions do not change the transport architecture.

17. Summary

The bridge should remain small:

- Use generated MAVLink C headers.
- Carry raw MAVLink bytes inside ordinary Meshtastic serial payloads.
- Let MAVLink parsing reconstruct frames after transport.
- Let the GCS and autopilot control message rates.
- Use FIFO occupancy and "RADIO_STATUS.txbuf" for flow control.
- Forward every MAVLink message without a whitelist.
- Decode only the few messages Meshtastic needs for its own position, battery, and discovery state.
- Reuse existing Meshtastic serial transport, node addressing, channels, encryption, acknowledgements, and routing.
- Do not add a second fragmentation, transaction, filtering, or pairing system.
