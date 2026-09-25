#pragma once

#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_MAVLINK

#include "mesh/MeshTypes.h"
#include <Arduino.h>

#if HAS_NETWORKING && defined(ARCH_ESP32)
#define MESHTASTIC_MAVLINK_UDP 1
#else
#define MESHTASTIC_MAVLINK_UDP 0
#endif

#ifndef MAVLINK_COMM_NUM_BUFFERS
#define MAVLINK_COMM_NUM_BUFFERS 1
#endif
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "common/mavlink.h"
#pragma GCC diagnostic pop

#include "MavlinkMeshTransport.h"

struct MavlinkBridgeStats {
    uint32_t uartRxBytes = 0;
    uint32_t meshTxBytes = 0;
    uint32_t meshRxBytes = 0;
    uint32_t framesToMesh = 0;
    uint32_t framesFromMesh = 0;
    uint32_t framesToUart = 0;
    uint32_t corruptFrames = 0;
    uint32_t framingErrors = 0;
    uint32_t inputOverflowBytes = 0;
    uint32_t outputOverflowBytes = 0;
    uint32_t rejectedSourceChunks = 0;
    uint32_t malformedMeshPayloads = 0;
    uint32_t duplicateFragments = 0;
    uint32_t reassemblyTimeouts = 0;
    uint32_t reassemblyEvictions = 0;
    uint32_t uartTxStallDrops = 0;
    uint32_t radioStatusSent = 0;
    uint32_t commandAckLocalIngress = 0;
    uint32_t commandAckFramesQueued = 0;
    uint32_t commandAckFramesSent = 0;
    uint32_t commandAckOutboundDrops = 0;
    uint32_t commandAckFramesReassembled = 0;
    uint32_t commandAckInboundDrops = 0;
    uint32_t commandAckLocalDelivery = 0;
    uint32_t meshPacketsAcked = 0;       // routing ACK from the destination node
    uint32_t meshPacketsImplicitAck = 0; // relay heard retransmitting; not proof of delivery
    uint32_t meshPacketsFailed = 0;      // routing NAK, including retries exhausted
    uint32_t meshPacketsNoResult = 0;    // no routing result before the in-flight timeout
    uint32_t inFlightOverflow = 0;       // acknowledged packet submitted with every record busy
    uint32_t lastAckLatencyMs = 0;
    uint32_t maxAckLatencyMs = 0;
    size_t inputHighWater = 0;
    size_t outputHighWater = 0;
};

#ifndef MESHTASTIC_MAVLINK_RADIO_STATUS_SYSID
#define MESHTASTIC_MAVLINK_RADIO_STATUS_SYSID 255
#endif

enum class MavlinkRole : uint8_t { UNKNOWN, AIR, GROUND };

struct MavlinkBatterySnapshot {
    bool hasLevel = false;
    uint8_t level = 0;
    bool hasVoltage = false;
    float voltage = 0;
};

struct MavlinkPositionSnapshot {
    bool hasFix = false;
    int32_t latI = 0;
    int32_t lonI = 0;
    int32_t altM = 0;
    bool hasGroundSpeed = false;
    uint32_t groundSpeedKmh = 0;
    bool hasGroundTrack = false;
    uint32_t groundTrack1e5 = 0;
    uint32_t fixType = 0;
    bool hasSats = false;
    uint32_t sats = 0;
};

#ifndef MESHTASTIC_MAVLINK_ROLE
#define MESHTASTIC_MAVLINK_ROLE 0
#endif

class NullStream : public Stream
{
  public:
    int available() override { return 0; }
    int read() override { return -1; }
    int peek() override { return -1; }
    size_t write(uint8_t) override { return 1; }
    size_t write(const uint8_t *, size_t n) override { return n; }
    int availableForWrite() override { return 1024; }
    void flush() override {}
};

/**
 * Symmetric MAVLink endpoint router over Meshtastic.
 *
 * Each local UART or UDP byte stream is parsed into complete raw MAVLink wire frames.
 * Frames are encapsulated whole, or explicitly fragmented when required. Receive-side
 * reassembly is keyed by Meshtastic source node and frame id, so independent endpoints
 * may share the mesh without their byte streams being interleaved.
 *
 * Routes (MAVLink sysid -> Meshtastic node) are learned from inbound frames; frames are
 * broadcast only until a route exists. There are no fixed ground, air, or peer roles.
 * Acknowledged classes go unicast with want_ack so Meshtastic's own retry carries them;
 * the routing result is tracked here only for instrumentation and flow control. Routes
 * expire through silence, not through a missing ACK. MAVLink's own transactions remain the
 * end-to-end receipt.
 */
class MavlinkBridge
{
  public:
    static constexpr size_t MAX_CHUNK = MavlinkMeshTransport::MAX_MESH_PAYLOAD;
    static constexpr size_t TRANSPORT_HEADER_SIZE = MavlinkMeshTransport::HEADER_SIZE;
    static constexpr size_t MAX_FRAGMENT_DATA = MavlinkMeshTransport::MAX_FRAGMENT_DATA;
    static constexpr uint32_t UART_TX_STALL_MS = 500;
    static constexpr uint32_t RADIO_STATUS_INTERVAL_MS = 500;
    // Hold briefly before transmitting so several frames can share one packet. Without this the
    // send is issued the moment a frame is queued, the aggregate container finds exactly one
    // frame waiting, and packing never engages. Sending starts early once a full pack is
    // available, so the wait only applies while the payload is still filling.
    static constexpr uint32_t COALESCE_WINDOW_MS = 50;
    static constexpr uint32_t ACTIVITY_WINDOW_MS = 10000;
    static constexpr uint32_t POSITION_THROTTLE_MS = 1000;
    static constexpr uint32_t GLOBAL_POS_FRESH_MS = 5000;
    static constexpr uint32_t BATTERY_STALENESS_MS = 120000;
    static constexpr uint32_t SYS_STATUS_DEFER_MS = 10000;
    static constexpr size_t IN_FLIGHT_SLOTS = 8;
    // Well beyond ReliableRouter's own retry window, which ends in a local MAX_RETRANSMIT NAK.
    static constexpr uint32_t IN_FLIGHT_TIMEOUT_MS = 60000;
    static constexpr uint32_t STATS_LOG_INTERVAL_MS = 30000;
    static constexpr uint32_t OVERFLOW_LOG_INTERVAL_MS = 1000;
    // Hand Meshtastic at most this many of our packets at a time. Frames waiting beyond that stay
    // in our queue, where priority and state replacement still apply; in the router's TX queue
    // stale state would just wait its turn.
    static constexpr size_t MAX_ROUTER_QUEUED = 2;
    static constexpr size_t RECENT_PACKET_IDS = 8;

    explicit MavlinkBridge(Stream *uart);

    void ingestSerialBytes(const uint8_t *data, size_t len, uint32_t now);
    bool wantsMeshSend(uint32_t now) const;
    /// Build the next packet payload. How to send it is then available from the planned*()
    /// accessors until commitMeshPayload().
    size_t peekMeshPayload(uint8_t *out, size_t capacity, uint32_t now);
    NodeNum plannedDestination() const { return transport.plannedDestination(); }
    bool plannedWantAck() const { return transport.plannedWantAck(); }
    int8_t plannedHopLimit() const { return transport.plannedHopLimit(); }
    void commitMeshPayload(size_t len, uint32_t now);

    /// Record a packet the mesh accepted, so its routing result can be matched by packet id.
    void noteSubmitted(PacketId id, NodeNum dest, bool wantAck, size_t len, uint32_t now);
    /// True while MAX_ROUTER_QUEUED of our recent packets (retransmissions included) still wait in Meshtastic's TX queue.
    bool routerBacklogged() const { return routerQueuedCount() >= MAX_ROUTER_QUEUED; }
    /// Routing ACK/NAK addressed to us; `from` is the node that produced it.
    void handleRoutingResult(PacketId requestId, NodeNum from, meshtastic_Routing_Error error, uint32_t now);

    void ingestMeshPayload(NodeNum source, const uint8_t *data, size_t len, int32_t rxRssi = 0, float rxSnr = 0,
                           int8_t hopsAway = -1);
    void processOutput(uint32_t now);
    void serviceFlowControl(uint32_t now);
    /// Expire unanswered in-flight records and periodically log counters while the link is active.
    void serviceHousekeeping(uint32_t now);

    const MavlinkBridgeStats &getStats();
    MavlinkRole getRole() const { return role; }
    bool takePositionSnapshot(uint32_t now, MavlinkPositionSnapshot &out);
    bool getBatterySnapshot(uint32_t now, MavlinkBatterySnapshot &out) const;

  private:
    void queueFrame(const mavlink_message_t &msg, uint32_t now);
    void flushPending(uint32_t now);
    void restartCoalesceWindow(uint32_t now);

    void snoopSerialBytes(const uint8_t *data, size_t len, uint32_t now);
    void handleSnoopedMessage(const mavlink_message_t &msg, uint32_t now);
    void snoopHeartbeat(const mavlink_message_t &msg);
    void snoopPosition(const mavlink_message_t &msg, uint32_t now);
    void snoopBattery(const mavlink_message_t &msg, uint32_t now);
    void snoopHighLatency2(const mavlink_message_t &msg, uint32_t now);
    bool fromAutopilot(const mavlink_message_t &msg) const;

    struct InFlight {
        bool used = false;
        PacketId id = 0;
        NodeNum dest = 0;
        uint16_t bytes = 0;
        uint32_t submittedMs = 0;
    };

    size_t inFlightCount() const;
    size_t routerQueuedCount() const;
    void logStats(uint32_t now);
    void logQueueOverflow(uint32_t now);

    MavlinkMeshTransport transport;
    InFlight inFlight[IN_FLIGHT_SLOTS];
    PacketId recentIds[RECENT_PACKET_IDS] = {};
    size_t recentIdNext = 0;
    uint32_t lastStatsLogMs = 0;
    uint32_t lastOverflowLogMs = 0;
    uint32_t loggedDropped = 0;
    uint32_t loggedEvicted = 0;

    uint8_t pendingFrame[MAVLINK_MAX_PACKET_LEN];
    size_t pendingLen = 0;
    size_t pendingOff = 0;
    uint32_t pendingSinceMs = 0;

    uint32_t outboundSinceMs = 0;
    bool outboundPending = false;

    uint32_t lastRadioStatusMs = 0;
    uint32_t lastActivityMs = 0;
    bool everActive = false;
    int32_t lastRxRssi = 0;
    float lastRxSnr = 0;
    bool haveRxMetadata = false;

    mavlink_message_t snoopWorking{};
    mavlink_status_t snoopStatus{};
    MavlinkRole role = (MavlinkRole)MESHTASTIC_MAVLINK_ROLE;
    bool haveAutopilot = false;
    uint8_t autopilotSysid = 0;
    uint8_t autopilotCompid = 0;
    MavlinkPositionSnapshot posSnap;
    bool positionDirty = false;
    uint32_t lastGlobalPosMs = 0;
    uint32_t lastPositionTakeMs = 0;
    MavlinkBatterySnapshot battSnap;
    uint32_t lastBatteryLevelMs = 0;
    uint32_t lastBatteryVoltageMs = 0;
    uint32_t lastBatteryStatusMs = 0;
    bool haveBatteryId = false;
    uint8_t batteryId = 0;

    Stream *uart;
    MavlinkBridgeStats stats;
};

extern MavlinkBridge *mavlinkBridge;

#endif
