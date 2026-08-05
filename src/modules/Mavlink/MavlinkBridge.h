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
 * The initial routing policy broadcasts every completed MAVLink frame. MAVLink sysid and
 * target fields remain authoritative and local MAVLink endpoints perform their normal
 * filtering. This yields any-to-any behavior without encoding permanent ground, air,
 * master, or paired-peer roles. Learned unicast can be added later as an optimization.
 */
class MavlinkBridge
{
  public:
    static constexpr size_t MAX_CHUNK = MavlinkMeshTransport::MAX_MESH_PAYLOAD;
    static constexpr size_t TRANSPORT_HEADER_SIZE = MavlinkMeshTransport::HEADER_SIZE;
    static constexpr size_t MAX_FRAGMENT_DATA = MavlinkMeshTransport::MAX_FRAGMENT_DATA;
    static constexpr uint32_t UART_TX_STALL_MS = 500;
    static constexpr uint32_t RADIO_STATUS_INTERVAL_MS = 500;
    static constexpr uint32_t ACTIVITY_WINDOW_MS = 10000;
    static constexpr uint32_t POSITION_THROTTLE_MS = 1000;
    static constexpr uint32_t GLOBAL_POS_FRESH_MS = 5000;
    static constexpr uint32_t BATTERY_STALENESS_MS = 120000;
    static constexpr uint32_t SYS_STATUS_DEFER_MS = 10000;

    explicit MavlinkBridge(Stream *uart);

    // Source-compatible constructor for older tests/callers. The peer argument is ignored.
    MavlinkBridge(Stream *uart, NodeNum legacyPeer);

    void ingestSerialBytes(const uint8_t *data, size_t len, uint32_t now);
    bool wantsMeshSend(uint32_t now) const;
    size_t peekMeshPayload(uint8_t *out, size_t capacity);
    void commitMeshPayload(size_t len, uint32_t now);
    void dropCurrentMeshFrame();

    void ingestMeshPayload(NodeNum source, const uint8_t *data, size_t len, int32_t rxRssi = 0, float rxSnr = 0);
    void processOutput(uint32_t now);
    void serviceFlowControl(uint32_t now);

    // Legacy SerialModule API. A zero destination means broadcast, which is the initial
    // any-to-any routing policy. There is no peer lock.
    NodeNum getPeer() const { return 0; }

    const MavlinkBridgeStats &getStats();
    MavlinkRole getRole() const { return role; }
    bool takePositionSnapshot(uint32_t now, MavlinkPositionSnapshot &out);
    bool getBatterySnapshot(uint32_t now, MavlinkBatterySnapshot &out) const;

  private:
    void queueFrame(const mavlink_message_t &msg, uint32_t now);
    void flushPending(uint32_t now);

    void snoopSerialBytes(const uint8_t *data, size_t len, uint32_t now);
    void handleSnoopedMessage(const mavlink_message_t &msg, uint32_t now);
    void snoopHeartbeat(const mavlink_message_t &msg);
    void snoopPosition(const mavlink_message_t &msg, uint32_t now);
    void snoopBattery(const mavlink_message_t &msg, uint32_t now);
    void snoopHighLatency2(const mavlink_message_t &msg, uint32_t now);
    bool fromAutopilot(const mavlink_message_t &msg) const;

    MavlinkMeshTransport transport;

    uint8_t pendingFrame[MAVLINK_MAX_PACKET_LEN];
    size_t pendingLen = 0;
    size_t pendingOff = 0;
    uint32_t pendingSinceMs = 0;

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
