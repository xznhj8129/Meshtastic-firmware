#pragma once

#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_MAVLINK

#include "mesh/MeshTypes.h"
#include <Arduino.h>

// Only the caller-owned buffer API (mavlink_frame_char_buffer) is used; one channel buffer set is plenty
#ifndef MAVLINK_COMM_NUM_BUFFERS
#define MAVLINK_COMM_NUM_BUFFERS 1
#endif
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "common/mavlink.h"
#pragma GCC diagnostic pop

/**
 * Fixed-size byte ring buffer. push() accepts the fitting prefix and reports how much
 * was taken; the caller counts the dropped remainder.
 */
template <size_t N> class ByteFifo
{
  public:
    size_t push(const uint8_t *data, size_t len)
    {
        size_t accepted = min(len, N - count);
        for (size_t i = 0; i < accepted; i++)
            buf[(head + count + i) % N] = data[i];
        count += accepted;
        if (count > highWaterMark)
            highWaterMark = count;
        return accepted;
    }

    size_t peek(uint8_t *out, size_t maxLen) const
    {
        size_t n = min(maxLen, count);
        for (size_t i = 0; i < n; i++)
            out[i] = buf[(head + i) % N];
        return n;
    }

    size_t pop(uint8_t *out, size_t maxLen)
    {
        size_t n = peek(out, maxLen);
        drop(n);
        return n;
    }

    void drop(size_t len)
    {
        size_t n = min(len, count);
        head = (head + n) % N;
        count -= n;
    }

    size_t size() const { return count; }
    size_t freeSpace() const { return N - count; }
    static constexpr size_t capacity() { return N; }
    size_t highWater() const { return highWaterMark; }

  private:
    uint8_t buf[N];
    size_t head = 0;
    size_t count = 0;
    size_t highWaterMark = 0;
};

struct MavlinkBridgeStats {
    uint32_t uartRxBytes = 0;
    uint32_t meshTxBytes = 0;
    uint32_t meshRxBytes = 0;
    uint32_t framesToUart = 0;  // complete frames written to UART
    uint32_t corruptFrames = 0; // known msgid, bad CRC - discarded
    uint32_t framingErrors = 0; // parser-level errors (bad flags, overruns)
    uint32_t inputOverflowBytes = 0;
    uint32_t outputOverflowBytes = 0;
    uint32_t rejectedSourceChunks = 0; // chunks from a node other than the locked peer
    uint32_t uartTxStallDrops = 0;     // frames dropped because UART TX stayed full
    uint32_t radioStatusSent = 0;      // locally generated RADIO_STATUS frames
    size_t inputHighWater = 0;
    size_t outputHighWater = 0;
};

// Source system id for locally generated RADIO_STATUS (255 = conventional GCS-side radio)
#ifndef MESHTASTIC_MAVLINK_RADIO_STATUS_SYSID
#define MESHTASTIC_MAVLINK_RADIO_STATUS_SYSID 255
#endif

// What the local UART is attached to, learned from snooped heartbeats (MAVLINK.md §9.1)
enum class MavlinkRole : uint8_t { UNKNOWN, AIR, GROUND };

struct MavlinkBatterySnapshot {
    bool hasLevel = false;
    uint8_t level = 0; // percent
    bool hasVoltage = false;
    float voltage = 0; // volts, sum of valid cells
};

struct MavlinkPositionSnapshot {
    bool hasFix = false; // lat/lon/alt valid
    int32_t latI = 0;    // degrees x 1e7 (MAVLink and Meshtastic share this encoding)
    int32_t lonI = 0;
    int32_t altM = 0; // meters MSL
    bool hasGroundSpeed = false;
    uint32_t groundSpeedKmh = 0; // km/h, matching the firmware's actual ground_speed convention
    bool hasGroundTrack = false;
    uint32_t groundTrack1e5 = 0; // degrees x 1e5, matching the firmware's ground_track convention
    uint32_t fixType = 0;        // MAVLink GPS fix type, pass-through
    bool hasSats = false;
    uint32_t sats = 0;
};

// Compile-time role override (MAVLINK.md section 9.1): 0 = auto (heartbeat-learned), 1 = force AIR,
// 2 = force GROUND. Role gates local telemetry snooping only; transport is identical either way.
#ifndef MESHTASTIC_MAVLINK_ROLE
#define MESHTASTIC_MAVLINK_ROLE 0
#endif

/**
 * Transparent MAVLink <-> mesh byte bridge (see MAVLINK.md).
 *
 * UART bytes go into a bounded input FIFO and leave as raw chunks in SERIAL_APP
 * payloads; received mesh chunks go into a bounded output FIFO and are fed through
 * the MAVLink parser so only complete frames reach the UART. Chunk boundaries are
 * independent of frame boundaries.
 *
 * Threading: every method must be called from the main cooperative scheduler
 * (OSThread runOnce / synchronous Router dispatch). No locking inside.
 */
class MavlinkBridge
{
  public:
    static constexpr size_t INPUT_FIFO_SIZE = 1024;
    static constexpr size_t OUTPUT_FIFO_SIZE = 512;
    static constexpr size_t MAX_CHUNK = meshtastic_Constants_DATA_PAYLOAD_LEN;
    static constexpr uint32_t FLUSH_INTERVAL_MS = 100;        // send a partial chunk after this age
    static constexpr uint32_t UART_TX_STALL_MS = 500;         // drop a pending frame after this long
    static constexpr uint32_t RADIO_STATUS_INTERVAL_MS = 500; // 2 Hz RADIO_STATUS (MAVLINK.md §7.1)
    static constexpr uint32_t ACTIVITY_WINDOW_MS = 10000;     // emit only while the link is in use
    static constexpr uint32_t POSITION_THROTTLE_MS = 1000;    // min interval between local position updates
    static constexpr uint32_t GLOBAL_POS_FRESH_MS = 5000;     // GPS_RAW_INT is only a fallback beyond this
    static constexpr uint32_t BATTERY_STALENESS_MS = 120000;  // hardware readings resume after this
    static constexpr uint32_t SYS_STATUS_DEFER_MS = 10000;    // SYS_STATUS yields to recent BATTERY_STATUS

    explicit MavlinkBridge(Stream *uart) : uart(uart) {}

    void ingestSerialBytes(const uint8_t *data, size_t len, uint32_t now);

    /// True when a chunk should be sent: a full payload is ready or data has aged past FLUSH_INTERVAL_MS
    bool wantsMeshSend(uint32_t now) const;

    /// Transactional handoff: peek copies without consuming; commit after the packet was accepted for send
    size_t peekMeshPayload(uint8_t *out, size_t capacity);
    void commitMeshPayload(size_t len, uint32_t now);

    void ingestMeshPayload(NodeNum source, const uint8_t *data, size_t len, int32_t rxRssi = 0, float rxSnr = 0);

    /// Parse output FIFO, write complete frames to UART. Bounded by UART TX capacity per call.
    void processOutput(uint32_t now);

    /// Periodic local RADIO_STATUS injection: txbuf = free input-FIFO %. Disable with
    /// MESHTASTIC_MAVLINK_NO_RADIO_STATUS for signed-only installations.
    void serviceFlowControl(uint32_t now);

    NodeNum getPeer() const { return lockedPeer; }
    const MavlinkBridgeStats &getStats();

    MavlinkRole getRole() const { return role; }

    /// Latest snooped aircraft position, at most one take per POSITION_THROTTLE_MS. Returns
    /// false when nothing new arrived since the last take. Consumer mirrors the GPS delivery
    /// path (nodeDB->setLocalPosition + positionModule->handleNewPosition).
    bool takePositionSnapshot(uint32_t now, MavlinkPositionSnapshot &out);

    /// Latest snooped aircraft battery. Returns false when stale (BATTERY_STALENESS_MS) or
    /// nothing was ever received; the caller then falls back to hardware readings.
    bool getBatterySnapshot(uint32_t now, MavlinkBatterySnapshot &out) const;

  private:
    bool acceptSource(NodeNum source);
    void queueFrame(const mavlink_message_t &msg, uint32_t now); // locally generated frames (canonical encode)
    void emitFrame(uint32_t now);                                // forward the captured raw wire bytes
    void flushPending(uint32_t now);

    // Local telemetry snooping (MAVLINK.md section 8): decode-only pass over UART ingest with
    // its own parser state pair; forwarding is untouched. Position/battery apply in AIR role.
    void snoopSerialBytes(const uint8_t *data, size_t len, uint32_t now);
    void handleSnoopedMessage(const mavlink_message_t &msg, uint32_t now);
    void snoopHeartbeat(const mavlink_message_t &msg);
    void snoopPosition(const mavlink_message_t &msg, uint32_t now);
    void snoopBattery(const mavlink_message_t &msg, uint32_t now);
    bool fromAutopilot(const mavlink_message_t &msg) const;

    ByteFifo<INPUT_FIFO_SIZE> inputFifo;
    ByteFifo<OUTPUT_FIFO_SIZE> outputFifo;

    // Caller-owned parser state (mavlink_frame_char_buffer) - one pair per bridge instance.
    // The parser is used only for frame-boundary detection, CRC validation and (Stage 3)
    // decode; forwarded traffic replays the captured raw wire bytes below, so transport is
    // byte-exact for non-canonical, signed and unknown-dialect frames.
    mavlink_message_t meshRxWorking{};
    mavlink_status_t meshRxStatus{};

    // Raw wire bytes of the frame currently being parsed. A byte fed while the parser is
    // idle starts a new capture; garbage bytes never accumulate (parser stays idle on them).
    uint8_t rawFrame[MAVLINK_MAX_PACKET_LEN];
    size_t rawLen = 0;
    bool rawOverflow = false; // defensive: buffer holds a max-size frame, so this should never trip
    // Signed frames report BAD_CRC at CRC2 (before the signature) and then OK once the
    // signature completes with no signing context - remember the CRC failure across it.
    bool pendingSignedBadCrc = false;
    uint32_t pendingBadCrcMsgid = 0;

    // One serialized frame in flight to the UART; survives short writes across calls
    uint8_t pendingFrame[MAVLINK_MAX_PACKET_LEN];
    size_t pendingLen = 0;
    size_t pendingOff = 0;
    uint32_t pendingSinceMs = 0;

    uint32_t firstPendingMs = 0; // when the input FIFO last went non-empty

    // Flow control (RADIO_STATUS)
    uint32_t lastRadioStatusMs = 0;
    uint32_t lastActivityMs = 0;
    bool everActive = false;
    int32_t lastRxRssi = 0; // final-LoRa-hop metadata from the last accepted chunk
    float lastRxSnr = 0;
    bool haveRxMetadata = false;

    // Snooping state
    mavlink_message_t snoopWorking{};
    mavlink_status_t snoopStatus{};
    MavlinkRole role = (MavlinkRole)MESHTASTIC_MAVLINK_ROLE;
    bool haveAutopilot = false; // first vehicle heartbeat learned; filters out gimbal/GCS chatter
    uint8_t autopilotSysid = 0;
    uint8_t autopilotCompid = 0;
    MavlinkPositionSnapshot posSnap;
    bool positionDirty = false;
    uint32_t lastGlobalPosMs = 0; // last GLOBAL_POSITION_INT (GPS_RAW_INT fallback beyond GLOBAL_POS_FRESH_MS)
    uint32_t lastPositionTakeMs = 0;
    MavlinkBatterySnapshot battSnap;
    uint32_t lastBatteryMs = 0;       // last accepted battery source of either kind
    uint32_t lastBatteryStatusMs = 0; // last BATTERY_STATUS (SYS_STATUS defers for SYS_STATUS_DEFER_MS)
    bool haveBatteryId = false;       // first-seen BATTERY_STATUS id sticks
    uint8_t batteryId = 0;

    NodeNum lockedPeer = 0; // first mesh node heard from; others rejected (prototype peer policy)
    Stream *uart;
    MavlinkBridgeStats stats;
    uint32_t lastInOverflowLogMs = 0;
    uint32_t lastOutOverflowLogMs = 0;
};

// Created by SerialModule when serial mode is MAVLINK; null otherwise
extern MavlinkBridge *mavlinkBridge;

#endif
