#pragma once

#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_MAVLINK

#include "mesh/MeshTypes.h"
#include <Arduino.h>

#define MAVLINK_COMM_NUM_BUFFERS 1
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
    uint32_t framesToUart = 0;      // complete frames written to UART
    uint32_t corruptFrames = 0;     // known msgid, bad CRC — discarded
    uint32_t inputOverflowBytes = 0;
    uint32_t outputOverflowBytes = 0;
    uint32_t rejectedSourceChunks = 0; // chunks from a node other than the locked peer
    uint32_t uartTxStallDrops = 0;     // frames dropped because UART TX stayed full
    size_t inputHighWater = 0;
    size_t outputHighWater = 0;
};

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
    static constexpr uint32_t FLUSH_INTERVAL_MS = 100; // send a partial chunk after this age
    static constexpr uint32_t UART_TX_STALL_MS = 500;  // drop a pending frame after this long

    explicit MavlinkBridge(Stream *uart) : uart(uart) {}

    void ingestSerialBytes(const uint8_t *data, size_t len, uint32_t now);

    /// True when a chunk should be sent: a full payload is ready or data has aged past FLUSH_INTERVAL_MS
    bool wantsMeshSend(uint32_t now) const;

    /// Transactional handoff: peek copies without consuming; commit after the packet was accepted for send
    size_t peekMeshPayload(uint8_t *out, size_t capacity);
    void commitMeshPayload(size_t len, uint32_t now);

    void ingestMeshPayload(NodeNum source, const uint8_t *data, size_t len);

    /// Parse output FIFO, write complete frames to UART. Bounded by UART TX capacity per call.
    void processOutput(uint32_t now);

    NodeNum getPeer() const { return lockedPeer; }
    const MavlinkBridgeStats &getStats();

  private:
    bool acceptSource(NodeNum source);
    void queueFrame(const mavlink_message_t &msg, uint32_t now);
    void flushPending(uint32_t now);

    ByteFifo<INPUT_FIFO_SIZE> inputFifo;
    ByteFifo<OUTPUT_FIFO_SIZE> outputFifo;

    // Caller-owned parser state (mavlink_frame_char_buffer) — one pair per bridge instance
    mavlink_message_t meshRxWorking{};
    mavlink_status_t meshRxStatus{};

    // One serialized frame in flight to the UART; survives short writes across calls
    uint8_t pendingFrame[MAVLINK_MAX_PACKET_LEN];
    size_t pendingLen = 0;
    size_t pendingOff = 0;
    uint32_t pendingSinceMs = 0;

    uint32_t firstPendingMs = 0; // when the input FIFO last went non-empty
    NodeNum lockedPeer = 0;      // first mesh node heard from; others rejected (prototype peer policy)
    Stream *uart;
    MavlinkBridgeStats stats;
    uint32_t lastInOverflowLogMs = 0;
    uint32_t lastOutOverflowLogMs = 0;
};

// Created by SerialModule when serial mode is MAVLINK; null otherwise
extern MavlinkBridge *mavlinkBridge;

#endif
