#include "MavlinkBridge.h"

#if !MESHTASTIC_EXCLUDE_MAVLINK

#include "DebugConfiguration.h"
#include <Throttle.h>

MavlinkBridge *mavlinkBridge;

void MavlinkBridge::ingestSerialBytes(const uint8_t *data, size_t len, uint32_t now)
{
    if (!len)
        return;
    stats.uartRxBytes += len;
    if (inputFifo.size() == 0)
        firstPendingMs = now;
    size_t accepted = inputFifo.push(data, len);
    if (accepted < len) {
        stats.inputOverflowBytes += len - accepted;
        if (!Throttle::isWithinTimespanMs(lastInOverflowLogMs, 5000)) {
            lastInOverflowLogMs = now;
            LOG_WARN("MAVLink input FIFO overflow, %u bytes dropped total", stats.inputOverflowBytes);
        }
    }
}

bool MavlinkBridge::wantsMeshSend(uint32_t now) const
{
    size_t queued = inputFifo.size();
    if (queued >= MAX_CHUNK)
        return true;
    return queued > 0 && (now - firstPendingMs) >= FLUSH_INTERVAL_MS;
}

size_t MavlinkBridge::peekMeshPayload(uint8_t *out, size_t capacity)
{
    return inputFifo.peek(out, min(capacity, MAX_CHUNK));
}

void MavlinkBridge::commitMeshPayload(size_t len, uint32_t now)
{
    inputFifo.drop(len);
    stats.meshTxBytes += len;
    if (inputFifo.size() > 0)
        firstPendingMs = now; // restart the age clock for the remaining bytes
}

void MavlinkBridge::ingestMeshPayload(NodeNum source, const uint8_t *data, size_t len)
{
    if (!acceptSource(source)) {
        stats.rejectedSourceChunks++;
        return;
    }
    stats.meshRxBytes += len;
    size_t accepted = outputFifo.push(data, len);
    if (accepted < len) {
        stats.outputOverflowBytes += len - accepted;
        if (!Throttle::isWithinTimespanMs(lastOutOverflowLogMs, 5000)) {
            lastOutOverflowLogMs = millis();
            LOG_WARN("MAVLink output FIFO overflow, %u bytes dropped total", stats.outputOverflowBytes);
        }
    }
}

bool MavlinkBridge::acceptSource(NodeNum source)
{
    if (lockedPeer == 0) {
        lockedPeer = source;
        LOG_INFO("MAVLink bridge locked to peer 0x%08x", source);
    }
    return source == lockedPeer;
}

void MavlinkBridge::processOutput(uint32_t now)
{
    flushPending(now);
    while (pendingLen == 0) { // stop parsing while a frame is stuck waiting for UART space
        uint8_t c;
        if (!outputFifo.pop(&c, 1))
            break;
        mavlink_message_t msg;
        mavlink_status_t status;
        uint8_t res = mavlink_frame_char_buffer(&meshRxWorking, &meshRxStatus, c, &msg, &status);
        if (res == MAVLINK_FRAMING_OK) {
            queueFrame(msg, now);
        } else if (res == MAVLINK_FRAMING_BAD_CRC) {
            // No CRC_EXTRA entry means an unknown-dialect message: the wire checksum is preserved
            // in msg, so forward it untouched. A known msgid with bad CRC is real corruption.
            if (mavlink_get_msg_entry(msg.msgid) == nullptr)
                queueFrame(msg, now);
            else
                stats.corruptFrames++;
        }
        flushPending(now);
    }
}

void MavlinkBridge::queueFrame(const mavlink_message_t &msg, uint32_t now)
{
    pendingLen = mavlink_msg_to_send_buffer(pendingFrame, &msg);
    pendingOff = 0;
    pendingSinceMs = now;
}

void MavlinkBridge::flushPending(uint32_t now)
{
    if (!pendingLen)
        return;
    int avail = uart->availableForWrite();
    if (avail > 0) {
        size_t n = min((size_t)avail, pendingLen - pendingOff);
        size_t written = uart->write(pendingFrame + pendingOff, n);
        pendingOff += written;
        if (written)
            pendingSinceMs = now;
    }
    if (pendingOff >= pendingLen) {
        pendingLen = pendingOff = 0;
        stats.framesToUart++;
    } else if ((now - pendingSinceMs) >= UART_TX_STALL_MS) {
        // UART TX has been full for too long (reader gone?) — drop rather than deadlock
        stats.uartTxStallDrops++;
        pendingLen = pendingOff = 0;
    }
}

const MavlinkBridgeStats &MavlinkBridge::getStats()
{
    stats.inputHighWater = inputFifo.highWater();
    stats.outputHighWater = outputFifo.highWater();
    return stats;
}

#endif
