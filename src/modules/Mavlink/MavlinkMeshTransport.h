#pragma once

#include "mesh/MeshTypes.h"
#include <Arduino.h>

/**
 * Minimal frame-aware transport for MAVLink over Meshtastic.
 *
 * Local MAVLink byte streams are parsed into complete, byte-exact wire frames before
 * transmission. A frame is carried whole when it fits, or split into explicit fragments.
 * Receive-side partial state is isolated by (Meshtastic source node, frame id), so packets
 * from independent MAVLink systems may arrive interleaved without corrupting one another.
 *
 * Routing is deliberately permissive in this first implementation: callers broadcast the
 * resulting transport packets. Normal MAVLink sysid/target semantics provide logical
 * deconfliction. Learned unicast routing can be added later as an airtime optimization.
 */
class MavlinkMeshTransport
{
  public:
    static constexpr uint8_t MAGIC0 = 'M';
    static constexpr uint8_t MAGIC1 = 'V';
    static constexpr uint8_t VERSION = 1;
    static constexpr size_t HEADER_SIZE = 9;

    // Aggregate container. Small frames dominate this link: a 21-byte HEARTBEAT costs the same
    // airtime as a full payload, so one frame per packet wastes most of every transmission.
    // Whole frames are packed together when several are queued; oversized frames still use the
    // single-frame fragment format above.
    //   [0] MAGIC0 [1] MAGIC1 [2] VERSION_AGGREGATE [3] frame count
    //   then, per frame: uint16 length, followed by the exact MAVLink wire bytes
    static constexpr uint8_t VERSION_AGGREGATE = 2;
    static constexpr size_t AGGREGATE_HEADER_SIZE = 4;
    static constexpr size_t AGGREGATE_LENGTH_PREFIX = 2;
    // Bounded by both queue depths: never pack more frames than the receiver can enqueue at once.
    static constexpr size_t MAX_AGGREGATE_FRAMES = 4;
    static constexpr size_t MAX_MESH_PAYLOAD = meshtastic_Constants_DATA_PAYLOAD_LEN - 32; // 201
    static constexpr size_t MAX_FRAGMENT_DATA = MAX_MESH_PAYLOAD - HEADER_SIZE;            // 192
    static constexpr size_t TX_QUEUE_DEPTH = 4;
    static constexpr size_t RX_QUEUE_DEPTH = 4;
    static constexpr size_t REASSEMBLY_SLOTS = 4;
    static constexpr uint32_t REASSEMBLY_TIMEOUT_MS = 5000;
    static constexpr size_t MAX_FRAGMENT_COUNT =
        (MAVLINK_MAX_PACKET_LEN + MAX_FRAGMENT_DATA - 1) / MAX_FRAGMENT_DATA;

    static_assert(MAX_FRAGMENT_DATA > 0, "MAVLink mesh transport header exceeds payload capacity");
    static_assert(MAX_FRAGMENT_COUNT <= 32, "Fragment bitmap is too small");

    struct Counters {
        uint32_t localFramesQueued = 0;
        uint32_t outboundFramesSent = 0;
        uint32_t outboundFramesDropped = 0;
        uint32_t outboundDropBytes = 0;
        uint32_t corruptFrames = 0;
        uint32_t framingErrors = 0;
        uint32_t malformedPayloads = 0;
        uint32_t duplicateFragments = 0;
        uint32_t reassemblyTimeouts = 0;
        uint32_t reassemblyEvictions = 0;
        uint32_t reassembledFrames = 0;
        uint32_t inboundFramesDropped = 0;
        uint32_t inboundDropBytes = 0;
        uint32_t commandAckFramesQueued = 0;
        uint32_t commandAckFramesSent = 0;
        uint32_t commandAckOutboundDrops = 0;
        uint32_t commandAckFramesReassembled = 0;
        uint32_t commandAckInboundDrops = 0;
        size_t outboundHighWater = 0;
        size_t inboundHighWater = 0;
    };

    void ingestLocalBytes(const uint8_t *data, size_t len)
    {
        for (size_t i = 0; i < len; i++)
            ingestLocalByte(data[i]);
    }

    bool hasOutbound() const { return txCount != 0; }

    size_t peekOutbound(uint8_t *out, size_t capacity) const
    {
        size_t framesPacked = 0;
        return planOutbound(out, capacity, framesPacked);
    }

    bool commitOutbound(size_t payloadLen)
    {
        if (!txCount)
            return false;

        // Replanning against the committed length reproduces the peek exactly: the queue has not
        // changed, the packing is greedy, and the plan consumed precisely payloadLen bytes.
        size_t framesPacked = 0;
        if (planOutbound(nullptr, payloadLen, framesPacked) != payloadLen)
            return false;

        if (!framesPacked) {
            TxFrame &frame = txFrames[txHead];
            frame.nextFragment++;
            if (frame.nextFragment >= fragmentCount(frame.len))
                retireOutboundHead();
            return true;
        }

        for (size_t i = 0; i < framesPacked; i++)
            retireOutboundHead();
        return true;
    }

    void dropOutbound()
    {
        if (!txCount)
            return;
        if (isCommandAckFrame(txFrames[txHead].bytes, txFrames[txHead].len))
            counters.commandAckOutboundDrops++;
        counters.outboundFramesDropped++;
        counters.outboundDropBytes += txFrames[txHead].len;
        txHead = (txHead + 1) % TX_QUEUE_DEPTH;
        txCount--;
    }

    uint8_t outboundFreePercent() const
    {
        return (uint8_t)(((TX_QUEUE_DEPTH - txCount) * 100U) / TX_QUEUE_DEPTH);
    }

    void ingestMeshPayload(NodeNum source, const uint8_t *data, size_t len, uint32_t now)
    {
        expireReassembly(now);
        if (!data || len < AGGREGATE_HEADER_SIZE || data[0] != MAGIC0 || data[1] != MAGIC1) {
            counters.malformedPayloads++;
            return;
        }

        if (data[2] == VERSION_AGGREGATE) {
            ingestAggregatePayload(data, len);
            return;
        }

        if (data[2] != VERSION || len < HEADER_SIZE) {
            counters.malformedPayloads++;
            return;
        }

        const uint16_t frameId = getU16(data + 3);
        const uint8_t fragmentIndex = data[5];
        const uint8_t fragments = data[6];
        const uint16_t frameLen = getU16(data + 7);
        if (!frameId || !frameLen || frameLen > MAVLINK_MAX_PACKET_LEN || !fragments ||
            fragments > MAX_FRAGMENT_COUNT || fragmentIndex >= fragments || fragments != fragmentCount(frameLen)) {
            counters.malformedPayloads++;
            return;
        }

        const size_t offset = (size_t)fragmentIndex * MAX_FRAGMENT_DATA;
        const size_t expectedFragmentLen = min(MAX_FRAGMENT_DATA, (size_t)frameLen - offset);
        if (offset >= frameLen || len != HEADER_SIZE + expectedFragmentLen) {
            counters.malformedPayloads++;
            return;
        }

        ReassemblySlot *slot = findReassembly(source, frameId);
        if (!slot)
            slot = allocateReassembly(source, frameId, frameLen, fragments, now);
        if (!slot)
            return;
        if (slot->frameLen != frameLen || slot->fragmentCount != fragments) {
            slot->active = false;
            counters.malformedPayloads++;
            return;
        }

        const uint32_t bit = 1UL << fragmentIndex;
        if (slot->receivedMask & bit) {
            counters.duplicateFragments++;
            slot->lastUpdateMs = now;
            return;
        }

        memcpy(slot->bytes + offset, data + HEADER_SIZE, expectedFragmentLen);
        slot->receivedMask |= bit;
        slot->lastUpdateMs = now;

        const uint32_t completeMask = fragments == 32 ? UINT32_MAX : ((1UL << fragments) - 1UL);
        if (slot->receivedMask == completeMask) {
            enqueueInbound(slot->bytes, slot->frameLen);
            slot->active = false;
        }
    }

    bool popInbound(uint8_t *out, size_t capacity, size_t &len)
    {
        if (!rxCount)
            return false;
        const RxFrame &frame = rxFrames[rxHead];
        if (capacity < frame.len)
            return false;
        memcpy(out, frame.bytes, frame.len);
        len = frame.len;
        rxHead = (rxHead + 1) % RX_QUEUE_DEPTH;
        rxCount--;
        return true;
    }

    const Counters &getCounters() const { return counters; }

  private:
    struct TxFrame {
        uint16_t frameId = 0;
        uint16_t len = 0;
        uint8_t nextFragment = 0;
        uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
    };

    struct RxFrame {
        uint16_t len = 0;
        uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
    };

    struct ReassemblySlot {
        bool active = false;
        NodeNum source = 0;
        uint16_t frameId = 0;
        uint16_t frameLen = 0;
        uint8_t fragmentCount = 0;
        uint32_t receivedMask = 0;
        uint32_t lastUpdateMs = 0;
        uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
    };

    static void putU16(uint8_t *out, uint16_t value)
    {
        out[0] = (uint8_t)(value & 0xff);
        out[1] = (uint8_t)(value >> 8);
    }

    static uint16_t getU16(const uint8_t *in) { return (uint16_t)in[0] | ((uint16_t)in[1] << 8); }

    static bool frameHasMessageId(const uint8_t *frame, size_t len, uint32_t wanted)
    {
        if (!frame)
            return false;
        if (len >= 6 && frame[0] == 0xfe)
            return frame[5] == wanted;
        if (len >= 10 && frame[0] == 0xfd) {
            const uint32_t msgid =
                (uint32_t)frame[7] | ((uint32_t)frame[8] << 8) | ((uint32_t)frame[9] << 16);
            return msgid == wanted;
        }
        return false;
    }

    static bool isCommandAckFrame(const uint8_t *frame, size_t len)
    {
        return frameHasMessageId(frame, len, MAVLINK_MSG_ID_COMMAND_ACK);
    }

    static uint8_t fragmentCount(size_t frameLen)
    {
        return (uint8_t)((frameLen + MAX_FRAGMENT_DATA - 1) / MAX_FRAGMENT_DATA);
    }

    /**
     * Build the next outbound payload, optionally writing it. Returns the payload length and
     * reports how many whole frames were aggregated; a count of zero means the single-frame
     * fragment format was used for the head frame.
     *
     * Called by both peek and commit so the two always agree without carrying pending state.
     */
    size_t planOutbound(uint8_t *out, size_t capacity, size_t &framesPacked) const
    {
        framesPacked = 0;
        if (!txCount)
            return 0;

        const TxFrame &head = txFrames[txHead];
        const size_t wholeCost = AGGREGATE_HEADER_SIZE + AGGREGATE_LENGTH_PREFIX + head.len;

        // A frame already mid-fragmentation, or one too large to carry whole, keeps the
        // single-frame format.
        if (head.nextFragment != 0 || wholeCost > capacity) {
            if (capacity < HEADER_SIZE)
                return 0;
            const size_t offset = (size_t)head.nextFragment * MAX_FRAGMENT_DATA;
            if (offset >= head.len)
                return 0;
            const size_t fragmentLen = min(MAX_FRAGMENT_DATA, (size_t)head.len - offset);
            const size_t totalLen = HEADER_SIZE + fragmentLen;
            if (capacity < totalLen)
                return 0;
            if (out) {
                out[0] = MAGIC0;
                out[1] = MAGIC1;
                out[2] = VERSION;
                putU16(out + 3, head.frameId);
                out[5] = head.nextFragment;
                out[6] = fragmentCount(head.len);
                putU16(out + 7, head.len);
                memcpy(out + HEADER_SIZE, head.bytes + offset, fragmentLen);
            }
            return totalLen;
        }

        size_t used = AGGREGATE_HEADER_SIZE;
        size_t count = 0;
        while (count < txCount && count < MAX_AGGREGATE_FRAMES) {
            const TxFrame &frame = txFrames[(txHead + count) % TX_QUEUE_DEPTH];
            if (frame.nextFragment != 0 || !frame.len)
                break;
            const size_t need = AGGREGATE_LENGTH_PREFIX + frame.len;
            if (used + need > capacity)
                break;
            if (out) {
                putU16(out + used, frame.len);
                memcpy(out + used + AGGREGATE_LENGTH_PREFIX, frame.bytes, frame.len);
            }
            used += need;
            count++;
        }
        if (!count)
            return 0;

        if (out) {
            out[0] = MAGIC0;
            out[1] = MAGIC1;
            out[2] = VERSION_AGGREGATE;
            out[3] = (uint8_t)count;
        }
        framesPacked = count;
        return used;
    }

    void retireOutboundHead()
    {
        const TxFrame &frame = txFrames[txHead];
        if (isCommandAckFrame(frame.bytes, frame.len))
            counters.commandAckFramesSent++;
        txHead = (txHead + 1) % TX_QUEUE_DEPTH;
        txCount--;
        counters.outboundFramesSent++;
    }

    void resetCapture()
    {
        rawLen = 0;
        rawOverflow = false;
        pendingSignedBadCrc = false;
        pendingBadCrcMsgid = 0;
    }

    void ingestLocalByte(uint8_t c)
    {
        if (localStatus.parse_state == MAVLINK_PARSE_STATE_IDLE)
            resetCapture();
        if (rawLen < sizeof(rawFrame))
            rawFrame[rawLen++] = c;
        else
            rawOverflow = true;

        mavlink_message_t msg;
        mavlink_status_t status;
        const uint8_t result = mavlink_frame_char_buffer(&localWorking, &localStatus, c, &msg, &status);
        if (result == MAVLINK_FRAMING_OK) {
            if (pendingSignedBadCrc) {
                if (mavlink_get_msg_entry(pendingBadCrcMsgid) == nullptr)
                    enqueueCapture();
                else
                    counters.corruptFrames++;
            } else {
                enqueueCapture();
            }
            resetCapture();
        } else if (result == MAVLINK_FRAMING_BAD_CRC) {
            if (localStatus.parse_state == MAVLINK_PARSE_STATE_SIGNATURE_WAIT) {
                pendingSignedBadCrc = true;
                pendingBadCrcMsgid = localWorking.msgid;
            } else {
                if (mavlink_get_msg_entry(msg.msgid) == nullptr)
                    enqueueCapture();
                else
                    counters.corruptFrames++;
                resetCapture();
            }
        }
    }

    void enqueueCapture()
    {
        if (rawOverflow || !rawLen || rawLen > MAVLINK_MAX_PACKET_LEN) {
            counters.framingErrors++;
            return;
        }
        const bool commandAck = isCommandAckFrame(rawFrame, rawLen);
        if (txCount >= TX_QUEUE_DEPTH) {
            if (commandAck)
                counters.commandAckOutboundDrops++;
            counters.outboundFramesDropped++;
            counters.outboundDropBytes += rawLen;
            return;
        }

        TxFrame &frame = txFrames[(txHead + txCount) % TX_QUEUE_DEPTH];
        frame.frameId = nextFrameId++;
        if (!nextFrameId)
            nextFrameId = 1;
        frame.len = (uint16_t)rawLen;
        frame.nextFragment = 0;
        memcpy(frame.bytes, rawFrame, rawLen);
        txCount++;
        counters.localFramesQueued++;
        if (commandAck)
            counters.commandAckFramesQueued++;
        counters.outboundHighWater = max(counters.outboundHighWater, txCount);
    }

    /**
     * Unpack an aggregate container. Every frame inside arrived whole, so no reassembly state is
     * involved. A malformed length stops the whole payload rather than guessing at the remainder.
     */
    void ingestAggregatePayload(const uint8_t *data, size_t len)
    {
        const uint8_t count = data[3];
        if (!count || count > MAX_AGGREGATE_FRAMES) {
            counters.malformedPayloads++;
            return;
        }

        size_t offset = AGGREGATE_HEADER_SIZE;
        for (uint8_t i = 0; i < count; i++) {
            if (offset + AGGREGATE_LENGTH_PREFIX > len) {
                counters.malformedPayloads++;
                return;
            }
            const uint16_t frameLen = getU16(data + offset);
            offset += AGGREGATE_LENGTH_PREFIX;
            if (!frameLen || frameLen > MAVLINK_MAX_PACKET_LEN || offset + frameLen > len) {
                counters.malformedPayloads++;
                return;
            }
            enqueueInbound(data + offset, frameLen);
            offset += frameLen;
        }
    }

    ReassemblySlot *findReassembly(NodeNum source, uint16_t frameId)
    {
        for (size_t i = 0; i < REASSEMBLY_SLOTS; i++) {
            if (reassembly[i].active && reassembly[i].source == source && reassembly[i].frameId == frameId)
                return &reassembly[i];
        }
        return nullptr;
    }

    ReassemblySlot *allocateReassembly(NodeNum source, uint16_t frameId, uint16_t frameLen, uint8_t fragments,
                                       uint32_t now)
    {
        ReassemblySlot *slot = nullptr;
        for (size_t i = 0; i < REASSEMBLY_SLOTS; i++) {
            if (!reassembly[i].active) {
                slot = &reassembly[i];
                break;
            }
        }
        if (!slot) {
            slot = &reassembly[0];
            for (size_t i = 1; i < REASSEMBLY_SLOTS; i++) {
                if ((int32_t)(reassembly[i].lastUpdateMs - slot->lastUpdateMs) < 0)
                    slot = &reassembly[i];
            }
            counters.reassemblyEvictions++;
        }

        slot->active = true;
        slot->source = source;
        slot->frameId = frameId;
        slot->frameLen = frameLen;
        slot->fragmentCount = fragments;
        slot->receivedMask = 0;
        slot->lastUpdateMs = now;
        return slot;
    }

    void expireReassembly(uint32_t now)
    {
        for (size_t i = 0; i < REASSEMBLY_SLOTS; i++) {
            if (reassembly[i].active && (now - reassembly[i].lastUpdateMs) >= REASSEMBLY_TIMEOUT_MS) {
                reassembly[i].active = false;
                counters.reassemblyTimeouts++;
            }
        }
    }

    void enqueueInbound(const uint8_t *data, size_t len)
    {
        const bool commandAck = isCommandAckFrame(data, len);
        if (rxCount >= RX_QUEUE_DEPTH) {
            if (commandAck)
                counters.commandAckInboundDrops++;
            counters.inboundFramesDropped++;
            counters.inboundDropBytes += len;
            return;
        }
        RxFrame &frame = rxFrames[(rxHead + rxCount) % RX_QUEUE_DEPTH];
        frame.len = (uint16_t)len;
        memcpy(frame.bytes, data, len);
        rxCount++;
        counters.reassembledFrames++;
        if (commandAck)
            counters.commandAckFramesReassembled++;
        counters.inboundHighWater = max(counters.inboundHighWater, rxCount);
    }

    TxFrame txFrames[TX_QUEUE_DEPTH];
    size_t txHead = 0;
    size_t txCount = 0;
    uint16_t nextFrameId = 1;

    RxFrame rxFrames[RX_QUEUE_DEPTH];
    size_t rxHead = 0;
    size_t rxCount = 0;

    ReassemblySlot reassembly[REASSEMBLY_SLOTS];

    mavlink_message_t localWorking{};
    mavlink_status_t localStatus{};
    uint8_t rawFrame[MAVLINK_MAX_PACKET_LEN];
    size_t rawLen = 0;
    bool rawOverflow = false;
    bool pendingSignedBadCrc = false;
    uint32_t pendingBadCrcMsgid = 0;

    Counters counters;
};
