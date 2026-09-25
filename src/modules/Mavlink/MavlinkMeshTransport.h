#pragma once

#include "mesh/MeshTypes.h"
#include <Arduino.h>

/**
 * Frame-aware transport for MAVLink over Meshtastic.
 *
 * Local MAVLink byte streams are parsed into complete, byte-exact wire frames before
 * transmission. A frame is carried whole when it fits, or split into explicit fragments.
 * Receive-side partial state is isolated by (Meshtastic source node, frame id), so packets
 * from independent MAVLink systems may arrive interleaved without corrupting one another.
 *
 * Each queued frame is classified by message id. The class sets its priority, whether the
 * packet carrying it asks Meshtastic for a routing ACK (engaging ReliableRouter's retry), and
 * whether a newer copy from the same sender replaces it before transmission. Frames from one
 * sender stream (sysid, compid) always leave in arrival order, so MAVLink signing timestamps
 * stay monotonic; priority only reorders between streams.
 *
 * Destinations for acknowledged classes are learned from inbound traffic (MAVLink sysid ->
 * Meshtastic node); until a route exists they are broadcast for discovery. Periodic state is
 * always broadcast and never acknowledged. The transport does no retransmission of its own:
 * Meshtastic retries acknowledged packets, MAVLink retries its own transactions.
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
    static constexpr size_t MAX_MESH_PAYLOAD = meshtastic_Constants_DATA_PAYLOAD_LEN - 32; // 201
    static constexpr size_t MAX_FRAGMENT_DATA = MAX_MESH_PAYLOAD - HEADER_SIZE;            // 192
    static constexpr size_t TX_QUEUE_DEPTH = 8;
    static constexpr size_t RX_QUEUE_DEPTH = 8;
    // Bounded by the receive queue: never pack more frames than the receiver can enqueue at once.
    static constexpr size_t MAX_AGGREGATE_FRAMES = RX_QUEUE_DEPTH;
    static constexpr size_t REASSEMBLY_SLOTS = 4;
    static constexpr uint32_t REASSEMBLY_TIMEOUT_MS = 5000;
    static constexpr size_t ROUTE_SLOTS = 8;
    static constexpr uint32_t ROUTE_TIMEOUT_MS = 120000;
    static constexpr size_t MAX_FRAGMENT_COUNT = (MAVLINK_MAX_PACKET_LEN + MAX_FRAGMENT_DATA - 1) / MAX_FRAGMENT_DATA;

    static_assert(MAX_FRAGMENT_DATA > 0, "MAVLink mesh transport header exceeds payload capacity");
    static_assert(MAX_FRAGMENT_COUNT <= 32, "Fragment bitmap is too small");
    static_assert(TX_QUEUE_DEPTH <= 255 && MAX_AGGREGATE_FRAMES <= 255, "Slot indices are stored as uint8_t");

    // Declaration order is priority order.
    enum class TrafficClass : uint8_t { TRANSACTION, EVENT, STATE, BULK };

    struct Counters {
        uint32_t localFramesQueued = 0;
        uint32_t outboundFramesSent = 0;
        uint32_t outboundFramesDropped = 0;
        uint32_t outboundDropBytes = 0;
        uint32_t outboundFramesEvicted = 0;
        uint32_t stateFramesReplaced = 0;
        uint32_t unicastPackets = 0;
        uint32_t broadcastPackets = 0;
        uint32_t ackRequestedPackets = 0;
        uint32_t routesLearned = 0;
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

    struct Route {
        bool used = false;
        uint8_t sysid = 0;
        NodeNum node = 0;
        int8_t hops = -1; // hops away of the packet that last refreshed this route; -1 unknown
        uint32_t lastSeenMs = 0;
    };

    void ingestLocalBytes(const uint8_t *data, size_t len)
    {
        for (size_t i = 0; i < len; i++)
            ingestLocalByte(data[i]);
    }

    bool hasOutbound() const { return txCount != 0; }

    size_t outboundCount() const { return txCount; }

    /**
     * Build the next outbound payload and record the plan that produced it; the planned*()
     * accessors describe how to send it. commitOutbound() retires exactly the frames recorded here.
     */
    size_t peekOutbound(uint8_t *out, size_t capacity, uint32_t now)
    {
        plan = Plan();
        buildPlan(out, capacity, now);
        return plan.len;
    }

    /// Learned node, or broadcast.
    NodeNum plannedDestination() const { return plan.dest; }
    bool plannedWantAck() const { return plan.wantAck; }
    /**
     * Hops needed to reach the planned destination (for broadcast: the farthest live peer), so
     * nodes in between relay the packet no further than someone who wants it. -1 when unknown,
     * including broadcast discovery before any route exists: send with the configured hop limit.
     */
    int8_t plannedHopLimit() const { return plan.hopLimit; }

    bool commitOutbound(size_t payloadLen)
    {
        if (!plan.valid || plan.len != payloadLen)
            return false;
        plan.valid = false;

        if (isBroadcast(plan.dest))
            counters.broadcastPackets++;
        else
            counters.unicastPackets++;
        if (plan.wantAck)
            counters.ackRequestedPackets++;

        if (plan.fragment) {
            TxFrame &frame = txFrames[plan.slots[0]];
            if (frame.nextFragment == 0)
                frame.fragmentDest = plan.dest;
            frame.nextFragment++;
            if (frame.nextFragment >= fragmentCount(frame.len))
                retireSlot(plan.slots[0]);
            return true;
        }

        for (size_t i = 0; i < plan.count; i++)
            retireSlot(plan.slots[i]);
        return true;
    }

    uint8_t outboundFreePercent() const { return (uint8_t)(((TX_QUEUE_DEPTH - txCount) * 100U) / TX_QUEUE_DEPTH); }

    /// `hopsAway` is how far the carrying packet travelled (-1 unknown); it scopes later sends to that node.
    void ingestMeshPayload(NodeNum source, const uint8_t *data, size_t len, uint32_t now, int8_t hopsAway = -1)
    {
        expireReassembly(now);
        if (!data || len < AGGREGATE_HEADER_SIZE || data[0] != MAGIC0 || data[1] != MAGIC1) {
            counters.malformedPayloads++;
            return;
        }

        if (data[2] == VERSION_AGGREGATE) {
            ingestAggregatePayload(source, data, len, now, hopsAway);
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
        if (!frameId || !frameLen || frameLen > MAVLINK_MAX_PACKET_LEN || !fragments || fragments > MAX_FRAGMENT_COUNT ||
            fragmentIndex >= fragments || fragments != fragmentCount(frameLen)) {
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
            enqueueInbound(source, slot->bytes, slot->frameLen, now, hopsAway);
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

    size_t activeRouteCount(uint32_t now) const
    {
        size_t n = 0;
        for (const auto &route : routes)
            n += routeLive(route, now) ? 1 : 0;
        return n;
    }

    const Route *getRoutes() const { return routes; }

    bool routeLive(const Route &route, uint32_t now) const
    {
        return route.used && (uint32_t)(now - route.lastSeenMs) <= ROUTE_TIMEOUT_MS;
    }

    const Counters &getCounters() const { return counters; }

    static TrafficClass classify(uint32_t msgid, bool &replaceable)
    {
        replaceable = false;
        switch (msgid) {
        case MAVLINK_MSG_ID_COMMAND_INT:
        case MAVLINK_MSG_ID_COMMAND_LONG:
        case MAVLINK_MSG_ID_COMMAND_ACK:
        case MAVLINK_MSG_ID_COMMAND_CANCEL:
        case MAVLINK_MSG_ID_SET_MODE:
        case MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL:
        case MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK:
        case MAVLINK_MSG_ID_SETUP_SIGNING:
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        case MAVLINK_MSG_ID_PARAM_SET:
        case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ:
        case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST:
        case MAVLINK_MSG_ID_PARAM_EXT_SET:
        case MAVLINK_MSG_ID_PARAM_EXT_ACK:
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
        case MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST:
        case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
        case MAVLINK_MSG_ID_MISSION_COUNT:
        case MAVLINK_MSG_ID_MISSION_REQUEST:
        case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
        case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
        case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
        case MAVLINK_MSG_ID_MISSION_ACK:
        case MAVLINK_MSG_ID_SET_HOME_POSITION:
        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
        case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
        case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
        case MAVLINK_MSG_ID_LOG_ERASE:
        case MAVLINK_MSG_ID_LOG_REQUEST_END:
        case MAVLINK_MSG_ID_REQUEST_EVENT:
        case MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR:
        case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
        case MSG_ID_PROTOCOL_VERSION:
            return TrafficClass::TRANSACTION;

        case MAVLINK_MSG_ID_STATUSTEXT:
        case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
        case MAVLINK_MSG_ID_EVENT:
            return TrafficClass::EVENT;

        case MAVLINK_MSG_ID_MISSION_ITEM:
        case MAVLINK_MSG_ID_MISSION_ITEM_INT:
        case MAVLINK_MSG_ID_PARAM_VALUE:
        case MAVLINK_MSG_ID_PARAM_EXT_VALUE:
        case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL:
        case MAVLINK_MSG_ID_LOG_ENTRY:
        case MAVLINK_MSG_ID_LOG_DATA:
            return TrafficClass::BULK;

        // Periodic state: only the newest unsent copy per sender is worth airtime.
        case MAVLINK_MSG_ID_HEARTBEAT:
        case MAVLINK_MSG_ID_SYS_STATUS:
        case MAVLINK_MSG_ID_SYSTEM_TIME:
        case MAVLINK_MSG_ID_GPS_RAW_INT:
        case MAVLINK_MSG_ID_GPS_STATUS:
        case MAVLINK_MSG_ID_SCALED_IMU:
        case MAVLINK_MSG_ID_RAW_IMU:
        case MAVLINK_MSG_ID_SCALED_PRESSURE:
        case MAVLINK_MSG_ID_ATTITUDE:
        case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
        case MAVLINK_MSG_ID_MISSION_CURRENT:
        case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
        case MAVLINK_MSG_ID_RC_CHANNELS:
        case MAVLINK_MSG_ID_MANUAL_CONTROL:
        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
        case MAVLINK_MSG_ID_VFR_HUD:
        case MAVLINK_MSG_ID_ATTITUDE_TARGET:
        case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
        case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
        case MAVLINK_MSG_ID_HIGHRES_IMU:
        case MAVLINK_MSG_ID_RADIO_STATUS:
        case MAVLINK_MSG_ID_TIMESYNC:
        case MAVLINK_MSG_ID_SCALED_IMU2:
        case MAVLINK_MSG_ID_POWER_STATUS:
        case MAVLINK_MSG_ID_ALTITUDE:
        case MAVLINK_MSG_ID_BATTERY_STATUS:
        case MAVLINK_MSG_ID_ESTIMATOR_STATUS:
        case MAVLINK_MSG_ID_WIND_COV:
        case MAVLINK_MSG_ID_HIGH_LATENCY:
        case MAVLINK_MSG_ID_HIGH_LATENCY2:
        case MAVLINK_MSG_ID_VIBRATION:
        case MAVLINK_MSG_ID_HOME_POSITION:
        case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
        case MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS:
        case MAVLINK_MSG_ID_ODOMETRY:
        case MAVLINK_MSG_ID_UTM_GLOBAL_POSITION:
        case MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE:
        case MSG_ID_CURRENT_MODE:
        case MSG_ID_AVAILABLE_MODES_MONITOR:
            replaceable = true;
            return TrafficClass::STATE;

        // Unknown and custom messages travel unacknowledged in arrival order, never replaced.
        default:
            return TrafficClass::STATE;
        }
    }

    static bool classWantsAck(TrafficClass cls) { return cls != TrafficClass::STATE; }

  private:
    // Newer than the pinned common dialect headers; PX4 emits them as part of its background floor.
    static constexpr uint32_t MSG_ID_PROTOCOL_VERSION = 300;
    static constexpr uint32_t MSG_ID_CURRENT_MODE = 436;
    static constexpr uint32_t MSG_ID_AVAILABLE_MODES_MONITOR = 437;

    struct FrameInfo {
        uint32_t msgid = 0;
        uint8_t sysid = 0;
        uint8_t compid = 0;
        uint8_t targetSystem = 0; // 0: broadcast, or the message has no target field
    };

    struct TxFrame {
        bool used = false;
        uint32_t order = 0; // arrival sequence; lower is older
        uint16_t frameId = 0;
        uint16_t len = 0;
        uint8_t nextFragment = 0;
        NodeNum fragmentDest = 0; // fixed once the first fragment has gone out
        TrafficClass cls = TrafficClass::STATE;
        bool replaceable = false;
        FrameInfo info;
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

    struct Plan {
        bool valid = false;
        bool fragment = false;
        bool wantAck = false;
        int8_t hopLimit = -1;
        NodeNum dest = NODENUM_BROADCAST;
        size_t len = 0;
        size_t count = 0;
        uint8_t slots[MAX_AGGREGATE_FRAMES] = {};
    };

    static void putU16(uint8_t *out, uint16_t value)
    {
        out[0] = (uint8_t)(value & 0xff);
        out[1] = (uint8_t)(value >> 8);
    }

    static uint16_t getU16(const uint8_t *in) { return (uint16_t)in[0] | ((uint16_t)in[1] << 8); }

    static bool parseFrameInfo(const uint8_t *frame, size_t len, FrameInfo &info)
    {
        size_t headerLen = 0;
        size_t payloadLen = 0;
        if (len >= 8 && frame[0] == 0xfe) {
            payloadLen = frame[1];
            headerLen = 6;
            info.sysid = frame[3];
            info.compid = frame[4];
            info.msgid = frame[5];
        } else if (len >= 12 && frame[0] == 0xfd) {
            payloadLen = frame[1];
            headerLen = 10;
            info.sysid = frame[5];
            info.compid = frame[6];
            info.msgid = (uint32_t)frame[7] | ((uint32_t)frame[8] << 8) | ((uint32_t)frame[9] << 16);
        } else {
            return false;
        }
        if (len < headerLen + payloadLen)
            return false;

        // MAVLink 2 trims trailing zero bytes, so a target offset past the payload means target 0.
        info.targetSystem = 0;
        const mavlink_msg_entry_t *entry = mavlink_get_msg_entry(info.msgid);
        if (entry && (entry->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_SYSTEM) && entry->target_system_ofs < payloadLen)
            info.targetSystem = frame[headerLen + entry->target_system_ofs];
        return true;
    }

    static uint8_t fragmentCount(size_t frameLen) { return (uint8_t)((frameLen + MAX_FRAGMENT_DATA - 1) / MAX_FRAGMENT_DATA); }

    static bool sameStream(const TxFrame &a, const TxFrame &b)
    {
        return a.info.sysid == b.info.sysid && a.info.compid == b.info.compid;
    }

    static bool higherPriority(const TxFrame &a, const TxFrame &b)
    {
        return a.cls != b.cls ? a.cls < b.cls : (int32_t)(a.order - b.order) < 0;
    }

    const Route *findRoute(uint8_t sysid, uint32_t now) const
    {
        for (const auto &route : routes) {
            if (routeLive(route, now) && route.sysid == sysid)
                return &route;
        }
        return nullptr;
    }

    /// The only remote node currently exchanging MAVLink with us, or 0 when there are none or several.
    NodeNum singleActivePeer(uint32_t now) const
    {
        NodeNum peer = 0;
        for (const auto &route : routes) {
            if (!routeLive(route, now))
                continue;
            if (peer && route.node != peer)
                return 0;
            peer = route.node;
        }
        return peer;
    }

    /**
     * Acknowledged classes go unicast to the learned node. State always broadcasts: one transmission
     * reaches every listener, and NextHopRouter treats an unacknowledged unicast we originate as
     * awaiting an implicit ACK the destination never sends, so it would re-send, re-flood, and mark
     * the route failed for stale data.
     */
    NodeNum resolveDestination(const TxFrame &frame, uint32_t now) const
    {
        if (frame.nextFragment != 0)
            return frame.fragmentDest;
        if (!classWantsAck(frame.cls))
            return NODENUM_BROADCAST;
        if (frame.info.targetSystem) {
            if (const Route *route = findRoute(frame.info.targetSystem, now))
                return route->node;
        }
        NodeNum peer = singleActivePeer(now);
        return peer ? peer : NODENUM_BROADCAST;
    }

    /// Hops to the farthest live route through `dest` (every live route for broadcast); -1 if none or unknown.
    int8_t hopsFor(NodeNum dest, uint32_t now) const
    {
        int8_t hops = -1;
        for (const auto &route : routes) {
            if (!routeLive(route, now) || !(isBroadcast(dest) || route.node == dest))
                continue;
            if (route.hops < 0)
                return -1;
            hops = max(hops, route.hops);
        }
        return hops;
    }

    void learnRoute(uint8_t sysid, NodeNum node, int8_t hops, uint32_t now)
    {
        if (!sysid || !node)
            return;
        Route *slot = nullptr;
        for (auto &route : routes) {
            if (route.used && route.sysid == sysid) {
                slot = &route;
                break;
            }
        }
        if (!slot) {
            for (auto &route : routes) {
                if (!route.used) {
                    slot = &route;
                    break;
                }
            }
        }
        if (!slot) {
            slot = &routes[0];
            for (auto &route : routes) {
                if ((int32_t)(route.lastSeenMs - slot->lastSeenMs) < 0)
                    slot = &route;
            }
        }
        const bool changed = !routeLive(*slot, now) || slot->sysid != sysid || slot->node != node || slot->hops != hops;
        slot->used = true;
        slot->sysid = sysid;
        slot->node = node;
        slot->hops = hops;
        slot->lastSeenMs = now;
        if (changed) {
            counters.routesLearned++;
            LOG_INFO("MAVLink route learned: sysid %u via node 0x%08x, %d hops away", (unsigned)sysid, node, (int)hops);
        }
    }

    /// True when no older queued frame from the same sender stream is still waiting to be planned.
    bool isStreamHead(size_t slot, const bool *planned) const
    {
        const TxFrame &frame = txFrames[slot];
        for (size_t i = 0; i < TX_QUEUE_DEPTH; i++) {
            const TxFrame &other = txFrames[i];
            if (i != slot && other.used && !planned[i] && sameStream(frame, other) && (int32_t)(other.order - frame.order) < 0)
                return false;
        }
        return true;
    }

    void buildPlan(uint8_t *out, size_t capacity, uint32_t now)
    {
        if (!txCount)
            return;

        // A frame part-way through fragmentation finishes before anything else is sent.
        int lead = -1;
        for (size_t i = 0; i < TX_QUEUE_DEPTH; i++) {
            if (txFrames[i].used && txFrames[i].nextFragment != 0)
                lead = (int)i;
        }

        if (lead < 0) {
            for (size_t i = 0; i < TX_QUEUE_DEPTH; i++) {
                if (txFrames[i].used && (lead < 0 || higherPriority(txFrames[i], txFrames[lead])))
                    lead = (int)i;
            }
            // Priority picks the stream; the stream's oldest frame goes first.
            for (size_t i = 0; i < TX_QUEUE_DEPTH; i++) {
                if (txFrames[i].used && sameStream(txFrames[i], txFrames[lead]) &&
                    (int32_t)(txFrames[i].order - txFrames[lead].order) < 0)
                    lead = (int)i;
            }
        }

        const TxFrame &head = txFrames[lead];
        plan.dest = resolveDestination(head, now);
        plan.hopLimit = hopsFor(plan.dest, now);
        const size_t wholeCost = AGGREGATE_HEADER_SIZE + AGGREGATE_LENGTH_PREFIX + head.len;

        if (head.nextFragment != 0 || wholeCost > capacity) {
            if (capacity < HEADER_SIZE)
                return;
            const size_t offset = (size_t)head.nextFragment * MAX_FRAGMENT_DATA;
            if (offset >= head.len)
                return;
            const size_t fragmentLen = min(MAX_FRAGMENT_DATA, (size_t)head.len - offset);
            const size_t totalLen = HEADER_SIZE + fragmentLen;
            if (capacity < totalLen)
                return;
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
            plan.valid = true;
            plan.fragment = true;
            plan.count = 1;
            plan.slots[0] = (uint8_t)lead;
            plan.len = totalLen;
            plan.wantAck = !isBroadcast(plan.dest) && classWantsAck(head.cls);
            return;
        }

        // Pack the lead, then the best remaining frames bound for the same node, respecting
        // per-stream order. Greedy by priority; a frame becomes eligible once its predecessor is in.
        bool planned[TX_QUEUE_DEPTH] = {};
        size_t used = AGGREGATE_HEADER_SIZE;
        int next = lead;
        while (next >= 0 && plan.count < MAX_AGGREGATE_FRAMES) {
            const TxFrame &frame = txFrames[next];
            if (out) {
                putU16(out + used, frame.len);
                memcpy(out + used + AGGREGATE_LENGTH_PREFIX, frame.bytes, frame.len);
            }
            used += AGGREGATE_LENGTH_PREFIX + frame.len;
            planned[next] = true;
            plan.slots[plan.count++] = (uint8_t)next;
            if (classWantsAck(frame.cls))
                plan.wantAck = true;

            next = -1;
            for (size_t i = 0; i < TX_QUEUE_DEPTH; i++) {
                const TxFrame &candidate = txFrames[i];
                if (!candidate.used || planned[i] || candidate.nextFragment != 0)
                    continue;
                if (used + AGGREGATE_LENGTH_PREFIX + candidate.len > capacity)
                    continue;
                if (!isStreamHead(i, planned) || resolveDestination(candidate, now) != plan.dest)
                    continue;
                if (next < 0 || higherPriority(candidate, txFrames[next]))
                    next = (int)i;
            }
        }

        if (out) {
            out[0] = MAGIC0;
            out[1] = MAGIC1;
            out[2] = VERSION_AGGREGATE;
            out[3] = (uint8_t)plan.count;
        }
        plan.valid = true;
        plan.len = used;
        plan.wantAck = plan.wantAck && !isBroadcast(plan.dest);
    }

    void retireSlot(size_t slot)
    {
        TxFrame &frame = txFrames[slot];
        if (frame.info.msgid == MAVLINK_MSG_ID_COMMAND_ACK)
            counters.commandAckFramesSent++;
        frame.used = false;
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

    int freeSlot() const
    {
        for (size_t i = 0; i < TX_QUEUE_DEPTH; i++) {
            if (!txFrames[i].used)
                return (int)i;
        }
        return -1;
    }

    /**
     * Pick a queued frame the incoming one may displace. State is always expendable to newer
     * traffic; bulk only yields to transactions and events. Transactions and events are never
     * evicted, and neither is a frame part-way through fragmentation.
     */
    int evictionVictim(TrafficClass incoming) const
    {
        int victim = -1;
        for (size_t i = 0; i < TX_QUEUE_DEPTH; i++) {
            const TxFrame &frame = txFrames[i];
            if (!frame.used || frame.nextFragment != 0)
                continue;
            const bool expendable =
                frame.cls == TrafficClass::STATE || (frame.cls == TrafficClass::BULK && incoming <= TrafficClass::EVENT);
            if (!expendable)
                continue;
            if (victim < 0) {
                victim = (int)i;
                continue;
            }
            const TxFrame &best = txFrames[victim];
            // Prefer unacknowledged state over bulk, then the oldest.
            if (frame.cls != best.cls ? frame.cls == TrafficClass::STATE : (int32_t)(frame.order - best.order) < 0)
                victim = (int)i;
        }
        return victim;
    }

    void enqueueCapture()
    {
        FrameInfo info;
        if (rawOverflow || !rawLen || rawLen > MAVLINK_MAX_PACKET_LEN || !parseFrameInfo(rawFrame, rawLen, info)) {
            counters.framingErrors++;
            return;
        }
        plan.valid = false;

        bool replaceable = false;
        const TrafficClass cls = classify(info.msgid, replaceable);
        const bool commandAck = info.msgid == MAVLINK_MSG_ID_COMMAND_ACK;

        int slot = -1;
        if (replaceable) {
            for (size_t i = 0; i < TX_QUEUE_DEPTH; i++) {
                const TxFrame &frame = txFrames[i];
                if (frame.used && frame.replaceable && frame.nextFragment == 0 && frame.info.msgid == info.msgid &&
                    frame.info.sysid == info.sysid && frame.info.compid == info.compid) {
                    slot = (int)i;
                    counters.stateFramesReplaced++;
                    break;
                }
            }
        }
        if (slot < 0) {
            slot = freeSlot();
            if (slot < 0) {
                slot = evictionVictim(cls);
                if (slot < 0) {
                    if (commandAck)
                        counters.commandAckOutboundDrops++;
                    counters.outboundFramesDropped++;
                    counters.outboundDropBytes += rawLen;
                    return;
                }
                counters.outboundFramesEvicted++;
                counters.outboundDropBytes += txFrames[slot].len;
            } else {
                txCount++;
            }
        }

        // A replacement takes a new arrival position: it is now the stream's newest frame.
        TxFrame &frame = txFrames[slot];
        frame.used = true;
        frame.order = nextOrder++;
        frame.frameId = nextFrameId++;
        if (!nextFrameId)
            nextFrameId = 1;
        frame.len = (uint16_t)rawLen;
        frame.nextFragment = 0;
        frame.fragmentDest = 0;
        frame.cls = cls;
        frame.replaceable = replaceable;
        frame.info = info;
        memcpy(frame.bytes, rawFrame, rawLen);
        counters.localFramesQueued++;
        if (commandAck)
            counters.commandAckFramesQueued++;
        counters.outboundHighWater = max(counters.outboundHighWater, txCount);
    }

    /**
     * Unpack an aggregate container. Every frame inside arrived whole, so no reassembly state is
     * involved. A malformed length stops the whole payload rather than guessing at the remainder.
     */
    void ingestAggregatePayload(NodeNum source, const uint8_t *data, size_t len, uint32_t now, int8_t hopsAway)
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
            enqueueInbound(source, data + offset, frameLen, now, hopsAway);
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

    ReassemblySlot *allocateReassembly(NodeNum source, uint16_t frameId, uint16_t frameLen, uint8_t fragments, uint32_t now)
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

    void enqueueInbound(NodeNum source, const uint8_t *data, size_t len, uint32_t now, int8_t hopsAway)
    {
        FrameInfo info;
        if (parseFrameInfo(data, len, info))
            learnRoute(info.sysid, source, hopsAway, now);

        const bool commandAck = info.msgid == MAVLINK_MSG_ID_COMMAND_ACK;
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
    size_t txCount = 0;
    uint32_t nextOrder = 1;
    uint16_t nextFrameId = 1;
    Plan plan;

    RxFrame rxFrames[RX_QUEUE_DEPTH];
    size_t rxHead = 0;
    size_t rxCount = 0;

    ReassemblySlot reassembly[REASSEMBLY_SLOTS];
    Route routes[ROUTE_SLOTS];

    mavlink_message_t localWorking{};
    mavlink_status_t localStatus{};
    uint8_t rawFrame[MAVLINK_MAX_PACKET_LEN];
    size_t rawLen = 0;
    bool rawOverflow = false;
    bool pendingSignedBadCrc = false;
    uint32_t pendingBadCrcMsgid = 0;

    Counters counters;
};
