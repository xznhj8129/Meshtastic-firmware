#include "MavlinkBridge.h"

#if !MESHTASTIC_EXCLUDE_MAVLINK

#include "DebugConfiguration.h"
#include "NodeDB.h"
#include "Router.h"
#if MESHTASTIC_MAVLINK_UDP
#include "MavlinkUdpServer.h"
#endif

MavlinkBridge *mavlinkBridge;

static NullStream mavlinkNullStream;

namespace
{
bool decodeCommandAckFrame(const uint8_t *frame, size_t len, uint16_t &command, uint8_t &result, uint8_t &sysid, uint8_t &compid,
                           uint8_t &seq)
{
    if (!frame)
        return false;

    size_t headerLen = 0;
    size_t payloadLen = 0;
    uint32_t msgid = 0;
    if (len >= 6 && frame[0] == 0xfe) {
        payloadLen = frame[1];
        headerLen = 6;
        seq = frame[2];
        sysid = frame[3];
        compid = frame[4];
        msgid = frame[5];
    } else if (len >= 10 && frame[0] == 0xfd) {
        payloadLen = frame[1];
        headerLen = 10;
        seq = frame[4];
        sysid = frame[5];
        compid = frame[6];
        msgid = (uint32_t)frame[7] | ((uint32_t)frame[8] << 8) | ((uint32_t)frame[9] << 16);
    } else {
        return false;
    }

    if (msgid != MAVLINK_MSG_ID_COMMAND_ACK || payloadLen < 3 || len < headerLen + payloadLen + 2)
        return false;

    const uint8_t *payload = frame + headerLen;
    command = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    result = payload[2];
    return true;
}

void logCommandAckFrame(const char *stage, const uint8_t *frame, size_t len)
{
    uint16_t command = 0;
    uint8_t result = 0;
    uint8_t sysid = 0;
    uint8_t compid = 0;
    uint8_t seq = 0;
    if (decodeCommandAckFrame(frame, len, command, result, sysid, compid, seq)) {
        LOG_INFO("MAVLink COMMAND_ACK %s command=%u result=%u sysid=%u compid=%u seq=%u", stage, (unsigned)command,
                 (unsigned)result, (unsigned)sysid, (unsigned)compid, (unsigned)seq);
    }
}
} // namespace

MavlinkBridge::MavlinkBridge(Stream *serial) : uart(serial ? serial : &mavlinkNullStream)
{
    if (moduleConfig.serial.peer_node)
        LOG_WARN("MAVLink mesh mode ignores serial.peer_node 0x%08x; routes are learned from traffic",
                 moduleConfig.serial.peer_node);
}

void MavlinkBridge::ingestSerialBytes(const uint8_t *data, size_t len, uint32_t now)
{
    if (!data || !len)
        return;
    const auto before = transport.getCounters();
    stats.uartRxBytes += len;
    lastActivityMs = now;
    everActive = true;
    snoopSerialBytes(data, len, now);
    transport.ingestLocalBytes(data, len);
    if (transport.hasOutbound() && !outboundPending) {
        outboundPending = true;
        outboundSinceMs = now;
    }
    const auto &after = transport.getCounters();
    if (after.commandAckFramesQueued != before.commandAckFramesQueued) {
        LOG_INFO("MAVLink COMMAND_ACK transport queued count=%u",
                 (unsigned)(after.commandAckFramesQueued - before.commandAckFramesQueued));
    }
    if (after.commandAckOutboundDrops != before.commandAckOutboundDrops) {
        LOG_WARN("MAVLink COMMAND_ACK transport outbound drop count=%u",
                 (unsigned)(after.commandAckOutboundDrops - before.commandAckOutboundDrops));
    }
}

bool MavlinkBridge::wantsMeshSend(uint32_t now) const
{
    if (!transport.hasOutbound())
        return false;

    // A full pack gains nothing from waiting.
    if (transport.outboundCount() >= MavlinkMeshTransport::MAX_AGGREGATE_FRAMES)
        return true;

    // Without a recorded start the frame predates the window; do not strand it.
    if (!outboundPending)
        return true;

    return (uint32_t)(now - outboundSinceMs) >= COALESCE_WINDOW_MS;
}

size_t MavlinkBridge::peekMeshPayload(uint8_t *out, size_t capacity, uint32_t now)
{
    return transport.peekOutbound(out, min(capacity, MAX_CHUNK), now);
}

void MavlinkBridge::commitMeshPayload(size_t len, uint32_t now)
{
    const uint32_t ackSentBefore = transport.getCounters().commandAckFramesSent;
    if (transport.commitOutbound(len)) {
        stats.meshTxBytes += len;
        if (transport.getCounters().commandAckFramesSent != ackSentBefore)
            LOG_INFO("MAVLink COMMAND_ACK mesh transmission committed");
    }
    restartCoalesceWindow(now);
}

void MavlinkBridge::noteSubmitted(PacketId id, NodeNum dest, bool wantAck, size_t len, uint32_t now)
{
    LOG_DEBUG("MAVLink mesh packet 0x%08x submitted to 0x%08x want_ack=%d bytes=%u", id, dest, wantAck, (unsigned)len);
    recentIds[recentIdNext] = id;
    recentIdNext = (recentIdNext + 1) % RECENT_PACKET_IDS;
    if (!wantAck)
        return;
    InFlight *slot = nullptr;
    for (auto &record : inFlight) {
        if (!record.used) {
            slot = &record;
            break;
        }
    }
    if (!slot) {
        // Every record busy: keep tracking the newest packets, count what fell off.
        slot = &inFlight[0];
        for (auto &record : inFlight) {
            if ((int32_t)(record.submittedMs - slot->submittedMs) < 0)
                slot = &record;
        }
        stats.inFlightOverflow++;
        LOG_WARN("MAVLink in-flight table full; untracking packet 0x%08x to 0x%08x", slot->id, slot->dest);
    }
    slot->used = true;
    slot->id = id;
    slot->dest = dest;
    slot->bytes = (uint16_t)len;
    slot->submittedMs = now;
}

void MavlinkBridge::handleRoutingResult(PacketId requestId, NodeNum from, meshtastic_Routing_Error error, uint32_t now)
{
    InFlight *record = nullptr;
    for (auto &candidate : inFlight) {
        if (candidate.used && candidate.id == requestId) {
            record = &candidate;
            break;
        }
    }
    if (!record)
        return;

    const uint32_t latency = now - record->submittedMs;
    if (error != meshtastic_Routing_Error_NONE) {
        // Not treated as route loss: on a direct link MAX_RETRANSMIT is usually a lost ACK for data
        // that arrived. Routes expire only through silence.
        stats.meshPacketsFailed++;
        LOG_WARN("MAVLink mesh packet 0x%08x to 0x%08x: no routing ACK after %u ms, error=%d", requestId, record->dest,
                 (unsigned)latency, (int)error);
        record->used = false;
        return;
    }
    if (from != record->dest) {
        // A relay rebroadcasting our packet; ReliableRouter stops retrying, the destination's ACK may still come.
        stats.meshPacketsImplicitAck++;
        LOG_INFO("MAVLink mesh packet 0x%08x to 0x%08x implicit ACK via 0x%08x after %u ms", requestId, record->dest, from,
                 (unsigned)latency);
        return;
    }
    stats.meshPacketsAcked++;
    stats.lastAckLatencyMs = latency;
    stats.maxAckLatencyMs = max(stats.maxAckLatencyMs, latency);
    LOG_INFO("MAVLink mesh packet 0x%08x ACK from 0x%08x after %u ms (%u bytes)", requestId, from, (unsigned)latency,
             (unsigned)record->bytes);
    record->used = false;
}

size_t MavlinkBridge::inFlightCount() const
{
    size_t n = 0;
    for (const auto &record : inFlight)
        n += record.used ? 1 : 0;
    return n;
}

size_t MavlinkBridge::routerQueuedCount() const
{
    if (!router)
        return 0;
    size_t n = 0;
    for (PacketId id : recentIds)
        n += (id && router->findInTxQueue(nodeDB->getNodeNum(), id)) ? 1 : 0;
    return n;
}

void MavlinkBridge::logQueueOverflow(uint32_t now)
{
    const auto &t = transport.getCounters();
    if (t.outboundFramesDropped == loggedDropped && t.outboundFramesEvicted == loggedEvicted)
        return;
    if (lastOverflowLogMs && (uint32_t)(now - lastOverflowLogMs) < OVERFLOW_LOG_INTERVAL_MS)
        return;
    lastOverflowLogMs = now;
    LOG_WARN("MAVLink queue full: %u new frames dropped, %u queued state/bulk frames evicted (totals %u/%u)",
             (unsigned)(t.outboundFramesDropped - loggedDropped), (unsigned)(t.outboundFramesEvicted - loggedEvicted),
             (unsigned)t.outboundFramesDropped, (unsigned)t.outboundFramesEvicted);
    loggedDropped = t.outboundFramesDropped;
    loggedEvicted = t.outboundFramesEvicted;
}

void MavlinkBridge::serviceHousekeeping(uint32_t now)
{
    for (auto &record : inFlight) {
        if (record.used && (uint32_t)(now - record.submittedMs) >= IN_FLIGHT_TIMEOUT_MS) {
            stats.meshPacketsNoResult++;
            LOG_WARN("MAVLink mesh packet 0x%08x to 0x%08x: no routing result after %u ms", record.id, record.dest,
                     (unsigned)(now - record.submittedMs));
            record.used = false;
        }
    }
    logQueueOverflow(now);
    if (everActive && (now - lastActivityMs) <= ACTIVITY_WINDOW_MS &&
        (lastStatsLogMs == 0 || (uint32_t)(now - lastStatsLogMs) >= STATS_LOG_INTERVAL_MS)) {
        lastStatsLogMs = now;
        logStats(now);
    }
}

void MavlinkBridge::logStats(uint32_t now)
{
    const auto &s = getStats();
    const auto &t = transport.getCounters();
    // Several short lines: the firmware log buffer truncates anything much past 190 characters.
    LOG_INFO("MAVLink tx: queue=%u/%u router=%u frames=%u unicast=%u broadcast=%u", (unsigned)transport.outboundCount(),
             (unsigned)MavlinkMeshTransport::TX_QUEUE_DEPTH, (unsigned)routerQueuedCount(), (unsigned)t.outboundFramesSent,
             (unsigned)t.unicastPackets, (unsigned)t.broadcastPackets);
    LOG_INFO("MAVLink acks: requested=%u acked=%u implicit=%u failed=%u noresult=%u inflight=%u ms=%u/%u",
             (unsigned)t.ackRequestedPackets, (unsigned)s.meshPacketsAcked, (unsigned)s.meshPacketsImplicitAck,
             (unsigned)s.meshPacketsFailed, (unsigned)s.meshPacketsNoResult, (unsigned)inFlightCount(),
             (unsigned)s.lastAckLatencyMs, (unsigned)s.maxAckLatencyMs);
    LOG_INFO("MAVLink queue: uart_in=%u replaced=%u evicted=%u dropped=%u", (unsigned)s.uartRxBytes,
             (unsigned)t.stateFramesReplaced, (unsigned)t.outboundFramesEvicted, (unsigned)t.outboundFramesDropped);
    LOG_INFO("MAVLink rx: frames=%u dropped=%u malformed=%u reasm_timeout=%u uart_stall=%u", (unsigned)t.reassembledFrames,
             (unsigned)t.inboundFramesDropped, (unsigned)t.malformedPayloads, (unsigned)t.reassemblyTimeouts,
             (unsigned)s.uartTxStallDrops);
    const auto *routes = transport.getRoutes();
    for (size_t i = 0; i < MavlinkMeshTransport::ROUTE_SLOTS; i++) {
        if (transport.routeLive(routes[i], now))
            LOG_INFO("MAVLink route: sysid %u via 0x%08x, %d hops, heard %u ms ago", (unsigned)routes[i].sysid, routes[i].node,
                     (int)routes[i].hops, (unsigned)(now - routes[i].lastSeenMs));
    }
}

void MavlinkBridge::restartCoalesceWindow(uint32_t now)
{
    if (transport.hasOutbound()) {
        outboundPending = true;
        outboundSinceMs = now;
    } else {
        outboundPending = false;
    }
}

void MavlinkBridge::ingestMeshPayload(NodeNum source, const uint8_t *data, size_t len, int32_t rxRssi, float rxSnr,
                                      int8_t hopsAway)
{
    if (!data || !len)
        return;
    const auto before = transport.getCounters();
    stats.meshRxBytes += len;
    lastActivityMs = millis();
    everActive = true;
    if (rxRssi != 0 || rxSnr != 0) {
        lastRxRssi = rxRssi;
        lastRxSnr = rxSnr;
        haveRxMetadata = true;
    }
    transport.ingestMeshPayload(source, data, len, millis(), hopsAway);
    const auto &after = transport.getCounters();
    if (after.commandAckFramesReassembled != before.commandAckFramesReassembled) {
        LOG_INFO("MAVLink COMMAND_ACK mesh reassembled from node 0x%08x", source);
    }
    if (after.commandAckInboundDrops != before.commandAckInboundDrops) {
        LOG_WARN("MAVLink COMMAND_ACK dropped from inbound queue, source node 0x%08x", source);
    }
}

void MavlinkBridge::processOutput(uint32_t now)
{
    flushPending(now);
    while (pendingLen == 0) {
        size_t len = 0;
        if (!transport.popInbound(pendingFrame, sizeof(pendingFrame), len))
            break;
        pendingLen = len;
        pendingOff = 0;
        pendingSinceMs = now;
        flushPending(now);
        if (pendingLen != 0)
            break;
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
#if MESHTASTIC_MAVLINK_UDP
        if (mavlinkUdpServer)
            mavlinkUdpServer->writeFrame(pendingFrame, pendingLen, now);
#endif
        uint16_t command = 0;
        uint8_t result = 0;
        uint8_t sysid = 0;
        uint8_t compid = 0;
        uint8_t seq = 0;
        if (decodeCommandAckFrame(pendingFrame, pendingLen, command, result, sysid, compid, seq)) {
            stats.commandAckLocalDelivery++;
            logCommandAckFrame("local endpoint delivered", pendingFrame, pendingLen);
        }
        pendingLen = pendingOff = 0;
        stats.framesToUart++;
    } else if ((now - pendingSinceMs) >= UART_TX_STALL_MS) {
        uint16_t command = 0;
        uint8_t result = 0;
        uint8_t sysid = 0;
        uint8_t compid = 0;
        uint8_t seq = 0;
        if (decodeCommandAckFrame(pendingFrame, pendingLen, command, result, sysid, compid, seq))
            LOG_WARN("MAVLink COMMAND_ACK local endpoint stalled and dropped");
        stats.uartTxStallDrops++;
        pendingLen = pendingOff = 0;
    }
}

void MavlinkBridge::serviceFlowControl(uint32_t now)
{
#ifndef MESHTASTIC_MAVLINK_NO_RADIO_STATUS
    if (pendingLen)
        return;
    if (!everActive || (now - lastActivityMs) > ACTIVITY_WINDOW_MS)
        return;
    if ((now - lastRadioStatusMs) < RADIO_STATUS_INTERVAL_MS)
        return;
    lastRadioStatusMs = now;

    mavlink_radio_status_t rs{};
    // The tightest of: our frame queue, Meshtastic's TX queue, and acknowledged packets still awaiting a result.
    uint8_t txbuf = transport.outboundFreePercent();
    if (router) {
        const meshtastic_QueueStatus qs = router->getQueueStatus();
        if (qs.maxlen)
            txbuf = min(txbuf, (uint8_t)((qs.free * 100U) / qs.maxlen));
    }
    rs.txbuf = min(txbuf, (uint8_t)(((IN_FLIGHT_SLOTS - inFlightCount()) * 100U) / IN_FLIGHT_SLOTS));
    rs.rssi = haveRxMetadata ? (uint8_t)constrain(lastRxRssi + 127, 0, 254) : UINT8_MAX;
    rs.noise = haveRxMetadata ? (uint8_t)constrain((int32_t)(lastRxRssi - lastRxSnr) + 127, 0, 254) : UINT8_MAX;
    rs.remrssi = UINT8_MAX;
    rs.remnoise = UINT8_MAX;
    const auto &transportStats = transport.getCounters();
    rs.rxerrors = (uint16_t)min(transportStats.corruptFrames + transportStats.framingErrors + transportStats.malformedPayloads,
                                (uint32_t)UINT16_MAX);
    rs.fixed = 0;

    mavlink_message_t msg;
    mavlink_msg_radio_status_encode(MESHTASTIC_MAVLINK_RADIO_STATUS_SYSID, MAV_COMP_ID_TELEMETRY_RADIO, &msg, &rs);
    queueFrame(msg, now);
    flushPending(now);
    stats.radioStatusSent++;
#endif
}

const MavlinkBridgeStats &MavlinkBridge::getStats()
{
    const auto &transportStats = transport.getCounters();
    stats.framesToMesh = transportStats.outboundFramesSent;
    stats.framesFromMesh = transportStats.reassembledFrames;
    stats.corruptFrames = transportStats.corruptFrames;
    stats.framingErrors = transportStats.framingErrors;
    stats.inputOverflowBytes = transportStats.outboundDropBytes;
    stats.outputOverflowBytes = transportStats.inboundDropBytes;
    stats.malformedMeshPayloads = transportStats.malformedPayloads;
    stats.duplicateFragments = transportStats.duplicateFragments;
    stats.reassemblyTimeouts = transportStats.reassemblyTimeouts;
    stats.reassemblyEvictions = transportStats.reassemblyEvictions;
    stats.commandAckFramesQueued = transportStats.commandAckFramesQueued;
    stats.commandAckFramesSent = transportStats.commandAckFramesSent;
    stats.commandAckOutboundDrops = transportStats.commandAckOutboundDrops;
    stats.commandAckFramesReassembled = transportStats.commandAckFramesReassembled;
    stats.commandAckInboundDrops = transportStats.commandAckInboundDrops;
    stats.inputHighWater = transportStats.outboundHighWater;
    stats.outputHighWater = transportStats.inboundHighWater;
    return stats;
}

void MavlinkBridge::snoopSerialBytes(const uint8_t *data, size_t len, uint32_t now)
{
    for (size_t i = 0; i < len; i++) {
        mavlink_message_t msg;
        mavlink_status_t status;
        uint8_t res = mavlink_frame_char_buffer(&snoopWorking, &snoopStatus, data[i], &msg, &status);
        if (res == MAVLINK_FRAMING_OK) {
            if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                mavlink_command_ack_t ack;
                mavlink_msg_command_ack_decode(&msg, &ack);
                stats.commandAckLocalIngress++;
                LOG_INFO("MAVLink COMMAND_ACK local ingress command=%u result=%u sysid=%u compid=%u seq=%u",
                         (unsigned)ack.command, (unsigned)ack.result, (unsigned)msg.sysid, (unsigned)msg.compid,
                         (unsigned)msg.seq);
            }
            handleSnoopedMessage(msg, now);
        }
    }
}

void MavlinkBridge::handleSnoopedMessage(const mavlink_message_t &msg, uint32_t now)
{
    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        snoopHeartbeat(msg);
        return;
    }
    // Only the locally learned airframe may populate this node's position and battery state.
    // Other systems can share the mesh and local MAVLink bus without becoming this node's aircraft.
    if (role != MavlinkRole::AIR || !fromAutopilot(msg))
        return;
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
    case MAVLINK_MSG_ID_GPS_RAW_INT:
        snoopPosition(msg, now);
        break;
    case MAVLINK_MSG_ID_BATTERY_STATUS:
    case MAVLINK_MSG_ID_SYS_STATUS:
        snoopBattery(msg, now);
        break;
    case MAVLINK_MSG_ID_HIGH_LATENCY2:
        snoopHighLatency2(msg, now);
        break;
    default:
        break;
    }
}

void MavlinkBridge::snoopHeartbeat(const mavlink_message_t &msg)
{
#if MESHTASTIC_MAVLINK_ROLE != 0
    return;
#endif
    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(&msg, &hb);
    if (haveAutopilot) {
        if (msg.sysid == autopilotSysid)
            role = MavlinkRole::AIR;
        return;
    }
    if (hb.type == MAV_TYPE_GCS) {
        role = MavlinkRole::GROUND;
        return;
    }
    if (hb.autopilot != MAV_AUTOPILOT_INVALID) {
        haveAutopilot = true;
        autopilotSysid = msg.sysid;
        autopilotCompid = msg.compid;
        role = MavlinkRole::AIR;
        LOG_INFO("MAVLink autopilot learned: sysid %u compid %u, role AIR", msg.sysid, msg.compid);
    }
}

bool MavlinkBridge::fromAutopilot(const mavlink_message_t &msg) const
{
    // sysid identifies the airframe. Components belonging to that system may legitimately
    // emit position or battery messages; a different sysid is a different airframe.
    return !haveAutopilot || msg.sysid == autopilotSysid;
}

void MavlinkBridge::snoopPosition(const mavlink_message_t &msg, uint32_t now)
{
    if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        mavlink_global_position_int_t gp;
        mavlink_msg_global_position_int_decode(&msg, &gp);
        posSnap.latI = gp.lat;
        posSnap.lonI = gp.lon;
        posSnap.altM = gp.alt / 1000;
        float vx = gp.vx, vy = gp.vy;
        posSnap.groundSpeedKmh = (uint32_t)(sqrtf(vx * vx + vy * vy) * 0.036f + 0.5f);
        posSnap.hasGroundSpeed = true;
        if (gp.hdg != UINT16_MAX && gp.hdg <= 36000) {
            posSnap.groundTrack1e5 = (uint32_t)gp.hdg * 1000;
            posSnap.hasGroundTrack = true;
        }
        posSnap.hasFix = true;
        lastGlobalPosMs = now;
        positionDirty = true;
    } else {
        mavlink_gps_raw_int_t g;
        mavlink_msg_gps_raw_int_decode(&msg, &g);
        posSnap.fixType = g.fix_type;
        if (g.satellites_visible != UINT8_MAX) {
            posSnap.sats = g.satellites_visible;
            posSnap.hasSats = true;
        }
        if ((now - lastGlobalPosMs) > GLOBAL_POS_FRESH_MS && g.fix_type >= 2) {
            posSnap.latI = g.lat;
            posSnap.lonI = g.lon;
            posSnap.altM = g.alt / 1000;
            posSnap.hasFix = true;
            if (g.vel != UINT16_MAX) {
                posSnap.groundSpeedKmh = (uint32_t)(g.vel * 0.036f + 0.5f);
                posSnap.hasGroundSpeed = true;
            }
            if (g.cog != UINT16_MAX && g.cog <= 36000) {
                posSnap.groundTrack1e5 = (uint32_t)g.cog * 1000;
                posSnap.hasGroundTrack = true;
            }
        }
        positionDirty = true;
    }
}

void MavlinkBridge::snoopBattery(const mavlink_message_t &msg, uint32_t now)
{
    if (msg.msgid == MAVLINK_MSG_ID_BATTERY_STATUS) {
        mavlink_battery_status_t b;
        mavlink_msg_battery_status_decode(&msg, &b);
        if (!haveBatteryId) {
            haveBatteryId = true;
            batteryId = b.id;
        } else if (b.id != batteryId) {
            return;
        }
        uint32_t mvSum = 0;
        bool anyCell = false;
        for (size_t i = 0; i < 10; i++) {
            if (b.voltages[i] != UINT16_MAX) {
                mvSum += b.voltages[i];
                anyCell = true;
            }
        }
        for (size_t i = 0; i < 4; i++) {
            if (b.voltages_ext[i] != 0 && b.voltages_ext[i] != UINT16_MAX) {
                mvSum += b.voltages_ext[i];
                anyCell = true;
            }
        }
        battSnap.hasVoltage = anyCell;
        if (anyCell) {
            battSnap.voltage = mvSum / 1000.0f;
            lastBatteryVoltageMs = now;
        }
        battSnap.hasLevel = (b.battery_remaining >= 0);
        if (battSnap.hasLevel) {
            battSnap.level = (uint8_t)min((int)b.battery_remaining, 100);
            lastBatteryLevelMs = now;
        }
        lastBatteryStatusMs = now;
    } else {
        if (lastBatteryStatusMs && (now - lastBatteryStatusMs) <= SYS_STATUS_DEFER_MS)
            return;
        mavlink_sys_status_t s;
        mavlink_msg_sys_status_decode(&msg, &s);
        battSnap.hasVoltage = (s.voltage_battery != UINT16_MAX);
        if (battSnap.hasVoltage) {
            battSnap.voltage = s.voltage_battery / 1000.0f;
            lastBatteryVoltageMs = now;
        }
        battSnap.hasLevel = (s.battery_remaining >= 0);
        if (battSnap.hasLevel) {
            battSnap.level = (uint8_t)min((int)s.battery_remaining, 100);
            lastBatteryLevelMs = now;
        }
    }
}

void MavlinkBridge::snoopHighLatency2(const mavlink_message_t &msg, uint32_t now)
{
    mavlink_high_latency2_t hl;
    mavlink_msg_high_latency2_decode(&msg, &hl);

    posSnap.latI = hl.latitude;
    posSnap.lonI = hl.longitude;
    posSnap.altM = hl.altitude;
    posSnap.hasFix = true;
    if (hl.heading <= 180) {
        posSnap.groundTrack1e5 = (uint32_t)hl.heading * 2 * 100000;
        posSnap.hasGroundTrack = true;
    }
    lastGlobalPosMs = now;
    positionDirty = true;

    if (hl.battery >= 0) {
        battSnap.hasLevel = true;
        battSnap.level = (uint8_t)min((int)hl.battery, 100);
        lastBatteryLevelMs = now;
    }
}

bool MavlinkBridge::takePositionSnapshot(uint32_t now, MavlinkPositionSnapshot &out)
{
    if (!positionDirty)
        return false;
    if (lastPositionTakeMs && (now - lastPositionTakeMs) < POSITION_THROTTLE_MS)
        return false;
    lastPositionTakeMs = now;
    positionDirty = false;
    out = posSnap;
    return true;
}

bool MavlinkBridge::getBatterySnapshot(uint32_t now, MavlinkBatterySnapshot &out) const
{
    out = battSnap;
    out.hasLevel = lastBatteryLevelMs && (now - lastBatteryLevelMs) <= BATTERY_STALENESS_MS;
    out.hasVoltage = lastBatteryVoltageMs && (now - lastBatteryVoltageMs) <= BATTERY_STALENESS_MS;
    return out.hasLevel || out.hasVoltage;
}

#endif
