#include "MavlinkBridge.h"

#if !MESHTASTIC_EXCLUDE_MAVLINK

#include "DebugConfiguration.h"
#include "NodeDB.h"
#if MESHTASTIC_MAVLINK_UDP
#include "MavlinkUdpServer.h"
#endif

MavlinkBridge *mavlinkBridge;

static NullStream mavlinkNullStream;

MavlinkBridge::MavlinkBridge(Stream *serial) : uart(serial ? serial : &mavlinkNullStream)
{
    if (moduleConfig.serial.peer_node)
        LOG_WARN("MAVLink mesh mode ignores serial.peer_node; routing is any-to-any");
}

MavlinkBridge::MavlinkBridge(Stream *serial, NodeNum legacyPeer) : MavlinkBridge(serial)
{
    if (legacyPeer)
        LOG_WARN("MAVLink legacy peer 0x%08x ignored; routing is any-to-any", legacyPeer);
}

void MavlinkBridge::ingestSerialBytes(const uint8_t *data, size_t len, uint32_t now)
{
    if (!data || !len)
        return;
    stats.uartRxBytes += len;
    lastActivityMs = now;
    everActive = true;
    snoopSerialBytes(data, len, now);
    transport.ingestLocalBytes(data, len);
}

bool MavlinkBridge::wantsMeshSend(uint32_t now) const
{
    (void)now;
    return transport.hasOutbound();
}

size_t MavlinkBridge::peekMeshPayload(uint8_t *out, size_t capacity)
{
    return transport.peekOutbound(out, min(capacity, MAX_CHUNK));
}

void MavlinkBridge::commitMeshPayload(size_t len, uint32_t now)
{
    (void)now;
    if (transport.commitOutbound(len))
        stats.meshTxBytes += len;
}

void MavlinkBridge::dropCurrentMeshFrame()
{
    transport.dropOutbound();
}

void MavlinkBridge::ingestMeshPayload(NodeNum source, const uint8_t *data, size_t len, int32_t rxRssi, float rxSnr)
{
    if (!data || !len)
        return;
    stats.meshRxBytes += len;
    lastActivityMs = millis();
    everActive = true;
    if (rxRssi != 0 || rxSnr != 0) {
        lastRxRssi = rxRssi;
        lastRxSnr = rxSnr;
        haveRxMetadata = true;
    }
    transport.ingestMeshPayload(source, data, len, millis());
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
        pendingLen = pendingOff = 0;
        stats.framesToUart++;
    } else if ((now - pendingSinceMs) >= UART_TX_STALL_MS) {
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
    rs.txbuf = transport.outboundFreePercent();
    rs.rssi = haveRxMetadata ? (uint8_t)constrain(lastRxRssi + 127, 0, 254) : UINT8_MAX;
    rs.noise = haveRxMetadata ? (uint8_t)constrain((int32_t)(lastRxRssi - lastRxSnr) + 127, 0, 254) : UINT8_MAX;
    rs.remrssi = UINT8_MAX;
    rs.remnoise = UINT8_MAX;
    const auto &transportStats = transport.getCounters();
    rs.rxerrors = (uint16_t)min(transportStats.corruptFrames + transportStats.framingErrors +
                                    transportStats.malformedPayloads,
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
        if (res == MAVLINK_FRAMING_OK)
            handleSnoopedMessage(msg, now);
    }
}

void MavlinkBridge::handleSnoopedMessage(const mavlink_message_t &msg, uint32_t now)
{
    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        snoopHeartbeat(msg);
        return;
    }
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
        if (msg.sysid == autopilotSysid && msg.compid == autopilotCompid)
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
    return !haveAutopilot || (msg.sysid == autopilotSysid && msg.compid == autopilotCompid);
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
