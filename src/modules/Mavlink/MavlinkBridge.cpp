#include "MavlinkBridge.h"

#if !MESHTASTIC_EXCLUDE_MAVLINK

#include "DebugConfiguration.h"
#if MESHTASTIC_MAVLINK_UDP
#include "MavlinkUdpServer.h"
#endif
#include <Throttle.h>

MavlinkBridge *mavlinkBridge;

static NullStream mavlinkNullStream;

MavlinkBridge::MavlinkBridge(Stream *serial) : uart(serial ? serial : &mavlinkNullStream) {}

void MavlinkBridge::ingestSerialBytes(const uint8_t *data, size_t len, uint32_t now)
{
    if (!len)
        return;
    stats.uartRxBytes += len;
    lastActivityMs = now;
    everActive = true;
    snoopSerialBytes(data, len, now);
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

void MavlinkBridge::ingestMeshPayload(NodeNum source, const uint8_t *data, size_t len, int32_t rxRssi, float rxSnr)
{
    if (!len)
        return; // an empty chunk must not capture the peer lock
    if (!acceptSource(source)) {
        stats.rejectedSourceChunks++;
        return;
    }
    stats.meshRxBytes += len;
    lastActivityMs = millis();
    everActive = true;
    if (rxRssi != 0 || rxSnr != 0) {
        lastRxRssi = rxRssi;
        lastRxSnr = rxSnr;
        haveRxMetadata = true;
    }
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

        if (meshRxStatus.parse_state == MAVLINK_PARSE_STATE_IDLE) {
            rawLen = 0;
            rawOverflow = false;
            pendingSignedBadCrc = false;
        }
        if (rawLen < sizeof(rawFrame))
            rawFrame[rawLen++] = c;
        else
            rawOverflow = true;

        mavlink_message_t msg;
        mavlink_status_t status;
        uint8_t res = mavlink_frame_char_buffer(&meshRxWorking, &meshRxStatus, c, &msg, &status);
        stats.framingErrors += status.packet_rx_drop_count; // parse errors on this byte

        if (res == MAVLINK_FRAMING_OK) {
            if (pendingSignedBadCrc) {
                // Signed frame whose CRC failed at CRC2; the parser forgot that once the
                // signature completed. Apply the known/unknown rule to the recorded msgid.
                if (mavlink_get_msg_entry(pendingBadCrcMsgid) == nullptr)
                    emitFrame(now); // unknown dialect - forward the wire bytes untouched
                else
                    stats.corruptFrames++;
            } else {
                emitFrame(now); // valid frame - forward the wire bytes
            }
        } else if (res == MAVLINK_FRAMING_BAD_CRC) {
            if (meshRxStatus.parse_state == MAVLINK_PARSE_STATE_SIGNATURE_WAIT) {
                // Signed frame, 13 signature bytes still in flight; msg is not valid yet.
                pendingSignedBadCrc = true;
                pendingBadCrcMsgid = meshRxWorking.msgid; // header is parsed, msgid is valid
            } else if (mavlink_get_msg_entry(msg.msgid) == nullptr) {
                // No CRC_EXTRA entry: unknown-dialect message, wire checksum preserved in
                // the raw capture. A known msgid with bad CRC is real corruption.
                emitFrame(now);
            } else {
                stats.corruptFrames++;
            }
        }
        flushPending(now);
    }
}

void MavlinkBridge::emitFrame(uint32_t now)
{
    if (rawOverflow || rawLen == 0) {
        stats.framingErrors++; // can't reconstruct the exact wire bytes
        return;
    }
    memcpy(pendingFrame, rawFrame, rawLen);
    pendingLen = rawLen;
    pendingOff = 0;
    pendingSinceMs = now;
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
        // One completed frame per datagram to the registered UDP client (MAVLINK.md section 4.4)
        if (mavlinkUdpServer)
            mavlinkUdpServer->writeFrame(pendingFrame, pendingLen, now);
#endif
        pendingLen = pendingOff = 0;
        stats.framesToUart++;
    } else if ((now - pendingSinceMs) >= UART_TX_STALL_MS) {
        // UART TX has been full for too long (reader gone?) - drop rather than deadlock
        stats.uartTxStallDrops++;
        pendingLen = pendingOff = 0;
    }
}

void MavlinkBridge::serviceFlowControl(uint32_t now)
{
#ifndef MESHTASTIC_MAVLINK_NO_RADIO_STATUS
    if (pendingLen)
        return; // never interleave with a partially written frame
    if (!everActive || (now - lastActivityMs) > ACTIVITY_WINDOW_MS)
        return; // don't spam a silent port
    if ((now - lastRadioStatusMs) < RADIO_STATUS_INTERVAL_MS)
        return;
    lastRadioStatusMs = now;

    mavlink_radio_status_t rs{};
    rs.txbuf = (uint8_t)((inputFifo.freeSpace() * 100U) / inputFifo.capacity());
    // SiK-style mapping of the final LoRa hop; UINT8_MAX = unknown per the RADIO_STATUS spec
    rs.rssi = haveRxMetadata ? (uint8_t)constrain(lastRxRssi + 127, 0, 254) : UINT8_MAX;
    rs.noise = haveRxMetadata ? (uint8_t)constrain((int32_t)(lastRxRssi - lastRxSnr) + 127, 0, 254) : UINT8_MAX;
    rs.remrssi = UINT8_MAX;
    rs.remnoise = UINT8_MAX;
    rs.rxerrors = (uint16_t)min(stats.corruptFrames + stats.framingErrors, (uint32_t)UINT16_MAX);
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
    stats.inputHighWater = inputFifo.highWater();
    stats.outputHighWater = outputFifo.highWater();
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
        return; // position and battery snooping apply to the aircraft bus only
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
    case MAVLINK_MSG_ID_GPS_RAW_INT:
        snoopPosition(msg, now);
        break;
    case MAVLINK_MSG_ID_BATTERY_STATUS:
    case MAVLINK_MSG_ID_SYS_STATUS:
        snoopBattery(msg, now);
        break;
    default:
        break;
    }
}

void MavlinkBridge::snoopHeartbeat(const mavlink_message_t &msg)
{
#if MESHTASTIC_MAVLINK_ROLE != 0
    return; // role is compile-time forced; heartbeats can't change it
#endif
    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(&msg, &hb);
    if (haveAutopilot) {
        // Only the learned autopilot may reaffirm AIR; gimbal/GCS heartbeats on the vehicle
        // bus must not flap the role.
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
        posSnap.altM = gp.alt / 1000; // mm to m
        // vx/vy are cm/s; the firmware's ground_speed convention is km/h (see GPS.cpp)
        float vx = gp.vx, vy = gp.vy;
        posSnap.groundSpeedKmh = (uint32_t)(sqrtf(vx * vx + vy * vy) * 0.036f + 0.5f);
        posSnap.hasGroundSpeed = true;
        if (gp.hdg != UINT16_MAX && gp.hdg <= 36000) {
            posSnap.groundTrack1e5 = (uint32_t)gp.hdg * 1000; // centideg to deg x 1e5
            posSnap.hasGroundTrack = true;
        }
        posSnap.hasFix = true;
        lastGlobalPosMs = now;
        positionDirty = true;
    } else { // GPS_RAW_INT: ancillary fix/sats, full fallback only when GLOBAL_POSITION_INT is stale
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
        if (!haveBatteryId) { // first-seen battery id sticks; others are ignored
            haveBatteryId = true;
            batteryId = b.id;
        } else if (b.id != batteryId) {
            return;
        }
        // voltages[]: UINT16_MAX = invalid; cell 0 may carry the overall voltage. voltages_ext[]:
        // 0 = not supported (zero-truncatable), so both 0 and UINT16_MAX are skipped there.
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
        battSnap.voltage = mvSum / 1000.0f;
        battSnap.hasLevel = (b.battery_remaining >= 0); // -1 = autopilot does not estimate
        if (battSnap.hasLevel)
            battSnap.level = (uint8_t)min((int)b.battery_remaining, 100);
        lastBatteryStatusMs = now;
        lastBatteryMs = now;
    } else { // SYS_STATUS: fallback only when no recent BATTERY_STATUS
        if (lastBatteryStatusMs && (now - lastBatteryStatusMs) <= SYS_STATUS_DEFER_MS)
            return;
        mavlink_sys_status_t s;
        mavlink_msg_sys_status_decode(&msg, &s);
        battSnap.hasVoltage = (s.voltage_battery != UINT16_MAX);
        if (battSnap.hasVoltage)
            battSnap.voltage = s.voltage_battery / 1000.0f;
        battSnap.hasLevel = (s.battery_remaining >= 0);
        if (battSnap.hasLevel)
            battSnap.level = (uint8_t)min((int)s.battery_remaining, 100);
        lastBatteryMs = now;
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
    if (!lastBatteryMs || (now - lastBatteryMs) > BATTERY_STALENESS_MS)
        return false;
    out = battSnap;
    return true;
}

#endif
