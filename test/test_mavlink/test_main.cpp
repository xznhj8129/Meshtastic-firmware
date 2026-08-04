#include "MeshTypes.h" // include before TestUtil.h
#include "TestUtil.h"
#include <unity.h>

#if !MESHTASTIC_EXCLUDE_MAVLINK

#include "modules/Mavlink/MavlinkBridge.h"

class TestStream : public Stream
{
  public:
    int available() override { return 0; }
    int read() override { return -1; }
    int peek() override { return -1; }
    size_t write(uint8_t c) override
    {
        if (writtenLen >= sizeof(written))
            return 0;
        written[writtenLen++] = c;
        return 1;
    }
    size_t write(const uint8_t *data, size_t n) override
    {
        size_t accepted = min(n, sizeof(written) - writtenLen);
        memcpy(written + writtenLen, data, accepted);
        writtenLen += accepted;
        return accepted;
    }
    int availableForWrite() override { return sizeof(written) - writtenLen; }
    void flush() override {}

    uint8_t written[2048]{};
    size_t writtenLen = 0;
};

static uint16_t serializeFrame(const mavlink_message_t &msg, uint8_t *bytes)
{
    return mavlink_msg_to_send_buffer(bytes, &msg);
}

static void ingestFrame(MavlinkBridge &bridge, const mavlink_message_t &msg, uint32_t now)
{
    uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = serializeFrame(msg, bytes);
    bridge.ingestSerialBytes(bytes, len, now);
}

static void learnAutopilot(MavlinkBridge &bridge, uint32_t now)
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC,
                               MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, MAV_STATE_ACTIVE);
    ingestFrame(bridge, msg, now);
}

static void putU16(uint8_t *out, uint16_t value)
{
    out[0] = value & 0xff;
    out[1] = value >> 8;
}

static size_t buildMeshFragment(uint16_t frameId, uint8_t fragmentIndex, const uint8_t *frame, size_t frameLen,
                                uint8_t *out)
{
    const size_t maxData = MavlinkBridge::MAX_FRAGMENT_DATA;
    const uint8_t fragmentCount = (frameLen + maxData - 1) / maxData;
    const size_t offset = (size_t)fragmentIndex * maxData;
    const size_t dataLen = min(maxData, frameLen - offset);
    out[0] = MavlinkMeshTransport::MAGIC0;
    out[1] = MavlinkMeshTransport::MAGIC1;
    out[2] = MavlinkMeshTransport::VERSION;
    putU16(out + 3, frameId);
    out[5] = fragmentIndex;
    out[6] = fragmentCount;
    putU16(out + 7, frameLen);
    memcpy(out + MavlinkBridge::TRANSPORT_HEADER_SIZE, frame + offset, dataLen);
    return MavlinkBridge::TRANSPORT_HEADER_SIZE + dataLen;
}

void test_complete_mavlink_frame_round_trip()
{
    TestStream sourceStream;
    TestStream sinkStream;
    MavlinkBridge sender(&sourceStream);
    MavlinkBridge receiver(&sinkStream);

    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(42, MAV_COMP_ID_AUTOPILOT1, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC,
                               MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, MAV_STATE_ACTIVE);
    uint8_t expected[MAVLINK_MAX_PACKET_LEN];
    const uint16_t expectedLen = serializeFrame(msg, expected);
    sender.ingestSerialBytes(expected, expectedLen, 1000);

    TEST_ASSERT_TRUE(sender.wantsMeshSend(1000));
    uint8_t packet[meshtastic_Constants_DATA_PAYLOAD_LEN];
    const size_t packetLen = sender.peekMeshPayload(packet, sizeof(packet));
    TEST_ASSERT_GREATER_THAN(MavlinkBridge::TRANSPORT_HEADER_SIZE, packetLen);
    TEST_ASSERT_EQUAL_UINT8(MavlinkMeshTransport::MAGIC0, packet[0]);
    TEST_ASSERT_EQUAL_UINT8(MavlinkMeshTransport::MAGIC1, packet[1]);
    TEST_ASSERT_EQUAL_UINT8(MavlinkMeshTransport::VERSION, packet[2]);

    receiver.ingestMeshPayload(0x11111111, packet, packetLen);
    sender.commitMeshPayload(packetLen, 1001);
    receiver.processOutput(1002);

    TEST_ASSERT_EQUAL_UINT16(expectedLen, sinkStream.writtenLen);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, sinkStream.written, expectedLen);
    TEST_ASSERT_EQUAL_UINT32(1, sender.getStats().framesToMesh);
    TEST_ASSERT_EQUAL_UINT32(1, receiver.getStats().framesFromMesh);
}

void test_interleaved_sources_reassemble_independently()
{
    TestStream sinkStream;
    MavlinkBridge receiver(&sinkStream, 0x12345678); // legacy peer argument must not restrict sources

    const size_t frameALen = MavlinkBridge::MAX_FRAGMENT_DATA + 17;
    const size_t frameBLen = MavlinkBridge::MAX_FRAGMENT_DATA + 7;
    uint8_t frameA[MAVLINK_MAX_PACKET_LEN];
    uint8_t frameB[MAVLINK_MAX_PACKET_LEN];
    for (size_t i = 0; i < frameALen; i++)
        frameA[i] = (uint8_t)(0x20 + (i % 31));
    for (size_t i = 0; i < frameBLen; i++)
        frameB[i] = (uint8_t)(0x80 + (i % 29));

    uint8_t packet[meshtastic_Constants_DATA_PAYLOAD_LEN];
    size_t len = buildMeshFragment(10, 0, frameA, frameALen, packet);
    receiver.ingestMeshPayload(0xaaaaaaaa, packet, len);
    len = buildMeshFragment(20, 0, frameB, frameBLen, packet);
    receiver.ingestMeshPayload(0xbbbbbbbb, packet, len);
    len = buildMeshFragment(20, 1, frameB, frameBLen, packet);
    receiver.ingestMeshPayload(0xbbbbbbbb, packet, len);
    len = buildMeshFragment(10, 1, frameA, frameALen, packet);
    receiver.ingestMeshPayload(0xaaaaaaaa, packet, len);

    receiver.processOutput(2000);

    TEST_ASSERT_EQUAL(frameBLen + frameALen, sinkStream.writtenLen);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(frameB, sinkStream.written, frameBLen);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(frameA, sinkStream.written + frameBLen, frameALen);
    TEST_ASSERT_EQUAL_UINT32(2, receiver.getStats().framesFromMesh);
    TEST_ASSERT_EQUAL_UINT32(0, receiver.getStats().rejectedSourceChunks);
    TEST_ASSERT_EQUAL_HEX32(0, receiver.getPeer());
}

void test_malformed_transport_payload_is_dropped()
{
    TestStream stream;
    MavlinkBridge bridge(&stream);
    uint8_t bad[MavlinkBridge::TRANSPORT_HEADER_SIZE]{};
    bridge.ingestMeshPayload(0x11111111, bad, sizeof(bad));
    bridge.processOutput(1000);
    TEST_ASSERT_EQUAL_UINT32(1, bridge.getStats().malformedMeshPayloads);
    TEST_ASSERT_EQUAL_UINT32(0, stream.writtenLen);
}

void test_high_latency_level_does_not_refresh_stale_voltage()
{
    TestStream stream;
    MavlinkBridge bridge(&stream);
    learnAutopilot(bridge, 1000);

    mavlink_battery_status_t battery{};
    battery.id = 0;
    battery.battery_remaining = 80;
    for (size_t i = 0; i < 10; i++)
        battery.voltages[i] = UINT16_MAX;
    for (size_t i = 0; i < 4; i++)
        battery.voltages_ext[i] = UINT16_MAX;
    battery.voltages[0] = 12000;

    mavlink_message_t batteryMsg;
    mavlink_msg_battery_status_encode(1, MAV_COMP_ID_AUTOPILOT1, &batteryMsg, &battery);
    ingestFrame(bridge, batteryMsg, 2000);

    MavlinkBatterySnapshot snap;
    TEST_ASSERT_TRUE(bridge.getBatterySnapshot(2001, snap));
    TEST_ASSERT_TRUE(snap.hasLevel);
    TEST_ASSERT_TRUE(snap.hasVoltage);
    TEST_ASSERT_EQUAL_UINT8(80, snap.level);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 12.0f, snap.voltage);

    mavlink_high_latency2_t hl{};
    hl.battery = 70;
    mavlink_message_t hlMsg;
    mavlink_msg_high_latency2_encode(1, MAV_COMP_ID_AUTOPILOT1, &hlMsg, &hl);
    const uint32_t hlTime = 2000 + MavlinkBridge::BATTERY_STALENESS_MS - 1000;
    ingestFrame(bridge, hlMsg, hlTime);

    const uint32_t queryTime = 2000 + MavlinkBridge::BATTERY_STALENESS_MS + 1000;
    TEST_ASSERT_TRUE(bridge.getBatterySnapshot(queryTime, snap));
    TEST_ASSERT_TRUE(snap.hasLevel);
    TEST_ASSERT_FALSE(snap.hasVoltage);
    TEST_ASSERT_EQUAL_UINT8(70, snap.level);
}

void test_sys_status_voltage_only_expires_independently()
{
    TestStream stream;
    MavlinkBridge bridge(&stream);
    learnAutopilot(bridge, 1000);

    mavlink_sys_status_t status{};
    status.voltage_battery = 12345;
    status.battery_remaining = -1;
    mavlink_message_t msg;
    mavlink_msg_sys_status_encode(1, MAV_COMP_ID_AUTOPILOT1, &msg, &status);
    ingestFrame(bridge, msg, 2000);

    MavlinkBatterySnapshot snap;
    TEST_ASSERT_TRUE(bridge.getBatterySnapshot(2001, snap));
    TEST_ASSERT_TRUE(snap.hasVoltage);
    TEST_ASSERT_FALSE(snap.hasLevel);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 12.345f, snap.voltage);

    TEST_ASSERT_FALSE(bridge.getBatterySnapshot(2000 + MavlinkBridge::BATTERY_STALENESS_MS + 1, snap));
    TEST_ASSERT_FALSE(snap.hasVoltage);
    TEST_ASSERT_FALSE(snap.hasLevel);
}

#endif

void setUp(void) {}
void tearDown(void) {}

void setup()
{
    initializeTestEnvironment();
    UNITY_BEGIN();
#if !MESHTASTIC_EXCLUDE_MAVLINK
    RUN_TEST(test_complete_mavlink_frame_round_trip);
    RUN_TEST(test_interleaved_sources_reassemble_independently);
    RUN_TEST(test_malformed_transport_payload_is_dropped);
    RUN_TEST(test_high_latency_level_does_not_refresh_stale_voltage);
    RUN_TEST(test_sys_status_voltage_only_expires_independently);
#endif
    exit(UNITY_END());
}

void loop() {}
