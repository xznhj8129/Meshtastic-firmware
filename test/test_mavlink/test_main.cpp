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
    size_t write(uint8_t) override { return 1; }
    size_t write(const uint8_t *, size_t n) override { return n; }
    int availableForWrite() override { return 1024; }
    void flush() override {}
};

static void ingestFrame(MavlinkBridge &bridge, const mavlink_message_t &msg, uint32_t now)
{
    uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(bytes, &msg);
    bridge.ingestSerialBytes(bytes, len, now);
}

static void learnAutopilot(MavlinkBridge &bridge, uint32_t now)
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC,
                               MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, MAV_STATE_ACTIVE);
    ingestFrame(bridge, msg, now);
}

void test_configured_peer_is_immediate_and_exclusive()
{
    TestStream stream;
    const NodeNum configured = 0x12345678;
    MavlinkBridge bridge(&stream, configured);
    TEST_ASSERT_EQUAL_HEX32(configured, bridge.getPeer());

    const uint8_t byte = 0xfd;
    bridge.ingestMeshPayload(0x87654321, &byte, 1);
    TEST_ASSERT_EQUAL_UINT32(1, bridge.getStats().rejectedSourceChunks);
    TEST_ASSERT_EQUAL_HEX32(configured, bridge.getPeer());
}

void test_zero_peer_keeps_first_sender_discovery()
{
    TestStream stream;
    MavlinkBridge bridge(&stream, 0);
    TEST_ASSERT_EQUAL_HEX32(0, bridge.getPeer());

    bridge.ingestMeshPayload(0x11111111, nullptr, 0);
    TEST_ASSERT_EQUAL_HEX32(0, bridge.getPeer());

    const uint8_t byte = 0xfd;
    bridge.ingestMeshPayload(0x11111111, &byte, 1);
    TEST_ASSERT_EQUAL_HEX32(0x11111111, bridge.getPeer());
    bridge.ingestMeshPayload(0x22222222, &byte, 1);
    TEST_ASSERT_EQUAL_UINT32(1, bridge.getStats().rejectedSourceChunks);
}

void test_high_latency_level_does_not_refresh_stale_voltage()
{
    TestStream stream;
    MavlinkBridge bridge(&stream, 0);
    learnAutopilot(bridge, 1000);

    mavlink_battery_status_t battery{};
    battery.id = 0;
    battery.battery_remaining = 80;
    for (auto &voltage : battery.voltages)
        voltage = UINT16_MAX;
    for (auto &voltage : battery.voltages_ext)
        voltage = UINT16_MAX;
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
    MavlinkBridge bridge(&stream, 0);
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
    RUN_TEST(test_configured_peer_is_immediate_and_exclusive);
    RUN_TEST(test_zero_peer_keeps_first_sender_discovery);
    RUN_TEST(test_high_latency_level_does_not_refresh_stale_voltage);
    RUN_TEST(test_sys_status_voltage_only_expires_independently);
#endif
    exit(UNITY_END());
}

void loop() {}
