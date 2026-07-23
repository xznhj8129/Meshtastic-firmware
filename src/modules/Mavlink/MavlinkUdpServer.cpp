#include "MavlinkUdpServer.h"

#if MESHTASTIC_MAVLINK_UDP

#include "DebugConfiguration.h"
#if HAS_WIFI
#include <WiFi.h>
#endif
#if HAS_ETHERNET && defined(ARCH_ESP32)
#include <ETH.h>
#endif
#if HAS_ETHERNET && defined(USE_CH390D)
#include "ESP32_CH390.h"
#endif

MavlinkUdpServer *mavlinkUdpServer;

bool MavlinkUdpServer::networkUp() const
{
    // Mirrors MQTT's isConnectedToNetwork(), plus WiFi AP mode
#ifdef USE_WS5500
    if (ETH.connected())
        return true;
#elif defined(USE_CH390D)
    if (CH390.isConnected())
        return true;
#endif
#if HAS_WIFI
    if (WiFi.isConnected())
        return true;
    wifi_mode_t mode = WiFi.getMode();
    return mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA;
#else
    return false;
#endif
}

void MavlinkUdpServer::poll(uint32_t now)
{
    if (!networkUp()) {
        if (open) {
            udp.stop();
            open = false;
            clientKnown = false;
        }
        return;
    }
    if (!open) {
        open = udp.begin(PORT) != 0;
        if (open)
            LOG_INFO("MAVLink UDP server listening on port %u", PORT);
        return;
    }
    if (clientKnown && (now - lastClientMs) > CLIENT_TIMEOUT_MS) {
        clientKnown = false;
        LOG_INFO("MAVLink UDP client timed out");
    }
    while (udp.parsePacket() > 0) {
        IPAddress srcIp = udp.remoteIP();
        uint16_t srcPort = udp.remotePort();
        if (!clientKnown || srcIp != clientIp || srcPort != clientPort) {
            clientIp = srcIp;
            clientPort = srcPort;
            clientKnown = true;
            LOG_INFO("MAVLink UDP client registered: %s:%u", srcIp.toString().c_str(), srcPort);
        }
        lastClientMs = now;
        uint8_t buf[128];
        while (udp.available() > 0) { // drain the whole datagram, any size
            int n = udp.read(buf, sizeof(buf));
            if (n <= 0)
                break;
            rxDatagrams++;
            rxBytes += n;
            if (mavlinkBridge)
                mavlinkBridge->ingestSerialBytes(buf, n, now);
        }
    }
}

void MavlinkUdpServer::writeFrame(const uint8_t *data, size_t len, uint32_t now)
{
    if (!open || !hasClient(now))
        return;
    if (udp.beginPacket(clientIp, clientPort)) {
        udp.write(data, len);
        if (udp.endPacket()) {
            txDatagrams++;
            txBytes += len;
        }
    }
}

#endif
