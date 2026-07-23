#pragma once

#include "MavlinkBridge.h"

#if MESHTASTIC_MAVLINK_UDP

#include <Arduino.h>
#include <WiFiUdp.h>

/**
 * MAVLink UDP server endpoint (MAVLINK.md section 4.4). Binds port 14550 when the
 * network (WiFi STA/AP or ESP-IDF Ethernet) is up and waits for a client: the first
 * datagram registers the client by source IP/port, later datagrams refresh or
 * re-point the registration, and 30 s of silence expires it. One client at a time.
 *
 * Ingress datagram bytes enter the bridge input FIFO like UART bytes; every frame
 * completed on the local endpoint is teed back here as one datagram.
 *
 * Threading: same invariant as MavlinkBridge, main cooperative scheduler only.
 */
class MavlinkUdpServer
{
  public:
    static constexpr uint16_t PORT = 14550; // MAVLink GCS standard
    static constexpr uint32_t CLIENT_TIMEOUT_MS = 30000;

    void poll(uint32_t now); // socket lifecycle + datagram ingress
    void writeFrame(const uint8_t *data, size_t len, uint32_t now);
    bool hasClient(uint32_t now) const { return clientKnown && (now - lastClientMs) <= CLIENT_TIMEOUT_MS; }

  private:
    bool networkUp() const;

    WiFiUDP udp;
    bool open = false;
    bool clientKnown = false;
    IPAddress clientIp;
    uint16_t clientPort = 0;
    uint32_t lastClientMs = 0;

    uint32_t rxDatagrams = 0;
    uint32_t txDatagrams = 0;
    uint32_t rxBytes = 0;
    uint32_t txBytes = 0;
};

// Created by SerialModule alongside MavlinkBridge; null otherwise
extern MavlinkUdpServer *mavlinkUdpServer;

#endif
