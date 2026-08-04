#pragma once

#include "MeshModule.h"
#include "Router.h"
#include "SinglePortModule.h"
#include "concurrency/OSThread.h"
#include "configuration.h"
#include <Arduino.h>
#include <functional>

#if (defined(ARCH_ESP32) || defined(ARCH_NRF52) || defined(ARCH_RP2040) || defined(ARCH_STM32WL)) &&                             \
    !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32C3)

static constexpr auto Serial_Mode_MAVLINK = meshtastic_ModuleConfig_SerialConfig_Serial_Mode_MAVLINK;

class SerialModule : public StreamAPI, private concurrency::OSThread
{
    bool firstTime = 1;
    unsigned long lastNmeaTime = millis();
    char outbuf[90] = "";

  public:
    SerialModule();

    static bool isValidConfig(const meshtastic_ModuleConfig_SerialConfig &config);

  protected:
    virtual int32_t runOnce() override;

    virtual bool checkIsConnected() override;

  private:
    uint32_t getBaudRate();
    void sendTelemetry(meshtastic_Telemetry m);
    void processWXSerial();
};

extern SerialModule *serialModule;

class SerialModuleRadio : public SinglePortModule
{
    uint32_t lastRxID = 0;
    char outbuf[90] = "";

  public:
    SerialModuleRadio();

    void sendPayload(NodeNum dest = NODENUM_BROADCAST, bool wantReplies = false);

#if !MESHTASTIC_EXCLUDE_MAVLINK
    /// Send one frame-aware MAVLink transport packet. The initial any-to-any policy broadcasts it.
    bool sendMavlinkChunk();

  private:
    uint32_t mavlinkBackoffStartMs = 0;
    uint32_t mavlinkBackoffMs = 0;
    uint32_t mavlinkChannelWarnMs = 0;

  public:
#endif

  protected:
    virtual ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;
};

extern SerialModuleRadio *serialModuleRadio;

#endif
