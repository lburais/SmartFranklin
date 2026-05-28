#pragma once

#include <Arduino.h>
#include <WiFi.h>

#include "config_store.h"

class Wifi {
public:
    bool init();
    bool process();
    bool isInitialized() const;

private:
    bool m_initialized = false;
    bool m_captivePortalStarted = false;
    String m_hostname = CONFIG.hostname;
    const char* const m_tag = "WIFI";

    unsigned long m_lastStatusMs = 0;
    unsigned long m_lastReconnectAttemptMs = 0;
    unsigned long m_staConnectedSinceMs = 0;
    unsigned long m_lastExternalCheckMs = 0;

    bool m_mdnsStarted = false;
    wl_status_t m_lastLoggedStaStatus = WL_IDLE_STATUS;

    static constexpr unsigned long kReconnectIntervalMs = 15000;
    static constexpr unsigned long kStatusIntervalMs = 60000;
    static constexpr unsigned long kExternalCheckIntervalMs = 30000;

    //bool waitForWiFiApReady(uint32_t timeoutMs);
};

extern Wifi WIFI_TASK;
