#include <ESPmDNS.h>
#include <WiFi.h>

#include "captive_portal.h"
#include "network.h"
#include "config_store.h"
#include "log.h"
#include "mqtt.h"
#include "tasks.h"

namespace {

const char* wifiStatusToString(const wl_status_t status)
{
    switch (status) {
    case WL_NO_SHIELD:       return "NO_SHIELD";
    case WL_IDLE_STATUS:     return "IDLE";
    case WL_NO_SSID_AVAIL:   return "NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED:  return "SCAN_COMPLETED";
    case WL_CONNECTED:       return "CONNECTED";
    case WL_CONNECT_FAILED:  return "CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "CONNECTION_LOST";
    case WL_DISCONNECTED:    return "DISCONNECTED";
    default:                 return "UNKNOWN";
    }
}

bool hasExternalConnectivity()
{
    WiFiClient client;
    client.setTimeout(1200);

    // Try a short TCP handshake to a stable public DNS endpoint.
    if (!client.connect("1.1.1.1", 53)) {
        return false;
    }

    client.stop();
    return true;
}

}  // namespace

Wifi WIFI_TASK;

bool Wifi::isInitialized() const
{
    return m_initialized;
}

bool Wifi::init()
{
    m_initialized = false;
    m_mdnsStarted = false;
    m_lastStatusMs = 0;
    m_lastReconnectAttemptMs = 0;
    m_staConnectedSinceMs = 0;
    m_lastExternalCheckMs = 0;
    m_lastLoggedStaStatus = WL_IDLE_STATUS;

    m_hostname = CONFIG.hostname.isEmpty() ? String("franklin") : CONFIG.hostname;

    WiFi.persistent(false);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    WiFi.mode(WIFI_AP_STA);

    if (!WiFi.setHostname(m_hostname.c_str())) {
        SF_LOGW("[%s] Failed to set hostname to %s", m_tag, m_hostname.c_str());
    } else {
        SF_LOGI("[%s] Hostname: %s", m_tag, m_hostname.c_str());
    }

    if (WiFi.softAP(CONFIG.ap_ssid.c_str(), CONFIG.ap_pass.c_str())) {
        SF_LOGI("[%s] AP %s: OK", m_tag, CONFIG.ap_ssid.c_str());
    } else {
        SF_LOGW("[%s] AP %s: FAIL", m_tag, CONFIG.ap_ssid.c_str());
        return false;
    }

    if (!CONFIG.sta_ssid.isEmpty()) {
        WiFi.begin(CONFIG.sta_ssid.c_str(), CONFIG.sta_pass.c_str());
        SF_LOGI("[%s] STA connecting to %s", m_tag, CONFIG.sta_ssid.c_str());
    }

    const uint32_t startMs = millis();
    while ((millis() - startMs) < 5000) {
        const wifi_mode_t mode = WiFi.getMode();
        const bool apModeActive = (mode == WIFI_AP || mode == WIFI_AP_STA);
        const IPAddress apIp = WiFi.softAPIP();
        const bool hasValidApIp = (apIp != INADDR_NONE) && (apIp != IPAddress(0, 0, 0, 0));

        if (apModeActive && hasValidApIp) {
            SF_LOGI("[%s] AP ready ssid:%s ip:%s", m_tag, CONFIG.ap_ssid.c_str(), apIp.toString().c_str());

            if (!m_captivePortalStarted) {
                captive_portal_start();
                m_captivePortalStarted = true;
            }

            m_initialized = true;
            return true;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    const IPAddress apIp = WiFi.softAPIP();
    SF_LOGW(
        "[%s] AP not ready ssid:%s mode:%d ip:%s", 
        m_tag,
        CONFIG.ap_ssid.c_str(), 
        static_cast<int>(WiFi.getMode()), 
        apIp.toString().c_str());
    return false;
}

bool Wifi::process()
{
    if (!m_initialized) {
        return false;
    }

    const wl_status_t staStatus = WiFi.status();

    if (staStatus != m_lastLoggedStaStatus) {
        m_lastLoggedStaStatus = staStatus;
        SF_LOGI(
            "[%s] STA status=%s sta_ip=%s ap_ip=%s", 
            m_tag,
            wifiStatusToString(staStatus),
            WiFi.localIP().toString().c_str(),
            WiFi.softAPIP().toString().c_str());
    }

    if (staStatus == WL_CONNECTED) {
        if (m_staConnectedSinceMs == 0) {
            m_staConnectedSinceMs = millis();
            SF_LOGI(
                "[%s] STA connected ssid:%s ip:%s gw:%s dns:%s rssi:%d", 
                m_tag,
                WiFi.SSID().c_str(),
                WiFi.localIP().toString().c_str(),
                WiFi.gatewayIP().toString().c_str(),
                WiFi.dnsIP().toString().c_str(),
                WiFi.RSSI());
        }
    } else {
        m_staConnectedSinceMs = 0;
        if (m_mdnsStarted) {
            MDNS.end();
            m_mdnsStarted = false;
            SF_LOGI("[%s] mDNS stopped", m_tag);
        }
    }

    if (staStatus == WL_CONNECTED && !m_mdnsStarted) {
        SF_LOGI("[%s] restart mDNS", m_tag);
        if (MDNS.begin(m_hostname.c_str())) {
            MDNS.addService("http", "tcp", 80);
            m_mdnsStarted = true;
            SF_LOGI("[%s] mDNS available at http://%s.local", m_tag, m_hostname.c_str());
        } else {
            SF_LOGW("[%s] mDNS start failed for %s.local", m_tag, m_hostname.c_str());
        }
    }

    if (staStatus == WL_CONNECTED && (millis() - m_lastExternalCheckMs > kExternalCheckIntervalMs)) {
        m_lastExternalCheckMs = millis();

        if (!hasExternalConnectivity()) {
            SF_LOGW("[%s] STA has no external connectivity, forcing reconnect", m_tag);

            if (WiFi.SSID() == CONFIG.sta_ssid) {
                WiFi.reconnect();
            } else if (!CONFIG.sta_ssid.isEmpty()) {
                WiFi.begin(CONFIG.sta_ssid.c_str(), CONFIG.sta_pass.c_str());
            }

            m_lastReconnectAttemptMs = millis();
        }
    }

    if (!CONFIG.sta_ssid.isEmpty() && staStatus != WL_CONNECTED) {
        if (millis() - m_lastReconnectAttemptMs > kReconnectIntervalMs) {
            SF_LOGW("[%s] STA reconnecting...", m_tag);

            if (WiFi.SSID() == CONFIG.sta_ssid) {
                WiFi.reconnect();
            } else {
                WiFi.begin(CONFIG.sta_ssid.c_str(), CONFIG.sta_pass.c_str());
            }

            m_lastReconnectAttemptMs = millis();
        }
    }

    if (millis() - m_lastStatusMs > kStatusIntervalMs) {
        SF_LOGI("[%s] publishing:status=%s sta_ip=%s ap_ip=%s", 
            m_tag,
            wifiStatusToString(staStatus),
            WiFi.localIP().toString().c_str(),
            WiFi.softAPIP().toString().c_str());

        const String mode = (WiFi.getMode() == WIFI_AP_STA) ? "AP+STA" :
                            (WiFi.getMode() == WIFI_STA) ? "STA" :
                            (WiFi.getMode() == WIFI_AP) ? "AP" :
                            "UNKNOWN";

        sf_mqtt::publish("smartfranklin/system/wifi/mode", mode.c_str());
        sf_mqtt::publish("smartfranklin/system/wifi/hostname", m_hostname.c_str());
        sf_mqtt::publish("smartfranklin/system/wifi/ap_ip", WiFi.softAPIP().toString().c_str());
        sf_mqtt::publish("smartfranklin/system/wifi/sta_ip", WiFi.localIP().toString().c_str());
        sf_mqtt::publish("smartfranklin/system/wifi/rssi", String(WiFi.RSSI()).c_str());

        m_lastStatusMs = millis();
    }

    return true;
}

// ---- FreeRTOS task ----

void taskWiFi(void *pv)
{
    (void)pv;
    SF_LOGI("[WIFI] Task started");

    uint32_t nextInitAttemptMs = 0;

    auto isRetryDue = [](const uint32_t nowMs, const uint32_t nextAttemptMs) {
        return static_cast<int32_t>(nowMs - nextAttemptMs) >= 0;
    };

    auto scheduleRetry = [](uint32_t& nextAttemptMs, const uint32_t nowMs) {
        nextAttemptMs = nowMs + 10000UL;  // 10 second retry interval
    };

    for (;;) {
        const uint32_t nowMs = millis();

        if (!WIFI_TASK.isInitialized() && isRetryDue(nowMs, nextInitAttemptMs)) {
            if (!WIFI_TASK.init()) {
                SF_LOGW("[WIFI] Init failed");
                scheduleRetry(nextInitAttemptMs, nowMs);
            }
        }

        if (WIFI_TASK.isInitialized()) {
            WIFI_TASK.process();
        }

        const int wifiLoopMs = (CONFIG.task_wifi_loop_ms > 0) ? CONFIG.task_wifi_loop_ms : 500;
        vTaskDelay(pdMS_TO_TICKS(wifiLoopMs));
    }
}
