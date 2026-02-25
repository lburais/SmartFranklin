#include "wifi_setup.h"
#include <WiFi.h>

void setupWiFiApSta(const char *apSsid,
                    const char *apPass,
                    const char *staSsid,
                    const char *staPass)
{
    WiFi.mode(WIFI_AP_STA);
    bool apOk = WiFi.softAP(apSsid, apPass);
    Serial.printf("[WiFi] AP %s: %s\n", apSsid, apOk ? "OK" : "FAIL");

    if (staSsid && strlen(staSsid) > 0) {
        WiFi.begin(staSsid, staPass);
        Serial.printf("[WiFi] Connecting to STA %s\n", staSsid);
        unsigned long start = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
            delay(500);
            Serial.print(".");
        }
        Serial.println();
        Serial.printf("[WiFi] STA status: %d, IP: %s\n",
                      WiFi.status(),
                      WiFi.localIP().toString().c_str());
    }
}
