#include "wifi_setup.h"

#include <WiFi.h>

void setupWiFiApSta(const char *apSsid,
                    const char *apPass,
                    const char *staSsid,
                    const char *staPass)
{
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(apSsid, apPass);

    if (staSsid && staSsid[0] != '\0') {
        WiFi.begin(staSsid, staPass);
    }
}
