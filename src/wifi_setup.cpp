/*
 * ============================================================================
 * WiFi Setup Module - SmartFranklin
 * ============================================================================
 *
 * File:        wifi_setup.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: Configures AP+STA dual-mode WiFi startup behavior used by the
 *              main bootstrap sequence.
 *
 * Author:      Laurent Burais
 * Date:        10 March 2026
 * Version:     1.1
 *
 * ============================================================================
 */

#include "wifi_setup.h"

#include <WiFi.h>

/**
 * @brief Initialize WiFi in AP+STA mode and optionally start STA association.
 *
 * AP mode is always enabled to keep a local recovery channel available.
 * STA association is started only when a non-empty station SSID is supplied.
 */
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
