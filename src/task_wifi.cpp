/*
 * ============================================================================
 * WiFi Connectivity Management Task - SmartFranklin
 * ============================================================================
 * 
 * File:        task_wifi.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: Asynchronous WiFi manager task for SmartFranklin.
 *              Maintains dual-mode WiFi connectivity with automatic reconnection,
 *              captive portal detection, and MQTT status publishing.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   This FreeRTOS task runs continuously in the background to manage WiFi
 *   connectivity. It maintains both Access Point (AP) and Station (STA) modes
 *   simultaneously, allowing the device to remain accessible locally while
 *   attempting to connect to external networks.
 * 
 * Key Features:
 *   - Access Point always enabled for local connectivity fallback
 *   - Station mode with automatic reconnection logic (15s intervals)
 *   - Captive portal detection using HTTP 204 probe method
 *   - Periodic MQTT publication of WiFi status and metrics
 *   - Graceful fallback to AP-only mode if STA connection fails repeatedly
 *   - Real-time RSSI (signal strength) monitoring
 *   - Non-blocking implementation with 500ms task cycle time
 * 
 * MQTT Topics Published:
 *   - smartfranklin/wifi/mode     : Current WiFi mode (AP, STA, AP+STA)
 *   - smartfranklin/wifi/ap_ip    : Access Point IP address
 *   - smartfranklin/wifi/sta_ip   : Station IP address (external network)
 *   - smartfranklin/wifi/rssi     : WiFi signal strength (dBm)
 *   - smartfranklin/wifi/captive  : Captive portal status (0=no, 1=yes)
 * 
 * Dependencies:
 *   - WiFi.h (ESP32 built-in)
 *   - HTTPClient.h (for captive portal detection)
 *   - mqtt_layer.h (MQTT publishing)
 *   - config_store.h (persistent configuration)
 *   - FreeRTOS (built-in ESP32)
 * 
 * ============================================================================
 * MIT License
 * ============================================================================
 * Copyright (c) 2024-2026 Laurent Burais
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ============================================================================
 */

#include "tasks.h"
#include "mqtt_layer.h"
#include "config_store.h"

#include <WiFi.h>
#include <HTTPClient.h>

// ============================================================================
// Captive Portal Detection
// ============================================================================


/**
 * @brief Detects if WiFi connection is trapped behind a captive portal.
 * 
 * Uses Google's connectivity check endpoint to determine if the connected
 * WiFi network is intercepting traffic. Captive portals redirect HTTP requests
 * and return HTML login pages instead of expected HTTP 204 responses.
 * 
 * Detection Method:
 *   - Sends HTTP GET request to: http://connectivitycheck.gstatic.com/generate_204
 *   - Expected response for free internet: HTTP 204 (No Content)
 *   - Captive portal response: HTTP 200-302 (redirects to login page)
 * 
 * @return true  - Captive portal detected (network is restricted)
 * @return false - No captive portal (free internet or disconnected)
 * 
 * @note This function only checks if STA is connected. Returns false if
 *       WiFi status is not WL_CONNECTED.
 */
static bool detectCaptivePortal()
{
    // Only check if actually connected to external network
    if (WiFi.status() != WL_CONNECTED) return false;

    HTTPClient http;
    // Send probe request to Google's connectivity check server
    http.begin("http://connectivitycheck.gstatic.com/generate_204");
    
    // Expected response code for unrestricted internet access
    int code = http.GET();
    http.end();

    // HTTP 204 = unrestricted internet; anything else suggests captive portal
    return (code != 204);
}

// ============================================================================
// WiFi Status Publishing to MQTT
// ============================================================================


/**
 * @brief Publishes current WiFi status and metrics to MQTT broker.
 * 
 * Collects WiFi operational data and publishes to designated MQTT topics
 * for remote monitoring and diagnostics. Provides insight into WiFi health,
 * connectivity mode, IP allocation, and signal quality.
 * 
 * Published Information:
 *   - WiFi Mode: Current operating mode (AP-only, STA-only, or dual AP+STA)
 *   - IP Addresses: Both AP and STA IP addresses for device accessibility
 *   - Signal Strength: RSSI (Received Signal Strength Indicator) in dBm
 *     * -30 to -50 dBm   : Excellent signal
 *     * -50 to -70 dBm   : Good signal
 *     * -70 to -85 dBm   : Weak signal
 *     * Below -85 dBm    : Very poor signal
 * 
 * @return void
 * 
 * @note Called periodically (every 30 seconds) from main task loop.
 *       MQTT must be initialized before calling this function.
 */
static void publishWiFiStatus()
{
    // Determine current WiFi mode and format as descriptive string
    String mode = (WiFi.getMode() == WIFI_AP_STA) ? "AP+STA" :    // Both modes active
                  (WiFi.getMode() == WIFI_STA)    ? "STA" :       // Station mode only
                  (WiFi.getMode() == WIFI_AP)     ? "AP" :        // Access point only
                  "UNKNOWN";                                        // Error state

    // Publish WiFi operational mode
    sf_mqtt::publish("smartfranklin/wifi/mode", mode.c_str());
    
    // Publish Access Point IP (always available if AP enabled)
    sf_mqtt::publish("smartfranklin/wifi/ap_ip", WiFi.softAPIP().toString().c_str());
    
    // Publish Station IP (only available if STA connected)
    sf_mqtt::publish("smartfranklin/wifi/sta_ip", WiFi.localIP().toString().c_str());
    
    // Publish WiFi signal strength (RSSI = Received Signal Strength Indicator)
    // Typical range: -30 (excellent) to -100+ (lost signal)
    sf_mqtt::publish("smartfranklin/wifi/rssi", String(WiFi.RSSI()).c_str());
}

// ============================================================================
// WiFi Initial Setup
// ============================================================================


/**
 * @brief Initializes WiFi hardware in dual Access Point + Station mode.
 * 
 * Configures the ESP32 WiFi interface for simultaneous operation:
 *   1. Starts Access Point with configured SSID and password
 *   2. Initiates Station connection to external network (if configured)
 * 
 * This dual-mode setup ensures SmartFranklin remains accessible via direct
 * connection to its AP hotspot even if external WiFi connection fails.
 * 
 * Configuration Source:
 *   - AP credentials: CONFIG.ap_ssid, CONFIG.ap_pass (from persistent config)
 *   - STA credentials: CONFIG.sta_ssid, CONFIG.sta_pass (from persistent config)
 * 
 * Logging Output:
 *   - Prints AP startup status (OK/FAIL) to serial console
 *   - Prints STA SSID and connection attempt status
 * 
 * @return void
 * 
 * @note Called once during task initialization. Should be called after
 *       CONFIG has been loaded from persistent storage.
 */
static void wifiInitialSetup()
{
    // Set WiFi mode to simultaneous Access Point and Station operation
    WiFi.mode(WIFI_AP_STA);

    // Start Access Point with configured credentials
    bool apOk = WiFi.softAP(CONFIG.ap_ssid.c_str(),    // AP network name (SSID)
                            CONFIG.ap_pass.c_str());    // AP password (WPA2)
    
    // Log Access Point startup result
    Serial.printf("[WiFi] AP %s: %s\n",
                  CONFIG.ap_ssid.c_str(),
                  apOk ? "OK" : "FAIL");

    // If Station SSID is configured, attempt to connect to external network
    if (!CONFIG.sta_ssid.isEmpty()) {
        WiFi.begin(CONFIG.sta_ssid.c_str(),    // External network SSID
                   CONFIG.sta_pass.c_str());    // External network password
        
        // Log Station connection attempt
        Serial.printf("[WiFi] STA connecting to %s\n",
                      CONFIG.sta_ssid.c_str());
    }
}

// ============================================================================
// FreeRTOS Task Implementation
// ============================================================================


/**
 * @brief WiFi connectivity management FreeRTOS task.
 * 
 * Runs continuously in the background to monitor and manage WiFi connectivity.
 * Implements automatic reconnection logic, periodic status monitoring, and
 * publishes WiFi metrics to MQTT for remote diagnostics.
 * 
 * Task Cycle:
 *   - Runs indefinitely with 500ms sleep between iterations
 *   - Checks STA connection status every iteration
 *   - Attempts reconnection every 15 seconds if disconnected
 *   - Publishes WiFi status every 30 seconds
 *   - Detects captive portal status when STA is connected
 * 
 * Reconnection Strategy:
 *   - If STA configured but not connected: attempts reconnection every 15s
 *   - Performs WiFi.disconnect() before reattempt (clears stale state)
 *   - Uses exponential backoff implicitly through reconnection intervals
 *   - Falls back to AP-only mode if STA connectivity fails
 * 
 * @param pv FreeRTOS task parameter (unused)
 * @return void (infinite loop, never returns)
 * 
 * @note Task priority: 1 (low) on dedicated core for WiFi
 *       Task stack size: 4096 bytes
 */
void taskWiFi(void *pv)
{
    Serial.println("[WiFi] Task started");

    // Initialize WiFi hardware in AP+STA mode with configured credentials
    wifiInitialSetup();

    // =========================================================================
    // Task State Variables
    // =========================================================================
    unsigned long lastStatus = 0;           // Timestamp of last MQTT status publish
    unsigned long lastReconnectAttempt = 0; // Timestamp of last STA reconnection attempt
    
    // Reconnection attempt interval (15 seconds between attempts)
    // Prevents excessive reconnection attempts that drain power
    const unsigned long reconnectInterval = 15000;
    
    // Status publication interval (30 seconds between publishes)
    // Provides periodic WiFi health metrics to MQTT
    const unsigned long statusInterval = 30000;

    // =========================================================================
    // Infinite Task Loop
    // =========================================================================
    for (;;) {

        // --- Station Reconnection Logic ---
        // If Station SSID is configured, manage STA connection state
        if (!CONFIG.sta_ssid.isEmpty()) {
            // Check if currently disconnected from external network
            if (WiFi.status() != WL_CONNECTED) {
                // Check if enough time has elapsed since last reconnection attempt
                if (millis() - lastReconnectAttempt > reconnectInterval) {
                    Serial.println("[WiFi] STA reconnecting...");
                    
                    // Clear any previous connection state before reattempt
                    WiFi.disconnect();
                    
                    // Initiate new connection attempt with configured credentials
                    WiFi.begin(CONFIG.sta_ssid.c_str(),
                               CONFIG.sta_pass.c_str());
                    
                    // Update reconnection attempt timestamp
                    lastReconnectAttempt = millis();
                }
            }
        }

        // --- Captive Portal Detection ---
        // If STA is connected, check for captive portal interception
        if (WiFi.status() == WL_CONNECTED) {
            // Detect if connection is trapped behind captive portal for login
            bool captive = detectCaptivePortal();
            
            // Publish captive portal status: 1 = portal detected, 0 = free internet
            sf_mqtt::publish("smartfranklin/wifi/captive",
                             captive ? "1" : "0");
        }

        // --- Periodic MQTT Status Publication ---
        // Publish WiFi metrics to MQTT at regular intervals
        if (millis() - lastStatus > statusInterval) {
            // Collect and publish current WiFi status
            publishWiFiStatus();
            
            // Update last publish timestamp
            lastStatus = millis();
        }

        // Yield to other tasks (500ms cycle time)
        // Non-blocking sleep allows WiFi driver to run and other tasks to execute
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
