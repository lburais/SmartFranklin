/*
 * ============================================================================
 * WiFi Dual-Mode Setup Header - SmartFranklin
 * ============================================================================
 * 
 * File:        wifi_setup.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: WiFi dual-mode (AP + STA) initialization helper for ESP32.
 *              Configures simultaneous Access Point and Station operation.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   This module provides WiFi configuration for the ESP32 to operate as:
 *   - Access Point (AP): Local hotspot for device configuration and fallback
 *   - Station (STA): Client connection to external WiFi networks
 *   
 *   Dual-mode operation ensures SmartFranklin remains accessible even when
 *   the external network connection fails or is unavailable.
 * 
 * Dependencies:
 *   - Arduino.h (ESP32 core library)
 *   - WiFi library (built-in ESP32)
 * 
 * ============================================================================
 * MIT License
 * ============================================================================
 * Copyright (c) 2026 Laurent Burais
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * ============================================================================
 */

#pragma once
#include <Arduino.h>

// ============================================================================
// Function Declarations
// ============================================================================
/**
 * @brief Initializes ESP32 in dual WiFi mode (Access Point + Station).
 * 
 * Configures the ESP32 to simultaneously operate as:
 *   1. WiFi Access Point (AP): Broadcasts SSID for direct device connection
 *   2. WiFi Station (STA): Connects to external WiFi network
 * 
 * This dual-mode approach provides:
 *   - Fallback connectivity: Device stays accessible via AP if STA fails
 *   - Flexible network configuration: Users can connect locally or remotely
 *   - Reduced downtime: No single-point-of-failure for network access
 * 
 * @param apSsid  - Access Point SSID (broadcast name) for local connections
 *                  Example: "SmartFranklin-AP"
 * 
 * @param apPass  - Access Point password (minimum 8 characters for WPA2)
 *                  Example: "smartfranklin"
 * 
 * @param staSsid - Station SSID (external network name) to connect to
 *                  Retrieved from persistent configuration
 *                  Example: "MyHomeWiFi"
 * 
 * @param staPass - Station password for external network authentication
 *                  Retrieved from persistent configuration
 *                  Example: "MyWiFiPassword123"
 * 
 * @return void
 * 
 * @note Call this function during system initialization (setup()).
 *       If STA connection fails, implement captive_portal_start() as fallback.
 * 
 * @example
 *   setupWiFiApSta("SmartFranklin-AP", 
 *                  "smartfranklin",
 *                  "HomeNetwork",
 *                  "homepassword");
 */
void setupWiFiApSta(const char *apSsid,   // Access Point SSID
                    const char *apPass,   // Access Point password
                    const char *staSsid,  // Station SSID
                    const char *staPass); // Station password

