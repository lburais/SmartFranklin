/*
 * ============================================================================
 * Captive Portal Implementation - SmartFranklin
 * ============================================================================
 * 
 * File:        captive_portal.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: Captive portal DNS server for WiFi configuration interface.
 *              Redirects all DNS queries to device IP for seamless user setup.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   A captive portal is a network access point that intercepts HTTP/DNS traffic
 *   to redirect users to a login or configuration page. SmartFranklin uses this
 *   technique to automatically present the WiFi configuration interface when a
 *   user connects to the device's Access Point (AP).
 * 
 * How It Works:
 *   1. Device broadcasts WiFi AP hotspot (e.g., "SmartFranklin-AP")
 *   2. User connects their device (phone/laptop) to the AP
 *   3. User's device sends DNS query (e.g., "google.com")
 *   4. Captive portal DNS server intercepts query
 *   5. DNS server responds with device IP instead of real DNS answer
 *   6. Browser redirects to device's web dashboard (smartfranklin.local or 192.168.4.1)
 *   7. User sees configuration interface without manual IP entry
 * 
 * Benefits:
 *   - Seamless user experience: No manual IP address entry required
 *   - Mobile-friendly: Works on iOS and Android automatic detection
 *   - Cross-platform: Compatible with Windows, macOS, Linux
 *   - Zero configuration: Works immediately upon AP connection
 * 
 * DNS Spoofing Method:
 *   - Listens on UDP port 53 (standard DNS port)
 *   - Wildcards "*" catch ALL DNS queries regardless of domain
 *   - Responds with AP IP address (typically 192.168.4.1)
 *   - Integrates with web dashboard for full configuration UI
 * 
 * Dependencies:
 *   - DNSServer.h (Arduino WiFi library)
 *   - WiFi.h (ESP32 WiFi support)
 *   - web_dashboard.h (configuration interface)
 *   - captive_portal.h (header declarations)
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

#include "captive_portal.h"
#include <DNSServer.h>
#include <M5Unified.h>
#include <WiFi.h>

// ============================================================================
// DNS Server Instance
// ============================================================================
// Static DNS server for intercepting and redirecting domain queries
// Scope limited to file (not exported) as it's only used internally
static DNSServer dnsServer;

// ============================================================================
// Captive Portal Initialization
// ============================================================================

/**
 * @brief Starts the captive portal DNS server.
 * 
 * Initializes a DNS server that intercepts ALL DNS queries and responds with
 * the device's Access Point IP address. This technique automatically redirects
 * user browsers to the device's configuration dashboard when they connect to
 * the SmartFranklin WiFi hotspot.
 * 
 * Behavior:
 *   - Listens on standard DNS port 53 (UDP)
 *   - Wildcard pattern "*" captures all domain queries
 *   - Responds with AP IP address (typically 192.168.4.1) to every query
 *   - User's browser receives this IP and navigates to the configuration page
 *   - No manual DNS or IP configuration required from user
 * 
 * Typical User Experience:
 *   a) User connects to "SmartFranklin-AP" WiFi network
 *   b) Device automatically opens "Sign in to Network" popup
 *   c) User taps "Sign in" button (or browser opens automatically)
 *   d) Browser loads configuration dashboard at 192.168.4.1
 *   e) User can configure WiFi credentials, MQTT settings, etc.
 * 
 * Technical Details:
 *   - DNS Protocol: RFC 1035 compliant responses
 *   - Response IP: Obtained from WiFi.softAPIP() (AP interface)
 *   - Query Filtering: Wildcard "*" accepts any domain name
 *   - Port: Standard DNS port 53 (UDP)
 *   - Non-blocking: Server runs in background, doesn't block execution
 * 
 * Prerequisites:
 *   - WiFi Access Point must be started before calling this function
 *   - WiFi.mode() should be set to WIFI_AP or WIFI_AP_STA
 *   - Valid AP IP address must be assigned (typically 192.168.4.1)
 * 
 * @return void
 * 
 * @note This function should be called during system initialization (setup())
 *       when external WiFi connection is unavailable or fails.
 *       Example: if (WiFi.status() != WL_CONNECTED) { captive_portal_start(); }
 * 
 * @see WiFi.softAPIP() - Returns AP interface IP address
 * @see web_dashboard.h - Provides web configuration interface
 */
void captive_portal_start()
{
    // Configure and start DNS server:
    // - Port 53: Standard DNS port (UDP protocol)
    // - Wildcard "*": Match all domain queries (e.g., google.com → device IP)
    // - WiFi.softAPIP(): Respond with AP IP address (typically 192.168.4.1)
    dnsServer.start(53, "*", WiFi.softAPIP());
    
        M5_LOGI("[Captive Portal] DNS server started on %s:53",
            WiFi.softAPIP().toString().c_str());
}
