/*
 * ============================================================================
 * Captive Portal Implementation - SmartFranklin
 * ============================================================================
 * 
 * File:        captive_portal.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: Captive portal DNS helper used by the WiFi module.
 *              Starts a wildcard DNS server that resolves every query to the
 *              AP IP so browsers land on the embedded web UI.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   The current implementation only starts the DNS side of the captive portal.
 *   HTTP handling is provided separately by the web dashboard routes.
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
 * The server is non-blocking and remains owned by this translation unit.
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
#include "log.h"

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
 * @note Call after the AP interface is started and has a valid IP address.
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
    
    SF_LOGI("[Captive Portal] DNS server started on %s:53", WiFi.softAPIP().toString().c_str());
}
