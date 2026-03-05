/*
 * ============================================================================
 * Captive Portal Module - SmartFranklin
 * ============================================================================
 * 
 * File:        captive_portal.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Header file for WiFi captive portal functionality. Provides
 *              access point setup and web-based configuration interface for
 *              initial device setup and network configuration.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   The captive portal module creates a WiFi access point and web server
 *   for initial device configuration. When SmartFranklin cannot connect
 *   to configured WiFi networks, it automatically starts the captive portal
 *   to allow users to configure WiFi credentials, MQTT settings, and other
 *   system parameters through a web interface.
 * 
 * WiFi Access Point:
 *   - SSID: "SmartFranklin-Setup" (configurable)
 *   - Security: Open network (no password required)
 *   - IP Address: 192.168.4.1 (ESP32 default AP IP)
 *   - Channel: Auto-selected for best performance
 *   - Maximum Connections: 4 simultaneous clients
 * 
 * Web Server Features:
 *   - HTTP Server: Port 80 for web interface
 *   - Captive Portal: Automatic redirect to configuration page
 *   - DNS Server: Redirects all DNS queries to configuration page
 *   - File Serving: HTML, CSS, JavaScript for web interface
 *   - Form Handling: POST requests for configuration updates
 * 
 * Configuration Options:
 *   - WiFi Networks: SSID and password configuration
 *   - MQTT Broker: Host, port, credentials setup
 *   - Device Settings: Device name, timezone, other parameters
 *   - Security: Password protection for configuration access
 *   - Validation: Input validation and error checking
 * 
 * Portal Activation:
 *   - Automatic: Starts when WiFi connection fails
 *   - Manual: Can be triggered by button press or command
 *   - Timeout: Automatically stops after successful configuration
 *   - Status: LED indicators show portal active state
 * 
 * User Experience:
 *   - Auto-Connect: Devices automatically connect to portal SSID
 *   - Browser Redirect: Automatic navigation to configuration page
 *   - Mobile Friendly: Responsive design for phone/tablet access
 *   - Real-time Feedback: AJAX updates for configuration status
 *   - Success Confirmation: Clear indication of successful setup
 * 
 * Security Considerations:
 *   - Open Network: No encryption (acceptable for setup only)
 *   - Short Duration: Portal active only during configuration
 *   - Input Validation: Prevents malicious configuration attempts
 *   - Session Management: No persistent sessions required
 *   - Firewall: Limited access to device functionality
 * 
 * Integration:
 *   - WiFi Manager: Coordinates with WiFi connection attempts
 *   - Configuration Store: Saves settings to persistent storage
 *   - System Restart: Applies new configuration after setup
 *   - Status Display: Shows portal status on device screen
 * 
 * Dependencies:
 *   - WiFi.h: ESP32 WiFi access point functionality
 *   - WebServer.h: HTTP server for web interface
 *   - DNSServer.h: DNS server for captive portal
 *   - config_store.h: Configuration storage access
 *   - M5Unified.h: Display and UI feedback
 * 
 * Limitations:
 *   - Single AP: Cannot create multiple access points
 *   - No HTTPS: HTTP-only for simplicity
 *   - Memory Usage: Web interface consumes RAM
 *   - Concurrent Users: Limited to 4 simultaneous connections
 *   - Browser Dependent: Requires JavaScript-enabled browser
 * 
 * Best Practices:
 *   - Keep portal active for limited time only
 *   - Provide clear instructions for users
 *   - Validate all configuration inputs
 *   - Test on multiple devices and browsers
 *   - Monitor memory usage during portal operation
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

/**
 * @brief Starts the WiFi captive portal for device configuration.
 * 
 * Initializes and starts the captive portal access point and web server.
 * Creates an open WiFi network that allows users to connect and configure
 * the device through a web interface. Automatically handles DNS redirection
 * and web page serving for a seamless configuration experience.
 * 
 * Portal Setup Process:
 *   1. Stop any existing WiFi connections
 *   2. Create access point with "SmartFranklin-Setup" SSID
 *   3. Start DNS server for captive portal functionality
 *   4. Start HTTP web server on port 80
 *   5. Load and serve configuration web pages
 *   6. Handle client connections and configuration requests
 * 
 * Access Point Configuration:
 *   - SSID: SmartFranklin-Setup (hardcoded for consistency)
 *   - Password: None (open network for easy access)
 *   - IP Address: 192.168.4.1 (ESP32 AP default)
 *   - Subnet: 192.168.4.0/24
 *   - Channel: Auto-selected
 * 
 * Web Server Features:
 *   - Root page (/) serves main configuration interface
 *   - POST /config handles configuration form submissions
 *   - Static file serving for CSS, JavaScript, images
 *   - JSON API endpoints for AJAX requests
 *   - Error page handling for invalid requests
 * 
 * DNS Server:
 *   - Listens on port 53
 *   - Redirects all domain queries to 192.168.4.1
 *   - Enables captive portal behavior on mobile devices
 *   - Handles wildcard DNS queries
 * 
 * Configuration Handling:
 *   - WiFi credentials (SSID/password)
 *   - MQTT broker settings (host/port/user/pass)
 *   - Device parameters (name/timezone/etc.)
 *   - Validation and error checking
 *   - Persistent storage of new configuration
 * 
 * User Interaction:
 *   - Connect device to "SmartFranklin-Setup" WiFi
 *   - Open browser (automatically redirected)
 *   - Fill configuration form
 *   - Submit and wait for confirmation
 *   - Device restarts with new configuration
 * 
 * Timeout and Exit:
 *   - Portal remains active until configuration complete
 *   - Automatic timeout after 30 minutes of inactivity
 *   - Manual exit through configuration success
 *   - System restart applies new settings
 * 
 * Error Handling:
 *   - AP creation failures logged and retried
 *   - Web server errors handled gracefully
 *   - Invalid configuration inputs rejected
 *   - Memory allocation failures managed
 * 
 * Security Notes:
 *   - Open WiFi network (acceptable for setup phase)
 *   - No encryption on web interface
 *   - Input validation prevents malicious inputs
 *   - Short operational time minimizes exposure
 * 
 * @note This function blocks until configuration is complete or timeout.
 *       Device will restart automatically after successful configuration.
 *       Portal should only be active during initial setup or reconfiguration.
 * 
 * @see WiFiManager - WiFi connection management
 * @see config_store - Configuration persistence
 */
void captive_portal_start();