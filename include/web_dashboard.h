/*
 * ============================================================================
 * Web Dashboard Module - SmartFranklin
 * ============================================================================
 * 
 * File:        web_dashboard.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Header file for the embedded web‑server dashboard.  Declares
 *              initialization routine and related types used to serve the
 *              configuration/status web pages that run on the device.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   The web dashboard module implements a lightweight HTTP/HTTPS server on the
 *   SmartFranklin device.  The dashboard exposes a simple web interface that
 *   allows local clients (desktop browser, mobile phone) to view system status,
 *   change configuration options and trigger actions without requiring an
 *   external cloud service.  It is intended to run over the same Wi‑Fi or
 *   cellular link used by the rest of the system or in AP/access‑point mode.
 * 
 * Features:
 *   - Serves HTML/JavaScript pages from SPIFFS/LittleFS
 *   - REST‑style JSON API for status, configuration and control
 *   - Optional basic authentication and TLS support
 *   - Auto‑refresh of sensor values and connection indicators
 *   - Configuration forms for network, MQTT, calibration parameters
 *   - Firmware‑update page (OTA) when enabled
 *   - Lightweight footprint suitable for ESP32
 * 
 * Integration:
 *   - Called from main setup() after networking is brought up
 *   - Reads and writes CONFIG structure for persistent settings
 *   - Publishes DATA snapshot to the web API on demand
 *   - Uses Mongoose or ESPAsyncWebServer under the hood
 *   - Works together with captive‑portal module to configure Wi‑Fi
 * 
 * Dependencies:
 *   - Arduino.h  : basic types
 *   - FS.h, SPIFFS/LittleFS : file system for static assets
 *   - WiFi.h / Ethernet.h  : network stack
 *   - web_dashboard.cpp : implementation of the functions declared here
 * 
 * Limitations:
 *   - Designed for local LAN access only; not hardened for public
 *     Internet exposure.
 *   - Single‑user interface, no multi‑session support.
 *   - Pages are static; extensive dynamic interaction is via the JSON API.
 *   - TLS requires certificate provisioning and increases RAM usage.
 * 
 * Best practices:
 *   - Protect access with a password when used on untrusted networks.
 *   - Use the API from automation scripts rather than screen scraping.
 *   - Keep static assets small to conserve flash and RAM.
 *   - Regenerate OTA pages after firmware updates if asset layout changes.
 * 
 * Usage:
 *   web_dashboard_init();   // call once in setup()
 *   // nothing else required; server runs in background
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
 * @brief Initialise the HTTP(S) web dashboard.
 *
 * Sets up the internal web server, mounts the filesystem containing the
 * frontend assets, registers URI handlers for status/configuration APIs
 * and, if requested, enables authentication/TLS.  After this call the
 * dashboard will respond to incoming browser requests without further
 * intervention; it runs inside its own task or the Arduino loop depending
 * on the underlying web‑server library.
 *
 * Call once during setup() after the network interface has been started.
 * The function is re‑entrant; calling it a second time has no effect.
 */
void web_dashboard_init();
```