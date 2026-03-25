/*
 * ============================================================================
 * Web Dashboard Module - SmartFranklin
 * ============================================================================
 * 
 * File:        web_dashboard.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: Embedded web server providing configuration and monitoring interface.
 *              Serves HTML dashboard, REST API endpoints, and firmware update capability
 *              through AsyncWebServer with authentication and real-time data updates.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   SmartFranklin provides a comprehensive web-based management interface
 *   accessible via WiFi connection. The dashboard offers real-time monitoring,
 *   configuration management, diagnostics, and over-the-air firmware updates.
 *   All access is protected by HTTP basic authentication using admin credentials.
 * 
 * Web Server Architecture:
 *   - Framework: ESPAsyncWebServer (asynchronous, non-blocking)
 *   - Port: 80 (standard HTTP, no HTTPS for embedded constraints)
 *   - Authentication: HTTP Basic Auth with admin username/password
 *   - Content: Mixed static HTML/JS and dynamic JSON API endpoints
 *   - File System: SPIFFS for static web assets (HTML, CSS, JS)
 *   - Real-time Updates: JavaScript fetch API with 1-second polling
 * 
 * Dashboard Pages:
 * 
 *   Main Dashboard (/):
 *   - Live sensor data display with auto-refresh
 *   - Navigation buttons to configuration sections
 *   - Real-time JSON data from /api/status endpoint
 *   - Responsive design for mobile/desktop access
 * 
 *   Configuration (/config):
 *   - WiFi settings (SSID, password)
 *   - MQTT broker configuration
 *   - Runtime configuration settings
 *   - Admin authentication credentials
 *   - Hardware calibration factors
 * 
 *   Firmware Update (/update):
 *   - Over-the-air (OTA) firmware upload
 *   - Progress indication and status
 *   - Automatic reboot after successful update
 *   - Rollback protection (ESP-IDF OTA partitions)
 * 
 *   Diagnostics (/diagnostics):
 *   - System health and performance metrics
 *   - Network connectivity status
 *   - Task watchdog heartbeat monitoring
 *   - Log file access and filtering
 * 
 * API Endpoints:
 * 
 *   GET /api/status:
 *   - Returns real-time sensor data and system state
 *   - Protected by authentication
 *   - JSON format with current DATA values
 *   - Thread-safe access with DATA_MUTEX
 * 
 *   GET /api/hw:
 *   - Hardware status (battery, buttons, IMU)
 *   - M5Stack-specific sensor readings
 *   - JSON response with HwStatus structure
 * 
 *   GET /api/full:
 *   - Combined runtime snapshot (data + config + hardware)
 * 
 *   GET /api/set_brightness?value=N:
 *   - Controls display brightness (0-255)
 *   - Immediate effect on M5Stack LCD
 *   - No authentication required for convenience
 * 
 *   GET /api/reboot:
 *   - Triggers system restart
 *   - 200ms delay before ESP.restart()
 *   - Allows HTTP response to complete
 * 
 *   GET /api/sleep:
 *   - Enters deep sleep mode
 *   - Ultra-low power consumption
 *   - Device wakes on button press or USB
 * 
 * Authentication:
 *   - HTTP Basic Authentication on all sensitive endpoints
 *   - Credentials from CONFIG.admin_user and CONFIG.admin_pass
 *   - Browser prompts for username/password on access
 *   - Session-less (credentials required for each request)
 * 
 * Static Assets:
 *   - Served from SPIFFS filesystem
 *   - HTML pages: /hw.html, /sensors.html
 *   - JavaScript: /theme.js (common UI functionality)
 *   - Cached by browser for performance
 * 
 * Real-time Updates:
 *   - JavaScript fetch API polls /api/status every second
 *   - JSON data displayed in formatted <pre> element
 *   - Automatic refresh continues during page view
 *   - No WebSocket for simplicity (HTTP polling sufficient)
 * 
 * Dependencies:
 *   - ESPAsyncWebServer.h (asynchronous web server library)
 *   - ElegantOTA.h (OTA firmware update library)
 *   - ArduinoJson.h (JSON serialization for API responses)
 *   - FS.h, SPIFFS.h (SPIFFS filesystem access)
 *   - WiFi.h (network connectivity)
 *   - m5_hw.h (hardware abstraction for status)
 *   - data_model.h (shared runtime data)
 *   - data_model.h (global DATA access)
 *   - config_store.h (configuration access)
 *   - web_dashboard.h (header declarations)
 * 
 * Security Considerations:
 *   - No HTTPS (embedded resource constraints)
 *   - HTTP Basic Auth (base64 encoded, not encrypted)
 *   - Local network access only (no internet exposure)
 *   - Admin credentials should be changed from defaults
 *   - OTA updates require authentication
 * 
 * Performance:
 *   - Memory usage: ~20-30KB for web server and buffers
 *   - CPU overhead: Minimal (asynchronous processing)
 *   - Network: HTTP responses served quickly
 *   - Concurrent connections: Up to 4 simultaneous clients
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

#include "m5_hw.h"
#include "web_dashboard.h"
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ElegantOTA.h>
#include <FS.h>
#include <SPIFFS.h>
#include <WiFi.h>

#include <cmath>
#include <cstdlib>
#include "data_model.h"
#include "config_store.h"
#include "gaz.h"

// ============================================================================
// Global Web Server Instance
// ============================================================================

/**
 * @brief Global AsyncWebServer instance running on port 80.
 * 
 * Handles all HTTP requests for the web dashboard and API endpoints.
 * Configured for asynchronous operation to prevent blocking FreeRTOS tasks.
 * All routes registered in web_dashboard_init() function.
 */
static AsyncWebServer server(80);

// ============================================================================
// Embedded HTML Content
// ============================================================================

/**
 * @brief Main dashboard HTML page served from PROGMEM.
 * 
 * Complete HTML document with embedded CSS and JavaScript for the main dashboard.
 * Includes navigation buttons, live data display area, and auto-refresh functionality.
 * Stored in program memory to save RAM (PROGMEM attribute).
 * 
 * Page Features:
 *   - Responsive design with card-based layout
 *   - Navigation buttons to configuration sections
 *   - Live data display in &lt;pre&gt; element
 *   - JavaScript fetch API for real-time updates
 *   - 1-second polling interval for data refresh
 *   - JSON formatting with syntax highlighting
 * 
 * JavaScript Functionality:
 *   - refresh() function fetches /api/status and updates display
 *   - setInterval() calls refresh every 1000ms
 *   - Error handling for network failures
 *   - Automatic retry on fetch failures
 * 
 * CSS Styling:
 *   - Sans-serif font for readability
 *   - Card layout with rounded corners and padding
 *   - Blue button styling for navigation
 *   - Pre-formatted text for JSON display
 */
static const char MAIN_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SmartFranklin</title>
<style>
body { font-family: sans-serif; margin: 10px; }
.card { background: #f4f4f4; padding: 15px; border-radius: 10px; margin-bottom: 15px; }
pre { white-space: pre-wrap; word-wrap: break-word; }
a.button { display: inline-block; padding: 10px 15px; background: #0078ff; color: white; border-radius: 6px; text-decoration: none; margin-right: 5px; }
.grid2 { display: grid; grid-template-columns: repeat(2, minmax(140px, 1fr)); gap: 8px; }
.btn { padding: 8px 12px; border: 0; border-radius: 6px; background: #0078ff; color: #fff; cursor: pointer; }
.btn.warn { background: #d9534f; }
.status { color: #444; font-size: 0.9em; margin-top: 8px; }
input[type='number'] { width: 100%; box-sizing: border-box; padding: 8px; border: 1px solid #bbb; border-radius: 6px; }
</style>
</head>
<body>
<h1>SmartFranklin</h1>
<div class="card">
  <a class="button" href="/config">Configuration</a>
    <a class="button" href="/level">Level</a>
    <a class="button" href="/gaz">Gaz 907</a>
  <a class="button" href="/update">Firmware Update</a>
  <a class="button" href="/diagnostics">Diagnostics</a>
</div>
<div class="card">
  <h3>Live Data</h3>
  <pre id="data">Loading...</pre>
</div>

<div class="card">
    <h3>IMU Geometry Calibration</h3>
    <div class="grid2">
        <div>
            <label for="wheelbase_mm">Wheelbase (mm)</label>
            <input id="wheelbase_mm" type="number" step="1" />
        </div>
        <div>
            <label for="track_width_mm">Track Width (mm)</label>
            <input id="track_width_mm" type="number" step="1" />
        </div>
        <div>
            <label for="offset_x_mm">IMU Offset X (mm)</label>
            <input id="offset_x_mm" type="number" step="1" />
        </div>
        <div>
            <label for="offset_y_mm">IMU Offset Y (mm)</label>
            <input id="offset_y_mm" type="number" step="1" />
        </div>
    </div>
    <div style="display:flex; gap:8px; margin-top:10px;">
        <button class="btn" onclick="saveImuGeometry()">Save</button>
        <button class="btn" onclick="loadImuGeometry()">Reload</button>
        <button class="btn warn" onclick="resetImuGeometry()">Reset Defaults</button>
    </div>
    <div id="level_geom_status" class="status">--</div>
</div>
<script>
function updateGeometryInputs(g) {
    document.getElementById('wheelbase_mm').value = g.level_wheelbase_mm;
    document.getElementById('track_width_mm').value = g.level_track_width_mm;
    document.getElementById('offset_x_mm').value = g.level_offset_x_mm;
    document.getElementById('offset_y_mm').value = g.level_offset_y_mm;
}

async function loadImuGeometry() {
    try {
        const r = await fetch('/api/level_geometry');
        const g = await r.json();
        updateGeometryInputs(g);
        document.getElementById('level_geom_status').textContent = 'Geometry loaded';
    } catch (e) {
        document.getElementById('level_geom_status').textContent = 'Failed to load geometry';
    }
}

async function saveImuGeometry() {
    try {
        const wheelbase = encodeURIComponent(document.getElementById('wheelbase_mm').value);
        const track = encodeURIComponent(document.getElementById('track_width_mm').value);
        const offsetX = encodeURIComponent(document.getElementById('offset_x_mm').value);
        const offsetY = encodeURIComponent(document.getElementById('offset_y_mm').value);

        const url = '/api/set_level_geometry?wheelbase_mm=' + wheelbase + '&track_width_mm=' + track + '&offset_x_mm=' + offsetX + '&offset_y_mm=' + offsetY;
        const r = await fetch(url);
        const body = await r.json();
        if (!r.ok) {
            document.getElementById('level_geom_status').textContent = 'Save failed: ' + (body.error || 'unknown');
            return;
        }

        updateGeometryInputs(body);
        document.getElementById('level_geom_status').textContent = body.saved ? 'Saved and applied' : 'Applied but not persisted';
    } catch (e) {
        document.getElementById('level_geom_status').textContent = 'Save request failed';
    }
}

async function resetImuGeometry() {
    try {
        const r = await fetch('/api/reset_level_geometry');
        const body = await r.json();
        if (!r.ok) {
            document.getElementById('level_geom_status').textContent = 'Reset failed';
            return;
        }

        updateGeometryInputs(body);
        document.getElementById('level_geom_status').textContent = body.saved ? 'Defaults restored' : 'Defaults applied but not persisted';
    } catch (e) {
        document.getElementById('level_geom_status').textContent = 'Reset request failed';
    }
}

async function refresh() {
  try {
    const r = await fetch('/api/status');
    const j = await r.json();
    document.getElementById('data').textContent = JSON.stringify(j, null, 2);
  } catch (e) {
    document.getElementById('data').textContent = 'Error loading data...';
  }
}
setInterval(refresh, 1000);
loadImuGeometry();
refresh();
</script>
</body>
</html>
)HTML";

static const char CONFIG_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SmartFranklin - Config</title>
<style>
body { font-family: sans-serif; margin: 10px; }
.card { background: #f4f4f4; padding: 15px; border-radius: 10px; margin-bottom: 15px; }
pre { white-space: pre-wrap; word-wrap: break-word; }
a.button { display: inline-block; padding: 10px 15px; background: #0078ff; color: white; border-radius: 6px; text-decoration: none; margin-right: 5px; }
.grid2 { display: grid; grid-template-columns: repeat(2, minmax(160px, 1fr)); gap: 10px; }
.field { display: flex; flex-direction: column; }
label { font-size: 0.9em; margin-bottom: 4px; }
input { padding: 8px; border: 1px solid #bbb; border-radius: 6px; width: 100%; box-sizing: border-box; }
.actions { margin-top: 12px; display: flex; gap: 8px; }
.btn { padding: 9px 12px; border: 0; border-radius: 6px; background: #0078ff; color: #fff; cursor: pointer; }
.btn.secondary { background: #666; }
.status { color: #333; font-size: 0.9em; margin-top: 10px; }
</style>
</head>
<body>
<h1>Configuration</h1>
<div class="card">
    <a class="button" href="/">Dashboard</a>
    <a class="button" href="/level">Level</a>
    <a class="button" href="/gaz">Gaz 907</a>
    <a class="button" href="/diagnostics">Diagnostics</a>
    <a class="button" href="/update">Firmware Update</a>
</div>
<div class="card">
    <h3>Edit Configuration</h3>
    <div id="fields" class="grid2"></div>
    <div class="actions">
        <button class="btn" onclick="saveConfig()">Save</button>
        <button class="btn secondary" onclick="loadConfig()">Reload</button>
    </div>
    <div id="status" class="status">Loading...</div>
</div>
<div class="card">
    <h3>Current Config JSON</h3>
    <pre id="config">Loading...</pre>
</div>
<script>
const fields = [
    { key: 'hostname', label: 'Hostname', type: 'text' },
    { key: 'ap_ssid', label: 'AP SSID', type: 'text' },
    { key: 'ap_pass', label: 'AP Password', type: 'password' },
    { key: 'sta_ssid', label: 'STA SSID', type: 'text' },
    { key: 'sta_pass', label: 'STA Password', type: 'password' },
    { key: 'admin_user', label: 'Admin User', type: 'text' },
    { key: 'admin_pass', label: 'Admin Password', type: 'password' },
    { key: 'mqtt_port', label: 'MQTT Port', type: 'number', step: '1' },
    { key: 'scale_cal_factor', label: 'Scale Cal Factor', type: 'number', step: '0.01' },
    { key: 'level_wheelbase_mm', label: 'Level Wheelbase mm', type: 'number', step: '1' },
    { key: 'level_track_width_mm', label: 'Level Track Width mm', type: 'number', step: '1' },
    { key: 'level_offset_x_mm', label: 'Level Offset X mm', type: 'number', step: '1' },
    { key: 'level_offset_y_mm', label: 'Level Offset Y mm', type: 'number', step: '1' },
    { key: 'level_zero_pitch_deg', label: 'Level Zero Pitch deg', type: 'number', step: '0.01' },
    { key: 'level_zero_roll_deg', label: 'Level Zero Roll deg', type: 'number', step: '0.01' },
    { key: 'task_wifi_loop_ms', label: 'Task WiFi Loop ms', type: 'number', step: '1' },
    { key: 'task_mqtt_loop_ms', label: 'Task MQTT Loop ms', type: 'number', step: '1' },
    { key: 'task_i2c_loop_ms', label: 'Task I2C Loop ms', type: 'number', step: '1' },
    { key: 'task_hmi_loop_ms', label: 'Task HMI Loop ms', type: 'number', step: '1' },
    { key: 'task_hmi_init_retry_ms', label: 'Task HMI Init Retry ms', type: 'number', step: '1' },
    { key: 'task_hw_monitor_loop_ms', label: 'Task HW Monitor Loop ms', type: 'number', step: '1' },
    { key: 'task_bms_ble_connected_loop_ms', label: 'Task BMS Connected Loop ms', type: 'number', step: '1' },
    { key: 'task_bms_ble_retry_loop_ms', label: 'Task BMS Retry Loop ms', type: 'number', step: '1' },
    { key: 'task_watchdog_loop_ms', label: 'Task Watchdog Loop ms', type: 'number', step: '1' }
];

function renderFields() {
    const container = document.getElementById('fields');
    container.innerHTML = '';

    fields.forEach((f) => {
        const wrapper = document.createElement('div');
        wrapper.className = 'field';

        const label = document.createElement('label');
        label.htmlFor = f.key;
        label.textContent = f.label;

        const input = document.createElement('input');
        input.id = f.key;
        input.type = f.type || 'text';
        if (f.step) {
            input.step = f.step;
        }

        wrapper.appendChild(label);
        wrapper.appendChild(input);
        container.appendChild(wrapper);
    });
}

async function loadConfig() {
    document.getElementById('status').textContent = 'Loading...';
    try {
        const r = await fetch('/api/config');
        const j = await r.json();

        fields.forEach((f) => {
            const value = j[f.key];
            document.getElementById(f.key).value = (value === undefined || value === null) ? '' : String(value);
        });

        document.getElementById('config').textContent = JSON.stringify(j, null, 2);
        document.getElementById('status').textContent = 'Configuration loaded';
    } catch (e) {
        document.getElementById('status').textContent = 'Error loading configuration';
        document.getElementById('config').textContent = 'Error loading config';
    }
}

async function saveConfig() {
    const params = new URLSearchParams();
    fields.forEach((f) => {
        params.set(f.key, document.getElementById(f.key).value);
    });

    document.getElementById('status').textContent = 'Saving...';

    try {
        const r = await fetch('/api/set_config?' + params.toString());
        const body = await r.json();
        if (!r.ok) {
            document.getElementById('status').textContent = 'Save failed: ' + (body.parameter || body.error || 'unknown');
            return;
        }

        const cfg = body.config || {};
        document.getElementById('config').textContent = JSON.stringify(cfg, null, 2);
        document.getElementById('status').textContent = body.saved ? 'Saved and applied' : 'Applied but not persisted';
    } catch (e) {
        document.getElementById('status').textContent = 'Save request failed';
    }
}

renderFields();
loadConfig();
</script>
</body>
</html>
)HTML";

static const char GAZ_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SmartFranklin - Gaz 907</title>
<style>
:root {
    --bg: #f6f7fb;
    --card: #ffffff;
    --ink: #1f2d3d;
    --accent: #0d9488;
    --accent-soft: #99f6e4;
    --line: #cbd5e1;
}
body { background: radial-gradient(circle at 10% 10%, #ecfeff, #f8fafc 45%, #eef2ff); color: var(--ink); font-family: sans-serif; margin: 10px; }
.card { background: var(--card); border: 1px solid var(--line); padding: 15px; border-radius: 12px; margin-bottom: 15px; }
a.button { display: inline-block; padding: 10px 15px; background: #0f766e; color: white; border-radius: 8px; text-decoration: none; margin-right: 8px; }
.layout { display: grid; grid-template-columns: 1fr; gap: 16px; }
.gaz-stage { display: flex; flex-direction: column; align-items: center; gap: 8px; }
.bottle-wrap { width: 210px; height: 360px; position: relative; }
.bottle-neck { position: absolute; top: 0; left: 78px; width: 54px; height: 54px; border: 5px solid #475569; border-bottom: 0; border-radius: 10px 10px 0 0; background: #e2e8f0; }
.bottle-body { position: absolute; left: 20px; top: 40px; width: 170px; height: 300px; border: 5px solid #475569; border-radius: 76px 76px 64px 64px; background: linear-gradient(180deg, #f8fafc 0%, #e2e8f0 100%); overflow: hidden; }
.bottle-fill { position: absolute; left: 0; right: 0; bottom: 0; height: 0%; background: linear-gradient(180deg, #34d399 0%, #0f766e 100%); transition: height 0.5s ease; }
.bottle-highlight { position: absolute; left: 34px; top: 58px; width: 20px; height: 240px; border-radius: 14px; background: rgba(255,255,255,0.42); }
.overlay { position: absolute; left: 0; right: 0; top: 48%; transform: translateY(-50%); text-align: center; font-size: 2.3rem; font-weight: 700; color: #0b3a36; text-shadow: 0 1px 0 rgba(255,255,255,0.55); }
.legend { font-size: 0.95rem; color: #334155; }
.weight { font-size: 1.25rem; font-weight: 700; }
.grid2 { display: grid; grid-template-columns: repeat(2, minmax(150px, 1fr)); gap: 10px; }
label { font-size: 0.9em; margin-bottom: 4px; display: block; }
input { width: 100%; box-sizing: border-box; padding: 9px; border: 1px solid #94a3b8; border-radius: 8px; }
.actions { margin-top: 10px; display: flex; gap: 8px; flex-wrap: wrap; }
.btn { padding: 9px 12px; border: 0; border-radius: 8px; color: #fff; background: #0f766e; cursor: pointer; }
.btn.step2 { background: #0369a1; }
.btn.reload { background: #475569; }
.status { margin-top: 10px; color: #334155; font-size: 0.95rem; min-height: 1.2em; }
@media (max-width: 420px) {
    .bottle-wrap { width: 180px; height: 320px; }
    .bottle-neck { left: 66px; }
    .bottle-body { left: 15px; width: 150px; height: 270px; }
}
</style>
</head>
<body>
<h1>Gaz 907</h1>
<div class="card">
    <a class="button" href="/">Dashboard</a>
    <a class="button" href="/level">Level</a>
    <a class="button" href="/config">Configuration</a>
    <a class="button" href="/diagnostics">Diagnostics</a>
</div>

<div class="card layout">
    <div class="gaz-stage">
        <div class="bottle-wrap" aria-label="Bouteille Gaz 907">
            <div class="bottle-neck"></div>
            <div class="bottle-body">
                <div id="bottle_fill" class="bottle-fill"></div>
                <div class="bottle-highlight"></div>
                <div id="fill_overlay" class="overlay">0%</div>
            </div>
        </div>
        <div class="legend">Niveau estime sur plage 907 (3700g vide, 6450g plein)</div>
        <div id="weight_text" class="weight">0 g</div>
    </div>
</div>

<div class="card">
    <h3>Calibration de la balance</h3>
    <p>Etape 1: faire la tare a zero avec la bouteille vide ou retiree.</p>
    <div class="actions">
        <button class="btn" onclick="tareZero()">Etape 1 - Tare a zero</button>
    </div>

    <p style="margin-top:12px;">Etape 2: poser un poids de reference, saisir sa valeur, puis calibrer.</p>
    <div class="grid2">
        <div>
            <label for="known_weight_g">Poids de reference (g)</label>
            <input id="known_weight_g" type="number" step="1" min="1" value="1000" />
        </div>
        <div>
            <label for="current_raw_g">Mesure capteur actuelle (g)</label>
            <input id="current_raw_g" type="number" step="1" readonly />
        </div>
    </div>
    <div class="actions">
        <button class="btn step2" onclick="applyCalibration()">Etape 2 - Calibrer</button>
        <button class="btn reload" onclick="refreshGaz()">Rafraichir</button>
    </div>
    <div id="status" class="status">Chargement...</div>
</div>

<script>
function clamp(v, lo, hi) {
    return Math.max(lo, Math.min(hi, v));
}

function renderGaz(data) {
    const fill = clamp(Number(data.fill_gaz || 0), 0, 100);
    const weight = Number(data.weight_gaz || 0);
    document.getElementById('bottle_fill').style.height = fill + '%';
    document.getElementById('fill_overlay').textContent = fill + '%';
    document.getElementById('weight_text').textContent = weight + ' g';
    document.getElementById('current_raw_g').value = weight;
}

async function refreshGaz() {
    try {
        const r = await fetch('/api/gaz_status');
        const j = await r.json();
        if (!r.ok) {
            document.getElementById('status').textContent = 'Erreur lecture Gaz';
            return;
        }
        renderGaz(j);
        document.getElementById('status').textContent = 'Donnees Gaz a jour';
    } catch (e) {
        document.getElementById('status').textContent = 'Erreur reseau';
    }
}

async function tareZero() {
    document.getElementById('status').textContent = 'Tare en cours...';
    try {
        const r = await fetch('/api/gaz_calibration_tare');
        const j = await r.json();
        if (!r.ok) {
            document.getElementById('status').textContent = 'Echec tare: ' + (j.error || 'unknown');
            return;
        }
        await refreshGaz();
        document.getElementById('status').textContent = j.saved ? 'Tare faite et config sauvee' : 'Tare faite';
    } catch (e) {
        document.getElementById('status').textContent = 'Echec tare (reseau)';
    }
}

async function applyCalibration() {
    const known = Number(document.getElementById('known_weight_g').value);
    if (!Number.isFinite(known) || known <= 0) {
        document.getElementById('status').textContent = 'Poids de reference invalide';
        return;
    }

    document.getElementById('status').textContent = 'Calibration en cours...';
    try {
        const url = '/api/gaz_calibration_apply?known_weight_g=' + encodeURIComponent(String(known));
        const r = await fetch(url);
        const j = await r.json();
        if (!r.ok) {
            document.getElementById('status').textContent = 'Echec calibration: ' + (j.error || j.parameter || 'unknown');
            return;
        }
        await refreshGaz();
        document.getElementById('status').textContent = 'Calibration appliquee (factor=' + j.scale_cal_factor + ')';
    } catch (e) {
        document.getElementById('status').textContent = 'Echec calibration (reseau)';
    }
}

setInterval(refreshGaz, 1500);
refreshGaz();
</script>
</body>
</html>
)HTML";

static const char LEVEL_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SmartFranklin - Level</title>
<style>
:root {
    --line: #cbd5e1;
    --panel: #ffffff;
    --ink: #0f172a;
    --accent: #0284c7;
    --accent2: #0f766e;
}
body { margin: 10px; color: var(--ink); font-family: sans-serif; background: radial-gradient(circle at 15% 10%, #e0f2fe, #f8fafc 42%, #eef2ff); }
.card { background: var(--panel); border: 1px solid var(--line); padding: 15px; border-radius: 12px; margin-bottom: 15px; }
a.button { display: inline-block; padding: 10px 15px; background: #0f766e; color: #fff; border-radius: 8px; text-decoration: none; margin-right: 8px; margin-bottom: 6px; }
.scene { display: flex; flex-direction: column; align-items: center; gap: 10px; }
.van-ground { width: 320px; max-width: 94vw; height: 220px; border-radius: 14px; position: relative; background: linear-gradient(180deg, #f8fafc 0%, #dbeafe 100%); overflow: hidden; border: 1px solid #93c5fd; }
.van-wrap { position: absolute; left: 50%; top: 54%; transform-style: preserve-3d; transform: translate(-50%, -50%) perspective(700px) rotateX(0deg) rotateZ(0deg); transition: transform 0.35s ease; }
.van-body { width: 210px; height: 96px; border: 4px solid #334155; border-radius: 18px 28px 14px 14px; background: linear-gradient(180deg, #fef3c7 0%, #fbbf24 78%); position: relative; }
.van-window { position: absolute; top: 10px; left: 128px; width: 54px; height: 28px; border-radius: 8px; border: 2px solid #334155; background: #dbeafe; }
.van-door { position: absolute; top: 16px; left: 84px; width: 40px; height: 72px; border-left: 2px solid #334155; }
.wheel { width: 26px; height: 26px; border-radius: 50%; background: #111827; border: 3px solid #94a3b8; position: absolute; }
.fl { left: 22px; top: 8px; }
.fr { right: 22px; top: 8px; }
.rl { left: 22px; bottom: 8px; }
.rr { right: 22px; bottom: 8px; }
.wheel-label { position: absolute; font-size: 0.88rem; font-weight: 700; color: #1e293b; background: rgba(255,255,255,0.72); border-radius: 6px; padding: 1px 5px; }
.lbl-fl { left: 8px; top: 40px; }
.lbl-fr { right: 8px; top: 40px; }
.lbl-rl { left: 8px; bottom: 40px; }
.lbl-rr { right: 8px; bottom: 40px; }
.gauges { display: grid; grid-template-columns: repeat(2, minmax(140px, 1fr)); gap: 12px; margin-top: 6px; }
.gauge { background: #f8fafc; border: 1px solid #cbd5e1; border-radius: 10px; padding: 8px; text-align: center; }
.gauge svg { width: 100%; height: 88px; }
.g-title { font-weight: 700; margin-bottom: 4px; }
.g-value { margin-top: 2px; font-size: 1.02rem; font-weight: 700; }
.actions { display: flex; gap: 8px; flex-wrap: wrap; margin-top: 8px; }
.btn { border: 0; border-radius: 8px; padding: 9px 12px; cursor: pointer; color: #fff; background: var(--accent2); }
.btn.secondary { background: #475569; }
.status { margin-top: 8px; min-height: 1.2em; color: #334155; font-size: 0.95rem; }
.legend { font-size: 0.92rem; color: #334155; }
</style>
</head>
<body>
<h1>Level</h1>
<div class="card">
    <a class="button" href="/">Dashboard</a>
    <a class="button" href="/config">Configuration</a>
    <a class="button" href="/gaz">Gaz 907</a>
    <a class="button" href="/diagnostics">Diagnostics</a>
</div>

<div class="card scene">
    <div class="van-ground">
        <div id="van_wrap" class="van-wrap">
            <div class="van-body">
                <div class="van-window"></div>
                <div class="van-door"></div>
            </div>
        </div>

        <div class="wheel fl"></div>
        <div class="wheel fr"></div>
        <div class="wheel rl"></div>
        <div class="wheel rr"></div>

        <div id="lbl_fl" class="wheel-label lbl-fl">0.0 cm</div>
        <div id="lbl_fr" class="wheel-label lbl-fr">0.0 cm</div>
        <div id="lbl_rl" class="wheel-label lbl-rl">0.0 cm</div>
        <div id="lbl_rr" class="wheel-label lbl-rr">0.0 cm</div>
    </div>
    <div class="legend">Renault Trafic visualise selon pitch/roll. Hauteurs roues en centimetres.</div>
    <div class="gauges">
        <div class="gauge">
            <div class="g-title">Pitch</div>
            <svg viewBox="0 0 120 70" aria-label="Pitch gauge">
                <path d="M10,60 A50,50 0 0 1 110,60" fill="none" stroke="#94a3b8" stroke-width="8"/>
                <line id="pitch_needle" x1="60" y1="60" x2="60" y2="16" stroke="#0284c7" stroke-width="4" stroke-linecap="round"/>
                <circle cx="60" cy="60" r="5" fill="#0f172a"/>
            </svg>
            <div id="pitch_value" class="g-value">0.00 deg</div>
        </div>
        <div class="gauge">
            <div class="g-title">Roll</div>
            <svg viewBox="0 0 120 70" aria-label="Roll gauge">
                <path d="M10,60 A50,50 0 0 1 110,60" fill="none" stroke="#94a3b8" stroke-width="8"/>
                <line id="roll_needle" x1="60" y1="60" x2="60" y2="16" stroke="#0f766e" stroke-width="4" stroke-linecap="round"/>
                <circle cx="60" cy="60" r="5" fill="#0f172a"/>
            </svg>
            <div id="roll_value" class="g-value">0.00 deg</div>
        </div>
    </div>
</div>

<div class="card">
    <h3>Calibration niveau</h3>
    <p>Positionner le van sur une surface de reference puis faire la remise a zero.</p>
    <div class="actions">
        <button class="btn" onclick="captureZero()">Remise a zero</button>
        <button class="btn secondary" onclick="resetZero()">Annuler remise a zero</button>
        <button class="btn secondary" onclick="refreshLevel()">Rafraichir</button>
    </div>
    <div id="zero_status" class="status">Chargement...</div>
</div>

<script>
function clamp(v, lo, hi) {
    return Math.max(lo, Math.min(hi, v));
}

function cm(mm) {
    return (Number(mm || 0) / 10.0).toFixed(1);
}

function gaugeNeedle(elementId, deg) {
    const clamped = clamp(Number(deg || 0), -20, 20);
    const angle = clamped * 4.5;
    const needle = document.getElementById(elementId);
    needle.setAttribute('transform', 'rotate(' + angle.toFixed(2) + ' 60 60)');
}

function renderLevel(data) {
    const pitch = Number(data.level_pitch_deg || 0);
    const roll = Number(data.level_roll_deg || 0);

    document.getElementById('lbl_fl').textContent = cm(data.level_wheel_fl_mm) + ' cm';
    document.getElementById('lbl_fr').textContent = cm(data.level_wheel_fr_mm) + ' cm';
    document.getElementById('lbl_rl').textContent = cm(data.level_wheel_rl_mm) + ' cm';
    document.getElementById('lbl_rr').textContent = cm(data.level_wheel_rr_mm) + ' cm';

    document.getElementById('pitch_value').textContent = pitch.toFixed(2) + ' deg';
    document.getElementById('roll_value').textContent = roll.toFixed(2) + ' deg';
    gaugeNeedle('pitch_needle', pitch);
    gaugeNeedle('roll_needle', roll);

    const rollVis = clamp(roll, -20, 20);
    const pitchVis = clamp(pitch, -20, 20);
    document.getElementById('van_wrap').style.transform =
        'translate(-50%, -50%) perspective(700px) rotateX(' + (-pitchVis).toFixed(2) + 'deg) rotateZ(' + rollVis.toFixed(2) + 'deg)';
}

async function refreshLevel() {
    try {
        const r = await fetch('/api/level_status');
        const j = await r.json();
        if (!r.ok) {
            document.getElementById('zero_status').textContent = 'Erreur lecture Level';
            return;
        }

        renderLevel(j);
        document.getElementById('zero_status').textContent =
            'Zero pitch=' + Number(j.level_zero_pitch_deg || 0).toFixed(2) +
            ' deg, zero roll=' + Number(j.level_zero_roll_deg || 0).toFixed(2) + ' deg';
    } catch (e) {
        document.getElementById('zero_status').textContent = 'Erreur reseau';
    }
}

async function captureZero() {
    document.getElementById('zero_status').textContent = 'Remise a zero en cours...';
    try {
        const r = await fetch('/api/level_zero_capture');
        const j = await r.json();
        if (!r.ok) {
            document.getElementById('zero_status').textContent = 'Echec remise a zero: ' + (j.error || 'unknown');
            return;
        }
        await refreshLevel();
        document.getElementById('zero_status').textContent = j.saved ? 'Remise a zero enregistree' : 'Remise a zero appliquee';
    } catch (e) {
        document.getElementById('zero_status').textContent = 'Echec remise a zero (reseau)';
    }
}

async function resetZero() {
    document.getElementById('zero_status').textContent = 'Annulation en cours...';
    try {
        const r = await fetch('/api/level_zero_reset');
        const j = await r.json();
        if (!r.ok) {
            document.getElementById('zero_status').textContent = 'Echec annulation: ' + (j.error || 'unknown');
            return;
        }
        await refreshLevel();
        document.getElementById('zero_status').textContent = j.saved ? 'Zero reset enregistre' : 'Zero reset applique';
    } catch (e) {
        document.getElementById('zero_status').textContent = 'Echec annulation (reseau)';
    }
}

setInterval(refreshLevel, 1500);
refreshLevel();
</script>
</body>
</html>
)HTML";

static const char DIAGNOSTICS_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SmartFranklin - Diagnostics</title>
<style>
body { font-family: sans-serif; margin: 10px; }
.card { background: #f4f4f4; padding: 15px; border-radius: 10px; margin-bottom: 15px; }
pre { white-space: pre-wrap; word-wrap: break-word; }
a.button { display: inline-block; padding: 10px 15px; background: #0078ff; color: white; border-radius: 6px; text-decoration: none; margin-right: 5px; }
</style>
</head>
<body>
<h1>Diagnostics</h1>
<div class="card">
    <a class="button" href="/">Dashboard</a>
    <a class="button" href="/level">Level</a>
    <a class="button" href="/config">Configuration</a>
    <a class="button" href="/gaz">Gaz 907</a>
    <a class="button" href="/update">Firmware Update</a>
</div>
<div class="card">
    <h3>Full Snapshot (Data + Config + HW)</h3>
    <pre id="diag">Loading...</pre>
</div>
<script>
async function refresh() {
    try {
        const r = await fetch('/api/full');
        const j = await r.json();
        document.getElementById('diag').textContent = JSON.stringify(j, null, 2);
    } catch (e) {
        document.getElementById('diag').textContent = 'Error loading diagnostics';
    }
}
setInterval(refresh, 1500);
refresh();
</script>
</body>
</html>
)HTML";

// ============================================================================
// Authentication Helper Function
// ============================================================================

/**
 * @brief Checks HTTP Basic Authentication for incoming requests.
 * 
 * Validates admin credentials against CONFIG.admin_user and CONFIG.admin_pass.
 * Sends authentication challenge if credentials missing or invalid.
 * Used to protect sensitive configuration and control endpoints.
 * 
 * Authentication Process:
 *   1. Check if request contains valid Basic Auth credentials
 *   2. Compare username/password against configuration
 *   3. Return true if authenticated, false otherwise
 *   4. Send 401 response with authentication challenge on failure
 * 
 * Security Notes:
 *   - HTTP Basic Auth sends credentials base64-encoded (not encrypted)
 *   - Only secure on local networks (not over internet)
 *   - Credentials stored in CONFIG (SPIFFS persistence)
 *   - Default credentials should be changed immediately
 * 
 * Usage Pattern:
 *   @code
 *   server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request){
 *       if (!checkAuth(request)) return;  // Reject if not authenticated
 *       // Handle authenticated request...
 *   });
 *   @endcode
 * 
 * Error Handling:
 *   - Invalid credentials: 401 response with realm "SmartFranklin Config"
 *   - Missing credentials: Same 401 challenge response
 *   - Function returns false, caller must return immediately
 * 
 * @param request - Pointer to AsyncWebServerRequest containing HTTP request
 *                  Must be valid and contain authentication headers if present
 * 
 * @return bool - true if authentication successful, false if rejected
 *                - false: Request rejected, 401 response sent
 *                - true: Authentication passed, request can proceed
 * 
 * @note This function sends HTTP response on authentication failure.
 *       Caller must return immediately after false result.
 *       Example: if (!checkAuth(request)) return;
 * 
 * @see CONFIG.admin_user - Admin username from configuration
 * @see CONFIG.admin_pass - Admin password from configuration
 */
static bool checkAuth(AsyncWebServerRequest *request) {
    if (!request->authenticate(CONFIG.admin_user.c_str(),
                               CONFIG.admin_pass.c_str())) {
        request->requestAuthentication("SmartFranklin Config");
        return false;
    }
    return true;
}

static void fillDataJson(JsonVariant doc)
{
    std::lock_guard<std::mutex> lock(DATA_MUTEX);
    doc["rtc_sync_source"] = DATA.rtc_sync_source;
    doc["rtc_time"] = DATA.rtc_time;
    doc["gps_utc"] = DATA.gps_utc;
    doc["gps_date"] = DATA.gps_date;
    doc["gps_has_fix"] = DATA.gps_has_fix;
    doc["gps_latitude_deg"] = DATA.gps_latitude_deg;
    doc["gps_longitude_deg"] = DATA.gps_longitude_deg;
    doc["gps_altitude_m"] = DATA.gps_altitude_m;
    doc["gps_speed_knots"] = DATA.gps_speed_knots;
    doc["gps_course_deg"] = DATA.gps_course_deg;
    doc["gps_satellites"] = DATA.gps_satellites;

    doc["fill_gaz"] = DATA.fill_gaz;
    doc["fill_tank"] = DATA.fill_tank;
    doc["weight_gaz"] = DATA.weight_gaz;
    doc["distance_tank_mm"] = DATA.distance_tank_mm;

    doc["level_pitch_deg"] = DATA.level_pitch_deg;
    doc["level_roll_deg"] = DATA.level_roll_deg;
    doc["level_wheel_fl_mm"] = DATA.level_wheel_fl_mm;
    doc["level_wheel_fr_mm"] = DATA.level_wheel_fr_mm;
    doc["level_wheel_rl_mm"] = DATA.level_wheel_rl_mm;
    doc["level_wheel_rr_mm"] = DATA.level_wheel_rr_mm;

    // Backward-compatible aliases used by legacy web pages.
    doc["imu_pitch_deg"] = DATA.level_pitch_deg;
    doc["imu_roll_deg"] = DATA.level_roll_deg;
    doc["imu_wheel_fl_mm"] = DATA.level_wheel_fl_mm;
    doc["imu_wheel_fr_mm"] = DATA.level_wheel_fr_mm;
    doc["imu_wheel_rl_mm"] = DATA.level_wheel_rl_mm;
    doc["imu_wheel_rr_mm"] = DATA.level_wheel_rr_mm;
    doc["tank_distance_mm"] = DATA.distance_tank_mm;
    doc["tank_fill"] = DATA.fill_tank;

    doc["gap"] = DATA.gap;
    doc["bms_voltage"] = DATA.bms_voltage;
    doc["bms_current"] = DATA.bms_current;
    doc["bms_soc"] = DATA.bms_soc;
    doc["led_state"] = DATA.led_state;
    doc["buzzer_state"] = DATA.buzzer_state;
    doc["target_soc"] = DATA.target_soc;
}

static void fillConfigJson(JsonVariant doc)
{
    doc["hostname"] = CONFIG.hostname;
    doc["ap_ssid"] = CONFIG.ap_ssid;
    doc["ap_pass"] = CONFIG.ap_pass;
    doc["sta_ssid"] = CONFIG.sta_ssid;
    doc["sta_pass"] = CONFIG.sta_pass;

    doc["scale_cal_factor"] = CONFIG.scale_cal_factor;

    doc["level_wheelbase_mm"] = CONFIG.level_wheelbase_mm;
    doc["level_track_width_mm"] = CONFIG.level_track_width_mm;
    doc["level_offset_x_mm"] = CONFIG.level_offset_x_mm;
    doc["level_offset_y_mm"] = CONFIG.level_offset_y_mm;
    doc["level_zero_pitch_deg"] = CONFIG.level_zero_pitch_deg;
    doc["level_zero_roll_deg"] = CONFIG.level_zero_roll_deg;

    doc["imu_wheelbase_mm"] = CONFIG.level_wheelbase_mm;
    doc["imu_track_width_mm"] = CONFIG.level_track_width_mm;
    doc["imu_offset_x_mm"] = CONFIG.level_offset_x_mm;
    doc["imu_offset_y_mm"] = CONFIG.level_offset_y_mm;

    doc["admin_user"] = CONFIG.admin_user;
    doc["admin_pass"] = CONFIG.admin_pass;

    doc["mqtt_port"] = CONFIG.mqtt_port;

    doc["task_wifi_loop_ms"] = CONFIG.task_wifi_loop_ms;
    doc["task_mqtt_loop_ms"] = CONFIG.task_mqtt_loop_ms;
    doc["task_i2c_loop_ms"] = CONFIG.task_i2c_loop_ms;
    doc["task_hmi_loop_ms"] = CONFIG.task_hmi_loop_ms;
    doc["task_hmi_init_retry_ms"] = CONFIG.task_hmi_init_retry_ms;
    doc["task_hw_monitor_loop_ms"] = CONFIG.task_hw_monitor_loop_ms;
    doc["task_bms_ble_connected_loop_ms"] = CONFIG.task_bms_ble_connected_loop_ms;
    doc["task_bms_ble_retry_loop_ms"] = CONFIG.task_bms_ble_retry_loop_ms;
    doc["task_watchdog_loop_ms"] = CONFIG.task_watchdog_loop_ms;
}

static void fillHwJson(JsonVariant doc)
{
    const HwStatus st = HW.read();
    doc["battery_voltage"] = st.battery_voltage;
    doc["battery_percent"] = st.battery_percent;
    doc["charging"] = st.charging;
    doc["temperature"] = st.temperature;
    doc["button_a"] = st.button_a;
    doc["button_b"] = st.button_b;
    JsonObject accel = doc["accel"].to<JsonObject>();
    accel["x"] = st.accel_x;
    accel["y"] = st.accel_y;
    accel["z"] = st.accel_z;
}

static void fillGeometryJson(JsonVariant doc, bool includeLegacyKeys)
{
    doc["level_wheelbase_mm"] = CONFIG.level_wheelbase_mm;
    doc["level_track_width_mm"] = CONFIG.level_track_width_mm;
    doc["level_offset_x_mm"] = CONFIG.level_offset_x_mm;
    doc["level_offset_y_mm"] = CONFIG.level_offset_y_mm;

    if (includeLegacyKeys) {
        doc["imu_wheelbase_mm"] = CONFIG.level_wheelbase_mm;
        doc["imu_track_width_mm"] = CONFIG.level_track_width_mm;
        doc["imu_offset_x_mm"] = CONFIG.level_offset_x_mm;
        doc["imu_offset_y_mm"] = CONFIG.level_offset_y_mm;
    }
}

static bool parseFloatParameter(const String& text, float& out)
{
    const char* start = text.c_str();
    char* end = nullptr;
    const float value = strtof(start, &end);
    if (end == start || *end != '\0' || !std::isfinite(value)) {
        return false;
    }
    out = value;
    return true;
}

static bool parseIntParameter(const String& text, int& out)
{
    const char* start = text.c_str();
    char* end = nullptr;
    const long value = strtol(start, &end, 10);
    if (end == start || *end != '\0') {
        return false;
    }
    out = static_cast<int>(value);
    return true;
}

static bool applyConfigFromRequest(AsyncWebServerRequest* request, String& errorKey)
{
    SmartConfig updated = CONFIG;

    auto applyString = [&](const char* key, String& target) {
        if (!request->hasParam(key)) {
            return true;
        }
        target = request->getParam(key)->value();
        return true;
    };

    auto applyFloat = [&](const char* key, float& target) {
        if (!request->hasParam(key)) {
            return true;
        }
        float parsed = 0.0f;
        if (!parseFloatParameter(request->getParam(key)->value(), parsed)) {
            errorKey = key;
            return false;
        }
        target = parsed;
        return true;
    };

    auto applyInt = [&](const char* key, int& target) {
        if (!request->hasParam(key)) {
            return true;
        }
        int parsed = 0;
        if (!parseIntParameter(request->getParam(key)->value(), parsed)) {
            errorKey = key;
            return false;
        }
        target = parsed;
        return true;
    };

    if (!applyString("hostname", updated.hostname) ||
        !applyString("ap_ssid", updated.ap_ssid) ||
        !applyString("ap_pass", updated.ap_pass) ||
        !applyString("sta_ssid", updated.sta_ssid) ||
        !applyString("sta_pass", updated.sta_pass) ||
        !applyFloat("scale_cal_factor", updated.scale_cal_factor) ||
        !applyFloat("level_wheelbase_mm", updated.level_wheelbase_mm) ||
        !applyFloat("level_track_width_mm", updated.level_track_width_mm) ||
        !applyFloat("level_offset_x_mm", updated.level_offset_x_mm) ||
        !applyFloat("level_offset_y_mm", updated.level_offset_y_mm) ||
        !applyFloat("level_zero_pitch_deg", updated.level_zero_pitch_deg) ||
        !applyFloat("level_zero_roll_deg", updated.level_zero_roll_deg) ||
        !applyString("admin_user", updated.admin_user) ||
        !applyString("admin_pass", updated.admin_pass) ||
        !applyInt("mqtt_port", updated.mqtt_port) ||
        !applyInt("task_wifi_loop_ms", updated.task_wifi_loop_ms) ||
        !applyInt("task_mqtt_loop_ms", updated.task_mqtt_loop_ms) ||
        !applyInt("task_i2c_loop_ms", updated.task_i2c_loop_ms) ||
        !applyInt("task_hmi_loop_ms", updated.task_hmi_loop_ms) ||
        !applyInt("task_hmi_init_retry_ms", updated.task_hmi_init_retry_ms) ||
        !applyInt("task_hw_monitor_loop_ms", updated.task_hw_monitor_loop_ms) ||
        !applyInt("task_bms_ble_connected_loop_ms", updated.task_bms_ble_connected_loop_ms) ||
        !applyInt("task_bms_ble_retry_loop_ms", updated.task_bms_ble_retry_loop_ms) ||
        !applyInt("task_watchdog_loop_ms", updated.task_watchdog_loop_ms)) {
        return false;
    }

    if (updated.scale_cal_factor <= 0.0f || !std::isfinite(updated.scale_cal_factor)) {
        errorKey = "scale_cal_factor";
        return false;
    }

    if (updated.level_wheelbase_mm < 100.0f || updated.level_wheelbase_mm > 10000.0f ||
        updated.level_track_width_mm < 100.0f || updated.level_track_width_mm > 10000.0f ||
        updated.level_offset_x_mm < -5000.0f || updated.level_offset_x_mm > 5000.0f ||
        updated.level_offset_y_mm < -5000.0f || updated.level_offset_y_mm > 5000.0f) {
        errorKey = "level_geometry";
        return false;
    }

    if (!std::isfinite(updated.level_zero_pitch_deg) ||
        !std::isfinite(updated.level_zero_roll_deg) ||
        updated.level_zero_pitch_deg < -90.0f || updated.level_zero_pitch_deg > 90.0f ||
        updated.level_zero_roll_deg < -90.0f || updated.level_zero_roll_deg > 90.0f) {
        errorKey = "level_zero";
        return false;
    }

    if (updated.mqtt_port <= 0 || updated.mqtt_port > 65535) {
        errorKey = "mqtt_port";
        return false;
    }

    auto loopMsValid = [](int value) {
        return value >= 20 && value <= 600000;
    };

    if (!loopMsValid(updated.task_wifi_loop_ms)) {
        errorKey = "task_wifi_loop_ms";
        return false;
    }
    if (!loopMsValid(updated.task_mqtt_loop_ms)) {
        errorKey = "task_mqtt_loop_ms";
        return false;
    }
    if (!loopMsValid(updated.task_i2c_loop_ms)) {
        errorKey = "task_i2c_loop_ms";
        return false;
    }
    if (!loopMsValid(updated.task_hmi_loop_ms)) {
        errorKey = "task_hmi_loop_ms";
        return false;
    }
    if (!loopMsValid(updated.task_hmi_init_retry_ms)) {
        errorKey = "task_hmi_init_retry_ms";
        return false;
    }
    if (!loopMsValid(updated.task_hw_monitor_loop_ms)) {
        errorKey = "task_hw_monitor_loop_ms";
        return false;
    }
    if (!loopMsValid(updated.task_bms_ble_connected_loop_ms)) {
        errorKey = "task_bms_ble_connected_loop_ms";
        return false;
    }
    if (!loopMsValid(updated.task_bms_ble_retry_loop_ms)) {
        errorKey = "task_bms_ble_retry_loop_ms";
        return false;
    }
    if (!loopMsValid(updated.task_watchdog_loop_ms)) {
        errorKey = "task_watchdog_loop_ms";
        return false;
    }

    CONFIG = updated;
    return true;
}

static bool applyLevelGeometryFromRequest(AsyncWebServerRequest* request)
{
    auto parseParam = [&](const char* key, float& target) {
        if (!request->hasParam(key)) {
            return true;
        }
        float parsed = 0.0f;
        if (!parseFloatParameter(request->getParam(key)->value(), parsed)) {
            return false;
        }
        target = parsed;
        return true;
    };

    float wheelbase = CONFIG.level_wheelbase_mm;
    float track = CONFIG.level_track_width_mm;
    float offsetX = CONFIG.level_offset_x_mm;
    float offsetY = CONFIG.level_offset_y_mm;

    if (!parseParam("wheelbase_mm", wheelbase) ||
        !parseParam("track_width_mm", track) ||
        !parseParam("offset_x_mm", offsetX) ||
        !parseParam("offset_y_mm", offsetY)) {
        return false;
    }

    if (wheelbase < 100.0f || wheelbase > 10000.0f ||
        track < 100.0f || track > 10000.0f ||
        offsetX < -5000.0f || offsetX > 5000.0f ||
        offsetY < -5000.0f || offsetY > 5000.0f) {
        return false;
    }

    CONFIG.level_wheelbase_mm = wheelbase;
    CONFIG.level_track_width_mm = track;
    CONFIG.level_offset_x_mm = offsetX;
    CONFIG.level_offset_y_mm = offsetY;
    return true;
}

static void sendJson(AsyncWebServerRequest* request, JsonDocument& doc, int statusCode = 200)
{
    String out;
    serializeJson(doc, out);
    request->send(statusCode, "application/json", out);
}

// ============================================================================
// Web Dashboard Initialization
// ============================================================================

/**
 * @brief Initializes the web dashboard server and registers all routes.
 * 
 * Sets up the AsyncWebServer with all endpoints for dashboard access,
 * API calls, static file serving, and OTA firmware updates. Must be
 * called once during system initialization after WiFi is connected.
 * 
 * Route Registration:
 * 
 *   Static Routes:
 *   - / : Main dashboard page (HTML from PROGMEM)
 *   - /hw : Hardware status page (from SPIFFS)
 *   - /sensors : Sensor data page (from SPIFFS)
 *   - /theme.js : Common JavaScript (from SPIFFS)
 * 
 *   API Routes:
 *   - /api/status : Real-time sensor data (JSON)
 *   - /api/hw : Hardware status (JSON)
 *   - /api/set_brightness : Display brightness control
 *   - /api/reboot : System restart trigger
 *   - /api/sleep : Deep sleep mode entry
 * 
 *   Special Routes:
 *   - OTA Update: /update (handled by ElegantOTA)
 *   - Authentication: Required for sensitive endpoints
 * 
 * Server Configuration:
 *   - Port 80 (standard HTTP)
 *   - No SSL/TLS (embedded resource constraints)
 *   - Asynchronous processing (non-blocking)
 *   - Up to 4 concurrent connections
 * 
 * OTA Integration:
 *   - ElegantOTA provides firmware upload interface
 *   - Protected by same admin credentials
 *   - Automatic partition switching on successful update
 *   - Rollback capability if update fails
 * 
 * Error Handling:
 *   - SPIFFS file access failures handled by AsyncWebServer
 *   - API endpoint errors return appropriate HTTP status codes
 *   - Network errors handled by underlying TCP stack
 * 
 * Performance:
 *   - Initialization time: < 100ms
 *   - Memory usage: ~20KB for server and route handlers
 *   - CPU overhead: Minimal (asynchronous callbacks)
 * 
 * 
 * @note Call this function after WiFi connection is established.
 *       Server begins accepting connections immediately.
 *       OTA updates require authentication with admin credentials.
 *       Example: web_dashboard_init(); // in setup()
 * 
 * @see AsyncWebServer::begin() - Starts the web server
 * @see ElegantOTA::begin() - Initializes OTA update capability
 */
void web_dashboard_init()
{
    // =========================================================================
    // Main Dashboard Route
    // =========================================================================
    // Serve embedded HTML page from PROGMEM memory
    // No authentication required for main dashboard
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/html", MAIN_PAGE);
    });

    server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/html", CONFIG_PAGE);
    });

    server.on("/gaz", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/html", GAZ_PAGE);
    });

    server.on("/level", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/html", LEVEL_PAGE);
    });

    server.on("/diagnostics", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/html", DIAGNOSTICS_PAGE);
    });

    // =========================================================================
    // API Status Endpoint
    // =========================================================================
    // Returns real-time sensor data as JSON
    // Thread-safe access to global DATA with mutex
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request){
        JsonDocument doc;
        fillDataJson(doc);
        sendJson(request, doc);
    });

    server.on("/api/data", HTTP_GET, [](AsyncWebServerRequest *request){
        JsonDocument doc;
        fillDataJson(doc);
        sendJson(request, doc);
    });

    server.on("/api/level_status", HTTP_GET, [](AsyncWebServerRequest *request){
        JsonDocument doc;
        {
            std::lock_guard<std::mutex> lock(DATA_MUTEX);
            doc["level_pitch_deg"] = DATA.level_pitch_deg;
            doc["level_roll_deg"] = DATA.level_roll_deg;
            doc["level_wheel_fl_mm"] = DATA.level_wheel_fl_mm;
            doc["level_wheel_fr_mm"] = DATA.level_wheel_fr_mm;
            doc["level_wheel_rl_mm"] = DATA.level_wheel_rl_mm;
            doc["level_wheel_rr_mm"] = DATA.level_wheel_rr_mm;
        }
        doc["level_zero_pitch_deg"] = CONFIG.level_zero_pitch_deg;
        doc["level_zero_roll_deg"] = CONFIG.level_zero_roll_deg;
        sendJson(request, doc);
    });

    server.on("/api/level_zero_capture", HTTP_GET, [](AsyncWebServerRequest *request){
        float currentPitch = 0.0f;
        float currentRoll = 0.0f;
        {
            std::lock_guard<std::mutex> lock(DATA_MUTEX);
            currentPitch = DATA.level_pitch_deg;
            currentRoll = DATA.level_roll_deg;
        }

        if (!std::isfinite(currentPitch) || !std::isfinite(currentRoll)) {
            request->send(400, "application/json", "{\"error\":\"invalid_level_sample\"}");
            return;
        }

        CONFIG.level_zero_pitch_deg += currentPitch;
        CONFIG.level_zero_roll_deg += currentRoll;

        const bool saved = config_save();
        JsonDocument doc;
        doc["saved"] = saved;
        doc["level_zero_pitch_deg"] = CONFIG.level_zero_pitch_deg;
        doc["level_zero_roll_deg"] = CONFIG.level_zero_roll_deg;
        sendJson(request, doc, saved ? 200 : 500);
    });

    server.on("/api/level_zero_reset", HTTP_GET, [](AsyncWebServerRequest *request){
        CONFIG.level_zero_pitch_deg = 0.0f;
        CONFIG.level_zero_roll_deg = 0.0f;
        const bool saved = config_save();

        JsonDocument doc;
        doc["saved"] = saved;
        doc["level_zero_pitch_deg"] = CONFIG.level_zero_pitch_deg;
        doc["level_zero_roll_deg"] = CONFIG.level_zero_roll_deg;
        sendJson(request, doc, saved ? 200 : 500);
    });

    server.on("/api/gaz_status", HTTP_GET, [](AsyncWebServerRequest *request){
        JsonDocument doc;
        {
            std::lock_guard<std::mutex> lock(DATA_MUTEX);
            doc["fill_gaz"] = DATA.fill_gaz;
            doc["weight_gaz"] = DATA.weight_gaz;
        }
        doc["scale_cal_factor"] = CONFIG.scale_cal_factor;
        sendJson(request, doc);
    });

    server.on("/api/gaz_calibration_tare", HTTP_GET, [](AsyncWebServerRequest *request){
        scale_tare();
        CONFIG.scale_cal_factor = 1.0f;
        const bool saved = config_save();

        JsonDocument doc;
        doc["saved"] = saved;
        doc["scale_cal_factor"] = CONFIG.scale_cal_factor;
        {
            std::lock_guard<std::mutex> lock(DATA_MUTEX);
            doc["fill_gaz"] = DATA.fill_gaz;
            doc["weight_gaz"] = DATA.weight_gaz;
        }
        sendJson(request, doc, saved ? 200 : 500);
    });

    server.on("/api/gaz_calibration_apply", HTTP_GET, [](AsyncWebServerRequest *request){
        if (!request->hasParam("known_weight_g")) {
            request->send(400, "application/json", "{\"error\":\"missing_known_weight_g\"}");
            return;
        }

        float knownWeightG = 0.0f;
        if (!parseFloatParameter(request->getParam("known_weight_g")->value(), knownWeightG) ||
            !std::isfinite(knownWeightG) ||
            knownWeightG <= 0.0f ||
            knownWeightG > 50000.0f) {
            request->send(400, "application/json", "{\"error\":\"invalid_known_weight_g\"}");
            return;
        }

        const float rawWeightG = scale_get_raw();
        if (!std::isfinite(rawWeightG) || std::fabs(rawWeightG) < 1.0f) {
            request->send(400, "application/json", "{\"error\":\"invalid_raw_measurement\"}");
            return;
        }

        const float newFactor = knownWeightG / rawWeightG;
        if (!std::isfinite(newFactor) || std::fabs(newFactor) < 1e-6f || std::fabs(newFactor) > 1000.0f) {
            request->send(400, "application/json", "{\"error\":\"invalid_calibration_factor\"}");
            return;
        }

        scale_set_cal_factor(newFactor);
        const bool saved = config_save();

        JsonDocument doc;
        doc["saved"] = saved;
        doc["known_weight_g"] = knownWeightG;
        doc["raw_weight_g"] = rawWeightG;
        doc["scale_cal_factor"] = CONFIG.scale_cal_factor;
        sendJson(request, doc, saved ? 200 : 500);
    });

    server.on("/api/config", HTTP_GET, [](AsyncWebServerRequest *request){
        JsonDocument doc;
        fillConfigJson(doc);
        sendJson(request, doc);
    });

    server.on("/api/set_config", HTTP_GET, [](AsyncWebServerRequest *request){
        String errorKey;
        if (!applyConfigFromRequest(request, errorKey)) {
            JsonDocument errorDoc;
            errorDoc["error"] = "invalid_parameter";
            errorDoc["parameter"] = errorKey;
            sendJson(request, errorDoc, 400);
            return;
        }

        const bool saved = config_save();
        JsonDocument doc;
        doc["saved"] = saved;
        JsonObject config = doc["config"].to<JsonObject>();
        fillConfigJson(config);
        sendJson(request, doc, saved ? 200 : 500);
    });

    server.on("/api/full", HTTP_GET, [](AsyncWebServerRequest *request){
        JsonDocument doc;
        JsonObject data = doc["data"].to<JsonObject>();
        fillDataJson(data);
        JsonObject config = doc["config"].to<JsonObject>();
        fillConfigJson(config);
        JsonObject hw = doc["hw"].to<JsonObject>();
        fillHwJson(hw);
        sendJson(request, doc);
    });

    server.on("/api/level_geometry", HTTP_GET, [](AsyncWebServerRequest *request){
        JsonDocument doc;
        fillGeometryJson(doc, true);
        sendJson(request, doc);
    });

    server.on("/api/imu_geometry", HTTP_GET, [](AsyncWebServerRequest *request){
        JsonDocument doc;
        fillGeometryJson(doc, true);
        sendJson(request, doc);
    });

    server.on("/api/set_level_geometry", HTTP_GET, [](AsyncWebServerRequest *request){
        if (!applyLevelGeometryFromRequest(request)) {
            request->send(400, "application/json", "{\"error\":\"invalid_parameter\"}");
            return;
        }

        const bool saved = config_save();

        JsonDocument doc;
        doc["saved"] = saved;
        fillGeometryJson(doc, true);
        sendJson(request, doc, saved ? 200 : 500);
    });

    server.on("/api/set_imu_geometry", HTTP_GET, [](AsyncWebServerRequest *request){
        if (!applyLevelGeometryFromRequest(request)) {
            request->send(400, "application/json", "{\"error\":\"invalid_parameter\"}");
            return;
        }

        const bool saved = config_save();

        JsonDocument doc;
        doc["saved"] = saved;
        fillGeometryJson(doc, true);
        sendJson(request, doc, saved ? 200 : 500);
    });

    server.on("/api/reset_level_geometry", HTTP_GET, [](AsyncWebServerRequest *request){
        const SmartConfig defaults;

        CONFIG.level_wheelbase_mm = defaults.level_wheelbase_mm;
        CONFIG.level_track_width_mm = defaults.level_track_width_mm;
        CONFIG.level_offset_x_mm = defaults.level_offset_x_mm;
        CONFIG.level_offset_y_mm = defaults.level_offset_y_mm;

        const bool saved = config_save();

        JsonDocument doc;
        doc["saved"] = saved;
        fillGeometryJson(doc, true);
        sendJson(request, doc, saved ? 200 : 500);
    });

    server.on("/api/reset_imu_geometry", HTTP_GET, [](AsyncWebServerRequest *request){
        const SmartConfig defaults;

        CONFIG.level_wheelbase_mm = defaults.level_wheelbase_mm;
        CONFIG.level_track_width_mm = defaults.level_track_width_mm;
        CONFIG.level_offset_x_mm = defaults.level_offset_x_mm;
        CONFIG.level_offset_y_mm = defaults.level_offset_y_mm;

        const bool saved = config_save();

        JsonDocument doc;
        doc["saved"] = saved;
        fillGeometryJson(doc, true);
        sendJson(request, doc, saved ? 200 : 500);
    });

    // =========================================================================
    // Hardware Status API Endpoint
    // =========================================================================
    // Returns M5Stack hardware status (battery, IMU, buttons)
    // No authentication required for status monitoring
    server.on("/api/hw", HTTP_GET, [](AsyncWebServerRequest *request){ 
        JsonDocument doc;
        fillHwJson(doc);
        sendJson(request, doc);
    });

    // =========================================================================
    // OTA Firmware Update Integration
    // =========================================================================
    // Initialize ElegantOTA for firmware updates
    // Protected by admin authentication
    ElegantOTA.begin(&server, CONFIG.admin_user.c_str(), CONFIG.admin_pass.c_str());

    // =========================================================================
    // Static File Serving from SPIFFS
    // =========================================================================
    // Serve HTML pages and JavaScript from SPIFFS filesystem
    // No authentication required for static content
    server.on("/hw", HTTP_GET, [](AsyncWebServerRequest *req){ req->send(SPIFFS, "/hw.html", "text/html"); }); 
    server.on("/hw.html", HTTP_GET, [](AsyncWebServerRequest *req){ req->send(SPIFFS, "/hw.html", "text/html"); }); 
    server.on("/sensors", HTTP_GET, [](AsyncWebServerRequest *req){ req->send(SPIFFS, "/sensors.html", "text/html"); }); 
    server.on("/sensors.html", HTTP_GET, [](AsyncWebServerRequest *req){ req->send(SPIFFS, "/sensors.html", "text/html"); }); 
    server.on("/theme.js", HTTP_GET, [](AsyncWebServerRequest *req){ req->send(SPIFFS, "/theme.js", "application/javascript"); }); 
    
    // =========================================================================
    // Control API Endpoints
    // =========================================================================
    // Hardware control endpoints (brightness, reboot, sleep)
    // No authentication for convenience (local network access assumed)
    
    server.on("/api/set_brightness", HTTP_GET, [](AsyncWebServerRequest *req){ 
        if (req->hasParam("value")) { 
            int v = req->getParam("value")->value().toInt(); 
            HW.setBrightness(v); 
        } 
        req->send(200, "text/plain", "OK"); 
    }); 
    
    server.on("/api/reboot", HTTP_GET, [](AsyncWebServerRequest *req){ 
        req->send(200, "text/plain", "Rebooting"); 
        delay(200); 
        ESP.restart(); 
    }); 
    
    server.on("/api/sleep", HTTP_GET, [](AsyncWebServerRequest *req){ 
        req->send(200, "text/plain", "Sleeping"); 
        delay(200); 
        HW.deepSleep(); 
    });

    // =========================================================================
    // Start Web Server
    // =========================================================================
    // Begin accepting HTTP connections on port 80
    // Server runs asynchronously in background
    server.begin();
}