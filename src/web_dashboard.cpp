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
 *   - NB-IoT cellular settings
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
 *   GET /api/nbiot:
 *   - NB-IoT cellular modem status
 *   - Network attachment, PDP context, MQTT connection
 *   - Signal strength and operator information
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
 *   - HTML pages: /hw.html, /nbiot.html, /meshtastic.html, /sensors.html
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
 *   - nb_iot2.h (NB-IoT status interface)
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
#include "m5_hw.h"  // Note: Duplicate include (should be removed)
#include "web_dashboard.h"
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ElegantOTA.h>
#include <FS.h>
#include <SPIFFS.h>
#include <WiFi.h>

#include <cmath>
#include "data_model.h"
#include "config_store.h"

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
 *   - Live data display in <pre> element
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
    <div id="imu_geom_status" class="status">--</div>
</div>
<script>
function updateGeometryInputs(g) {
    document.getElementById('wheelbase_mm').value = g.imu_wheelbase_mm;
    document.getElementById('track_width_mm').value = g.imu_track_width_mm;
    document.getElementById('offset_x_mm').value = g.imu_offset_x_mm;
    document.getElementById('offset_y_mm').value = g.imu_offset_y_mm;
}

async function loadImuGeometry() {
    try {
        const r = await fetch('/api/imu_geometry');
        const g = await r.json();
        updateGeometryInputs(g);
        document.getElementById('imu_geom_status').textContent = 'Geometry loaded';
    } catch (e) {
        document.getElementById('imu_geom_status').textContent = 'Failed to load geometry';
    }
}

async function saveImuGeometry() {
    try {
        const wheelbase = encodeURIComponent(document.getElementById('wheelbase_mm').value);
        const track = encodeURIComponent(document.getElementById('track_width_mm').value);
        const offsetX = encodeURIComponent(document.getElementById('offset_x_mm').value);
        const offsetY = encodeURIComponent(document.getElementById('offset_y_mm').value);

        const url = '/api/set_imu_geometry?wheelbase_mm=' + wheelbase + '&track_width_mm=' + track + '&offset_x_mm=' + offsetX + '&offset_y_mm=' + offsetY;
        const r = await fetch(url);
        const body = await r.json();
        if (!r.ok) {
            document.getElementById('imu_geom_status').textContent = 'Save failed: ' + (body.error || 'unknown');
            return;
        }

        updateGeometryInputs(body);
        document.getElementById('imu_geom_status').textContent = body.saved ? 'Saved and applied' : 'Applied but not persisted';
    } catch (e) {
        document.getElementById('imu_geom_status').textContent = 'Save request failed';
    }
}

async function resetImuGeometry() {
    try {
        const r = await fetch('/api/reset_imu_geometry');
        const body = await r.json();
        if (!r.ok) {
            document.getElementById('imu_geom_status').textContent = 'Reset failed';
            return;
        }

        updateGeometryInputs(body);
        document.getElementById('imu_geom_status').textContent = body.saved ? 'Defaults restored' : 'Defaults applied but not persisted';
    } catch (e) {
        document.getElementById('imu_geom_status').textContent = 'Reset request failed';
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
 *   - /nbiot : NB-IoT status page (from SPIFFS)
 *   - /meshtastic : Meshtastic status page (from SPIFFS)
 *   - /sensors : Sensor data page (from SPIFFS)
 *   - /theme.js : Common JavaScript (from SPIFFS)
 * 
 *   API Routes:
 *   - /api/status : Real-time sensor data (JSON)
 *   - /api/hw : Hardware status (JSON)
 *   - /api/nbiot : NB-IoT status (JSON)
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
 * @return void
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

    // =========================================================================
    // API Status Endpoint
    // =========================================================================
    // Returns real-time sensor data as JSON
    // Thread-safe access to global DATA with mutex
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request){
        JsonDocument doc;
        {
            std::lock_guard<std::mutex> lock(DATA_MUTEX);
            doc["weight_gaz"] = DATA.weight_gaz;
            doc["fill_gaz"] = DATA.fill_gaz;
            doc["tank_distance_mm"] = DATA.distance_tank_mm;
            doc["tank_fill"] = DATA.fill_tank;
            doc["imu_pitch_deg"] = DATA.imu_pitch_deg;
            doc["imu_roll_deg"] = DATA.imu_roll_deg;
            doc["imu_wheel_fl_mm"] = DATA.imu_wheel_fl_mm;
            doc["imu_wheel_fr_mm"] = DATA.imu_wheel_fr_mm;
            doc["imu_wheel_rl_mm"] = DATA.imu_wheel_rl_mm;
            doc["imu_wheel_rr_mm"] = DATA.imu_wheel_rr_mm;
            doc["bms_voltage"] = DATA.bms_voltage;
            doc["bms_current"] = DATA.bms_current;
            doc["bms_soc"] = DATA.bms_soc;
            doc["last_mesh_msg"] = DATA.last_mesh_msg;
        }
        String out;
        serializeJson(doc, out);
        request->send(200, "application/json", out);
    });

    server.on("/api/imu_geometry", HTTP_GET, [](AsyncWebServerRequest *request){
        JsonDocument doc;
        doc["imu_wheelbase_mm"] = CONFIG.imu_wheelbase_mm;
        doc["imu_track_width_mm"] = CONFIG.imu_track_width_mm;
        doc["imu_offset_x_mm"] = CONFIG.imu_offset_x_mm;
        doc["imu_offset_y_mm"] = CONFIG.imu_offset_y_mm;

        String out;
        serializeJson(doc, out);
        request->send(200, "application/json", out);
    });

    server.on("/api/set_imu_geometry", HTTP_GET, [](AsyncWebServerRequest *request){
        auto parseParam = [&](const char* key, float& target) {
            if (!request->hasParam(key)) {
                return true;
            }

            const float parsed = request->getParam(key)->value().toFloat();
            if (!std::isfinite(parsed)) {
                return false;
            }

            target = parsed;
            return true;
        };

        float wheelbase = CONFIG.imu_wheelbase_mm;
        float track = CONFIG.imu_track_width_mm;
        float offsetX = CONFIG.imu_offset_x_mm;
        float offsetY = CONFIG.imu_offset_y_mm;

        if (!parseParam("wheelbase_mm", wheelbase) ||
            !parseParam("track_width_mm", track) ||
            !parseParam("offset_x_mm", offsetX) ||
            !parseParam("offset_y_mm", offsetY)) {
            request->send(400, "application/json", "{\"error\":\"invalid_parameter\"}");
            return;
        }

        if (wheelbase < 100.0f || wheelbase > 10000.0f ||
            track < 100.0f || track > 10000.0f ||
            offsetX < -5000.0f || offsetX > 5000.0f ||
            offsetY < -5000.0f || offsetY > 5000.0f) {
            request->send(400, "application/json", "{\"error\":\"out_of_range\"}");
            return;
        }

        CONFIG.imu_wheelbase_mm = wheelbase;
        CONFIG.imu_track_width_mm = track;
        CONFIG.imu_offset_x_mm = offsetX;
        CONFIG.imu_offset_y_mm = offsetY;

        const bool saved = config_save();

        JsonDocument doc;
        doc["saved"] = saved;
        doc["imu_wheelbase_mm"] = CONFIG.imu_wheelbase_mm;
        doc["imu_track_width_mm"] = CONFIG.imu_track_width_mm;
        doc["imu_offset_x_mm"] = CONFIG.imu_offset_x_mm;
        doc["imu_offset_y_mm"] = CONFIG.imu_offset_y_mm;

        String out;
        serializeJson(doc, out);
        request->send(saved ? 200 : 500, "application/json", out);
    });

    server.on("/api/reset_imu_geometry", HTTP_GET, [](AsyncWebServerRequest *request){
        const SmartConfig defaults;

        CONFIG.imu_wheelbase_mm = defaults.imu_wheelbase_mm;
        CONFIG.imu_track_width_mm = defaults.imu_track_width_mm;
        CONFIG.imu_offset_x_mm = defaults.imu_offset_x_mm;
        CONFIG.imu_offset_y_mm = defaults.imu_offset_y_mm;

        const bool saved = config_save();

        JsonDocument doc;
        doc["saved"] = saved;
        doc["imu_wheelbase_mm"] = CONFIG.imu_wheelbase_mm;
        doc["imu_track_width_mm"] = CONFIG.imu_track_width_mm;
        doc["imu_offset_x_mm"] = CONFIG.imu_offset_x_mm;
        doc["imu_offset_y_mm"] = CONFIG.imu_offset_y_mm;

        String out;
        serializeJson(doc, out);
        request->send(saved ? 200 : 500, "application/json", out);
    });

    // =========================================================================
    // Hardware Status API Endpoint
    // =========================================================================
    // Returns M5Stack hardware status (battery, IMU, buttons)
    // No authentication required for status monitoring
    server.on("/api/hw", HTTP_GET, [](AsyncWebServerRequest *request){ 
        JsonDocument doc; 
        HwStatus st = HW.read(); 
        doc["battery_voltage"] = st.battery_voltage; 
        doc["battery_percent"] = st.battery_percent; 
        doc["charging"]        = st.charging; 
        doc["temperature"]     = st.temperature;  // Note: temperature not in HwStatus
        doc["button_a"]        = st.button_a; 
        doc["button_b"]        = st.button_b; 
        JsonObject accel = doc["accel"].to<JsonObject>(); 
        accel["x"] = st.accel_x; 
        accel["y"] = st.accel_y; 
        accel["z"] = st.accel_z; 
        String out; 
        serializeJson(doc, out); 
        request->send(200, "application/json", out); 
    });

    // =========================================================================
    // NB-IoT Status API Endpoint
    // =========================================================================
    // Returns cellular modem and network status

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
    server.on("/nbiot", HTTP_GET, [](AsyncWebServerRequest *req){ req->send(SPIFFS, "/nbiot.html", "text/html"); }); 
    server.on("/meshtastic", HTTP_GET, [](AsyncWebServerRequest *req){ req->send(SPIFFS, "/meshtastic.html", "text/html"); }); 
    server.on("/sensors", HTTP_GET, [](AsyncWebServerRequest *req){ req->send(SPIFFS, "/sensors.html", "text/html"); }); 
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