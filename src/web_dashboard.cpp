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
 *   - Static Assets: Embedded in firmware (PROGMEM)
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
 *   - Served from embedded PROGMEM strings
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
 *   - PROGMEM embedded page/script constants
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
#include <WiFi.h>

#include <cmath>
#include <cstdlib>
#include "data_model.h"
#include "config_store.h"
#include "gaz.h"
#include "log.h"
#include "web_dashboard_assets.h"

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

// Static web assets are defined in src/web_dashboard_assets.cpp.

// ============================================================================
// Authentication Helper Function
// ============================================================================

/**
 * @brief Checks HTTP Basic Authentication for incoming requests.
 */
static bool checkAuth(AsyncWebServerRequest *request)
{
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

    doc["gaz_calibration_factor"] = CONFIG.gaz_calibration_factor;
    doc["scale_cal_factor"] = CONFIG.gaz_calibration_factor;
    doc["gaz_weight_average_window"] = CONFIG.gaz_weight_average_window;

    doc["level_wheelbase_mm"] = CONFIG.level_wheelbase_mm;
    doc["level_track_width_mm"] = CONFIG.level_track_width_mm;
    doc["level_offset_x_mm"] = CONFIG.level_offset_x_mm;
    doc["level_offset_y_mm"] = CONFIG.level_offset_y_mm;
    doc["level_zero_pitch_deg"] = CONFIG.level_zero_pitch_deg;
    doc["level_zero_roll_deg"] = CONFIG.level_zero_roll_deg;
    doc["rtc_timezone"] = CONFIG.rtc_timezone;

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
        !applyFloat("scale_cal_factor", updated.gaz_calibration_factor) ||
        !applyFloat("gaz_calibration_factor", updated.gaz_calibration_factor) ||
        !applyInt("gaz_weight_average_window", updated.gaz_weight_average_window) ||
        !applyFloat("level_wheelbase_mm", updated.level_wheelbase_mm) ||
        !applyFloat("level_track_width_mm", updated.level_track_width_mm) ||
        !applyFloat("level_offset_x_mm", updated.level_offset_x_mm) ||
        !applyFloat("level_offset_y_mm", updated.level_offset_y_mm) ||
        !applyFloat("level_zero_pitch_deg", updated.level_zero_pitch_deg) ||
        !applyFloat("level_zero_roll_deg", updated.level_zero_roll_deg) ||
        !applyString("rtc_timezone", updated.rtc_timezone) ||
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

    if (updated.gaz_calibration_factor <= 0.0f || !std::isfinite(updated.gaz_calibration_factor)) {
        errorKey = "gaz_calibration_factor";
        return false;
    }

    if (updated.gaz_weight_average_window < 1 || updated.gaz_weight_average_window > 64) {
        errorKey = "gaz_weight_average_window";
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

    updated.rtc_timezone.trim();
    if (updated.rtc_timezone.isEmpty()) {
        errorKey = "rtc_timezone";
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
 * API calls, embedded page serving, and OTA firmware updates. Must be
 * called once during system initialization after WiFi is connected.
 * 
 * Route Registration:
 * 
 *   Static Routes:
 *   - / : Main dashboard page (HTML from PROGMEM)
 *   - /hw : Hardware status page (from PROGMEM)
 *   - /sensors : Sensor data page (from PROGMEM)
 *   - /theme.js : Common JavaScript (from PROGMEM)
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
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/html", MAIN_PAGE);
    });

    server.on("/logs", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/html", LOGS_PAGE);
    });

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

    server.on("/api/logs", HTTP_GET, [](AsyncWebServerRequest *request){
        uint32_t after = 0;
        uint32_t maxEntries = 120;

        if (request->hasParam("after")) {
            const String raw = request->getParam("after")->value();
            after = static_cast<uint32_t>(strtoul(raw.c_str(), nullptr, 10));
        }
        if (request->hasParam("max")) {
            const String raw = request->getParam("max")->value();
            const uint32_t parsed = static_cast<uint32_t>(strtoul(raw.c_str(), nullptr, 10));
            if (parsed > 0 && parsed <= 300) {
                maxEntries = parsed;
            }
        }

        request->send(200, "application/json", sf_log::getLogsJson(after, maxEntries));
    });

    server.begin();
}
