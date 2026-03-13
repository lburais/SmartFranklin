/*
 * ============================================================================
 * SmartFranklin - Main Application Entry Point
 * ============================================================================
 * 
 * Project:     SmartFranklin IoT Device Controller
 * Description: Main setup and loop for M5Stack-based IoT hub with WiFi, MQTT,
 *              BLE, NB-IoT, and Meshtastic bridge capabilities.
 * 
 * Author:      Laurent Burais
 * Date:        10 March 2026
 * Version:     1.1
 * 
 * Features:
 *   - M5Stack hardware initialization (IMU, RTC, power management)
 *   - Dual WiFi mode (AP + STA) with captive portal fallback
 *   - MQTT broker integration for remote command handling
 *   - Multi-sensor support (distance, weight, tilt, RTC)
 *   - BLE communication for battery management systems
*   - Meshtastic bridge for mesh networking
 *   - Web dashboard for device management
 *   - Configuration persistence via SPIFFS
 * 
 * Hardware:    M5Stack with integrated IMU and RTC
 * Platform:    ESP32 (FreeRTOS)
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

#include <Arduino.h>
#include <M5Unified.h>
#include <SPIFFS.h>
#include <WiFi.h>

#include "tasks.h"
#include "wifi_setup.h"
#include "mqtt.h"
#include "command_handler.h"
#include "web_dashboard.h"
#include "config_store.h"
#include "captive_portal.h"
#include "meshtastic_bridge.h"

// ============================================================================
// Task Handle Declarations
// ============================================================================
// FreeRTOS task handles for managing concurrent operations across both cores

TaskHandle_t taskWiFiHandle             = nullptr;  // WiFi connectivity management
TaskHandle_t taskMqttHandle             = nullptr;  // MQTT client+broker communication
TaskHandle_t taskDistanceHandle         = nullptr;  // Distance sensor reading
TaskHandle_t taskWeightHandle           = nullptr;  // Weight sensor reading
TaskHandle_t taskTiltHandle             = nullptr;  // Tilt sensor reading
TaskHandle_t taskRtcHandle              = nullptr;  // Real-time clock synchronization
TaskHandle_t taskGpsHandle              = nullptr;  // Gravity DFR1103 GPS/RTC task
TaskHandle_t taskBmsBleHandle           = nullptr;  // BLE battery management system
TaskHandle_t taskHmiHandle              = nullptr;  // HMI/display task
TaskHandle_t taskMeshtasticBridgeHandle = nullptr;  // Meshtastic mesh bridge

static constexpr uint8_t DISPLAY_UI_ROTATION = 3;
static constexpr uint8_t DISPLAY_UI_BRIGHTNESS = 255;

namespace {

/**
 * @brief Builds a valid MQTT URI from host/URI and configured port.
 *
 * Rules:
 * - If host already includes scheme (e.g. mqtt://...), keep as-is.
 * - If no scheme and no explicit port, append `ext_mqtt_port`.
 * - If no scheme and port already present in host, preserve host:port.
 */
std::string buildMqttUri(const String& hostOrUri, int port)
{
    std::string uri = std::string(hostOrUri.c_str());
    if (uri.empty()) {
        return uri;
    }

    if (uri.find("://") != std::string::npos) {
        return uri;
    }

    const bool hasExplicitPort = uri.find(':') != std::string::npos;
    if (!hasExplicitPort) {
        const int effectivePort = (port > 0) ? port : 1883;
        uri += ":" + std::to_string(effectivePort);
    }

    return std::string("mqtt://") + uri;
}

} // namespace

// ============================================================================
// setup() - System Initialization
// ============================================================================
/**
 * Initializes all hardware, peripherals, and creates FreeRTOS tasks.
 * Called once at device startup. Configures M5Stack, loads persistent config,
 * establishes WiFi connectivity, initializes MQTT, and launches all service tasks.
 */
void setup() {

    // --- M5 Hardware Initialization ---
    // Configure M5Stack with power delivery, IMU, and RTC enabled
    auto cfg = M5.config();
    cfg.output_power = true;   // Enable 5V output power for peripheral devices
    cfg.internal_imu = true;   // Enable internal 6-axis IMU (accelerometer + gyroscope)
    cfg.internal_rtc = true;   // Enable internal real-time clock for timekeeping
    M5.begin(cfg);

    // Initialize serial communication at 115200 baud
    Serial.begin(115200);

    // Initialize SPIFFS filesystem for configuration file persistence
    SPIFFS.begin(true);

    // --- Configuration Loading ---
    // Load saved configuration from SPIFFS, or use defaults if missing
    config_load();

    // Enumerate I2C units (direct, via PAHUB, and bridge candidates)
    const uint16_t i2c_entries = HW.enumerateI2CUnits();
    M5_LOGI("[I2C] startup enumeration discovered %u entries", i2c_entries);
    const I2CEnumerationReport& i2c_report = HW.getLastI2CEnumerationReport();
    const bool gravity_path_detected = i2c_report.gravity_on_wire
        || i2c_report.gravity_on_wire_pahub
        || i2c_report.gravity_on_ex
        || i2c_report.gravity_on_ex_pahub;

    xTaskCreatePinnedToCore(taskWiFi,            "WIFI",      2048, nullptr, 3,  &taskWiFiHandle,            1);

    // --- WiFi Dual-Mode Setup ---
    // Initialize both AP (access point) and STA (station) modes
    // Allows device to work as standalone hotspot and connect to external network
    // ...existing code...

    // If station connection fails, start captive portal for WiFi setup
    if (WiFi.status() != WL_CONNECTED) {
        captive_portal_start();
    }

    // --- Command Handler Initialization ---
    // Initialize the command processing system for handling remote commands
    command_handler_init();

    // --- MQTT Layer Setup ---
    // Configure and initialize ESP-MQTT client for external broker communication
    if (CONFIG.ext_mqtt_enabled && !CONFIG.ext_mqtt_host.isEmpty()) {
        std::string uri = buildMqttUri(CONFIG.ext_mqtt_host, CONFIG.ext_mqtt_port);

        sf_mqtt::Config mcfg;
        mcfg.uri       = uri;                                    // MQTT broker URI
        mcfg.username  = std::string(CONFIG.ext_mqtt_user.c_str());
        mcfg.password  = std::string(CONFIG.ext_mqtt_pass.c_str());
        mcfg.client_id = std::string("SmartFranklin");

        const bool mqtt_ok = sf_mqtt::init(mcfg, [](const std::string &topic, const std::string &payload){
            meshtastic_bridge_handle_mqtt(String(topic.c_str()), String(payload.c_str()));
            command_handle(String(topic.c_str()), String(payload.c_str()));
        });

        if (!mqtt_ok) {
            M5_LOGW("[MQTT] External MQTT init failed for URI: %s", uri.c_str());
        }
    } else if (CONFIG.ext_mqtt_enabled && CONFIG.ext_mqtt_host.isEmpty()) {
        M5_LOGW("[MQTT] External MQTT enabled but host is empty; skipping init");
    } else {
        M5_LOGI("[MQTT] External MQTT disabled in config; skipping init");
    }

    // --- Web Dashboard Initialization ---
    // Start web-based management interface
    web_dashboard_init();

    // =========================================================================
    // FreeRTOS Task Creation
    // =========================================================================
    // Tasks run concurrently across dual ESP32 cores (Core 0 and Core 1)
    // Stack sizes: 2048-8192 bytes (larger for BLE/mesh operations)
    // Priority levels: 1 (low) to 3 (high); higher = more CPU scheduling time
    
    xTaskCreatePinnedToCore(taskHmi,              "HMI",      8192, nullptr, 3,  &taskHmiHandle,            1);

    // Start MQTT processing as early as possible after HMI so publishers
    // can use the MQTT client path sooner during startup.
    xTaskCreatePinnedToCore(taskMqtt,             "MQTT",     4096, nullptr, 3,  &taskMqttHandle,            1);

    xTaskCreatePinnedToCore(taskWatchdog,         "WATCHDOG", 2048, nullptr, 3,  nullptr,                    0);

    xTaskCreatePinnedToCore(taskHwMonitor,        "HW_MON",   4096, nullptr, 1,  nullptr,                    0);

    if (i2c_report.distance_on_wire
        || i2c_report.distance_on_wire_pahub
        || i2c_report.distance_on_ex
        || i2c_report.distance_on_ex_pahub) {
        xTaskCreatePinnedToCore(taskDistance, "DISTANCE", 4096, nullptr, 2, &taskDistanceHandle, 1);
    } else {
        M5_LOGW("[TASK] Skipping DISTANCE task: no compatible distance path found");
    }

    if (i2c_report.weight_on_wire || i2c_report.weight_on_wire_pahub) {
        xTaskCreatePinnedToCore(taskWeight, "WEIGHT", 4096, nullptr, 2, &taskWeightHandle, 1);
    } else {
        M5_LOGW("[TASK] Skipping WEIGHT task: no Wire-compatible weight path found");
    }

    xTaskCreatePinnedToCore(taskTilt,             "TILT",     4096, nullptr, 2,  &taskTiltHandle,            1);
    
    xTaskCreatePinnedToCore(taskRtc,              "RTC",      4096, nullptr, 2,  &taskRtcHandle,             1);

    xTaskCreatePinnedToCore(taskGps,              "GPS",      6144, nullptr, 2,  &taskGpsHandle,            1);

    xTaskCreatePinnedToCore(taskBmsBle,           "BMS_BLE",  8192, nullptr, 2,  &taskBmsBleHandle,          0);
    
    const bool has_negative_meshtastic_probe = gravity_path_detected
        && i2c_report.gravity_probe_ran
        && !i2c_report.c6l_activity_detected;
    if (CONFIG.meshtastic_bridge_enabled && !has_negative_meshtastic_probe) {
        xTaskCreatePinnedToCore(taskMeshtasticBridge, "MESH_BR", 8192, nullptr, 2, &taskMeshtasticBridgeHandle, 0);
    } else {
        M5_LOGW("[TASK] Skipping MESH_BR task: enumeration/probe does not confirm C6L path");
    }

    // NB-IoT2 task creation fully removed

    M5_LOGI("SmartFranklin setup complete.");
}

// ============================================================================
// loop() - Main Event Loop
// ============================================================================
/**
 * Main loop executed repeatedly by the Arduino framework.
 * Runtime work is handled by FreeRTOS tasks.
 */
void loop() {
    delay(100);
}
