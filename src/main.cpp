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
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Features:
 *   - M5Stack hardware initialization (IMU, RTC, power management)
 *   - Dual WiFi mode (AP + STA) with captive portal fallback
 *   - MQTT broker integration for remote command handling
 *   - Multi-sensor support (distance, weight, tilt, RTC)
 *   - BLE communication for battery management systems
 *   - Meshtastic bridge for mesh networking
 *   - NB-IoT connectivity
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
#include "mqtt_layer.h"
#include "command_handler.h"
#include "web_dashboard.h"
#include "config_store.h"
#include "captive_portal.h"
#include "mqtt_bridge.h"
#include "meshtastic_bridge.h"

// Set to 1 to stop execution before DISTANCE task launch (debug only).
#ifndef STOP_BEFORE_DISTANCE_TASK
#define STOP_BEFORE_DISTANCE_TASK 0
#endif

// ============================================================================
// Task Handle Declarations
// ============================================================================
// FreeRTOS task handles for managing concurrent operations across both cores

TaskHandle_t taskWiFiHandle             = nullptr;  // WiFi connectivity management
TaskHandle_t taskMqttBrokerHandle       = nullptr;  // MQTT broker communication
TaskHandle_t taskDistanceHandle         = nullptr;  // Distance sensor reading
TaskHandle_t taskWeightHandle           = nullptr;  // Weight sensor reading
TaskHandle_t taskTiltHandle             = nullptr;  // Tilt sensor reading
TaskHandle_t taskRtcHandle              = nullptr;  // Real-time clock synchronization
TaskHandle_t taskBmsBleHandle           = nullptr;  // BLE battery management system
TaskHandle_t taskDisplayHandle          = nullptr;  // M5Stack display updates
TaskHandle_t taskMeshtasticBridgeHandle = nullptr;  // Meshtastic mesh bridge
TaskHandle_t taskNbiotHandle            = nullptr;  // NB-IoT cellular communication

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

    // Initialize serial communication at 115200 baud for debugging
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

    // --- WiFi Dual-Mode Setup ---
    // Initialize both AP (access point) and STA (station) modes
    // Allows device to work as standalone hotspot and connect to external network
    setupWiFiApSta(
        "SmartFranklin-AP",              // Access point SSID for direct connection
        "smartfranklin",                 // Access point password
        CONFIG.sta_ssid.c_str(),         // Station SSID from persistent config
        CONFIG.sta_pass.c_str()          // Station password from persistent config
    );

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
        std::string uri = std::string(CONFIG.ext_mqtt_host.c_str());
        if (uri.find("://") == std::string::npos) {
            uri = std::string("mqtt://") + uri;
        }

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

    // --- Web Dashboard and Bridge Initialization ---
    // Start web-based management interface and MQTT device bridge
    web_dashboard_init();
    mqtt_bridge_init();

    // =========================================================================
    // FreeRTOS Task Creation
    // =========================================================================
    // Tasks run concurrently across dual ESP32 cores (Core 0 and Core 1)
    // Stack sizes: 2048-8192 bytes (larger for BLE/mesh operations)
    // Priority levels: 1 (low) to 3 (high); higher = more CPU scheduling time
    
    xTaskCreatePinnedToCore(taskHwMonitor,        "HW_MON",   4096, nullptr, 1,  nullptr,                    0);
    xTaskCreatePinnedToCore(taskMqttBroker,       "MQTT_BRK", 4096, nullptr, 3,  &taskMqttBrokerHandle,      1);

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
    xTaskCreatePinnedToCore(taskBmsBle,           "BMS_BLE",  8192, nullptr, 2,  &taskBmsBleHandle,          0);
    xTaskCreatePinnedToCore(taskDisplay,          "DISPLAY",  4096, nullptr, 1,  &taskDisplayHandle,         1);

    const bool has_negative_meshtastic_probe = gravity_path_detected
        && i2c_report.gravity_probe_ran
        && !i2c_report.c6l_activity_detected;
    if (CONFIG.meshtastic_bridge_enabled && !has_negative_meshtastic_probe) {
        xTaskCreatePinnedToCore(taskMeshtasticBridge, "MESH_BR", 8192, nullptr, 2, &taskMeshtasticBridgeHandle, 0);
    } else {
        M5_LOGW("[TASK] Skipping MESH_BR task: enumeration/probe does not confirm C6L path");
    }

    const bool has_negative_nbiot_probe = gravity_path_detected
        && i2c_report.gravity_probe_ran
        && !i2c_report.nb_iot2_confirmed;
    if (CONFIG.nbiot_enabled && !has_negative_nbiot_probe) {
        xTaskCreatePinnedToCore(taskNbiot, "NB_IOT", 8192, nullptr, 2, &taskNbiotHandle, 0);
    } else {
        M5_LOGW("[TASK] Skipping NB_IOT task: enumeration/probe does not confirm NB-IoT2 path");
    }

    xTaskCreatePinnedToCore(taskWatchdog,         "WATCHDOG", 2048, nullptr, 3,  nullptr,                    0);

    M5_LOGI("SmartFranklin setup complete.");
}

// ============================================================================
// loop() - Main Event Loop
// ============================================================================
/**
 * Main loop executed repeatedly by the Arduino framework.
 * Handles M5Stack button input and bridges MQTT loop for message processing.
 * Long-press detection on Button A allows config reset.
 * Pressing Button B performs an immediate reboot.
 */
void loop() {
    // Update M5Stack internal state (buttons, sensors, power management)
    M5.update();

    // =========================================================================
    // Button A Long-Press Handler (Configuration Reset)
    // =========================================================================
    // Detects 5+ second press on Button A to trigger factory reset
    static unsigned long pressStart = 0;
    
    if (M5.BtnA.isPressed()) {
        // Button press detected: record the start time if not already recorded
        if (pressStart == 0) pressStart = millis();
        
        // Check if button has been held for more than 5 seconds
        if (millis() - pressStart > 5000) {
            SPIFFS.remove("/config.json");  // Delete configuration file
            ESP.restart();                   // Restart device to load defaults
        }
    } else {
        // Button released: reset the press timer
        pressStart = 0;
    }

    // Reboot immediately on Button B press event
    if (M5.BtnB.wasPressed()) {
        M5_LOGI("----- SmartFranklin restarted -----");
        ESP.restart();
    }

    // Process MQTT bridge events and message delivery
    mqtt_bridge_loop();
    
    // Yield to other tasks (50ms cycle time for main loop)
    delay(50);
}
