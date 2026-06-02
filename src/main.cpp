/*
 * ============================================================================
 * SmartFranklin - Main Application Entry Point
 * ============================================================================
 * 
 * Project:     SmartFranklin IoT Device Controller
 * Description: Main setup and loop for the M5Stack-based SmartFranklin hub
 *              with WiFi, MQTT, BLE, local HMI, and unified I2C sensor tasks.
 * 
 * Author:      Laurent Burais
 * Date:        10 March 2026
 * Version:     1.1
 * 
 * Features:
 *   - M5Stack hardware initialization (power management)
 *   - Dual WiFi mode (AP + STA) with captive portal fallback
 *   - MQTT broker integration for remote command handling
 *   - Multi-sensor support (gaz, tank, level, rtc, gps)
 *   - BLE communication for battery management systems
 *   - Web dashboard for device management
 *   - Configuration persistence via SPIFFS
 * 
 * Hardware:    M5Stack device family
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
#include "mqtt.h"
#include "command_handler.h"
#include "web_dashboard.h"
#include "config_store.h"
#include "interfaces.h"
#include "log.h"
#include "network.h"

// ============================================================================
// Task Handle Declarations
// ============================================================================
// FreeRTOS task handles for managing concurrent operations across both cores

TaskHandle_t taskWiFiHandle             = nullptr;  // WiFi connectivity management
TaskHandle_t taskMqttHandle             = nullptr;  // MQTT client+broker communication
TaskHandle_t taskI2cHandle              = nullptr;  // Unified I2C sensors task
TaskHandle_t taskBmsBleHandle           = nullptr;  // BLE battery management system
TaskHandle_t taskHmiHandle              = nullptr;  // HMI/display task

// static constexpr uint8_t DISPLAY_UI_ROTATION = 3;
// static constexpr uint8_t DISPLAY_UI_BRIGHTNESS = 255;

namespace {

bool waitForLocalMqttBrokerReady(uint32_t timeoutMs)
{
    const uint32_t startMs = millis();
    while ((millis() - startMs) < timeoutMs) {
        if (sf_mqtt::is_local_broker_ready()) {
            const bool probeOk = sf_mqtt::publish_local("smartfranklin/system/mqtt_broker/health", "boot_probe", 0, false);
            if (probeOk) {
                SF_LOGI("[MQTT] Local broker health check PASS");
                return true;
            }

            SF_LOGW("[MQTT] Local broker ready but health publish probe failed");
            return false;
        }

        delay(100);
    }

    SF_LOGW("[MQTT] Local broker health check FAIL (startup timeout)");
    return false;
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

    // =========================================================================
    // M5 Hardware Initialization
    // =========================================================================

    // Configure M5Stack with board-appropriate internal peripherals
    auto cfg = M5.config();
    cfg.output_power = true;   // Enable 5V output power for peripheral devices
    cfg.internal_imu = true;
    cfg.internal_rtc = true;
    cfg.fallback_board = m5::board_t::board_M5Station;

    M5.begin(cfg);

    // Initialize serial communication at 115200 baud
    Serial.begin(115200);

    // Initialize SPIFFS filesystem for configuration file persistence
    SPIFFS.begin(true);

    // --- Configuration Loading ---
    // Load saved configuration from SPIFFS, or use defaults if missing
    config_load();

    // =========================================================================
    // WiFi AP+STA FreeRTOS Task creation
    // =========================================================================

    xTaskCreatePinnedToCore(taskWiFi, "WIFI", 8192, nullptr, 3, &taskWiFiHandle, 0);

    // Wait for access point to be ready.
    while (!WIFI_TASK.isInitialized()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // =========================================================================
    // MQTT broker FreeRTOS Task creation
    // =========================================================================

    xTaskCreatePinnedToCore(taskMqtt, "MQTT", 4096, nullptr, 3,  &taskMqttHandle, 0);

    // Validate local MQTT broker startup and local publish path.
    (void)waitForLocalMqttBrokerReady(5000);

    // =========================================================================
    // Command Handler Initialization
    // =========================================================================

    // Initialize the command processing system for handling remote commands
    command_handler_init();

    // =========================================================================
    // Web Dashboard Initialization
    // =========================================================================

    // Start web-based management interface
    web_dashboard_init();

    // =========================================================================
    // Interfaces Initialization
    // =========================================================================

    // Configure all interfaces
    // sf_interfaces::configure_all_sensors();

    // =========================================================================
    // FreeRTOS Task Creation
    // =========================================================================
    // Tasks run concurrently across dual ESP32 cores (Core 0 and Core 1)
    // Stack sizes: 2048-8192 bytes (larger for BLE/mesh operations)
    // Priority levels: 1 (low) to 3 (high); higher = more CPU scheduling time
    
    #ifdef ENABLE_HMI
    xTaskCreatePinnedToCore(taskHmi,              "HMI",      8192, nullptr, 3,  &taskHmiHandle,      0);
    #endif

    #if defined(ENABLE_GAZ) || defined(ENABLE_TANK) || defined(ENABLE_LEVEL) || defined(ENABLE_RTC) || defined(ENABLE_GPS) || defined(ENABLE_AXP) || defined(ENABLE_INA)
    xTaskCreatePinnedToCore(taskI2c,              "I2C",      8192, nullptr, 2, &taskI2cHandle,       1);
    #endif

    #ifdef ENABLE_BATTERY
    xTaskCreatePinnedToCore(taskBmsBle,           "BMS_BLE",  8192, nullptr, 2, &taskBmsBleHandle,   0);
    #endif

    #ifdef ENABLE_HW_MONITOR
    xTaskCreatePinnedToCore(taskHwMonitor,        "HW_MON",   4096, nullptr, 1, nullptr,              0);
    #endif

    #ifdef ENBLE_WATCHDOG
    xTaskCreatePinnedToCore(taskWatchdog,         "WATCHDOG", 2048, nullptr, 3, nullptr,              0);
    #endif

    SF_LOGI("[SF] SmartFranklin setup complete.");
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
