/*
 * ============================================================================
 * Configuration Storage Module - SmartFranklin
 * ============================================================================
 * 
 * File:        config_store.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: Persistent configuration management using SPIFFS filesystem.
 *              Loads, saves, and provides default configuration for all
 *              device subsystems (WiFi, MQTT, NB-IoT, authentication).
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   SmartFranklin requires numerous configuration parameters to operate:
 *   WiFi credentials, MQTT broker details, NB-IoT settings, authentication
 *   credentials, and hardware calibration factors. This module manages the
 *   complete lifecycle of configuration data.
 * 
 * Storage System:
 *   - File System: SPIFFS (SPI Flash File System) on ESP32
 *   - Storage Format: JSON (human-readable, editable)
 *   - File Path: /config.json (root of SPIFFS partition)
 *   - Persistence: Data survives device power cycles
 *   - Size Limit: Depends on ESP32 flash allocation (typically 1-4MB)
 * 
 * Configuration Categories:
 * 
 *   1. WiFi Settings
 *      - Station SSID/Password: External network connection
 *      - Access Point: Always enabled for local connectivity
 * 
 *   2. Authentication
 *      - Admin Username/Password: Web dashboard access control
 * 
 *   3. Local MQTT Broker
 *      - Port: Local broker listen port
 * 
 *   4. NB-IoT Cellular
 *      - APN: Carrier network access point name
 *      - MQTT Details: Cellular backup connectivity
 * 
 *   6. Hardware Calibration
 *      - Scale Calibration Factor: Weight sensor accuracy adjustment
 * 
 * Default Configuration:
 *   If no config.json exists, sensible defaults are applied:
 *   - WiFi: Empty credentials (user must configure)
 *   - MQTT: Local broker enabled on default port
 *   - NB-IoT: Enabled with 1NCE carrier APN
 *   - Admin: Default credentials (admin/admin - change on first boot!)
 * 
 * Dependencies:
 *   - ArduinoJson (JSON serialization/deserialization)
 *   - SPIFFS.h (ESP32 filesystem support)
 *   - config_store.h (header declarations)
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

#include "config_store.h"
#include <ArduinoJson.h>
#include <FS.h>
#include <M5Unified.h>
#include <SPIFFS.h>

// ============================================================================
// Global Configuration Object
// ============================================================================
// Global SmartConfig instance exposed to entire application
// All modules read configuration from this object after config_load() completes
SmartConfig CONFIG;

// ============================================================================
// Constants
// ============================================================================
// File system path for configuration JSON file
// Located at SPIFFS root directory for easy access
static const char *CFG_PATH = "/config.json";

// ============================================================================
// Configuration Loading Function
// ============================================================================

/**
 * @brief Loads configuration from SPIFFS or initializes with defaults.
 * 
 * Attempts to read configuration from SPIFFS filesystem. If the configuration
 * file does not exist, initializes CONFIG with sensible default values. This
 * ensures the device always has valid configuration even on first boot.
 * 
 * Loading Strategy:
 *   1. Initialize SPIFFS filesystem
 *   2. Check if /config.json exists
 *   3. If missing: Apply default configuration
 *   4. If exists: Parse JSON and populate CONFIG object
 *   5. Apply fallback defaults for any missing JSON fields
 * 
 * Fallback Defaults:
 *   The deserializer uses the pipe operator (|) to provide defaults:
 *     config_value = doc["field"] | default_value
 *   
 *   If JSON field is missing or invalid type, default is used.
 *   This ensures robustness against partial or corrupted config files.
 * 
 * Default Values Applied:
 * 
 *   WiFi & Admin:
 *   - sta_ssid: "" (user must configure)
 *   - sta_pass: "" (user must configure)
 *   - admin_user: "admin" (⚠️  CHANGE ON FIRST BOOT!)
 *   - admin_pass: "admin" (⚠️  CHANGE ON FIRST BOOT!)
 *   - gaz_calibration_factor: 1.0 (uncalibrated)
 * 
 *   Local MQTT:
 *   - mqtt_port: 1883 (standard MQTT port)
 * 
 * Return Value:
 *   - true: Configuration successfully loaded or defaults created
 *   - false: SPIFFS initialization failed or JSON parsing error
 * 
 * @return bool - true on success, false on SPIFFS/JSON error
 * 
 * @note This function must be called during setup() before any module
 *       attempts to access the global CONFIG object.
 *       Example: config_load(); // Called from main setup()
 * 
 * @see CONFIG - Global configuration object populated by this function
 * @see config_save() - Saves modified configuration back to SPIFFS
 */
bool config_load()
{
    SmartConfig defaultCONFIG;

    // Initialize SPIFFS filesystem with auto-formatting on error
    if (!SPIFFS.begin(true)) return false;

    // =========================================================================
    // Default Configuration (when configuration file does not exist)
    // =========================================================================
    // Applied when /config.json is missing (e.g., fresh device or factory reset)
    if (!SPIFFS.exists(CFG_PATH)) {
        
        CONFIG = defaultCONFIG;
        return true;
    }

    // =========================================================================
    // Load from Configuration File
    // =========================================================================
    // File exists: Parse JSON and populate CONFIG object
    
    // Open configuration file for reading
    File f = SPIFFS.open(CFG_PATH, "r");
    if (!f) return false;

    // Deserialize JSON from file into document
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, f);
    f.close();
    
    // Check for JSON parsing errors (malformed file)
    if (err) return false;

    // =========================================================================
    // WiFi & Admin Credentials
    // =========================================================================
    // Load WiFi connection parameters and web dashboard authentication
    
    CONFIG.hostname = doc["hostname"] | defaultCONFIG.hostname;
    CONFIG.ap_ssid = doc["ap_ssid"] | defaultCONFIG.ap_ssid;
    CONFIG.ap_pass = doc["ap_pass"] | defaultCONFIG.ap_pass;
    CONFIG.sta_ssid = doc["sta_ssid"] | defaultCONFIG.sta_ssid;
    CONFIG.sta_pass = doc["sta_pass"] | defaultCONFIG.sta_pass;

    CONFIG.gaz_calibration_factor = doc["gaz_calibration_factor"] |
                                    doc["scale_cal_factor"] |
                                    defaultCONFIG.gaz_calibration_factor;
    CONFIG.gaz_weight_average_window = doc["gaz_weight_average_window"] | defaultCONFIG.gaz_weight_average_window;
    CONFIG.gaz_i2c_port = doc["gaz_i2c_port"] | defaultCONFIG.gaz_i2c_port;
    CONFIG.tank_i2c_port = doc["tank_i2c_port"] | defaultCONFIG.tank_i2c_port;
    CONFIG.level_i2c_port = doc["level_i2c_port"] | defaultCONFIG.level_i2c_port;
    CONFIG.rtc_i2c_port = doc["rtc_i2c_port"] | defaultCONFIG.rtc_i2c_port;
    CONFIG.gps_i2c_port = doc["gps_i2c_port"] | defaultCONFIG.gps_i2c_port;
    
    CONFIG.level_wheelbase_mm = doc["level_wheelbase_mm"] | defaultCONFIG.level_wheelbase_mm;
    CONFIG.level_track_width_mm = doc["level_track_width_mm"] | defaultCONFIG.level_track_width_mm;
    CONFIG.level_offset_x_mm = doc["level_offset_x_mm"] | defaultCONFIG.level_offset_x_mm;
    CONFIG.level_offset_y_mm = doc["level_offset_y_mm"] | defaultCONFIG.level_offset_y_mm;
    CONFIG.level_zero_pitch_deg = doc["level_zero_pitch_deg"] | defaultCONFIG.level_zero_pitch_deg;
    CONFIG.level_zero_roll_deg = doc["level_zero_roll_deg"] | defaultCONFIG.level_zero_roll_deg;

    CONFIG.admin_user = doc["admin_user"] | defaultCONFIG.admin_user;
    CONFIG.admin_pass = doc["admin_pass"] | defaultCONFIG.admin_pass;

    // const bool hasMqttPort = !doc["mqtt_port"].isNull();
    // const bool hasLegacyExtMqttPort = !doc["ext_mqtt_port"].isNull();

    // CONFIG.mqtt_port = doc["mqtt_port"] | defaultCONFIG.mqtt_port;

    CONFIG.task_wifi_loop_ms = doc["task_wifi_loop_ms"] | defaultCONFIG.task_wifi_loop_ms;
    CONFIG.task_mqtt_loop_ms = doc["task_mqtt_loop_ms"] | defaultCONFIG.task_mqtt_loop_ms;
    CONFIG.task_i2c_loop_ms = doc["task_i2c_loop_ms"] | defaultCONFIG.task_i2c_loop_ms;
    CONFIG.task_hmi_loop_ms = doc["task_hmi_loop_ms"] | defaultCONFIG.task_hmi_loop_ms;
    CONFIG.task_hmi_init_retry_ms = doc["task_hmi_init_retry_ms"] | defaultCONFIG.task_hmi_init_retry_ms;
    CONFIG.task_hw_monitor_loop_ms = doc["task_hw_monitor_loop_ms"] | defaultCONFIG.task_hw_monitor_loop_ms;
    CONFIG.task_bms_ble_connected_loop_ms = doc["task_bms_ble_connected_loop_ms"] | defaultCONFIG.task_bms_ble_connected_loop_ms;
    CONFIG.task_bms_ble_retry_loop_ms = doc["task_bms_ble_retry_loop_ms"] | defaultCONFIG.task_bms_ble_retry_loop_ms;
    CONFIG.task_watchdog_loop_ms = doc["task_watchdog_loop_ms"] | defaultCONFIG.task_watchdog_loop_ms;

    // if (!hasMqttPort && hasLegacyExtMqttPort) {
    //     M5_LOGW("[CONFIG] Deprecated key 'ext_mqtt_port' detected and ignored; using mqtt_port=%d", CONFIG.mqtt_port);
    //     if (config_save()) {
    //         M5_LOGI("[CONFIG] config.json migrated to mqtt_port-only format");
    //     } else {
    //         M5_LOGW("[CONFIG] failed to persist mqtt_port-only migration");
    //     }
    // }

    return true;
}

// ============================================================================
// Configuration Saving Function
// ============================================================================

/**
 * @brief Saves current configuration to SPIFFS as JSON file.
 * 
 * Serializes the global CONFIG object to JSON format and writes to SPIFFS
 * filesystem. Call this function whenever configuration changes to persist
 * them across device power cycles.
 * 
 * Serialization Process:
 *   1. Create new JsonDocument
 *   2. Populate document with all CONFIG field values
 *   3. Open /config.json for writing (creates or overwrites)
 *   4. Serialize JSON document to file
 *   5. Close file and return success status
 * 
 * Persisted Fields:
 *   All CONFIG object members are serialized to JSON:
 *   - WiFi credentials (sta_ssid, sta_pass)
 *   - Authentication (admin_user, admin_pass)
 *   - Hardware calibration (gaz_calibration_factor)
 *   - MQTT broker settings (local and NB-IoT)
 * 
 * Return Value:
 *   - true: File written successfully
 *   - false: File open failed or write error
 * 
 * @return bool - true on successful save, false on file error
 * 
 * @note Call this after modifying any CONFIG field to persist changes.
 *       Changes are only saved to CONFIG object without calling config_save()
 *       will not survive device restart.
 * 
 *       Example:
 *       CONFIG.sta_ssid = "MyNetwork";
 *       CONFIG.sta_pass = "MyPassword";
 *       config_save();  // Persist changes
 * 
 * @see CONFIG - Global configuration object being saved
 * @see config_load() - Loads saved configuration at startup
 */
bool config_save()
{
    // Create JSON document for serialization
    JsonDocument doc;

    // =========================================================================
    // WiFi & Admin Credentials
    // =========================================================================
    // Serialize WiFi connection parameters and authentication
    
    doc["hostname"] = CONFIG.hostname;                      // Device hostname
    doc["ap_ssid"] = CONFIG.ap_ssid;                        // Local AP SSID
    doc["ap_pass"] = CONFIG.ap_pass;                        // Local AP password
    doc["sta_ssid"] = CONFIG.sta_ssid;                      // External network SSID
    doc["sta_pass"] = CONFIG.sta_pass;                      // External network password
    doc["gaz_calibration_factor"] = CONFIG.gaz_calibration_factor;      // Weight sensor calibration
    doc["gaz_weight_average_window"] = CONFIG.gaz_weight_average_window; // Gaz smoothing window
    doc["gaz_i2c_port"] = CONFIG.gaz_i2c_port;                          // Gaz I2C port selection (e.g. A1)
    doc["tank_i2c_port"] = CONFIG.tank_i2c_port;
    doc["level_i2c_port"] = CONFIG.level_i2c_port;
    doc["rtc_i2c_port"] = CONFIG.rtc_i2c_port;
    doc["gps_i2c_port"] = CONFIG.gps_i2c_port;
    doc["level_wheelbase_mm"] = CONFIG.level_wheelbase_mm;
    doc["level_track_width_mm"] = CONFIG.level_track_width_mm;
    doc["level_offset_x_mm"] = CONFIG.level_offset_x_mm;
    doc["level_offset_y_mm"] = CONFIG.level_offset_y_mm;
    doc["level_zero_pitch_deg"] = CONFIG.level_zero_pitch_deg;
    doc["level_zero_roll_deg"] = CONFIG.level_zero_roll_deg;

    doc["admin_user"] = CONFIG.admin_user;                  // Web dashboard username
    doc["admin_pass"] = CONFIG.admin_pass;                  // Web dashboard password

    // =========================================================================
    // Local MQTT Broker Configuration
    // =========================================================================

    doc["mqtt_port"] = CONFIG.mqtt_port;                    // Broker port number

    // =========================================================================
    // Task Timing Configuration
    // =========================================================================

    doc["task_wifi_loop_ms"] = CONFIG.task_wifi_loop_ms;
    doc["task_mqtt_loop_ms"] = CONFIG.task_mqtt_loop_ms;
    doc["task_i2c_loop_ms"] = CONFIG.task_i2c_loop_ms;
    doc["task_hmi_loop_ms"] = CONFIG.task_hmi_loop_ms;
    doc["task_hmi_init_retry_ms"] = CONFIG.task_hmi_init_retry_ms;
    doc["task_hw_monitor_loop_ms"] = CONFIG.task_hw_monitor_loop_ms;
    doc["task_bms_ble_connected_loop_ms"] = CONFIG.task_bms_ble_connected_loop_ms;
    doc["task_bms_ble_retry_loop_ms"] = CONFIG.task_bms_ble_retry_loop_ms;
    doc["task_watchdog_loop_ms"] = CONFIG.task_watchdog_loop_ms;

    // Open configuration file for writing (creates or overwrites existing)
    File f = SPIFFS.open(CFG_PATH, "w");
    if (!f) return false;

    // Serialize JSON document to file
    serializeJson(doc, f);
    
    // Close file and release resources
    f.close();
    
    return true;
}
