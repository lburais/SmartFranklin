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
 *   3. External MQTT Broker
 *      - Host, Port, Credentials: Cloud connectivity
 *      - Enable/Disable: Toggle cloud integration
 * 
 *   4. MQTT Bridge
 *      - Topic Prefixes: Local/Cloud message separation
 *      - QoS Level: Reliability setting (0, 1, or 2)
 *      - Loop Detection: Prevent message routing loops
 * 
 *   5. NB-IoT Cellular
 *      - APN: Carrier network access point name
 *      - MQTT Details: Cellular backup connectivity
 * 
 *   6. Hardware Calibration
 *      - Scale Calibration Factor: Weight sensor accuracy adjustment
 * 
 * Default Configuration:
 *   If no config.json exists, sensible defaults are applied:
 *   - WiFi: Empty credentials (user must configure)
 *   - MQTT: Disabled until configured
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
#include "pahub_channels.h"
#include <ArduinoJson.h>
#include <FS.h>
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
 *   - scale_cal_factor: 1.0 (uncalibrated)
 * 
 *   External MQTT:
 *   - ext_mqtt_host: "" (disabled)
 *   - ext_mqtt_port: 1883 (standard MQTT port)
 *   - ext_mqtt_user: "" (no authentication)
 *   - ext_mqtt_pass: "" (no authentication)
 *   - ext_mqtt_enabled: false (disabled by default)
 * 
 *   MQTT Bridge (Local ↔ Cloud relay):
 *   - mqtt_bridge_enabled: true (enabled by default)
 *   - mqtt_bridge_prefix_internal: "local/" (local topic prefix)
 *   - mqtt_bridge_prefix_external: "cloud/" (cloud topic prefix)
 *   - mqtt_bridge_qos: 1 (at-least-once delivery)
 *   - mqtt_bridge_retain: false (don't retain messages)
 *   - mqtt_bridge_loop_detection: true (prevent message loops)
 * 
 *   NB-IoT Cellular:
 *   - nbiot_enabled: true (enabled by default)
 *   - nbiot_apn: "iot.1nce.net" (1NCE carrier APN)
 *   - nbiot_mqtt_host: "" (not configured)
 *   - nbiot_mqtt_port: 1883 (standard MQTT port)
 *   - nbiot_mqtt_user: "" (no authentication)
 *   - nbiot_mqtt_pass: "" (no authentication)
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
    // Initialize SPIFFS filesystem with auto-formatting on error
    if (!SPIFFS.begin(true)) return false;

    // =========================================================================
    // Default Configuration (when configuration file does not exist)
    // =========================================================================
    // Applied when /config.json is missing (e.g., fresh device or factory reset)
    if (!SPIFFS.exists(CFG_PATH)) {

        // --- WiFi & Hardware Calibration ---
        CONFIG.ap_ssid  = "SmartFranklin-AP"; // Default Access Point SSID
        CONFIG.ap_pass  = "smartfranklin";    // Default Access Point password
        CONFIG.sta_ssid = "";                 // No external network configured
        CONFIG.sta_pass = "";                 // No password set
        CONFIG.scale_cal_factor = 1.0f;    // Uncalibrated weight sensor

        // --- Web Dashboard Authentication ---
        CONFIG.admin_user = "admin";       // ⚠️  Default credentials - CHANGE!
        CONFIG.admin_pass = "admin";       // ⚠️  Default credentials - CHANGE!

        // --- External MQTT Broker (Cloud Connectivity) ---
        CONFIG.ext_mqtt_host = "";         // No broker configured yet
        CONFIG.ext_mqtt_port = 1883;       // Standard MQTT port
        CONFIG.ext_mqtt_user = "";         // No authentication
        CONFIG.ext_mqtt_pass = "";         // No authentication
        CONFIG.ext_mqtt_enabled = false;   // Cloud disabled until configured

        // --- MQTT Bridge (Local ↔ Cloud Message Relay) ---
        CONFIG.mqtt_bridge_enabled = true;              // Bridge enabled by default
        CONFIG.mqtt_bridge_prefix_internal = "local/";  // Local message prefix
        CONFIG.mqtt_bridge_prefix_external = "cloud/";  // Cloud message prefix
        CONFIG.mqtt_bridge_qos = 1;                      // At-least-once delivery
        CONFIG.mqtt_bridge_retain = false;               // Don't persist messages
        CONFIG.mqtt_bridge_loop_detection = true;        // Prevent routing loops

        // --- NB-IoT Cellular Connectivity ---
        CONFIG.nbiot_enabled   = true;                  // Cellular enabled by default
        CONFIG.nbiot_apn       = "iot.1nce.net";        // 1NCE carrier APN
        CONFIG.nbiot_mqtt_host = "";                    // No cellular MQTT configured
        CONFIG.nbiot_mqtt_port = 1883;                  // Standard MQTT port
        CONFIG.nbiot_mqtt_user = "";                    // No authentication
        CONFIG.nbiot_mqtt_pass = "";                    // No authentication

        // --- Meshtastic Bridge Configuration ---
        CONFIG.meshtastic_bridge_enabled = false;                // Disabled by default
        CONFIG.meshtastic_mqtt_prefix    = "smartfranklin/mesh/in/";
        CONFIG.meshtastic_baud           = 115200;               // Default UART baud
        CONFIG.meshtastic_pin_rx         = 33;                   // Default RX pin
        CONFIG.meshtastic_pin_tx         = 32;                   // Default TX pin

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
    
    CONFIG.ap_ssid = doc["ap_ssid"] | "SmartFranklin-AP";   // AP SSID (local network)
    CONFIG.ap_pass = doc["ap_pass"] | "smartfranklin";      // AP password
    CONFIG.sta_ssid = doc["sta_ssid"] | "";                 // Station SSID (external network)
    CONFIG.sta_pass = doc["sta_pass"] | "";                 // Station password
    CONFIG.scale_cal_factor = doc["scale_cal_factor"] | 1.0f;  // Weight sensor calibration

    CONFIG.admin_user = doc["admin_user"] | "admin";    // Web dashboard username
    CONFIG.admin_pass = doc["admin_pass"] | "admin";    // Web dashboard password

    // =========================================================================
    // External MQTT Broker Configuration
    // =========================================================================
    // Cloud connectivity settings for remote MQTT broker
    
    CONFIG.ext_mqtt_host = doc["ext_mqtt_host"] | "";           // Broker hostname/IP
    CONFIG.ext_mqtt_port = doc["ext_mqtt_port"] | 1883;          // Broker port (default 1883)
    CONFIG.ext_mqtt_user = doc["ext_mqtt_user"] | "";            // Broker username
    CONFIG.ext_mqtt_pass = doc["ext_mqtt_pass"] | "";            // Broker password
    CONFIG.ext_mqtt_enabled = doc["ext_mqtt_enabled"] | false;   // Enable/disable flag

    // =========================================================================
    // MQTT Bridge Settings (Local ↔ Cloud Message Relay)
    // =========================================================================
    // Configuration for relaying messages between local and cloud MQTT brokers
    
    CONFIG.mqtt_bridge_enabled = doc["mqtt_bridge_enabled"] | true;    // Bridge enable flag
    CONFIG.mqtt_bridge_prefix_internal = doc["mqtt_bridge_prefix_internal"] | "local/";   // Local topic prefix
    CONFIG.mqtt_bridge_prefix_external = doc["mqtt_bridge_prefix_external"] | "cloud/";   // Cloud topic prefix
    CONFIG.mqtt_bridge_qos = doc["mqtt_bridge_qos"] | 1;                // Quality of Service (0/1/2)
    CONFIG.mqtt_bridge_retain = doc["mqtt_bridge_retain"] | false;      // Message retention flag
    CONFIG.mqtt_bridge_loop_detection = doc["mqtt_bridge_loop_detection"] | true;  // Loop prevention

    // =========================================================================
    // NB-IoT Cellular Configuration
    // =========================================================================
    // Settings for 4G LTE-M/NB-IoT cellular backup connectivity
    
    CONFIG.nbiot_enabled   = doc["nbiot_enabled"]   | true;             // Cellular enable flag
    CONFIG.nbiot_apn       = doc["nbiot_apn"]       | "iot.1nce.net";   // Carrier APN (1NCE default)
    CONFIG.nbiot_mqtt_host = doc["nbiot_mqtt_host"] | "";               // Cellular MQTT broker
    CONFIG.nbiot_mqtt_port = doc["nbiot_mqtt_port"] | 1883;             // Cellular MQTT port
    CONFIG.nbiot_mqtt_user = doc["nbiot_mqtt_user"] | "";               // Cellular MQTT username
    CONFIG.nbiot_mqtt_pass = doc["nbiot_mqtt_pass"] | "";               // Cellular MQTT password

    // =========================================================================
    // Meshtastic Bridge Configuration
    // =========================================================================

    CONFIG.meshtastic_bridge_enabled = doc["meshtastic_bridge_enabled"] | false;
    CONFIG.meshtastic_mqtt_prefix    = doc["meshtastic_mqtt_prefix"] | "smartfranklin/mesh/in/";
    CONFIG.meshtastic_baud           = doc["meshtastic_baud"] | 115200;
    CONFIG.meshtastic_pin_rx         = doc["meshtastic_pin_rx"] | 33;
    CONFIG.meshtastic_pin_tx         = doc["meshtastic_pin_tx"] | 32;

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
 *   - Hardware calibration (scale_cal_factor)
 *   - MQTT broker settings (external and NB-IoT)
 *   - MQTT bridge configuration (prefixes, QoS, loop detection)
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
    
    doc["ap_ssid"] = CONFIG.ap_ssid;                        // Local AP SSID
    doc["ap_pass"] = CONFIG.ap_pass;                        // Local AP password
    doc["sta_ssid"] = CONFIG.sta_ssid;                      // External network SSID
    doc["sta_pass"] = CONFIG.sta_pass;                      // External network password
    doc["scale_cal_factor"] = CONFIG.scale_cal_factor;      // Weight sensor calibration

    doc["admin_user"] = CONFIG.admin_user;                  // Web dashboard username
    doc["admin_pass"] = CONFIG.admin_pass;                  // Web dashboard password

    // =========================================================================
    // External MQTT Broker Configuration
    // =========================================================================
    // Serialize cloud MQTT broker connection details
    
    doc["ext_mqtt_host"] = CONFIG.ext_mqtt_host;            // Broker hostname/IP
    doc["ext_mqtt_port"] = CONFIG.ext_mqtt_port;            // Broker port number
    doc["ext_mqtt_user"] = CONFIG.ext_mqtt_user;            // Broker username
    doc["ext_mqtt_pass"] = CONFIG.ext_mqtt_pass;            // Broker password
    doc["ext_mqtt_enabled"] = CONFIG.ext_mqtt_enabled;      // Enable/disable flag

    // =========================================================================
    // MQTT Bridge Settings
    // =========================================================================
    // Serialize local-to-cloud message relay configuration
    
    doc["mqtt_bridge_enabled"] = CONFIG.mqtt_bridge_enabled;            // Bridge enable flag
    doc["mqtt_bridge_prefix_internal"] = CONFIG.mqtt_bridge_prefix_internal;  // Local prefix
    doc["mqtt_bridge_prefix_external"] = CONFIG.mqtt_bridge_prefix_external;  // Cloud prefix
    doc["mqtt_bridge_qos"] = CONFIG.mqtt_bridge_qos;                    // Quality of Service
    doc["mqtt_bridge_retain"] = CONFIG.mqtt_bridge_retain;              // Message retention
    doc["mqtt_bridge_loop_detection"] = CONFIG.mqtt_bridge_loop_detection;    // Loop prevention

    // =========================================================================
    // NB-IoT Cellular Configuration
    // =========================================================================
    // Serialize 4G LTE-M/NB-IoT backup connectivity settings
    
    doc["nbiot_enabled"]   = CONFIG.nbiot_enabled;          // Cellular enable flag
    doc["nbiot_apn"]       = CONFIG.nbiot_apn;              // Carrier APN
    doc["nbiot_mqtt_host"] = CONFIG.nbiot_mqtt_host;        // Cellular MQTT broker
    doc["nbiot_mqtt_port"] = CONFIG.nbiot_mqtt_port;        // Cellular MQTT port
    doc["nbiot_mqtt_user"] = CONFIG.nbiot_mqtt_user;        // Cellular MQTT username
    doc["nbiot_mqtt_pass"] = CONFIG.nbiot_mqtt_pass;        // Cellular MQTT password

    // =========================================================================
    // Meshtastic Bridge Configuration
    // =========================================================================

    doc["meshtastic_bridge_enabled"] = CONFIG.meshtastic_bridge_enabled;
    doc["meshtastic_mqtt_prefix"]    = CONFIG.meshtastic_mqtt_prefix;
    doc["meshtastic_baud"]           = CONFIG.meshtastic_baud;
    doc["meshtastic_pin_rx"]         = CONFIG.meshtastic_pin_rx;
    doc["meshtastic_pin_tx"]         = CONFIG.meshtastic_pin_tx;

    // Open configuration file for writing (creates or overwrites existing)
    File f = SPIFFS.open(CFG_PATH, "w");
    if (!f) return false;

    // Serialize JSON document to file
    serializeJson(doc, f);
    
    // Close file and release resources
    f.close();
    
    return true;
}
