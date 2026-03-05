/*
 * ============================================================================
 * Command Handler Module - SmartFranklin
 * ============================================================================
 * 
 * File:        command_handler.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: MQTT command processing engine for remote device control.
 *              Parses JSON commands and executes corresponding device operations.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   This module implements a JSON-based command interface for remote control
 *   of SmartFranklin via MQTT. Commands are received as JSON payloads on
 *   dedicated MQTT topics and processed to execute device operations or
 *   retrieve system status information.
 * 
 * Command Structure:
 *   All commands follow a standardized JSON format with mandatory "cmd" field
 *   and optional parameters specific to each command type.
 * 
 *   General Format:
 *   {
 *     "cmd": "command_name",
 *     "param1": value1,
 *     "param2": value2
 *   }
 * 
 * Supported Commands:
 *   1. "set"  - Modify device state (LED, buzzer, battery targets)
 *   2. "get"  - Query system status (BMS, system metrics)
 * 
 * Dependencies:
 *   - ArduinoJson (JSON parsing library)
 *   - command_handler.h (header declarations)
 *   - data_model.h (global DATA object and state variables)
 *   - mqtt_layer.h (MQTT publishing)
 *   - tasks.h (FreeRTOS task management)
 * 
 * Error Handling:
 *   Invalid commands or JSON format errors are reported via MQTT:
 *   - smartfranklin/error: "invalid_json" (malformed JSON payload)
 *   - smartfranklin/error: "missing_cmd" (no "cmd" field in JSON)
 *   - smartfranklin/error: "unknown_cmd" (unrecognized command)
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

#include "command_handler.h"
#include "data_model.h"
#include "mqtt_layer.h"
#include "tasks.h"

#include <ArduinoJson.h>

// ============================================================================
// SET Command Handler
// ============================================================================


/**
 * @brief Processes "set" commands to modify device state and configuration.
 * 
 * Handles commands that change device operating state:
 *   - LED on/off control
 *   - Buzzer on/off control
 *   - Battery management system (BMS) target state of charge
 * 
 * Command Format:
 *   {
 *     "cmd": "set",
 *     "led": true,              (optional) Enable/disable LED
 *     "buzzer": false,          (optional) Enable/disable buzzer
 *     "target_soc": 80          (optional) Target battery SOC (0-100%)
 *   }
 * 
 * Response Behavior:
 *   Each successfully set parameter is confirmed via MQTT publication:
 *   - smartfranklin/led/state: "0" or "1"
 *   - smartfranklin/buzzer/state: "0" or "1"
 *   - smartfranklin/bms/target_soc: numeric value
 * 
 * Data Model Updates:
 *   Changes are persisted in the global DATA object:
 *   - DATA.led_state: Current LED operating state
 *   - DATA.buzzer_state: Current buzzer operating state
 *   - DATA.target_soc: Target battery charge percentage (0-100)
 * 
 * Parameter Validation:
 *   - LED/Buzzer: Expects boolean (true/false) values
 *   - Target SOC: Expects integer (0-100) representing percentage
 *   - Missing parameters: Silently ignored (optional fields)
 *   - Invalid types: Parameter skipped without error
 * 
 * @param doc - ArduinoJson JsonDocument containing parsed command JSON
 *              Must contain "led", "buzzer", or "target_soc" fields
 * 
 * @return void
 * 
 * @note Called by command_handler_process() when "set" command is detected.
 *       All state changes are immediately published to corresponding MQTT topics.
 */
static void handle_set(JsonDocument& doc) {
    // --- LED Control ---
    // Check if JSON contains boolean "led" field
    if (doc["led"].is<bool>()) {
        // Extract LED state from command
        bool state = doc["led"];
        
        // Update global device state
        DATA.led_state = state;
        
        // Publish confirmation to MQTT topic (1 = on, 0 = off)
        sf_mqtt::publish("smartfranklin/led/state", state ? "1" : "0");
    }

    // --- Buzzer Control ---
    // Check if JSON contains boolean "buzzer" field
    if (doc["buzzer"].is<bool>()) {
        // Extract buzzer state from command
        bool state = doc["buzzer"];
        
        // Update global device state
        DATA.buzzer_state = state;
        
        // Publish confirmation to MQTT topic (1 = on, 0 = off)
        sf_mqtt::publish("smartfranklin/buzzer/state", state ? "1" : "0");
    }

    // --- Battery Target State of Charge ---
    // Check if JSON contains integer "target_soc" field
    if (doc["target_soc"].is<int>()) {
        // Extract target SOC percentage (0-100%)
        int soc = doc["target_soc"];
        
        // Update global device state
        DATA.target_soc = soc;
        
        // Publish confirmation to MQTT topic with target percentage
        sf_mqtt::publish("smartfranklin/bms/target_soc", String(soc).c_str());
    }
}

// ============================================================================
// GET Command Handler
// ============================================================================


/**
 * @brief Processes "get" commands to retrieve device status and metrics.
 * 
 * Handles commands that query device operational state and system health.
 * Returns comprehensive JSON responses with requested information categories.
 * 
 * Command Format:
 *   {
 *     "cmd": "get",
 *     "what": "bms"             Required: Query target ("bms" or "system")
 *   }
 * 
 * Supported Query Targets:
 * 
 *   1. "bms" - Battery Management System Status
 *      Publishes to: smartfranklin/bms/status
 *      Response JSON:
 *      {
 *        "voltage": 48.5,        Battery voltage in volts (floating point)
 *        "current": 2.3,         Battery current in amps (floating point)
 *        "soc": 85               State of charge percentage (0-100)
 *      }
 *      Use case: Monitor battery health, charging status, and capacity
 * 
 *   2. "system" - System Health and Resource Status
 *      Publishes to: smartfranklin/system/status
 *      Response JSON:
 *      {
 *        "uptime": 3600000,      System uptime in milliseconds since boot
 *        "free_heap": 65536      Available RAM in bytes (heap memory)
 *      }
 *      Use case: Monitor system stability, memory usage, and runtime
 * 
 * Response Behavior:
 *   - Queries are JSON-formatted and published to specific MQTT topics
 *   - Responses contain dynamic data from global DATA object and system calls
 *   - Invalid query targets produce no response (silently ignored)
 * 
 * Parameter Validation:
 *   - "what" field: Expects string value ("bms" or "system")
 *   - Unknown targets: Silently ignored (no error published)
 *   - Missing "what": Command fails with "missing_cmd" error
 * 
 * @param doc - ArduinoJson JsonDocument containing parsed command JSON
 *              Must contain "what" field specifying query target
 * 
 * @return void
 * 
 * @note Called by command_handler_process() when "get" command is detected.
 *       Responses are published asynchronously to MQTT broker.
 * 
 * @see DATA global object - Contains BMS voltage, current, and SOC values
 * @see ESP.getFreeHeap() - Returns available heap memory in bytes
 */
static void handle_get(JsonDocument& doc) {
    // Check if JSON contains string "what" field specifying query target
    if (doc["what"].is<String>()) {
        // Extract query target (e.g., "bms", "system")
        String what = doc["what"].as<String>();

        // --- Battery Management System Status Query ---
        if (what == "bms") {
            // Create JSON response document
            JsonDocument out;
            
            // Populate BMS metrics from global data model
            out["voltage"] = DATA.bms_voltage;  // Battery voltage (volts)
            out["current"] = DATA.bms_current;  // Battery current (amps)
            out["soc"]     = DATA.bms_soc;      // State of charge (0-100%)

            // Serialize JSON to string for MQTT transmission
            String payload;
            serializeJson(out, payload);
            
            // Publish response to BMS status topic
            sf_mqtt::publish("smartfranklin/bms/status", payload.c_str());
        }

        // --- System Health and Resource Status Query ---
        if (what == "system") {
            // Create JSON response document
            JsonDocument out;
            
            // Populate system metrics
            out["uptime"] = millis();              // System uptime (milliseconds since boot)
            out["free_heap"] = ESP.getFreeHeap(); // Available RAM (bytes)

            // Serialize JSON to string for MQTT transmission
            String payload;
            serializeJson(out, payload);
            
            // Publish response to system status topic
            sf_mqtt::publish("smartfranklin/system/status", payload.c_str());
        }
    }
}

// ============================================================================
// Main Command Processing Engine
// ============================================================================


/**
 * @brief Parses and processes incoming JSON commands from MQTT.
 * 
 * Central command handler that deserializes JSON command payloads,
 * validates command structure, and routes to appropriate handler functions.
 * Implements comprehensive error reporting via MQTT error topic.
 * 
 * Processing Flow:
 *   1. Deserialize JSON string to JsonDocument
 *   2. Check for deserializaton errors (malformed JSON)
 *   3. Extract "cmd" field from JSON root
 *   4. Validate command type (known vs. unknown)
 *   5. Route to handler (handle_set, handle_get, etc.)
 *   6. Return error descriptions for invalid inputs
 * 
 * Command Routing:
 *   - "set"   → handle_set()   : Modify device state
 *   - "get"   → handle_get()   : Query system status
 *   - others  → Error response : Unknown command publish
 * 
 * Input Validation:
 *   The function validates at multiple levels:
 *   - JSON syntax: Rejects malformed/unparseable JSON
 *   - Command field: Requires presence of "cmd" field
 *   - Command value: Rejects unknown command identifiers
 * 
 * Error Responses:
 *   Errors are published to "smartfranklin/error" MQTT topic:
 * 
 *   Error Type              | Cause                    | Published Value
 *   ------------------------|--------------------------|------------------
 *   "invalid_json"          | Malformed JSON syntax    | JSON parse error
 *   "missing_cmd"           | No "cmd" field in JSON   | Command field absent
 *   "unknown_cmd"           | Unrecognized command     | cmd != "set"/"get"
 * 
 * JSON Examples:
 * 
 *   Valid "set" command:
 *   {"cmd":"set","led":true,"buzzer":false,"target_soc":80}
 * 
 *   Valid "get" command:
 *   {"cmd":"get","what":"bms"}
 * 
 *   Invalid: Missing "cmd" field
 *   {"led":true,"buzzer":false}
 *   → Publishes: smartfranklin/error = "missing_cmd"
 * 
 *   Invalid: Malformed JSON
 *   {cmd:"set",led:true}
 *   → Publishes: smartfranklin/error = "invalid_json"
 * 
 * @param json - JSON string containing command payload
 *               Expected format: {"cmd":"...", param1:value1, ...}
 *               Example: {"cmd":"set","led":true}
 * 
 * @return void
 * 
 * @note Typically called from MQTT message callback when command topic received.
 *       All errors and results are reported asynchronously via MQTT.
 * 
 * @see handle_set()  - Handler for state modification commands
 * @see handle_get()  - Handler for status query commands
 * @see sf_mqtt::publish() - Publishes response/error to MQTT broker
 */
void command_handler_process(const String& json) {
    // Create JSON document for deserialization
    JsonDocument doc;
    
    // Attempt to parse JSON string into document
    DeserializationError err = deserializeJson(doc, json);

    // --- JSON Parsing Validation ---
    // Check if JSON deserialization failed
    if (err) {
        // Publish error indicating malformed JSON syntax
        sf_mqtt::publish("smartfranklin/error", "invalid_json");
        return;
    }

    // --- Command Field Extraction and Validation ---
    // Check if JSON root contains string "cmd" field
    if (doc["cmd"].is<String>()) {
        // Extract command identifier
        String cmd = doc["cmd"].as<String>();

        // --- Command Routing: SET ---
        // Route "set" commands to state modification handler
        if (cmd == "set") {
            handle_set(doc);
            return;
        }

        // --- Command Routing: GET ---
        // Route "get" commands to status query handler
        if (cmd == "get") {
            handle_get(doc);
            return;
        }

        // --- Unknown Command Error ---
        // Publish error for unrecognized command identifier
        sf_mqtt::publish("smartfranklin/error", "unknown_cmd");
        return;
    }

    // --- Missing Command Field Error ---
    // Publish error if JSON lacks "cmd" field
    sf_mqtt::publish("smartfranklin/error", "missing_cmd");
}
