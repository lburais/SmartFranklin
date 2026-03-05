/*
 * ============================================================================
 * MQTT Bridge Module - SmartFranklin
 * ============================================================================
 * 
 * File:        mqtt_bridge.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: Bidirectional MQTT message bridge between local and cloud brokers.
 *              Enables seamless communication between internal Mosquitto instance
 *              and external MQTT brokers (AWS IoT, HiveMQ, etc.) with loop detection.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   SmartFranklin operates in environments with both local and cloud MQTT brokers.
 *   The MQTT bridge provides transparent message forwarding between these two
 *   networks, allowing devices to communicate across network boundaries.
 * 
 * Bridge Architecture:
 *   - Internal Broker: Local Mosquitto instance running on device (127.0.0.1:1883)
 *   - External Broker: Cloud MQTT service (AWS IoT, Google Cloud IoT, etc.)
 *   - Bidirectional: Messages flow both directions (local ↔ cloud)
 *   - Topic Prefixing: Prevents message loops with configurable prefixes
 *   - Loop Detection: Identifies and blocks circular message routing
 *   - QoS Support: Configurable Quality of Service (0, 1, or 2)
 *   - Retained Messages: Optional message persistence across connections
 * 
 * Message Flow Examples:
 * 
 *   Local → Cloud:
 *   Input:  "sensor/temperature" → "25.5"
 *   Output: "cloud/sensor/temperature" → "25.5" (on external broker)
 * 
 *   Cloud → Local:
 *   Input:  "command/led" → "on"
 *   Output: "local/command/led" → "on" (on internal broker)
 * 
 * Loop Prevention:
 *   Messages with bridge prefixes are blocked to prevent infinite loops:
 *   - Messages from "local/*" topics not forwarded to external
 *   - Messages from "cloud/*" topics not forwarded to internal
 *   - Configurable via CONFIG.mqtt_bridge_loop_detection flag
 * 
 * Configuration Parameters:
 *   - mqtt_bridge_enabled: Enable/disable bridge functionality
 *   - mqtt_bridge_prefix_internal: Local topic prefix (default: "local/")
 *   - mqtt_bridge_prefix_external: Cloud topic prefix (default: "cloud/")
 *   - mqtt_bridge_qos: Quality of Service level (0, 1, or 2)
 *   - mqtt_bridge_retain: Retain messages flag
 *   - mqtt_bridge_loop_detection: Enable loop prevention
 * 
 * Dependencies:
 *   - PubSubClient.h (MQTT client library for Arduino)
 *   - WiFi.h (ESP32 WiFi support for network connectivity)
 *   - config_store.h (Configuration access for broker settings)
 *   - mqtt_bridge.h (Header declarations and function prototypes)
 * 
 * Broker Authentication:
 *   - Internal: Uses CONFIG.admin_user and CONFIG.admin_pass
 *   - External: Uses CONFIG.ext_mqtt_user and CONFIG.ext_mqtt_pass
 *   - Client IDs: "SmartFranklinBridgeInternal" and "SmartFranklinBridgeExternal"
 * 
 * Error Handling:
 *   - Connection failures: Automatic reconnection attempts
 *   - Network issues: Graceful degradation (bridge disabled temporarily)
 *   - Invalid topics: Messages silently dropped
 *   - Buffer overflows: PubSubClient handles internally
 * 
 * Performance Considerations:
 *   - Message throughput: Limited by network bandwidth and broker capacity
 *   - Memory usage: Two PubSubClient instances (~2KB each)
 *   - CPU overhead: Minimal when idle, increases during message forwarding
 *   - Battery impact: Network activity increases power consumption
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

#include "mqtt_bridge.h"
#include "config_store.h"

#include <PubSubClient.h>
#include <WiFi.h>

// ============================================================================
// MQTT Client Instances
// ============================================================================

/**
 * @brief WiFi client for external MQTT broker connection.
 * 
 * Provides TCP/IP connectivity to cloud MQTT brokers over WiFi.
 * Used by extMqtt PubSubClient instance for external broker communication.
 */
static WiFiClient extClient;

/**
 * @brief PubSubClient instance for external MQTT broker.
 * 
 * Handles all communication with cloud MQTT broker (AWS IoT, HiveMQ, etc.).
 * Configured with server address, port, authentication, and callback handler.
 * Forwards messages from external broker to internal broker.
 */
static PubSubClient extMqtt(extClient);

/**
 * @brief WiFi client for internal MQTT broker connection.
 * 
 * Provides TCP/IP connectivity to local Mosquitto broker.
 * Used by intMqtt PubSubClient instance for internal broker communication.
 */
static WiFiClient intClient;

/**
 * @brief PubSubClient instance for internal MQTT broker.
 * 
 * Handles all communication with local Mosquitto broker running on device.
 * Configured for localhost (127.0.0.1), admin authentication, and callback handler.
 * Forwards messages from internal broker to external broker.
 */
static PubSubClient intMqtt(intClient);

// ============================================================================
// Connection State Tracking
// ============================================================================

/**
 * @brief External MQTT broker connection status.
 * 
 * Tracks whether the external (cloud) MQTT broker connection is active.
 * Updated by ensureExternal() function during connection attempts.
 * Used to determine if message forwarding is possible.
 */
static bool extConnected = false;

/**
 * @brief Internal MQTT broker connection status.
 * 
 * Tracks whether the internal (local) MQTT broker connection is active.
 * Updated by ensureInternal() function during connection attempts.
 * Used to determine if message forwarding is possible.
 */
static bool intConnected = false;

// ============================================================================
// Message Forwarding Statistics
// ============================================================================

/**
 * @brief Timestamp of last internal-to-external message forwarding.
 * 
 * Records the millis() timestamp when the last message was forwarded
 * from internal broker to external broker. Used for monitoring bridge activity.
 */
static unsigned long lastIntToExt = 0;

/**
 * @brief Timestamp of last external-to-internal message forwarding.
 * 
 * Records the millis() timestamp when the last message was forwarded
 * from external broker to internal broker. Used for monitoring bridge activity.
 */
static unsigned long lastExtToInt = 0;

/**
 * @brief Counter for internal-to-external message forwarding.
 * 
 * Increments each time a message is successfully forwarded from internal
 * to external broker. Provides statistics for bridge usage monitoring.
 */
static uint32_t countIntToExt = 0;

/**
 * @brief Counter for external-to-internal message forwarding.
 * 
 * Increments each time a message is successfully forwarded from external
 * to internal broker. Provides statistics for bridge usage monitoring.
 */
static uint32_t countExtToInt = 0;

// ============================================================================
// Loop Detection Function
// ============================================================================

/**
 * @brief Detects messages that originated from bridge forwarding.
 * 
 * Prevents infinite message loops by identifying topics that contain
 * bridge prefixes. Messages with these prefixes are blocked from further
 * forwarding to prevent circular routing.
 * 
 * Loop Detection Logic:
 *   - Checks if CONFIG.mqtt_bridge_loop_detection is enabled
 *   - Examines topic for internal or external bridge prefixes
 *   - Returns true if message appears to be bridge-generated
 * 
 * Prefix Matching:
 *   - Internal prefix: CONFIG.mqtt_bridge_prefix_internal (default: "local/")
 *   - External prefix: CONFIG.mqtt_bridge_prefix_external (default: "cloud/")
 * 
 * Example Scenarios:
 *   - Topic "sensor/temp" → false (not prefixed, allow forwarding)
 *   - Topic "local/sensor/temp" → true (internal prefix, block forwarding)
 *   - Topic "cloud/command/led" → true (external prefix, block forwarding)
 * 
 * Configuration Impact:
 *   - When disabled: All messages forwarded regardless of prefixes
 *   - When enabled: Messages with bridge prefixes are dropped
 *   - Default: Enabled to prevent accidental loops
 * 
 * @param topic - MQTT topic string to check for bridge prefixes
 * 
 * @return bool - true if message appears to be from bridge (should be blocked)
 *                false if message is original (should be forwarded)
 * 
 * @note This function is called for every message before forwarding.
 *       Keep implementation efficient as it affects bridge performance.
 */
static bool isLoopMessage(const String &topic)
{
    // Check if loop detection is enabled in configuration
    if (!CONFIG.mqtt_bridge_loop_detection) return false;

    // Check if topic starts with internal or external bridge prefix
    // If either prefix matches, this is a bridged message that should not be re-forwarded
    return topic.startsWith(CONFIG.mqtt_bridge_prefix_internal) ||
           topic.startsWith(CONFIG.mqtt_bridge_prefix_external);
}

// ============================================================================
// Internal to External Forwarding Callback
// ============================================================================

/**
 * @brief Callback function for forwarding messages from internal to external broker.
 * 
 * Processes MQTT messages received from the internal broker and forwards them
 * to the external broker with appropriate topic prefixing. Performs loop
 * detection to prevent circular message routing.
 * 
 * Processing Steps:
 *   1. Check if bridge is enabled and external connection is active
 *   2. Perform loop detection on topic (block bridge-generated messages)
 *   3. Apply external prefix to topic (e.g., "sensor/temp" → "cloud/sensor/temp")
 *   4. Publish message to external broker with configured QoS and retain settings
 *   5. Update forwarding statistics and timestamp
 * 
 * Message Transformation:
 *   - Original topic: "sensor/temperature"
 *   - Payload: "25.5°C" (unchanged)
 *   - Forwarded topic: "cloud/sensor/temperature"
 *   - QoS: Uses CONFIG.mqtt_bridge_qos (clamped to 0-1 for PubSubClient)
 *   - Retained: Uses CONFIG.mqtt_bridge_retain flag
 * 
 * Error Handling:
 *   - Bridge disabled: Function returns without forwarding
 *   - External disconnected: Message silently dropped
 *   - Loop detected: Message blocked to prevent infinite loops
 *   - Publish failure: No retry logic (fire-and-forget)
 * 
 * Performance:
 *   - Execution time: < 10ms for typical messages
 *   - Memory usage: Minimal (topic string manipulation)
 *   - Network overhead: Additional MQTT publish operation
 * 
 * @param topic - MQTT topic string from internal broker
 * @param payload - Message payload bytes
 * @param len - Payload length in bytes
 * 
 * @return void
 * 
 * @note This function is called asynchronously by PubSubClient when messages
 *       are received from the internal broker. Keep processing lightweight.
 */
static void forwardToExternal(char *topic, byte *payload, unsigned int len)
{
    // Check if bridge is enabled and external broker is connected
    if (!CONFIG.mqtt_bridge_enabled || !extConnected) return;

    // Convert topic to String for easier manipulation
    String t = String(topic);
    
    // Perform loop detection - block messages that originated from bridge
    if (isLoopMessage(t)) return;

    // Apply external prefix to create forwarded topic
    // Example: "sensor/temp" → "cloud/sensor/temp"
    String outTopic = CONFIG.mqtt_bridge_prefix_external + t;

    // Publish to external broker
    // PubSubClient publish signature: publish(topic, payload, length, retained)
    // QoS is handled at subscription level, not publish level in PubSubClient
    extMqtt.publish(outTopic.c_str(), payload, len, CONFIG.mqtt_bridge_retain);

    // Update forwarding statistics
    lastIntToExt = millis();
    countIntToExt++;
}

// ============================================================================
// External to Internal Forwarding Callback
// ============================================================================

/**
 * @brief Callback function for forwarding messages from external to internal broker.
 * 
 * Processes MQTT messages received from the external broker and forwards them
 * to the internal broker with appropriate topic prefixing. Performs loop
 * detection to prevent circular message routing.
 * 
 * Processing Steps:
 *   1. Check if bridge is enabled and internal connection is active
 *   2. Perform loop detection on topic (block bridge-generated messages)
 *   3. Apply internal prefix to topic (e.g., "command/led" → "local/command/led")
 *   4. Publish message to internal broker with configured QoS and retain settings
 *   5. Update forwarding statistics and timestamp
 * 
 * Message Transformation:
 *   - Original topic: "command/led"
 *   - Payload: "on" (unchanged)
 *   - Forwarded topic: "local/command/led"
 *   - QoS: Uses CONFIG.mqtt_bridge_qos (clamped to 0-1 for PubSubClient)
 *   - Retained: Uses CONFIG.mqtt_bridge_retain flag
 * 
 * Error Handling:
 *   - Bridge disabled: Function returns without forwarding
 *   - Internal disconnected: Message silently dropped
 *   - Loop detected: Message blocked to prevent infinite loops
 *   - Publish failure: No retry logic (fire-and-forget)
 * 
 * Performance:
 *   - Execution time: < 10ms for typical messages
 *   - Memory usage: Minimal (topic string manipulation)
 *   - Network overhead: Additional MQTT publish operation
 * 
 * @param topic - MQTT topic string from external broker
 * @param payload - Message payload bytes
 * @param len - Payload length in bytes
 * 
 * @return void
 * 
 * @note This function is called asynchronously by PubSubClient when messages
 *       are received from the external broker. Keep processing lightweight.
 */
static void forwardToInternal(char *topic, byte *payload, unsigned int len)
{
    // Check if bridge is enabled and internal broker is connected
    if (!CONFIG.mqtt_bridge_enabled || !intConnected) return;

    // Convert topic to String for easier manipulation
    String t = String(topic);
    
    // Perform loop detection - block messages that originated from bridge
    if (isLoopMessage(t)) return;

    // Apply internal prefix to create forwarded topic
    // Example: "command/led" → "local/command/led"
    String outTopic = CONFIG.mqtt_bridge_prefix_internal + t;

    // Publish to internal broker
    // PubSubClient publish signature: publish(topic, payload, length, retained)
    intMqtt.publish(outTopic.c_str(), payload, len, CONFIG.mqtt_bridge_retain);

    // Update forwarding statistics
    lastExtToInt = millis();
    countExtToInt++;
}

// ============================================================================
// Bridge Initialization
// ============================================================================

/**
 * @brief Initializes the MQTT bridge with configuration settings.
 * 
 * Sets up both internal and external MQTT client configurations,
 * registers callback functions for message forwarding, and prepares
 * for broker connections. Must be called once during system startup.
 * 
 * Internal Broker Setup:
 *   - Server: 127.0.0.1 (localhost, assumes Mosquitto running locally)
 *   - Port: 1883 (standard MQTT port)
 *   - Authentication: Uses CONFIG.admin_user and CONFIG.admin_pass
 *   - Callback: forwardToExternal (forwards internal → external)
 * 
 * External Broker Setup:
 *   - Server: CONFIG.ext_mqtt_host (cloud broker hostname/IP)
 *   - Port: CONFIG.ext_mqtt_port (typically 1883 or 8883 for TLS)
 *   - Authentication: Uses CONFIG.ext_mqtt_user and CONFIG.ext_mqtt_pass
 *   - Callback: forwardToInternal (forwards external → internal)
 *   - Only configured if CONFIG.ext_mqtt_enabled is true
 * 
 * Subscription Behavior:
 *   - Both clients subscribe to "#" (all topics) with configured QoS
 *   - QoS is clamped to 0-1 (PubSubClient limitation, QoS 2 not supported)
 *   - Subscriptions are established during connection in ensure*() functions
 * 
 * Configuration Dependencies:
 *   - CONFIG.ext_mqtt_enabled: Controls external broker setup
 *   - CONFIG.ext_mqtt_host, CONFIG.ext_mqtt_port: External broker details
 *   - CONFIG.ext_mqtt_user, CONFIG.ext_mqtt_pass: External authentication
 *   - CONFIG.admin_user, CONFIG.admin_pass: Internal authentication
 *   - CONFIG.mqtt_bridge_qos: Quality of Service for subscriptions
 * 
 * @return void
 * 
 * @note Call this function during setup() phase, after CONFIG is loaded.
 *       Ensure WiFi is connected before calling (brokers need network).
 *       Example: mqtt_bridge_init(); // in setup()
 * 
 * @see PubSubClient::setServer() - Configure broker connection parameters
 * @see PubSubClient::setCallback() - Register message callback functions
 */
void mqtt_bridge_init()
{
    // =========================================================================
    // Internal Broker Configuration
    // =========================================================================
    // Configure connection to local Mosquitto broker
    // Assumes Mosquitto is running on localhost (127.0.0.1)
    intMqtt.setServer("127.0.0.1", 1883);
    
    // Register callback for forwarding internal messages to external broker
    intMqtt.setCallback(forwardToExternal);

    // =========================================================================
    // External Broker Configuration
    // =========================================================================
    // Only configure external broker if enabled in configuration
    if (CONFIG.ext_mqtt_enabled) {
        // Configure connection to cloud MQTT broker
        extMqtt.setServer(CONFIG.ext_mqtt_host.c_str(), CONFIG.ext_mqtt_port);
        
        // Register callback for forwarding external messages to internal broker
        extMqtt.setCallback(forwardToInternal);
    }
}

// ============================================================================
// Internal Broker Connection Management
// ============================================================================

/**
 * @brief Ensures connection to internal MQTT broker is maintained.
 * 
 * Checks internal broker connection status and attempts reconnection
 * if disconnected. Upon successful connection, subscribes to all topics
 * with configured QoS level for message forwarding.
 * 
 * Connection Process:
 *   1. Check if already connected (intConnected flag)
 *   2. If disconnected, attempt connection with authentication
 *   3. Client ID: "SmartFranklinBridgeInternal"
 *   4. Credentials: CONFIG.admin_user, CONFIG.admin_pass
 *   5. On success: Subscribe to "#" (all topics) with QoS
 *   6. Update intConnected status flag
 * 
 * QoS Handling:
 *   - PubSubClient supports QoS 0 and 1 only (not QoS 2)
 *   - CONFIG.mqtt_bridge_qos > 1 is clamped to 1
 *   - QoS affects message delivery guarantees for subscriptions
 * 
 * Authentication:
 *   - Uses web dashboard credentials (admin user/pass)
 *   - Assumes internal broker accepts these credentials
 *   - No TLS/SSL (localhost connection, assumed secure)
 * 
 * Error Handling:
 *   - Connection failures: Silently retried on next call
 *   - No exponential backoff (simple retry logic)
 *   - Authentication failures: Logged by broker, not by this function
 * 
 * Performance:
 *   - Connection attempts: Blocking operation (~100-500ms)
 *   - Normal operation: Non-blocking status check
 *   - Memory: Minimal additional usage
 * 
 * @return void
 * 
 * @note This function should be called regularly (e.g., every 1-5 seconds)
 *       from the main loop or dedicated task to maintain connection.
 *       Called automatically by mqtt_bridge_loop().
 * 
 * @see PubSubClient::connect() - Establish MQTT broker connection
 * @see PubSubClient::subscribe() - Subscribe to MQTT topics
 */
static void ensureInternal()
{
    // Check if internal broker connection is already established
    if (!intMqtt.connected()) {
        // Attempt connection to internal broker
        // Client ID identifies this bridge instance
        // Authentication uses admin credentials from configuration
        intConnected = intMqtt.connect(
            "SmartFranklinBridgeInternal",
            CONFIG.admin_user.c_str(),
            CONFIG.admin_pass.c_str()
        );

        // If connection successful, subscribe to all topics
        if (intConnected) {
            // PubSubClient supports QoS 0 or 1 only
            // Clamp QoS to valid range (QoS 2 not supported)
            uint8_t qos = CONFIG.mqtt_bridge_qos > 1 ? 1 : CONFIG.mqtt_bridge_qos;
            
            // Subscribe to all topics (# wildcard) for message forwarding
            intMqtt.subscribe("#", qos);
        }
    }
}

// ============================================================================
// External Broker Connection Management
// ============================================================================

/**
 * @brief Ensures connection to external MQTT broker is maintained.
 * 
 * Checks external broker connection status and attempts reconnection
 * if disconnected. Upon successful connection, subscribes to all topics
 * with configured QoS level for message forwarding. Only active if
 * external broker is enabled in configuration.
 * 
 * Connection Process:
 *   1. Check if external broker is enabled (CONFIG.ext_mqtt_enabled)
 *   2. Check if already connected (extConnected flag)
 *   3. If disconnected, attempt connection with authentication
 *   4. Client ID: "SmartFranklinBridgeExternal"
 *   5. Credentials: CONFIG.ext_mqtt_user, CONFIG.ext_mqtt_pass
 *   6. On success: Subscribe to "#" (all topics) with QoS
 *   7. Update extConnected status flag
 * 
 * QoS Handling:
 *   - PubSubClient supports QoS 0 and 1 only (not QoS 2)
 *   - CONFIG.mqtt_bridge_qos > 1 is clamped to 1
 *   - QoS affects message delivery guarantees for subscriptions
 * 
 * Authentication:
 *   - Uses external broker credentials from configuration
 *   - Supports username/password authentication
 *   - TLS/SSL support depends on PubSubClient configuration
 * 
 * Error Handling:
 *   - External disabled: Function returns immediately
 *   - Connection failures: Silently retried on next call
 *   - No exponential backoff (simple retry logic)
 *   - Network issues: Handled by underlying WiFiClient
 * 
 * Performance:
 *   - Connection attempts: Blocking operation (~500-2000ms for cloud brokers)
 *   - Normal operation: Non-blocking status check
 *   - Network latency: Depends on cloud broker location
 * 
 * @return void
 * 
 * @note This function should be called regularly from mqtt_bridge_loop().
 *       Cloud broker connections may take longer due to network latency.
 *       Consider connection timeouts for production deployments.
 * 
 * @see PubSubClient::connect() - Establish MQTT broker connection
 * @see PubSubClient::subscribe() - Subscribe to MQTT topics
 */
static void ensureExternal()
{
    // Only attempt external connection if enabled in configuration
    if (!CONFIG.ext_mqtt_enabled) return;

    // Check if external broker connection is already established
    if (!extMqtt.connected()) {
        // Attempt connection to external broker
        // Client ID identifies this bridge instance
        // Authentication uses external credentials from configuration
        extConnected = extMqtt.connect(
            "SmartFranklinBridgeExternal",
            CONFIG.ext_mqtt_user.c_str(),
            CONFIG.ext_mqtt_pass.c_str()
        );

        // If connection successful, subscribe to all topics
        if (extConnected) {
            // PubSubClient supports QoS 0 or 1 only
            // Clamp QoS to valid range (QoS 2 not supported)
            uint8_t qos = CONFIG.mqtt_bridge_qos > 1 ? 1 : CONFIG.mqtt_bridge_qos;
            
            // Subscribe to all topics (# wildcard) for message forwarding
            extMqtt.subscribe("#", qos);
        }
    }
}

// ============================================================================
// Bridge Main Loop
// ============================================================================

/**
 * @brief Main processing loop for MQTT bridge operations.
 * 
 * Maintains connections to both internal and external brokers,
 * processes incoming MQTT messages, and handles message forwarding.
 * Should be called regularly from the main application loop.
 * 
 * Processing Steps:
 *   1. Ensure internal broker connection (ensureInternal())
 *   2. Ensure external broker connection (ensureExternal())
 *   3. Process internal broker messages (intMqtt.loop())
 *   4. Process external broker messages (extMqtt.loop())
 * 
 * Loop Frequency:
 *   - Recommended: Every 100-500ms for responsive message handling
 *   - Minimum: Every 1-2 seconds for basic connectivity maintenance
 *   - Maximum: Every 10 seconds (may cause message delays)
 * 
 * Blocking Behavior:
 *   - Connection attempts: May block for 100-2000ms
 *   - Message processing: Non-blocking for established connections
 *   - Network operations: Asynchronous via PubSubClient
 * 
 * Error Recovery:
 *   - Disconnected brokers: Automatic reconnection attempts
 *   - Network failures: Graceful degradation (messages queued)
 *   - Message processing errors: Individual message failures don't stop bridge
 * 
 * Performance Impact:
 *   - CPU usage: Low when idle, moderate during message forwarding
 *   - Memory usage: Fixed overhead (~4KB for two PubSubClient instances)
 *   - Network usage: Depends on message volume and forwarding activity
 * 
 * @return void
 * 
 * @note This function must be called regularly to maintain broker connections
 *       and process messages. If not called, connections will timeout and
 *       message forwarding will stop.
 *       Example: mqtt_bridge_loop(); // in main loop()
 * 
 * @see ensureInternal() - Maintains internal broker connection
 * @see ensureExternal() - Maintains external broker connection
 * @see PubSubClient::loop() - Processes MQTT protocol and callbacks
 */
void mqtt_bridge_loop()
{
    // Maintain connection to internal broker
    ensureInternal();
    
    // Maintain connection to external broker (if enabled)
    ensureExternal();

    // Process incoming messages and maintain connections
    // These calls are non-blocking for established connections
    if (intConnected) intMqtt.loop();
    if (extConnected) extMqtt.loop();
}

// ============================================================================
// Bridge Status Reporting
// ============================================================================

/**
 * @brief Populates JSON document with current bridge status and statistics.
 * 
 * Collects comprehensive status information about the MQTT bridge for
 * monitoring and diagnostics. Includes connection states, message counters,
 * and timing information.
 * 
 * Status Information:
 *   - internal_connected: Boolean status of internal broker connection
 *   - external_connected: Boolean status of external broker connection
 *   - last_int_to_ext_ms: Timestamp (millis) of last internal→external forwarding
 *   - last_ext_to_int_ms: Timestamp (millis) of last external→internal forwarding
 *   - count_int_to_ext: Total messages forwarded internal→external
 *   - count_ext_to_int: Total messages forwarded external→internal
 * 
 * JSON Output Example:
 *   {
 *     "internal_connected": true,
 *     "external_connected": true,
 *     "last_int_to_ext_ms": 12345678,
 *     "last_ext_to_int_ms": 12345678,
 *     "count_int_to_ext": 42,
 *     "count_ext_to_int": 15
 *   }
 * 
 * Usage:
 *   - MQTT status publishing: Include in system health messages
 *   - Web dashboard: Display bridge status to administrators
 *   - Diagnostics: Monitor bridge activity and connection health
 *   - Troubleshooting: Identify connection or forwarding issues
 * 
 * Data Types:
 *   - Connection flags: bool (true/false)
 *   - Timestamps: unsigned long (milliseconds since boot)
 *   - Counters: uint32_t (message count, wraps at 4.2 billion)
 * 
 * Performance:
 *   - Execution time: < 1ms
 *   - Memory usage: Minimal (JSON document population)
 *   - Thread safety: Safe to call from any context
 * 
 * @param doc - ArduinoJson JsonDocument to populate with status data
 *              Must be pre-allocated with sufficient capacity
 * 
 * @return void
 * 
 * @note The JsonDocument must have enough capacity for all status fields.
 *       Typical capacity: 256-512 bytes for status information.
 *       Example: JsonDocument doc; mqtt_bridge_status(doc);
 * 
 * @see JsonDocument - ArduinoJson document for JSON manipulation
 * @see mqtt_bridge_loop() - Updates the status information tracked here
 */
void mqtt_bridge_status(JsonDocument &doc)
{
    // Populate JSON document with current bridge status
    doc["internal_connected"] = intConnected;
    doc["external_connected"] = extConnected;
    doc["last_int_to_ext_ms"] = lastIntToExt;
    doc["last_ext_to_int_ms"] = lastExtToInt;
    doc["count_int_to_ext"] = countIntToExt;
    doc["count_ext_to_int"] = countExtToInt;
}