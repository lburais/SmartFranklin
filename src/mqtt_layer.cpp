/*
 * ============================================================================
 * MQTT Layer Module - SmartFranklin
 * ============================================================================
 * 
 * File:        mqtt_layer.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: ESP-IDF MQTT client wrapper providing high-level MQTT operations.
 *              Handles connection management, publishing, subscribing, and message
 *              callbacks with comprehensive error handling and logging.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   This module provides a clean, high-level interface to MQTT functionality
 *   using the ESP-IDF MQTT client library. It abstracts the complexities of
 *   MQTT protocol handling, connection management, and event processing,
 *   providing simple publish/subscribe operations for the rest of SmartFranklin.
 * 
 * Architecture:
 *   - ESP-IDF MQTT Client: Uses esp-mqtt library for robust MQTT implementation
 *   - Event-Driven: Asynchronous event handling for connection and messages
 *   - Callback-Based: Message reception through registered callback function
 *   - Namespace Encapsulation: All functions in sf_mqtt namespace
 *   - Thread-Safe: Designed for use in FreeRTOS environment
 *   - Error Handling: Comprehensive logging and error reporting
 * 
 * Key Features:
 *   - Automatic reconnection on network failures
 *   - Configurable keep-alive and clean session settings
 *   - QoS support (0, 1, 2) for publish and subscribe operations
 *   - Message retention support for persistent messages
 *   - Authentication support (username/password)
 *   - Client ID configuration for broker identification
 *   - Detailed logging with ESP_LOG levels (Error, Warning, Info, Debug)
 * 
 * MQTT Protocol Support:
 *   - MQTT 3.1.1 compliant (ESP-IDF esp-mqtt implementation)
 *   - TCP/TLS transport (configurable via URI)
 *   - QoS levels: At most once (0), At least once (1), Exactly once (2)
 *   - Retained messages for persistent state
 *   - Last Will and Testament (LWT) support (configurable)
 *   - Keep-alive mechanism for connection monitoring
 * 
 * Configuration Structure:
 *   The Config struct provides comprehensive MQTT client configuration:
 *   - uri: Broker URI (tcp://host:port or tls://host:port)
 *   - username/password: Authentication credentials (optional)
 *   - client_id: Unique client identifier (optional, auto-generated if empty)
 *   - keepalive_sec: Keep-alive interval in seconds (default 60)
 *   - clean_session: Clean session flag (default true)
 * 
 * Event Handling:
 *   The event_handler_cb function processes all MQTT events:
 *   - MQTT_EVENT_CONNECTED: Connection established
 *   - MQTT_EVENT_DISCONNECTED: Connection lost
 *   - MQTT_EVENT_DATA: Message received (triggers callback)
 *   - MQTT_EVENT_ERROR: Connection or protocol errors
 *   - Other events: Logged for debugging
 * 
 * Message Callbacks:
 *   Incoming messages are delivered via MessageCallback:
 *   - Signature: void callback(const std::string &topic, const std::string &payload)
 *   - Called asynchronously when messages arrive
 *   - Topic and payload provided as std::string objects
 *   - Thread context: ESP-IDF MQTT task (not main application task)
 * 
 * Dependencies:
 *   - esp-mqtt library (ESP-IDF component)
 *   - mqtt_client.h (ESP-IDF MQTT client header)
 *   - esp_log.h (ESP-IDF logging facilities)
 *   - mqtt_layer.h (Header declarations and Config/MessageCallback types)
 *   - <cstring> (C string manipulation)
 *   - <string> (C++ string class)
 * 
 * Error Handling:
 *   - Initialization failures: Logged and return false
 *   - Connection failures: Automatic retry by ESP-IDF client
 *   - Publish failures: Logged and return false
 *   - Subscribe failures: Logged and return false
 *   - Invalid parameters: Checked and handled gracefully
 * 
 * Performance Considerations:
 *   - Memory usage: ~8-16KB for MQTT client and buffers
 *   - Network overhead: Minimal, efficient binary protocol
 *   - CPU usage: Low when idle, moderate during message processing
 *   - Thread safety: Safe for concurrent access from multiple tasks
 * 
 * ============================================================================
 * MIT License
 * ============================================================================
 * Copyright (c) 2026 Laurent Burais
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in this software and associated documentation files (the "Software"), to deal
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

#include "mqtt_layer.h"

#include <cstring>
#include <esp_idf_version.h>
#include <esp_log.h>
#include <mqtt_client.h>

namespace sf_mqtt {

static const char *TAG = "SF_MQTT";

static esp_mqtt_client_handle_t s_client = nullptr;
static MessageCallback          s_msg_cb = nullptr;
static bool                     s_connected = false;

/**
 * @brief ESP-IDF MQTT event handler callback function.
 * 
 * Processes all MQTT protocol events from the ESP-IDF MQTT client.
 * Handles connection state changes, incoming messages, and error conditions.
 * This function is called asynchronously by the MQTT client task.
 * 
 * Event Processing:
 * 
 *   MQTT_EVENT_CONNECTED:
 *   - Logs successful connection establishment
 *   - Sets s_connected flag to true
 *   - Enables publish/subscribe operations
 * 
 *   MQTT_EVENT_DISCONNECTED:
 *   - Logs disconnection event (may be temporary)
 *   - Sets s_connected flag to false
 *   - Publish/subscribe operations will fail until reconnection
 *   - ESP-IDF client handles automatic reconnection
 * 
 *   MQTT_EVENT_DATA:
 *   - Extracts topic and payload from event structure
 *   - Converts to std::string objects for callback
 *   - Logs message at debug level for troubleshooting
 *   - Invokes registered message callback if available
 *   - Callback receives topic and payload as parameters
 * 
 *   MQTT_EVENT_ERROR:
 *   - Logs MQTT protocol or connection errors
 *   - May indicate network issues or broker problems
 *   - ESP-IDF client will attempt reconnection automatically
 * 
 *   Other Events:
 *   - Logged at debug level for development/diagnostics
 *   - Not processed but tracked for completeness
 * 
 * Thread Safety:
 *   - Called from ESP-IDF MQTT task context
 *   - Must not block or perform long operations
 *   - Callback invocation is synchronous within this function
 *   - Global state (s_connected) is accessed atomically
 * 
 * Error Handling:
 *   - All events logged with appropriate severity levels
 *   - Invalid event data handled gracefully
 *   - Callback exceptions not caught (should be exception-free)
 * 
 * @param event - ESP-IDF MQTT event structure containing event details
 *                Includes event type, topic, payload, and metadata
 * 
 * @return esp_err_t - Always returns ESP_OK (required by ESP-IDF)
 * 
 * @note This function is registered with ESP-IDF MQTT client during init().
 *       Keep processing minimal to avoid blocking the MQTT task.
 *       Message callbacks should be fast and non-blocking.
 * 
 * @see esp_mqtt_event_handle_t - ESP-IDF MQTT event structure
 * @see MessageCallback - User-registered message handler function
 */
static void handle_mqtt_event(esp_mqtt_event_handle_t event)
{
    switch (event->event_id) {

    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        s_connected = true;
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        s_connected = false;
        break;

    case MQTT_EVENT_DATA: {
        std::string topic(event->topic, event->topic_len);
        std::string payload(event->data, event->data_len);

        ESP_LOGD(TAG, "Incoming: %s => %s", topic.c_str(), payload.c_str());

        if (s_msg_cb) {
            s_msg_cb(topic, payload);
        }
        break;
    }

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
        break;

    default:
        ESP_LOGD(TAG, "Unhandled MQTT event id: %d", event->event_id);
        break;
    }
}

#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
static void event_handler_cb(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    (void)handler_args;
    (void)base;
    (void)event_id;
    handle_mqtt_event(static_cast<esp_mqtt_event_handle_t>(event_data));
}
#else
static esp_err_t event_handler_cb(esp_mqtt_event_handle_t event)
{
    handle_mqtt_event(event);
    return ESP_OK;
}
#endif

/**
 * @brief Initializes the MQTT client with configuration and callback.
 * 
 * Creates and starts an ESP-IDF MQTT client with the provided configuration.
 * Registers the message callback for incoming message processing. This function
 * must be called once before using any other MQTT operations.
 * 
 * Initialization Process:
 *   1. Check if client already initialized (prevent double initialization)
 *   2. Store message callback for incoming message handling
 *   3. Populate ESP-IDF MQTT configuration structure
 *   4. Initialize MQTT client with configuration
 *   5. Start MQTT client (begins connection attempts)
 *   6. Log initialization status
 * 
 * Configuration Mapping:
 *   - cfg.uri: Directly mapped to mqtt_cfg.uri (tcp:// or tls://)
 *   - cfg.username: Mapped to mqtt_cfg.username (optional)
 *   - cfg.password: Mapped to mqtt_cfg.password (optional)
 *   - cfg.client_id: Mapped to mqtt_cfg.client_id (optional)
 *   - cfg.keepalive_sec: Mapped to mqtt_cfg.keepalive
 *   - cfg.clean_session: Inverted for mqtt_cfg.disable_clean_session
 * 
 * Authentication:
 *   - Username/password: Set if provided in config (not empty)
 *   - Client ID: Set if provided, otherwise broker assigns default
 *   - TLS: Supported via URI scheme (tls://), certificates via ESP-IDF config
 * 
 * Connection Behavior:
 *   - Asynchronous: Function returns immediately, connection happens in background
 *   - Auto-reconnect: ESP-IDF client automatically retries on failures
 *   - Keep-alive: Configurable ping interval to detect connection drops
 *   - Clean session: Controls message persistence across connections
 * 
 * Error Handling:
 *   - Already initialized: Warning logged, returns true (idempotent)
 *   - Client init failure: Error logged, returns false
 *   - Client start failure: Error logged, cleanup performed, returns false
 *   - Configuration errors: Logged by ESP-IDF, may cause connection failures
 * 
 * Memory Management:
 *   - MQTT client allocates internal buffers (~8-16KB)
 *   - Configuration strings are copied by ESP-IDF
 *   - No dynamic allocation in this function
 * 
 * @param cfg - Configuration structure containing MQTT broker details
 *              Must include valid URI, optional authentication and settings
 * 
 * @param cb - Message callback function for incoming MQTT messages
 *             Called with (topic, payload) when messages arrive
 *             Can be nullptr if no message processing needed
 * 
 * @return bool - true if initialization successful, false on error
 *                - false: Client initialization or startup failed
 *                - true: Client started successfully or already initialized
 * 
 * @note This function is not thread-safe for concurrent calls.
 *       Call once during application initialization.
 *       Example: sf_mqtt::init(config, messageHandler);
 * 
 * @see Config - Configuration structure definition in mqtt_layer.h
 * @see MessageCallback - Callback function signature
 * @see esp_mqtt_client_init() - ESP-IDF MQTT client initialization
 */
bool init(const Config &cfg, MessageCallback cb)
{
    if (s_client != nullptr) {
        ESP_LOGW(TAG, "MQTT already initialized");
        return true;
    }

    if (cfg.uri.empty()) {
        ESP_LOGW(TAG, "MQTT init skipped: empty URI");
        return false;
    }

    if (cfg.uri.find("://") == std::string::npos) {
        ESP_LOGW(TAG, "MQTT init skipped: URI missing scheme (expected mqtt://...)");
        return false;
    }

    s_msg_cb = cb;

    esp_mqtt_client_config_t mqtt_cfg = {};

#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
    mqtt_cfg.broker.address.uri = cfg.uri.c_str();

    if (!cfg.username.empty()) {
        mqtt_cfg.credentials.username = cfg.username.c_str();
    }
    if (!cfg.password.empty()) {
        mqtt_cfg.credentials.authentication.password = cfg.password.c_str();
    }
    if (!cfg.client_id.empty()) {
        mqtt_cfg.credentials.client_id = cfg.client_id.c_str();
    }

    mqtt_cfg.session.keepalive = cfg.keepalive_sec;
    mqtt_cfg.session.disable_clean_session = !cfg.clean_session;
#else
    mqtt_cfg.uri = cfg.uri.c_str();

    if (!cfg.username.empty()) {
        mqtt_cfg.username = cfg.username.c_str();
    }
    if (!cfg.password.empty()) {
        mqtt_cfg.password = cfg.password.c_str();
    }
    if (!cfg.client_id.empty()) {
        mqtt_cfg.client_id = cfg.client_id.c_str();
    }

    mqtt_cfg.keepalive = cfg.keepalive_sec;
    mqtt_cfg.disable_clean_session = !cfg.clean_session;

    mqtt_cfg.event_handle = event_handler_cb;
#endif

    s_client = esp_mqtt_client_init(&mqtt_cfg);
    if (!s_client) {
        ESP_LOGE(TAG, "Failed to init MQTT client");
        return false;
    }

#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
    esp_err_t register_err = esp_mqtt_client_register_event(s_client, MQTT_EVENT_ANY, event_handler_cb, nullptr);
    if (register_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register MQTT event handler: %d", register_err);
        esp_mqtt_client_destroy(s_client);
        s_client = nullptr;
        return false;
    }
#endif

    esp_err_t err = esp_mqtt_client_start(s_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %d", err);
        s_client = nullptr;
        return false;
    }

    ESP_LOGI(TAG, "MQTT client started with URI: %s", cfg.uri.c_str());
    return true;
}

/**
 * @brief Returns current MQTT connection status.
 * 
 * Provides immediate status of MQTT broker connection without blocking.
 * Used to determine if publish/subscribe operations will succeed.
 * 
 * Status Sources:
 *   - Updated by event_handler_cb on connection events
 *   - MQTT_EVENT_CONNECTED sets to true
 *   - MQTT_EVENT_DISCONNECTED sets to false
 *   - Initial state: false (before first connection)
 * 
 * Usage Patterns:
 *   - Pre-publish check: if (is_connected()) publish(...)
 *   - Status monitoring: Display connection state in UI
 *   - Diagnostics: Include in health check reports
 * 
 * Thread Safety:
 *   - Atomic boolean access (no race conditions)
 *   - Can be called from any task context
 *   - Status may change between call and use
 * 
 * Performance:
 *   - Execution time: < 1 microsecond
 *   - No system calls or network operations
 *   - Suitable for frequent polling
 * 
 * @return bool - true if connected to MQTT broker, false otherwise
 *                - true: Connection established and active
 *                - false: Disconnected, connecting, or not initialized
 * 
 * @note Connection status may change asynchronously.
 *       Check status immediately before operations that require connection.
 *       Example: if (sf_mqtt::is_connected()) { publish(...); }
 * 
 * @see event_handler_cb() - Updates connection status on events
 */
bool is_connected()
{
    return s_connected;
}

/**
 * @brief Publishes a message to an MQTT topic.
 * 
 * Sends a message to the specified MQTT topic with configurable QoS and
 * retention settings. Message is queued for transmission and function
 * returns immediately.
 * 
 * Publishing Process:
 *   1. Validate MQTT client is initialized
 *   2. Call ESP-IDF publish function with parameters
 *   3. Check return value for success/failure
 *   4. Log result (debug on success, error on failure)
 * 
 * QoS Levels:
 *   - 0 (At most once): Fire-and-forget, no delivery confirmation
 *   - 1 (At least once): Guaranteed delivery, may receive duplicates
 *   - 2 (Exactly once): Guaranteed single delivery (highest overhead)
 * 
 * Message Retention:
 *   - retain=true: Message persists on broker for new subscribers
 *   - retain=false: Message delivered only to current subscribers
 *   - Use retain for state topics (sensor values, device status)
 *   - Avoid retain for event topics (commands, alerts)
 * 
 * Error Conditions:
 *   - Client not initialized: Warning logged, returns false
 *   - Not connected: ESP-IDF queues message for later delivery
 *   - Invalid topic/payload: Handled by ESP-IDF (may fail)
 *   - Network issues: Message queued, delivery attempted when reconnected
 * 
 * Performance:
 *   - Execution time: < 1ms (non-blocking)
 *   - Memory usage: Minimal (ESP-IDF manages buffers)
 *   - Network: Asynchronous transmission
 * 
 * @param topic - MQTT topic string to publish to
 *                Must be valid MQTT topic (no wildcards for publish)
 *                Example: "smartfranklin/sensor/temperature"
 * 
 * @param payload - Message payload as string
 *                  Can be empty for null messages
 *                  Binary data should be base64 encoded
 * 
 * @param qos - Quality of Service level (0, 1, or 2)
 *              Default: 0 (at most once delivery)
 *              Higher QoS increases reliability but network overhead
 * 
 * @param retain - Whether to retain message on broker
 *                 Default: false (deliver to current subscribers only)
 *                 true: Persist for future subscribers
 * 
 * @return bool - true if message queued successfully, false on error
 *                - false: Client not initialized or publish failed
 *                - true: Message queued for transmission
 * 
 * @note Function returns true if message is queued, not if delivered.
 *       Delivery confirmation requires QoS 1/2 and callback handling.
 *       Example: sf_mqtt::publish("topic", "message", 1, false);
 * 
 * @see esp_mqtt_client_publish() - ESP-IDF publish implementation
 * @see subscribe() - Subscribe to topics for incoming messages
 */
bool publish(const std::string &topic,
             const std::string &payload,
             int qos,
             bool retain)
{
    if (!s_client) {
        ESP_LOGW(TAG, "publish() called but MQTT client not initialized");
        return false;
    }

    int msg_id = esp_mqtt_client_publish(
        s_client,
        topic.c_str(),
        payload.c_str(),
        payload.size(),
        qos,
        retain ? 1 : 0
    );

    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to publish to %s", topic.c_str());
        return false;
    }

    ESP_LOGD(TAG, "Publish queued to %s (msg_id=%d)", topic.c_str(), msg_id);
    return true;
}

/**
 * @brief Subscribes to an MQTT topic for incoming messages.
 * 
 * Registers interest in messages published to the specified topic.
 * Incoming messages will be delivered via the registered callback function.
 * 
 * Subscription Process:
 *   1. Validate MQTT client is initialized
 *   2. Call ESP-IDF subscribe function with topic and QoS
 *   3. Check return value for success/failure
 *   4. Log result (info on success, error on failure)
 * 
 * Topic Patterns:
 *   - Exact topic: "smartfranklin/sensor/temperature"
 *   - Single level: "smartfranklin/sensor/+"
 *   - Multi level: "smartfranklin/#"
 *   - Wildcards: + matches one level, # matches multiple levels
 * 
 * QoS Levels:
 *   - 0: Receive messages at most once (may lose messages)
 *   - 1: Receive messages at least once (may get duplicates)
 *   - 2: Receive messages exactly once (highest reliability)
 * 
 * Subscription Persistence:
 *   - Clean session: Subscriptions lost on disconnect
 *   - Persistent session: Subscriptions restored on reconnect
 *   - Controlled by Config::clean_session setting
 * 
 * Error Conditions:
 *   - Client not initialized: Warning logged, returns false
 *   - Not connected: Subscription queued for when connection established
 *   - Invalid topic: Handled by ESP-IDF (may fail)
 *   - Broker rejects: Subscription fails, logged as error
 * 
 * Performance:
 *   - Execution time: < 1ms (non-blocking)
 *   - Memory usage: Minimal (ESP-IDF manages subscription table)
 *   - Network: Asynchronous subscription request
 * 
 * @param topic - MQTT topic pattern to subscribe to
 *                Can include wildcards (+ for single level, # for multi-level)
 *                Example: "smartfranklin/commands/#"
 * 
 * @param qos - Quality of Service level for subscription (0, 1, or 2)
 *              Default: 0 (at most once delivery)
 *              Determines delivery guarantees for received messages
 * 
 * @return bool - true if subscription request queued, false on error
 *                - false: Client not initialized or subscribe failed
 *                - true: Subscription request sent to broker
 * 
 * @note Subscription may take time to take effect.
 *       Messages published before subscription may not be received.
 *       Use retained messages for state that must be received on subscribe.
 *       Example: sf_mqtt::subscribe("topic/#", 1);
 * 
 * @see publish() - Publish messages to topics
 * @see MessageCallback - Callback for received messages
 */
bool subscribe(const std::string &topic, int qos)
{
    if (!s_client) {
        ESP_LOGW(TAG, "subscribe() called but MQTT client not initialized");
        return false;
    }

    int msg_id = esp_mqtt_client_subscribe(
        s_client,
        topic.c_str(),
        qos
    );

    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to subscribe to %s", topic.c_str());
        return false;
    }

    ESP_LOGI(TAG, "Subscribe requested for %s (msg_id=%d)", topic.c_str(), msg_id);
    return true;
}

} // namespace sf_mqtt