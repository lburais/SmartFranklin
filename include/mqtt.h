/**
 * ============================================================================
 * MQTT Interface Module - SmartFranklin
 * ============================================================================
 *
 * File:        mqtt.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Public MQTT API used by SmartFranklin runtime modules.
 *
 * Author:      Laurent Burais
 * Date:        12 March 2026
 * Version:     1.1
 *
 * Overview:
 *   This API exposes two MQTT paths: an optional ESP-IDF MQTT client wrapper
 *   for direct broker connectivity, and helpers for the embedded local broker
 *   started by `taskMqtt`.
 *
 * Integration Notes:
 *   Most firmware modules call `publish(...)` and let the implementation route
 *   to the external client when present or to the local broker otherwise.
 *   Startup health checks can query `is_local_broker_ready()` directly.
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

#pragma once

#include <string>
#include <functional>

extern "C" {
#include "mqtt_client.h"
}

namespace sf_mqtt {

/**
 * @brief Callback type for MQTT message reception.
 *
 * Receives topic and payload as `std::string` values when the optional
 * ESP-IDF MQTT client dispatches inbound messages.
 *
 * @see init() registers this callback for client-delivered messages.
 */
using MessageCallback = std::function<void(const std::string &topic,
                                           const std::string &payload)>;

/**
 * @brief Configuration structure for MQTT client setup.
 * 
 * Contains all parameters needed to establish MQTT broker connection.
 * Provides sensible defaults for optional parameters while allowing
 * full customization of connection settings.
 */
struct Config {
    /**
     * @brief MQTT broker URI for connection.
     * 
     * Complete URI specifying broker location and protocol.
     * Supports various schemes for different connection types.
     * 
     * Format: scheme://host:port/path
     * Schemes:
     *   - mqtt:// - Plain TCP connection
     *   - mqtts:// - TLS encrypted TCP connection
     *   - ws:// - WebSocket connection
     *   - wss:// - WebSocket secure connection
     * 
     * Examples:
     *   - "mqtt://broker.hivemq.com"
     *   - "mqtt://192.168.1.100:1883"
     *   - "mqtts://secure-broker.com:8883"
     * 
     * Default: Empty (must be specified)
     * Required: Yes
     */
    std::string uri;

    /**
     * @brief Username for broker authentication.
     * 
     * Username credential for MQTT broker authentication.
     * Required if broker has authentication enabled.
     * 
     * Default: Empty (no authentication)
     * Required: No (depends on broker configuration)
     * Security: Stored in memory during connection
     */
    std::string username;

    /**
     * @brief Password for broker authentication.
     * 
     * Password credential for MQTT broker authentication.
     * Required if broker has authentication enabled.
     * 
     * Default: Empty (no authentication)
     * Required: No (depends on broker configuration)
     * Security: Stored in memory during connection
     */
    std::string password;

    /**
     * @brief Unique client identifier for MQTT session.
     * 
     * Client ID used to identify this client to the broker.
     * Must be unique across all clients connecting to the broker.
     * If empty, broker may assign a random ID.
     * 
     * Default: Empty (broker-assigned)
     * Required: No (but recommended for session persistence)
     * Format: 1-23 characters, alphanumeric and special chars
     * Persistence: Used for clean session management
     */
    std::string client_id;

    /**
     * @brief Clean session flag for connection behavior.
     * 
     * Controls whether to start a clean session or resume previous.
     * Affects subscription persistence and queued messages.
     * 
     * Behavior:
     *   - true: Clean session, no previous state restored
     *   - false: Resume previous session if client_id matches
     * 
     * Default: true (clean session)
     * Use Case: Set false for persistent subscriptions
     */
    bool        clean_session = true;

    /**
     * @brief Keep-alive interval in seconds.
     * 
     * Time interval for sending ping messages to maintain connection.
     * Broker disconnects client if no messages received within 1.5x interval.
     * 
     * Default: 60 seconds
     * Range: 1-65535 seconds
     * Performance: Lower values increase network traffic
     * Reliability: Higher values risk undetected disconnections
     */
    int         keepalive_sec = 60;

    /**
     * @brief TLS encryption enable flag.
     * 
     * Controls whether to use TLS encryption for the connection.
     * Automatically enabled for mqtts:// and wss:// URIs.
     * 
     * Default: false (plain connection)
     * Security: Enables end-to-end encryption
     * Performance: Slight overhead for encryption/decryption
     * Certificates: Requires proper certificate configuration
     */
    bool        use_tls       = false;
};

/**
 * @brief Initializes the MQTT client with configuration and callback.
 * 
 * Sets up the MQTT client with provided configuration and establishes
 * connection to the broker. Registers message callback for incoming
 * messages. Must be called before using other MQTT functions.
 * 
 * @param cfg - Configuration structure with broker settings
 * @param cb - Callback function for received messages
 * @return true if initialization successful, false on error
 * 
 * Initialization Process:
 * 
 * Configuration Validation:
 * 
 * Error Handling:
 * 
 * Performance:
 * 
 * Usage Notes:
 * 
 * @note Blocking operation during connection establishment.
 * 
 * @see Config - Configuration structure
 * @see MessageCallback - Callback function type
 * @see is_connected() - Check connection status
 */
bool init(const Config &cfg, MessageCallback cb);

/**
 * @brief Checks current MQTT broker connection status.
 * 
 * Returns the current connection state of the MQTT client.
 * Useful for monitoring connection health and triggering
 * reconnection logic if needed.
 * 
 * @return true if connected to broker, false if disconnected
 * 
 * Connection States:
 * 
 * Status Sources:
 * 
 * Usage Patterns:
 * 
 * Performance:
 * 
 * @note Function checks current state, doesn't initiate connections.
 * 
 * @see init() - Establish initial connection
 * @see publish() - Operations requiring connection
 */
bool is_connected();

/**
 * @brief Publishes a message directly to the embedded local MQTT broker.
 *
 * This path is served by the unified MQTT task (`taskMqtt`) and is intended
 * as a fallback when the external ESP-IDF MQTT client is not initialized.
 *
 * @param topic MQTT topic string for local publication
 * @param payload Message payload
 * @param qos QoS level (0..2)
 * @param retain Retain flag
 * @return true if queued for local broker clients, false if the local broker
 *         is not ready
 */
bool publish_local(const std::string &topic,
                   const std::string &payload,
                   int qos = 1,
                   bool retain = false);

/**
 * @brief Returns whether the embedded local MQTT broker is started.
 *
 * This reflects readiness of the local broker managed by `taskMqtt`.
 * It can be used during startup health checks before publishing locally.
 */
bool is_local_broker_ready();

/**
 * @brief Publishes a message to an MQTT topic.
 * 
 * Sends a message to the specified MQTT topic with given QoS and retain settings.
 * Message is queued for delivery and function returns immediately.
 * 
 * @param topic - MQTT topic string for message publication
 * @param payload - Message payload as binary data string
 * @param qos - Quality of Service level (0, 1, or 2)
 * @param retain - Retain flag for message persistence
 * @return true if message queued successfully, false on error
 * 
 * QoS Levels:
 * 
 * Retain Behavior:
 * 
 * Topic Format:
 * 
 * Error Conditions:
 * 
 * Performance:
 * 
 * Usage Examples:
 * 
 * @note Function queues message for sending, doesn't wait for delivery.
 * 
 * @see subscribe() - Receive messages on topics
 * @see is_connected() - Check connection status
 */
bool publish(const std::string &topic,
             const std::string &payload,
             int qos = 1,
             bool retain = false);

/**
 * @brief Subscribes to an MQTT topic for message reception.
 * 
 * Registers interest in messages published to the specified topic.
 * Incoming messages will be delivered via the callback registered in init().
 * Supports MQTT wildcards for flexible topic filtering.
 * 
 * @param topic - MQTT topic pattern to subscribe to
 * @param qos - Quality of Service level for subscription (0, 1, or 2)
 * @return true if subscription successful, false on error
 * 
 * Topic Patterns:
 * 
 * QoS Levels:
 * 
 * Subscription Management:
 * 
 * Error Conditions:
 * 
 * Performance:
 * 
 * Usage Examples:
 * 
 * @note Requires active connection from init().
 * 
 * @see publish() - Send messages to topics
 * @see init() - Register message callback
 * @see MessageCallback - Message reception callback
 */
bool subscribe(const std::string &topic, int qos = 1);

} // namespace sf_mqtt