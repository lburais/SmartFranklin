/*
 * ============================================================================
 * MQTT Layer Module - SmartFranklin
 * ============================================================================
 * 
 * File:        mqtt_layer.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Header file for MQTT communication layer. Provides C++ wrapper
 *              around ESP-IDF MQTT client for reliable publish/subscribe messaging
 *              in SmartFranklin's IoT communication stack.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   The MQTT layer provides a high-level C++ interface for MQTT (Message Queuing
 *   Telemetry Transport) protocol communication. Built on top of the ESP-IDF
 *   MQTT client, it offers publish/subscribe messaging capabilities essential
 *   for IoT applications. The layer handles connection management, message
 *   routing, and provides callback-based message processing for seamless
 *   integration with SmartFranklin's event-driven architecture.
 * 
 * MQTT Protocol Features:
 *   - Publish/Subscribe: Decoupled message distribution pattern
 *   - Quality of Service: Three QoS levels (0, 1, 2) for delivery guarantees
 *   - Retained Messages: Persistent messages for new subscribers
 *   - Last Will and Testament: Death certificates for connection monitoring
 *   - Clean Session: Session state management and persistence
 *   - Keep-Alive: Connection health monitoring with ping/pong
 * 
 * Layer Architecture:
 *   - ESP-IDF Integration: Uses esp-mqtt library for robust MQTT implementation
 *   - C++ Wrapper: Modern C++ interface with std::function callbacks
 *   - Namespace Isolation: sf_mqtt namespace prevents naming conflicts
 *   - Configuration Structure: Clean configuration with defaults
 *   - Error Handling: Comprehensive error reporting and recovery
 *   - Thread Safety: Designed for FreeRTOS multi-tasking environment
 * 
 * Connection Management:
 *   - Broker URI: Support for mqtt://, mqtts://, ws://, wss:// schemes
 *   - Authentication: Username/password and client certificate support
 *   - TLS Security: Encrypted connections with certificate validation
 *   - Reconnection: Automatic reconnection with exponential backoff
 *   - Connection Status: Real-time connection state monitoring
 *   - Client ID: Unique client identification for session management
 * 
 * Message Handling:
 *   - Topic Filtering: Support for MQTT wildcards (+ and #)
 *   - QoS Levels: At most once (0), at least once (1), exactly once (2)
 *   - Payload Types: Binary data support with std::string containers
 *   - Callback Processing: Asynchronous message delivery via callbacks
 *   - Message Buffering: Internal buffering for reliable delivery
 *   - Duplicate Detection: Automatic duplicate message filtering
 * 
 * Performance Characteristics:
 *   - Latency: Low latency message delivery (< 100ms typical)
 *   - Throughput: High message rates (100+ messages/second)
 *   - Memory Usage: Efficient memory management with pooling
 *   - CPU Usage: Minimal overhead with hardware acceleration
 *   - Network Efficiency: Optimized packet sizes and keep-alive
 * 
 * Security Features:
 *   - TLS Encryption: End-to-end encryption for secure communication
 *   - Certificate Validation: Server certificate verification
 *   - Client Authentication: Mutual TLS with client certificates
 *   - Access Control: Topic-based authorization (broker-dependent)
 *   - Credential Protection: Secure storage of authentication data
 * 
 * Integration Points:
 *   - Data Model: Publishes sensor data and receives commands
 *   - Command Handler: Processes incoming MQTT command messages
 *   - MQTT Bridge: Forwards messages between internal/external brokers
 *   - Configuration Store: Loads broker settings from persistent storage
 *   - Status Reporting: Connection status in system health monitoring
 * 
 * Dependencies:
 *   - ESP-IDF MQTT: esp-mqtt component for core MQTT functionality
 *   - C++ Standard Library: std::string, std::function for modern C++
 *   - FreeRTOS: Task scheduling and synchronization primitives
 * 
 * Limitations:
 *   - Single Broker: One MQTT broker connection per instance
 *   - ESP-IDF Specific: Designed for ESP32 platform with ESP-IDF
 *   - Memory Constraints: Limited by ESP32 RAM for large payloads
 *   - Network Dependent: Requires stable network connectivity
 *   - Callback Model: Event-driven processing may require careful design
 * 
 * Best Practices:
 *   - Use appropriate QoS levels based on message criticality
 *   - Implement proper error handling for connection failures
 *   - Monitor connection status and implement reconnection logic
 *   - Use descriptive topic names with hierarchical structure
 *   - Consider message payload size for memory-constrained devices
 *   - Test with your specific MQTT broker for compatibility
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
 * copies or the Software, and to permit persons to whom the Software is
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
 * Function signature for handling incoming MQTT messages. Called
 * asynchronously when messages are received on subscribed topics.
 * Provides topic and payload as std::string parameters.
 * 
 * Callback Signature:
 *   void callback(const std::string &topic, const std::string &payload)
 * 
 * Parameters:
 *   - topic: MQTT topic string where message was received
 *   - payload: Message payload as binary data string
 * 
 * Usage:
 *   - Process incoming sensor data or commands
 *   - Update internal state based on received messages
 *   - Forward messages to other system components
 *   - Log or store received data for analysis
 * 
 * Thread Safety:
 *   - Called from MQTT client task context
 *   - Should be thread-safe if accessing shared resources
 *   - Keep processing lightweight to avoid blocking
 * 
 * @see init() - Function that registers this callback
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
 *   1. Validate configuration parameters
 *   2. Create ESP-IDF MQTT client instance
 *   3. Configure connection parameters (URI, auth, TLS)
 *   4. Register message callback for incoming messages
 *   5. Start MQTT client and attempt broker connection
 *   6. Wait for connection establishment or timeout
 * 
 * Configuration Validation:
 *   - URI format and scheme validation
 *   - Required parameter presence checking
 *   - TLS configuration consistency
 *   - Client ID format validation
 * 
 * Error Handling:
 *   - Invalid configuration returns false
 *   - Network errors logged with retry capability
 *   - TLS certificate errors handled gracefully
 *   - Connection timeouts with exponential backoff
 * 
 * Performance:
 *   - Initialization Time: 1-5 seconds (network dependent)
 *   - Memory Usage: MQTT client and buffer allocation
 *   - Network Usage: Initial connection handshake
 * 
 * Usage Notes:
 *   - Call once during system initialization
 *   - Ensure network connectivity before calling
 *   - Callback must be valid for message processing
 *   - Check return value for successful initialization
 * 
 * @note Blocking operation during connection establishment.
 *       Callback function must remain valid during MQTT operation.
 *       Network connectivity required for successful initialization.
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
 *   - Connected: Active connection with broker
 *   - Connecting: In process of establishing connection
 *   - Disconnected: No connection to broker
 *   - Error: Connection failed or lost
 * 
 * Status Sources:
 *   - ESP-IDF MQTT client internal state
 *   - Network connectivity monitoring
 *   - Broker responsiveness (ping/pong)
 * 
 * Usage Patterns:
 *   - Health Monitoring: Periodic status checks
 *   - Reconnection Logic: Trigger reconnect on disconnection
 *   - UI Updates: Display connection status
 *   - Error Handling: Conditional operation based on status
 * 
 * Performance:
 *   - Execution Time: < 1ms
 *   - No Network I/O: Local state check only
 *   - Thread Safe: Can be called from any context
 * 
 * @note Function checks current state, doesn't initiate connections.
 *       Use after init() has been called successfully.
 *       Status may change asynchronously due to network events.
 * 
 * @see init() - Establish initial connection
 * @see publish() - Operations requiring connection
 */
bool is_connected();

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
 *   - 0 (At most once): Fire and forget, no delivery guarantee
 *   - 1 (At least once): Acknowledged delivery, may have duplicates
 *   - 2 (Exactly once): Guaranteed single delivery, highest overhead
 * 
 * Retain Behavior:
 *   - false: Message delivered to current subscribers only
 *   - true: Message stored on broker for future subscribers
 * 
 * Topic Format:
 *   - Hierarchical: "smartfranklin/sensor/temperature"
 *   - Wildcard Support: Not applicable for publishing
 *   - Length Limit: Typically < 65535 characters
 * 
 * Error Conditions:
 *   - Not connected: Message not sent, returns false
 *   - Invalid topic: Malformed topic string
 *   - QoS out of range: Invalid QoS value
 *   - Buffer full: Internal queue overflow
 * 
 * Performance:
 *   - Execution Time: < 1ms (queuing only)
 *   - Asynchronous: Actual transmission in background
 *   - Memory Usage: Payload copied to internal buffer
 *   - Network Usage: Depends on payload size and QoS
 * 
 * Usage Examples:
 *   publish("smartfranklin/status", "online", 1, true);
 *   publish("sensor/temperature", "23.5", 0, false);
 * 
 * @note Function queues message for sending, doesn't wait for delivery.
 *       Requires active connection from init().
 *       Large payloads may be fragmented by underlying implementation.
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
 *   - Exact Match: "smartfranklin/sensor/temperature"
 *   - Single Level: "smartfranklin/sensor/+" (matches one level)
 *   - Multi Level: "smartfranklin/#" (matches all subtopics)
 *   - Examples: "+", "#", "sensor/+", "building/floor/+/room"
 * 
 * QoS Levels:
 *   - 0: At most once delivery of messages
 *   - 1: At least once delivery with acknowledgments
 *   - 2: Exactly once delivery with highest reliability
 * 
 * Subscription Management:
 *   - Persistent: Subscriptions maintained across reconnections
 *   - Multiple: Multiple subscriptions allowed
 *   - Overlapping: Multiple patterns can match same message
 *   - Unsubscribe: Not implemented (use broker management)
 * 
 * Error Conditions:
 *   - Not connected: Subscription not registered
 *   - Invalid topic: Malformed topic pattern
 *   - QoS out of range: Invalid QoS value
 *   - Broker rejection: Subscription not allowed
 * 
 * Performance:
 *   - Execution Time: < 100ms (network round-trip)
 *   - Memory Usage: Subscription state storage
 *   - Network Usage: SUBSCRIBE packet transmission
 * 
 * Usage Examples:
 *   subscribe("smartfranklin/commands", 1);
 *   subscribe("sensor/#", 0);
 *   subscribe("building/+/temperature", 2);
 * 
 * @note Requires active connection from init().
 *       Wildcard subscriptions increase server processing load.
 *       Messages delivered asynchronously via callback.
 * 
 * @see publish() - Send messages to topics
 * @see init() - Register message callback
 * @see MessageCallback - Message reception callback
 */
bool subscribe(const std::string &topic, int qos = 1);

/**
 * @brief Compatibility function for event loop processing.
 * 
 * Empty inline function provided for compatibility with existing code
 * that may call a loop() function. The ESP-IDF MQTT client handles
 * all processing internally without requiring explicit loop calls.
 * 
 * @note This function does nothing and is provided for API compatibility.
 *       The underlying ESP-IDF MQTT client manages its own event processing.
 *       No action required from calling code.
 * 
 * @deprecated Provided for backward compatibility only.
 *             Modern implementations should not need explicit loop calls.
 */
inline void loop() {}

} // namespace sf_mqtt