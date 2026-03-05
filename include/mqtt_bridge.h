/*
 * ============================================================================
 * MQTT Bridge Module - SmartFranklin
 * ============================================================================
 * 
 * File:        mqtt_bridge.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Header file for MQTT message bridging functionality. Provides
 *              bidirectional message forwarding between internal and external
 *              MQTT brokers, enabling seamless integration with cloud services
 *              and remote monitoring systems.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   The MQTT bridge module enables SmartFranklin to connect to external MQTT
 *   brokers while maintaining local MQTT functionality. It provides transparent
 *   message forwarding between internal (local) and external (cloud) MQTT
 *   networks, allowing seamless integration with IoT platforms, cloud services,
 *   and remote monitoring systems. The bridge supports topic mapping, QoS
 *   preservation, and loop detection to prevent message storms.
 * 
 * MQTT Bridge Architecture:
 *   - Internal Broker: Local MQTT broker running on SmartFranklin
 *   - External Broker: Cloud or remote MQTT broker (AWS IoT, HiveMQ, etc.)
 *   - Message Flow: Bidirectional forwarding with topic transformation
 *   - Connection Management: Independent connections to both brokers
 *   - State Tracking: Message deduplication and loop prevention
 *   - Configuration: Flexible topic mapping and filtering rules
 * 
 * Bridge Functionality:
 *   - Topic Mapping: Transform topic names between internal/external
 *   - QoS Preservation: Maintain message quality of service levels
 *   - Retain Flag: Preserve retained message status
 *   - Loop Detection: Prevent infinite forwarding loops
 *   - Filtering: Selective message forwarding based on topics
 *   - Status Monitoring: Bridge health and statistics reporting
 * 
 * Topic Mapping Strategy:
 *   - Internal Topics: "smartfranklin/sensor/temperature" (local)
 *   - External Topics: "devices/smartfranklin/sensor/temperature" (cloud)
 *   - Prefix Addition: Add configurable prefixes for namespace isolation
 *   - Bidirectional: Support both directions with different mappings
 *   - Wildcards: Support MQTT wildcards for flexible routing
 * 
 * Connection Management:
 *   - Dual Connections: Separate clients for internal and external brokers
 *   - Reconnection: Automatic reconnection on connection failures
 *   - Authentication: Support username/password for both brokers
 *   - TLS Support: Secure connections to external brokers
 *   - Keep-Alive: Maintain connections with periodic pings
 * 
 * Message Processing:
 *   - Subscription: Subscribe to configured topic patterns
 *   - Forwarding: Relay messages between brokers
 *   - Transformation: Apply topic mapping and filtering
 *   - Deduplication: Prevent duplicate message forwarding
 *   - Error Handling: Graceful failure on forwarding errors
 * 
 * Performance Considerations:
 *   - Throughput: Limited by network bandwidth and broker capacity
 *   - Latency: Additional hop through bridge increases latency
 *   - Memory Usage: Message buffering for reliable delivery
 *   - CPU Load: JSON processing and topic transformation
 *   - Network Usage: Dual connections increase bandwidth usage
 * 
 * Security Features:
 *   - Authentication: Secure connections to external brokers
 *   - Encryption: TLS/SSL support for external connections
 *   - Access Control: Topic-based filtering and authorization
 *   - Loop Prevention: Detect and block circular message routing
 *   - Audit Logging: Message forwarding activity logging
 * 
 * Integration Points:
 *   - Configuration Store: Bridge settings from CONFIG structure
 *   - MQTT Layer: Internal broker connection and messaging
 *   - Status Reporting: Bridge status in system health monitoring
 *   - Error Handling: Integration with system error reporting
 * 
 * Dependencies:
 *   - ArduinoJson.h: JSON document handling for status reporting
 *   - MQTT Library: PubSubClient or similar for broker connections
 *   - Configuration: Bridge settings from config_store.h
 * 
 * Limitations:
 *   - Single External Broker: One external broker connection
 *   - Topic Mapping: Static prefix-based mapping only
 *   - No Message Transformation: Payload unchanged during forwarding
 *   - Memory Constraints: Limited buffering on resource-constrained devices
 *   - Network Dependent: Requires stable connectivity for reliable operation
 * 
 * Best Practices:
 *   - Use descriptive topic prefixes for clear namespace separation
 *   - Monitor bridge status and connection health
 *   - Implement proper error handling for connection failures
 *   - Test topic mappings thoroughly before deployment
 *   - Consider message volume and bandwidth implications
 *   - Use QoS 1 for critical messages requiring acknowledgment
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

#pragma once
#include <ArduinoJson.h>

/**
 * @brief Initializes the MQTT bridge system.
 * 
 * Sets up connections to both internal and external MQTT brokers and
 * configures message forwarding rules. Establishes subscriptions and
 * prepares the bridge for bidirectional message forwarding.
 * 
 * Initialization Sequence:
 *   1. Load bridge configuration from CONFIG structure
 *   2. Connect to internal MQTT broker (local)
 *   3. Connect to external MQTT broker (cloud/remote)
 *   4. Set up topic subscriptions for message forwarding
 *   5. Configure topic mapping rules and filters
 *   6. Initialize message deduplication and loop detection
 *   7. Start bridge monitoring and statistics collection
 * 
 * Configuration Requirements:
 *   - Internal broker: Assumes local MQTT service is running
 *   - External broker: Host, port, credentials from CONFIG
 *   - Topic prefixes: Internal and external prefix strings
 *   - Bridge enabled: CONFIG.mqtt_bridge_enabled must be true
 * 
 * Connection Management:
 *   - Internal: Connect to localhost or configured internal broker
 *   - External: Connect using CONFIG.ext_mqtt_* parameters
 *   - Authentication: Support username/password for both
 *   - TLS: Optional secure connection for external broker
 * 
 * Error Handling:
 *   - Connection failures logged with retry logic
 *   - Missing configuration handled gracefully
 *   - Bridge disabled if configuration invalid
 *   - Partial initialization allows limited functionality
 * 
 * Performance:
 *   - Initialization Time: 2-10 seconds (network dependent)
 *   - Memory Usage: MQTT client instances and buffers
 *   - Network Connections: Two simultaneous MQTT connections
 * 
 * Usage Notes:
 *   - Call after MQTT services are initialized
 *   - Ensure CONFIG is loaded before calling
 *   - Bridge can be re-initialized after configuration changes
 *   - Monitor logs for connection status and errors
 * 
 * @note Requires valid MQTT broker configurations in CONFIG.
 *       Bridge functionality is disabled if CONFIG.mqtt_bridge_enabled is false.
 *       Both internal and external brokers must be accessible.
 * 
 * @see mqtt_bridge_loop() - Main bridge processing function
 * @see mqtt_bridge_status() - Bridge status reporting
 * @see CONFIG - Configuration structure with bridge settings
 */
void mqtt_bridge_init();

/**
 * @brief Main processing loop for MQTT bridge message forwarding.
 * 
 * Processes incoming messages from both brokers and forwards them
 * according to configured rules. Handles connection maintenance,
 * message deduplication, and error recovery. Should be called
 * regularly from the main program loop or dedicated task.
 * 
 * Processing Cycle:
 *   1. Check and maintain broker connections
 *   2. Process incoming messages from internal broker
 *   3. Process incoming messages from external broker
 *   4. Apply topic mapping and filtering rules
 *   5. Forward messages to appropriate destination broker
 *   6. Update bridge statistics and health monitoring
 *   7. Handle connection failures and reconnection logic
 * 
 * Message Forwarding:
 *   - Direction: Internal→External and External→Internal
 *   - Topic Mapping: Apply configured prefix transformations
 *   - QoS Preservation: Maintain original QoS levels
 *   - Retain Handling: Preserve retain flags appropriately
 *   - Loop Detection: Prevent circular message routing
 * 
 * Connection Monitoring:
 *   - Health Checks: Periodic connection validation
 *   - Reconnection: Automatic reconnection on failures
 *   - Timeout Handling: Configurable connection timeouts
 *   - Error Recovery: Graceful degradation on issues
 * 
 * Performance:
 *   - Execution Time: Variable (depends on message volume)
 *   - CPU Usage: Low when idle, higher during message processing
 *   - Memory Usage: Message buffers and processing overhead
 *   - Network Usage: Bidirectional message forwarding
 * 
 * Error Handling:
 *   - Connection Loss: Automatic reconnection attempts
 *   - Message Loss: Logging and retry mechanisms
 *   - Invalid Messages: Filtering and error logging
 *   - Resource Limits: Buffer management and overflow handling
 * 
 * Usage Pattern:
 *   void loop() {
 *       mqtt_bridge_loop();
 *       // Other processing...
 *   }
 * 
 * Integration:
 *   - Task Scheduling: Call from FreeRTOS task or main loop
 *   - Frequency: Call as frequently as possible for responsiveness
 *   - Blocking: Non-blocking operation with internal timeouts
 *   - Thread Safety: Designed for single-threaded operation
 * 
 * @note Function should be called regularly (every loop iteration).
 *       Handles all bridge processing asynchronously.
 *       Connection issues are handled internally with logging.
 * 
 * @see mqtt_bridge_init() - Bridge initialization
 * @see mqtt_bridge_status() - Status monitoring
 */
void mqtt_bridge_loop();

/**
 * @brief Retrieves current MQTT bridge status and statistics.
 * 
 * Populates a JSON document with comprehensive bridge status information
 * including connection states, message counters, and performance metrics.
 * Used for monitoring, debugging, and system health reporting.
 * 
 * @param doc - ArduinoJson document to populate with status data
 * 
 * Status Information:
 *   - Connection Status: Internal and external broker connections
 *   - Message Counters: Messages forwarded in both directions
 *   - Error Counters: Connection failures and message errors
 *   - Performance Metrics: Throughput and latency statistics
 *   - Configuration: Current bridge settings and mappings
 *   - Health Indicators: Overall bridge operational status
 * 
 * JSON Structure:
 *   {
 *     "bridge_enabled": true,
 *     "internal_connected": true,
 *     "external_connected": true,
 *     "messages_forwarded_internal_to_external": 1234,
 *     "messages_forwarded_external_to_internal": 567,
 *     "connection_errors": 2,
 *     "last_message_time": "2026-03-05T14:30:25Z",
 *     "topic_mappings": {
 *       "internal_prefix": "smartfranklin/",
 *       "external_prefix": "cloud/smartfranklin/"
 *     }
 *   }
 * 
 * Data Collection:
 *   - Real-time: Current connection and operational status
 *   - Counters: Persistent counters since initialization
 *   - Timestamps: Last activity and error timestamps
 *   - Configuration: Current active settings
 * 
 * Performance:
 *   - Execution Time: < 10ms (JSON serialization)
 *   - Memory Usage: ArduinoJson document size
 *   - Thread Safety: Safe for concurrent access
 *   - No Blocking: Synchronous status retrieval
 * 
 * Usage Pattern:
 *   JsonDocument doc;
 *   mqtt_bridge_status(doc);
 *   serializeJson(doc, Serial);
 * 
 * Integration:
 *   - MQTT Publishing: Status published to monitoring topics
 *   - Web Interface: Status displayed in captive portal
 *   - Logging: Status included in system health logs
 *   - API Endpoints: Status exposed through REST APIs
 * 
 * Error Handling:
 *   - JSON Errors: Graceful handling of serialization failures
 *   - Missing Data: Default values for unavailable information
 *   - Memory Limits: Respect ArduinoJson document size limits
 * 
 * @note Function populates the provided JsonDocument with status data.
 *       Ensure document has sufficient capacity for status information.
 *       Status data is read-only and doesn't affect bridge operation.
 * 
 * @see mqtt_bridge_init() - Bridge initialization
 * @see mqtt_bridge_loop() - Bridge processing
 * @see ArduinoJson - JSON library documentation
 */
void mqtt_bridge_status(JsonDocument &doc);