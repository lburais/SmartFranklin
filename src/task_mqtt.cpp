/*
 * ============================================================================
 * MQTT Task Module - SmartFranklin
 * ============================================================================
 * 
 * File:        task_mqtt.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: FreeRTOS task for unified MQTT processing (local broker + external
 *              client). Handles broker servicing, connection maintenance,
 *              and incoming message dispatching to registered callbacks.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   SmartFranklin communicates with cloud services and other IoT devices
 *   through MQTT (Message Queuing Telemetry Transport) protocol. This task
 *   maintains the persistent connection to the MQTT broker, processes
 *   incoming messages, and ensures reliable message delivery. It serves
 *   as the central communication hub for all MQTT-based functionality.
 * 
 * MQTT Protocol Features:
 *   - Publish/Subscribe: Asynchronous message passing
 *   - Quality of Service: QoS 0 (at most once), QoS 1 (at least once), QoS 2 (exactly once)
 *   - Retained Messages: Persistent messages for new subscribers
 *   - Last Will and Testament: Connection status notification
 *   - Clean Session: Session state management on reconnect
 * 
 * Task Responsibilities:
 *   - Connection Maintenance: Keep MQTT broker connection alive
 *   - Message Processing: Handle incoming messages and route to callbacks
 *   - Subscription Management: Maintain topic subscriptions
 *   - Reconnection Logic: Automatic reconnection on connection loss
 *   - Keep-Alive: Send periodic ping messages to maintain connection
 *   - Error Handling: Graceful handling of connection failures
 * 
 * Connection Management:
 *   - Broker Address: CONFIG.mqtt_broker (IP address or hostname)
 *   - Port: CONFIG.mqtt_port (typically 1883 for non-secure, 8883 for TLS)
 *   - Client ID: CONFIG.mqtt_client_id (unique identifier)
 *   - Authentication: CONFIG.mqtt_user and CONFIG.mqtt_pass
 *   - TLS/SSL: Optional secure connection support
 *   - Keep-Alive Interval: CONFIG.mqtt_keepalive seconds
 * 
 * Message Processing:
 *   - Incoming Messages: Routed to registered callback functions
 *   - Topic Filtering: Callbacks receive only subscribed topics
 *   - Payload Handling: Raw byte arrays or string conversion
 *   - QoS Acknowledgment: Automatic handling of QoS levels
 *   - Duplicate Detection: MQTT library handles duplicate messages
 * 
 * Subscription Management:
 *   - Dynamic Subscriptions: Topics can be added/removed at runtime
 *   - Wildcard Support: + (single level) and # (multi-level) wildcards
 *   - QoS per Subscription: Different QoS levels for different topics
 *   - Unsubscription: Clean removal of topic subscriptions
 * 
 * Error Handling:
 *   - Connection Failures: Automatic retry with exponential backoff
 *   - Network Issues: Graceful degradation during connectivity loss
 *   - Broker Unavailable: Logging and continued retry attempts
 *   - Authentication Failures: Configuration validation and logging
 *   - Message Processing Errors: Individual message failure handling
 * 
 * Performance Considerations:
 *   - CPU Usage: Low (mostly waiting for network events)
 *   - Memory Usage: Minimal (MQTT library overhead)
 *   - Network Bandwidth: Efficient binary protocol
 *   - Task Priority: Standard priority for reliable communication
 *   - Loop Frequency: 10Hz (100ms) for responsive message processing
 * 
 * Dependencies:
 *   - Arduino.h (FreeRTOS task functions)
 *   - M5Unified.h (M5Stack logging utilities)
 *   - M5Utility.h (M5Stack utility functions)
 *   - tasks.h (Task definitions and priorities)
 *   - mqtt.h (MQTT abstraction layer with sf_mqtt namespace)
 * 
 * Configuration Integration:
 *   - CONFIG.mqtt_broker: Broker hostname/IP address
 *   - CONFIG.mqtt_port: Connection port number
 *   - CONFIG.mqtt_client_id: Unique client identifier
 *   - CONFIG.mqtt_user: Authentication username
 *   - CONFIG.mqtt_pass: Authentication password
 *   - CONFIG.mqtt_keepalive: Keep-alive interval in seconds
 * 
 * Limitations:
 *   - Single Broker Connection: No multi-broker failover support
 *   - Fixed Loop Rate: 10Hz processing (not configurable)
 *   - No Message Queuing: Real-time processing only
 *   - Memory Constraints: Limited by ESP32 RAM for large payloads
 *   - No Offline Buffering: Messages lost during disconnection
 *   - Synchronous Processing: Blocking during message callbacks
 * 
 * Best Practices:
 *   - Use descriptive topic hierarchies (e.g., smartfranklin/sensor/temperature)
 *   - Implement QoS 1 for critical messages requiring delivery confirmation
 *   - Use retained messages for status information
 *   - Configure appropriate keep-alive intervals (30-60 seconds typical)
 *   - Monitor connection status for system health indicators
 *   - Validate topic names and payload formats in callbacks
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

#include <Arduino.h>
#include <M5Unified.h>
#include <M5Utility.h>
#include <sMQTTBroker.h>

#include "tasks.h"
#include "mqtt.h"
#include "config_store.h"

namespace {

sMQTTBroker             s_localBroker;
bool                    s_localBrokerStarted = false;

uint32_t mqttLoopMs()
{
    const int loopMs = CONFIG.task_mqtt_loop_ms;
    return (loopMs > 0) ? static_cast<uint32_t>(loopMs) : 20UL;
}

unsigned short localBrokerPort()
{
    const int cfgPort = CONFIG.ext_mqtt_port;
    if (cfgPort <= 0 || cfgPort > 65535) {
        return 1883;
    }
    return static_cast<unsigned short>(cfgPort);
}

void ensureLocalBrokerStarted()
{
    static uint32_t nextRetryAtMs = 0;

    if (s_localBrokerStarted) {
        return;
    }

    const uint32_t now = millis();
    if (now < nextRetryAtMs) {
        return;
    }

    const unsigned short port = localBrokerPort();
    if (s_localBroker.init(port)) {
        s_localBrokerStarted = true;
        M5_LOGI("[MQTT] Local broker started on port %u", port);

        // Publier la configuration du broker sur le topic système
        sf_mqtt::publish("smartfranklin/system/mqtt_broker/config/host", CONFIG.ext_mqtt_host.c_str());
        sf_mqtt::publish("smartfranklin/system/mqtt_broker/config/port", std::to_string(port));
        return;
    }

    nextRetryAtMs = now + 5000;
    M5_LOGW("[MQTT] Local broker init failed on port %u, retrying in 5s", port);
}

} // namespace

namespace sf_mqtt {

bool publish_local(const std::string &topic,
                   const std::string &payload,
                   int qos,
                   bool retain)
{
    if (!s_localBrokerStarted) {
        return false;
    }

    const int normalizedQos = (qos < 0) ? 0 : ((qos > 2) ? 2 : qos);
    s_localBroker.publish(topic,
                          payload,
                          static_cast<unsigned char>(normalizedQos),
                          retain);
    return true;
}

} // namespace sf_mqtt

// ============================================================================
// FreeRTOS Task Implementation
// ============================================================================

/**
 * @brief FreeRTOS task for unified MQTT processing.
 * 
 * Main task function that maintains the MQTT broker connection and processes
 * incoming messages. Runs indefinitely, calling the MQTT library's loop
 * function at regular intervals to handle network communication and message
 * dispatching.
 * 
 * Task Behavior:
 *   - Initialize with startup logging to serial console
 *   - Enter infinite loop for continuous MQTT processing
 *   - Delay 20ms between loop iterations (50Hz processing rate)
 *   - Handle reconnection, keep-alive, and message routing automatically
 * 
 * Loop Processing:
 *   - Network I/O: Send/receive MQTT packets
 *   - Connection Management: Handle connect/disconnect/reconnect
 *   - Message Reception: Route incoming messages to callbacks
 *   - Keep-Alive: Send ping messages to maintain connection
 *   - Error Recovery: Handle network errors and reconnection
 * 
 * Task Configuration:
 *   - Update Rate: 50Hz (20ms delay between loop calls)
 *   - Priority: tskIDLE_PRIORITY + 1 (standard priority)
 *   - Stack Size: 4096 bytes (sufficient for MQTT processing)
 *   - Core Affinity: No restriction (runs on any core)
 * 
 * Error Handling:
 *   - Connection failures handled by mqtt_layer
 *   - Network timeouts managed by MQTT library
 *   - Message processing errors logged internally
 *   - Task continues running despite individual failures
 * 
 * Performance:
 *   - CPU Usage: Low (mostly network I/O waiting)
 *   - Memory Usage: Minimal (MQTT library state)
 *   - Network Usage: Efficient MQTT protocol
 *   - Responsiveness: 20ms loop interval
 *   - Reliability: Automatic reconnection and error recovery
 * 
 * Integration:
 *   - Provides MQTT connectivity for all system components
 *   - Enables cloud communication and remote monitoring
 *   - Supports bidirectional message passing
 *   - Handles authentication and security
 * 
 * @param pv - FreeRTOS task parameter (unused, nullptr)
 * 
 * @return void (task runs indefinitely)
 * 
 * @note This task is essential for MQTT functionality in SmartFranklin.
 *       All MQTT operations depend on this task running continuously.
 *       The 20ms loop interval provides good responsiveness without
 *       excessive CPU usage.
 * 
 * @see mqtt.h - MQTT abstraction layer documentation
 */
void taskMqtt(void *pv)
{
    (void)pv;

    M5_LOGI("[MQTT] MQTT task started");

    for (;;) {
        ensureLocalBrokerStarted();

        if (s_localBrokerStarted) {
            s_localBroker.update();
        }

        // Delay based on configurable task loop period.
        vTaskDelay(pdMS_TO_TICKS(mqttLoopMs()));
    }
}
 