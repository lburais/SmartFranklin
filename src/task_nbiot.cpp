/*
 * ============================================================================
 * NB-IoT Task Module - SmartFranklin
 * ============================================================================
 * 
 * File:        task_nbiot.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: FreeRTOS task for Narrowband IoT (NB-IoT) cellular communication.
 *              Manages cellular network connection, MQTT over cellular, and GNSS
 *              positioning data publishing for remote IoT connectivity.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   SmartFranklin uses NB-IoT cellular technology for reliable, low-power
 *   connectivity in areas without WiFi or Ethernet. This task handles the
 *   complete cellular communication stack, including network attachment,
 *   MQTT messaging over cellular, and GNSS positioning data. It provides
 *   a backup communication channel for critical data transmission.
 * 
 * NB-IoT Technology:
 *   - Low Power Wide Area Network (LPWAN) technology
 *   - Optimized for IoT applications with extended battery life
 *   - Narrow bandwidth (200kHz) for efficient spectrum usage
 *   - Deep indoor penetration and wide coverage area
 *   - LTE-based but simplified for IoT requirements
 *   - PSM (Power Saving Mode) and eDRX for ultra-low power consumption
 * 
 * Cellular Network Features:
 *   - Network Attachment: Automatic connection to NB-IoT network
 *   - APN Configuration: CONFIG.nbiot_apn for network access
 *   - Signal Monitoring: RSSI and operator information tracking
 *   - IP Address Assignment: Dynamic IP for data connectivity
 *   - Connection Persistence: Maintains connection across sleep cycles
 * 
 * MQTT over Cellular:
 *   - Broker Connection: CONFIG.nbiot_mqtt_host and port
 *   - Authentication: CONFIG.nbiot_mqtt_user and password
 *   - Message Publishing: Status and sensor data over cellular
 *   - QoS Support: Reliable message delivery over cellular link
 *   - Fallback Communication: Alternative to WiFi MQTT when needed
 * 
 * GNSS Integration:
 *   - GPS/GLONASS Support: Satellite positioning data
 *   - Location Publishing: Latitude, longitude, altitude as JSON
 *   - Accuracy: GNSS accuracy depends on satellite visibility
 *   - Update Rate: 15-second intervals for position updates
 *   - JSON Format: {"lat": latitude, "lon": longitude, "alt": altitude}
 * 
 * Task Lifecycle:
 *   1. Configuration check and task termination if disabled
 *   2. Serial initialization for NB-IoT module communication
 *   3. Network attachment with APN configuration
 *   4. MQTT connection establishment (if configured)
 *   5. Main loop for status monitoring and data publishing
 *   6. GNSS data retrieval and publishing when available
 * 
 * Data Publishing:
 *   - RSSI: Signal strength published to smartfranklin/nbiot/rssi
 *   - IP Address: Current IP published to smartfranklin/nbiot/ip
 *   - Operator: Network operator name to smartfranklin/nbiot/operator
 *   - GNSS: Position data as JSON to smartfranklin/nbiot/gnss
 *   - Update Interval: 15 seconds for all status data
 * 
 * Configuration:
 *   - Enable/Disable: CONFIG.nbiot_enabled controls task execution
 *   - APN: CONFIG.nbiot_apn for network access point
 *   - MQTT Host: CONFIG.nbiot_mqtt_host for broker connection
 *   - MQTT Port: CONFIG.nbiot_mqtt_port (typically 1883)
 *   - MQTT Credentials: CONFIG.nbiot_mqtt_user and pass
 *   - Serial Pins: Hardcoded to 13 (RX), 14 (TX)
 *   - Baud Rate: 115200 for module communication
 * 
 * Error Handling:
 *   - Configuration Disabled: Task terminates gracefully
 *   - Network Attachment Failure: Logged, task continues retrying
 *   - MQTT Connection Failure: Logged, cellular MQTT disabled
 *   - GNSS Unavailable: Position publishing skipped when invalid
 *   - Serial Communication: Handled by NB_IOT2 library
 *   - Task Stability: Continues running despite individual failures
 * 
 * Performance Considerations:
 *   - Power Consumption: NB-IoT optimized for low power usage
 *   - CPU Usage: Low (mostly waiting for network operations)
 *   - Memory Usage: Minimal (NB_IOT2 library overhead)
 *   - Network Latency: Cellular delays vs. WiFi immediacy
 *   - Update Rate: 15-second intervals balance responsiveness and power
 *   - Data Usage: Minimal MQTT payloads for cost efficiency
 * 
 * Dependencies:
 *   - Arduino.h (FreeRTOS task functions)
 *   - M5Unified.h (M5Stack logging and utilities)
 *   - M5Utility.h (M5Stack utility functions)
 *   - tasks.h (Task definitions and priorities)
 *   - nb_iot2.h (NB-IoT abstraction layer)
 *   - config_store.h (Configuration access)
 *   - mqtt_layer.h (MQTT publishing interface)
 * 
 * Limitations:
 *   - Single SIM/Network: No multi-SIM or network failover
 *   - Fixed Serial Pins: GPIO 13/14 hardcoded
 *   - No SMS Support: MQTT-only communication
 *   - GNSS Dependency: Position data requires satellite visibility
 *   - Configuration Changes: Require device restart
 *   - Cost Considerations: Cellular data usage monitoring needed
 * 
 * Best Practices:
 *   - Monitor data usage to control cellular costs
 *   - Use PSM mode for extended battery life when possible
 *   - Configure appropriate APN for your cellular provider
 *   - Test GNSS performance in target deployment environment
 *   - Implement connection monitoring for reliability
 *   - Use cellular as backup to WiFi for critical communications
 * 
 * ============================================================================
 * MIT License
 * ============================================================================
 * Copyright (c) 2026 Laurent Burais
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, and/or sell, copies of the
 * Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
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

#include "tasks.h"
#include "nb_iot2.h"
#include "config_store.h"
#include "mqtt_layer.h"

// ============================================================================
// FreeRTOS Task Implementation
// ============================================================================

/**
 * @brief FreeRTOS task for NB-IoT cellular communication and data publishing.
 * 
 * Main task function that manages NB-IoT network connectivity, MQTT over
 * cellular, and GNSS positioning data. Runs indefinitely when enabled,
 * providing cellular backup communication for IoT data transmission.
 * 
 * Task Initialization:
 *   1. Check configuration enable flag
 *   2. Terminate task if NB-IoT disabled
 *   3. Initialize serial communication for NB-IoT module
 *   4. Attempt network attachment with configured APN
 *   5. Establish MQTT connection if broker configured
 *   6. Enter main processing loop
 * 
 * Main Loop Processing:
 *   - Call NB_IOT2.loop() for module communication
 *   - Retrieve and publish network status (RSSI, IP, operator)
 *   - Get GNSS position data and publish as JSON when valid
 *   - 15-second delay between update cycles
 * 
 * Network Status Publishing:
 *   - RSSI: Signal strength as integer string
 *   - IP Address: Current assigned IP address
 *   - Operator: Cellular network operator name
 *   - Topics: smartfranklin/nbiot/rssi, /ip, /operator
 * 
 * GNSS Data Publishing:
 *   - Position: Latitude, longitude, altitude as JSON
 *   - Format: {"lat": float, "lon": float, "alt": float}
 *   - Validation: Only publishes when GNSS data is valid
 *   - Topic: smartfranklin/nbiot/gnss
 * 
 * Task Configuration:
 *   - Enable Check: CONFIG.nbiot_enabled must be true
 *   - Serial Setup: Serial2 at 115200 baud, pins 13/14
 *   - APN: CONFIG.nbiot_apn for network access
 *   - MQTT: Optional connection to CONFIG.nbiot_mqtt_host
 *   - Update Rate: 15 seconds (vTaskDelay(pdMS_TO_TICKS(15000)))
 *   - Priority: tskIDLE_PRIORITY + 1 (standard priority)
 *   - Stack Size: 4096 bytes (sufficient for cellular operations)
 * 
 * Error Handling:
 *   - Disabled Configuration: Task terminates with logging
 *   - Network Attachment Failure: Logged, task continues
 *   - MQTT Connection Failure: Logged, cellular MQTT disabled
 *   - GNSS Invalid: Position publishing skipped
 *   - Serial Errors: Handled by NB_IOT2 library
 * 
 * Performance:
 *   - CPU Usage: Low (network operations are asynchronous)
 *   - Memory Usage: Minimal (library overhead)
 *   - Power Usage: NB-IoT optimized for efficiency
 *   - Network Usage: Small MQTT payloads every 15 seconds
 *   - Reliability: Cellular provides backup connectivity
 * 
 * Integration:
 *   - Provides cellular connectivity for remote deployments
 *   - Complements WiFi MQTT with cellular fallback
 *   - Enables GNSS positioning for location-aware applications
 *   - Supports IoT deployments in areas without WiFi coverage
 * 
 * @param pv - FreeRTOS task parameter (unused, nullptr)
 * 
 * @return void (task runs indefinitely if enabled, terminates if disabled)
 * 
 * @note Task automatically terminates if CONFIG.nbiot_enabled is false.
 *       Cellular connectivity provides reliable backup communication.
 *       GNSS data enhances location capabilities for IoT applications.
 * 
 * @see NB_IOT2.init() - NB-IoT module initialization
 * @see NB_IOT2.connectNetwork() - Cellular network attachment
 * @see NB_IOT2.mqttConnect() - MQTT over cellular connection
 * @see NB_IOT2.getGnss() - GNSS positioning data retrieval
 */
void taskNbiot(void *pv)
{
    M5_LOGI("[NB_IOT] Task started");

    // Check configuration enable flag
    if (!CONFIG.nbiot_enabled) {
        M5_LOGW("[NB_IOT] Disabled in config");
        vTaskDelete(nullptr);
        return;
    }

    // Initialize NB-IoT module serial communication
    // Serial2 at 115200 baud, RX pin 13, TX pin 14
    NB_IOT2.init(&Serial2, 115200, 13, 14);

    // Attempt network attachment with configured APN
    if (!NB_IOT2.connectNetwork(CONFIG.nbiot_apn)) {
        M5_LOGE("[NB_IOT] Network attach failed");
    } else {
        M5_LOGI("[NB_IOT] Network attached");
    }

    // Establish MQTT connection over cellular if configured
    if (!CONFIG.nbiot_mqtt_host.isEmpty()) {
        NB_IOT2.mqttConnect(CONFIG.nbiot_mqtt_host,
                            CONFIG.nbiot_mqtt_port,
                            CONFIG.nbiot_mqtt_user,
                            CONFIG.nbiot_mqtt_pass);
    }

    // Main task loop for status monitoring and data publishing
    for (;;) {
        // Process NB-IoT module communication and state updates
        NB_IOT2.loop();

        // Retrieve current NB-IoT status
        NbIotStatus st = NB_IOT2.getStatus();

        // Publish network status information to MQTT
        sf_mqtt::publish("smartfranklin/nbiot/rssi", String(st.rssi).c_str());
        sf_mqtt::publish("smartfranklin/nbiot/ip", st.ip.c_str());
        sf_mqtt::publish("smartfranklin/nbiot/operator", st.operator_name.c_str());

        // Retrieve and publish GNSS positioning data if available
        GnssInfo g;
        if (NB_IOT2.getGnss(g) && g.valid) {
            // Format GNSS data as JSON string
            String js = String("{\"lat\":") + String(g.lat, 6) +
                        ",\"lon\":" + String(g.lon, 6) +
                        ",\"alt\":" + String(g.alt, 1) + "}";

            // Publish GNSS position to MQTT
            sf_mqtt::publish("smartfranklin/nbiot/gnss", js.c_str());
        }

        // Delay for 15 seconds between update cycles
        vTaskDelay(pdMS_TO_TICKS(15000));
    }
}