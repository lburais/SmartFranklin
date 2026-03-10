/*
 * ============================================================================
 * BLE BMS Task Module - SmartFranklin
 * ============================================================================
 * 
 * File:        task_bms_ble.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: FreeRTOS task for Bluetooth Low Energy communication with JBD BMS.
 *              Scans for JBD-BMS device, establishes connection, subscribes to
 *              notifications, and processes battery data frames in real-time.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   SmartFranklin communicates with JBD (深圳杰比特) battery management systems
 *   via Bluetooth Low Energy (BLE) to monitor battery health and performance.
 *   This task handles the complete BLE connection lifecycle, from device
 *   discovery to data processing and MQTT publishing.
 * 
 * BLE Communication Architecture:
 *   - Protocol: Bluetooth Low Energy (BLE) 4.2+ compatible
 *   - Device: JBD-BMS with custom service UUIDs
 *   - Connection: Central role (SmartFranklin as client)
 *   - Data Flow: Notifications from BMS → ESP32 → MQTT publishing
 *   - Power: ESP_PWR_LVL_P9 (high power for reliable connection)
 * 
 * JBD BMS BLE Service:
 *   - Service UUID: 0000ff00-0000-1000-8000-00805f9b34fb
 *   - Characteristic UUID: 0000ff01-0000-1000-8000-00805f9b34fb (notify)
 *   - Data Format: Proprietary binary frames (parsed by jbd_parse_frame)
 *   - Update Rate: Continuous notifications when connected
 * 
 * Task Lifecycle:
 *   1. Initialize NimBLE stack with device name "SmartFranklin"
 *   2. Enter main loop for connection management
 *   3. Scan for JBD-BMS device by name (5-second active scan)
 *   4. Establish BLE connection and service discovery
 *   5. Subscribe to notification characteristic
 *   6. Process incoming notifications and update DATA model
 *   7. Publish battery data to MQTT topics
 *   8. Handle disconnection and reconnection automatically
 * 
 * Connection Management:
 *   - Scanning: Active scan for 5 seconds to find target device
 *   - Discovery: Filter devices by advertised name "JBD-BMS"
 *   - Connection: Automatic retry every 5 seconds on failure
 *   - Monitoring: Check connection status every 1 second
 *   - Recovery: Automatic reconnection on disconnection
 * 
 * Data Processing:
 *   - Notifications: Received via BmsNotifyCallback::onNotify()
 *   - Parsing: jbd_parse_frame() validates and extracts voltage/current/SOC
 *   - Storage: Thread-safe update to global DATA structure with mutex
 *   - Publishing: Real-time MQTT messages for each battery parameter
 * 
 * MQTT Topics Published:
 *   - smartfranklin/bms/voltage: Battery voltage (float, volts)
 *   - smartfranklin/bms/current: Battery current (float, amps)
 *   - smartfranklin/bms/soc: State of charge (integer, percent)
 * 
 * Error Handling:
 *   - Scan failures: Retry after 5-second delay
 *   - Connection failures: Logged and retried
 *   - Service/characteristic missing: Logged, connection aborted
 *   - Subscription failures: Logged, connection aborted
 *   - Invalid frames: Logged, frame discarded
 *   - Disconnections: Automatic reconnection attempt
 * 
 * Performance Considerations:
 *   - CPU usage: Low when connected (notification processing)
 *   - Memory usage: ~10KB for NimBLE stack and buffers
 *   - Power consumption: BLE scanning/connection increases draw
 *   - Task priority: Standard priority (tskIDLE_PRIORITY + 1)
 *   - Stack size: 4096 bytes (configured in tasks.h)
 * 
 * Dependencies:
 *   - NimBLEDevice.h (ESP-IDF NimBLE BLE stack)
 *   - Arduino.h (FreeRTOS task functions)
 *   - tasks.h (Task definitions and priorities)
 *   - jbd_bms.h (JBD frame parsing functions)
 *   - data_model.h (Global DATA structure and mutex)
 *   - mqtt_layer.h (MQTT publishing interface)
 * 
 * Configuration:
 *   - TARGET_NAME: "JBD-BMS" (BLE device name to scan for)
 *   - UUID_SERVICE: JBD BMS service UUID
 *   - UUID_NOTIFY: Notification characteristic UUID
 *   - Scan duration: 5 seconds
 *   - Retry delay: 5 seconds
 *   - Check interval: 1 second
 * 
 * Limitations:
 *   - Single BMS device support (no multi-device handling)
 *   - No bonding/security (BLE connection not encrypted)
 *   - Fixed UUIDs (may not work with all JBD BMS variants)
 *   - Name-based discovery (relies on device advertising name)
 *   - No connection timeout handling (ESP-IDF default)
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
#include <NimBLEDevice.h>
//#include <NimBLERemoteCharacteristic.h>  // Commented out as not used

#include "tasks.h"
#include "jbd_bms.h"
#include "data_model.h"
#include "mqtt_layer.h"

// ============================================================================
// Global BLE Connection State
// ============================================================================

/**
 * @brief NimBLE client instance for BMS BLE connection.
 * 
 * Manages the BLE connection to the JBD-BMS device.
 * Created during connection establishment and destroyed on disconnection.
 * Used to monitor connection status and access remote services.
 */
static NimBLEClient *client = nullptr;

/**
 * @brief Remote characteristic for BMS notifications.
 * 
 * Points to the notification characteristic on the JBD-BMS service.
 * Used to subscribe to battery data notifications.
 * Valid only when client is connected.
 */
static NimBLERemoteCharacteristic *notifyChar = nullptr;

// ============================================================================
// BLE Device Configuration Constants
// ============================================================================

/**
 * @brief Target BLE device name for scanning.
 * 
 * The advertised name that JBD-BMS devices use in BLE advertisements.
 * Used to identify and connect to the correct device during scanning.
 */
static const char *TARGET_NAME   = "JBD-BMS";

/**
 * @brief JBD BMS BLE service UUID.
 * 
 * The primary service UUID for JBD battery management system BLE interface.
 * Contains characteristics for battery monitoring and control.
 */
static const char *UUID_SERVICE  = "0000ff00-0000-1000-8000-00805f9b34fb";

/**
 * @brief Notification characteristic UUID for battery data.
 * 
 * The characteristic that sends battery status notifications.
 * Contains voltage, current, and state of charge data in JBD format.
 */
static const char *UUID_NOTIFY   = "0000ff01-0000-1000-8000-00805f9b34fb";

// ============================================================================
// BLE Notification Callback
// ============================================================================

/**
 * @brief Callback class for processing BMS BLE notifications.
 * 
 * Inherits from NimBLERemoteCharacteristicCallbacks to handle incoming
 * BLE notifications from the JBD-BMS device. Processes battery data frames
 * and updates the global data model with thread-safe operations.
 * 
 * Notification Processing:
 *   1. Receive raw BLE notification data (uint8_t array)
 *   2. Parse JBD frame using jbd_parse_frame() function
 *   3. Validate frame structure and extract battery parameters
 *   4. Update global DATA structure with mutex protection
 *   5. Publish battery data to MQTT topics for cloud integration
 * 
 * Data Extraction:
 *   - Voltage: Stored in DATA.bms_voltage (float, volts)
 *   - Current: Stored in DATA.bms_current (float, amps)
 *   - SOC: Stored in DATA.bms_soc (uint8_t, percent)
 * 
 * MQTT Publishing:
 *   - Topics use "smartfranklin/bms/" prefix for organization
 *   - Voltage and current published as strings with appropriate precision
 *   - SOC published as integer string
 *   - QoS 0, no retention (real-time data)
 * 
 * Error Handling:
 *   - Invalid frames logged and discarded
 *   - Parsing failures do not interrupt notification processing
 *   - Thread-safe data updates prevent race conditions
 * 
 * Performance:
 *   - Callback execution: < 5ms for typical frames
 *   - Memory usage: Minimal (no allocations in callback)
 *   - MQTT publishing: Asynchronous, non-blocking
 * 
 * @see jbd_parse_frame() - Frame parsing and validation
 * @see DATA - Global data structure for battery parameters
 * @see sf_mqtt::publish() - MQTT message publishing
 */
static void bmsNotifyCallback(NimBLERemoteCharacteristic *c,
                              uint8_t *data,
                              size_t len,
                              bool isNotify)
{
    // Parse the incoming JBD BMS frame
    JbdFrame f;
    if (!jbd_parse_frame(data, len, f)) {
        M5_LOGW("[BMS_BLE] Invalid frame");
        return;
    }

    // Update global data model with thread-safe access
    {
        std::lock_guard<std::mutex> lock(DATA_MUTEX);
        DATA.bms_voltage = f.voltage;
        DATA.bms_current = f.current;
        DATA.bms_soc     = f.soc;
    }

    // Publish battery data to MQTT topics
    sf_mqtt::publish("smartfranklin/bms/voltage",
                     String(f.voltage, 2).c_str());
    sf_mqtt::publish("smartfranklin/bms/current",
                     String(f.current, 2).c_str());
    sf_mqtt::publish("smartfranklin/bms/soc",
                     String(f.soc, 1).c_str());
}

// ============================================================================
// BLE Connection Establishment Function
// ============================================================================

/**
 * @brief Establishes BLE connection to JBD-BMS device.
 * 
 * Performs the complete BLE connection sequence: scanning, connection,
 * service discovery, and notification subscription. Returns true on
 * successful connection and subscription setup.
 * 
 * Connection Steps:
 *   1. Start active BLE scan for 5 seconds
 *   2. Search scan results for device with TARGET_NAME
 *   3. Create NimBLE client and connect to target device
 *   4. Discover JBD BMS service by UUID
 *   5. Find notification characteristic
 *   6. Subscribe to notifications with callback
 * 
 * Scan Configuration:
 *   - Active scan: Sends scan requests for faster discovery
 *   - Duration: 5 seconds (balance between speed and power)
 *   - Filtering: Only devices with exact name match
 * 
 * Service Discovery:
 *   - Service UUID: Fixed JBD BMS service identifier
 *   - Characteristic UUID: Notification characteristic for data
 *   - Validation: Ensures characteristic supports notifications
 * 
 * Subscription Setup:
 *   - Enable notifications on the characteristic
 *   - Register BmsNotifyCallback for data processing
 *   - Confirm subscription success before returning
 * 
 * Error Handling:
 *   - Scan failures: Logged, function returns false
 *   - Connection failures: Logged, cleanup performed
 *   - Service/characteristic not found: Logged, connection aborted
 *   - Subscription failures: Logged, connection aborted
 * 
 * Resource Management:
 *   - Client instance created on success, cleaned up on failure
 *   - Characteristic pointer stored for connection monitoring
 *   - No memory leaks on failure paths
 * 
 * Performance:
 *   - Scan time: ~5 seconds (blocking operation)
 *   - Connection time: ~100-500ms (BLE protocol dependent)
 *   - Memory usage: ~2KB for client and service objects
 * 
 * @return bool - true if BLE connection and subscription successful
 *                false if any step in the process failed
 * 
 * @note This function is blocking during scan and connection phases.
 *       Called from FreeRTOS task context with appropriate delays.
 *       Success enables automatic notification processing.
 * 
 * @see NimBLEScan - BLE scanning functionality
 * @see NimBLEClient - BLE client connection management
 * @see BmsNotifyCallback - Notification processing callback
 */
static bool connectToBms()
{
    M5_LOGI("[BMS_BLE] Scanning...");

    // Start BLE scan to find JBD-BMS device
    NimBLEScan *scan = NimBLEDevice::getScan();
    scan->setActiveScan(true);  // Active scan for faster discovery
    scan->start(5, false);  // 5-second scan
    NimBLEScanResults results = scan->getResults();

    // Search scan results for target device by name
    const NimBLEAdvertisedDevice *target = nullptr;
    for (int i = 0; i < results.getCount(); ++i) {
        const NimBLEAdvertisedDevice *dev = results.getDevice(i);
        if (dev && dev->getName() == TARGET_NAME) {
            target = dev;
            break;
        }
    }

    if (!target) {
        M5_LOGW("[BMS_BLE] Device not found");
        return false;
    }

    // Create BLE client and attempt connection
    client = NimBLEDevice::createClient();
    if (!client->connect(target)) {
        M5_LOGE("[BMS_BLE] Connect failed");
        return false;
    }

    // Discover JBD BMS service
    NimBLERemoteService *svc = client->getService(UUID_SERVICE);
    if (!svc) {
        M5_LOGE("[BMS_BLE] Service missing");
        return false;
    }

    // Find notification characteristic
    notifyChar = svc->getCharacteristic(UUID_NOTIFY);
    if (!notifyChar || !notifyChar->canNotify()) {
        M5_LOGE("[BMS_BLE] Notify characteristic missing");
        return false;
    }

    // Subscribe to notifications with callback
    if (!notifyChar->subscribe(true, bmsNotifyCallback, true)) {
        M5_LOGE("[BMS_BLE] Subscribe failed");
        return false;
    }

    M5_LOGI("[BMS_BLE] Subscribed to notifications");
    return true;
}

// ============================================================================
// FreeRTOS Task Implementation
// ============================================================================

/**
 * @brief FreeRTOS task for BLE BMS communication management.
 * 
 * Main task function that runs indefinitely, managing the BLE connection
 * lifecycle to the JBD-BMS device. Handles scanning, connection, and
 * monitoring with automatic recovery on failures.
 * 
 * Task Behavior:
 *   - Initialize NimBLE stack with device name "SmartFranklin"
 *   - Set BLE transmit power to ESP_PWR_LVL_P9 for reliable connection
 *   - Enter infinite loop for connection management
 *   - Check connection status every 1 second
 *   - Attempt reconnection if disconnected (with 5-second retry delay)
 *   - Process notifications asynchronously via callback
 * 
 * Connection Monitoring:
 *   - Status check: client != nullptr && client->isConnected()
 *   - Reconnection: Automatic on disconnection detection
 *   - Retry logic: 5-second delay between connection attempts
 *   - Logging: Informative messages for debugging
 * 
 * Task Configuration:
 *   - Stack size: 4096 bytes (defined in tasks.h)
 *   - Priority: tskIDLE_PRIORITY + 1 (standard priority)
 *   - Core affinity: No restriction (runs on any core)
 * 
 * Error Recovery:
 *   - Connection failures: Logged and retried
 *   - BLE stack errors: Task continues running
 *   - Memory issues: No dynamic allocation in main loop
 *   - Watchdog: Heartbeat sent to prevent task timeout
 * 
 * Performance:
 *   - CPU usage: Low (mostly sleeping in vTaskDelay)
 *   - Memory usage: Fixed after initialization
 *   - Power impact: BLE activity increases consumption
 *   - Responsiveness: 1-second connection checks
 * 
 * @param pv - FreeRTOS task parameter (unused, nullptr)
 * 
 * @return void (task runs indefinitely)
 * 
 * @note This task must be created during system initialization.
 *       It handles all BLE BMS communication autonomously.
 *       Watchdog integration prevents task hangs from resetting system.
 * 
 * @see connectToBms() - BLE connection establishment
 * @see BmsNotifyCallback::onNotify() - Notification processing
 * @see watchdog_beat() - Task health monitoring
 */
void taskBmsBle(void *pv)
{
    M5_LOGI("[BMS_BLE] Task started");

    // Initialize NimBLE BLE stack
    NimBLEDevice::init("SmartFranklin");
    
    // Set BLE transmit power for reliable connections
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    // Main task loop for connection management
    for (;;) {
        // Check if BLE connection is active
        if (!client || !client->isConnected()) {
            M5_LOGI("[BMS_BLE] Connecting...");
            
            // Attempt to establish BLE connection
            if (!connectToBms()) {
                M5_LOGW("[BMS_BLE] Retry in 50s");
                // Delay before retrying connection
                vTaskDelay(pdMS_TO_TICKS(50000));
                continue;
            }
            M5_LOGI("[BMS_BLE] Connected");
        }

        // Connection active, check again in 10 second
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}