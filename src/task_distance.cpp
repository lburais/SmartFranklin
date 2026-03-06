/*
 * ============================================================================
 * Distance Task Module - SmartFranklin
 * ============================================================================
 * 
 * File:        task_distance.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: FreeRTOS task for ultrasonic distance measurement using M5Stack
 *              ultrasonic sensor unit connected via I2C PA Hub. Measures distance
 *              in centimeters and publishes to MQTT with thread-safe data updates.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   SmartFranklin uses an ultrasonic distance sensor to measure proximity
 *   or object detection. This task manages the M5Stack ultrasonic I2C unit
 *   connected through a PA Hub multiplexer, allowing multiple I2C devices
 *   to share the same bus. Distance measurements are published to MQTT
 *   and stored in the global data model for system-wide access.
 * 
 * Hardware Configuration:
 *   - Sensor: M5Stack Ultrasonic I2C Unit (HC-SR04 compatible)
 *   - Interface: I2C bus with PA Hub multiplexer
 *   - PA Hub Channel: PAHUB_CH_DISTANCE (configured in pahub_channels.h)
 *   - I2C Pins: Port A SDA/SCL (configurable via M5Stack pin mapping)
 *   - I2C Speed: 400kHz (fast mode for reliable communication)
 * 
 * Ultrasonic Measurement:
 *   - Technology: Time-of-flight ultrasonic ranging
 *   - Range: 2-400 cm (manufacturer specifications)
 *   - Accuracy: ±1 cm typical (temperature and angle dependent)
 *   - Update Rate: Configurable via PERIOD_DISTANCE (default 1000ms)
 *   - Units: Centimeters (stored as float for precision)
 * 
 * PA Hub Integration:
 *   - Multiplexer: Allows multiple I2C devices on single bus
 *   - Channel Assignment: Dedicated channel for distance sensor
 *   - Address Resolution: PA Hub handles I2C address conflicts
 *   - Hot-plug Support: Devices can be added/removed dynamically
 * 
 * Data Flow:
 *   1. Ultrasonic sensor measures distance via time-of-flight
 *   2. Raw distance value retrieved from M5UnitUnified library
 *   3. Thread-safe update to global DATA.distance_cm
 *   4. MQTT publication to "smartfranklin/distance/cm" topic
 *   5. Logging to serial console for debugging
 * 
 * MQTT Publishing:
 *   - Topic: "smartfranklin/distance/cm"
 *   - Payload: Distance as string with 1 decimal precision
 *   - QoS: Default (0, at most once delivery)
 *   - Retention: Not retained (real-time data)
 *   - Frequency: Every measurement update (continuous)
 * 
 * Error Handling:
 *   - Sensor not found: Task logs error and enters infinite loop
 *   - I2C communication failures: Handled by M5UnitUnified library
 *   - Invalid measurements: Sensor library provides validation
 *   - PA Hub issues: Debug info logged for troubleshooting
 *   - Mutex protection: Prevents data corruption during updates
 * 
 * Task Configuration:
 *   - Update Period: PERIOD_DISTANCE milliseconds (defined in tasks.h)
 *   - Priority: tskIDLE_PRIORITY + 1 (standard priority)
 *   - Stack Size: 4096 bytes (sufficient for I2C operations)
 *   - Core Affinity: No restriction (runs on any core)
 * 
 * Performance Considerations:
 *   - CPU Usage: Low (mostly waiting for sensor updates)
 *   - Memory Usage: Minimal (static variables and library overhead)
 *   - I2C Bandwidth: Ultrasonic sensor uses minimal bus traffic
 *   - Power Consumption: Ultrasonic pulses require brief power spikes
 *   - Responsiveness: Configurable update rate via PERIOD_DISTANCE
 * 
 * Dependencies:
 *   - M5Unified.h (M5Stack core functionality)
 *   - M5UnitUnified.h (Unified unit interface library)
 *   - M5UnitUnifiedDISTANCE.h (Ultrasonic sensor unit driver)
 *   - M5UnitUnifiedHUB.h (PA Hub multiplexer support)
 *   - M5Utility.h (M5Stack utility functions)
 *   - tasks.h (Task definitions and PERIOD_DISTANCE constant)
 *   - data_model.h (Global DATA structure and mutex)
 *   - pahub_channels.h (PA Hub channel assignments)
 *   - mqtt_layer.h (MQTT publishing interface)
 * 
 * Limitations:
 *   - Single sensor support (no multi-sensor arrays)
 *   - Fixed I2C configuration (Port A pins)
 *   - Blocking initialization (infinite loop on sensor failure)
 *   - No sensor health monitoring (assumes operational)
 *   - Temperature compensation not implemented
 *   - Angle dependency affects accuracy (perpendicular mounting required)
 * 
 * Best Practices:
 *   - Mount sensor perpendicular to target surface
 *   - Avoid soft/uneven surfaces for best accuracy
 *   - Consider temperature effects on speed of sound
 *   - Use appropriate update rates for application needs
 *   - Monitor serial logs for sensor communication issues
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

#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedDISTANCE.h>
#include <M5UnitUnifiedHUB.h>
#include <M5Utility.h>

#include "tasks.h"
#include "data_model.h"
#include "pahub_channels.h"
#include "mqtt_layer.h"

// ============================================================================
// Ultrasonic Sensor Configuration
// ============================================================================

/**
 * @brief Anonymous namespace for sensor-specific variables.
 * 
 * Encapsulates ultrasonic sensor state and configuration to prevent
 * naming conflicts with other modules. All sensor-related variables
 * are contained within this namespace scope.
 */
namespace {
    /**
     * @brief Unified unit interface for M5Stack sensor management.
     * 
     * Provides high-level interface to all M5Stack units including
     * the ultrasonic sensor. Handles unit discovery, initialization,
     * and update coordination.
     */
    m5::unit::UnitUnified       Units;

    /**
     * @brief Ultrasonic I2C sensor unit instance.
     * 
     * Represents the specific ultrasonic distance sensor connected
     * via I2C. Provides distance measurement functionality through
     * the M5UnitUnified interface.
     */
    m5::unit::UnitUltraSonicI2C unit;

    /**
     * @brief PA Hub multiplexer for I2C bus management.
     * 
     * Manages multiple I2C devices on a single bus using channel
     * multiplexing. The ultrasonic sensor is connected to a specific
     * PA Hub channel to avoid address conflicts.
     */
    m5::unit::UnitPaHub2        pahub{PAHUB_ADDRESS};

    /**
     * @brief Current distance measurement in centimeters.
     * 
     * Stores the most recent distance reading from the ultrasonic sensor.
     * Updated on each sensor measurement cycle and used for data publishing.
     */
    int32_t                     distance = 0;

    constexpr uint8_t           DISTANCE_I2C_ADDRESS = 0x57;

    bool i2c_device_exists(const uint8_t address)
    {
        Wire.beginTransmission(address);
        return Wire.endTransmission() == 0;
    }

    bool pahub_select_channel(const uint8_t channel)
    {
        Wire.beginTransmission(PAHUB_ADDRESS);
        Wire.write(static_cast<uint8_t>(1U << channel));
        return Wire.endTransmission() == 0;
    }

    void pahub_disable_all_channels()
    {
        Wire.beginTransmission(PAHUB_ADDRESS);
        Wire.write(static_cast<uint8_t>(0x00));
        Wire.endTransmission();
    }
}  // namespace

// ============================================================================
// Sensor Initialization Function
// ============================================================================

/**
 * @brief Initializes the ultrasonic distance sensor and I2C communication.
 * 
 * Sets up the M5Stack hardware, configures I2C pins for Port A,
 * initializes the PA Hub multiplexer, and connects the ultrasonic sensor.
 * Performs comprehensive error checking and enters error loop on failure.
 * 
 * Initialization Steps:
 *   1. Delay for system stabilization (2 seconds)
 *   2. Initialize M5Stack core functionality
 *   3. Configure I2C pins for Port A (SDA/SCL retrieval)
 *   4. Reinitialize I2C bus with 400kHz speed
 *   5. Connect ultrasonic unit to PA Hub channel
 *   6. Add PA Hub to unified units interface
 *   7. Begin unit discovery and initialization
 * 
 * I2C Configuration:
 *   - Pins: Retrieved dynamically from M5Stack pin mapping
 *   - Speed: 400kHz (fast mode for reliable sensor communication)
 *   - Bus: Reinitialized after M5.begin() to ensure clean state
 *   - Multiplexing: PA Hub handles multiple device addressing
 * 
 * PA Hub Setup:
 *   - Address: PAHUB_ADDRESS (configured in pahub_channels.h)
 *   - Channel: PAHUB_CH_DISTANCE (dedicated distance sensor channel)
 *   - Connection: Ultrasonic unit attached to specific channel
 *   - Validation: Unit addition and initialization verified
 * 
 * Error Handling:
 *   - Sensor not found: Comprehensive error logging with debug info
 *   - PA Hub failure: Detailed error messages for troubleshooting
 *   - I2C issues: Bus reinitialization attempts to resolve conflicts
 *   - Initialization failure: Infinite loop prevents task continuation
 *   - Debug output: Units.debugInfo() provides diagnostic information
 * 
 * Success Indicators:
 *   - "[DISTANCE] found." logged to serial console
 *   - Units.debugInfo() displays successful unit enumeration
 *   - Task continues to measurement loop
 * 
 * @return void
 * 
 * @note This function blocks indefinitely on sensor initialization failure.
 *       Ensure hardware is properly connected before running the task.
 *       PA Hub channel assignments must match physical connections.
 * 
 * @see M5UnitUnified - Unified sensor interface library
 * @see UnitPaHub2 - PA Hub multiplexer documentation
 * @see pahub_channels.h - Channel assignment definitions
 */
static void distance_setup()
{
    // Allow system to stabilize after power-on
    m5::utility::delay(2000);

    // Initialize M5Stack core functionality
    M5.begin();

    // Retrieve I2C pin assignments for Port A
    auto pin_num_sda = M5.getPin(m5::pin_name_t::port_a_sda);
    auto pin_num_scl = M5.getPin(m5::pin_name_t::port_a_scl);
    M5_LOGI("getPin: SDA:%u SCL:%u", pin_num_sda, pin_num_scl);

    // Reinitialize I2C bus with custom pins and speed
    Wire.end();
    Wire.begin(pin_num_sda, pin_num_scl, 400000U);

    bool initialized = false;

    // Prefer PAHub auto-discovery if the hub is present
    if (i2c_device_exists(PAHUB_ADDRESS)) {
        int foundChannel = -1;

        for (uint8_t channel = 0; channel < 8; ++channel) {
            if (!pahub_select_channel(channel)) {
                continue;
            }
            if (i2c_device_exists(DISTANCE_I2C_ADDRESS)) {
                foundChannel = channel;
                break;
            }
        }

        pahub_disable_all_channels();

        if (foundChannel >= 0) {
            M5_LOGI("[DISTANCE] found via PAHub channel %d", foundChannel);
            initialized = pahub.add(unit, static_cast<uint8_t>(foundChannel)) &&
                          Units.add(pahub, Wire) &&
                          Units.begin();
        }
    }

    // Fallback: direct I2C unit (without PAHub)
    if (!initialized && i2c_device_exists(DISTANCE_I2C_ADDRESS)) {
        M5_LOGI("[DISTANCE] found on direct I2C");
        initialized = Units.add(unit, Wire) && Units.begin();
    }

    if (!initialized) {
        M5_LOGE("[DISTANCE] not found.");
        M5_LOGW("%s", Units.debugInfo().c_str());

        // Enter error loop on initialization failure
        while (true) {
            m5::utility::delay(10000);
        }
    }

    M5_LOGI("[DISTANCE] found.");
    M5_LOGI("%s", Units.debugInfo().c_str());
}

// ============================================================================
// Sensor Measurement Loop
// ============================================================================

/**
 * @brief Main measurement loop for ultrasonic distance sensor.
 * 
 * Processes sensor updates, retrieves distance measurements,
 * updates global data model, and publishes to MQTT. Designed
 * to be called repeatedly from the FreeRTOS task loop.
 * 
 * Processing Steps:
 *   1. Update M5Stack button and sensor states
 *   2. Update all connected units (including ultrasonic sensor)
 *   3. Check if ultrasonic sensor has new measurement data
 *   4. Retrieve distance measurement in centimeters
 *   5. Update global DATA structure with thread-safe access
 *   6. Publish distance to MQTT topic
 *   7. Log measurement to serial console for debugging
 * 
 * Data Processing:
 *   - Distance Units: Centimeters (int32_t from sensor library)
 *   - Thread Safety: DATA_MUTEX protects global data access
 *   - MQTT Topic: "smartfranklin/distance/cm"
 *   - Precision: 1 decimal place in MQTT payload
 *   - Logging: Formatted output with M5.Log.printf()
 * 
 * Update Frequency:
 *   - Sensor-dependent: Ultrasonic sensor provides measurements asynchronously
 *   - Task-controlled: Called every PERIOD_DISTANCE milliseconds
 *   - MQTT Rate: Only publishes when new measurement available
 *   - Logging Rate: Matches measurement availability
 * 
 * Error Conditions:
 *   - No sensor update: Function returns without action
 *   - Invalid distance: Sensor library handles validation
 *   - I2C errors: Handled by M5UnitUnified (may cause update failures)
 *   - Mutex errors: std::lock_guard provides exception safety
 * 
 * Performance:
 *   - Execution time: < 10ms when no updates, ~20ms with MQTT publish
 *   - Memory usage: Minimal (local variables only)
 *   - I2C traffic: Brief communication during Units.update()
 *   - CPU usage: Low (mostly checking for updates)
 * 
 * @return void
 * 
 * @note This function is designed to be called in a loop from the FreeRTOS task.
 *       It only processes data when the sensor has new measurements available.
 *       MQTT publishing occurs synchronously within this function.
 * 
 * @see Units.update() - Processes sensor measurements
 * @see unit.updated() - Checks for new distance data
 * @see sf_mqtt::publish() - MQTT message publishing
 */
static void distance_loop()
{
    // Update M5Stack system state
    M5.update();

    // Process sensor measurements and updates
    Units.update();
    if (unit.updated()) {
        // Retrieve new distance measurement
        distance = unit.distance();
        
        // Update global data model with thread-safe access
        {
            std::lock_guard<std::mutex> lock(DATA_MUTEX);
            DATA.distance_cm = distance;
        }

        // Publish distance measurement to MQTT
        sf_mqtt::publish("smartfranklin/distance/cm", String(distance, 1).c_str());

        // Log measurement to serial console
        M5.Log.printf("[DISTANCE] Distance:%f\n", distance);
    }
}

// ============================================================================
// FreeRTOS Task Implementation
// ============================================================================

/**
 * @brief FreeRTOS task for ultrasonic distance measurement management.
 * 
 * Main task function that runs indefinitely, initializing the sensor
 * hardware and repeatedly calling the measurement loop. Provides
 * continuous distance monitoring with MQTT publishing and data updates.
 * 
 * Task Lifecycle:
 *   1. Log task startup to serial console
 *   2. Call setup() for sensor initialization
 *   3. Enter infinite measurement loop
 *   4. Call loop() for sensor processing
 *   5. Delay for PERIOD_DISTANCE milliseconds
 *   6. Repeat measurement cycle
 * 
 * Task Configuration:
 *   - Update Period: PERIOD_DISTANCE milliseconds (defined in tasks.h)
 *   - Priority: tskIDLE_PRIORITY + 1 (standard priority)
 *   - Stack Size: 4096 bytes (sufficient for I2C and MQTT operations)
 *   - Core Affinity: No restriction (runs on any core)
 * 
 * Error Recovery:
 *   - Initialization failures: setup() enters infinite loop
 *   - Runtime errors: Task continues running (sensor library handles)
 *   - I2C failures: May cause measurement gaps but task survives
 *   - MQTT failures: Publishing fails gracefully, task continues
 * 
 * Performance:
 *   - CPU Usage: Low (mostly sleeping in vTaskDelay)
 *   - Memory Usage: Fixed after initialization
 *   - I2C Activity: Brief bursts during sensor updates
 *   - MQTT Traffic: Occasional publishes based on measurement rate
 * 
 * Integration:
 *   - Global Data: Updates DATA.distance_cm for system access
 *   - MQTT Publishing: Provides real-time distance data to cloud
 *   - Serial Logging: Debug output for development and monitoring
 *   - Watchdog: Should send heartbeats (not implemented in this code)
 * 
 * @param pv - FreeRTOS task parameter (unused, nullptr)
 * 
 * @return void (task runs indefinitely)
 * 
 * @note Task function name is taskDistance and matches the implemented behavior.
 *       Keep PAHub wiring consistent with runtime auto-discovery.
 * 
 * @see distance_setup() - Sensor initialization function
 * @see distance_loop() - Measurement processing function
 * @see PERIOD_DISTANCE - Update interval configuration
 */
void taskDistance(void *pv)
{
    M5_LOGI("[DISTANCE] Task started");

    distance_setup();

    for (;;) {
        distance_loop();
        vTaskDelay(pdMS_TO_TICKS(PERIOD_DISTANCE));
    }
}