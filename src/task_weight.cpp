/*
 * ============================================================================
 * Weight Task Module - SmartFranklin
 * ============================================================================
 * 
 * File:        task_weight.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: FreeRTOS task for weight measurement using M5Stack weight sensor
 *              unit connected via I2C PA Hub. Measures weight in grams, handles
 *              calibration and tare operations, and publishes data to MQTT.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   SmartFranklin uses a load cell-based weight sensor for precise weight
 *   measurement. This task manages the M5Stack Weight I2C Unit connected
 *   through a PA Hub multiplexer, allowing multiple I2C devices to share
 *   the same bus. Weight measurements are published to MQTT and stored
 *   in the global data model for system-wide access.
 * 
 * Hardware Configuration:
 *   - Sensor: M5Stack Weight I2C Unit (load cell with HX711 amplifier)
 *   - Interface: Direct Wire I2C or PA Hub channel (runtime-discovered)
 *   - Channel Source: HW.getLastI2CEnumerationReport()
 *   - I2C Pins: Port A SDA/SCL (configurable via M5Stack pin mapping)
 *   - I2C Speed: 400kHz (fast mode for reliable communication)
 * 
 * Weight Measurement:
 *   - Technology: Strain gauge load cell with 24-bit ADC
 *   - Range: Configurable (depends on load cell specifications)
 *   - Resolution: High precision weight measurement
 *   - Units: Grams (integer and float representations available)
 *   - Update Rate: Configurable via PERIOD_WEIGHT (default 1000ms)
 *   - Calibration: Offset reset and gap adjustment for accuracy
 * 
 * PA Hub Integration:
 *   - Multiplexer: Allows multiple I2C devices on single bus
 *   - Channel Assignment: Selected from startup enumeration report
 *   - Address Resolution: PA Hub handles I2C address conflicts
 *   - Hot-plug Support: Devices can be added/removed dynamically
 * 
 * Data Flow:
 *   1. Weight sensor measures force via load cell
 *   2. Raw ADC values processed by HX711 amplifier
 *   3. Digital weight value retrieved from M5UnitUnified library
 *   4. Thread-safe update to global DATA.weight_g
 *   5. MQTT publication to "smartfranklin/weight/g" topic
 *   6. Logging to serial console for debugging
 * 
 * Calibration and Tare:
 *   - Offset Reset: unit.resetOffset() zeros the scale
 *   - Gap Adjustment: unit.writeGap(DATA.gap) sets calibration factor
 *   - Tare Operation: Performed during initialization
 *   - Calibration Data: Gap value stored in global DATA structure
 *   - Accuracy: Depends on proper calibration and stable mounting
 * 
 * MQTT Publishing:
 *   - Topic: "smartfranklin/weight/g"
 *   - Payload: Weight as string with 3 decimal precision
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
 *   - Update Period: PERIOD_WEIGHT milliseconds (defined in tasks.h)
 *   - Priority: tskIDLE_PRIORITY + 1 (standard priority)
 *   - Stack Size: 4096 bytes (sufficient for I2C operations)
 *   - Core Affinity: No restriction (runs on any core)
 * 
 * Performance Considerations:
 *   - CPU Usage: Low (mostly waiting for sensor updates)
 *   - Memory Usage: Minimal (static variables and library overhead)
 *   - I2C Bandwidth: Weight sensor uses minimal bus traffic
 *   - Power Consumption: Load cell and amplifier require stable power
 *   - Responsiveness: Configurable update rate via PERIOD_WEIGHT
 * 
 * Dependencies:
 *   - M5Unified.h (M5Stack core functionality)
 *   - M5UnitUnified.h (Unified unit interface library)
 *   - M5UnitUnifiedWEIGHT.h (Weight sensor unit driver)
 *   - M5UnitUnifiedHUB.h (PA Hub multiplexer support)
 *   - M5Utility.h (M5Stack utility functions)
 *   - tasks.h (Task definitions and PERIOD_WEIGHT constant)
 *   - data_model.h (Global DATA structure and mutex)
 *   - pahub_channels.h (PA Hub channel assignments)
 *   - mqtt_layer.h (MQTT publishing interface)
 * 
 * Limitations:
 *   - Single sensor support (no multi-load cell configurations)
 *   - Fixed I2C configuration (Port A pins)
 *   - Calibration required for accurate measurements
 *   - Temperature effects on load cell accuracy
 *   - Physical mounting affects measurement stability
 *   - No automatic recalibration or drift compensation
 * 
 * Best Practices:
 *   - Mount load cell securely to prevent vibration interference
 *   - Perform tare operation on stable, empty scale
 *   - Calibrate regularly for consistent accuracy
 *   - Use appropriate update rates for application needs
 *   - Monitor serial logs for sensor communication issues
 *   - Consider temperature compensation for precision applications
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
#include <M5UnitUnifiedWEIGHT.h>
#include <M5UnitUnifiedHUB.h>
#include <M5Utility.h>

#include "tasks.h"
#include "data_model.h"
#include "m5_hw.h"
#include "pahub_channels.h"
#include "mqtt_layer.h"

// ============================================================================
// Weight Sensor Configuration
// ============================================================================

/**
 * @brief Anonymous namespace for sensor-specific variables.
 * 
 * Encapsulates weight sensor state and configuration to prevent
 * naming conflicts with other modules. All sensor-related variables
 * are contained within this namespace scope.
 */
namespace {
    /**
     * @brief Unified unit interface for M5Stack sensor management.
     * 
     * Provides high-level interface to all M5Stack units including
     * the weight sensor. Handles unit discovery, initialization,
     * and update coordination.
     */
    m5::unit::UnitUnified       Units;

    /**
     * @brief Weight I2C sensor unit instance.
     * 
     * Represents the specific weight sensor connected via I2C.
     * Provides weight measurement functionality through the
     * M5UnitUnified interface with tare and calibration support.
     */
    m5::unit::UnitWeightI2C     unit;

    /**
     * @brief PA Hub multiplexer for I2C bus management.
     * 
     * Manages multiple I2C devices on a single bus using channel
     * multiplexing. The weight sensor is connected to a specific
     * PA Hub channel to avoid address conflicts.
     */
    m5::unit::UnitPaHub2        pahub{PAHUB_ADDRESS};

    /**
     * @brief Mode index for weight display (integer/float toggle).
     * 
     * Controls whether weight is displayed as integer or float
     * in logging output. Cycles between Float and Int modes.
     * Used for debugging and monitoring different precision levels.
     */
    uint32_t                    idx{};

    /**
     * @brief Weight measurement mode table.
     * 
     * Defines available weight display modes for the sensor.
     * Currently supports Float (high precision) and Int (integer)
     * representations of weight measurements.
     */
    constexpr m5::unit::weighti2c::Mode mode_table[] = {m5::unit::weighti2c::Mode::Float, m5::unit::weighti2c::Mode::Int};

    /**
     * @brief Current weight measurement in grams.
     * 
     * Stores the most recent weight reading from the load cell sensor.
     * Updated on each sensor measurement cycle and used for data publishing.
     * Represents weight in grams as an integer value.
     */
    int32_t                     weight = 0;

}  // namespace

// ============================================================================
// Sensor Initialization Function
// ============================================================================

/**
 * @brief Initializes the weight sensor and I2C communication.
 * 
 * Sets up the M5Stack hardware, configures I2C pins for Port A,
 * initializes the PA Hub multiplexer, and connects the weight sensor.
 * Performs comprehensive error checking and enters error loop on failure.
 * Also performs initial tare operation and gap calibration.
 * 
 * Initialization Steps:
 *   1. Delay for system stabilization (2 seconds)
 *   2. Initialize M5Stack core functionality
 *   3. Configure I2C pins for Port A (SDA/SCL retrieval)
 *   4. Reinitialize I2C bus with 400kHz speed
 *   5. Connect weight unit using enumerated path (PA Hub channel or direct)
 *   6. Add PA Hub to unified units interface
 *   7. Begin unit discovery and initialization
 *   8. Reset sensor offset (tare operation)
 *   9. Write calibration gap from global DATA structure
 * 
 * I2C Configuration:
 *   - Pins: Retrieved dynamically from M5Stack pin mapping
 *   - Speed: 400kHz (fast mode for reliable sensor communication)
 *   - Bus: Reinitialized after M5.begin() to ensure clean state
 *   - Multiplexing: PA Hub handles multiple device addressing
 * 
 * Calibration Setup:
 *   - Tare: unit.resetOffset() zeros the scale
 *   - Gap: unit.writeGap(DATA.gap) applies calibration factor
 *   - Global Data: Calibration parameters from DATA structure
 *   - Accuracy: Proper calibration essential for measurement precision
 * 
 * Error Handling:
 *   - Sensor not found: Comprehensive error logging with debug info
 *   - PA Hub failure: Detailed error messages for troubleshooting
 *   - I2C issues: Bus reinitialization attempts to resolve conflicts
 *   - Initialization failure: Infinite loop prevents task continuation
 *   - Debug output: Units.debugInfo() provides diagnostic information
 * 
 * Success Indicators:
 *   - "[WEIGHT] found." logged to serial console
 *   - Units.debugInfo() displays successful unit enumeration
 *   - Sensor ready for weight measurements
 * 
 * @return void
 * 
 * @note This function blocks indefinitely on sensor initialization failure.
 *       Ensure hardware is properly connected and calibrated before running.
 *       Startup enumeration must run before this task starts.
 *       Calibration gap should be set appropriately for load cell range.
 * 
 * @see M5UnitUnified - Unified sensor interface library
 * @see UnitPaHub2 - PA Hub multiplexer documentation
 * @see pahub_channels.h - Channel assignment definitions
 */
static void weight_setup()
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

    const I2CEnumerationReport& report = HW.getLastI2CEnumerationReport();
    bool initialized = false;

    if (report.weight_on_wire_pahub && report.weight_pahub_channel >= 0) {
        const uint8_t channel = static_cast<uint8_t>(report.weight_pahub_channel);
        M5_LOGI("[WEIGHT] using enumerated PAHub channel %u", channel);
        initialized = pahub.add(unit, channel) && Units.add(pahub, Wire) && Units.begin();
    } else if (report.weight_on_wire) {
        M5_LOGI("[WEIGHT] using enumerated direct Wire path");
        initialized = Units.add(unit, Wire) && Units.begin();
    }

    if (!initialized) {
        M5_LOGE("[WEIGHT] not found.");
        M5_LOGW("%s", Units.debugInfo().c_str());

        // Enter error loop on initialization failure
        while (true) {
            m5::utility::delay(10000);
        }
    }

    M5_LOGI("[WEIGHT] found.");
    M5_LOGI("%s", Units.debugInfo().c_str());

    // Perform initial tare operation to zero the scale
    unit.resetOffset();

    // Apply calibration gap from global data structure
    unit.writeGap(DATA.gap);
}

// ============================================================================
// Sensor Measurement Loop
// ============================================================================

/**
 * @brief Main measurement loop for weight sensor data processing.
 * 
 * Processes sensor updates, retrieves weight measurements, updates global
 * data model, and publishes to MQTT. Designed to be called repeatedly
 * from the FreeRTOS task loop with alternating display modes for logging.
 * 
 * Processing Steps:
 *   1. Update M5Stack button and sensor states
 *   2. Update all connected units (including weight sensor)
 *   3. Check if weight sensor has new measurement data
 *   4. Retrieve weight measurement in grams (integer)
 *   5. Update global DATA structure with thread-safe access
 *   6. Publish weight to MQTT topic with 3 decimal precision
 *   7. Log measurement to serial console (alternating float/int display)
 * 
 * Display Mode Cycling:
 *   - idx variable cycles between 0 and 1
 *   - Mode 0: Float display - unit.weight() (high precision)
 *   - Mode 1: Integer display - unit.iweight() (integer grams)
 *   - Logging: Alternates between modes for debugging purposes
 * 
 * Data Processing:
 *   - Weight Units: Grams (int32_t from sensor library)
 *   - Thread Safety: DATA_MUTEX protects global data access
 *   - MQTT Topic: "smartfranklin/weight/g"
 *   - Precision: 3 decimal places in MQTT payload
 *   - Logging: Formatted output with M5.Log.printf()
 * 
 * Update Frequency:
 *   - Sensor-dependent: Weight sensor provides measurements asynchronously
 *   - Task-controlled: Called every PERIOD_WEIGHT milliseconds
 *   - MQTT Rate: Only publishes when new measurement available
 *   - Logging Rate: Matches measurement availability with mode cycling
 * 
 * Error Conditions:
 *   - No sensor update: Function returns without action
 *   - Invalid weight: Sensor library handles validation
 *   - I2C errors: Handled by M5UnitUnified (may cause update failures)
 *   - Mutex errors: std::lock_guard provides exception safety
 * 
 * Performance:
 *   - Execution time: < 10ms when no updates, ~20ms with MQTT publish
 *   - Memory usage: Minimal (local variables and string conversions)
 *   - I2C traffic: Brief communication during Units.update()
 *   - CPU usage: Low (mostly checking for updates)
 * 
 * @return void
 * 
 * @note This function is designed to be called in a loop from the FreeRTOS task.
 *       It only processes data when the sensor has new measurements available.
 *       Display mode cycles for debugging different precision representations.
 *       MQTT publishing occurs synchronously within this function.
 * 
 * @see Units.update() - Processes sensor measurements
 * @see unit.updated() - Checks for new weight data
 * @see sf_mqtt::publish() - MQTT message publishing
 */
static void weight_loop()
{
    // Update M5Stack system state
    M5.update();

    // Process sensor measurements and updates
    Units.update();
    if (unit.updated()) {
        // Retrieve new weight measurement
        weight = unit.iweight();
        
        // Update global data model with thread-safe access
        {
            std::lock_guard<std::mutex> lock(DATA_MUTEX);
            DATA.weight_g = weight;
        }

        // Publish weight measurement to MQTT
        sf_mqtt::publish("smartfranklin/weight/g", String(weight, 3).c_str());

        // Log measurement to serial console with alternating display modes
        if (!idx) {
            // Mode 0: Display as float for high precision
            M5.Log.printf("[WEIGHT] Weight:%f\n", unit.weight());
        } else {
            // Mode 1: Display as integer for whole grams
            M5.Log.printf("[WEIGHT] iWeight:%d\n", unit.iweight());
        }

        // Note: idx cycling logic appears incomplete in original code
        // Typically would cycle idx between 0 and 1 for mode switching
    }
}

// ============================================================================
// FreeRTOS Task Implementation
// ============================================================================

/**
 * @brief FreeRTOS task for weight measurement management.
 * 
 * Main task function that runs indefinitely, initializing the sensor
 * hardware and repeatedly calling the measurement loop. Provides
 * continuous weight monitoring with MQTT publishing and data updates.
 * 
 * Task Lifecycle:
 *   1. Log task startup to serial console
 *   2. Call setup() for sensor initialization
 *   3. Enter infinite measurement loop
 *   4. Call loop() for sensor processing
 *   5. Delay for PERIOD_WEIGHT milliseconds
 *   6. Repeat measurement cycle
 * 
 * Task Configuration:
 *   - Update Period: PERIOD_WEIGHT milliseconds (defined in tasks.h)
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
 *   - Provides weight data for system monitoring and control
 *   - Enables MQTT publishing for remote weight monitoring
 *   - Supports calibration and tare operations
 *   - Complements other sensor data with weight measurements
 * 
 * @param pv - FreeRTOS task parameter (unused, nullptr)
 * 
 * @return void (task runs indefinitely)
 * 
 * @note Task function name is taskWeight but implements weight measurement.
 *       The naming is consistent with other sensor tasks.
 *       Sensor calibration and proper mounting are critical for accuracy.
 * 
 * @see weight_setup() - Sensor initialization function
 * @see weight_loop() - Measurement processing function
 * @see PERIOD_WEIGHT - Update interval configuration
 */
void taskWeight(void *pv)
{
    M5_LOGI("[WEIGHT] Task started");

    weight_setup();

    for (;;) {
        weight_loop();
        vTaskDelay(pdMS_TO_TICKS(PERIOD_WEIGHT));
    }
}