/*
 * ============================================================================
 * Data Model Module - SmartFranklin
 * ============================================================================
 * 
 * File:        data_model.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Header file defining the global data model structure for
 *              SmartFranklin. Provides thread-safe access to sensor data,
 *              actuator states, and system settings shared across all tasks.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   The data model serves as the central repository for all SmartFranklin
 *   system state information. Sensor readings, actuator statuses, and
 *   configuration parameters are stored in a global structure accessible
 *   throughout the application. Thread-safe access is provided through
 *   a mutex to prevent race conditions in the FreeRTOS multi-tasking environment.
 * 
 * Data Categories:
 *   - Sensors: Distance, orientation, weight, time measurements
 *   - Power: Battery voltage, current, state of charge
 *   - Actuators: LED and buzzer control states
 *   - Settings: Target charge level and other configurable parameters
 *   - Communication: Last mesh network messages
 * 
 * Thread Safety:
 *   - Mutex Protection: DATA_MUTEX guards all data access
 *   - Lock Scope: Minimal lock duration for performance
 *   - Read/Write Patterns: Separate handling for different access types
 *   - Deadlock Prevention: Consistent lock ordering across tasks
 * 
 * Data Flow:
 *   - Sensor Tasks: Update DATA fields with new measurements
 *   - Control Tasks: Read DATA for decision making
 *   - MQTT Tasks: Publish DATA values to external systems
 *   - Display Tasks: Show DATA values on LCD interface
 *   - Persistence: Critical data saved to configuration store
 * 
 * Update Frequency:
 *   - Sensors: Vary by sensor type (distance: 100ms, weight: 1s)
 *   - Actuators: Immediate on command receipt
 *   - Settings: On configuration changes
 *   - Time: Continuous RTC updates
 * 
 * Memory Management:
 *   - Static Allocation: Fixed-size structure in global memory
 *   - String Handling: Arduino String objects with heap allocation
 *   - Float Precision: Single precision (4 bytes) for sensor data
 *   - Integer Types: Appropriate sizes for range requirements
 * 
 * Data Validation:
 *   - Range Checking: Sensor tasks validate measurements
 *   - Default Values: Sensible defaults for uninitialized data
 *   - Type Safety: Strong typing prevents incorrect assignments
 *   - Bounds Checking: Prevent buffer overflows in strings
 * 
 * Integration Points:
 *   - MQTT Layer: Publishes data model changes
 *   - Display System: Renders data model values
 *   - Command Handler: Updates actuator states
 *   - Configuration Store: Persists settings across reboots
 * 
 * Dependencies:
 *   - Arduino.h: String class and basic types
 *   - mutex: C++11 standard library mutex (ESP32 Arduino core)
 * 
 * Limitations:
 *   - Memory Usage: Global structure consumes RAM continuously
 *   - Single Writer: Assumes single writer per field (no conflicts)
 *   - No History: Current values only (no time-series data)
 *   - No Units: Values stored as raw numbers (interpretation required)
 *   - No Metadata: No timestamps or quality indicators per value
 * 
 * Best Practices:
 *   - Lock Minimally: Hold mutex only during actual data access
 *   - Validate Data: Check sensor readings for reasonableness
 *   - Document Units: Clearly specify units for each field
 *   - Atomic Updates: Update related fields together under lock
 *   - Monitor Usage: Watch for memory pressure from string fields
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
#include <Arduino.h>
#include <mutex>

/**
 * @brief Global data structure containing all SmartFranklin system state.
 * 
 * Central repository for sensor measurements, actuator states, and system
 * settings. All fields are updated by various tasks and read by display,
 * MQTT, and control functions. Thread-safe access provided through DATA_MUTEX.
 */
struct SmartData {
    // ============================================================================
    // Distance Sensor Data
    // ============================================================================

    /**
     * @brief Distance measurement from ultrasonic or laser sensor.
     * 
     * Current distance reading from proximity sensor in centimeters.
     * Updated by distance measurement task at regular intervals.
     * Used for obstacle detection and proximity monitoring.
     * 
     * Units: Centimeters (cm)
     * Range: 0.0 to sensor maximum (typically 400-500 cm)
     * Precision: 1 decimal place
     * Update Rate: Configurable (default ~100ms)
     * Default: 0.0 (no measurement)
     */
    float distance_cm = 0;

    // ============================================================================
    // Orientation Sensor Data
    // ============================================================================

    /**
     * @brief Device pitch angle from IMU accelerometer.
     * 
     * Forward/backward tilt angle calculated from accelerometer data.
     * Positive values indicate forward tilt, negative backward tilt.
     * Used for orientation monitoring and stability control.
     * 
     * Units: Degrees (°)
     * Range: -90.0 to +90.0 (limited by atan2 function)
     * Precision: Calculated internally, stored as float
     * Update Rate: Configurable (default ~100ms)
     * Default: 0.0 (level orientation)
     */
    float pitch = 0;

    /**
     * @brief Device roll angle from IMU accelerometer.
     * 
     * Left/right tilt angle calculated from accelerometer data.
     * Positive values indicate right roll, negative left roll.
     * Used for orientation monitoring and stability control.
     * 
     * Units: Degrees (°)
     * Range: -90.0 to +90.0 (limited by atan2 function)
     * Precision: Calculated internally, stored as float
     * Update Rate: Configurable (default ~100ms)
     * Default: 0.0 (level orientation)
     */
    float roll = 0;

    // ============================================================================
    // Time and Communication Data
    // ============================================================================

    /**
     * @brief Current time from Real-Time Clock.
     * 
     * Formatted timestamp string from RTC module in ISO-like format.
     * Updated continuously by RTC task for system time synchronization.
     * Used for timestamping sensor data and logging.
     * 
    * Format: "YYYY-MM-DDTHH:MM:SSZ" (ISO 8601 UTC)
    * Example: "2026-03-05T14:30:25Z"
     * Update Rate: Continuous (every few seconds)
     * Default: Empty string (no time set)
     */
    String rtc_time = "";

    /**
     * @brief Last received mesh network message.
     * 
     * Most recent message received through mesh networking layer.
     * Stored for display and debugging purposes. May contain
     * sensor data or control commands from other mesh nodes.
     * 
     * Format: Application-specific message format
     * Update Rate: On message reception
     * Default: Empty string (no messages received)
     * Retention: Overwritten by new messages
     */
    String last_mesh_msg = "";

    // ============================================================================
    // Weight Sensor Data
    // ============================================================================

    /**
     * @brief Current weight measurement from load cell.
     * 
     * Weight reading from M5Stack weight sensor in grams.
     * Raw value from sensor, may require calibration adjustment.
     * Used for weight monitoring and threshold detection.
     * 
     * Units: Grams (g)
     * Range: Sensor-dependent (load cell specifications)
     * Precision: Integer grams (sensor resolution)
     * Update Rate: Configurable (default ~1 second)
     * Default: 0 (no weight measured)
     */
    int32_t weight_g = 0;

    /**
     * @brief Weight sensor calibration gap value.
     * 
     * Calibration offset for weight sensor accuracy.
     * Applied to raw sensor readings for precise measurements.
     * Determined through calibration procedure with known weights.
     * 
     * Units: Sensor-specific calibration units
     * Range: Depends on sensor and calibration procedure
     * Update Rate: On calibration changes
     * Default: 0 (no calibration offset)
     */
    int32_t gap = 0;

    // ============================================================================
    // Battery Management System Data
    // ============================================================================

    /**
     * @brief Battery pack voltage measurement.
     * 
     * Current voltage of the battery pack from BMS.
     * Used for battery monitoring and low-voltage protection.
     * Critical for power management and safety systems.
     * 
     * Units: Volts (V)
     * Range: 0.0 to battery maximum (typically 4.2V per cell)
     * Precision: 2-3 decimal places typical
     * Update Rate: Configurable (default ~1 second)
     * Default: 0.0 (no measurement)
     */
    float bms_voltage = 0.0f;

    /**
     * @brief Battery current draw measurement.
     * 
     * Current flowing into (+) or out of (-) the battery.
     * Positive values indicate charging, negative discharging.
     * Used for power consumption monitoring and diagnostics.
     * 
     * Units: Amperes (A)
     * Range: Negative (discharge) to positive (charge)
     * Precision: 2-3 decimal places typical
     * Update Rate: Configurable (default ~1 second)
     * Default: 0.0 (no current flow)
     */
    float bms_current = 0.0f;

    /**
     * @brief Battery state of charge percentage.
     * 
     * Estimated battery capacity remaining as percentage.
     * Calculated by BMS from voltage, current, and coulomb counting.
     * Critical for battery management and user interface display.
     * 
     * Units: Percentage (%)
     * Range: 0.0 to 100.0
     * Precision: 1 decimal place typical
     * Update Rate: Configurable (default ~1 second)
     * Default: 0.0 (unknown state)
     */
    float bms_soc = 0.0f;

    // ============================================================================
    // Actuator Control States
    // ============================================================================

    /**
     * @brief LED illumination state.
     * 
     * Current on/off state of system LED indicator.
     * Controlled by command handler or automation logic.
     * Used for status indication and user feedback.
     * 
     * Values: true (LED on), false (LED off)
     * Control: MQTT commands or local logic
     * Update Rate: On state change
     * Default: false (LED off)
     */
    bool led_state = false;

    /**
     * @brief Buzzer audio output state.
     * 
     * Current on/off state of audible buzzer.
     * Used for alerts, notifications, and user attention.
     * Controlled by command handler or alarm conditions.
     * 
     * Values: true (buzzer on), false (buzzer off)
     * Control: MQTT commands or system events
     * Update Rate: On state change
     * Default: false (buzzer off)
     */
    bool buzzer_state = false;

    // ============================================================================
    // System Settings
    // ============================================================================

    /**
     * @brief Target battery state of charge.
     * 
     * Desired battery charge level for charging operations.
     * Used by battery management algorithms to determine
     * when to stop charging. Configurable by user.
     * 
     * Units: Percentage (%)
     * Range: 0 to 100
     * Default: 80 (conservative charging)
     * Update Rate: On configuration changes
     * Persistence: Saved in configuration store
     */
    int target_soc = 80;
};

// ============================================================================
// Global Data Instance and Mutex
// ============================================================================

/**
 * @brief Global data model instance accessible system-wide.
 * 
 * Singleton instance of SmartData containing all current system state.
 * All tasks read from and write to this global structure. Thread-safe
 * access is mandatory using DATA_MUTEX to prevent race conditions.
 * 
 * Access Pattern:
 *   - Reading: Lock DATA_MUTEX, read fields, unlock
 *   - Writing: Lock DATA_MUTEX, update fields, unlock
 *   - Scope: Minimize lock duration for performance
 *   - Consistency: Update related fields atomically
 * 
 * Usage Examples:
 *   - Sensors: DATA.distance_cm = measurement;
 *   - Actuators: if (DATA.led_state) turn_on_led();
 *   - Display: show_value(DATA.bms_soc);
 *   - MQTT: publish("voltage", String(DATA.bms_voltage));
 * 
 * @see DATA_MUTEX - Thread synchronization primitive
 * @see SmartData - Data structure definition
 */
extern SmartData DATA;

/**
 * @brief Mutex for thread-safe access to global DATA structure.
 * 
 * Standard C++11 mutex providing exclusive access to DATA fields.
 * Must be locked before reading or writing any DATA members in
 * multi-tasking environment. Prevents race conditions and data corruption.
 * 
 * Usage Pattern:
 *   std::lock_guard<std::mutex> lock(DATA_MUTEX);
 *   // Access DATA fields here
 *   DATA.distance_cm = new_value;
 *   // Lock automatically released
 * 
 * Performance Considerations:
 *   - Lock Contention: Minimize time spent holding lock
 *   - Deadlocks: Avoid nested locks or inconsistent ordering
 *   - Priority: Higher priority tasks may preempt lower ones
 *   - Blocking: Tasks wait for lock if contended
 * 
 * Best Practices:
 *   - Use std::lock_guard for automatic unlock
 *   - Hold lock only during actual data access
 *   - Group related updates under single lock
 *   - Avoid long operations while holding lock
 *   - Document lock requirements in function comments
 * 
 * @see DATA - Global data structure
 * @see std::lock_guard - RAII mutex wrapper
 */
extern std::mutex DATA_MUTEX;