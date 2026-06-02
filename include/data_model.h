/**
 * ============================================================================
 * Data Model Module - SmartFranklin
 * ============================================================================
 * 
 * File:        data_model.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Header file defining the shared SmartData runtime model and
 *              synchronization primitive used across modules.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   `SmartData` is the shared runtime state exchanged between sensor modules,
 *   the HMI, MQTT publication, and web dashboard JSON endpoints.
 *
 * Data Categories:
 *   - Time/GPS state from RTC and GPS modules
 *   - Gaz and tank measurements from the unified I2C scheduler
 *   - Level/assiette measurements from the IMU module
 *   - BMS telemetry and actuator state
 *
 * Thread Safety:
 *   Access must be protected with `DATA_MUTEX` whenever multiple fields are
 *   read or written as one logical snapshot.
 *
 * Integration Points:
 *   Updated by runtime modules and consumed by HMI drawing, MQTT topics, and
 *   web dashboard JSON serialization.
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
    // Time and Communication Data
    // ============================================================================

    /**
     * @brief Last successful RTC synchronization source.
     */
    String rtc_sync_source = "none";

    /**
     * @brief Current ISO UTC time read from active RTC hardware.
     */
    String rtc_time = "";

    /**
     * @brief GPS UTC time string (HH:MM:SS).
     */
    String gps_utc = "";

    /**
     * @brief GPS date string (YYYY-MM-DD).
     */
    String gps_date = "";

    /**
     * @brief GPS fix availability flag.
     */
    bool gps_has_fix = false;

    /**
     * @brief GPS latitude in decimal degrees.
     */
    double gps_latitude_deg = 0.0;

    /**
     * @brief GPS longitude in decimal degrees.
     */
    double gps_longitude_deg = 0.0;

    /**
     * @brief GPS altitude in meters.
     */
    double gps_altitude_m = 0.0;

    /**
     * @brief GPS speed over ground in knots.
     */
    double gps_speed_knots = 0.0;

    /**
     * @brief GPS course over ground in degrees.
     */
    double gps_course_deg = 0.0;

    /**
     * @brief Number of satellites used by the GNSS fix.
     */
    uint8_t gps_satellites = 0;

    // ============================================================================
    // Weight Sensor Data
    // ============================================================================

    int32_t fill_gaz = 0;

    /**
     * @brief Tank fill percentage derived from ultrasonic distance.
     *
     * Units: Percentage (%)
     * Range: 0 to 100
     */
    int32_t fill_tank = 0;

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
    int32_t weight_gaz = 0;

    /**
     * @brief Ultrasonic distance from sensor to tank surface.
     *
     * Units: Millimeters (mm)
     * Range: 20 to 4500 (sensor-limited)
     */
    int32_t distance_tank_mm = 0;

    /**
     * @brief Level pitch angle in degrees.
     */
    float level_pitch_deg = 0.0f;

    /**
     * @brief Level roll angle in degrees.
     */
    float level_roll_deg = 0.0f;

    /**
     * @brief Front-left wheel relative height in millimeters.
     */
    float level_wheel_fl_mm = 0.0f;

    /**
     * @brief Front-right wheel relative height in millimeters.
     */
    float level_wheel_fr_mm = 0.0f;

    /**
     * @brief Rear-left wheel relative height in millimeters.
     */
    float level_wheel_rl_mm = 0.0f;

    /**
     * @brief Rear-right wheel relative height in millimeters.
     */
    float level_wheel_rr_mm = 0.0f;

    /**
     * @brief AXP battery voltage in volts.
     */
    float axp_battery_voltage = 0.0f;

    /**
     * @brief AXP battery level in percent.
     */
    int32_t axp_battery_percent = 0;

    /**
     * @brief AXP charging state.
     */
    bool axp_charging = false;

    /**
     * @brief AXP PMIC temperature in celsius.
     */
    float axp_temperature = 0.0f;

    /**
     * @brief INA3221 #1 online state at address 0x40.
     */
    bool ina1_online = false;

    /**
     * @brief INA3221 #2 online state at address 0x41.
     */
    bool ina2_online = false;

    float ina1_bus_voltage_ch1_v = 0.0f;
    float ina1_bus_voltage_ch2_v = 0.0f;
    float ina1_bus_voltage_ch3_v = 0.0f;
    float ina1_shunt_voltage_ch1_mv = 0.0f;
    float ina1_shunt_voltage_ch2_mv = 0.0f;
    float ina1_shunt_voltage_ch3_mv = 0.0f;
    float ina1_current_ch1_a = 0.0f;
    float ina1_current_ch2_a = 0.0f;
    float ina1_current_ch3_a = 0.0f;

    float ina2_bus_voltage_ch1_v = 0.0f;
    float ina2_bus_voltage_ch2_v = 0.0f;
    float ina2_bus_voltage_ch3_v = 0.0f;
    float ina2_shunt_voltage_ch1_mv = 0.0f;
    float ina2_shunt_voltage_ch2_mv = 0.0f;
    float ina2_shunt_voltage_ch3_mv = 0.0f;
    float ina2_current_ch1_a = 0.0f;
    float ina2_current_ch2_a = 0.0f;
    float ina2_current_ch3_a = 0.0f;

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
     * Range: 0.0 to battery maxLevelm (typically 4.2V per cell)
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
 * 
 * Usage Examples:
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
 * 
 * Performance Considerations:
 * 
 * Best Practices:
 * 
 * @see DATA - Global data structure
 * @see std::lock_guard - RAII mutex wrapper
 */
extern std::mutex DATA_MUTEX;