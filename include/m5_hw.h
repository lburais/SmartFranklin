/*
 * ============================================================================
 * M5Stack Hardware Abstraction Module - SmartFranklin
 * ============================================================================
 * 
 * File:        m5_hw.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Header file for M5Stack hardware abstraction layer. Provides
 *              unified interface to M5Stack Core device sensors, buttons,
 *              display, and power management functions.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   The M5 hardware abstraction module provides a clean, unified interface
 *   to M5Stack Core device capabilities. It encapsulates M5Unified library
 *   calls and provides consistent access to battery status, button states,
 *   accelerometer readings, display control, and power management. This
 *   abstraction simplifies hardware access and enables easier testing and
 *   portability.
 * 
 * M5Stack Hardware Integration:
 *   - Core Device: M5Stack Core series (Basic, Gray, Fire, etc.)
 *   - Display: 320x240 TFT LCD with ILI9342C controller
 *   - Buttons: Three physical buttons (A, B, C) with GPIO inputs
 *   - Battery: Built-in LiPo battery with fuel gauge
 *   - IMU: 6-axis MPU6886 accelerometer/gyroscope
 *   - Power: AXP192 power management IC
 *   - Speaker: PWM audio output for beeps and tones
 * 
 * Hardware Status Monitoring:
 *   - Battery Voltage: Real-time voltage measurement
 *   - Battery Percentage: State of charge estimation
 *   - Charging Status: AC adapter connection detection
 *   - Temperature: Device internal temperature
 *   - Button States: A and B button press detection
 *   - Accelerometer: 3-axis acceleration measurements
 * 
 * Display Control:
 *   - Brightness: Backlight level adjustment (0-255)
 *   - Power Management: Display sleep/wake control
 *   - Graphics: Drawing primitives through M5Unified
 *   - Text Rendering: Font and size selection
 * 
 * Power Management:
 *   - Deep Sleep: Ultra-low power sleep mode
 *   - Wake Sources: Timer, button, or GPIO interrupts
 *   - Battery Monitoring: Low voltage warnings
 *   - Power Efficiency: Optimized for battery operation
 * 
 * Abstraction Benefits:
 *   - Unified API: Consistent interface across M5Stack models
 *   - Error Handling: Graceful failure on hardware issues
 *   - Performance: Optimized access patterns
 *   - Maintainability: Centralized hardware interface
 *   - Testing: Mockable interface for unit testing
 * 
 * Integration:
 *   - Data Model: Hardware status updates global DATA structure
 *   - Tasks: Hardware polling in dedicated FreeRTOS tasks
 *   - Display: Brightness control from user interface
 *   - Power: Deep sleep integration with system sleep management
 * 
 * Dependencies:
 *   - Arduino.h: Basic types and utilities
 *   - M5Unified.h: M5Stack unified hardware interface
 * 
 * Limitations:
 *   - M5Stack Specific: Designed for M5Stack Core devices only
 *   - Single Instance: Global HW instance for system-wide access
 *   - Synchronous: Blocking calls for hardware access
 *   - No Caching: Real-time reads on each call
 *   - Limited Error Recovery: Basic error handling
 * 
 * Best Practices:
 *   - Initialize early in setup() before other hardware access
 *   - Poll hardware status regularly but not excessively
 *   - Handle button presses with debouncing logic
 *   - Monitor battery levels for low-power warnings
 *   - Use appropriate brightness levels for power efficiency
 * 
 * Safety Considerations:
 *   - Battery Monitoring: Prevent deep discharge damage
 *   - Temperature: Monitor for overheating conditions
 *   - Button Debouncing: Prevent false trigger from contact bounce
 *   - Sleep Safety: Ensure wake conditions are properly configured
 *   - Display Brightness: Avoid excessive brightness for eye safety
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
#include <M5Unified.h>

/**
 * @brief Hardware status structure containing current M5Stack device state.
 * 
 * Aggregates all relevant hardware sensor readings and status information
 * from the M5Stack device. Updated by read() method and used throughout
 * the application for hardware monitoring and control decisions.
 */
struct HwStatus {
    /**
     * @brief Battery voltage measurement.
     * 
     * Current voltage of the built-in LiPo battery pack.
     * Critical for battery monitoring and low-voltage protection.
     * 
     * Units: Volts (V)
     * Range: Typically 3.0V to 4.2V (depends on battery chemistry)
     * Precision: 2-3 decimal places
     * Update Rate: Real-time from AXP192 power IC
     * Default: 0.0 (no measurement or error)
     */
    float battery_voltage = 0;

    /**
     * @brief Battery state of charge percentage.
     * 
     * Estimated battery capacity remaining as percentage.
     * Calculated by AXP192 fuel gauge from voltage and current measurements.
     * 
     * Units: Percentage (%)
     * Range: 0.0 to 100.0
     * Precision: 1 decimal place typical
     * Accuracy: Approximate (depends on battery age and calibration)
     * Default: 0.0 (no measurement or error)
     */
    float battery_percent = 0;

    /**
     * @brief Battery charging status flag.
     * 
     * Indicates whether the device is currently connected to external power
     * and charging the battery. Important for power management decisions.
     * 
     * Values: true (charging), false (not charging)
     * Detection: AXP192 power input sensing
     * Update Rate: Real-time
     * Default: false (not charging)
     */
    bool  charging = false;

    /**
     * @brief Device internal temperature.
     * 
     * Temperature reading from the AXP192 power management IC.
     * Used for thermal monitoring and protection.
     * 
     * Units: Degrees Celsius (°C)
     * Range: -40°C to +85°C (AXP192 operating range)
     * Precision: 1°C typical
     * Location: Internal to power management IC
     * Default: 0.0 (no measurement or error)
     */
    float temperature = 0;

    /**
     * @brief Button A press state.
     * 
     * Current state of the left physical button on M5Stack front panel.
     * Used for user input and menu navigation.
     * 
     * Values: true (pressed), false (released)
     * Debouncing: Hardware debouncing in M5Unified library
     * Update Rate: Real-time
     * Default: false (not pressed)
     */
    bool  button_a = false;

    /**
     * @brief Button B press state.
     * 
     * Current state of the middle physical button on M5Stack front panel.
     * Used for user input and menu navigation.
     * 
     * Values: true (pressed), false (released)
     * Debouncing: Hardware debouncing in M5Unified library
     * Update Rate: Real-time
     * Default: false (not pressed)
     */
    bool  button_b = false;

    /**
     * @brief Accelerometer X-axis measurement.
     * 
     * Acceleration along the X-axis from the built-in MPU6886 IMU.
     * Used for orientation detection and motion sensing.
     * 
     * Units: m/s² (meters per second squared)
     * Range: Typically -2g to +2g (±19.6 m/s²)
     * Precision: 2-3 decimal places
     * Orientation: X-axis points right when screen faces user
     * Default: 0.0 (no acceleration or error)
     */
    float accel_x = 0;

    /**
     * @brief Accelerometer Y-axis measurement.
     * 
     * Acceleration along the Y-axis from the built-in MPU6886 IMU.
     * Used for orientation detection and motion sensing.
     * 
     * Units: m/s² (meters per second squared)
     * Range: Typically -2g to +2g (±19.6 m/s²)
     * Precision: 2-3 decimal places
     * Orientation: Y-axis points up when screen faces user
     * Default: 0.0 (no acceleration or error)
     */
    float accel_y = 0;

    /**
     * @brief Accelerometer Z-axis measurement.
     * 
     * Acceleration along the Z-axis from the built-in MPU6886 IMU.
     * Used for orientation detection and motion sensing.
     * 
     * Units: m/s² (meters per second squared)
     * Range: Typically -2g to +2g (±19.6 m/s²)
     * Precision: 2-3 decimal places
     * Orientation: Z-axis points out of screen when facing user
     * Default: 0.0 (no acceleration or error)
     */
    float accel_z = 0;
};

/**
 * @brief Result snapshot of the latest I2C enumeration pass.
 */
struct I2CEnumerationReport {
    uint16_t discovered_entries = 0;
    uint16_t gravity_bridge_hits = 0;

    bool pahub_found = false;
    bool distance_found = false;
    bool weight_found = false;
    bool rtc_addr_found = false;

    bool distance_on_wire = false;
    bool distance_on_wire_pahub = false;
    int8_t distance_pahub_channel = -1;
    bool distance_on_ex = false;
    bool distance_on_ex_pahub = false;
    int8_t distance_ex_pahub_channel = -1;

    bool weight_on_wire = false;
    bool weight_on_wire_pahub = false;
    int8_t weight_pahub_channel = -1;

    bool gravity_bridge_detected = false;
    bool gravity_on_wire = false;
    bool gravity_on_wire_pahub = false;
    bool gravity_on_ex = false;
    bool gravity_on_ex_pahub = false;
    bool gravity_probe_ran = false;
    bool nb_iot2_confirmed = false;
    bool c6l_activity_detected = false;
};

/**
 * @brief M5Stack hardware abstraction class.
 * 
 * Provides unified interface to M5Stack Core device hardware components.
 * Encapsulates M5Unified library calls and provides consistent error handling.
 * Singleton pattern with global HW instance for system-wide access.
 */
class M5Hardware {
public:
    /**
     * @brief Initializes M5Stack hardware components.
     * 
     * Sets up M5Unified library and initializes all hardware interfaces.
     * Must be called once during system startup before using other methods.
     * Configures display, power management, IMU, and button interfaces.
     * 
     * Initialization Sequence:
     *   1. Initialize M5Unified library with default configuration
     *   2. Set up display with default brightness and orientation
     *   3. Initialize power management (AXP192) for battery monitoring
     *   4. Configure IMU (MPU6886) for accelerometer readings
     *   5. Set up button input with debouncing
     *   6. Perform initial hardware status read
     * 
     * Error Handling:
     *   - Hardware failures logged but initialization continues
     *   - Graceful degradation on component failures
     *   - Status flags indicate available functionality
     * 
     * Performance:
     *   - Execution Time: ~100-500ms depending on hardware
     *   - Memory Usage: M5Unified library overhead
     *   - Power Impact: Initial power-up of all components
     * 
     * Usage Notes:
     *   - Call once in setup() before other hardware operations
     *   - Check return values from subsequent calls for errors
     *   - Hardware may require stabilization time after init
     * 
     * @note Blocking call that may take several hundred milliseconds.
     *       Ensure called early in system initialization.
     * 
     * @see M5Unified::begin() - Underlying initialization function
     */
    void init();

    /**
     * @brief Reads current hardware status from all sensors.
     * 
     * Updates and returns comprehensive hardware status including battery,
     * buttons, accelerometer, and temperature measurements. Provides
     * snapshot of current device state for monitoring and control.
     * 
     * @return HwStatus structure with current hardware readings
     * 
     * Reading Process:
     *   1. Update M5Unified system state (buttons, IMU)
     *   2. Read battery voltage and percentage from AXP192
     *   3. Check charging status and power input
     *   4. Read internal temperature sensor
     *   5. Get accelerometer data from MPU6886
     *   6. Update button states with debouncing
     *   7. Return populated status structure
     * 
     * Data Sources:
     *   - Battery: AXP192 power management IC
     *   - Buttons: GPIO inputs with hardware debouncing
     *   - Accelerometer: MPU6886 6-axis IMU
     *   - Temperature: AXP192 internal sensor
     * 
     * Performance:
     *   - Execution Time: ~10-50ms depending on I2C traffic
     *   - Thread Safety: Safe for concurrent calls (no shared state)
     *   - Caching: No caching - real-time readings each call
     *   - I2C Traffic: Multiple I2C transactions per call
     * 
     * Error Handling:
     *   - I2C failures return default values (0/false)
     *   - Sensor errors logged but don't stop execution
     *   - Invalid readings clamped to reasonable ranges
     * 
     * Usage Pattern:
     *   HwStatus status = HW.read();
     *   if (status.button_a) handle_button_press();
     *   if (status.battery_percent < 20) low_battery_warning();
     * 
     * @note Function is synchronous and may block on I2C communication.
     *       Call frequency should balance responsiveness vs. performance.
     *       Default values indicate measurement errors or unavailable sensors.
     * 
     * @see HwStatus - Return data structure
     * @see M5Unified::update() - Hardware state update
     */
    HwStatus read();

    /**
     * @brief Sets the display backlight brightness level.
     * 
     * Adjusts the TFT LCD backlight intensity for visibility and power
     * management. Higher values increase brightness but consume more power.
     * 
     * @param level - Brightness level from 0 (off) to 255 (maximum)
     * 
     * Brightness Control:
     *   - Range: 0-255 (0 = backlight off, 255 = maximum brightness)
     *   - Persistence: Setting maintained until changed or reset
     *   - Power Impact: Significant effect on battery life
     *   - Visibility: Adjust based on ambient lighting conditions
     * 
     * Implementation:
     *   - Uses M5Unified display brightness control
     *   - PWM control of backlight LED
     *   - Immediate effect on display visibility
     * 
     * Usage Considerations:
     *   - Low brightness (0-50) for dark environments and power saving
     *   - Medium brightness (100-150) for normal indoor use
     *   - High brightness (200-255) for outdoor or bright lighting
     *   - Set to 0 for display off (but still consumes some power)
     * 
     * Performance:
     *   - Execution Time: < 1ms
     *   - Power Consumption: Variable based on level
     *   - No blocking operations
     * 
     * @note Brightness setting affects power consumption significantly.
     *       Consider automatic brightness adjustment based on ambient light.
     *       Display may still be visible at level 0 due to LCD properties.
     * 
     * @see M5Unified::Display::setBrightness() - Underlying implementation
     */
    void setBrightness(uint8_t level);

    /**
     * @brief Enumerates visible I2C units on all supported buses.
     *
     * Scans direct `Wire` I2C, external `M5.Ex_I2C`, and each PAHUB channel
     * (when a PAHUB is detected on either bus). Also highlights addresses that
     * match common Gravity Dual UART bridge address ranges.
     *
     * @return Number of discovered bus entries (path/address combinations).
     */
    uint16_t enumerateI2CUnits();

    /**
     * @brief Returns the latest I2C enumeration report.
     */
    const I2CEnumerationReport& getLastI2CEnumerationReport() const;

    /**
     * @brief Puts the device into deep sleep mode.
     * 
     * Enters ultra-low power deep sleep state to conserve battery power.
     * Device can be woken by timer, button press, or other configured sources.
     * All system state is preserved in RTC memory.
     * 
     * Deep Sleep Characteristics:
     *   - Power Consumption: ~10-50µA (extremely low)
     *   - Wake Sources: Timer, GPIO interrupts, touch (configurable)
     *   - State Preservation: RTC memory and GPIO states maintained
     *   - Recovery Time: ~200-500ms wake-up time
     *   - System Reset: CPU and peripherals fully reset on wake
     * 
     * Configuration:
     *   - Wake Time: Configurable sleep duration in microseconds
     *   - Wake Pins: GPIO pins that can wake the device
     *   - Wake Level: HIGH or LOW trigger for wake pins
     *   - RTC Memory: 8KB preserved across deep sleep
     * 
     * Implementation:
     *   - Uses ESP32 deep sleep functionality
     *   - Configures wake sources before sleep
     *   - Executes system-level sleep command
     *   - No return from this function (device resets)
     * 
     * Safety Considerations:
     *   - Ensure wake conditions are properly configured
     *   - Save critical data before sleep
     *   - Consider battery voltage before long sleeps
     *   - Test wake conditions thoroughly
     * 
     * Usage Pattern:
     *   // Configure wake sources
     *   esp_sleep_enable_timer_wakeup(30000000); // 30 seconds
     *   HW.deepSleep(); // Device sleeps, no return
     * 
     * @note This function does not return - device resets on wake-up.
     *       Configure wake sources before calling deepSleep().
     *       All volatile state is lost (RAM cleared on wake).
     * 
     * @see esp_sleep_enable_timer_wakeup() - Timer wake configuration
     * @see esp_sleep_enable_ext0_wakeup() - GPIO wake configuration
     */
    void deepSleep();

private:
    /**
     * @brief Internal status storage for hardware state.
     * 
     * Private member variable storing the current hardware status.
     * Updated by read() method and used for internal state management.
     * Not directly accessible from outside the class.
     */
    HwStatus status;
    I2CEnumerationReport i2c_report;
};

// ============================================================================
// Global Hardware Instance
// ============================================================================

/**
 * @brief Global M5Stack hardware abstraction instance.
 * 
 * Singleton instance providing system-wide access to M5Stack hardware.
 * All hardware operations go through this global instance for consistency
 * and centralized management.
 * 
 * Usage Pattern:
 *   HW.init();              // Initialize hardware
 *   HwStatus status = HW.read();  // Read hardware state
 *   HW.setBrightness(128);  // Set display brightness
 *   HW.deepSleep();         // Enter deep sleep (no return)
 * 
 * Access Control:
 *   - Global Scope: Accessible from any source file
 *   - Single Instance: No multiple hardware interfaces
 *   - Thread Safety: Methods are synchronous (consider mutex if needed)
 *   - Lifetime: Available throughout program execution
 * 
 * @see M5Hardware - Hardware abstraction class
 * @see HwStatus - Hardware status structure
 */
extern M5Hardware HW;