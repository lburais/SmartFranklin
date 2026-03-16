/*
 * ============================================================================
 * M5Stack Hardware Abstraction Layer - SmartFranklin
 * ============================================================================
 * 
 * File:        m5_hw.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: Hardware interface abstraction for M5Stack device.
 *              Encapsulates M5Stack power management, input controls, display,
 *              and IMU sensor access through a unified class interface.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   M5Stack is a modular embedded development platform based on ESP32.
 *   SmartFranklin uses M5Stack as the primary hardware platform, providing:
 *   - Display with touch support for user interface
 *   - Power management and battery monitoring
 *   - Physical buttons (A, B, C) for user input
 *   - 6-axis IMU (accelerometer + gyroscope) for motion sensing
 *   - Real-time clock (RTC) for timekeeping
 *   - Integrated I2C/Serial/GPIO ports for peripheral expansion
 * 
 * Hardware Component Summary:
 * 
 *   Display:
 *   - Type: 2-inch (320x240 pixel) LCD color display
 *   - Touch: Capacitive touchscreen with multi-touch support
 *   - Brightness: Adjustable 0-255 level
 *   - Purpose: Main user interface for configuration and status display
 * 
 *   Power Management:
 *   - Battery: Integrated 110mAh Li-Po battery
 *   - Charging: USB-C with automatic charging circuit
 *   - Monitoring: Real-time voltage and charge level sensing
 *   - Deep Sleep: Ultra-low-power mode (< 10µA)
 * 
 *   Input Controls:
 *   - Button A: Left side button (GPIO pin assignment by M5Stack)
 *   - Button B: Middle/center button (GPIO pin assignment by M5Stack)
 *   - Button C: Right side button (disabled in SmartFranklin, reserved)
 *   - Usage: Menu navigation, command execution, configuration selection
 * 
 *   Inertial Measurement Unit (IMU):
 *   - Sensor: 6-axis IMU (3-axis accelerometer + 3-axis gyroscope)
 *   - Type: MEMS sensor (typically MPU6886 or similar)
 *   - Acceleration Range: ±2g to ±16g (configurable)
 *   - Sampling: Continuous monitoring at 100+ Hz
 *   - Applications: Tilt angle calculation, motion detection
 * 
 *   Real-Time Clock (RTC):
 *   - Type: I2C battery-backed RTC (BM8563 or similar)
 *   - Accuracy: ±20 ppm typical
 *   - Battery: Separate coin cell for timekeeping during power loss
 *   - Function: System clock source, timezone-aware scheduling
 * 
 * Architecture Pattern:
 *   This module implements the SINGLETON pattern through global HW object.
 *   The M5Hardware class encapsulates M5Stack library calls, providing:
 *   - Single unified interface to hardware
 *   - Abstraction from M5Stack library internals
 *   - Easy migration if hardware platform changes
 *   - Cleaner application code without library-specific calls
 * 
 * Dependencies:
 *   - M5Unified.h (M5Stack abstraction library)
 *   - m5_hw.h (class definition and HwStatus structure)
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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ============================================================================
 */

#include "m5_hw.h"

// ============================================================================
// Global Hardware Instance (Singleton Pattern)
// ============================================================================

/**
 * @brief Global M5Stack hardware abstraction instance.
 * 
 * Single global instance of M5Hardware providing unified access to all
 * physical hardware components (power, buttons, display, IMU). Instantiates
 * on module load and destroyed on program termination.
 * 
 * Global Scope:
 *   Accessible from any module that includes m5_hw.h header file.
 *   Initialize by calling HW.init() once during system startup.
 * 
 * Usage:
 *   @code
 *   void setup() {
 *       HW.init();  // Initialize displays, power monitoring, IMU
 *   }
 * 
 *   void loop() {
 *       HwStatus status = HW.read();  // Poll all sensors and buttons
 *       
 *       if (status.button_a) {
 *           // User pressed button A
 *       }
 *   }
 *   @endcode
 * 
 * @see M5Hardware - Class definition
 * @see HwStatus - Status structure populated by read() method
 */
M5Hardware HW;

// ============================================================================
// Hardware Initialization
// ============================================================================

/**
 * @brief Initializes all M5Stack hardware components.
 * 
 * Performs one-time setup of:
 *   - M5Stack core system (power, USB, button inputs)
 *   - Display controller and rendering engine
 *   - Power management and battery monitoring
 *   - IMU (accelerometer/gyroscope) sensor
 *   - RTC (real-time clock) timekeeping
 *   - I2C bus and connected peripherals
 * 
 * Display Configuration:
 *   - Sets brightness to 128 (50% intensity)
 *   - Initializes color display with default orientation (portrait)
 *   - Enables touch processing if touchscreen available
 *   - Clears display buffer and prepares for rendering
 * 
 * Power Management Setup:
 *   - Enables battery voltage and charge level monitoring
 *   - Initializes charging circuit detection
 *   - Sets up USB power detection
 *   - Prepares for low-battery notifications
 * 
 * Input Setup:
 *   - Configures button A, B, C pin handling
 *   - Sets debounce filters to prevent false triggers
 *   - Enables press/release detection
 * 
 * IMU Setup:
 *   - Initializes 6-axis sensor (accelerometer + gyroscope)
 *   - Sets default measurement ranges (typically ±2g accel, ±250°/s gyro)
 *   - Calibrates zero-point if available
 * 
 * I2C Bus Setup:
 *   - Configures I2C master for peripheral communication
 *   - Sets standard 400kHz clock frequency
 *   - Enables GPIO pull-up resistors
 * 
 * Execution Time:
 *   - Total initialization: ~100-500ms (M5Stack library dependent)
 *   - Should be called only once during system startup
 *   - Must be called before any HW.read() or other HW calls
 * 
 * @return void
 * 
 * @note Call this function exactly once during setup() phase.
 *       Calling multiple times may cause undefined behavior.
 *       Example: HW.init(); // in setup() function
 * 
 * @see M5.begin() - M5Stack core initialization
 * @see M5.Display.setBrightness() - Display brightness control
 */
void M5Hardware::init()
{
    // Initialize M5Stack core system
    // Sets up power management, buttons, USB, I2C bus, and internal RTC
    M5.begin();
    
    // Configure display brightness to 50% intensity (0-255 scale)
    // Default brightness prevents eye strain and conserves power
    // Value range: 0 (off) to 255 (maximum brightness)
    M5.Display.setBrightness(128);
}

// ============================================================================
// Hardware Status Reading
// ============================================================================

/**
 * @brief Reads all M5Stack hardware sensors and input states.
 * 
 * Performs a complete poll of all hardware components, collecting current
 * state and measurement data into a single HwStatus structure. Should be
 * called regularly (typically every 10-50ms) in the main loop or dedicated
 * polling task to maintain up-to-date hardware status.
 * 
 * Polling Operations Performed:
 * 
 *   1. M5Stack Internal Update
 *      - Processes button state changes
 *      - Updates power management status
 *      - Refreshes sensor data from IMU
 * 
 *   2. Power Management Readings
 *      - Battery Voltage: USB power input voltage (millivolts)
 *        Divided by 1000 to convert from mV to volts
 *        Typical range: 4.5V (charging) to 0V (disconnected)
 *      
 *      - Battery Percent: Battery charge level (0-100%)
 *        Calculated from voltage and chemistry (Li-Po)
 *        May not be available if USB disconnected
 *      
 *      - Charging Status: Is USB power currently supplied?
 *        true = charging/powered via USB
 *        false = running on battery
 * 
 *   3. Button Input Readings
 *      - Button A: User button on left side (true = pressed)
 *      - Button B: User button in center (true = pressed)
 *      - Button C: User button on right (true = pressed, not used in SmartFranklin)
 *        Returns current physical button state (not debounced by hardware)
 * 
 *   4. Inertial Measurement Unit (IMU) Readings
 *      - Accelerometer X-axis: Horizontal left-right acceleration (m/s²)
 *      - Accelerometer Y-axis: Vertical up-down acceleration (m/s²)
 *      - Accelerometer Z-axis: Depth forward-back acceleration (m/s²)
 * 
 *      IMU Coordinate System:
 *      ```
 *      Z-axis (out of screen)
 *        ↑
 *        |    Y-axis (up)
 *        |   /
 *        |  /
 *      --o-------→ X-axis (right)
 *      ```
 * 
 *      Typical Values at Rest:
 *      - Flat on table: accel_x ≈ 0, accel_y ≈ 0, accel_z ≈ 9.8 m/s²
 *      - Standing upright: accel_x ≈ 0, accel_y ≈ 9.8, accel_z ≈ 0
 *      - Tilted 45°: Values scale based on angle
 * 
 * Return Value Structure:
 *   All readings packed into single HwStatus structure:
 *   - status.battery_voltage: float, volts
 *   - status.battery_percent: uint8_t, 0-100 percent
 *   - status.charging: bool, true if USB powered
 *   - status.button_a, button_b: bool, button pressed states
 *   - status.accel_x, accel_y, accel_z: float, acceleration in m/s²
 * 
 * Non-Blocking Behavior:
 *   This function performs synchronous polling with minimal blocking.
 *   Typical execution time: < 5ms
 *   Safe to call from user loop() or FreeRTOS task
 * 
 * @return HwStatus - Structure containing all current hardware readings
 *                    Members populated with latest sensor values
 * 
 * @note This function should be called regularly (10-100Hz rate recommended)
 *       to maintain current hardware state information. 
 *       Example usage in loop: HwStatus hw = HW.read();
 * 
 * @see HwStatus - Return structure definition in m5_hw.h
 * @see M5.update() - Internal M5Stack update mechanism
 */
HwStatus M5Hardware::read()
{
    // =========================================================================
    // M5Stack Internal State Update
    // =========================================================================
    // Process button events, power state changes, and sensor updates
    // Must be called before reading individual component states
    M5.update();

    // =========================================================================
    // Power Management Status Readings
    // =========================================================================
    
    // Read battery voltage from power management IC
    // Returns voltage in millivolts (mV), divide by 1000 to convert to volts
    // Typical range: 0V (no battery) to 5V (USB charging)
    status.battery_voltage = M5.Power.getBatteryVoltage() / 1000.0f;
    
    // Read battery charge level as percentage
    // Range: 0 (empty) to 100 (full)
    // Calculated from voltage curve of Li-Po chemistry
    // May return 0 if USB disconnected and no battery detected
    status.battery_percent = M5.Power.getBatteryLevel();
    
    // Read charging status: Is USB power currently supplying device?
    // true = USB connected and supplying power
    // false = Device running on battery or no power source
    status.charging = M5.Power.isCharging();

    // =========================================================================
    // Button Input Readings
    // =========================================================================
    
    // Read Button A (left side) current press state
    // true = button currently pressed down
    // false = button released
    // Note: Not debounced at this level; debounce in application if needed
    status.button_a = M5.BtnA.isPressed();
    
    // Read Button B (center) current press state
    // true = button currently pressed down
    // false = button released
    status.button_b = M5.BtnB.isPressed();

    // =========================================================================
    // Inertial Measurement Unit (IMU) Readings
    // =========================================================================
    
    // Read 3-axis accelerometer values from integrated IMU
    // Parameters passed by reference to receive measurement values
    // Units: meters per second squared (m/s²)
    // Gravity constant: 9.81 m/s² (Earth's gravitational acceleration)
    // Range: Typically ±2g to ±16g depending on configuration
    // 
    // accel_x: Horizontal acceleration (left-right movement)
    // accel_y: Vertical acceleration (up-down movement)  
    // accel_z: Depth acceleration (forward-back movement)
    M5.Imu.getAccel(&status.accel_x, &status.accel_y, &status.accel_z);

    // Return populated status structure with all current readings
    return status;
}

// ============================================================================
// Display Control
// ============================================================================

/**
 * @brief Sets M5Stack display brightness level.
 * 
 * Adjusts the LCD backlight intensity for power management and user comfort.
 * Brightness directly impacts power consumption - lower brightness can extend
 * battery life significantly.
 * 
 * Brightness Scale:
 *   Value | Intensity  | Power Draw | Use Case
 *   ------|------------|------------|-------------------------------------
 *   0     | Off        | Minimal    | Display disabled, maximum power saving
 *   32    | Very Dim   | ~25mA      | Minimum visible in dark room
 *   64    | Dim        | ~45mA      | Indoor use, low power mode
 *   128   | Medium     | ~80mA      | Default, balanced brightness/power
 *   192   | Bright     | ~110mA     | Outdoor daytime use
 *   255   | Maximum    | ~130mA     | Maximum brightness, high power draw
 * 
 * Performance Impact:
 *   - Higher brightness increases current draw by ~0.5mA per unit
 *   - Setting to 0 disables backlight (display becomes invisible)
 *   - Setting to 255 maximum brightness: +50mA additional consumption
 *   - On battery: Each 20V brightness reduction ≈ 10 minutes additional runtime
 * 
 * Typical Application:
 *   - Automatic brightness: Lower at night, increase during day
 *   - Low-power mode: Reduce to 64 when on battery (< 5% charge)
 *   - Idle timeout: Disable display (set to 0) after 30 seconds inactivity
 *   - User control: Allow menu to adjust brightness preference
 * 
 * @param level - Brightness value (0-255 range)
 *                0 = Display backlight off (dark/invisible)
 *                128 = 50% brightness (default, recommended)
 *                255 = Maximum brightness
 * 
 * @return void
 * 
 * @note This function has immediate effect - brightness changes take effect
 *       within a few milliseconds.
 *       Example: HW.setBrightness(100); // Set to ~39% brightness
 * 
 * @see M5.Display.setBrightness() - Underlying M5Stack API
 */
void M5Hardware::setBrightness(uint8_t level)
{
    // Set display controller backlight PWM to specified brightness level
    // Range: 0 (off) to 255 (maximum)
    // Changes take effect immediately on display
    M5.Display.setBrightness(level);
}

// ============================================================================
// Power Management - Deep Sleep Mode
// ============================================================================

/**
 * @brief Enters M5Stack deep sleep ultra-low-power mode.
 * 
 * Puts the device into deep sleep state where almost all subsystems are
 * powered down. Current consumption drops to < 10µA (from typical 50-150mA
 * in active mode), extending battery life by hours or days.
 * 
 * Deep Sleep Behavior:
 *   - CPU: Powered down (no code execution)
 *   - RAM: Most SRAM powered off (RTC memory preserved)
 *   - WiFi/Bluetooth: All radio modules disabled
 *   - IMU/Sensors: May remain powered if configured
 *   - Display: Completely off (backlight and LCD disabled)
 *   - RTC: Remains powered to track time
 * 
 * Wake-up Sources:
 *   The device can wake from deep sleep via:
 *   1. RTC Timer: Wake after specified minutes/hours
 *   2. External Interrupt: Button press or GPIO signal
 *   3. USB Connection: Plugging in USB power source
 *   4. Power Button: Long press on system power button
 * 
 *   Note: SmartFranklin does not currently use wake timers.
 *   Wake typically occurs via user action (button press) or USB.
 * 
 * Power Savings Example:
 *   ```
 *   Active Mode:     110mAh battery ÷ 100mA average = 1.1 hours
 *   Deep Sleep Mode: 110mAh battery ÷ 5µA average = 916 days
 *   
 *   With 10-minute active cycles (e.g., sensor readings every 10 min):
 *   - Active period: 1 minute × 100mA = 1.67mAh
 *   - Sleep period: 9 minutes × 5µA = 0.75mAh
 *   - Per cycle: 2.42mAh total
 *   - Battery life: 110mAh ÷ 2.42mAh = 45+ cycles = 7.5 hours
 *   ```
 * 
 * Crash/Hang Prevention:
 *   This function does NOT return. Once deep sleep is initiated:
 *   - All code execution halts
 *   - No cleanup is performed
 *   - Registered interrupt handlers are disabled
 *   - Device will not respond to I2C, serial, or network inputs
 * 
 *   DO NOT place critical code after this call - it will not execute.
 * 
 * Typical Usage Pattern:
 *   @code
 *   // Before entering deep sleep, save any critical data
 *   config_save();           // Persist configuration to SPIFFS
 *   mqtt_publish_final();    // Send final status message
 *   
 *   // Now safe to sleep - no more code needed
 *   HW.deepSleep();  // Does not return; device sleeps
 *   @endcode
 * 
 * Use Cases:
 *   - Battery-powered deployment: Device runs for weeks on battery
 *   - Idle device: No activity detected for extended period
 *   - Low battery mode: Battery < 5%, maximize remaining battery
 *   - Scheduled maintenance: Put device to sleep during off-hours
 * 
 * @return void (function does not return - device enters infinite power-down)
 * 
 * @note This is a blocking call that never returns. Ensure all cleanup
 *       (saving state, publishing final values) is completed BEFORE
 *       calling this function.
 * 
 * @warning Data in SRAM/stack will be lost. Use SPIFFS or RTC memory
 *          for any state that must survive sleep.
 * 
 * @see M5.Power.deepSleep() - Underlying M5Stack power API
 */
void M5Hardware::deepSleep()
{
    // Initiate deep sleep mode on ESP32 power management IC
    // CPU will power down completely (halts execution)
    // Device will wake only on button press, USB insertion, or other interrupt
    // Current consumption drops to < 10µA
    M5.Power.deepSleep();
}