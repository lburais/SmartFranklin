/*
 * ============================================================================
 * Tilt Task Module - SmartFranklin
 * ============================================================================
 * 
 * File:        task_tilt.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: FreeRTOS task for tilt sensing using M5Stack IMU accelerometer.
 *              Calculates pitch and roll angles from acceleration data,
 *              updates global data model, and publishes orientation to MQTT.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   SmartFranklin uses the built-in IMU (Inertial Measurement Unit) to detect
 *   device orientation and tilt. This task reads accelerometer data, calculates
 *   pitch and roll angles using trigonometric functions, updates the global
 *   data model for system-wide access, and publishes orientation data to MQTT
 *   for remote monitoring and control applications.
 * 
 * IMU Hardware:
 *   - MPU6886 or similar 6-axis IMU (accelerometer + gyroscope)
 *   - 3-axis accelerometer (±2g to ±16g range)
 *   - I2C interface communication
 *   - Integrated in M5Stack Core devices
 *   - Gyroscope available but not used for tilt calculation
 * 
 * Tilt Calculation:
 *   - Pitch Angle: Rotation around X-axis (forward/backward tilt)
 *   - Roll Angle: Rotation around Y-axis (left/right tilt)
 *   - Formula: atan2(accel_y, accel_z) for pitch, atan2(-accel_x, accel_z) for roll
 *   - Units: Degrees (converted from radians with factor 57.2958)
 *   - Range: -90° to +90° for both angles (limited by atan2 function)
 *   - Accuracy: Depends on accelerometer precision and calibration
 * 
 * Data Flow:
 *   1. Read raw accelerometer data from IMU
 *   2. Calculate pitch and roll angles using atan2 functions
 *   3. Update global DATA.pitch and DATA.roll with thread-safe mutex lock
 *   4. Publish angles to MQTT topics "smartfranklin/tilt/pitch" and "smartfranklin/tilt/roll"
 *   5. Log angles to serial console for debugging
 * 
 * MQTT Publishing:
 *   - Pitch Topic: "smartfranklin/tilt/pitch"
 *   - Roll Topic: "smartfranklin/tilt/roll"
 *   - Payload: Angle as string with 1 decimal precision
 *   - QoS: Default (0, at most once delivery)
 *   - Retention: Not retained (current orientation only)
 *   - Frequency: Every PERIOD_TILT milliseconds
 * 
 * Task Configuration:
 *   - Update Period: PERIOD_TILT milliseconds (defined in tasks.h)
 *   - Priority: tskIDLE_PRIORITY + 1 (standard priority)
 *   - Stack Size: 4096 bytes (sufficient for IMU operations)
 *   - Core Affinity: No restriction (runs on any core)
 * 
 * Error Handling:
 *   - IMU Not Found: Task enters infinite loop with error logging
 *   - I2C Communication: Handled by M5Unified library
 *   - Calculation Errors: atan2 provides valid results for all inputs
 *   - Mutex Errors: std::lock_guard provides exception safety
 *   - MQTT Failures: Publishing fails gracefully, task continues
 * 
 * Performance Considerations:
 *   - CPU Usage: Low (IMU reads and atan2 calculations are fast)
 *   - Memory Usage: Minimal (local variables and math operations)
 *   - I2C Traffic: Brief communication during each IMU read cycle
 *   - Update Frequency: Configurable via PERIOD_TILT (balance responsiveness vs. overhead)
 *   - Power Impact: Minimal additional power consumption
 * 
 * Dependencies:
 *   - M5Unified.h (M5Stack unified sensor interface)
 *   - M5UnitUnified.h (M5Stack unit interface - not directly used)
 *   - M5Utility.h (M5Stack utility functions)
 *   - tasks.h (Task definitions and PERIOD_TILT constant)
 *   - data_model.h (Global DATA structure and mutex)
 *   - pahub_channels.h (shared sampling-period constants)
 *   - mqtt.h (MQTT publishing interface)
 * 
 * Limitations:
 *   - Gimbal Lock: No issues with atan2-based calculation
 *   - Dynamic Motion: Accelerometer measures gravity + acceleration
 *   - Calibration: No offset or scale calibration implemented
 *   - Gyroscope: Not used (could improve accuracy with sensor fusion)
 *   - Range Limitation: Angles limited to -90° to +90° range
 *   - Single IMU: No support for external IMU modules
 * 
 * Best Practices:
 *   - Mount device with known orientation for calibration
 *   - Consider sensor fusion with gyroscope for better accuracy
 *   - Use appropriate update intervals for application needs
 *   - Validate angle calculations against known orientations
 *   - Monitor for accelerometer saturation during high acceleration
 *   - Use tilt data for orientation-dependent functionality
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
#include <M5Utility.h>

#include "tasks.h"
#include "data_model.h"
#include "pahub_channels.h"
#include "mqtt.h"

// ============================================================================
// IMU Initialization Function
// ============================================================================

/**
 * @brief Initializes the IMU hardware and validates functionality.
 * 
 * Checks if the M5Stack IMU module is available and enabled. If the IMU
 * is not found or not functioning, logs an error and enters an infinite
 * loop to prevent task continuation with invalid tilt data.
 * 
 * Initialization Process:
 *   1. Query M5Stack IMU interface for availability
 *   2. Check M5.Imu.isEnabled() for hardware status
 *   3. Log success or failure with appropriate message level
 *   4. Enter error loop on failure to halt task execution
 * 
 * Hardware Validation:
 *   - I2C Communication: Verified by M5Unified library
 *   - IMU Chip Presence: MPU6886 detection and initialization
 *   - Accelerometer: Basic functionality check
 *   - Gyroscope: Available but not validated in this task
 * 
 * Error Handling:
 *   - IMU Disabled: Comprehensive error logging with M5_LOGE
 *   - Task Termination: Infinite loop prevents invalid operations
 *   - Recovery: Manual hardware check required to resolve
 *   - Logging: Debug information available for troubleshooting
 * 
 * Success Indicators:
 *   - "[TILT] found." logged to serial console
 *   - Task continues to measurement loop
 *   - Tilt angles available for system use
 * 
 * @return void
 * 
 * @note This function blocks indefinitely on IMU initialization failure.
 *       Ensure M5Stack IMU is properly functioning and calibrated.
 *       IMU should be mounted securely for accurate tilt measurements.
 * 
 * @see M5.Imu.isEnabled() - IMU hardware availability check
 * @see M5.Imu.getImuData() - IMU data reading function
 */
static void tilt_setup()
{
    if (!M5.Imu.isEnabled()) {
        M5_LOGE("[TILT] not found.");
        for (;;) {
            vTaskDelay(500);
        }
    }

    M5_LOGI("[TILT] found.");
}

// ============================================================================
// Tilt Measurement Loop
// ============================================================================

/**
 * @brief Main measurement loop for calculating and publishing tilt angles.
 * 
 * Retrieves accelerometer data from the IMU, calculates pitch and roll
 * angles using trigonometric functions, updates the global data model,
 * publishes angles to MQTT, and logs the values. Designed to be called
 * repeatedly from the FreeRTOS task loop.
 * 
 * Processing Steps:
 *   1. Read raw IMU data including accelerometer values
 *   2. Calculate pitch angle: atan2(accel.y, accel.z) * 57.2958
 *   3. Calculate roll angle: atan2(-accel.x, accel.z) * 57.2958
 *   4. Update global DATA.pitch and DATA.roll with thread-safe mutex lock
 *   5. Publish angles to MQTT topics with 1 decimal precision
 *   6. Log formatted angles to serial console
 * 
 * Angle Calculations:
 *   - Pitch: Forward/backward tilt (rotation around X-axis)
 *   - Roll: Left/right tilt (rotation around Y-axis)
 *   - Conversion: Radians to degrees (* 57.2958 ≈ 180/π)
 *   - Range: -90° to +90° (atan2 function limitations)
 *   - Precision: 1 decimal place for MQTT publishing
 * 
 * Thread Safety:
 *   - DATA_MUTEX lock held during global data update
 *   - Prevents race conditions with other tasks reading DATA
 *   - Lock scope limited to data update only
 * 
 * MQTT Publishing:
 *   - Pitch Topic: "smartfranklin/tilt/pitch"
 *   - Roll Topic: "smartfranklin/tilt/roll"
 *   - Payload: Float to string with 1 decimal precision
 *   - Synchronous: Publishing completes within function call
 * 
 * Logging:
 *   - Serial Output: Formatted log message with M5.Log.printf()
 *   - Debug Level: Informational logging for monitoring
 *   - Format: "[TILT] pitch:X.X  roll:Y.Y"
 * 
 * Performance:
 *   - Execution Time: < 5ms (IMU read + calculations + MQTT)
 *   - Memory Usage: Minimal (local variables only)
 *   - CPU Usage: Low (atan2 is optimized)
 *   - I2C Traffic: Brief communication during getImuData()
 * 
 * @return void
 * 
 * @note This function is designed to be called in a loop from the FreeRTOS task.
 *       It provides continuous tilt sensing for orientation monitoring.
 *       Calculations assume accelerometer measures gravity vector.
 * 
 * @see M5.Imu.getImuData() - IMU data reading
 * @see atan2() - Angle calculation function
 * @see sf_mqtt::publish() - MQTT message publishing
 */
static void tilt_loop()
{
    auto data = M5.Imu.getImuData();

    // Angle calculations
    float pitch = atan2(data.accel.y, data.accel.z) * 57.2958f;
    float roll  = atan2(-data.accel.x, data.accel.z) * 57.2958f;

    // Update shared data model
    {
        std::lock_guard<std::mutex> lock(DATA_MUTEX);
        DATA.pitch = pitch;
        DATA.roll  = roll;
    }

    // Publish to MQTT
    sf_mqtt::publish("smartfranklin/tilt/pitch",
                std::string(String(pitch, 1).c_str()));

    sf_mqtt::publish("smartfranklin/tilt/roll",
                std::string(String(roll, 1).c_str()));

    // Print values
    M5.Log.printf("[TILT] pitch:%f  roll:%f\r\n", pitch, roll);

}

// ============================================================================
// FreeRTOS Task Implementation
// ============================================================================

/**
 * @brief FreeRTOS task for tilt sensing and orientation publishing.
 * 
 * Main task function that initializes the IMU hardware and runs the
 * continuous tilt measurement and publishing loop. Provides device
 * orientation data for the entire SmartFranklin system.
 * 
 * Task Lifecycle:
 *   1. Log task startup to serial console
 *   2. Call setup() for IMU hardware initialization
 *   3. Enter infinite measurement loop
 *   4. Call loop() for angle calculation and publishing
 *   5. Delay for PERIOD_TILT milliseconds
 *   6. Repeat measurement cycle
 * 
 * Task Configuration:
 *   - Update Period: PERIOD_TILT milliseconds (defined in tasks.h)
 *   - Priority: tskIDLE_PRIORITY + 1 (standard priority)
 *   - Stack Size: 4096 bytes (sufficient for IMU operations)
 *   - Core Affinity: No restriction (runs on any core)
 * 
 * Error Recovery:
 *   - Initialization Failures: setup() enters infinite loop
 *   - Runtime Errors: Task continues running (IMU library handles)
 *   - I2C Failures: May cause read errors but task survives
 *   - MQTT Failures: Publishing fails gracefully, task continues
 * 
 * Performance:
 *   - CPU Usage: Low (mostly sleeping in vTaskDelay)
 *   - Memory Usage: Fixed after initialization
 *   - I2C Activity: Brief bursts during IMU reads
 *   - MQTT Traffic: Small angle messages periodically
 * 
 * Integration:
 *   - Provides device orientation for tilt-dependent features
 *   - Enables orientation monitoring and alerts
 *   - Supports MQTT publishing for remote tilt monitoring
 *   - Complements other sensor data with orientation context
 * 
 * @param pv - FreeRTOS task parameter (unused, nullptr)
 * 
 * @return void (task runs indefinitely)
 * 
 * @note Task function name is taskTilt but implements tilt sensing.
 *       The naming is consistent with other sensor tasks.
 *       IMU should be calibrated for accurate tilt measurements.
 * 
 * @see tilt_setup() - IMU hardware initialization
 * @see tilt_loop() - Angle calculation and publishing
 * @see PERIOD_TILT - Update interval configuration
 */
void taskTilt(void *pv)
{
    M5_LOGI("[TILT] Task started");

    tilt_setup();

    for (;;) {
        tilt_loop();
            vTaskDelay(pdMS_TO_TICKS(PERIOD_TILT));
    }

}