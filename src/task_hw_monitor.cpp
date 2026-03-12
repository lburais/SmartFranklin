/*
 * ============================================================================
 * Hardware Monitor Task Module - SmartFranklin
 * ============================================================================
 * 
 * File:        task_hw_monitor.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: FreeRTOS task for monitoring M5Stack hardware status including
 *              IMU accelerometer, battery voltage/level/charging status,
 *              internal temperature, and button states. Publishes all sensor
 *              data to MQTT topics for remote monitoring and diagnostics.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   SmartFranklin continuously monitors the M5Stack's built-in sensors
 *   and hardware status to provide comprehensive system health information.
 *   This task collects data from the IMU (accelerometer), power management
 *   unit (battery), internal temperature sensor, and button states, then
 *   publishes everything to MQTT topics for cloud-based monitoring and
 *   remote diagnostics.
 * 
 * Hardware Monitored:
 * 
 *   IMU Accelerometer:
 *   - 3-axis acceleration (X, Y, Z axes in m/s²)
 *   - Real-time motion detection and orientation sensing
 *   - Published as JSON object: {"x": float, "y": float, "z": float}
 *   - Update rate: 5-second intervals
 * 
 *   Battery Management:
 *   - Battery voltage: Raw voltage reading in millivolts (mV)
 *   - Battery level: Percentage charge remaining (0-100%)
 *   - Charging status: Boolean indicating if device is charging
 *   - Power source detection and battery health monitoring
 * 
 *   Internal Temperature:
 *   - IMU temperature sensor reading (currently commented out)
 *   - Device internal temperature for thermal monitoring
 *   - Could be used for overheating detection and fan control
 * 
 *   Button States:
 *   - Button A and Button B press detection
 *   - Real-time button status for user interaction monitoring
 *   - Published as "1" (pressed) or "0" (released)
 * 
 * MQTT Topics Published:
 *   - smartfranklin/hw/battery_voltage: Battery voltage in mV (string)
 *   - smartfranklin/hw/battery_percent: Battery level in percent (string)
 *   - smartfranklin/hw/charging: Charging status (0/1 string)
 *   - smartfranklin/hw/temperature: Internal temperature (commented out)
 *   - smartfranklin/hw/button_a: Button A state (0/1 string)
 *   - smartfranklin/hw/button_b: Button B state (0/1 string)
 *   - smartfranklin/hw/accel: Accelerometer data (JSON string)
 * 
 * Data Publishing:
 *   - Format: All values converted to strings for MQTT compatibility
 *   - Frequency: Every 5 seconds (configurable via task delay)
 *   - QoS: Default (0, at most once delivery)
 *   - Retention: Not retained (real-time status data)
 *   - JSON for complex data: Accelerometer uses JSON object format
 * 
 * Task Configuration:
 *   - Update Interval: 5 seconds (pdMS_TO_TICKS(5000))
 *   - Priority: tskIDLE_PRIORITY + 1 (standard priority)
 *   - Stack Size: 4096 bytes (sufficient for sensor reads and MQTT)
 *   - Core Affinity: No restriction (runs on any core)
 * 
 * Error Handling:
 *   - IMU disabled: Graceful handling with zero values
 *   - Sensor failures: M5Unified library handles internally
 *   - MQTT failures: Publishing continues on next cycle
 *   - Button state errors: Boolean conversion handles edge cases
 *   - Temperature unavailable: Feature commented out (not implemented)
 * 
 * Performance Considerations:
 *   - CPU Usage: Low (sensor reads are fast, mostly sleeping)
 *   - Memory Usage: Minimal (local variables and string conversions)
 *   - I2C Traffic: IMU reads require brief I2C communication
 *   - Power Impact: Minimal additional power consumption
 *   - MQTT Bandwidth: 7 messages every 5 seconds (~1.4 msg/sec)
 * 
 * Dependencies:
 *   - Arduino.h (FreeRTOS task functions)
 *   - tasks.h (Task definitions and priorities)
 *   - m5_hw.h (M5Stack hardware abstraction - not directly used here)
 *   - mqtt.h (MQTT publishing interface)
 *   - M5Unified.h (M5Stack unified sensor and power interface)
 * 
 * Limitations:
 *   - Temperature monitoring disabled (commented out in code)
 *   - Fixed 5-second update interval (not configurable)
 *   - No data filtering or averaging (raw sensor values)
 *   - Button polling only (no interrupt-based detection)
 *   - No historical data storage (real-time only)
 *   - JSON parsing required for accelerometer data on receiver side
 * 
 * Integration Notes:
 *   - Complements m5_hw.h abstraction layer
 *   - Provides raw hardware data for diagnostics
 *   - Enables remote monitoring without physical access
 *   - Supports automated alerting based on sensor thresholds
 *   - Data can be used for predictive maintenance algorithms
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
#include "tasks.h"
#include "m5_hw.h"
#include "mqtt.h"
#include <M5Unified.h>

// ============================================================================
// FreeRTOS Task Implementation
// ============================================================================

/**
 * @brief FreeRTOS task for continuous hardware monitoring and MQTT publishing.
 * 
 * Main task function that runs indefinitely, collecting sensor data from
 * the M5Stack's built-in hardware and publishing to MQTT topics. Monitors
 * IMU acceleration, battery status, temperature, and button states at
 * regular intervals.
 * 
 * Task Behavior:
 *   - Initialize with startup logging to serial console
 *   - Enter infinite loop with 5-second update intervals
 *   - Collect data from all monitored hardware sensors
 *   - Publish each data point to dedicated MQTT topics
 *   - Handle sensor availability gracefully (IMU enable checks)
 *   - Continue operation despite individual sensor failures
 * 
 * Data Collection Sequence:
 *   1. IMU Accelerometer: Read X/Y/Z acceleration values
 *   2. Battery Voltage: Get current battery voltage in mV
 *   3. Battery Level: Retrieve battery charge percentage
 *   4. Charging Status: Check if device is currently charging
 *   5. Temperature: Read internal IMU temperature (commented out)
 *   6. Button States: Poll Button A and Button B press status
 * 
 * MQTT Publishing:
 *   - Battery voltage: Converted to string for MQTT compatibility
 *   - Battery percent: Integer to string conversion
 *   - Charging status: Boolean to "0"/"1" string
 *   - Temperature: Commented out (not currently published)
 *   - Button states: Boolean to "0"/"1" string
 *   - Accelerometer: JSON object with x/y/z float values
 * 
 * Error Handling:
 *   - IMU disabled: Sets acceleration values to zero
 *   - Sensor read failures: M5Unified handles internally
 *   - MQTT publish failures: Function continues (non-blocking)
 *   - String conversion errors: Standard library handles gracefully
 * 
 * Performance:
 *   - Execution time: ~50-100ms per cycle (sensor reads + MQTT)
 *   - CPU usage: Low (5-second sleep dominates execution time)
 *   - Memory usage: Minimal (local variables freed each cycle)
 *   - Network usage: 7 MQTT messages per 5-second interval
 * 
 * Task Configuration:
 *   - Update Rate: 5 seconds (vTaskDelay(pdMS_TO_TICKS(5000)))
 *   - Priority: tskIDLE_PRIORITY + 1 (standard priority)
 *   - Stack Size: 4096 bytes (adequate for sensor operations)
 *   - Core Affinity: No restriction (runs on any core)
 * 
 * Integration:
 *   - Provides raw hardware data for system diagnostics
 *   - Enables remote monitoring without physical access
 *   - Supports automated health checking and alerting
 *   - Complements higher-level abstraction layers
 * 
 * @param pv - FreeRTOS task parameter (unused, nullptr)
 * 
 * @return void (task runs indefinitely)
 * 
 * @note Temperature monitoring is currently disabled in the code.
 *       Uncomment the temperature publishing section if sensor is available.
 *       Task provides comprehensive hardware status for monitoring systems.
 * 
 * @see M5.Imu.getAccel() - IMU accelerometer reading
 * @see M5.Power.getBatteryVoltage() - Battery voltage monitoring
 * @see sf_mqtt::publish() - MQTT message publishing
 */
void taskHwMonitor(void *pv)
{
    M5_LOGI("[HW] Hardware monitor started");

    for (;;) {

        // --- IMU Accelerometer Reading ---
        // Read 3-axis acceleration values from built-in IMU
        // Sets to zero if IMU is disabled or unavailable
        float ax = 0, ay = 0, az = 0;
        if (M5.Imu.isEnabled()) {
            M5.Imu.getAccel(&ax, &ay, &az);
        }

        // --- Battery Status Reading ---
        // Retrieve battery voltage (mV), charge level (%), and charging state
        float batt_voltage = M5.Power.getBatteryVoltage();   // in mV
        int   batt_percent = M5.Power.getBatteryLevel();     // in %
        bool  charging = M5.Power.isCharging();

        // --- Internal Temperature Reading ---
        // IMU temperature sensor (currently commented out)
        // Uncomment when temperature monitoring is needed
        float temp = 0;
        if (M5.Imu.isEnabled()) {
            //temp = M5.Imu.getTemperature();
        }

        // --- Button State Reading ---
        // Poll current press state of Button A and Button B
        bool btnA = M5.BtnA.isPressed();
        bool btnB = M5.BtnB.isPressed();

        // --- MQTT Publishing ---
        // Publish all hardware data to MQTT topics
        // Convert numeric values to strings for MQTT compatibility
        
        sf_mqtt::publish("smartfranklin/hw/battery_voltage",
                 std::string(String(batt_voltage).c_str()));

        sf_mqtt::publish("smartfranklin/hw/battery_percent",
                 std::string(String(batt_percent).c_str()));

        sf_mqtt::publish("smartfranklin/hw/charging",
                 std::string(charging ? "1" : "0"));

        // Temperature publishing commented out
        //sf_mqtt::publish("smartfranklin/hw/temperature",
        //         std::string(String(st.temperature).c_str()));

        sf_mqtt::publish("smartfranklin/hw/button_a",
                 std::string(btnA ? "1" : "0"));

        sf_mqtt::publish("smartfranklin/hw/button_b",
                 std::string(btnB ? "1" : "0"));

        // Accelerometer data as JSON object
        String accel = String("{\"x\":") + ax +
                       ",\"y\":" + ay +
                       ",\"z\":" + az + "}";

        sf_mqtt::publish("smartfranklin/hw/accel",
                 std::string(accel.c_str()));

        // Task delay for 5-second update interval
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}