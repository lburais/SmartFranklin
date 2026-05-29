/*
 * ============================================================================
 * Hardware Monitor Task Module - SmartFranklin
 * ============================================================================
 * 
 * File:        task_hw_monitor.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: FreeRTOS task for periodic publication of built-in M5 hardware
 *              status to MQTT. The current implementation publishes battery
 *              voltage, battery percentage, and charging state.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   This task is a thin polling loop around `M5.Power` and the `sf_mqtt`
 *   publish API. It intentionally stays small and does not own sensor fusion
 *   or UI state.
 * 
 * MQTT Topics Published:
 *   - smartfranklin/hw/battery_voltage: Battery voltage in mV (string)
 *   - smartfranklin/hw/battery_percent: Battery level in percent (string)
 *   - smartfranklin/hw/charging: Charging status (0/1 string)
 * 
 * The update cadence is controlled by `CONFIG.task_hw_monitor_loop_ms`.
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
#include <mutex>
#include "tasks.h"
#include "m5_hw.h"
#include "mqtt.h"
#include "config_store.h"
#include "data_model.h"
#include "log.h"
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
 * @param pv FreeRTOS task parameter (unused, nullptr)
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
    SF_LOGI("[HW] Hardware monitor started");

    for (;;) {

        // --- Battery Status Reading ---
        // Use AXP module state mirrored in DATA.
        float batt_voltage_v = 0.0f;
        int batt_percent = 0;
        bool charging = false;

        {
            std::lock_guard<std::mutex> lock(DATA_MUTEX);
            batt_voltage_v = DATA.axp_battery_voltage;
            batt_percent = static_cast<int>(DATA.axp_battery_percent);
            charging = DATA.axp_charging;
        }

        // --- MQTT Publishing ---
        // Publish all hardware data to MQTT topics
        // Convert numeric values to strings for MQTT compatibility
        
        sf_mqtt::publish("smartfranklin/hw/battery_voltage",
             std::string(String(batt_voltage_v).c_str()));

        sf_mqtt::publish("smartfranklin/hw/battery_percent",
                 std::string(String(batt_percent).c_str()));

        sf_mqtt::publish("smartfranklin/hw/charging",
                 std::string(charging ? "1" : "0"));

        const int hwMonitorLoopMs = (CONFIG.task_hw_monitor_loop_ms > 0) ? CONFIG.task_hw_monitor_loop_ms : 5000;
        vTaskDelay(pdMS_TO_TICKS(hwMonitorLoopMs));
    }
}