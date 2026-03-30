/**
 * ============================================================================
 * FreeRTOS Task Declarations - SmartFranklin
 * ============================================================================
 * 
 * File:        tasks.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Central declaration point for all SmartFranklin FreeRTOS tasks.
 *              Declares task entrypoints, shared task handles, and scheduler
 *              timing constants used by runtime orchestration.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   This header groups all task entrypoints created during system startup.
 *   It provides a stable contract between initialization code and subsystem
 *   modules that execute in dedicated FreeRTOS contexts.
 *
 * Task Responsibilities:
 *   - Connectivity lifecycle (WiFi and MQTT)
 *   - Sensor polling (I2C, gas, tank, battery)
 *   - Local UI rendering and operator interaction
 *   - Runtime supervision and watchdog feeds
 *
 * Dependencies:
 *   - Arduino core types
 *   - FreeRTOS task scheduler and task handle APIs
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

// ============================================================================
// Task Period Configuration (milliseconds)
// ============================================================================

/**
 * @brief Sampling period for weight task updates.
 */
#define PERIOD_WEIGHT       60000

// ============================================================================
// External Task Handle Declarations
// ============================================================================
// FreeRTOS task handles for runtime task management and control

extern TaskHandle_t taskWiFiHandle;             // WiFi connectivity management
extern TaskHandle_t taskMqttHandle;             // MQTT client+broker communication
extern TaskHandle_t taskWeightHandle;           // Weight sensor acquisition
extern TaskHandle_t taskGazHandle;              // Gaz/weight sensor acquisition
extern TaskHandle_t taskTankHandle;             // Tank ultrasonic acquisition
extern TaskHandle_t taskLevelHandle;            // Level/accelerometer sensor acquisition
extern TaskHandle_t taskRtcHandle;              // RTC acquisition
extern TaskHandle_t taskGpsHandle;              // GPS/GNSS acquisition
extern TaskHandle_t taskI2cHandle;              // Unified I2C sensor acquisition (DEPRECATED - use individual tasks)
extern TaskHandle_t taskBmsBleHandle;           // BLE battery management system
extern TaskHandle_t taskHmiHandle;              // HMI/display updates

// ============================================================================
// Task Function Declarations
// ============================================================================

/**
 * @brief WiFi connectivity management task.
 * 
 * Handles WiFi initialization, connection monitoring, and state management
 * for both Access Point (AP) and Station (STA) modes. Manages reconnection
 * attempts and fallback behavior when external network is unavailable.
 * 
 * @note FreeRTOS task entrypoint; does not return during normal runtime.
 */
void taskWiFi(void *pv);

/**
 * @brief Unified MQTT communication task.
 * 
 * Handles both local broker servicing and external MQTT client processing.
 * Publishes sensor data, processes incoming command messages, and manages
 * MQTT runtime health in a single FreeRTOS task.
 * 
 * @note FreeRTOS task entrypoint; does not return during normal runtime.
 */
void taskMqtt(void *pv);

/**
 * @brief Weight sensor data acquisition task.
 * 
 * Reads load cells or weight transducers at regular intervals, applies
 * averaging filters, handles tare/calibration operations, and publishes
 * weight measurements to MQTT for remote monitoring.
 * 
 * @note FreeRTOS task entrypoint; does not return during normal runtime.
 */
void taskWeight(void *pv);

/**
 * @brief Gaz bottle weight acquisition task.
 *
 * Uses the M5Stack Weight I2C unit to read gas bottle weight, applies the
 * persisted calibration gap, updates shared DATA, and publishes MQTT topics.
 *
 * @note FreeRTOS task entrypoint; does not return during normal runtime.
 */
void taskGaz(void *pv);

/**
 * @brief Unified I2C sensors acquisition task.
 *
 * Initializes and processes I2C sensors that share the Port A bus,
 * currently gas bottle weight (GAZ), tank ultrasonic level (TANK), IMU,
 * RTC, and DFRobot Gravity GPS.
 *
 * @note FreeRTOS task entrypoint; does not return during normal runtime.
 */
void taskI2c(void *pv);

/**
 * @brief Tank ultrasonic level acquisition task.
 *
 * Uses the M5Stack ultrasonic I2C unit to read distance-to-surface values,
 * derives fill percentage from configured tank mapping constants, updates
 * shared DATA, and publishes MQTT topics.
 *
 * @note FreeRTOS task entrypoint; does not return during normal runtime.
 */
void taskTank(void *pv);

/**
 * @brief Level sensor (accelerometer) acquisition task.
 *
 * Initializes and reads internal or external accelerometer (MPU or ADXL345),
 * derives device tilt/inclination angles, and publishes level measurements
 * for tank/container visualization.
 *
 * @note FreeRTOS task entrypoint; does not return during normal runtime.
 */
void taskLevel(void *pv);

/**
 * @brief Real-time clock (RTC) acquisition task.
 *
 * Initializes internal or external RTC, reads current date/time,
 * handles clock synchronization, and manages time-based operations.
 *
 * @note FreeRTOS task entrypoint; does not return during normal runtime.
 */
void taskRtc(void *pv);

/**
 * @brief GPS/GNSS acquisition task.
 *
 * Communicates with DFRobot Gravity GNSS receiver via I2C, reads
 * location and time data, and publishes GPS metrics.
 *
 * @note FreeRTOS task entrypoint; does not return during normal runtime.
 */
void taskGps(void *pv);

/**
 * @brief BLE battery management system (BMS) monitoring task.
 * 
 * Communicates with BMS devices via Bluetooth Low Energy, retrieves battery
 * voltage, current, SOC (state of charge), temperature, and health status.
 * Publishes battery metrics and manages low-power warnings.
 * 
 * @note FreeRTOS task entrypoint; does not return during normal runtime.
 */
void taskBmsBle(void *pv);

/**
 * @brief HMI display and button task.
 *
 * Runs the HMI rendering loop, handles screen navigation/calibration inputs,
 * and applies Button B long-press reboot behavior.
 *
 * @note FreeRTOS task entrypoint; does not return during normal runtime.
 */
void taskHmi(void *pv);

/**
 * @brief Hardware monitor task.
 *
 * Publishes M5 device hardware telemetry (battery, buttons, IMU) to MQTT.
 *
 * @note FreeRTOS task entrypoint; does not return during normal runtime.
 */
void taskHwMonitor(void *pv);

/**
 * @brief System watchdog and health monitoring task.
 * 
 * Monitors overall system health, detects task hangs or crashes, manages
 * watchdog timer to trigger automatic recovery, logs system statistics,
 * and implements graceful shutdown procedures if critical failures occur.
 * 
 * @note FreeRTOS task entrypoint; does not return during normal runtime.
 */
void taskWatchdog(void *pv);
