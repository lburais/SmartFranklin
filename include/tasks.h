/*
 * ============================================================================
 * FreeRTOS Task Declarations - SmartFranklin
 * ============================================================================
 * 
 * File:        tasks.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Central declaration point for all SmartFranklin FreeRTOS tasks.
 *              Defines prototypes for asynchronous subsystem operations.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   This header declares all FreeRTOS task functions used by SmartFranklin.
 *   Each task encapsulates an independent subsystem such as networking,
 *   sensors, BLE devices, MQTT layers, or hardware modules.
 *   
 *   Tasks are implemented in their respective modules and scheduled by
 *   FreeRTOS at runtime, providing clean separation of concerns and
 *   consistent concurrency structure across the entire system.
 * 
 * Task Responsibilities:
 *   - Managing hardware peripherals (NB-IoT, BLE, LoRa, weight sensors)
 *   - Maintaining network services (MQTT broker, WiFi)
 *   - Performing periodic data acquisition and publishing
 *   - Handling device-specific protocols and state machines
 *   - System monitoring and watchdog operations
 * 
 * Dependencies:
 *   - Arduino.h (ESP32 core library)
 *   - FreeRTOS kernel (built-in ESP32)
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
extern TaskHandle_t taskI2cHandle;              // Unified I2C sensor acquisition (GAZ + TANK + IMU + RTC + GPS)
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
 * @param pvParameters FreeRTOS task parameter (unused)
 * @return void (infinite loop, never returns)
 */
void taskWiFi(void *pvParameters);

/**
 * @brief Unified MQTT communication task.
 * 
 * Handles both local broker servicing and external MQTT client processing.
 * Publishes sensor data, processes incoming command messages, and manages
 * MQTT runtime health in a single FreeRTOS task.
 * 
 * @param pvParameters FreeRTOS task parameter (unused)
 * @return void (infinite loop, never returns)
 */
void taskMqtt(void *pvParameters);

/**
 * @brief Weight sensor data acquisition task.
 * 
 * Reads load cells or weight transducers at regular intervals, applies
 * averaging filters, handles tare/calibration operations, and publishes
 * weight measurements to MQTT for remote monitoring.
 * 
 * @param pvParameters FreeRTOS task parameter (unused)
 * @return void (infinite loop, never returns)
 */
void taskWeight(void *pvParameters);

/**
 * @brief Gaz bottle weight acquisition task.
 *
 * Uses the M5Stack Weight I2C unit to read gas bottle weight, applies the
 * persisted calibration gap, updates shared DATA, and publishes MQTT topics.
 *
 * @param pvParameters FreeRTOS task parameter (unused)
 * @return void (infinite loop, never returns)
 */
void taskGaz(void *pvParameters);

/**
 * @brief Unified I2C sensors acquisition task.
 *
 * Initializes and processes I2C sensors that share the Port A bus,
 * currently gas bottle weight (GAZ), tank ultrasonic level (TANK), IMU,
 * RTC, and DFRobot Gravity GPS.
 *
 * @param pvParameters FreeRTOS task parameter (unused)
 * @return void (infinite loop, never returns)
 */
void taskI2c(void *pvParameters);

/**
 * @brief Tank ultrasonic level acquisition task.
 *
 * Uses the M5Stack ultrasonic I2C unit to read distance-to-surface values,
 * derives fill percentage from configured tank mapping constants, updates
 * shared DATA, and publishes MQTT topics.
 *
 * @param pvParameters FreeRTOS task parameter (unused)
 * @return void (infinite loop, never returns)
 */
void taskTank(void *pvParameters);

/**
 * @brief BLE battery management system (BMS) monitoring task.
 * 
 * Communicates with BMS devices via Bluetooth Low Energy, retrieves battery
 * voltage, current, SOC (state of charge), temperature, and health status.
 * Publishes battery metrics and manages low-power warnings.
 * 
 * @param pvParameters FreeRTOS task parameter (unused)
 * @return void (infinite loop, never returns)
 */
void taskBmsBle(void *pvParameters);

/**
 * @brief HMI display and button task.
 *
 * Runs the HMI rendering loop, handles screen navigation/calibration inputs,
 * and applies Button B long-press reboot behavior.
 *
 * @param pvParameters FreeRTOS task parameter (unused)
 * @return void (infinite loop, never returns)
 */
void taskHmi(void *pvParameters);

/**
 * @brief Hardware monitor task.
 *
 * Publishes M5 device hardware telemetry (battery, buttons, IMU) to MQTT.
 *
 * @param pvParameters FreeRTOS task parameter (unused)
 * @return void (infinite loop, never returns)
 */
void taskHwMonitor(void *pvParameters);

/**
 * @brief System watchdog and health monitoring task.
 * 
 * Monitors overall system health, detects task hangs or crashes, manages
 * watchdog timer to trigger automatic recovery, logs system statistics,
 * and implements graceful shutdown procedures if critical failures occur.
 * 
 * @param pvParameters FreeRTOS task parameter (unused)
 * @return void (infinite loop, never returns)
 */
void taskWatchdog(void *pvParameters);
