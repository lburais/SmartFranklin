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
 *   - Maintaining network services (MQTT broker, bridges, WiFi, Meshtastic)
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
// External Task Handle Declarations
// ============================================================================
// FreeRTOS task handles for runtime task management and control

extern TaskHandle_t taskWiFiHandle;             // WiFi connectivity management
extern TaskHandle_t taskMqttBrokerHandle;       // MQTT broker communication
extern TaskHandle_t taskDistanceHandle;         // Distance sensor acquisition
extern TaskHandle_t taskWeightHandle;           // Weight sensor acquisition
extern TaskHandle_t taskTiltHandle;             // Tilt sensor acquisition
extern TaskHandle_t taskRtcHandle;              // Real-time clock synchronization
extern TaskHandle_t taskBmsBleHandle;           // BLE battery management system
extern TaskHandle_t taskDisplayHandle;          // M5Stack display updates
extern TaskHandle_t taskMeshtasticBridgeHandle; // Meshtastic mesh networking bridge
extern TaskHandle_t taskNbiotHandle;            // NB-IoT cellular communication

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
 * @brief MQTT broker communication task.
 * 
 * Maintains connection to external MQTT broker, publishes sensor data,
 * processes incoming command messages, and manages message queues.
 * Critical for cloud integration and remote device control.
 * 
 * @param pvParameters FreeRTOS task parameter (unused)
 * @return void (infinite loop, never returns)
 */
void taskMqttBroker(void *pvParameters);

/**
 * @brief Distance sensor data acquisition task.
 * 
 * Periodically reads distance sensor (ultrasonic/LIDAR), processes raw values,
 * applies calibration, and publishes measurements to MQTT topics.
 * Used for tank level, proximity detection, or distance monitoring.
 * 
 * @param pvParameters FreeRTOS task parameter (unused)
 * @return void (infinite loop, never returns)
 */
void taskDistance(void *pvParameters);

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
 * @brief Tilt/Angle sensor data acquisition task.
 * 
 * Acquires tilt angle measurements from accelerometer or inclinometer,
 * applies sensor fusion algorithms, and publishes inclination data
 * for slope monitoring or equipment orientation tracking.
 * 
 * @param pvParameters FreeRTOS task parameter (unused)
 * @return void (infinite loop, never returns)
 */
void taskTilt(void *pvParameters);

/**
 * @brief Real-time clock synchronization task.
 * 
 * Maintains system time using M5Stack's internal RTC, synchronizes with
 * NTP servers over network, manages time-based events, and provides
 * accurate timestamps for data logging and scheduled operations.
 * 
 * @param pvParameters FreeRTOS task parameter (unused)
 * @return void (infinite loop, never returns)
 */
void taskRtc(void *pvParameters);

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
 * @brief M5Stack display rendering task.
 * 
 * Updates device display with real-time system status, sensor values,
 * WiFi/MQTT connection state, battery level, and user interface elements.
 * Handles touch input and display refresh cycles.
 * 
 * @param pvParameters FreeRTOS task parameter (unused)
 * @return void (infinite loop, never returns)
 */
void taskDisplay(void *pvParameters);

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
 * @brief Meshtastic mesh networking bridge task.
 * 
 * Manages LoRa mesh network connectivity via Meshtastic protocol,
 * relays messages between local mesh and MQTT cloud, provides redundant
 * long-range communication in areas without WiFi coverage.
 * 
 * @param pvParameters FreeRTOS task parameter (unused)
 * @return void (infinite loop, never returns)
 */
void taskMeshtasticBridge(void *pvParameters);

/**
 * @brief NB-IoT cellular communication task.
 * 
 * Handles 4G LTE-M/NB-IoT modem operations for cellular backup connectivity,
 * manages SIM card operations, publishes data over cellular network when
 * WiFi unavailable, and handles cellular-specific protocols and AT commands.
 * 
 * @param pvParameters FreeRTOS task parameter (unused)
 * @return void (infinite loop, never returns)
 */
void taskNbiot(void *pvParameters);

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
