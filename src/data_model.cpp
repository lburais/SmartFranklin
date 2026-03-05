/*
 * ============================================================================
 * Global Data Model Implementation - SmartFranklin
 * ============================================================================
 * 
 * File:        data_model.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: Instantiation and initialization of global system data model.
 *              Defines the DATA object used throughout the application for
 *              real-time sensor readings, device state, and system metrics.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   SmartFranklin is a multi-threaded system with numerous concurrent tasks
 *   acquiring sensor data, managing network connections, and updating device
 *   state. The global DATA object provides a thread-safe mechanism for sharing
 *   real-time information across all modules and tasks.
 * 
 * Architecture:
 *   - Centralized Data Model: Single SOURCE OF TRUTH for all system state
 *   - Real-Time Updates: Tasks continuously update DATA with latest readings
 *   - Publishing Pipeline: MQTT and network modules read from DATA for transmission
 *   - Thread-Safe Access: Mutex (DATA_MUTEX) protects against data corruption
 * 
 * Data Categories in SmartData Object:
 * 
 *   1. Battery Management System (BMS)
 *      - bms_voltage: Battery voltage in volts (floating point)
 *      - bms_current: Battery charging/discharging current in amps
 *      - bms_soc: State of Charge percentage (0-100%)
 *      - bms_health: Battery health status (%)
 *      - Updated by: taskBmsBle (via BLE communication)
 * 
 *   2. Weight/Load Measurement
 *      - weight_raw: Raw ADC value from weight sensor (24-bit)
 *      - weight_kg: Calibrated weight in kilograms
 *      - Updated by: taskWeight (periodic sensor acquisition)
 * 
 *   3. Distance/Proximity Measurement
 *      - distance_mm: Distance in millimeters (ultrasonic or LIDAR)
 *      - Updated by: taskDistance (periodic sensor acquisition)
 * 
 *   4. Inclination/Tilt Measurement
 *      - tilt_degrees: Angle in degrees from horizontal
 *      - Updated by: taskTilt (IMU accelerometer processing)
 * 
 *   5. System Time
 *      - timestamp_unix: Current Unix timestamp (seconds since epoch)
 *      - Updated by: taskRtc (RTC synchronization)
 * 
 *   6. Device Control State
 *      - led_state: LED on/off state (boolean)
 *      - buzzer_state: Buzzer on/off state (boolean)
 *      - target_soc: Target battery state of charge for charging (0-100%)
 *      - Updated by: command_handler (remote MQTT commands)
 * 
 * Thread Safety:
 *   The global DATA_MUTEX protects against race conditions when multiple
 *   FreeRTOS tasks access DATA simultaneously:
 *   
 *   Example safe read pattern:
 *   {
 *     std::lock_guard<std::mutex> lock(DATA_MUTEX);
 *     float voltage = DATA.bms_voltage;  // Protected read
 *   }
 * 
 *   Example safe write pattern:
 *   {
 *     std::lock_guard<std::mutex> lock(DATA_MUTEX);
 *     DATA.weight_kg = 45.5;  // Protected write
 *   }
 * 
 * Dependencies:
 *   - data_model.h (SmartData structure definition and declarations)
 *   - <mutex> (C++ standard library for thread synchronization)
 * 
 * Notes:
 *   - This file contains the actual instantiation of global objects
 *   - Declaration prototypes are in data_model.h (included by all modules)
 *   - Should be compiled exactly once (avoid multiple inclusion)
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

#include "data_model.h"

// ============================================================================
// Global Data Model Instance
// ============================================================================
// Global system data object shared across all application modules.
// This is the primary DATA STORE for all real-time sensor readings, device
// state, and system metrics in SmartFranklin. All FreeRTOS tasks read from
// and write to this object for inter-task communication and data publishing.
//
// Access Pattern:
//   - Sensor tasks (taskBmsBle, taskWeight, taskDistance, etc.) continuously
//     update DATA with latest readings
//   - Communication tasks (taskMqttBroker, taskDisplay) read DATA to publish
//     or display current system state
//   - Command handlers (command_handler.cpp) modify DATA to reflect user
//     commands received via MQTT or web dashboard
//
// Thread Safety:
//   Always protect DATA access with DATA_MUTEX to prevent corruption from
//   concurrent FreeRTOS task access:
//
//   std::lock_guard<std::mutex> lock(DATA_MUTEX);
//   float current_voltage = DATA.bms_voltage;
//
// @see SmartData - Structure definition in data_model.h
// @see DATA_MUTEX - Synchronization primitive for thread-safe access
SmartData DATA;

// ============================================================================
// Global Synchronization Primitive
// ============================================================================
// Mutex protecting concurrent access to global DATA object.
// FreeRTOS tasks run concurrently across both ESP32 cores. Without proper
// synchronization, simultaneous reads/writes to DATA could corrupt data or
// produce inconsistent state values.
//
// Mutex Semantics:
//   - Only one task can hold the lock at a time (mutual exclusion)
//   - Tasks block until lock is released if already held
//   - Prevents lost writes and torn reads of complex structures
//   - std::lock_guard provides RAII-style automatic release
//
// Usage Pattern:
//   Always use lock_guard for exception-safe automatic unlock:
//
//   {
//     std::lock_guard<std::mutex> lock(DATA_MUTEX);
//     // Protected critical section
//     DATA.bms_voltage = 48.5f;
//     DATA.bms_current = 2.3f;
//   }  // Implicit unlock when lock_guard destructor called
//
//   Do NOT use manual lock/unlock (error-prone):
//
//   DATA_MUTEX.lock();
//   DATA.bms_voltage = 48.5f;
//   // if exception thrown here, unlock never happens!
//   DATA_MUTEX.unlock();
//
// Performance Considerations:
//   - Lock contention on DATA: Minimal, most tasks read more than write
//   - Critical section duration: Keep short (microseconds to milliseconds)
//   - Avoid nested locks or deadlock: Always acquire in consistent order
//
// @see DATA - Protected object synchronized by this mutex
std::mutex DATA_MUTEX;
