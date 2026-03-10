/*
 * ============================================================================
 * GPS Task Module - SmartFranklin
 * ============================================================================
 *
 * File:        task_gps.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: FreeRTOS task for Gravity DFR1103 GNSS/RTC lifecycle.
 *              Initializes the GPS module, runs periodic processing, and
 *              exposes task status over MQTT.
 *
 * Author:      Laurent Burais
 * Date:        10 March 2026
 * Version:     1.0
 *
 * Overview:
 *   This module is the scheduler-facing wrapper for the GPS subsystem.
 *   It owns task lifecycle behavior (startup, init retry, periodic loop)
 *   while the GPS class handles low-level bus access and data acquisition.
 *
 * Responsibilities:
 *   - Start and supervise GPS module initialization.
 *   - Retry initialization every 10 seconds if hardware is unavailable.
 *   - Trigger one GPS processing cycle every PERIOD_GPS milliseconds.
 *   - Publish task-level health/running/initialized status to MQTT.
 *   - Provide liveness checks via isGpsTaskRunning().
 *
 * Task State Model:
 *   - g_gpsTaskStarted:
 *       true only after GPS_MODULE.init() succeeds.
 *   - g_lastGpsLoopMs:
 *       heartbeat timestamp updated every loop iteration.
 *   - isGpsTaskRunning():
 *       combines FreeRTOS task state, heartbeat freshness,
 *       init state, and GPS module health in one boolean.
 *
 * MQTT Status Topics:
 *   - smartfranklin/gps/task/running
 *   - smartfranklin/gps/task/healthy
 *   - smartfranklin/gps/task/initialized
 *
 * Startup Behavior:
 *   1. Publish all status topics as "0".
 *   2. Loop on GPS_MODULE.init() until successful.
 *   3. Publish initialized="1" once startup is complete.
 *   4. Enter periodic operational loop.
 *
 * Runtime Behavior:
 *   - Every PERIOD_GPS:
 *       call GPS_MODULE.process(), refresh heartbeat, recompute status,
 *       and publish running/healthy topics.
 *   - The loop is non-blocking except for deterministic vTaskDelay().
 *
 * Error Handling:
 *   - Initialization failures do not crash task; they trigger retry wait.
 *   - Stale heartbeat or unhealthy module immediately flips running status.
 *   - Null task handle check protects status helper before task creation.
 *
 * Dependencies:
 *   - gps.h (GPS module singleton API)
 *   - tasks.h (task handle + period constants)
 *   - mqtt_layer.h (status topic publishing)
 *   - M5Unified (logging)
 *
 * Limitations:
 *   - Initialization retry delay is fixed to 10s.
 *   - Status topics are periodic and may briefly lag true task transitions.
 *   - No exponential backoff or persistent failure counter yet.
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
#include <M5Unified.h>

#include "tasks.h"
#include "gps.h"
#include "mqtt_layer.h"
#include "pahub_channels.h"

namespace {
// ============================================================================
// Task-Private Runtime State
// ============================================================================

/**
 * @brief Tracks whether taskGps completed module initialization.
 *
 * This flag is independent from FreeRTOS scheduler state and reflects
 * successful startup of the DFR1103 module.
 */
volatile bool g_gpsTaskStarted = false;

/**
 * @brief Millisecond timestamp of the latest task loop iteration.
 *
 * Used as a heartbeat for liveness checks in isGpsTaskRunning().
 */
volatile uint32_t g_lastGpsLoopMs = 0;
}  // namespace

/**
 * @brief Reports whether the GPS task is alive and the module is healthy.
 *
 * Conditions checked:
 * - task handle exists,
 * - scheduler state is running/ready/blocked,
 * - loop heartbeat is recent,
 * - GPS module reports healthy.
 *
 * @return true when task state, heartbeat, and module health are valid.
 */
bool isGpsTaskRunning()
{
    // Handle may be null early during boot before task is created.
    if (taskGpsHandle == nullptr) {
        return false;
    }

    // Accept normal operational scheduler states as active.
    const eTaskState state = eTaskGetState(taskGpsHandle);
    const bool activeState = (state == eRunning) || (state == eReady) || (state == eBlocked);

    // Heartbeat timeout allows normal jitter and occasional long cycles.
    const bool recentLoop = (millis() - g_lastGpsLoopMs) <= ((PERIOD_GPS * 2UL) + 5000UL);

    // Running status is valid only when task state and module health agree.
    return g_gpsTaskStarted && activeState && recentLoop && GPS_MODULE.isHealthy();
}

/**
 * @brief FreeRTOS task for DFR1103 lifecycle and status publishing.
 *
 * Task flow:
 * 1. Publish startup status (all zero).
 * 2. Retry init() every 10s until module becomes available.
 * 3. Publish initialized state.
 * 4. Loop: process one GPS sample, update heartbeat, publish running/healthy.
 *
 * MQTT status topics:
 * - smartfranklin/gps/task/running
 * - smartfranklin/gps/task/healthy
 * - smartfranklin/gps/task/initialized
 *
 * @param pv Unused task parameter.
 */
void taskGps(void *pv)
{
    (void)pv;
    M5_LOGI("[GPS] Task started");

    // Initialize public task status as offline before first hardware contact.
    g_gpsTaskStarted = false;
    sf_mqtt::publish("smartfranklin/gps/task/running", "0");
    sf_mqtt::publish("smartfranklin/gps/task/healthy", "0");
    sf_mqtt::publish("smartfranklin/gps/task/initialized", "0");

    // Keep retrying until the module is physically reachable and initialized.
    while (!GPS_MODULE.init()) {
        M5_LOGW("[GPS] init failed, retry in 10s");
        sf_mqtt::publish("smartfranklin/gps/task/running", "0");
        sf_mqtt::publish("smartfranklin/gps/task/healthy", "0");
        sf_mqtt::publish("smartfranklin/gps/task/initialized", "0");
        g_lastGpsLoopMs = millis();
        vTaskDelay(pdMS_TO_TICKS(10000));
    }

    // Mark successful startup once init() completes.
    g_gpsTaskStarted = true;
    sf_mqtt::publish("smartfranklin/gps/task/initialized", "1");

    for (;;) {
        // Trigger one full sample/update/publish cycle.
        GPS_MODULE.process();

        // Update heartbeat after each loop iteration.
        g_lastGpsLoopMs = millis();

        // Recompute and publish current task runtime state.
        const bool running = isGpsTaskRunning();
        sf_mqtt::publish("smartfranklin/gps/task/running", running ? "1" : "0");
        sf_mqtt::publish("smartfranklin/gps/task/healthy", GPS_MODULE.isHealthy() ? "1" : "0");

        // Periodic scheduling cadence for GPS processing.
        vTaskDelay(pdMS_TO_TICKS(PERIOD_GPS));
    }
}
