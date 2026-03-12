/*
 * ============================================================================
 * GPS Task Module - SmartFranklin
 * ============================================================================
 *
 * File:        task_gps.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: FreeRTOS task wrapper for Gravity DFR1103 GNSS/RTC lifecycle.
 *              Owns startup retry and periodic processing cadence while the
 *              GPS module implementation handles hardware I2C operations.
 *
 * Author:      Laurent Burais
 * Date:        12 March 2026
 * Version:     1.1
 *
 * Overview:
 *   This module provides the scheduler-facing task entrypoint for GPS.
 *   Its responsibilities are intentionally narrow and deterministic:
 *   - wait until GPS module initialization succeeds,
 *   - retry initialization on failure,
 *   - run one GPS processing cycle at a configurable period.
 *
 * Runtime Model:
 *   1. Task starts and logs startup.
 *   2. Calls GPS_MODULE.init() until it returns true.
 *   3. Waits 10 seconds between failed init attempts.
 *   4. Enters infinite processing loop:
 *        - call GPS_MODULE.process()
 *        - delay for configured loop period.
 *
 * Configuration Integration:
 *   - CONFIG.task_gps_loop_ms:
 *       Desired loop period in milliseconds.
 *       If value is non-positive, a safe fallback period is applied.
 *
 * Notes:
 *   - This file intentionally does not publish task status topics.
 *   - The retry delay is fixed to keep startup behavior predictable.
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

#include "gps.h"
#include "config_store.h"

namespace {

/** @brief Fixed delay between failed GPS init attempts. */
static constexpr uint32_t GPS_INIT_RETRY_MS = 10000UL;

/** @brief Safe default loop period when config value is invalid. */
static constexpr uint32_t GPS_LOOP_FALLBACK_MS = 60000UL;

/**
 * @brief Returns validated GPS loop period in milliseconds.
 *
 * Uses persisted config when valid, otherwise applies a conservative fallback.
 */
uint32_t gpsLoopMs()
{
    const int configuredMs = CONFIG.task_gps_loop_ms;
    return (configuredMs > 0) ? static_cast<uint32_t>(configuredMs)
                              : GPS_LOOP_FALLBACK_MS;
}

}  // namespace

/**
 * @brief FreeRTOS task for DFR1103 lifecycle.
 *
 * Task flow:
 * 1. Retry init() every 10s until module becomes available.
 * 2. Loop: process one GPS sample.
 *
 * @param pv Unused task parameter.
 */
void taskGps(void *pv)
{
    (void)pv;
    M5_LOGI("[GPS] Task started");

    // Keep retrying until the module is physically reachable and initialized.
    while (!GPS_MODULE.init()) {
        M5_LOGW("[GPS] init failed, retry in 10s");
        vTaskDelay(pdMS_TO_TICKS(GPS_INIT_RETRY_MS));
    }

    for (;;) {
        // Trigger one full sample/update/publish cycle.
        GPS_MODULE.process();

        // Periodic scheduling cadence for GPS processing.
        vTaskDelay(pdMS_TO_TICKS(gpsLoopMs()));
    }
}
