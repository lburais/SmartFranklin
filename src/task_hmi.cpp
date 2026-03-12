/*
 * ============================================================================
 * HMI Task Module - SmartFranklin
 * ============================================================================
 *
 * File:        task_hmi.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: FreeRTOS task wrapper for HMI runtime lifecycle.
 *              Owns task-level startup/retry behavior, periodic execution,
 *              and long-press reboot handling.
 *
 * Author:      Laurent Burais
 * Date:        12 March 2026
 * Version:     1.1
 *
 * Overview:
 *   This module binds the `HMI` class to a dedicated FreeRTOS task and keeps
 *   task orchestration concerns separate from rendering logic.
 *
 * Responsibilities:
 *   - Initialize HMI via GPS-style lifecycle (`init`/`process`)
 *   - Retry initialization when startup fails
 *   - Execute periodic HMI processing loop at fixed cadence
 *   - Handle Button B long-press reboot gesture
 *
 * Runtime model:
 *   1. Task starts and attempts HMI initialization.
 *   2. If init fails, task retries every second.
 *   3. On success, task enters steady processing loop.
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
 *
 * ============================================================================
 */

#include <Arduino.h>
#include <M5Unified.h>

#include "hmi.h"
#include "tasks.h"
#include "config_store.h"

namespace {

/** @brief Button B hold duration required to trigger reboot. */
static constexpr unsigned long HMI_REBOOT_HOLD_MS = 3000;

/** @brief Task-owned HMI runtime instance. */
HMI g_hmi;

}  // namespace

/**
 * @brief FreeRTOS HMI task entrypoint.
 *
 * Startup behavior:
 * - logs task creation
 * - repeatedly calls `g_hmi.init()` until success
 * - records heartbeat while retrying
 *
 * Steady-state behavior:
 * - executes one `g_hmi.process()` cycle
 * - handles Button B long-press reboot logic
 * - updates heartbeat and sleeps until next period
 *
 * @param pvParameters Unused task parameter.
 */
void taskHmi(void *pvParameters)
{
    (void)pvParameters;

    M5_LOGI("[HMI] Task started");

    while (!g_hmi.init()) {
        M5_LOGW("[HMI] init failed, retry in 1s");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Keep Button B long-press handling in the same task that updates buttons.
    unsigned long rebootPressStart = 0;

    for (;;) {
        g_hmi.process();

        if (M5.BtnB.isPressed()) {
            if (rebootPressStart == 0) {
                rebootPressStart = millis();
            }
            if (millis() - rebootPressStart > HMI_REBOOT_HOLD_MS) {
                M5_LOGI("----- SmartFranklin restarted -----");
                ESP.restart();
            }
        } else {
            rebootPressStart = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(CONFIG.task_hmi_loop_ms));
    }
}
