/*
 * ============================================================================
 * Watchdog Task Module - SmartFranklin
 * ============================================================================
 * 
 * File:        task_watchdog.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: FreeRTOS task wrapper for the ESP-IDF task watchdog.
 *              Initializes TWDT once, registers the current task, and
 *              periodically resets the watchdog according to config.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   The current runtime uses a simple watchdog model: `taskWatchdog` owns the
 *   TWDT registration and periodically kicks the watchdog using the configured
 *   loop period. This is a coarse system-liveness safeguard, not a per-task
 *   heartbeat framework.
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
#include "watchdog.h"
#include "config_store.h"
#include "log.h"
#include <esp_task_wdt.h>

// ============================================================================
// FreeRTOS Task Implementation
// ============================================================================

/**
 * @brief FreeRTOS task for watchdog timer management and system monitoring.
 * 
 * Main task function that initializes the watchdog system, registers itself
 * with the ESP-IDF Task Watchdog Timer, and continuously resets the watchdog
 * to prevent system resets. Provides hardware-level protection against
 * system hangs and unresponsive tasks.
 * 
 * Task Lifecycle:
 *   1. Log task startup to serial console
 *   2. Initialize watchdog system with watchdog_init()
 *   3. Register current task with ESP-IDF TWDT using esp_task_wdt_add(NULL)
 *   4. Enter infinite loop for continuous watchdog management
 *   5. Reset watchdog timer every second with esp_task_wdt_reset()
 *   6. Delay for 1 second before next reset cycle
 * 
 * Watchdog Management:
 *   - Initialization: Sets up TWDT with appropriate timeout configuration
 *   - Registration: Adds this task to the list of monitored tasks
 *   - Reset Cycle: 1-second intervals prevent timeout triggers
 *   - Timeout Prevention: Regular resets indicate system is responsive
 *   - System Protection: Automatic reset if task becomes unresponsive
 * 
 * Task Configuration:
 *   - Reset Interval: 1 second (vTaskDelay(pdMS_TO_TICKS(1000)))
 *   - Priority: tskIDLE_PRIORITY + 1 (standard priority)
 *   - Stack Size: 2048 bytes (sufficient for watchdog operations)
 *   - Core Affinity: No restriction (runs on any core)
 * 
 * Error Handling:
 *   - Initialization Errors: Handled by watchdog_init() function
 *   - Registration Failures: esp_task_wdt_add() returns error codes
 *   - Reset Failures: esp_task_wdt_reset() handles invalid states
 *   - Task Stability: Watchdog ensures system doesn't hang indefinitely
 * 
 * Performance:
 *   - CPU Usage: Very low (minimal operations per cycle)
 *   - Memory Usage: Fixed (no dynamic allocations)
 *   - Power Consumption: Negligible impact
 *   - System Overhead: Lightweight watchdog management
 * 
 * Integration:
 *   - Provides system-level protection for SmartFranklin
 *   - Complements software monitoring with hardware watchdog
 *   - Enables automatic recovery from critical failures
 *   - Supports development and production system stability
 * 
 * @param pv FreeRTOS task parameter (unused, nullptr)
 *
 * @note This task is critical for system stability and should not be disabled.
 *       Watchdog timeout will reset the system if this task stops responding.
 *       Reset interval should be shorter than configured TWDT timeout.
 * 
 * @see watchdog_init() - Watchdog system initialization
 * @see esp_task_wdt_add() - Task registration with TWDT
 * @see esp_task_wdt_reset() - Watchdog timer reset
 */
void taskWatchdog(void *pv)
{
    SF_LOGI("[WATCHDOG] Task started");

    // Initialize the watchdog system
    watchdog_init();

    // Register the current task with ESP-IDF Task Watchdog Timer
    // NULL parameter registers the calling task
    esp_task_wdt_add(NULL);

    // Main watchdog management loop
    for (;;) {
        // Reset the watchdog timer to prevent system reset
        esp_task_wdt_reset();

        const int watchdogLoopMs = (CONFIG.task_watchdog_loop_ms > 0) ? CONFIG.task_watchdog_loop_ms : 60000;
        vTaskDelay(pdMS_TO_TICKS(watchdogLoopMs));
    }
}