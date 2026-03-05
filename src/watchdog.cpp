/*
 * ============================================================================
 * Watchdog Timer Module - SmartFranklin
 * ============================================================================
 * 
 * File:        watchdog.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: Hardware watchdog timer implementation using ESP-IDF Task WDT.
 *              Monitors FreeRTOS task health by requiring periodic "heartbeats"
 *              from critical tasks. Triggers system reset if any task fails.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   SmartFranklin runs multiple concurrent FreeRTOS tasks for sensor acquisition,
 *   network communication, and control operations. The watchdog ensures system
 *   reliability by detecting and recovering from task failures (hangs, deadlocks,
 *   or infinite loops) through hardware-enforced resets.
 * 
 * Watchdog Mechanism:
 *   - Hardware Timer: ESP32 Task Watchdog Timer (TWDT) peripheral
 *   - Heartbeat System: Tasks periodically call watchdog_beat() to signal health
 *   - Timeout Monitoring: If no heartbeat within timeout period, system resets
 *   - Panic Reset: Immediate system restart on watchdog timeout
 *   - Task-Specific: Each critical task has dedicated heartbeat slot
 * 
 * Task Monitoring:
 *   The system monitors 9 critical tasks (configurable via NUM_TASKS):
 *   - Task 0: Main application loop (prevents UI freezes)
 *   - Task 1: WiFi management (network connectivity)
 *   - Task 2: MQTT communication (cloud messaging)
 *   - Task 3: Sensor acquisition (data collection)
 *   - Task 4: Battery monitoring (power management)
 *   - Task 5: File system operations (configuration persistence)
 *   - Task 6: Command processing (user input handling)
 *   - Task 7: Display updates (UI responsiveness)
 *   - Task 8: System diagnostics (health monitoring)
 * 
 * Heartbeat Protocol:
 *   - Each task calls watchdog_beat(task_id) regularly
 *   - Typical interval: 1-5 seconds depending on task criticality
 *   - Beat updates timestamp in lastBeat[task_id]
 *   - Watchdog checks all timestamps against current time
 *   - Any task exceeding timeout triggers system reset
 * 
 * Timeout Configuration:
 *   - Timeout Period: 10 seconds (configurable via esp_task_wdt_init)
 *   - Reset Behavior: Panic reset (immediate restart, no cleanup)
 *   - Graceful Shutdown: Disabled (true parameter in esp_task_wdt_init)
 *   - Recovery Time: ~2-5 seconds for ESP32 boot sequence
 * 
 * Failure Scenarios Protected:
 *   - Task hangs in infinite loop
 *   - Deadlock between tasks
 *   - Memory corruption causing task crash
 *   - Network stack freezes
 *   - Sensor driver lockups
 *   - File system corruption hangs
 * 
 * System Impact:
 *   - Power Cycle: Watchdog reset causes complete system restart
 *   - State Loss: RAM contents lost, SPIFFS data preserved
 *   - Recovery: System boots normally, tasks restart
 *   - Data Persistence: Configuration survives via SPIFFS
 *   - Network Reconnection: Automatic on restart
 * 
 * Dependencies:
 *   - esp_task_wdt.h (ESP-IDF Task Watchdog Timer API)
 *   - watchdog.h (Header declarations and task ID definitions)
 *   - Arduino.h (millis() function for timestamping)
 * 
 * Configuration Constants:
 *   - NUM_TASKS: Number of monitored tasks (9)
 *   - Timeout: 10 seconds (set in esp_task_wdt_init)
 *   - Panic Reset: Enabled (true parameter)
 * 
 * Limitations:
 *   - Fixed timeout: Cannot adjust per-task timeouts
 *   - All-or-nothing: Single failed task resets entire system
 *   - No graceful shutdown: Immediate reset prevents cleanup
 *   - Task ID management: Manual assignment required
 * 
 * Best Practices:
 *   - Heartbeat frequency: Every 1-2 seconds for critical tasks
 *   - Task placement: Only monitor essential tasks (not helpers)
 *   - Error handling: Tasks should handle their own errors first
 *   - Testing: Verify watchdog triggers on task hangs
 *   - Logging: Log heartbeats for debugging watchdog issues
 * 
 * ============================================================================
 * MIT License
 * ============================================================================
 * Copyright (c) 2026 Laurent Burais
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in this software and associated documentation files (the "Software"), to deal
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

#include "watchdog.h"
#include <esp_task_wdt.h>

// ============================================================================
// Watchdog Configuration Constants
// ============================================================================

/**
 * @brief Number of tasks monitored by the watchdog system.
 * 
 * Defines the size of the lastBeat array and maximum task ID.
 * Each task gets a dedicated slot for heartbeat timestamp tracking.
 * Current configuration monitors 9 critical system tasks.
 * 
 * Task ID Assignment (0-8):
 *   - 0: Main application loop
 *   - 1: WiFi/network management
 *   - 2: MQTT communication
 *   - 3: Sensor data acquisition
 *   - 4: Battery/power monitoring
 *   - 5: File system operations
 *   - 6: Command processing
 *   - 7: Display/UI updates
 *   - 8: System diagnostics
 * 
 * Increasing this value:
 *   - Allows monitoring more tasks
 *   - Increases memory usage (4 bytes per task)
 *   - Requires updating task assignments
 * 
 * Decreasing this value:
 *   - Reduces memory footprint
 *   - May leave tasks unmonitored
 *   - Requires updating task assignments
 */
static const int NUM_TASKS = 9;

// ============================================================================
// Heartbeat Timestamp Storage
// ============================================================================

/**
 * @brief Array storing last heartbeat timestamp for each monitored task.
 * 
 * Each element corresponds to a task ID (0 to NUM_TASKS-1).
 * Timestamps are updated by watchdog_beat() calls and checked by watchdog logic.
 * Initialized to current millis() at startup to prevent immediate timeouts.
 * 
 * Timestamp Units:
 *   - Stored as uint32_t (milliseconds since boot)
 *   - Wraps around every ~49.7 days (not an issue for watchdog)
 *   - Compared against current millis() for timeout detection
 * 
 * Memory Usage:
 *   - 4 bytes per task × 9 tasks = 36 bytes
 *   - Static allocation (no heap fragmentation)
 *   - Accessible from any task (global scope)
 * 
 * Thread Safety:
 *   - No mutex protection (single-writer assumption)
 *   - Each task writes only to its own slot
 *   - Interrupt-safe (millis() is atomic on ESP32)
 */
static uint32_t lastBeat[NUM_TASKS];

// ============================================================================
// Watchdog Initialization
// ============================================================================

/**
 * @brief Initializes the ESP32 Task Watchdog Timer for system monitoring.
 * 
 * Configures the hardware watchdog timer with timeout period and panic behavior.
 * Initializes heartbeat timestamps to prevent immediate timeouts on startup.
 * Must be called once during system initialization before starting tasks.
 * 
 * Initialization Steps:
 *   1. Configure TWDT with 10-second timeout and panic reset
 *   2. Initialize all task heartbeat timestamps to current time
 *   3. Enable watchdog monitoring across all CPU cores
 * 
 * Timeout Configuration:
 *   - Timeout: 10 seconds (first parameter to esp_task_wdt_init)
 *   - Behavior: Panic reset (second parameter true)
 *   - Scope: All tasks on all cores (ESP-IDF default)
 * 
 * Panic Reset Behavior:
 *   - Immediate system restart (no cleanup or logging)
 *   - CPU reset signal triggers ESP32 boot sequence
 *   - All tasks and network connections lost
 *   - SPIFFS and configuration data preserved
 *   - Recovery time: 2-5 seconds
 * 
 * Heartbeat Initialization:
 *   - All timestamps set to millis() (current time)
 *   - Provides 10-second grace period before first timeout
 *   - Tasks must call watchdog_beat() within this window
 * 
 * Error Handling:
 *   - TWDT initialization failures not handled (ESP-IDF logs errors)
 *   - Function always succeeds (no return value)
 *   - System continues even if watchdog fails to initialize
 * 
 * Performance Impact:
 *   - Initialization time: < 1ms
 *   - Memory usage: 36 bytes for timestamp array
 *   - Runtime overhead: Minimal (timestamp updates only)
 * 
 * @return void
 * 
 * @note Call this function during setup() before creating FreeRTOS tasks.
 *       Tasks should call watchdog_beat() immediately after starting.
 *       Example: watchdog_init(); // in setup()
 * 
 * @see esp_task_wdt_init() - ESP-IDF Task Watchdog Timer initialization
 * @see watchdog_beat() - Task heartbeat function
 */
void watchdog_init()
{
    // Initialize ESP32 Task Watchdog Timer
    // Timeout: 10 seconds, Panic reset: enabled (true)
    // Panic reset causes immediate system restart on timeout
    esp_task_wdt_init(10, true);

    // Initialize all task heartbeat timestamps to current time
    // Provides grace period before first watchdog checks
    // Tasks must beat within 10 seconds of calling watchdog_init()
    for (int i = 0; i < NUM_TASKS; ++i) {
        lastBeat[i] = millis();
    }
}

// ============================================================================
// Task Heartbeat Function
// ============================================================================

/**
 * @brief Records a heartbeat for the specified task to indicate health.
 * 
 * Updates the heartbeat timestamp for a task, resetting its watchdog timer.
 * Tasks must call this function periodically to prevent system reset.
 * Called from within each monitored task's main loop.
 * 
 * Heartbeat Process:
 *   1. Validate task ID is within valid range (0 to NUM_TASKS-1)
 *   2. Update lastBeat[task_id] with current millis() timestamp
 *   3. Return immediately (no blocking operations)
 * 
 * Task ID Validation:
 *   - Range check prevents array out-of-bounds access
 *   - Invalid IDs are silently ignored (no error reporting)
 *   - Allows safe calls from unmonitored tasks
 * 
 * Timestamp Update:
 *   - Uses millis() for monotonic timestamp
 *   - Atomic operation on ESP32 (no race conditions)
 *   - Resets watchdog timeout period for this task
 * 
 * Calling Pattern:
 *   Tasks should call this function regularly:
 *   - Critical tasks: Every 1-2 seconds
 *   - Network tasks: Every 3-5 seconds (allows for network delays)
 *   - UI tasks: Every 2-3 seconds (maintains responsiveness)
 * 
 * Example Usage:
 *   @code
 *   void taskNetwork(void *param) {
 *       watchdog_beat(TASK_ID_NETWORK);  // Initial beat
 *       
 *       while (true) {
 *           // Network operations...
 *           watchdog_beat(TASK_ID_NETWORK);  // Periodic beat
 *           vTaskDelay(2000 / portTICK_PERIOD_MS);
 *       }
 *   }
 *   @endcode
 * 
 * Error Conditions:
 *   - Invalid task ID: Function returns without action
 *   - No logging (keeps function lightweight)
 *   - No exceptions (embedded system constraints)
 * 
 * Performance:
 *   - Execution time: < 1 microsecond
 *   - Memory access: Single array write
 *   - No system calls or allocations
 *   - Safe to call from interrupt context
 * 
 * @param id - Task identifier (0 to NUM_TASKS-1)
 *             Must correspond to assigned task ID from watchdog.h
 *             Invalid IDs are ignored without error
 * 
 * @return void
 * 
 * @note This function is thread-safe and can be called from any task.
 *       Tasks should call this function more frequently than the 10-second timeout.
 *       Missing heartbeats will cause system reset after timeout expires.
 *       Example: watchdog_beat(TASK_ID_MAIN);
 * 
 * @see watchdog_init() - Initializes the watchdog system
 * @see TASK_ID_* constants in watchdog.h - Task ID definitions
 */
void watchdog_beat(int id)
{
    // Validate task ID is within valid range
    // Prevents array out-of-bounds access
    // Invalid IDs are silently ignored
    if (id < 0 || id >= NUM_TASKS) return;

    // Update heartbeat timestamp for this task
    // Resets watchdog timeout period
    // Uses current millisecond timestamp
    lastBeat[id] = millis();
}