/*
 * ============================================================================
 * Watchdog Module - SmartFranklin
 * ============================================================================
 * 
 * File:        watchdog.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Header file for watchdog timer functionality. Provides system
 *              health monitoring through periodic heartbeat signals from
 *              critical tasks, preventing system hangs and enabling automatic
 *              recovery through watchdog resets.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   The watchdog module implements a multi-task watchdog system for SmartFranklin,
 *   monitoring critical system components and tasks to ensure system reliability.
 *   Each monitored task periodically sends heartbeat signals ("beats") to the
 *   watchdog, and if any task fails to check in within its timeout period, the
 *   system automatically resets to recover from potential hangs or deadlocks.
 *   This provides robust operation for remote IoT devices that must operate
 *   unattended for extended periods.
 * 
 * Watchdog Architecture:
 *   - Hardware Watchdog: ESP32's built-in watchdog timers (RWDT, MWDT)
 *   - Software Layer: Task-specific heartbeat monitoring
 *   - Multi-Task Support: Individual timeouts for different tasks
 *   - Reset Recovery: Automatic system restart on timeout
 *   - Debug Support: Task status monitoring for troubleshooting
 *   - Configurable Timeouts: Adjustable timeout periods per task
 * 
 * Task Monitoring:
 *   - Task IDs: Unique identifiers for each monitored task
 *   - Heartbeat Period: Regular "beat" calls from each task
 *   - Timeout Windows: Configurable timeout for each task type
 *   - Status Tracking: Last heartbeat timestamp per task
 *   - Violation Detection: Automatic detection of missed heartbeats
 *   - Reset Trigger: System reset when critical task times out
 * 
 * Critical Tasks:
 *   - Main Loop: Core system loop task
 *   - Sensor Tasks: Data acquisition tasks (weight, distance, etc.)
 *   - Network Tasks: MQTT, WiFi, cellular connectivity
 *   - Display Tasks: UI update and user interface
 *   - Storage Tasks: Configuration and data persistence
 *   - System Tasks: Health monitoring and diagnostics
 * 
 * Timeout Configuration:
 *   - Short Timeouts: 1-5 seconds for critical real-time tasks
 *   - Medium Timeouts: 10-30 seconds for network operations
 *   - Long Timeouts: 60+ seconds for background tasks
 *   - Task-Specific: Different timeouts based on task criticality
 *   - Dynamic Adjustment: Timeouts can be adjusted based on conditions
 * 
 * Recovery Mechanism:
 *   - Hardware Reset: ESP32 watchdog triggers full system reset
 *   - Clean Restart: System reboots to known good state
 *   - State Preservation: Critical data may be lost (design consideration)
 *   - Boot Count: Increment counter for reset tracking
 *   - Diagnostic Logs: Reset cause logged for analysis
 *   - Graceful Shutdown: Attempt cleanup before reset if possible
 * 
 * Integration Points:
 *   - FreeRTOS Tasks: Each task calls watchdog_beat() periodically
 *   - Main Loop: watchdog_beat() called in Arduino loop()
 *   - Error Handling: Watchdog triggers on critical errors
 *   - Status Reporting: Watchdog status in system health monitoring
 *   - Configuration: Timeout values from CONFIG structure
 * 
 * Performance Impact:
 *   - CPU Overhead: Minimal (timestamp updates only)
 *   - Memory Usage: Small array for task status tracking
 *   - Timing Accuracy: Millisecond precision for timeout detection
 *   - Interrupt Safety: Watchdog operations are thread-safe
 *   - Power Impact: Negligible additional power consumption
 * 
 * Safety Considerations:
 *   - False Resets: Proper timeout selection prevents premature resets
 *   - Critical Tasks: Only monitor essential tasks to avoid unnecessary resets
 *   - Recovery Time: System unavailable during reset and boot process
 *   - Data Loss: Volatile data lost on reset (consider persistence)
 *   - User Experience: Unexpected resets can disrupt user interaction
 *   - Remote Access: Reset may interrupt remote connections
 * 
 * Dependencies:
 *   - ESP32 Watchdog: Hardware watchdog timer peripherals
 *   - Arduino.h: Basic timing and utility functions
 *   - FreeRTOS: Task scheduling and identification
 * 
 * Limitations:
 *   - Task Count: Limited by available watchdog timer resources
 *   - Timeout Precision: Millisecond resolution, not microsecond
 *   - No Selective Reset: Full system reset, not individual task restart
 *   - Memory Constraints: Status tracking limited by available RAM
 *   - Debug Difficulty: Reset makes debugging hanging tasks challenging
 *   - Boot Time: System unavailable during reset and restart sequence
 * 
 * Best Practices:
 *   - Conservative Timeouts: Set timeouts longer than normal operation
 *   - Regular Heartbeats: Call watchdog_beat() frequently in task loops
 *   - Task Identification: Use clear, documented task ID constants
 *   - Timeout Tuning: Adjust timeouts based on observed task behavior
 *   - Error Logging: Log before reset for post-mortem analysis
 *   - Test Scenarios: Test watchdog behavior with simulated hangs
 *   - Monitor Resets: Track reset frequency for system health assessment
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

/**
 * @brief Initializes the watchdog timer system.
 * 
 * Sets up the hardware and software watchdog infrastructure for
 * monitoring critical system tasks. Configures timeout periods,
 * initializes task status tracking, and enables watchdog protection.
 * Must be called early in system initialization.
 * 
 * Initialization Process:
 *   1. Configure ESP32 hardware watchdog timers (RWDT/MWDT)
 *   2. Set up task status tracking arrays
 *   3. Initialize timeout values for each task ID
 *   4. Enable watchdog timer interrupts
 *   5. Start background monitoring task
 *   6. Register initial heartbeat for main task
 * 
 * Hardware Configuration:
 *   - RWDT (RTC Watchdog): For deep sleep and boot monitoring
 *   - MWDT (Main Watchdog): For active task monitoring
 *   - Timeout Stages: Multiple timeout levels for different scenarios
 *   - Reset Action: Configured to trigger system reset on timeout
 *   - Clock Source: Internal timer for reliable operation
 * 
 * Task Configuration:
 *   - Task IDs: Predefined constants for each monitored task
 *   - Timeout Values: Individual timeouts loaded from CONFIG
 *   - Status Array: Timestamp storage for last heartbeat per task
 *   - Critical Tasks: Identification of tasks requiring reset on failure
 * 
 * Error Handling:
 *   - Hardware Failure: Graceful degradation if watchdog unavailable
 *   - Configuration Error: Default timeouts used if CONFIG invalid
 *   - Memory Error: Reduced functionality if allocation fails
 *   - Initialization Failure: Logged but system continues
 * 
 * Performance:
 *   - Initialization Time: < 100ms
 *   - Memory Usage: Small arrays for task status (O(number of tasks))
 *   - CPU Overhead: Minimal background monitoring
 *   - Power Impact: Negligible
 * 
 * Usage Notes:
 *   - Call once during setup() before starting tasks
 *   - Ensure CONFIG is loaded before calling
 *   - Tasks should call watchdog_beat() immediately after starting
 *   - Monitor logs for initialization status
 * 
 * @note Critical for system reliability in production deployments.
 *       Watchdog becomes active immediately after initialization.
 *       Tasks must call watchdog_beat() regularly to prevent resets.
 * 
 * @see watchdog_beat() - Heartbeat function for tasks
 */
void watchdog_init();

/**
 * @brief Sends heartbeat signal from a monitored task.
 * 
 * Updates the last heartbeat timestamp for the specified task ID,
 * indicating that the task is still active and responsive. This
 * prevents watchdog timeout and system reset. Should be called
 * regularly from each monitored task's main loop.
 * 
 * Heartbeat Process:
 *   1. Validate task ID is within valid range
 *   2. Update timestamp for the specified task
 *   3. Reset any pending timeout warnings for the task
 *   4. Optionally update hardware watchdog if configured
 *   5. Log heartbeat if debug logging enabled
 * 
 * Task ID Usage:
 *   - Predefined Constants: Use TASK_ID_* constants for consistency
 *   - Unique IDs: Each task must use unique ID (0-15 typically)
 *   - Documentation: Task IDs should be documented for maintenance
 *   - Range Checking: Invalid IDs logged but don't cause failure
 * 
 * Timing Requirements:
 *   - Frequency: Must be called more often than task timeout
 *   - Consistency: Regular intervals prevent timeout
 *   - Critical Sections: Call after completing critical operations
 *   - Error Recovery: Call after recovering from errors
 * 
 * Performance:
 *   - Execution Time: < 1ms (simple timestamp update)
 *   - Thread Safety: Atomic operations for timestamp updates
 *   - Memory Access: Minimal RAM access
 *   - No Blocking: Non-blocking operation
 * 
 * Usage Examples:
 *   // In main loop
 *   void loop() {
 *       // Do work...
 *       watchdog_beat(TASK_ID_MAIN);
 *       delay(1000);
 *   }
 * 
 *   // In FreeRTOS task
 *   void sensorTask(void *param) {
 *       watchdog_beat(TASK_ID_SENSOR);
 *       while (true) {
 *           // Sensor processing...
 *           watchdog_beat(TASK_ID_SENSOR);
 *           vTaskDelay(pdMS_TO_TICKS(5000));
 *       }
 *   }
 * 
 * Integration:
 *   - Task Loops: Called at regular intervals in task loops
 *   - Error Handlers: Called after error recovery
 *   - State Changes: Called when task enters stable state
 *   - Debug Points: Useful for monitoring task execution
 * 
 * @note Function must be called more frequently than the task timeout.
 *       Use predefined TASK_ID_* constants for task identification.
 *       Missing heartbeats will trigger system reset after timeout.
 * 
 * @see watchdog_init() - Required initialization
 * @see TASK_ID_* - Predefined task ID constants
 */
void watchdog_beat(int id);