/**
 * ============================================================================
 * Watchdog Module - SmartFranklin
 * ============================================================================
 * 
 * File:        watchdog.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Header file for watchdog timer functionality. Provides system
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 * 
 * Watchdog Architecture:
 * 
 * Task Monitoring:
 * 
 * Critical Tasks:
 * 
 * Timeout Configuration:
 * 
 * Recovery Mechanism:
 * 
 * Integration Points:
 * 
 * Performance Impact:
 * 
 * Safety Considerations:
 * 
 * Dependencies:
 * 
 * Limitations:
 * 
 * Best Practices:
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
 * 
 * Hardware Configuration:
 * 
 * Task Configuration:
 * 
 * Error Handling:
 * 
 * Performance:
 * 
 * Usage Notes:
 * 
 * @note Critical for system reliability in production deployments.
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
 * 
 * Task ID Usage:
 * 
 * Timing Requirements:
 * 
 * Performance:
 * 
 * Usage Examples:
 * 
 * 
 * Integration:
 * 
 * @note Function must be called more frequently than the task timeout.
 * 
 * @see watchdog_init() - Required initialization
 * @see TASK_ID_* - Predefined task ID constants
 */
void watchdog_beat(int id);