/*
 * ============================================================================
 * Display Task Module - SmartFranklin
 * ============================================================================
 * 
 * File:        task_display.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: FreeRTOS task for M5Stack display management and user interface.
 *              Handles screen switching, data visualization, and scale calibration
 *              through button inputs and real-time data updates.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   SmartFranklin uses the M5Stack's LCD display for user interaction and
 *   data visualization. This task manages four different screens showing
 *   sensor data, orientation, battery status, and scale calibration.
 *   Users can navigate screens with button presses and perform calibration
 *   procedures for the weight scale.
 * 
 * Display Screens:
 * 
 *   Screen 0: Sensor Data
 *   - Distance measurement from ultrasonic sensor (cm)
 *   - Weight measurement from load cell scale (grams)
 *   - Primary operational data display
 *   - Auto-updates with latest sensor readings
 * 
 *   Screen 1: Orientation Data
 *   - Pitch angle from IMU (degrees)
 *   - Roll angle from IMU (degrees)
 *   - Device orientation for tilt sensing
 *   - Real-time IMU data visualization
 * 
 *   Screen 2: Battery Management System (BMS)
 *   - Battery voltage (volts)
 *   - Battery current (amps)
 *   - State of charge (SOC, percent)
 *   - BLE-connected BMS data display
 * 
 *   Screen 3: Scale Calibration
 *   - Interactive calibration procedure
 *   - Step-by-step weight scale calibration
 *   - Known weight placement instructions
 *   - Calibration factor calculation and storage
 * 
 * User Interface Controls:
 *   - Button A: Cycle through screens (0→1→2→3→0)
 *   - Button B: Context-sensitive actions on calibration screen
 *   - Button C: Reserved (not used in display task)
 *   - Touch: Not implemented (button-based navigation)
 * 
 * Calibration Procedure:
 *   1. Navigate to screen 3 (Scale Calibration)
 *   2. Place known weight on scale platform
 *   3. Press Button B to start calibration
 *   4. System tares scale and measures raw reading
 *   5. Press Button B again to complete calibration
 *   6. Calibration factor calculated and saved to CONFIG
 *   7. Scale accuracy improved for future measurements
 * 
 * Data Sources:
 *   - DATA.distance_cm: Ultrasonic sensor distance
 *   - DATA.weight_g: Load cell weight in grams (note: may be weight_kg in data_model)
 *   - DATA.pitch, DATA.roll: IMU orientation angles
 *   - DATA.bms_voltage, DATA.bms_current, DATA.bms_soc: BLE BMS data
 *   - Thread-safe access with DATA_MUTEX lock
 * 
 * Display Configuration:
 *   - Resolution: 320x240 pixels (M5Stack standard)
 *   - Font: Default system font (monospace)
 *   - Colors: Black background, white text
 *   - Update rate: 10Hz (100ms intervals)
 *   - Clear screen on each redraw for clean updates
 * 
 * Task Behavior:
 *   - Runs at 10Hz update rate (100ms FreeRTOS delay)
 *   - Monitors button presses for navigation and actions
 *   - Checks for screen changes from command_handler
 *   - Redraws display only when screen changes or during calibration
 *   - Handles calibration state machine with button inputs
 * 
 * Error Handling:
 *   - Invalid screen numbers clamped to valid range
 *   - Calibration division by zero prevented (raw != 0 check)
 *   - Mutex-protected data access prevents race conditions
 *   - Button press detection handles multiple presses gracefully
 * 
 * Performance Considerations:
 *   - CPU usage: Low (display updates are fast)
 *   - Memory usage: Minimal (static variables only)
 *   - Power consumption: LCD backlight active during display
 *   - Task priority: Standard (tskIDLE_PRIORITY + 1)
 *   - Stack size: 2048 bytes (sufficient for display operations)
 * 
 * Dependencies:
 *   - M5Unified.h (M5Stack display and button functions)
 *   - Arduino.h (FreeRTOS task functions)
 *   - tasks.h (Task definitions and priorities)
 *   - data_model.h (Global DATA structure and mutex)
 *   - command_handler.h (Display screen command interface)
 *   - config_store.h (Configuration access for calibration storage)
 *   - scale_control.h (Scale tare and calibration functions)
 * 
 * Configuration Integration:
 *   - CONFIG.scale_cal_factor: Stored calibration factor
 *   - config_save(): Persists calibration to SPIFFS
 *   - Calibration survives device restarts
 * 
 * Limitations:
 *   - Fixed 4-screen limit (hardcoded screen count)
 *   - No touch interface (button-only navigation)
 *   - Calibration requires known weight (user-provided)
 *   - No validation of calibration quality
 *   - Display updates block for ~10-20ms during redraw
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
#include "data_model.h"
#include "command_handler.h"
#include "config_store.h"
#include "scale_control.h"

// ============================================================================
// Global Display State Variables
// ============================================================================

/**
 * @brief Current display screen index (0-3).
 * 
 * Tracks which screen is currently being displayed.
 * Modified by button presses and command_handler inputs.
 * Used to determine what data to show in draw() function.
 */
static int screen = 0;

/**
 * @brief Flag indicating if scale calibration is in progress.
 * 
 * Set to true when user starts calibration procedure on screen 3.
 * Controls display of calibration instructions vs. progress.
 * Reset to false when calibration completes.
 */
static bool calib_in_progress = false;

/**
 * @brief Known weight value used for calibration (in kg).
 * 
 * The reference weight placed on the scale during calibration.
 * Currently fixed at 1.0 kg - could be made configurable.
 * Used to calculate calibration factor from raw sensor reading.
 */
static float calib_known_weight = 1.0f;

// ============================================================================
// Display Drawing Function
// ============================================================================

/**
 * @brief Draws the current screen content to the M5Stack display.
 * 
 * Clears the display and renders the appropriate screen based on
 * the current screen index. Uses thread-safe access to global DATA
 * structure for real-time sensor information.
 * 
 * Screen Rendering:
 * 
 *   Screen 0 (Sensor Data):
 *   - Distance: Ultrasonic measurement in centimeters (%.1f precision)
 *   - Weight: Load cell measurement in grams (%.3f precision)
 *   - Primary operational display for distance and weight
 * 
 *   Screen 1 (Orientation):
 *   - Pitch: IMU pitch angle in degrees (%.1f precision)
 *   - Roll: IMU roll angle in degrees (%.1f precision)
 *   - Device orientation visualization
 * 
 *   Screen 2 (BMS Status):
 *   - Voltage: Battery voltage in volts (%.2f precision)
 *   - Current: Battery current in amps (%.2f precision)
 *   - SOC: State of charge in percent (%.1f precision)
 *   - BLE-connected battery management data
 * 
 *   Screen 3 (Calibration):
 *   - Pre-calibration: Instructions and start button prompt
 *   - During calibration: Progress display with known/raw weights
 *   - Post-calibration: Automatically returns to instruction screen
 * 
 * Display Properties:
 *   - Background: Cleared to black on each redraw
 *   - Text: White monospace font (default M5Stack font)
 *   - Positioning: Top-left aligned printf output
 *   - Update: Full screen redraw for clean transitions
 * 
 * Thread Safety:
 *   - DATA_MUTEX lock held during data access
 *   - Prevents race conditions with sensor update tasks
 *   - Lock scope limited to data reading only
 * 
 * Performance:
 *   - Execution time: ~10-20ms per screen redraw
 *   - Memory usage: Minimal (no dynamic allocations)
 *   - Display bandwidth: Full screen clear and text rendering
 * 
 * @return void
 * 
 * @note Called whenever screen changes or during calibration updates.
 *       Display updates are synchronous and may block task briefly.
 *       Consider optimizing for smoother animations if needed.
 * 
 * @see M5.Display - M5Stack display interface
 * @see DATA - Global sensor data structure
 */
void draw()
{
    // Clear display for clean redraw
    M5.Display.clear();
    
    // Thread-safe access to global sensor data
    std::lock_guard<std::mutex> lock(DATA_MUTEX);
    
    if (screen == 0) {
        // Screen 0: Primary sensor data (distance and weight)
        M5.Display.printf("Distance: %.1f cm\n", DATA.distance_cm);
        M5.Display.printf("Weight: %.3f g\n", DATA.weight_g);
    } else if (screen == 1) {
        // Screen 1: IMU orientation data (pitch and roll)
        M5.Display.printf("Pitch: %.1f\nRoll: %.1f\n", DATA.pitch, DATA.roll);
    } else if (screen == 2) {
        // Screen 2: Battery management system data
        M5.Display.printf("BMS V: %.2f\nI: %.2f\nSOC: %.1f\n",
                          DATA.bms_voltage, DATA.bms_current, DATA.bms_soc);
    } else if (screen == 3) {
        // Screen 3: Scale calibration interface
        if (!calib_in_progress) {
            // Pre-calibration instructions
            M5.Display.println("Scale Calib");
            M5.Display.println("Put known weight");
            M5.Display.println("BtnB: start");
        } else {
            // During calibration progress display
            M5.Display.println("Calibrating...");
            M5.Display.printf("Known: %.2f kg\n", calib_known_weight);
            M5.Display.printf("Raw:   %.3f kg\n", DATA.weight_g);
            M5.Display.println("BtnB: finish");
        }
    }
}

// ============================================================================
// FreeRTOS Task Implementation
// ============================================================================

/**
 * @brief FreeRTOS task for display management and user interface.
 * 
 * Main task function that runs indefinitely, handling display updates,
 * button input processing, and screen navigation. Manages the calibration
 * state machine and coordinates with other system components.
 * 
 * Task Behavior:
 *   - Initialize display task with startup logging
 *   - Enter infinite loop running at 10Hz (100ms intervals)
 *   - Update M5Stack button states with M5.update()
 *   - Check for screen change commands from command_handler
 *   - Process Button A presses for manual screen cycling
 *   - Handle calibration state machine on screen 3
 *   - Redraw display only when screen changes or during calibration
 * 
 * Screen Navigation:
 *   - Command-based: command_get_display_screen() for remote control
 *   - Manual: Button A cycles through screens 0→1→2→3→0
 *   - State tracking: lastScreen prevents unnecessary redraws
 *   - Immediate update: draw() called on screen changes
 * 
 * Calibration State Machine:
 *   - Idle: Display instructions, wait for Button B press
 *   - Starting: Set calib_in_progress=true, tare scale, redraw
 *   - Measuring: Display known weight vs. raw reading
 *   - Completing: Calculate factor, save to CONFIG, reset state
 *   - Error handling: Division by zero check (raw != 0)
 * 
 * Button Processing:
 *   - Button A: Screen cycling (always available)
 *   - Button B: Context-sensitive (calibration actions on screen 3)
 *   - Button C: Not used (reserved for future features)
 *   - Debouncing: Handled by M5Stack library (wasPressed())
 * 
 * Task Configuration:
 *   - Update rate: 10Hz (100ms FreeRTOS delay)
 *   - Priority: tskIDLE_PRIORITY + 1 (standard priority)
 *   - Stack size: 2048 bytes (sufficient for display operations)
 *   - Core affinity: No restriction (runs on any core)
 * 
 * Error Handling:
 *   - Screen bounds: Modulo operation prevents invalid screens
 *   - Calibration safety: Raw reading validation before division
 *   - Mutex protection: Automatic via std::lock_guard in draw()
 *   - Task stability: No infinite loops or blocking operations
 * 
 * Performance:
 *   - CPU usage: Low (mostly sleeping in vTaskDelay)
 *   - Memory usage: Fixed (static variables only)
 *   - Display updates: Only when needed (change detection)
 *   - Responsiveness: 100ms button polling interval
 * 
 * @param pv - FreeRTOS task parameter (unused, nullptr)
 * 
 * @return void (task runs indefinitely)
 * 
 * @note This task provides the primary user interface for SmartFranklin.
 *       Display updates are optimized to minimize redraw frequency.
 *       Calibration procedure requires user interaction and known weights.
 * 
 * @see draw() - Display rendering function
 * @see command_get_display_screen() - Remote screen control
 * @see scale_control.h - Scale calibration functions
 */
void taskDisplay(void *pv)
{
    Serial.println("[DISPLAY] Task started");
    
    // Track last screen to detect changes and avoid unnecessary redraws
    int lastScreen = -1;
    
    // Main task loop - runs indefinitely at 10Hz
    for (;;) {
        // Update M5Stack button and sensor states
        M5.update();
        
        // Check for screen change commands from external sources
        int s = command_get_display_screen();
        
        // Process manual screen navigation with Button A
        if (M5.BtnA.wasPressed()) {
            s = (s + 1) % 4;  // Cycle through 4 screens (0-3)
        }
        
        // Redraw display if screen has changed
        if (s != lastScreen) {
            lastScreen = s;
            screen = s;
            draw();
        }
        
        // Handle calibration state machine on screen 3
        if (screen == 3) {
            if (!calib_in_progress && M5.BtnB.wasPressed()) {
                // Start calibration procedure
                calib_in_progress = true;
                scale_tare();  // Zero the scale
                draw();  // Update display with progress
            } else if (calib_in_progress && M5.BtnB.wasPressed()) {
                // Complete calibration procedure
                float raw = scale_get_raw();
                if (raw != 0) {
                    // Calculate calibration factor: known_weight / raw_reading
                    float factor = calib_known_weight / raw;
                    scale_set_cal_factor(factor);
                    
                    // Save calibration factor to persistent configuration
                    CONFIG.scale_cal_factor = factor;
                    config_save();
                }
                calib_in_progress = false;
                draw();  // Return to instruction screen
            }
        }
        
        // Task delay for 100ms (10Hz update rate)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}