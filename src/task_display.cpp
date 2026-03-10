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
 *   data visualization. This task manages six different screens showing
 *   distance, weight, orientation, battery status, GNSS/RTC telemetry,
 *   and scale calibration tools.
 *   Users can navigate screens with button presses and perform calibration
 *   procedures for the weight scale.
 * 
 * Display Screens:
 * 
 *   Screen 0: Distance
 *   - Distance measurement from ultrasonic sensor (cm)
 *   - Auto-updates with latest sensor readings
 * 
 *   Screen 1: Weight
 *   - Weight measurement from load cell scale (grams)
 *   - Dedicated mass display for quick readout
 *
 *   Screen 2: Orientation Data
 *   - Pitch angle from IMU (degrees)
 *   - Roll angle from IMU (degrees)
 *   - Device orientation for tilt sensing
 *   - Real-time IMU data visualization
 * 
 *   Screen 3: Battery Management System (BMS)
 *   - Battery voltage (volts)
 *   - Battery current (amps)
 *   - State of charge (SOC, percent)
 *   - BLE-connected BMS data display
 * 
 *   Screen 4: GPS + RTC
 *   - GNSS fix state and satellites count
 *   - Latitude/longitude and altitude
 *   - UTC date/time from GNSS
 *   - RTC timestamp from DFR1103
 *
 *   Screen 5: Scale Calibration
 *   - Interactive calibration procedure
 *   - Step-by-step weight scale calibration
 *   - Known weight placement instructions
 *   - Calibration factor calculation and storage
 * 
 * User Interface Controls:
 *   - Button A: Cycle through screens (0→1→2→3→4→5→0)
 *   - Button B: Context-sensitive actions on calibration screen
 *   - Button C: Reserved (not used in display task)
 *   - Touch: Not implemented (button-based navigation)
 * 
 * Calibration Procedure:
 *   1. Navigate to screen 5 (Scale Calibration)
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
 *   - DATA.gps_fix, DATA.gps_satellites: GNSS fix quality indicators
 *   - DATA.gps_lat, DATA.gps_lon, DATA.gps_alt_m: GNSS position solution
 *   - DATA.gps_utc_date, DATA.gps_utc_time, DATA.gps_rtc_time: Time sources
 *   - Thread-safe access with DATA_MUTEX lock
 * 
 * Display Configuration:
 *   - Resolution: 320x240 pixels (M5Stack standard)
 *   - Rotation: Configurable via DISPLAY_ROTATION constant
 *   - Header: Boxed title bar at the top of every screen
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
 *   - Fixed 6-screen limit (hardcoded screen count)
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
#include "config_store.h"
#include "scale_control.h"

// ============================================================================
// Global Display State Variables
// ============================================================================

/**
 * @brief Current display screen index (0-5).
 * 
 * Tracks which screen is currently being displayed.
 * Modified by local button inputs.
 * Used to determine what data to show in draw() function.
 */
static int screen = 0;

/**
 * @brief Flag indicating if scale calibration is in progress.
 * 
 * Set to true when user starts calibration procedure on screen 5.
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

/**
 * @brief Display rotation applied at task startup.
 *
 * Rotation values follow M5GFX convention:
 *   0 = default orientation
 *   1 = rotate 90 degrees
 *   2 = rotate 180 degrees
 *   3 = rotate 270 degrees
 */
static constexpr uint8_t DISPLAY_ROTATION = 3;
static constexpr uint8_t DISPLAY_BRIGHTNESS = 255;

/**
 * @brief Title box layout and color configuration.
 */
static constexpr int16_t TITLE_BOX_X = 6;
static constexpr int16_t TITLE_BOX_Y = 6;
static constexpr int16_t TITLE_BOX_H = 34;
static constexpr int16_t CONTENT_X = 8;
static constexpr int16_t CONTENT_Y = TITLE_BOX_Y + TITLE_BOX_H + 10;
static constexpr uint16_t COLOR_TITLE_BG = 0x39E7;      // neutral gray
static constexpr uint16_t COLOR_TITLE_BORDER = 0xFFFF;  // white
static constexpr uint16_t COLOR_TITLE_TEXT = 0xFFFF;    // white
static constexpr uint16_t COLOR_CONTENT_TEXT = 0xFFFF;  // white
static constexpr uint16_t COLOR_CONTENT_BG = 0x0000;    // black
static constexpr uint8_t TITLE_TEXT_SIZE = 2;
static constexpr uint8_t CONTENT_TEXT_SIZE = 2;

/**
 * @brief Last known data snapshot used by display rendering.
 *
 * If DATA_MUTEX is temporarily unavailable, draw() reuses this cached copy
 * instead of blocking the display task and getting visually stuck on splash.
 */
struct DisplaySnapshot {
    float distance_cm = 0.0f;
    float weight_g = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    float bms_voltage = 0.0f;
    float bms_current = 0.0f;
    float bms_soc = 0.0f;
    bool gps_fix = false;
    uint8_t gps_satellites = 0;
    double gps_lat = 0.0;
    double gps_lon = 0.0;
    double gps_alt_m = 0.0;
    String gps_utc_date;
    String gps_utc_time;
    String gps_rtc_time;
};

static DisplaySnapshot g_last_snapshot;

/**
 * @brief Draws a boxed title at the top of the display.
 *
 * @param title Screen title text.
 */
static void drawTitleBox(const char* title)
{
    const int16_t boxW = static_cast<int16_t>(M5.Display.width()) - (2 * TITLE_BOX_X);

    M5.Display.fillRect(TITLE_BOX_X, TITLE_BOX_Y, boxW, TITLE_BOX_H, COLOR_TITLE_BG);
    M5.Display.drawRect(TITLE_BOX_X, TITLE_BOX_Y, boxW, TITLE_BOX_H, COLOR_TITLE_BORDER);

    M5.Display.setTextSize(TITLE_TEXT_SIZE);
    M5.Display.setTextColor(COLOR_TITLE_TEXT, COLOR_TITLE_BG);
    int16_t textX = TITLE_BOX_X + ((boxW - M5.Display.textWidth(title)) / 2);
    int16_t textY = TITLE_BOX_Y + ((TITLE_BOX_H - M5.Display.fontHeight()) / 2);
    if (textX < (TITLE_BOX_X + 2)) {
        textX = TITLE_BOX_X + 2;
    }
    if (textY < (TITLE_BOX_Y + 2)) {
        textY = TITLE_BOX_Y + 2;
    }
    M5.Display.setCursor(textX, textY);
    M5.Display.print(title);
}

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
 *   Screen 0 (Distance):
 *   - Distance: Ultrasonic measurement in centimeters (%.1f precision)
 *
 *   Screen 1 (Weight):
 *   - Weight: Load cell measurement in grams (%.3f precision)
 * 
 *   Screen 2 (Orientation):
 *   - Pitch: IMU pitch angle in degrees (%.1f precision)
 *   - Roll: IMU roll angle in degrees (%.1f precision)
 *   - Device orientation visualization
 * 
 *   Screen 3 (BMS Status):
 *   - Voltage: Battery voltage in volts (%.2f precision)
 *   - Current: Battery current in amps (%.2f precision)
 *   - SOC: State of charge in percent (%.1f precision)
 *   - BLE-connected battery management data
 * 
 *   Screen 4 (GPS + RTC):
 *   - GNSS fix and satellites count
 *   - Latitude/longitude in decimal degrees
 *   - Altitude and GNSS UTC date/time
 *   - RTC timestamp read from DFR1103
 *
 *   Screen 5 (Calibration):
 *   - Pre-calibration: Instructions and start button prompt
 *   - During calibration: Progress display with known/raw weights
 *   - Post-calibration: Automatically returns to instruction screen
 * 
 * Display Properties:
 *   - Title: Boxed screen title rendered at top of display
 *   - Background: Cleared to black on each redraw
 *   - Text: White monospace font (default M5Stack font)
 *   - Positioning: Content starts below title header
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
    // Snapshot shared data first, but never block display rendering on mutex.
    DisplaySnapshot snapshot = g_last_snapshot;
    if (DATA_MUTEX.try_lock()) {
        snapshot.distance_cm = DATA.distance_cm;
        snapshot.weight_g = DATA.weight_g;
        snapshot.pitch = DATA.pitch;
        snapshot.roll = DATA.roll;
        snapshot.bms_voltage = DATA.bms_voltage;
        snapshot.bms_current = DATA.bms_current;
        snapshot.bms_soc = DATA.bms_soc;
        snapshot.gps_fix = DATA.gps_fix;
        snapshot.gps_satellites = DATA.gps_satellites;
        snapshot.gps_lat = DATA.gps_lat;
        snapshot.gps_lon = DATA.gps_lon;
        snapshot.gps_alt_m = DATA.gps_alt_m;
        snapshot.gps_utc_date = DATA.gps_utc_date;
        snapshot.gps_utc_time = DATA.gps_utc_time;
        snapshot.gps_rtc_time = DATA.gps_rtc_time;
        DATA_MUTEX.unlock();
        g_last_snapshot = snapshot;
    }

    const char* title = "SmartFranklin";

    if (screen == 0) {
        title = "Tank";
    } else if (screen == 1) {
        title = "Gaz";
    } else if (screen == 2) {
        title = "Level";
    } else if (screen == 3) {
        title = "Battery";
    } else if (screen == 4) {
        title = "GPS + RTC";
    } else if (screen == 5) {
        title = "Scale Calibration";
    }

    // Clear display for clean redraw
    M5.Display.fillScreen(COLOR_CONTENT_BG);

    // Draw top title header and place content cursor under it.
    drawTitleBox(title);
    M5.Display.setTextSize((screen == 4) ? 1 : CONTENT_TEXT_SIZE);
    M5.Display.setTextColor(COLOR_CONTENT_TEXT, COLOR_CONTENT_BG);
    M5.Display.setCursor(CONTENT_X, CONTENT_Y);
    
    if (screen == 0) {
        // Screen 0: Distance readout
        M5.Display.printf("Distance: %.1f cm\n", snapshot.distance_cm);
    } else if (screen == 1) {
        // Screen 1: Weight readout
        M5.Display.printf("Weight: %.3f g\n", snapshot.weight_g);
    } else if (screen == 2) {
        // Screen 2: IMU orientation data (pitch and roll)
        M5.Display.printf("Pitch: %.1f\nRoll: %.1f\n", snapshot.pitch, snapshot.roll);
    } else if (screen == 3) {
        // Screen 3: Battery management system data
        M5.Display.printf("BMS V: %.2f\nI: %.2f\nSOC: %.1f\n",
                          snapshot.bms_voltage, snapshot.bms_current, snapshot.bms_soc);
    } else if (screen == 4) {
        // Screen 4: GNSS and RTC telemetry from DFR1103
        M5.Display.printf("Fix: %s  Sat: %u\n",
                          snapshot.gps_fix ? "YES" : "NO",
                          snapshot.gps_satellites);
        M5.Display.printf("Lat: %.6f\n", snapshot.gps_lat);
        M5.Display.printf("Lon: %.6f\n", snapshot.gps_lon);
        M5.Display.printf("Alt: %.1f m\n", snapshot.gps_alt_m);
        M5.Display.printf("UTC: %s %s\n", snapshot.gps_utc_date.c_str(), snapshot.gps_utc_time.c_str());
        M5.Display.println("RTC:");
        M5.Display.println(snapshot.gps_rtc_time.c_str());
    } else if (screen == 5) {
        // Screen 5: Scale calibration interface (always last screen)
        if (!calib_in_progress) {
            // Pre-calibration instructions
            M5.Display.println("Scale Calib");
            M5.Display.println("Put known weight");
            M5.Display.println("BtnB: start");
        } else {
            // During calibration progress display
            M5.Display.println("Calibrating...");
            M5.Display.printf("Known: %.2f kg\n", calib_known_weight);
            M5.Display.printf("Raw:   %.3f kg\n", snapshot.weight_g);
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
 *   - Process Button A presses for manual screen cycling
 *   - Handle calibration state machine on screen 5
 *   - Redraw display only when screen changes or during calibration
 * 
 * Screen Navigation:
 *   - Manual: Button A cycles through screens 0→1→2→3→4→5→0
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
 *   - Button B: Context-sensitive (calibration actions on screen 5)
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
 * @see scale_control.h - Scale calibration functions
 */
void taskDisplay(void *pv)
{
    (void)pv;

    M5_LOGI("[DISPLAY] Task started");

    // Force backlight PWM explicitly for M5StickC Plus2 (GPIO27).
    // This bypasses any runtime backlight state drift in higher-level APIs.
    ledcSetup(7, 12000, 8);
    ledcAttachPin(27, 7);
    ledcWrite(7, DISPLAY_BRIGHTNESS);

    // Force display to an active visible state.
    M5.Display.wakeup();
    M5.Display.setBrightness(DISPLAY_BRIGHTNESS);
    M5.Display.setTextSize(CONTENT_TEXT_SIZE);

    // Apply desired screen orientation once at task startup.
    M5.Display.setRotation(DISPLAY_ROTATION);

    M5.Display.fillScreen(COLOR_CONTENT_BG);
    
    // Initialize to the default screen.
    screen = 0;

    // Draw once at startup to avoid waiting for the first button event.
    draw();

    // Track last screen to detect changes and avoid unnecessary redraws
    int lastScreen = screen;

    // Edge tracking for deterministic button actions.
    bool btnA_prev = false;
    bool btnB_prev = false;

    // Periodic redraw prevents stale/blank screen if panel state changes.
    uint32_t lastRedrawMs = millis();

    // Main task loop - runs indefinitely at 10Hz
    for (;;) {
        // Update button state in the same task that consumes button events.
        M5.update();

        // Force panel active state in case other subsystem toggles power state.
        M5.Display.wakeup();
        M5.Display.setBrightness(DISPLAY_BRIGHTNESS);
        ledcWrite(7, DISPLAY_BRIGHTNESS);

        // Start from current local screen so button-based navigation persists.
        int s = screen;
        
        // Process manual screen navigation with Button A.
        const bool btnA_now = M5.BtnA.isPressed();
        const bool btnA_rising = (btnA_now && !btnA_prev);
        btnA_prev = btnA_now;

        const bool btnB_now = M5.BtnB.isPressed();
        const bool btnB_rising = (btnB_now && !btnB_prev);
        btnB_prev = btnB_now;

        if (btnA_rising) {
            s = (s + 1) % 6;  // Cycle through 6 screens (0-5)
        }
        
        // Redraw display if screen has changed
        if (s != lastScreen) {
            lastScreen = s;
            screen = s;
            draw();
            lastRedrawMs = millis();
        }

        // Keep display refreshed periodically even without user input.
        if ((millis() - lastRedrawMs) > 1000UL) {
            M5.Display.wakeup();
            M5.Display.setBrightness(DISPLAY_BRIGHTNESS);
            draw();
            lastRedrawMs = millis();
        }

        // Handle calibration state machine on screen 5
        if (screen == 5) {
            if (!calib_in_progress && btnB_rising) {
                // Start calibration procedure
                calib_in_progress = true;
                scale_tare();  // Zero the scale
                draw();  // Update display with progress
            } else if (calib_in_progress && btnB_rising) {
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

        // Higher poll rate improves responsiveness under system load.
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}