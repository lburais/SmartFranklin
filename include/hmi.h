/**
 * ============================================================================
 * HMI Interface Module - SmartFranklin
 * ============================================================================
 *
 * File:        hmi.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Public API for the Human-Machine Interface (HMI) runtime.
 *
 * Author:      Laurent Burais
 * Date:        12 March 2026
 * Version:     1.1
 *
 * Overview:
 *   The HMI owns the local display runtime, simple navigation state, touch/
 *   button handling, and LED status indication for interface health.
 *   Rendering is snapshot-based: shared DATA is copied under lock and then
 *   consumed without holding the mutex during drawing.
 *
 * Screen Map:
 *   0: tank
 *   1: gaz
 *   2: battery
 *   3: gps
 *   4: level
 *   5: calibration
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

#include "interfaces.h"

/**
 * @brief Status values for one interface LED.
 */
enum class PortStatus : uint8_t {
    Unset = 0,
    Initialized,
    NoData,
    Error,
    Ok,
    BatteryLow,
};

/**
 * @brief Human-Machine Interface runtime for display and local controls.
 *
 * Lifecycle model:
 * 1. `init()` configures display/backlight and draws first frame.
 * 2. Task periodically calls `process()` to update input/rendering.
 * 3. Supervisors can query `isInitialized()`.
 */
class HMI {
public:
    /**
     * @brief Initializes display/backlight state and HMI runtime variables.
     *
     * Applies display orientation/brightness, initializes local UI state,
     * and performs the first full frame draw.
     *
     * @return true when HMI startup path completed.
     */
    bool init();

    /**
     * @brief Executes one HMI processing cycle.
     *
     * The cycle handles button edges, screen switching, periodic redraw, and
     * calibration button logic.
     */
    void process();

    /**
     * @brief Reports HMI initialization status.
     * @return true after successful initialization.
     */
    bool isInitialized() const;

    /** @brief Updates one interface LED from the current port status. */
    static void setLed(sf_interfaces::InterfaceSensor sensor, PortStatus status = PortStatus::Unset);

private:
    /**
     * @brief Returns the active HMI screen name.
     * @return Stable lowercase screen name string.
     */
    const char* currentScreenName() const;

    /** @brief Number of available UI screens. */
    static constexpr int kScreenCount = 6;

    /**
     * @brief Immutable copy of shared DATA used by one render pass.
     *
     * Rendering never accesses shared global state directly; this snapshot is
     * captured under lock and then consumed lock-free by drawing helpers.
     */
    struct DisplaySnapshot {
        int32_t weight_gaz = 0;
        int32_t fill_gaz = 0;
        int32_t distance_tank_mm = 0;
        int32_t fill_tank = 0;
        float bms_voltage = 0.0f;
        float bms_current = 0.0f;
        float bms_soc = 0.0f;
        float level_pitch_deg = 0.0f;
        float level_roll_deg = 0.0f;
        float level_wheel_fl_mm = 0.0f;
        float level_wheel_fr_mm = 0.0f;
        float level_wheel_rl_mm = 0.0f;
        float level_wheel_rr_mm = 0.0f;
        bool gps_has_fix = false;
        uint8_t gps_satellites = 0;
        double gps_latitude_deg = 0.0;
        double gps_longitude_deg = 0.0;
        double gps_altitude_m = 0.0;
        double gps_speed_knots = 0.0;
        double gps_course_deg = 0.0;
        String gps_date;
        String gps_utc;
    };

    /** @brief Draws the currently selected screen. */
    void draw();

    /**
     * @brief Handles Button B action for calibration screen state transitions.
     * @param btnB_rising True when Button B edge is detected this cycle.
     */
    void handleCalibrationButton(bool btnB_rising);

    /**
     * @brief Draws top title area for the active page.
     * @param title Screen title text.
     */
    void drawTitleBox(const char* title) const;

    /**
     * @brief Refreshes per-frame snapshot from shared DATA under mutex.
     * @param snapshot Destination snapshot for current frame.
     */
    void updateSnapshot(DisplaySnapshot& snapshot);

    /** @brief Applies common text/color/cursor settings for page content area. */
    void beginContentArea() const;

    /** @brief Draws tank screen (Ultrasonic). */
    void drawTankScreen(const DisplaySnapshot& snapshot) const;
    /** @brief Draws weight screen (Gaz). */
    void drawGazScreen(const DisplaySnapshot& snapshot) const;

    /** @brief Draws battery/BMS telemetry screen. */
    void drawBatteryScreen(const DisplaySnapshot& snapshot) const;

    /** @brief Draws level telemetry screen. */
    void drawLevelScreen(const DisplaySnapshot& snapshot) const;

    /** @brief Draws GPS telemetry screen. */
    void drawGpsScreen(const DisplaySnapshot& snapshot) const;

    /** @brief Draws scale calibration guidance/progress screen. */
    void drawCalibrationScreen(const DisplaySnapshot& snapshot) const;

    /** @brief Active page index in range [0, kScreenCount-1]. */
    int screen_ = 4;

    /** @brief True while two-step calibration flow is in progress. */
    bool calib_in_progress_ = false;

    /** @brief Known calibration reference weight (kg). */
    float calib_known_weight_ = 1.0f;

    /** @brief Previous sampled Button A pressed state for edge detection. */
    bool btnA_prev_ = false;

    /** @brief Previous sampled Button B pressed state for edge detection. */
    bool btnB_prev_ = false;

    /** @brief True after successful init(). */
    bool initialized_ = false;

    /** @brief Last screen index published to MQTT, or -1 if none yet. */
    int last_published_screen_ = -1;

    /** @brief Previous quadrature state for M5 Dial rotary encoder. */
    int8_t dial_encoder_prev_state_ = 0;

    /** @brief Accumulated quadrature steps; one detent is approximately 4 transitions. */
    int8_t dial_encoder_accum_ = 0;

    /** @brief Touch hold start timestamp used for long-press to calibration. */
    uint32_t touch_hold_start_ms_ = 0;

    /** @brief Cached last known snapshot used by rendering helpers. */
    DisplaySnapshot last_snapshot_;
};
