/*
 * ============================================================================
 * HMI Interface Module - SmartFranklin
 * ============================================================================
 *
 * File:        hmi.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Declares the Human-Machine Interface (HMI) runtime used to
 *              render display pages, process local button inputs, and execute
 *              interactive scale calibration workflow.
 *
 * Author:      Laurent Burais
 * Date:        11 March 2026
 * Version:     1.0
 *
 * Overview:
 *   The HMI class provides a compact application-facing API that mirrors
 *   other SmartFranklin runtime modules (`init`, `process`, `isInitialized`,
 *   `isHealthy`).
 *
 *   It owns:
 *   - LCD page rendering for all local operator views
 *   - Button-driven page navigation and calibration control
 *   - Snapshot-based reads from shared DATA under mutex protection
 *   - Lightweight runtime health reporting for task supervision
 *
 * Screen Map:
 *   - 0: Tank (distance)
 *   - 1: Gaz (weight)
 *   - 2: Level (pitch/roll)
 *   - 3: Battery (BMS)
 *   - 4: GPS
 *   - 5: RTC
 *   - 6: Scale Calibration
 *
 * ============================================================================
 */

#pragma once

#include <Arduino.h>

/**
 * @brief Human-Machine Interface runtime for display and local controls.
 *
 * Lifecycle model:
 * 1. `init()` configures display/backlight and draws first frame.
 * 2. Task periodically calls `process()` to update input/rendering.
 * 3. Supervisors use `isInitialized()` and `isHealthy()` for task status.
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
     * @brief Reports whether `init()` completed successfully.
     * @return true after successful initialization.
     */
    bool isInitialized() const;

    /**
     * @brief Reports whether runtime heartbeat is considered healthy.
     * @return true when initialized and recent process activity exists.
     */
    bool isHealthy() const;

private:
    /** @brief Number of available UI screens. */
    static constexpr int kScreenCount = 7;

    /**
     * @brief Immutable copy of shared DATA used by one render pass.
     *
     * Rendering never accesses shared global state directly; this snapshot is
     * captured under lock and then consumed lock-free by drawing helpers.
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

    /** @brief Draws distance screen (Tank). */
    void drawTankScreen(const DisplaySnapshot& snapshot) const;

    /** @brief Draws weight screen (Gaz). */
    void drawGazScreen(const DisplaySnapshot& snapshot) const;

    /** @brief Draws tilt screen (Level). */
    void drawLevelScreen(const DisplaySnapshot& snapshot) const;

    /** @brief Draws battery/BMS telemetry screen. */
    void drawBatteryScreen(const DisplaySnapshot& snapshot) const;

    /** @brief Draws GPS telemetry screen. */
    void drawGpsScreen(const DisplaySnapshot& snapshot) const;

    /** @brief Draws RTC time screen. */
    void drawRtcScreen(const DisplaySnapshot& snapshot) const;

    /** @brief Draws scale calibration guidance/progress screen. */
    void drawCalibrationScreen(const DisplaySnapshot& snapshot) const;

    /** @brief Active page index in range [0, kScreenCount-1]. */
    int screen_ = 2;

    /** @brief True while two-step calibration flow is in progress. */
    bool calib_in_progress_ = false;

    /** @brief Known calibration reference weight (kg). */
    float calib_known_weight_ = 1.0f;

    /** @brief Previous sampled Button A pressed state for edge detection. */
    bool btnA_prev_ = false;

    /** @brief Previous sampled Button B pressed state for edge detection. */
    bool btnB_prev_ = false;

    /** @brief Timestamp of latest full redraw. */
    uint32_t last_redraw_ms_ = 0;

    /** @brief Timestamp of latest successful process cycle. */
    uint32_t last_process_ms_ = 0;

    /** @brief True after successful init(). */
    bool initialized_ = false;

    /** @brief Result flag of most recent process cycle. */
    bool last_process_ok_ = false;

    /** @brief Cached last known snapshot used by rendering helpers. */
    DisplaySnapshot last_snapshot_;
};
