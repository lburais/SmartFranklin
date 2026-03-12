/*
 * ============================================================================
 * HMI Implementation Module - SmartFranklin
 * ============================================================================
 *
 * File:        hmi.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: Implements HMI lifecycle and rendering helpers for the local
 *              SmartFranklin screen interface.
 *
 * Author:      Laurent Burais
 * Date:        11 March 2026
 * Version:     1.0
 *
 * Overview:
 *   This module provides concrete behavior for the HMI runtime contract
 *   defined in `hmi.h`.
 *
 *   Core responsibilities:
 *   - Display setup and first-frame boot rendering (`init`)
 *   - Input-driven processing loop (`process`)
 *   - Health/init status signals (`isInitialized`, `isHealthy`)
 *   - Snapshot-based drawing helpers for each page
 *   - Calibration interaction flow on dedicated screen
 *
 * Rendering model:
 *   - A stable copy of shared DATA is captured under mutex each frame.
 *   - Draw helpers consume that snapshot without holding locks.
 *   - Periodic redraw keeps UI current even when no buttons are pressed.
 *
 * ============================================================================
 */

#include "hmi.h"

#include <M5Unified.h>

#include "config_store.h"
#include "data_model.h"
#include "scale_control.h"

namespace {

/** @brief Display rotation used for M5StickC Plus2 visual orientation. */
static constexpr uint8_t DISPLAY_ROTATION = 3;

/** @brief Full backlight brightness value (0..255). */
static constexpr uint8_t DISPLAY_BRIGHTNESS = 255;

/** @brief Title box top-left x coordinate. */
static constexpr int16_t TITLE_BOX_X = 6;

/** @brief Title box top-left y coordinate. */
static constexpr int16_t TITLE_BOX_Y = 6;

/** @brief Title box height in pixels. */
static constexpr int16_t TITLE_BOX_H = 34;

/** @brief Content area x coordinate. */
static constexpr int16_t CONTENT_X = 8;

/** @brief Content area y coordinate (below title area). */
static constexpr int16_t CONTENT_Y = TITLE_BOX_Y + TITLE_BOX_H + 10;

/** @brief Title background color. */
static constexpr uint16_t COLOR_TITLE_BG = 0x39E7;

/** @brief Title border color. */
static constexpr uint16_t COLOR_TITLE_BORDER = 0xFFFF;

/** @brief Title text color. */
static constexpr uint16_t COLOR_TITLE_TEXT = 0xFFFF;

/** @brief Main content text color. */
static constexpr uint16_t COLOR_CONTENT_TEXT = 0xFFFF;

/** @brief Main background color. */
static constexpr uint16_t COLOR_CONTENT_BG = 0x0000;

/** @brief Title text scale factor. */
static constexpr uint8_t TITLE_TEXT_SIZE = 2;

/** @brief Content text scale factor for most pages. */
static constexpr uint8_t CONTENT_TEXT_SIZE = 2;

} // namespace

/**
 * @brief Initializes display hardware state and internal HMI runtime state.
 *
 * Steps performed:
 * - configure backlight PWM and wake display
 * - apply brightness, rotation, and color defaults
 * - reset screen/calibration/button state machine variables
 * - draw initial frame and mark runtime initialized/healthy
 *
 * @return true when initialization path completes.
 */
bool HMI::init()
{
    M5_LOGI("[HMI] init");

    ledcSetup(7, 12000, 8);
    ledcAttachPin(27, 7);
    ledcWrite(7, DISPLAY_BRIGHTNESS);

    M5.Display.wakeup();
    M5.Display.setBrightness(DISPLAY_BRIGHTNESS);
    M5.Display.setTextSize(CONTENT_TEXT_SIZE);
    M5.Display.setRotation(DISPLAY_ROTATION);
    M5.Display.invertDisplay(false);
    M5.Display.fillScreen(COLOR_CONTENT_BG);

    screen_ = 2;
    calib_in_progress_ = false;
    btnA_prev_ = false;
    btnB_prev_ = false;

    draw();
    last_redraw_ms_ = millis();
    last_process_ms_ = last_redraw_ms_;
    initialized_ = true;
    last_process_ok_ = true;

    return true;
}

/**
 * @brief Executes one frame of HMI runtime processing.
 *
 * Work done each call:
 * - refresh button states (`M5.update`)
 * - detect navigation button edges
 * - redraw on page changes
 * - redraw periodically to refresh telemetry
 * - process calibration button transitions
 * - update runtime heartbeat/health markers
 */
void HMI::process()
{
    if (!initialized_) {
        return;
    }

    M5.update();

    int next_screen = screen_;

    const bool btnA_now = M5.BtnA.isPressed();
    const bool btnA_rising = M5.BtnA.wasPressed() || (btnA_now && !btnA_prev_);
    btnA_prev_ = btnA_now;

    const bool btnB_now = M5.BtnB.isPressed();
    const bool btnB_rising = M5.BtnB.wasPressed() || (btnB_now && !btnB_prev_);
    btnB_prev_ = btnB_now;

    if (btnA_rising) {
        next_screen = (next_screen + 1) % kScreenCount;
    }

    if (next_screen != screen_) {
        screen_ = next_screen;
        draw();
        last_redraw_ms_ = millis();
    }

    if ((millis() - last_redraw_ms_) > 250UL) {
        draw();
        last_redraw_ms_ = millis();
    }

    handleCalibrationButton(btnB_rising);
    last_process_ms_ = millis();
    last_process_ok_ = true;
}

/**
 * @brief Returns true once `init()` has completed.
 */
bool HMI::isInitialized() const
{
    return initialized_;
}

/**
 * @brief Returns runtime health based on process heartbeat freshness.
 *
 * Health is considered true only when:
 * - initialized flag is true
 * - previous process cycle did not fail
 * - heartbeat timestamp is recent
 */
bool HMI::isHealthy() const
{
    if (!initialized_ || !last_process_ok_) {
        return false;
    }

    return (millis() - last_process_ms_) <= 5000UL;
}

/**
 * @brief Handles Button B behavior specific to calibration page.
 *
 * Flow:
 * - first press enters calibration mode and tares scale
 * - second press computes new calibration factor and persists config
 */
void HMI::handleCalibrationButton(bool btnB_rising)
{
    if (screen_ != 6 || !btnB_rising) {
        return;
    }

    if (!calib_in_progress_) {
        calib_in_progress_ = true;
        scale_tare();
        draw();
        last_redraw_ms_ = millis();
        return;
    }

    const float raw = scale_get_raw();
    if (raw != 0.0f) {
        const float factor = calib_known_weight_ / raw;
        scale_set_cal_factor(factor);
        CONFIG.scale_cal_factor = factor;
        config_save();
    }
    calib_in_progress_ = false;
    draw();
    last_redraw_ms_ = millis();
}

/**
 * @brief Draws the top title band used by all pages.
 * @param title Page title text.
 */
void HMI::drawTitleBox(const char* title) const
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

/**
 * @brief Draws current page based on active screen index.
 */
void HMI::draw()
{
    DisplaySnapshot snapshot = last_snapshot_;
    updateSnapshot(snapshot);

    M5.Display.fillScreen(COLOR_CONTENT_BG);

    if (screen_ == 0) {
        drawTankScreen(snapshot);
    } else if (screen_ == 1) {
        drawGazScreen(snapshot);
    } else if (screen_ == 2) {
        drawLevelScreen(snapshot);
    } else if (screen_ == 3) {
        drawBatteryScreen(snapshot);
    } else if (screen_ == 4) {
        drawGpsScreen(snapshot);
    } else if (screen_ == 5) {
        drawRtcScreen(snapshot);
    } else if (screen_ == 6) {
        drawCalibrationScreen(snapshot);
    }
}

/**
 * @brief Copies shared global data into a local frame snapshot.
 *
 * Snapshot capture is guarded by DATA mutex to guarantee internal
 * consistency for one frame render.
 */
void HMI::updateSnapshot(DisplaySnapshot& snapshot)
{
    {
        std::lock_guard<std::mutex> lock(DATA_MUTEX);

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
    }

    last_snapshot_ = snapshot;
}

/**
 * @brief Applies common content text settings and cursor placement.
 */
void HMI::beginContentArea() const
{
    M5.Display.setTextSize((screen_ == 4 || screen_ == 5) ? 1 : CONTENT_TEXT_SIZE);
    M5.Display.setTextColor(COLOR_CONTENT_TEXT, COLOR_CONTENT_BG);
    M5.Display.setCursor(CONTENT_X, CONTENT_Y);
}

/** @brief Draw helper for Tank distance page. */
void HMI::drawTankScreen(const DisplaySnapshot& snapshot) const
{
    drawTitleBox("Tank");
    beginContentArea();
    M5.Display.printf("Distance: %.1f cm\n", snapshot.distance_cm);
}

/** @brief Draw helper for Gaz weight page. */
void HMI::drawGazScreen(const DisplaySnapshot& snapshot) const
{
    drawTitleBox("Gaz");
    beginContentArea();
    M5.Display.printf("Weight: %.3f g\n", snapshot.weight_g);
}

/** @brief Draw helper for Level tilt telemetry page. */
void HMI::drawLevelScreen(const DisplaySnapshot& snapshot) const
{
    drawTitleBox("Level");
    beginContentArea();
    M5.Display.printf("Pitch: %.2f\n", snapshot.pitch);
    M5.Display.printf("Roll:  %.2f\n", snapshot.roll);
}

/** @brief Draw helper for battery/BMS telemetry page. */
void HMI::drawBatteryScreen(const DisplaySnapshot& snapshot) const
{
    drawTitleBox("Battery");
    beginContentArea();
    M5.Display.printf("BMS V: %.2f\nI: %.2f\nSOC: %.1f\n",
                      snapshot.bms_voltage,
                      snapshot.bms_current,
                      snapshot.bms_soc);
}

/** @brief Draw helper for GPS telemetry page. */
void HMI::drawGpsScreen(const DisplaySnapshot& snapshot) const
{
    drawTitleBox("GPS");
    beginContentArea();
    M5.Display.printf("Fix: %s  Sat: %u\n",
                      snapshot.gps_fix ? "YES" : "NO",
                      snapshot.gps_satellites);
    M5.Display.printf("Lat: %.6f\n", snapshot.gps_lat);
    M5.Display.printf("Lon: %.6f\n", snapshot.gps_lon);
    M5.Display.printf("Alt: %.1f m\n", snapshot.gps_alt_m);
}

/** @brief Draw helper for RTC telemetry page. */
void HMI::drawRtcScreen(const DisplaySnapshot& snapshot) const
{
    drawTitleBox("RTC");
    beginContentArea();
    M5.Display.println("GPS UTC:");
    M5.Display.printf("%s %s\n", snapshot.gps_utc_date.c_str(), snapshot.gps_utc_time.c_str());
    M5.Display.println("RTC:");
    M5.Display.println(snapshot.gps_rtc_time.c_str());
}

/** @brief Draw helper for interactive scale calibration page. */
void HMI::drawCalibrationScreen(const DisplaySnapshot& snapshot) const
{
    drawTitleBox("Scale Calibration");
    beginContentArea();
    if (!calib_in_progress_) {
        M5.Display.println("Scale Calib");
        M5.Display.println("Put known weight");
        M5.Display.println("BtnB: start");
        return;
    }

    M5.Display.println("Calibrating...");
    M5.Display.printf("Known: %.2f kg\n", calib_known_weight_);
    M5.Display.printf("Raw:   %.3f kg\n", snapshot.weight_g);
    M5.Display.println("BtnB: finish");
}
