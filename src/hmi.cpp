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
 * Date:        12 March 2026
 * Version:     1.1
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
 *   - MQTT publication of active screen name on UI transitions
 *
 * Rendering Model:
 *   - A stable copy of shared DATA is captured under mutex each frame.
 *   - Draw helpers consume that snapshot without holding locks.
 *   - Periodic redraw keeps UI current even when no buttons are pressed.
 *
 * Dependencies:
 *   - M5Unified (display, buttons, logging)
 *   - data_model.h (shared runtime state + mutex)
 *   - scale_control.h (tare + calibration factor helpers)
 *   - config_store.h (persisted calibration factor)
 *   - mqtt.h (screen-name topic publishing)
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

#include "hmi.h"

#include <M5GFX.h>
#include <M5Unified.h>

#include "config_store.h"
#include "data_model.h"
#include "mqtt.h"
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

/** @brief Startup splash background color. */
static constexpr uint16_t COLOR_SPLASH_BG = 0xFD20;

/** @brief Startup splash text color. */
static constexpr uint16_t COLOR_SPLASH_TEXT = 0xFFFF;

/** @brief Startup splash dwell time in milliseconds. */
static constexpr uint32_t SPLASH_DELAY_MS = 1000;

/**
 * @brief Draws one centered text line using M5GFX horizontal centering.
 */
void drawCenteredTextLine(lgfx::LGFXBase& surface,
                          const char* text,
                          int16_t center_x,
                          int16_t top_y,
                          uint8_t text_size,
                          uint16_t fg,
                          uint16_t bg)
{
    surface.setTextSize(text_size);
    surface.setTextColor(fg, bg);
    surface.drawCenterString(text, center_x, top_y);
}

}  // namespace

/**
 * @brief Initializes display hardware state and internal HMI runtime state.
 *
 * Steps performed:
 * - wake display and apply visual defaults
 * - apply brightness, rotation, and color defaults
 * - reset screen/calibration/button state machine variables
 * - draw initial frame
 *
 * @return true when initialization path completes.
 */
bool HMI::init()
{
    M5_LOGI("[HMI] init");
    M5GFX& lcd = M5.Display;

    initialized_ = false;

    lcd.wakeup();
    lcd.setBrightness(DISPLAY_BRIGHTNESS);
    lcd.setTextSize(CONTENT_TEXT_SIZE);
    lcd.setRotation(DISPLAY_ROTATION);
    lcd.invertDisplay(false);

    // Show startup splash before entering normal page rendering.
    lcd.fillScreen(COLOR_SPLASH_BG);
    const char* splash = "SmartFranklin";
    lcd.setTextSize(CONTENT_TEXT_SIZE);
    const int16_t splashY = (lcd.height() - lcd.fontHeight()) / 2;
    drawCenteredTextLine(
        lcd,
        splash,
        static_cast<int16_t>(lcd.width() / 2),
        splashY,
        CONTENT_TEXT_SIZE,
        COLOR_SPLASH_TEXT,
        COLOR_SPLASH_BG);

    delay(SPLASH_DELAY_MS);

    lcd.fillScreen(COLOR_CONTENT_BG);

    screen_ = 2;
    calib_in_progress_ = false;
    btnA_prev_ = false;
    btnB_prev_ = false;
    last_published_screen_ = -1;

    draw();
    initialized_ = true;

    return true;
}

/**
 * @brief Executes one frame of HMI runtime processing.
 *
 * Work done each call:
 * - refresh button states (`M5.update`)
 * - detect navigation button edges
 * - redraw on page changes
 * - redraw current page to refresh telemetry
 * - process calibration button transitions
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
    }

    draw();

    handleCalibrationButton(btnB_rising);
}

/**
 * @brief Returns HMI availability status.
 */
bool HMI::isInitialized() const
{
    return initialized_;
}

const char* HMI::currentScreenName() const
{
    switch (screen_) {
    case 0: return "tank";
    case 1: return "gaz";
    case 2: return "level";
    case 3: return "battery";
    case 4: return "gps";
    case 5: return "rtc";
    case 6: return "calibration";
    default: return "level";
    }
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
}

/**
 * @brief Draws the top title band used by all pages.
 * @param title Page title text.
 */
void HMI::drawTitleBox(const char* title) const
{
    M5GFX& lcd = M5.Display;
    const int16_t boxW = static_cast<int16_t>(lcd.width()) - (2 * TITLE_BOX_X);

    lcd.fillRect(TITLE_BOX_X, TITLE_BOX_Y, boxW, TITLE_BOX_H, COLOR_TITLE_BG);
    lcd.drawRect(TITLE_BOX_X, TITLE_BOX_Y, boxW, TITLE_BOX_H, COLOR_TITLE_BORDER);

    lcd.setTextSize(TITLE_TEXT_SIZE);
    const int16_t textY = TITLE_BOX_Y + ((TITLE_BOX_H - lcd.fontHeight()) / 2);
    drawCenteredTextLine(
        lcd,
        title,
        static_cast<int16_t>(TITLE_BOX_X + (boxW / 2)),
        textY,
        TITLE_TEXT_SIZE,
        COLOR_TITLE_TEXT,
        COLOR_TITLE_BG);
}

/**
 * @brief Draws current page based on active screen index.
 */
void HMI::draw()
{
    DisplaySnapshot snapshot = last_snapshot_;
    updateSnapshot(snapshot);

    M5GFX& lcd = M5.Display;
    lcd.fillScreen(COLOR_CONTENT_BG);

    switch (screen_) {
    case 0: drawTankScreen(snapshot); break;
    case 1: drawGazScreen(snapshot); break;
    case 2: drawLevelScreen(snapshot); break;
    case 3: drawBatteryScreen(snapshot); break;
    case 4: drawGpsScreen(snapshot); break;
    case 5: drawRtcScreen(snapshot); break;
    case 6: drawCalibrationScreen(snapshot); break;
    default:
        drawLevelScreen(snapshot);
        break;
    }

    // Emit current screen when it changes so MQTT reflects UI navigation.
    if (last_published_screen_ != screen_) {
        sf_mqtt::publish("smartfranklin/hmi/screen", currentScreenName());
        last_published_screen_ = screen_;
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
    M5GFX& lcd = M5.Display;
    lcd.setTextSize((screen_ == 4 || screen_ == 5) ? 1 : CONTENT_TEXT_SIZE);
    lcd.setTextColor(COLOR_CONTENT_TEXT, COLOR_CONTENT_BG);
    lcd.setCursor(CONTENT_X, CONTENT_Y);
}

/** @brief Draw helper for Tank distance page. */
void HMI::drawTankScreen(const DisplaySnapshot& snapshot) const
{
    M5_LOGI("[HMI] drawTankScreen");
    M5GFX& lcd = M5.Display;
    drawTitleBox("Tank");
    beginContentArea();
    lcd.printf("Distance: %.1f cm\n", snapshot.distance_cm);
}

/** @brief Draw helper for Gaz weight page. */
void HMI::drawGazScreen(const DisplaySnapshot& snapshot) const
{
    M5_LOGI("[HMI] drawGazScreen");
    M5GFX& lcd = M5.Display;
    drawTitleBox("Gaz");
    beginContentArea();
    lcd.printf("Weight: %.3f g\n", snapshot.weight_g);
}

/** @brief Draw helper for Level tilt telemetry page. */
void HMI::drawLevelScreen(const DisplaySnapshot& snapshot) const
{
    M5_LOGI("[HMI] drawLevelScreen");
    M5GFX& lcd = M5.Display;
    drawTitleBox("Level");
    beginContentArea();
    lcd.printf("Pitch: %.2f\n", snapshot.pitch);
    lcd.printf("Roll:  %.2f\n", snapshot.roll);
}

/** @brief Draw helper for battery/BMS telemetry page. */
void HMI::drawBatteryScreen(const DisplaySnapshot& snapshot) const
{
    M5_LOGI("[HMI] drawBatteryScreen");
    M5GFX& lcd = M5.Display;
    drawTitleBox("Battery");
    beginContentArea();
    lcd.printf("BMS V: %.2f\nI: %.2f\nSOC: %.1f\n",
               snapshot.bms_voltage,
               snapshot.bms_current,
               snapshot.bms_soc);
}

/** @brief Draw helper for GPS telemetry page. */
void HMI::drawGpsScreen(const DisplaySnapshot& snapshot) const
{
    M5_LOGI("[HMI] drawGpsScreen");
    M5GFX& lcd = M5.Display;
    drawTitleBox("GPS");
    beginContentArea();
    lcd.printf("Fix: %s  Sat: %u\n",
               snapshot.gps_fix ? "YES" : "NO",
               snapshot.gps_satellites);
    lcd.printf("Lat: %.6f\n", snapshot.gps_lat);
    lcd.printf("Lon: %.6f\n", snapshot.gps_lon);
    lcd.printf("Alt: %.1f m\n", snapshot.gps_alt_m);
}

/** @brief Draw helper for RTC telemetry page. */
void HMI::drawRtcScreen(const DisplaySnapshot& snapshot) const
{
    M5_LOGI("[HMI] drawRtcScreen");
    M5GFX& lcd = M5.Display;
    drawTitleBox("RTC");
    beginContentArea();
    lcd.println("GPS UTC:");
    lcd.printf("%s %s\n", snapshot.gps_utc_date.c_str(), snapshot.gps_utc_time.c_str());
    lcd.println("RTC:");
    lcd.println(snapshot.gps_rtc_time.c_str());
}

/** @brief Draw helper for interactive scale calibration page. */
void HMI::drawCalibrationScreen(const DisplaySnapshot& snapshot) const
{
    M5_LOGI("[HMI] drawCalibrationScreen");
    M5GFX& lcd = M5.Display;
    drawTitleBox("Scale Calibration");
    beginContentArea();
    if (!calib_in_progress_) {
        lcd.println("Scale Calib");
        lcd.println("Put known weight");
        lcd.println("BtnB: start");
        return;
    }

    lcd.println("Calibrating...");
    lcd.printf("Known: %.2f kg\n", calib_known_weight_);
    lcd.printf("Raw:   %.3f kg\n", snapshot.weight_g);
    lcd.println("BtnB: finish");
}
