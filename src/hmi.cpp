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
 *   - gaz.h (tare + calibration factor helpers)
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
#include <Adafruit_NeoPixel.h>

#include <memory>
#include <mutex>

#include "config_store.h"
#include "data_model.h"
#include "gaz.h"
#include "mqtt.h"
#include "interfaces.h"

namespace {

/** @brief Display rotation used for M5StickC Plus2 visual orientation. */
static constexpr uint8_t DISPLAY_ROTATION = 1;

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

constexpr size_t kBoardLedCount = 7;
constexpr size_t kBoardLedPin = 4;

std::mutex g_hmiLedMutex;
bool g_hmiLedConfigured = false;

size_t ledIndexForPortName(const sf_interfaces::InterfaceName name)
{
    switch (name) {
    case sf_interfaces::InterfaceName::PortA1: return 0;
    case sf_interfaces::InterfaceName::PortA2: return 6;
    case sf_interfaces::InterfaceName::PortB1: return 1;
    case sf_interfaces::InterfaceName::PortB2: return 5;
    case sf_interfaces::InterfaceName::PortC1: return 2;
    case sf_interfaces::InterfaceName::PortC2: return 4;
    default: return -1;
    }
}

Adafruit_NeoPixel strip(kBoardLedCount, kBoardLedPin, NEO_GRB + NEO_KHZ800);

bool initBoardLeds()
{
    if (g_hmiLedConfigured && M5.Led.getCount() == kBoardLedCount) {
        return true;
    }

    strip.begin();
    strip.setBrightness(50);   // 0–255, ici 50
    strip.show();

    g_hmiLedConfigured = true; 

    return g_hmiLedConfigured;
}

void setPortLedInitResultLocked(const sf_interfaces::InterfaceSensor sensor, const bool initialized)
{
    if (!initBoardLeds()) {
        return;
    }

    sf_interfaces::InterfaceName portName = sf_interfaces::getName(sensor);
    const size_t index = ledIndexForPortName(portName);
    if (index == -1) {
        return;
    }

    if (!initialized) {
        strip.setPixelColor(index, strip.Color(255, 0, 0));
        strip.show();
        return;
    }

    const sf_interfaces::InterfaceType type = sf_interfaces::getType(sensor);
    if (type == sf_interfaces::InterfaceType::I2C) {
        strip.setPixelColor(index, strip.Color(0, 255, 0));
    } else {
        strip.setPixelColor(index, strip.Color(0, 0, 255));
    }
    strip.show();
}

}  // namespace

void hmiSetAllBoardLedsWhite()
{
    std::lock_guard<std::mutex> lock(g_hmiLedMutex);

    if (!initBoardLeds()) {
        return;
    }

    strip.fill(strip.Color(255, 255, 255), 0, kBoardLedCount);
    strip.show();
}

void hmiSetPortLedStatus(const sf_interfaces::InterfaceSensor sensor, const bool initialized, const bool error)
{
    std::lock_guard<std::mutex> lock(g_hmiLedMutex);
    setPortLedInitResultLocked(sensor, initialized);
}

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

    hmiSetAllBoardLedsWhite();

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

    screen_ = 4;
    calib_in_progress_ = false;
    btnA_prev_ = false;
    btnB_prev_ = false;
    last_published_screen_ = -1;
    dial_encoder_prev_state_ = 0;
    dial_encoder_accum_ = 0;
    touch_hold_start_ms_ = 0;

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
    bool btnB_rising = M5.BtnB.wasPressed() || (btnB_now && !btnB_prev_);
    btnB_prev_ = btnB_now;

    if (btnA_rising) {
        next_screen = (next_screen + 1) % kScreenCount;
    }

    // Long touch (>5s) opens calibration page.
    bool anyTouchPressed = false;
    const auto touchCount = M5.Touch.getCount();
    for (uint8_t i = 0; i < touchCount; ++i) {
        if (M5.Touch.getDetail(i).isPressed()) {
            anyTouchPressed = true;
            break;
        }
    }

    if (anyTouchPressed) {
        if (touch_hold_start_ms_ == 0) {
            touch_hold_start_ms_ = millis();
        } else if ((millis() - touch_hold_start_ms_) >= 5000UL) {
            next_screen = 5;
        }
    } else {
        touch_hold_start_ms_ = 0;
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
    case 2: return "battery";
    case 3: return "gps";
    case 4: return "level";
    case 5: return "calibration";
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
    if (screen_ != 5 || !btnB_rising) {
        return;
    }

    if (!calib_in_progress_) {
        calib_in_progress_ = true;
        if (!scale_tare()) {
            calib_in_progress_ = false;
            M5_LOGW("[HMI] scale tare failed");
        }
        draw();
        return;
    }

    const float raw = scale_get_raw();
    if (raw != 0.0f) {
        const float knownWeightG = calib_known_weight_ * 1000.0f;
        const float factor = raw / knownWeightG;
        if (scale_set_cal_factor(factor)) {
            CONFIG.gaz_calibration_factor = factor;
            config_save();
        } else {
            M5_LOGW("[HMI] scale calibration apply failed");
        }
    }
    calib_in_progress_ = false;
    draw();
}

/**
 * @brief Draws the top title band used by all pages.
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
    case 2: drawBatteryScreen(snapshot); break;
    case 3: drawGpsScreen(snapshot); break;
    case 4: drawLevelScreen(snapshot); break;
    case 5: drawCalibrationScreen(snapshot); break;
    default:
        drawLevelScreen(snapshot);
        break;
    }

    // Emit current screen when it changes so MQTT reflects UI navigation.
    if (last_published_screen_ != screen_) {
        sf_mqtt::publish("smartfranklin/hmi/screen", currentScreenName(), 1, true);
        M5_LOGI("[HMI] %s", currentScreenName());
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

        snapshot.weight_gaz = DATA.weight_gaz;
        snapshot.fill_gaz = DATA.fill_gaz;
        snapshot.distance_tank_mm = DATA.distance_tank_mm;
        snapshot.fill_tank = DATA.fill_tank;
        snapshot.bms_voltage = DATA.bms_voltage;
        snapshot.bms_current = DATA.bms_current;
        snapshot.bms_soc = DATA.bms_soc;
        snapshot.level_pitch_deg = DATA.level_pitch_deg;
        snapshot.level_roll_deg = DATA.level_roll_deg;
        snapshot.level_wheel_fl_mm = DATA.level_wheel_fl_mm;
        snapshot.level_wheel_fr_mm = DATA.level_wheel_fr_mm;
        snapshot.level_wheel_rl_mm = DATA.level_wheel_rl_mm;
        snapshot.level_wheel_rr_mm = DATA.level_wheel_rr_mm;
        snapshot.gps_has_fix = DATA.gps_has_fix;
        snapshot.gps_satellites = DATA.gps_satellites;
        snapshot.gps_latitude_deg = DATA.gps_latitude_deg;
        snapshot.gps_longitude_deg = DATA.gps_longitude_deg;
        snapshot.gps_altitude_m = DATA.gps_altitude_m;
        snapshot.gps_speed_knots = DATA.gps_speed_knots;
        snapshot.gps_course_deg = DATA.gps_course_deg;
        snapshot.gps_date = DATA.gps_date;
        snapshot.gps_utc = DATA.gps_utc;
    }
    last_snapshot_ = snapshot;
}

/** @brief Draw helper for Tank ultrasonic page. */
void HMI::drawTankScreen(const DisplaySnapshot& snapshot) const {
    M5GFX& lcd = M5.Display;
    drawTitleBox("Tank");
    beginContentArea();
    lcd.printf("Distance: %d mm\nFill: %d%%", snapshot.distance_tank_mm, snapshot.fill_tank);
}

/**
 * @brief Applies common content text settings and cursor placement.
 */
void HMI::beginContentArea() const
{
    M5GFX& lcd = M5.Display;
    //lcd.setTextSize((screen_ == 4 || screen_ == 5) ? 1 : CONTENT_TEXT_SIZE);
    lcd.setTextSize(CONTENT_TEXT_SIZE);
    lcd.setTextColor(COLOR_CONTENT_TEXT, COLOR_CONTENT_BG);
    lcd.setCursor(CONTENT_X, CONTENT_Y);
}

/** @brief Draw helper for Gaz weight page. */
void HMI::drawGazScreen(const DisplaySnapshot& snapshot) const
{
    M5GFX& lcd = M5.Display;
    drawTitleBox("Gaz");

    const int16_t bottleW = 86;
    const int16_t bottleH = 126;
    const int16_t bottleX = static_cast<int16_t>((lcd.width() - bottleW) / 2);
    const int16_t bottleY = CONTENT_Y + 4;

    drawGazBottle(bottleX, bottleY, bottleW, bottleH, snapshot.fill_gaz);

    lcd.setTextSize(1);
    lcd.setTextColor(COLOR_CONTENT_TEXT, COLOR_CONTENT_BG);
    char weightText[32];
    snprintf(weightText, sizeof(weightText), "%d g", snapshot.weight_gaz);
    lcd.drawCenterString(weightText, static_cast<int16_t>(lcd.width() / 2), static_cast<int16_t>(bottleY + bottleH + 4));

    const int16_t footerY = static_cast<int16_t>(lcd.height() - lcd.fontHeight() - 3);
    lcd.drawCenterString("calibrate", static_cast<int16_t>(lcd.width() / 2), footerY);
}

void HMI::drawGazBottle(int16_t x, int16_t y, int16_t w, int16_t h, int32_t fillPct) const
{
    M5GFX& lcd = M5.Display;

    const int32_t clampedFill = std::max<int32_t>(0, std::min<int32_t>(100, fillPct));

    const int16_t neckW = static_cast<int16_t>(w / 3);
    const int16_t neckH = 14;
    const int16_t neckX = static_cast<int16_t>(x + (w - neckW) / 2);
    const int16_t neckY = y;
    const int16_t bodyY = static_cast<int16_t>(y + neckH - 2);
    const int16_t bodyH = static_cast<int16_t>(h - neckH + 2);

    const uint16_t bottleBorder = 0xFFFF;
    const uint16_t bottleBody = 0x6B4D;
    const uint16_t fillColor = 0x07E0;

    lcd.fillRoundRect(x, bodyY, w, bodyH, 20, bottleBody);
    lcd.drawRoundRect(x, bodyY, w, bodyH, 20, bottleBorder);
    lcd.fillRoundRect(neckX, neckY, neckW, neckH, 4, bottleBody);
    lcd.drawRoundRect(neckX, neckY, neckW, neckH, 4, bottleBorder);

    const int16_t innerPad = 6;
    const int16_t innerX = static_cast<int16_t>(x + innerPad);
    const int16_t innerY = static_cast<int16_t>(bodyY + innerPad);
    const int16_t innerW = static_cast<int16_t>(w - 2 * innerPad);
    const int16_t innerH = static_cast<int16_t>(bodyH - 2 * innerPad);

    const int16_t fillH = static_cast<int16_t>((innerH * clampedFill) / 100);
    const int16_t fillY = static_cast<int16_t>(innerY + innerH - fillH);

    lcd.drawRoundRect(innerX, innerY, innerW, innerH, 10, 0xC618);
    if (fillH > 0) {
        lcd.fillRect(innerX + 1, fillY, innerW - 2, fillH, fillColor);
    }

    char pct[8];
    snprintf(pct, sizeof(pct), "%ld%%", static_cast<long>(clampedFill));
    lcd.setTextSize(2);
    lcd.setTextColor(COLOR_CONTENT_TEXT, COLOR_CONTENT_BG);
    lcd.drawCenterString(pct, static_cast<int16_t>(x + (w / 2)), static_cast<int16_t>(innerY + (innerH / 2) - 8));
}

/** @brief Draw helper for battery/BMS telemetry page. */
void HMI::drawBatteryScreen(const DisplaySnapshot& snapshot) const
{
    M5GFX& lcd = M5.Display;
    drawTitleBox("Battery");
    beginContentArea();
    lcd.printf("BMS V: %.2f\nBMS I: %.2f\nSOC: %.1f",
               snapshot.bms_voltage,
               snapshot.bms_current,
               snapshot.bms_soc);
}

/** @brief Draw helper for GPS telemetry page. */
void HMI::drawGpsScreen(const DisplaySnapshot& snapshot) const
{
    M5GFX& lcd = M5.Display;
    drawTitleBox("GPS");
    beginContentArea();
    lcd.printf("Fix: %s\n", snapshot.gps_has_fix ? "YES" : "NO");
    lcd.printf("Sats: %u\n", static_cast<unsigned>(snapshot.gps_satellites));
    lcd.printf("Lat: %.5f\n", snapshot.gps_latitude_deg);
    lcd.printf("Lon: %.5f\n", snapshot.gps_longitude_deg);
    lcd.printf("Alt: %.1f m\n", snapshot.gps_altitude_m);
    lcd.printf("Spd: %.1f kn\n", snapshot.gps_speed_knots);
    lcd.printf("COG: %.1f\n", snapshot.gps_course_deg);
    lcd.printf("UTC: %s %s\n", snapshot.gps_date.c_str(), snapshot.gps_utc.c_str());
}

/** @brief Draw helper for level telemetry page. */
void HMI::drawLevelScreen(const DisplaySnapshot& snapshot) const
{
    M5GFX& lcd = M5.Display;
    drawTitleBox("Level");
    beginContentArea();
    const float fl_cm = snapshot.level_wheel_fl_mm / 10.0f;
    const float fr_cm = snapshot.level_wheel_fr_mm / 10.0f;
    const float rl_cm = snapshot.level_wheel_rl_mm / 10.0f;
    const float rr_cm = snapshot.level_wheel_rr_mm / 10.0f;
    lcd.printf("Pitch: %.2f\n", snapshot.level_pitch_deg);
    lcd.printf("Roll: %.2f\n", snapshot.level_roll_deg);
    lcd.printf("RL: %.0f cm - FL: %.0f cm\n", rl_cm, fl_cm);
    lcd.printf("RR: %.0f cm - FR: %.0f cm\n", rr_cm, fr_cm);
}

/** @brief Draw helper for interactive scale calibration page. */
void HMI::drawCalibrationScreen(const DisplaySnapshot& snapshot) const
{
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
    lcd.printf("Raw:   %.3f kg\n", static_cast<float>(snapshot.weight_gaz) / 1000.0f);
    lcd.println("BtnB: finish");
}
