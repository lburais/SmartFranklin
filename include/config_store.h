/**
 * @file config_store.h
 * SPDX-License-Identifier: MIT
 *
 * ============================================================================
 * Configuration Store Interface - SmartFranklin
 * ============================================================================
 *
 * File:        config_store.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Declares persistent runtime configuration schema and load/save
 *              entry points backed by SPIFFS.
 *
 * Author:      Laurent Burais
 * Date:        10 March 2026
 * Version:     1.1
 *
 * ============================================================================
 * MIT License
 * ============================================================================
 * Copyright (c) 2026 Laurent Burais
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * ============================================================================
 */

#pragma once

#include <Arduino.h>

/**
 * @brief Persistent SmartFranklin configuration model.
 *
 * Fields are initialized with safe defaults used when no persisted config is
 * available yet.
 */
struct SmartConfig {
    // WiFi
    String hostname = "franklin";
    String ap_ssid = "SmartFranklin-AP";
    String ap_pass = "smartfranklin";
    String sta_ssid = "jrdl";
    String sta_pass = "05121996190119942106196801071964";

    // Gaz calibration and smoothing
    float gaz_calibration_factor = 36.0f;
    int gaz_weight_average_window = 10;
    String gaz_i2c_port = "A1";

    String tank_i2c_port = "A2";
    
    String rtc_timezone = "Europe/Paris";
    
    String gps_i2c_port = "B1";

    // Level geometry calibration (millimeters)
    // Renault Trafic III L2H1
    float level_wheelbase_mm = 3498.0f;
    float level_track_width_mm = 1662.0f;
    float level_offset_x_mm = 0.0f;
    float level_offset_y_mm = 0.0f;
    float level_zero_pitch_deg = 0.0f;
    float level_zero_roll_deg = 0.0f;

    // Admin credentials
    String admin_user = "admin";
    String admin_pass = "admin";

    // Local MQTT broker
    int mqtt_port = 1883;

    // Task timing configuration (milliseconds)
    int task_wifi_loop_ms = 500;
    int task_mqtt_loop_ms = 250;
    int task_i2c_loop_ms = 1000;
    int task_hmi_loop_ms = 1000;
    int task_hmi_init_retry_ms = 1000;
    int task_hw_monitor_loop_ms = 5000;
    int task_bms_ble_connected_loop_ms = 10000;
    int task_bms_ble_retry_loop_ms = 50000;
    int task_watchdog_loop_ms = 60000;
};

extern SmartConfig CONFIG;

/**
 * @brief Load persisted config from storage into @ref CONFIG.
 * @return True on successful parse/load, false when defaults are kept.
 */
bool config_load();

/**
 * @brief Persist current @ref CONFIG values to storage.
 * @return True when write succeeds, false otherwise.
 */
bool config_save();
