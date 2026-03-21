/*
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
    String ap_ssid = "SmartFranklin-AP";
    String ap_pass = "smartfranklin";
    String sta_ssid = "jrdl";
    String sta_pass = "05121996190119942106196801071964";

    // Scale calibration
    float scale_cal_factor = 1.0f;

    // IMU geometry calibration (millimeters)
    float imu_wheelbase_mm = 2200.0f;
    float imu_track_width_mm = 1600.0f;
    float imu_offset_x_mm = 180.0f;
    float imu_offset_y_mm = -120.0f;

    // Admin credentials
    String admin_user = "admin";
    String admin_pass = "admin";

    // Local MQTT broker
    int mqtt_port = 1883;

    // NB-IoT
    bool nbiot_enabled = false;
    String nbiot_apn = "iot.1nce.net";
    String nbiot_mqtt_host;
    int nbiot_mqtt_port = 1883;
    String nbiot_mqtt_user;
    String nbiot_mqtt_pass;

    // Meshtastic bridge
    bool meshtastic_bridge_enabled = false;
    String meshtastic_mqtt_prefix = "smartfranklin/mesh/in/";
    int meshtastic_baud = 115200;
    int meshtastic_pin_rx = 33;
    int meshtastic_pin_tx = 32;

    // Task timing configuration (milliseconds)
    int task_gps_loop_ms = 60000;
    int task_mqtt_loop_ms = 250;
    int task_hmi_loop_ms = 1000;
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
