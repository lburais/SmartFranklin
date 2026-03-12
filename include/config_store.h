/*
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

    // Admin credentials
    String admin_user = "admin";
    String admin_pass = "admin";

    // External MQTT
    String ext_mqtt_host;
    int ext_mqtt_port = 1883;
    String ext_mqtt_user;
    String ext_mqtt_pass;
    bool ext_mqtt_enabled = false;

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
