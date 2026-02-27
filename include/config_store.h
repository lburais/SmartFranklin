#pragma once
#include <Arduino.h>

struct SmartConfig {
    // WiFi
    String sta_ssid;
    String sta_pass;

    // Scale calibration
    float scale_cal_factor;

    // Admin credentials
    String admin_user;
    String admin_pass;

    // External MQTT
    String ext_mqtt_host;
    int    ext_mqtt_port;
    String ext_mqtt_user;
    String ext_mqtt_pass;
    bool   ext_mqtt_enabled;

    // MQTT bridge
    bool   mqtt_bridge_enabled;
    String mqtt_bridge_prefix_internal;
    String mqtt_bridge_prefix_external;
    int    mqtt_bridge_qos;
    bool   mqtt_bridge_retain;
    bool   mqtt_bridge_loop_detection;

    // NB-IoT
    bool   nbiot_enabled;
    String nbiot_apn;
    String nbiot_mqtt_host;
    int    nbiot_mqtt_port;
    String nbiot_mqtt_user;
    String nbiot_mqtt_pass;
};

// Global config instance
extern SmartConfig CONFIG;

// Load/save API
bool config_load();
bool config_save();
