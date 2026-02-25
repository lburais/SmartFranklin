#pragma once
#include <Arduino.h>

struct SmartConfig {
    String sta_ssid;
    String sta_pass;
    float  scale_cal_factor;
    bool   nbiot_enabled; 
    String nbiot_apn; 
    String nbiot_mqtt_host; 
    int    nbiot_mqtt_port; 
    String nbiot_mqtt_user; 
    String nbiot_mqtt_pass;
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
};

extern SmartConfig CONFIG;

bool config_load();
bool config_save();
