#pragma once

#include <Arduino.h>

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

    // MQTT bridge
    bool mqtt_bridge_enabled = false;
    String mqtt_bridge_prefix_internal = "local/";
    String mqtt_bridge_prefix_external = "cloud/";
    int mqtt_bridge_qos = 1;
    bool mqtt_bridge_retain = false;
    bool mqtt_bridge_loop_detection = true;

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
};

extern SmartConfig CONFIG;

bool config_load();
bool config_save();
