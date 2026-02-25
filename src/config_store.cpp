#include "config_store.h"
#include <ArduinoJson.h>
#include <FS.h>
#include <SPIFFS.h>

SmartConfig CONFIG;
static const char *CFG_PATH = "/config.json";

bool config_load()
{
    if (!SPIFFS.begin(true)) return false;
    if (!SPIFFS.exists(CFG_PATH)) {
        CONFIG.sta_ssid = "";
        CONFIG.sta_pass = "";
        CONFIG.scale_cal_factor = 1.0f;
        CONFIG.admin_user = "admin";
        CONFIG.admin_pass = "admin";

        CONFIG.ext_mqtt_host = "";
        CONFIG.ext_mqtt_port = 1883;
        CONFIG.ext_mqtt_user = "";
        CONFIG.ext_mqtt_pass = "";
        CONFIG.ext_mqtt_enabled = false;

        CONFIG.mqtt_bridge_enabled = true;
        CONFIG.mqtt_bridge_prefix_internal = "local/";
        CONFIG.mqtt_bridge_prefix_external = "cloud/";
        CONFIG.mqtt_bridge_qos = 1;
        CONFIG.mqtt_bridge_retain = false;
        CONFIG.mqtt_bridge_loop_detection = true;
    doc["nbiot_enabled"]   = CONFIG.nbiot_enabled; 
    doc["nbiot_apn"]       = CONFIG.nbiot_apn; 
    doc["nbiot_mqtt_host"] = CONFIG.nbiot_mqtt_host; 
    doc["nbiot_mqtt_port"] = CONFIG.nbiot_mqtt_port; 
    doc["nbiot_mqtt_user"] = CONFIG.nbiot_mqtt_user; 
    doc["nbiot_mqtt_pass"] = CONFIG.nbiot_mqtt_pass;
    CONFIG.nbiot_enabled   = doc["nbiot_enabled"]   | true; 
    CONFIG.nbiot_apn       = (const char*)doc["nbiot_apn"]       | "iot.1nce.net"; 
    CONFIG.nbiot_mqtt_host = (const char*)doc["nbiot_mqtt_host"] | ""; 
    CONFIG.nbiot_mqtt_port = doc["nbiot_mqtt_port"] | 1883; 
    CONFIG.nbiot_mqtt_user = (const char*)doc["nbiot_mqtt_user"] | ""; 
    CONFIG.nbiot_mqtt_pass = (const char*)doc["nbiot_mqtt_pass"] | "";
        return true;
    }

    File f = SPIFFS.open(CFG_PATH, "r");
    if (!f) return false;

    DynamicJsonDocument doc(2048);
    DeserializationError err = deserializeJson(doc, f);
    f.close();
    if (err) return false;

    CONFIG.sta_ssid = (const char*)doc["sta_ssid"] | "";
    CONFIG.sta_pass = (const char*)doc["sta_pass"] | "";
    CONFIG.scale_cal_factor = doc["scale_cal_factor"] | 1.0f;
    CONFIG.admin_user = (const char*)doc["admin_user"] | "admin";
    CONFIG.admin_pass = (const char*)doc["admin_pass"] | "admin";

    CONFIG.ext_mqtt_host = (const char*)doc["ext_mqtt_host"] | "";
    CONFIG.ext_mqtt_port = doc["ext_mqtt_port"] | 1883;
    CONFIG.ext_mqtt_user = (const char*)doc["ext_mqtt_user"] | "";
    CONFIG.ext_mqtt_pass = (const char*)doc["ext_mqtt_pass"] | "";
    CONFIG.ext_mqtt_enabled = doc["ext_mqtt_enabled"] | false;

    CONFIG.mqtt_bridge_enabled = doc["mqtt_bridge_enabled"] | true;
    CONFIG.mqtt_bridge_prefix_internal = (const char*)doc["mqtt_bridge_prefix_internal"] | "local/";
    CONFIG.mqtt_bridge_prefix_external = (const char*)doc["mqtt_bridge_prefix_external"] | "cloud/";
    CONFIG.mqtt_bridge_qos = doc["mqtt_bridge_qos"] | 1;
    CONFIG.mqtt_bridge_retain = doc["mqtt_bridge_retain"] | false;
    CONFIG.mqtt_bridge_loop_detection = doc["mqtt_bridge_loop_detection"] | true;
    doc["nbiot_enabled"]   = CONFIG.nbiot_enabled; 
    doc["nbiot_apn"]       = CONFIG.nbiot_apn; 
    doc["nbiot_mqtt_host"] = CONFIG.nbiot_mqtt_host; 
    doc["nbiot_mqtt_port"] = CONFIG.nbiot_mqtt_port; 
    doc["nbiot_mqtt_user"] = CONFIG.nbiot_mqtt_user; 
    doc["nbiot_mqtt_pass"] = CONFIG.nbiot_mqtt_pass;
    CONFIG.nbiot_enabled   = doc["nbiot_enabled"]   | true; 
    CONFIG.nbiot_apn       = (const char*)doc["nbiot_apn"]       | "iot.1nce.net"; 
    CONFIG.nbiot_mqtt_host = (const char*)doc["nbiot_mqtt_host"] | ""; 
    CONFIG.nbiot_mqtt_port = doc["nbiot_mqtt_port"] | 1883; 
    CONFIG.nbiot_mqtt_user = (const char*)doc["nbiot_mqtt_user"] | ""; 
    CONFIG.nbiot_mqtt_pass = (const char*)doc["nbiot_mqtt_pass"] | "";

    return true;
}

bool config_save()
{
    DynamicJsonDocument doc(2048);
    doc["sta_ssid"] = CONFIG.sta_ssid;
    doc["sta_pass"] = CONFIG.sta_pass;
    doc["scale_cal_factor"] = CONFIG.scale_cal_factor;
    doc["admin_user"] = CONFIG.admin_user;
    doc["admin_pass"] = CONFIG.admin_pass;

    doc["ext_mqtt_host"] = CONFIG.ext_mqtt_host;
    doc["ext_mqtt_port"] = CONFIG.ext_mqtt_port;
    doc["ext_mqtt_user"] = CONFIG.ext_mqtt_user;
    doc["ext_mqtt_pass"] = CONFIG.ext_mqtt_pass;
    doc["ext_mqtt_enabled"] = CONFIG.ext_mqtt_enabled;

    doc["mqtt_bridge_enabled"] = CONFIG.mqtt_bridge_enabled;
    doc["mqtt_bridge_prefix_internal"] = CONFIG.mqtt_bridge_prefix_internal;
    doc["mqtt_bridge_prefix_external"] = CONFIG.mqtt_bridge_prefix_external;
    doc["mqtt_bridge_qos"] = CONFIG.mqtt_bridge_qos;
    doc["mqtt_bridge_retain"] = CONFIG.mqtt_bridge_retain;
    doc["mqtt_bridge_loop_detection"] = CONFIG.mqtt_bridge_loop_detection;
    doc["nbiot_enabled"]   = CONFIG.nbiot_enabled; 
    doc["nbiot_apn"]       = CONFIG.nbiot_apn; 
    doc["nbiot_mqtt_host"] = CONFIG.nbiot_mqtt_host; 
    doc["nbiot_mqtt_port"] = CONFIG.nbiot_mqtt_port; 
    doc["nbiot_mqtt_user"] = CONFIG.nbiot_mqtt_user; 
    doc["nbiot_mqtt_pass"] = CONFIG.nbiot_mqtt_pass;
    CONFIG.nbiot_enabled   = doc["nbiot_enabled"]   | true; 
    CONFIG.nbiot_apn       = (const char*)doc["nbiot_apn"]       | "iot.1nce.net"; 
    CONFIG.nbiot_mqtt_host = (const char*)doc["nbiot_mqtt_host"] | ""; 
    CONFIG.nbiot_mqtt_port = doc["nbiot_mqtt_port"] | 1883; 
    CONFIG.nbiot_mqtt_user = (const char*)doc["nbiot_mqtt_user"] | ""; 
    CONFIG.nbiot_mqtt_pass = (const char*)doc["nbiot_mqtt_pass"] | "";

    File f = SPIFFS.open(CFG_PATH, "w");
    if (!f) return false;
    serializeJson(doc, f);
    f.close();
    return true;
}
