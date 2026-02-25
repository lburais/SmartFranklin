#include "task_meshtastic_bridge.cpp"
#include "task_sensors.cpp"
#include "task_hw_monitor.cpp"
#include "task_hw_monitor.cpp"
#include "m5_hw.h"
#include <Arduino.h>
#include <M5Unified.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include "tasks.h"
#include "wifi_setup.h"
#include "mqtt_layer.h"
#include "command_handler.h"
#include "web_dashboard.h"
#include "config_store.h"
#include "captive_portal.h"
#include "plugin_manager.h"
#include "mqtt_bridge.h"

TaskHandle_t taskMqttBrokerHandle = nullptr;
TaskHandle_t taskDistanceHandle = nullptr;
TaskHandle_t taskWeightHandle = nullptr;
TaskHandle_t taskTiltHandle = nullptr;
TaskHandle_t taskRtcHandle = nullptr;
TaskHandle_t taskBmsBleHandle = nullptr;
TaskHandle_t taskDisplayHandle = nullptr;
TaskHandle_t taskMeshtasticBridgeHandle = nullptr;
TaskHandle_t taskNbiotHandle = nullptr;

void setup()
    xTaskCreatePinnedToCore(taskHwMonitor, "HW_MON", 4096, nullptr, 1, nullptr, 0);
{
    auto cfg = M5.config();
    M5.begin(cfg);
    Serial.begin(115200);
    SPIFFS.begin(true);

    config_load();

    setupWiFiApSta("SmartFranklin-AP", "smartfranklin",
                   CONFIG.sta_ssid.c_str(),
                   CONFIG.sta_pass.c_str());

    if (WiFi.status() != WL_CONNECTED) {
        captive_portal_start();
    }

    command_handler_init();
    mqtt_layer_init();
    mqtt_set_callback([](const String &topic, const String &payload){
        command_handle(topic, payload);
    });

    web_dashboard_init();
    mqtt_bridge_init();
    plugin_setup_all();

    xTaskCreatePinnedToCore(taskMqttBroker, "MQTT_BROKER", 4096, nullptr, 3, &taskMqttBrokerHandle, 1);
    xTaskCreatePinnedToCore(taskDistance,    "DISTANCE",    4096, nullptr, 2, &taskDistanceHandle,    1);
    xTaskCreatePinnedToCore(taskWeight,      "WEIGHT",      4096, nullptr, 2, &taskWeightHandle,      1);
    xTaskCreatePinnedToCore(taskTilt,        "TILT",        4096, nullptr, 2, &taskTiltHandle,        1);
    xTaskCreatePinnedToCore(taskRtc,         "RTC",         4096, nullptr, 2, &taskRtcHandle,         1);
    xTaskCreatePinnedToCore(taskBmsBle,      "BMS_BLE",     8192, nullptr, 2, &taskBmsBleHandle,      0);
    xTaskCreatePinnedToCore(taskDisplay,     "DISPLAY",     4096, nullptr, 1, &taskDisplayHandle,     1);
    xTaskCreatePinnedToCore(taskMeshtasticBridge, "MESH_BR", 8192, nullptr, 2, &taskMeshtasticBridgeHandle, 0);
    xTaskCreatePinnedToCore(taskNbiot,       "NB_IOT",      8192, nullptr, 2, &taskNbiotHandle,       0);
    xTaskCreatePinnedToCore(taskWatchdog,    "WATCHDOG",    2048, nullptr, 3, nullptr,                0);

    Serial.println("SmartFranklin setup complete.");
}

void loop()
{
    M5.update();

    static unsigned long pressStart = 0;
    if (M5.BtnA.isPressed()) {
        if (pressStart == 0) pressStart = millis();
        if (millis() - pressStart > 5000) {
            SPIFFS.remove("/config.json");
            ESP.restart();
        }
    } else {
        pressStart = 0;
    }

    mqtt_bridge_loop();
    plugin_loop_all();
    delay(50);
}
