#include "mqtt_layer.h"
#include "config_store.h"
#include <PubSubClient.h>
#include <WiFi.h>

static WiFiClient wifiClient;
static PubSubClient mqttClient(wifiClient);
static MqttCallback g_cb = nullptr;

void mqtt_layer_init()
{
    if (CONFIG.ext_mqtt_enabled && CONFIG.ext_mqtt_host.length() > 0) {
        mqttClient.setServer(CONFIG.ext_mqtt_host.c_str(), CONFIG.ext_mqtt_port);
    } else {
        mqttClient.setServer("127.0.0.1", 1883);
    }
}

void mqtt_publish(const String &topic, const String &payload)
{
    mqttClient.publish(topic.c_str(), payload.c_str());
}

void mqtt_set_callback(MqttCallback cb)
{
    g_cb = cb;
    mqttClient.setCallback([](char *topic, byte *payload, unsigned int len){
        if (!g_cb) return;
        String t = topic;
        String p;
        p.reserve(len);
        for (unsigned int i = 0; i < len; ++i) p += (char)payload[i];
        g_cb(t, p);
    });
}

void mqtt_loop()
{
    if (!mqttClient.connected()) {
        if (CONFIG.ext_mqtt_enabled && CONFIG.ext_mqtt_host.length() > 0) {
            mqttClient.connect("SmartFranklinExt",
                               CONFIG.ext_mqtt_user.c_str(),
                               CONFIG.ext_mqtt_pass.c_str());
        } else {
            mqttClient.connect("SmartFranklinInt");
        }
    }
    mqttClient.loop();
}
