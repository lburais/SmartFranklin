#pragma once
#include <Arduino.h>
#include <functional>

using MqttCallback = std::function<void(const String&, const String&)>;

void mqtt_layer_init();
void mqtt_publish(const String &topic, const String &payload);
void mqtt_set_callback(MqttCallback cb);
void mqtt_loop();
