#pragma once
#include <ArduinoJson.h>

void mqtt_bridge_init();
void mqtt_bridge_loop();
void mqtt_bridge_status(JsonDocument &doc);
