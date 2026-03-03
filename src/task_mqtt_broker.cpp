#include <Arduino.h>

#include "tasks.h"
#include "mqtt_layer.h"

void taskMqttBroker(void *pv)
{
    Serial.println("[MQTT] Broker task started");
    for (;;) {
        sf_mqtt::loop();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
