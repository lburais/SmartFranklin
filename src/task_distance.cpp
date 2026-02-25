#include <Arduino.h>
#include <Wire.h>
#include <M5Unit-Sonic.h>
#include <M5Unit-PaHub.h>
#include "tasks.h"
#include "data_model.h"
#include "pahub_channels.h"
#include "mqtt_layer.h"

PaHub hubDist;
UNIT_SONIC sonic;

void taskDistance(void *pv)
{
    Serial.println("[DISTANCE] Task started");
    hubDist.begin(Wire);
    sonic.begin(&hubDist, PAHUB_CH_ULTRASONIC);

    for (;;) {
        float d = sonic.getDistance();
        {
            std::lock_guard<std::mutex> lock(DATA_MUTEX);
            DATA.distance_cm = d;
        }
        mqtt_publish("smartfranklin/distance/cm", String(d, 1));
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
