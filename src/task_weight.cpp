#include <Arduino.h>
#include <Wire.h>
#include <M5UnitWeightI2C.h>
#include <M5Unit-PaHub.h>
#include "tasks.h"
#include "data_model.h"
#include "pahub_channels.h"
#include "mqtt_layer.h"
#include "config_store.h"


PaHub hubW;
M5Unit::WeightI2C scale;

void taskWeight(void *pv)
{
    Serial.println("[WEIGHT] Task started");
    hubW.begin(Wire);
    scale.begin(&hubW, PAHUB_CH_WEIGHT);
    scale.setCalFactor(CONFIG.scale_cal_factor);

for (;;) {
        float w = scale.getValue();
        {
            std::lock_guard<std::mutex> lock(DATA_MUTEX);
            DATA.weight_kg = w;
        }
        mqtt_publish("smartfranklin/weight/kg", String(w, 3));
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
