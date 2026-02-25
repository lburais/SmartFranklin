#include <Arduino.h>
#include <M5Unified.h>
#include "tasks.h"
#include "data_model.h"
#include "mqtt_layer.h"

void taskRtc(void *pv)
{
    Serial.println("[RTC] Task started");
    for (;;) {
        auto t = M5.Rtc.getDateTime();
        char buf[32];
        snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
                 t.date.year, t.date.month, t.date.day,
                 t.time.hours, t.time.minutes, t.time.seconds);
        {
            std::lock_guard<std::mutex> lock(DATA_MUTEX);
            DATA.rtc_time = buf;
        }
        mqtt_publish("smartfranklin/time", String(buf));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
