#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5Utility.h>

#include "tasks.h"
#include "data_model.h"
#include "pahub_channels.h"
#include "mqtt_layer.h"

// SETUP

void setup()
{
    if (!M5.Rtc.isEnabled()) {
        M5_LOGE("[RTC] not found.");
        for (;;) {
            vTaskDelay(500);
        }
    }

    M5_LOGI("[RTC] found.");

}


// LOOP

void loop()
{
    auto dt = M5.Rtc.getDateTime();

    char buf[32];

    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
                dt.date.year, dt.date.month, dt.date.date,
                dt.time.hours, dt.time.minutes, dt.time.seconds);

    // Mise à jour du modèle partagé
    {
        std::lock_guard<std::mutex> lock(DATA_MUTEX);
        DATA.rtc_time = buf;
    }

    // Publication MQTT
    sf_mqtt::publish("smartfranklin/rtc/time", std::string(String(buf).c_str()));

    // Affichage des valeurs
    M5.Log.printf("[RTC] time:%s\r\n", buf);

}

void taskRtc(void *pv)
{
    M5_LOGI("[RTC] Task started");

    setup();

    for (;;) {
            loop();
            vTaskDelay(pdMS_TO_TICKS(PERIOD_RTC));
    }

}
