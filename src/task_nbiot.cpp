#include <Arduino.h>
#include <M5Unified.h>
#include <M5Utility.h>

#include "tasks.h"
#include "nb_iot2.h"
#include "config_store.h"
#include "mqtt_layer.h"

void taskNbiot(void *pv)
{
    M5_LOGI("[NB_IOT] Task started");

    if (!CONFIG.nbiot_enabled) {
        M5_LOGW("[NB_IOT] Disabled in config");
        vTaskDelete(nullptr);
        return;
    }

    NB_IOT2.init(&Serial2, 115200, 13, 14);

    if (!NB_IOT2.connectNetwork(CONFIG.nbiot_apn)) {
        M5_LOGE("[NB_IOT] Network attach failed");
    } else {
        M5_LOGI("[NB_IOT] Network attached");
    }

    if (!CONFIG.nbiot_mqtt_host.isEmpty()) {
        NB_IOT2.mqttConnect(CONFIG.nbiot_mqtt_host,
                            CONFIG.nbiot_mqtt_port,
                            CONFIG.nbiot_mqtt_user,
                            CONFIG.nbiot_mqtt_pass);
    }

    for (;;) {
        NB_IOT2.loop();

        NbIotStatus st = NB_IOT2.getStatus();

        sf_mqtt::publish("smartfranklin/nbiot/rssi", String(st.rssi).c_str());
        sf_mqtt::publish("smartfranklin/nbiot/ip", st.ip.c_str());
        sf_mqtt::publish("smartfranklin/nbiot/operator", st.operator_name.c_str());

        GnssInfo g;
        if (NB_IOT2.getGnss(g) && g.valid) {
            String js = String("{\"lat\":") + String(g.lat, 6) +
                        ",\"lon\":" + String(g.lon, 6) +
                        ",\"alt\":" + String(g.alt, 1) + "}";
            sf_mqtt::publish("smartfranklin/nbiot/gnss", js.c_str());
        }

        vTaskDelay(pdMS_TO_TICKS(15000));
    }
}
