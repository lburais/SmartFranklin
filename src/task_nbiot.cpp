#include <Arduino.h>
#include "tasks.h"
#include "nb_iot2.h"
#include "config_store.h"
#include "mqtt_layer.h"

void taskNbiot(void *pv)
{
    Serial.println("[NB_IOT] Task started");

    if (!CONFIG.nbiot_enabled) {
        Serial.println("[NB_IOT] Disabled in config");
        vTaskDelete(nullptr);
        return;
    }

    NB_IOT2.init(&Serial2, 115200, 13, 14);

    if (!NB_IOT2.connectNetwork(CONFIG.nbiot_apn)) {
        Serial.println("[NB_IOT] Network attach failed");
    } else {
        Serial.println("[NB_IOT] Network attached");
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
        mqtt_publish("smartfranklin/nbiot/rssi", String(st.rssi));
        mqtt_publish("smartfranklin/nbiot/ip", st.ip);
        mqtt_publish("smartfranklin/nbiot/operator", st.operator_name);

        GnssInfo g;
        if (NB_IOT2.getGnss(g) && g.valid) {
            String js = String("{\"lat\":") + String(g.lat, 6) +
                        ",\"lon\":" + String(g.lon, 6) +
                        ",\"alt\":" + String(g.alt, 1) + "}";
            mqtt_publish("smartfranklin/nbiot/gnss", js);
        }

        vTaskDelay(pdMS_TO_TICKS(15000));
    }
}
