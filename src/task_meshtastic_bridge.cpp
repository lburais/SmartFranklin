#include <Arduino.h>
#include "tasks.h"
#include "meshtastic_bridge.h"
#include "data_model.h"
#include "mqtt_layer.h"

void taskMeshtasticBridge(void *pv)
{
    Serial.println("[MESH] Task started");
    meshtastic_bridge_init();

    for (;;) {
        String msg;

        if (meshtastic_poll_received(msg)) {

            {
                std::lock_guard<std::mutex> lock(DATA_MUTEX);
                DATA.last_mesh_msg = msg;
            }

            sf_mqtt::publish(
                "smartfranklin/mesh/in",
                std::string(msg.c_str())
            );
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
