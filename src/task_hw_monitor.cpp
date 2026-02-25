#include <Arduino.h>
#include "tasks.h"
#include "m5_hw.h"
#include "mqtt_layer.h"

void taskHwMonitor(void *pv)
{
    Serial.println("[HW] Hardware monitor started");

    for (;;) {
        HwStatus st = HW.read();

        mqtt_publish("smartfranklin/hw/battery_voltage", String(st.battery_voltage));
        mqtt_publish("smartfranklin/hw/battery_percent", String(st.battery_percent));
        mqtt_publish("smartfranklin/hw/charging", st.charging ? "1" : "0");
        mqtt_publish("smartfranklin/hw/temperature", String(st.temperature));

        mqtt_publish("smartfranklin/hw/button_a", st.button_a ? "1" : "0");
        mqtt_publish("smartfranklin/hw/button_b", st.button_b ? "1" : "0");

        String accel = String("{\"x\":") + st.accel_x +
                       ",\"y\":" + st.accel_y +
                       ",\"z\":" + st.accel_z + "}";
        mqtt_publish("smartfranklin/hw/accel", accel);

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
