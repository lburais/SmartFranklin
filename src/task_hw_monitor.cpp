#include <Arduino.h>
#include "tasks.h"
#include "m5_hw.h"
#include "mqtt_layer.h"
#include <M5Unified.h>

void taskHwMonitor(void *pv)
{
    Serial.println("[HW] Hardware monitor started");

    for (;;) {

        // --- Lecture IMU ---
        float ax = 0, ay = 0, az = 0;
        if (M5.Imu.isEnabled()) {
            M5.Imu.getAccel(&ax, &ay, &az);
        }

        // --- Lecture batterie ---
        float batt_voltage = M5.Power.getBatteryVoltage();   // en mV
        int   batt_percent = M5.Power.getBatteryLevel();     // en %
        bool  charging = M5.Power.isCharging();

        // --- Lecture température interne IMU (si dispo) ---
        float temp = 0;
        if (M5.Imu.isEnabled()) {
            //temp = M5.Imu.getTemperature();
        }

        // --- Boutons ---
        bool btnA = M5.BtnA.isPressed();
        bool btnB = M5.BtnB.isPressed();

        // --- Publication MQTT ---
        sf_mqtt::publish("smartfranklin/hw/battery_voltage",
                 std::string(String(batt_voltage).c_str()));

        sf_mqtt::publish("smartfranklin/hw/battery_percent",
                 std::string(String(batt_percent).c_str()));

        sf_mqtt::publish("smartfranklin/hw/charging",
                 std::string(charging ? "1" : "0"));

        //sf_mqtt::publish("smartfranklin/hw/temperature",
        //         std::string(String(st.temperature).c_str()));

        sf_mqtt::publish("smartfranklin/hw/button_a",
                 std::string(btnA ? "1" : "0"));

        sf_mqtt::publish("smartfranklin/hw/button_b",
                 std::string(btnB ? "1" : "0"));

        String accel = String("{\"x\":") + ax +
                       ",\"y\":" + ay +
                       ",\"z\":" + az + "}";

        sf_mqtt::publish("smartfranklin/hw/accel",
                 std::string(accel.c_str()));

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
