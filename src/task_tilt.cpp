#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <M5Unit-PaHub.h>
#include "tasks.h"
#include "data_model.h"
#include "pahub_channels.h"
#include "mqtt_layer.h"

PaHub hubT;
Adafruit_MPU6050 mpu;

void taskTilt(void *pv)
{
    Serial.println("[TILT] Task started");
    hubT.begin(Wire);
    hubT.selectPort(PAHUB_CH_MPU6050);
    mpu.begin();

    for (;;) {
        sensors_event_t a, g, t;
        mpu.getEvent(&a, &g, &t);
        float pitch = atan2(a.acceleration.y, a.acceleration.z) * 57.3;
        float roll  = atan2(-a.acceleration.x, a.acceleration.z) * 57.3;
        {
            std::lock_guard<std::mutex> lock(DATA_MUTEX);
            DATA.pitch = pitch;
            DATA.roll = roll;
        }
        mqtt_publish("smartfranklin/tilt/pitch", String(pitch, 1));
        mqtt_publish("smartfranklin/tilt/roll", String(roll, 1));
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
