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

    // --- Initialisation PaHub + MPU6050 ---
    hubT.begin(Wire);
    hubT.selectPort(PAHUB_CH_MPU6050);

    if (!mpu.begin()) {
        Serial.println("[TILT] ERROR: MPU6050 not detected on PaHub");
        vTaskDelete(nullptr);
        return;
    }

    // Configuration IMU
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    for (;;) {

        // Sélection du port PaHub à chaque lecture (important)
        hubT.selectPort(PAHUB_CH_MPU6050);

        sensors_event_t a, g, t;
        mpu.getEvent(&a, &g, &t);

        // Calculs d’angles
        float pitch = atan2(a.acceleration.y, a.acceleration.z) * 57.2958f;
        float roll  = atan2(-a.acceleration.x, a.acceleration.z) * 57.2958f;

        // Mise à jour du modèle partagé
        {
            std::lock_guard<std::mutex> lock(DATA_MUTEX);
            DATA.pitch = pitch;
            DATA.roll  = roll;
        }

        // Publication MQTT
        sf_mqtt::publish("smartfranklin/tilt/pitch",
                 std::string(String(pitch, 1).c_str()));

        sf_mqtt::publish("smartfranklin/tilt/roll",
                 std::string(String(roll, 1).c_str()));

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
