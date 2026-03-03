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
    if (!M5.Imu.isEnabled()) {
        M5_LOGE("[TILT] not found.");
        for (;;) {
            vTaskDelay(500);
        }
    }

    M5_LOGI("[TILT] found.");
}


// LOOP

void loop()
{
    auto data = M5.Imu.getImuData();

    // Calculs d’angles
    float pitch = atan2(data.accel.y, data.accel.z) * 57.2958f;
    float roll  = atan2(-data.accel.x, data.accel.z) * 57.2958f;

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

    // Print values
    M5.Log.printf("[TILT] pitch:%f  roll:%f\r\n", pitch, roll);

}

void taskTilt(void *pv)
{
    M5_LOGI("[TILT] Task started");

    setup();

    for (;;) {
            loop();
            vTaskDelay(pdMS_TO_TICKS(PERIOD_TILT));
    }

}
