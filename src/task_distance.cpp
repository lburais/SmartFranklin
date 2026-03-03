#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedDISTANCE.h>
#include <M5UnitUnifiedHUB.h>
#include <M5Utility.h>

#include "tasks.h"
#include "data_model.h"
#include "pahub_channels.h"
#include "mqtt_layer.h"

// ULTRASONIC I2C UNIT OVER PAHUB

namespace {
m5::unit::UnitUnified       Units;
m5::unit::UnitUltraSonicI2C unit;
m5::unit::UnitPaHub2        pahub{PAHUB_ADDRESS};

int32_t                     distance = 0;
}  // namespace

// SETUP

void setup()
{
    m5::utility::delay(2000);

    M5.begin();

    auto pin_num_sda = M5.getPin(m5::pin_name_t::port_a_sda);
    auto pin_num_scl = M5.getPin(m5::pin_name_t::port_a_scl);
    M5_LOGI("getPin: SDA:%u SCL:%u", pin_num_sda, pin_num_scl);

    Wire.end();
    Wire.begin(pin_num_sda, pin_num_scl, 400000U);

    if (!pahub.add(unit, PAHUB_CH_DISTANCE) ||      // Connect scale to pahub channel PAHUB_CH_DISTANCE
        !Units.add(pahub, Wire) ||                       // Connect pahub to core
        !Units.begin()) {
        M5_LOGE("[DISTANCE] not found.");
        M5_LOGW("%s", Units.debugInfo().c_str());

        while (true) {
            m5::utility::delay(10000);
        }
    }

    M5_LOGI("[DISTANCE] found.");
    M5_LOGI("%s", Units.debugInfo().c_str());

}

// LOOP

void loop()
{
    M5.update();

    Units.update();
    if (unit.updated()) {

        distance = unit.distance();
        
        {
            std::lock_guard<std::mutex> lock(DATA_MUTEX);
            DATA.distance_cm = distance;
        }

        sf_mqtt::publish("smartfranklin/distance/cm", String(distance, 1).c_str());

        M5.Log.printf("[DISTANCE] Distance:%f\n", distance);
    }
}

void taskWeight(void *pv)
{
    M5_LOGI("[DISTANCE] Task started");

    setup();

    for (;;) {
            loop();
            vTaskDelay(pdMS_TO_TICKS(PERIOD_DISTANCE));
    }
}

