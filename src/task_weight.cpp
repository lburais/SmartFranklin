#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedWEIGHT.h>
#include <M5UnitUnifiedHUB.h>
#include <M5Utility.h>

#include "tasks.h"
#include "data_model.h"
#include "pahub_channels.h"
#include "mqtt_layer.h"

// WEIGHT I2C UNIT OVER PAHUB

using m5::unit::weighti2c::Mode;

namespace {
m5::unit::UnitUnified Units;

m5::unit::UnitWeightI2C unit;
m5::unit::UnitPaHub2    pahub{PAHUB_ADDRESS};

uint32_t idx{};
constexpr Mode mode_table[] = {Mode::Float, Mode::Int};

int32_t weight = 0;
}  

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

    if (!pahub.add(unit, PAHUB_CH_WEIGHT) ||      // Connect scale to pahub channel PAHUB_CH_WEIGHT
        !Units.add(pahub, Wire) ||                       // Connect pahub to core
        !Units.begin()) {
        M5_LOGE("[WEIGHT] not found.");
        M5_LOGW("%s", Units.debugInfo().c_str());

        while (true) {
            m5::utility::delay(10000);
        }
    }

    M5_LOGI("[WEIGHT] found.");
    M5_LOGI("%s", Units.debugInfo().c_str());

    unit.resetOffset();

    unit.writeGap(DATA.gap);

}

// LOOP

void loop()
{
    M5.update();

    Units.update();
    if (unit.updated()) {
        weight = unit.iweight();

        {
            std::lock_guard<std::mutex> lock(DATA_MUTEX);
            DATA.weight_g = weight;
        }

        sf_mqtt::publish("smartfranklin/weight/g", String(weight, 3).c_str());

        if (!idx) {
            M5.Log.printf("[WEIGHT] Weight:%f\n", unit.weight());
        } else {
            M5.Log.printf("[WEIGHT] iWeight:%d\n", unit.iweight());
        }

    }
}

void taskWeight(void *pv)
{
    M5_LOGI("[WEIGHT] Task started");

    setup();

    for (;;) {
            loop();
            vTaskDelay(pdMS_TO_TICKS(PERIOD_WEIGHT));
    }
}
