#include <Arduino.h>
#include <M5Unified.h>

#include "config_store.h"
#include "gaz.h"
#include "tank.h"

namespace {

static constexpr uint32_t I2C_SENSORS_INIT_RETRY_MS = 10000UL;
static constexpr uint32_t I2C_SENSORS_LOOP_FALLBACK_MS = 60000UL;

uint32_t i2cSensorsLoopMs()
{
    const int gazMs = CONFIG.task_gaz_loop_ms;
    const int tankMs = CONFIG.task_tank_loop_ms;

    if (gazMs > 0 && tankMs > 0) {
        return static_cast<uint32_t>((gazMs < tankMs) ? gazMs : tankMs);
    }

    if (gazMs > 0) {
        return static_cast<uint32_t>(gazMs);
    }

    if (tankMs > 0) {
        return static_cast<uint32_t>(tankMs);
    }

    return I2C_SENSORS_LOOP_FALLBACK_MS;
}

}  // namespace

void taskI2cSensors(void *pv)
{
    (void)pv;
    M5_LOGI("[I2C_SENSORS] Task started");

    bool gazInitialized = false;
    bool tankInitialized = false;

    // I2C init

    for (;;) {
#ifndef DISABLE_GAZ
        if (!gazInitialized) {
            gazInitialized = GAZ_MODULE.init();
            if (!gazInitialized) {
                M5_LOGW("[I2C_SENSORS] GAZ init failed");
            }
        }
#else
        gazInitialized = true;
#endif

#ifndef DISABLE_TANK
        if (!tankInitialized) {
            tankInitialized = TANK_MODULE.init();
            if (!tankInitialized) {
                M5_LOGW("[I2C_SENSORS] TANK init failed");
            }
        }
#else
        tankInitialized = true;
#endif

        if (gazInitialized && tankInitialized) {
            break;
        }

        M5_LOGW("[I2C_SENSORS] init incomplete, retry in 10s");
        vTaskDelay(pdMS_TO_TICKS(I2C_SENSORS_INIT_RETRY_MS));
    }

    for (;;) {
#ifndef DISABLE_GAZ
        GAZ_MODULE.process();
#endif
#ifndef DISABLE_TANK
        TANK_MODULE.process();
#endif

        vTaskDelay(pdMS_TO_TICKS(i2cSensorsLoopMs()));
    }
}
