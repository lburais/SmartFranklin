#include <Arduino.h>
#include <M5Unified.h>

#include "config_store.h"
#include "tank.h"

namespace {

static constexpr uint32_t TANK_INIT_RETRY_MS = 10000UL;
static constexpr uint32_t TANK_LOOP_FALLBACK_MS = 60000UL;

uint32_t tankLoopMs()
{
    const int configuredMs = CONFIG.task_tank_loop_ms;
    return (configuredMs > 0) ? static_cast<uint32_t>(configuredMs)
                              : TANK_LOOP_FALLBACK_MS;
}

}  // namespace

void taskTank(void *pv)
{
    (void)pv;
    M5_LOGI("[TANK] Task started");

    while (!TANK_MODULE.init()) {
        M5_LOGW("[TANK] init failed, retry in 10s");
        vTaskDelay(pdMS_TO_TICKS(TANK_INIT_RETRY_MS));
    }

    for (;;) {
        TANK_MODULE.process();
        vTaskDelay(pdMS_TO_TICKS(tankLoopMs()));
    }
}
