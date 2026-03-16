#include <Arduino.h>
#include <M5Unified.h>

#include "config_store.h"
#include "gaz.h"

namespace {

static constexpr uint32_t GAZ_INIT_RETRY_MS = 10000UL;
static constexpr uint32_t GAZ_LOOP_FALLBACK_MS = 60000UL;

uint32_t gazLoopMs()
{
    const int configuredMs = CONFIG.task_gaz_loop_ms;
    return (configuredMs > 0) ? static_cast<uint32_t>(configuredMs)
                              : GAZ_LOOP_FALLBACK_MS;
}

}  // namespace

void taskGaz(void *pv)
{
    (void)pv;
    M5_LOGI("[GAZ] Task started");

    while (!GAZ_MODULE.init()) {
        M5_LOGW("[GAZ] init failed, retry in 10s");
        vTaskDelay(pdMS_TO_TICKS(GAZ_INIT_RETRY_MS));
    }

    for (;;) {
        GAZ_MODULE.process();
        vTaskDelay(pdMS_TO_TICKS(gazLoopMs()));
    }
}