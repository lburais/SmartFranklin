/*
 * SmartFranklin - GPS task wrapper
 * SPDX-License-Identifier: MIT
 *
 * FreeRTOS task entrypoint that initializes the GPS module and executes
 * periodic processing cycles.
 */

#include <Arduino.h>
#include <M5Unified.h>

#include "gps.h"
#include "config_store.h"

namespace {

/** @brief Fixed delay between failed GPS init attempts. */
static constexpr uint32_t GPS_INIT_RETRY_MS = 10000UL;

/** @brief Safe default loop period when config value is invalid. */
static constexpr uint32_t GPS_LOOP_FALLBACK_MS = 60000UL;

/**
 * @brief Returns validated GPS loop period in milliseconds.
 *
 * Uses persisted config when valid, otherwise applies a conservative fallback.
 */
uint32_t gpsLoopMs()
{
    const int configuredMs = CONFIG.task_gps_loop_ms;
    return (configuredMs > 0) ? static_cast<uint32_t>(configuredMs)
                              : GPS_LOOP_FALLBACK_MS;
}

}  // namespace

/**
 * @brief FreeRTOS task for DFR1103 lifecycle.
 *
 * Task flow:
 * 1. Retry init() every 10s until module becomes available.
 * 2. Loop: process one GPS sample.
 *
 * @param pv Unused task parameter.
 */
void taskGps(void *pv)
{
    (void)pv;
    M5_LOGI("[GPS] Task started");

    // Keep retrying until the module is physically reachable and initialized.
    while (!GPS_MODULE.init()) {
        M5_LOGW("[GPS] init failed, retry in 10s");
        vTaskDelay(pdMS_TO_TICKS(GPS_INIT_RETRY_MS));
    }

    for (;;) {
        // Trigger one full sample/update/publish cycle.
        GPS_MODULE.process();

        // Periodic scheduling cadence for GPS processing.
        vTaskDelay(pdMS_TO_TICKS(gpsLoopMs()));
    }
}
