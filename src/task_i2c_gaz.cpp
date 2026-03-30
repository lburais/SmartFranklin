#include <Arduino.h>
#include <M5Unified.h>

#include "config_store.h"
#include "gaz.h"
#include "task_i2c_shared.h"

void taskGaz(void* pv)
{
    (void)pv;
    M5_LOGI("[GAZ] Task started");

    sf_task_i2c::initializeI2cShared();

    bool initialized = false;
    uint32_t nextInitAttemptMs = 0;

    sf_i2c::Device device{};
    device.route = sf_i2c::Route{};
    device.address = 0x26;
    device.tag = "gaz";
    device.deviceName = "M5Stack Weight I2C Unit";

    auto isRetryDue = [](uint32_t nowMs, uint32_t nextAttemptMs) {
        return static_cast<int32_t>(nowMs - nextAttemptMs) >= 0;
    };

    auto scheduleRetry = [](uint32_t& nextAttemptMs, uint32_t nowMs) {
        nextAttemptMs = nowMs + sf_task_i2c::kInitRetryMs;
    };

    for (;;) {
        const uint32_t nowMs = millis();

#ifndef DISABLE_GAZ
        if (!initialized && isRetryDue(nowMs, nextInitAttemptMs)) {
            if (xSemaphoreTake(sf_task_i2c::g_i2cMutex, pdMS_TO_TICKS(100))) {
                if (!sf_task_i2c::g_i2c.detectRoute(device.address, device.route)) {
                    M5_LOGW("[GAZ] Route detection failed");
                    sf_task_i2c::g_i2c.publishConfiguration(device);
                    initialized = false;
                    scheduleRetry(nextInitAttemptMs, nowMs);
                } else {
                    sf_task_i2c::g_i2c.publishConfiguration(device);
                    initialized = GAZ_MODULE.init(sf_i2c::isInternalRoute(device.route.mode),
                                                  device.address,
                                                  device.route.mode,
                                                  device.route.paHubChannel);
                    if (!initialized) {
                        scheduleRetry(nextInitAttemptMs, nowMs);
                    }
                }
                xSemaphoreGive(sf_task_i2c::g_i2cMutex);

                if (!initialized) {
                    M5_LOGW("[GAZ] Init failed");
                }
            }
        }

        if (initialized) {
            GAZ_MODULE.process();
        }
#endif

        const int loopMs = (CONFIG.task_i2c_loop_ms > 0) ? CONFIG.task_i2c_loop_ms : 1000;
        vTaskDelay(pdMS_TO_TICKS(loopMs));
    }
}
