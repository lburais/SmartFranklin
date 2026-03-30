#include <Arduino.h>
#include <M5Unified.h>

#include "config_store.h"
#include "tank.h"
#include "task_i2c_shared.h"

void taskTank(void* pv)
{
    (void)pv;
    M5_LOGI("[TANK] Task started");

    sf_task_i2c::initializeI2cShared();

    bool initialized = false;
    uint32_t nextInitAttemptMs = 0;

    sf_i2c::Device device{};
    device.route = sf_i2c::Route{};
    device.address = 0x57;
    device.tag = "tank";
    device.deviceName = "M5Stack Unit Ultrasonic I2C (RCWL-9600)";

    auto isRetryDue = [](uint32_t nowMs, uint32_t nextAttemptMs) {
        return static_cast<int32_t>(nowMs - nextAttemptMs) >= 0;
    };

    auto scheduleRetry = [](uint32_t& nextAttemptMs, uint32_t nowMs) {
        nextAttemptMs = nowMs + sf_task_i2c::kInitRetryMs;
    };

    for (;;) {
        const uint32_t nowMs = millis();

#ifndef DISABLE_TANK
        if (!initialized && isRetryDue(nowMs, nextInitAttemptMs)) {
            if (xSemaphoreTake(sf_task_i2c::g_i2cMutex, pdMS_TO_TICKS(100))) {
                if (!sf_task_i2c::g_i2c.detectRoute(device.address, device.route)) {
                    M5_LOGW("[TANK] Route detection failed");
                    sf_task_i2c::g_i2c.publishConfiguration(device);
                    initialized = false;
                    scheduleRetry(nextInitAttemptMs, nowMs);
                } else {
                    sf_task_i2c::g_i2c.publishConfiguration(device);
                    bool channelSelected = sf_task_i2c::selectPaHubIfNeeded(sf_task_i2c::g_i2c, device, "TANK");
                    if (channelSelected) {
                        initialized = TANK_MODULE.init(sf_i2c::isInternalRoute(device.route.mode), device.address);
                        sf_task_i2c::disablePaHubIfNeeded(sf_task_i2c::g_i2c, device);
                        if (!initialized) {
                            scheduleRetry(nextInitAttemptMs, nowMs);
                        }
                    } else {
                        initialized = false;
                        scheduleRetry(nextInitAttemptMs, nowMs);
                    }
                }
                xSemaphoreGive(sf_task_i2c::g_i2cMutex);

                if (!initialized) {
                    M5_LOGW("[TANK] Init failed");
                }
            }
        }

        if (initialized && xSemaphoreTake(sf_task_i2c::g_i2cMutex, pdMS_TO_TICKS(100))) {
            if (sf_task_i2c::selectPaHubIfNeeded(sf_task_i2c::g_i2c, device, "TANK")) {
                TANK_MODULE.process();
                sf_task_i2c::disablePaHubIfNeeded(sf_task_i2c::g_i2c, device);
            }
            xSemaphoreGive(sf_task_i2c::g_i2cMutex);
        }
#endif

        const int loopMs = (CONFIG.task_i2c_loop_ms > 0) ? CONFIG.task_i2c_loop_ms : 1000;
        vTaskDelay(pdMS_TO_TICKS(loopMs));
    }
}
