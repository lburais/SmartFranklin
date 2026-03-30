#include <Arduino.h>
#include <M5Unified.h>

#include "config_store.h"
#include "i2c.h"
#include "tank.h"

void taskTank(void* pv)
{
    (void)pv;
    M5_LOGI("[TANK] Task started");

    sf_i2c::I2C i2c{};
    i2c.beginPortA();

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
        nextAttemptMs = nowMs + sf_i2c::kInitRetryMs;
    };

    for (;;) {
        const uint32_t nowMs = millis();

#ifndef DISABLE_TANK
        if (!initialized && isRetryDue(nowMs, nextInitAttemptMs)) {
            if (!sf_i2c::resolveRouteFromConfiguredPort(CONFIG.tank_i2c_port, device.route, "TANK") ||
                !i2c.deviceExistsOnRoute(device.address, device.route)) {
                initialized = false;
                scheduleRetry(nextInitAttemptMs, nowMs);
            } else {
                i2c.beginRoute(device.route);
                i2c.publishConfiguration(device);
                initialized = TANK_MODULE.init(sf_i2c::isInternalRoute(device.route.mode), device.address);
                if (!initialized) {
                    scheduleRetry(nextInitAttemptMs, nowMs);
                }
            }

            if (!initialized) {
                M5_LOGW("[TANK] Init failed");
            }
        }

        if (initialized) {
            i2c.beginRoute(device.route);
            TANK_MODULE.process();
        }
#endif

        const int loopMs = (CONFIG.task_i2c_loop_ms > 0) ? CONFIG.task_i2c_loop_ms : 1000;
        vTaskDelay(pdMS_TO_TICKS(loopMs));
    }
}
