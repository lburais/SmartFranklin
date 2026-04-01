#include <Arduino.h>
#include <M5Unified.h>

#include "config_store.h"
#include "i2c.h"
#include "tank.h"

void taskTank(void* pv)
{
    (void)pv;
    M5_LOGI("[TANK] Task started");

    i2cBeginPortA();

    bool initialized = false;
    uint32_t nextInitAttemptMs = 0;

    constexpr uint8_t deviceAddress = 0x57;

    auto isRetryDue = [](uint32_t nowMs, uint32_t nextAttemptMs) {
        return static_cast<int32_t>(nowMs - nextAttemptMs) >= 0;
    };

    auto scheduleRetry = [](uint32_t& nextAttemptMs, uint32_t nowMs) {
        nextAttemptMs = nowMs + kI2cInitRetryMs;
    };

    for (;;) {
        const uint32_t nowMs = millis();

#ifndef DISABLE_TANK
        if (!initialized && isRetryDue(nowMs, nextInitAttemptMs)) {
            const String configuredPort = i2cConfiguredPortForSensor(sf_ports::PortSensor::Tank, "TANK");
            if (!i2cBeginConfiguredPort(configuredPort, "TANK") ||
                !i2cDeviceExistsOnConfiguredPort(deviceAddress, configuredPort, "TANK")) {
                initialized = false;
                scheduleRetry(nextInitAttemptMs, nowMs);
            } else {
                i2cPublishConfiguration("tank", configuredPort, deviceAddress);
                initialized = TANK_MODULE.init(configuredPort, deviceAddress);
                if (!initialized) {
                    scheduleRetry(nextInitAttemptMs, nowMs);
                }
            }

            if (!initialized) {
                M5_LOGW("[TANK] Init failed");
            }
        }

        if (initialized) {
            const String configuredPort = i2cConfiguredPortForSensor(sf_ports::PortSensor::Tank, "TANK");
            i2cBeginConfiguredPort(configuredPort, "TANK");
            TANK_MODULE.process();
        }
#endif

        const int loopMs = (CONFIG.task_i2c_loop_ms > 0) ? CONFIG.task_i2c_loop_ms : 1000;
        vTaskDelay(pdMS_TO_TICKS(loopMs));
    }
}
