#include <Arduino.h>
#include <M5Unified.h>

#include "config_store.h"
#include "gps.h"
#include "hmi.h"
#include "interfaces.h"

void taskGps(void* pv)
{
    (void)pv;
    M5_LOGI("[GPS] Task started");

    bool initialized = false;
    uint32_t nextInitAttemptMs = 0;

    auto isRetryDue = [](uint32_t nowMs, uint32_t nextAttemptMs) {
        return static_cast<int32_t>(nowMs - nextAttemptMs) >= 0;
    };

    auto scheduleRetry = [](uint32_t& nextAttemptMs, uint32_t nowMs) {
        nextAttemptMs = nowMs + sf_i2c::kI2cInitRetryMs;
    };

    for (;;) {
        const uint32_t nowMs = millis();

        if (!initialized && isRetryDue(nowMs, nextInitAttemptMs)) {
            if (!sf_i2c::i2cBeginConfiguredPort(sf_interfaces::InterfaceSensor::Gps) ||
                !sf_i2c::i2cDeviceExistsOnConfiguredPort(sf_interfaces::InterfaceSensor::Gps, GPS_MODULE.deviceAddress)) {
                initialized = false;
                hmiSetPortLedStatus(sf_interfaces::InterfaceSensor::Gps, false, false);
                scheduleRetry(nextInitAttemptMs, nowMs);
            } else {
                sf_i2c::i2cPublishConfiguration(sf_interfaces::InterfaceSensor::Gps, GPS_MODULE.deviceAddress);
                initialized = GPS_MODULE.init();
                hmiSetPortLedStatus(sf_interfaces::InterfaceSensor::Gps, initialized, false);
                if (!initialized) {
                    scheduleRetry(nextInitAttemptMs, nowMs);
                }
            }

            if (!initialized) {
                M5_LOGW("[GPS] Init failed");
            }
        }

        if (initialized) {
            sf_i2c::i2cBeginConfiguredPort(sf_interfaces::InterfaceSensor::Gps);
            GPS_MODULE.process();
        }

        const int loopMs = (CONFIG.task_i2c_loop_ms > 0) ? CONFIG.task_i2c_loop_ms : 1000;
        vTaskDelay(pdMS_TO_TICKS(loopMs));
    }
}
