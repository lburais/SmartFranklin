#include <Arduino.h>
#include <M5Unified.h>

#include "config_store.h"
#include "gps.h"
#include "hmi.h"
#include "i2c.h"

void taskGps(void* pv)
{
    (void)pv;
    M5_LOGI("[GPS] Task started");

    i2cBeginPortA();

    bool initialized = false;
    uint32_t nextInitAttemptMs = 0;

    constexpr uint8_t deviceAddress = 0x66;

    auto isRetryDue = [](uint32_t nowMs, uint32_t nextAttemptMs) {
        return static_cast<int32_t>(nowMs - nextAttemptMs) >= 0;
    };

    auto scheduleRetry = [](uint32_t& nextAttemptMs, uint32_t nowMs) {
        nextAttemptMs = nowMs + kI2cInitRetryMs;
    };

    for (;;) {
        const uint32_t nowMs = millis();

#ifndef DISABLE_GPS
        if (!initialized && isRetryDue(nowMs, nextInitAttemptMs)) {
            const String configuredPort = i2cConfiguredPortForSensor(sf_ports::PortSensor::Gps, "GPS");
            if (!i2cBeginConfiguredPort(configuredPort, "GPS") ||
                !i2cDeviceExistsOnConfiguredPort(deviceAddress, configuredPort, "GPS")) {
                initialized = false;
                hmiSetPortLedStatus(configuredPort, false, false);
                scheduleRetry(nextInitAttemptMs, nowMs);
            } else {
                i2cPublishConfiguration("gps", configuredPort, deviceAddress);
                initialized = GPS_MODULE.init(GPS::Source::ExternalDfrobotGravity,
                                              configuredPort,
                                              deviceAddress);
                hmiSetPortLedStatus(configuredPort, initialized, false);
                if (!initialized) {
                    scheduleRetry(nextInitAttemptMs, nowMs);
                }
            }

            if (!initialized) {
                M5_LOGW("[GPS] Init failed");
            }
        }

        if (initialized) {
            const String configuredPort = i2cConfiguredPortForSensor(sf_ports::PortSensor::Gps, "GPS");
            i2cBeginConfiguredPort(configuredPort, "GPS");
            GPS_MODULE.process();
        }
#endif

        const int loopMs = (CONFIG.task_i2c_loop_ms > 0) ? CONFIG.task_i2c_loop_ms : 1000;
        vTaskDelay(pdMS_TO_TICKS(loopMs));
    }
}
