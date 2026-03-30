#include <Arduino.h>
#include <M5Unified.h>

#include "config_store.h"
#include "i2c.h"
#include "rtc.h"

void taskRtc(void* pv)
{
    (void)pv;
    M5_LOGI("[RTC] Task started");

    sf_i2c::I2C i2c{};
    i2c.beginPortA();

    bool initialized = false;
    uint32_t nextInitAttemptMs = 0;

    sf_i2c::Device internalRtcDevice{};
    internalRtcDevice.route = sf_i2c::Route{};
    internalRtcDevice.route.mode = sf_i2c::RouteMode::Internal;
    internalRtcDevice.address = 0x51;
    internalRtcDevice.tag = "rtc";
    internalRtcDevice.deviceName = "M5 internal RTC";

    sf_i2c::Device externalRtcDevice{};
    externalRtcDevice.route = sf_i2c::Route{};
    externalRtcDevice.address = 0x51;
    externalRtcDevice.tag = "rtc";
    externalRtcDevice.deviceName = "M5Stack RTC Unit or Seeed PCD85063TP";

    sf_i2c::Device activeRtcDevice{};
    activeRtcDevice.route = sf_i2c::Route{};
    activeRtcDevice.address = 0x51;
    activeRtcDevice.tag = "rtc";
    activeRtcDevice.deviceName = "RTC";

    auto isRetryDue = [](uint32_t nowMs, uint32_t nextAttemptMs) {
        return static_cast<int32_t>(nowMs - nextAttemptMs) >= 0;
    };

    auto scheduleRetry = [](uint32_t& nextAttemptMs, uint32_t nowMs) {
        nextAttemptMs = nowMs + sf_i2c::kInitRetryMs;
    };

    for (;;) {
        const uint32_t nowMs = millis();

#ifndef DISABLE_RTC
        if (!initialized && isRetryDue(nowMs, nextInitAttemptMs)) {
            if (!sf_i2c::resolveRouteFromConfiguredPort(CONFIG.rtc_i2c_port, externalRtcDevice.route, "RTC")) {
                initialized = false;
                scheduleRetry(nextInitAttemptMs, nowMs);
            } else if (externalRtcDevice.route.mode == sf_i2c::RouteMode::Internal &&
                       RTC_MODULE.init(RTC::Source::InternalRtc, true, internalRtcDevice.address)) {
                initialized = true;
                activeRtcDevice = internalRtcDevice;
            } else if (!i2c.deviceExistsOnRoute(externalRtcDevice.address, externalRtcDevice.route)) {
                initialized = false;
                scheduleRetry(nextInitAttemptMs, nowMs);
            } else {
                i2c.beginRoute(externalRtcDevice.route);
                const bool isInternalRoute = sf_i2c::isInternalRoute(externalRtcDevice.route.mode);
                initialized = RTC_MODULE.init(RTC::Source::ExternalM5StackRtcUnit,
                                              isInternalRoute,
                                              externalRtcDevice.address);
                if (!initialized) {
                    initialized = RTC_MODULE.init(RTC::Source::ExternalSeeedPcd85063tp,
                                                  isInternalRoute,
                                                  externalRtcDevice.address);
                }
                if (initialized) {
                    activeRtcDevice = externalRtcDevice;
                }
            }

            if (!initialized) {
                M5_LOGW("[RTC] Init failed");
                scheduleRetry(nextInitAttemptMs, nowMs);
            } else {
                i2c.publishConfiguration(activeRtcDevice);
            }
        }

        if (initialized && activeRtcDevice.route.mode == sf_i2c::RouteMode::Internal) {
            RTC_MODULE.process();
        } else if (initialized) {
            i2c.beginRoute(activeRtcDevice.route);
            RTC_MODULE.process();
        }
#endif

        const int loopMs = (CONFIG.task_i2c_loop_ms > 0) ? CONFIG.task_i2c_loop_ms : 1000;
        vTaskDelay(pdMS_TO_TICKS(loopMs));
    }
}
