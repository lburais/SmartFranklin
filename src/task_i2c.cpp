#include "tasks.h"

#include "axp.h"
#include "gaz.h"
#include "gps.h"
#include "ina.h"
#include "interfaces.h"
#include "level.h"
#include "log.h"
#include "rtc.h"
#include "tank.h"

namespace {

struct I2cSlot {
    const char* name;
    sf_interfaces::InterfaceSensor sensor;
    bool enabled;

    bool (*isInitialized)();
    bool (*init)();
    void (*process)();

    uint32_t nextInitAttemptMs;
    uint32_t nextProcessMs;
};

bool isDue(const uint32_t nowMs, const uint32_t targetMs)
{
    return static_cast<int32_t>(nowMs - targetMs) >= 0;
}

uint32_t recurrenceOrDefaultMs(const sf_interfaces::InterfaceSensor sensor)
{
    const uint32_t recurrenceMs = sf_interfaces::getRecurrenceMs(sensor);
    return (recurrenceMs > 0U) ? recurrenceMs : 1000U;
}

bool gazIsInitialized() { return GAZ_TASK.isInitialized(); }
bool gazInit() { return GAZ_TASK.init(); }
void gazProcess() { (void)GAZ_TASK.process(); }

bool tankIsInitialized() { return TANK_TASK.isInitialized(); }
bool tankInit() { return TANK_TASK.init(); }
void tankProcess() { (void)TANK_TASK.process(); }

bool levelIsInitialized() { return LEVEL_TASK.isInitialized(); }
bool levelInit() { return LEVEL_TASK.init(); }
void levelProcess() { LEVEL_TASK.process(); }

bool rtcIsInitialized() { return RTC_TASK.isInitialized(); }
bool rtcInit() { return RTC_TASK.init(); }
void rtcProcess() { RTC_TASK.process(); }

bool gpsIsInitialized() { return GPS_TASK.isInitialized(); }
bool gpsInit() { return GPS_TASK.init(); }
void gpsProcess() { (void)GPS_TASK.process(); }

bool axpIsInitialized() { return AXP_TASK.isInitialized(); }
bool axpInit() { return AXP_TASK.init(); }
void axpProcess() { AXP_TASK.process(); }

bool inaIsInitialized() { return INA_TASK.isInitialized(); }
bool inaInit() { return INA_TASK.init(); }
void inaProcess() { INA_TASK.process(); }

}  // namespace

void taskI2c(void* pv)
{
    (void)pv;
    SF_LOGI("[I2C] Unified task started");

    I2cSlot slots[] = {
        {"GAZ",   sf_interfaces::InterfaceSensor::Gaz,
#if defined(ENABLE_GAZ)
            true,
#else
            false,
#endif
            gazIsInitialized, gazInit, gazProcess, 0U, 0U},
        {"TANK",  sf_interfaces::InterfaceSensor::Tank,
#if defined(ENABLE_TANK)
            true,
#else
            false,
#endif
            tankIsInitialized, tankInit, tankProcess, 0U, 0U},
        {"LEVEL", sf_interfaces::InterfaceSensor::Imu,
#if defined(ENABLE_LEVEL)
            true,
#else
            false,
#endif
            levelIsInitialized, levelInit, levelProcess, 0U, 0U},
        {"RTC",   sf_interfaces::InterfaceSensor::Rtc,
#if defined(ENABLE_RTC)
            true,
#else
            false,
#endif
            rtcIsInitialized, rtcInit, rtcProcess, 0U, 0U},
        {"GPS",   sf_interfaces::InterfaceSensor::Gps,
#if defined(ENABLE_GPS)
            true,
#else
            false,
#endif
            gpsIsInitialized, gpsInit, gpsProcess, 0U, 0U},
        {"AXP",   sf_interfaces::InterfaceSensor::Axp,
    #if defined(ENABLE_AXP)
            true,
    #else
            false,
    #endif
            axpIsInitialized, axpInit, axpProcess, 0U, 0U},
        {"INA",   sf_interfaces::InterfaceSensor::Ina1,
    #if defined(ENABLE_INA)
            true,
    #else
            false,
    #endif
            inaIsInitialized, inaInit, inaProcess, 0U, 0U},
    };

    sf_interfaces::configure_all_sensors();

    for (;;) {
        const uint32_t nowMs = millis();
        uint32_t nextWakeMs = nowMs + 1000U;

        for (auto& slot : slots) {
            if (!slot.enabled) {
                continue;
            }

            if (!slot.isInitialized()) {
                if (isDue(nowMs, slot.nextInitAttemptMs)) {
                    if (!slot.init()) {
                        SF_LOGW("[I2C][%s] Init failed", slot.name);
                        slot.nextInitAttemptMs = nowMs + sf_interfaces::kInterfaceInitRetryMs;
                    } else {
                        slot.nextProcessMs = nowMs;
                    }
                }

                if (slot.nextInitAttemptMs < nextWakeMs) {
                    nextWakeMs = slot.nextInitAttemptMs;
                }
                continue;
            }

            if (isDue(nowMs, slot.nextProcessMs)) {
                slot.process();
                slot.nextProcessMs = nowMs + recurrenceOrDefaultMs(slot.sensor);
            }

            if (slot.nextProcessMs < nextWakeMs) {
                nextWakeMs = slot.nextProcessMs;
            }
        }

        const uint32_t nowAfterWorkMs = millis();
        const uint32_t sleepMsRaw = (nextWakeMs > nowAfterWorkMs) ? (nextWakeMs - nowAfterWorkMs) : 20U;
        const uint32_t sleepMs = (sleepMsRaw < 20U) ? 20U : sleepMsRaw;
        vTaskDelay(pdMS_TO_TICKS(sleepMs));
    }
}
