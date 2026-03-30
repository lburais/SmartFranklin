#include <Arduino.h>
#include <M5Unified.h>

#include "config_store.h"
#include "level.h"
#include "task_i2c_shared.h"

void taskLevel(void* pv)
{
    (void)pv;
    M5_LOGI("[LEVEL] Task started");

    sf_task_i2c::initializeI2cShared();

    bool initialized = false;
    uint32_t nextInitAttemptMs = 0;

    sf_i2c::Device internalLevelDevice{};
    internalLevelDevice.route = sf_i2c::Route{};
    internalLevelDevice.route.mode = sf_i2c::RouteMode::Internal;
    internalLevelDevice.route.paHubChannel = -1;
    internalLevelDevice.address = 0x00;
    internalLevelDevice.tag = "level";
    internalLevelDevice.deviceName = "M5 internal Level sensor";
    internalLevelDevice.levelType = sf_i2c::LevelType::InternalM5;

    sf_i2c::Device levelMpuDevice{};
    levelMpuDevice.route = sf_i2c::Route{};
    levelMpuDevice.address = 0x68;
    levelMpuDevice.tag = "level";
    levelMpuDevice.deviceName = "M5Stack External Level Unit (MPU-compatible)";
    levelMpuDevice.levelType = sf_i2c::LevelType::ExternalMpuUnit;

    sf_i2c::Device levelAdxlDevice{};
    levelAdxlDevice.route = sf_i2c::Route{};
    levelAdxlDevice.address = 0x53;
    levelAdxlDevice.tag = "level";
    levelAdxlDevice.deviceName = "SeeedStudio ADXL345 Level Sensor";
    levelAdxlDevice.levelType = sf_i2c::LevelType::ExternalAdxl345;

    sf_i2c::Device activeLevelDevice{};
    activeLevelDevice.route = sf_i2c::Route{};
    activeLevelDevice.address = 0x00;
    activeLevelDevice.tag = "level";
    activeLevelDevice.deviceName = "Level";
    activeLevelDevice.levelType = sf_i2c::LevelType::None;

    auto isRetryDue = [](uint32_t nowMs, uint32_t nextAttemptMs) {
        return static_cast<int32_t>(nowMs - nextAttemptMs) >= 0;
    };

    auto scheduleRetry = [](uint32_t& nextAttemptMs, uint32_t nowMs) {
        nextAttemptMs = nowMs + sf_task_i2c::kInitRetryMs;
    };

    for (;;) {
        const uint32_t nowMs = millis();

#ifndef DISABLE_LEVEL
        if (!initialized && isRetryDue(nowMs, nextInitAttemptMs)) {
            if (xSemaphoreTake(sf_task_i2c::g_i2cMutex, pdMS_TO_TICKS(100))) {
                if (LEVEL_MODULE.init(sf_task_i2c::levelSourceFromLevelType(internalLevelDevice.levelType), true, internalLevelDevice.address)) {
                    initialized = true;
                    activeLevelDevice = internalLevelDevice;
                } else {
                    if (sf_task_i2c::g_i2c.detectRoute(levelMpuDevice.address, levelMpuDevice.route)) {
                        bool channelSelected = sf_task_i2c::selectPaHubIfNeeded(sf_task_i2c::g_i2c, levelMpuDevice, "LEVEL");
                        if (channelSelected) {
                            initialized = LEVEL_MODULE.init(sf_task_i2c::levelSourceFromLevelType(levelMpuDevice.levelType),
                                                            sf_i2c::isInternalRoute(levelMpuDevice.route.mode),
                                                            levelMpuDevice.address);
                            sf_task_i2c::disablePaHubIfNeeded(sf_task_i2c::g_i2c, levelMpuDevice);
                            if (initialized) {
                                activeLevelDevice = levelMpuDevice;
                            }
                        }
                    }

                    if (!initialized && sf_task_i2c::g_i2c.detectRoute(levelAdxlDevice.address, levelAdxlDevice.route)) {
                        bool channelSelected = sf_task_i2c::selectPaHubIfNeeded(sf_task_i2c::g_i2c, levelAdxlDevice, "LEVEL");
                        if (channelSelected) {
                            initialized = LEVEL_MODULE.init(sf_task_i2c::levelSourceFromLevelType(levelAdxlDevice.levelType),
                                                            sf_i2c::isInternalRoute(levelAdxlDevice.route.mode),
                                                            levelAdxlDevice.address);
                            sf_task_i2c::disablePaHubIfNeeded(sf_task_i2c::g_i2c, levelAdxlDevice);
                            if (initialized) {
                                activeLevelDevice = levelAdxlDevice;
                            }
                        }
                    }
                }

                if (!initialized) {
                    M5_LOGW("[LEVEL] Init failed");
                    scheduleRetry(nextInitAttemptMs, nowMs);
                } else {
                    sf_task_i2c::g_i2c.publishConfiguration(activeLevelDevice);
                }
                xSemaphoreGive(sf_task_i2c::g_i2cMutex);
            }
        }

        if (initialized && activeLevelDevice.levelType == sf_i2c::LevelType::InternalM5) {
            LEVEL_MODULE.process();
        } else if (initialized && xSemaphoreTake(sf_task_i2c::g_i2cMutex, pdMS_TO_TICKS(100))) {
            if (sf_task_i2c::selectPaHubIfNeeded(sf_task_i2c::g_i2c, activeLevelDevice, "LEVEL")) {
                LEVEL_MODULE.process();
                sf_task_i2c::disablePaHubIfNeeded(sf_task_i2c::g_i2c, activeLevelDevice);
            }
            xSemaphoreGive(sf_task_i2c::g_i2cMutex);
        }
#endif

        const int loopMs = (CONFIG.task_i2c_loop_ms > 0) ? CONFIG.task_i2c_loop_ms : 1000;
        vTaskDelay(pdMS_TO_TICKS(loopMs));
    }
}
