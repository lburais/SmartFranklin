#include <Arduino.h>
#include <M5Unified.h>

#include "config_store.h"
#include "i2c.h"
#include "level.h"

namespace {

Level::Source levelSourceFromLevelType(const sf_i2c::LevelType levelType)
{
    switch (levelType) {
    case sf_i2c::LevelType::InternalM5:
        return Level::Source::InternalM5;
    case sf_i2c::LevelType::ExternalMpuUnit:
        return Level::Source::ExternalMpuUnit;
    case sf_i2c::LevelType::ExternalAdxl345:
        return Level::Source::ExternalAdxl345;
    case sf_i2c::LevelType::None:
    default:
        return Level::Source::None;
    }
}

}  // namespace

void taskLevel(void* pv)
{
    (void)pv;
    M5_LOGI("[LEVEL] Task started");

    sf_i2c::I2C i2c{};
    i2c.beginPortA();

    bool initialized = false;
    uint32_t nextInitAttemptMs = 0;

    sf_i2c::Device internalLevelDevice{};
    internalLevelDevice.route = sf_i2c::Route{};
    internalLevelDevice.route.mode = sf_i2c::RouteMode::Internal;
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
        nextAttemptMs = nowMs + sf_i2c::kInitRetryMs;
    };

    for (;;) {
        const uint32_t nowMs = millis();

#ifndef DISABLE_LEVEL
        if (!initialized && isRetryDue(nowMs, nextInitAttemptMs)) {
            if (!sf_i2c::resolveRouteFromConfiguredPort(CONFIG.level_i2c_port, levelMpuDevice.route, "LEVEL")) {
                initialized = false;
                scheduleRetry(nextInitAttemptMs, nowMs);
            } else if (levelMpuDevice.route.mode == sf_i2c::RouteMode::Internal &&
                       LEVEL_MODULE.init(levelSourceFromLevelType(internalLevelDevice.levelType),
                                         true,
                                         internalLevelDevice.address)) {
                initialized = true;
                activeLevelDevice = internalLevelDevice;
            } else {
                levelAdxlDevice.route = levelMpuDevice.route;

                if (i2c.deviceExistsOnRoute(levelMpuDevice.address, levelMpuDevice.route)) {
                    i2c.beginRoute(levelMpuDevice.route);
                    initialized = LEVEL_MODULE.init(levelSourceFromLevelType(levelMpuDevice.levelType),
                                                    sf_i2c::isInternalRoute(levelMpuDevice.route.mode),
                                                    levelMpuDevice.address);
                    if (initialized) {
                        activeLevelDevice = levelMpuDevice;
                    }
                }

                if (!initialized && i2c.deviceExistsOnRoute(levelAdxlDevice.address, levelAdxlDevice.route)) {
                    i2c.beginRoute(levelAdxlDevice.route);
                    initialized = LEVEL_MODULE.init(levelSourceFromLevelType(levelAdxlDevice.levelType),
                                                    sf_i2c::isInternalRoute(levelAdxlDevice.route.mode),
                                                    levelAdxlDevice.address);
                    if (initialized) {
                        activeLevelDevice = levelAdxlDevice;
                    }
                }
            }

            if (!initialized) {
                M5_LOGW("[LEVEL] Init failed");
                scheduleRetry(nextInitAttemptMs, nowMs);
            } else {
                i2c.publishConfiguration(activeLevelDevice);
            }
        }

        if (initialized && activeLevelDevice.levelType == sf_i2c::LevelType::InternalM5) {
            LEVEL_MODULE.process();
        } else if (initialized) {
            i2c.beginRoute(activeLevelDevice.route);
            LEVEL_MODULE.process();
        }
#endif

        const int loopMs = (CONFIG.task_i2c_loop_ms > 0) ? CONFIG.task_i2c_loop_ms : 1000;
        vTaskDelay(pdMS_TO_TICKS(loopMs));
    }
}
