#include "task_i2c_shared.h"

#include <M5Unified.h>

namespace sf_task_i2c {

sf_i2c::I2C g_i2c{};
SemaphoreHandle_t g_i2cMutex = nullptr;

namespace {
portMUX_TYPE g_i2cInitMux = portMUX_INITIALIZER_UNLOCKED;
bool g_i2cStarted = false;
}  // namespace

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

bool selectPaHubIfNeeded(const sf_i2c::I2C& i2c, const sf_i2c::Device& device, const char* label)
{
    if (!sf_i2c::isPaHubRoute(device.route.mode)) {
        return true;
    }

    if (device.route.paHubChannel < 0) {
        M5_LOGW("[%s] invalid PAHub channel %d", label, device.route.paHubChannel);
        return false;
    }

    if (!i2c.selectPaHubChannel(device.route.mode, static_cast<uint8_t>(device.route.paHubChannel))) {
        M5_LOGW("[%s] failed to select PAHub channel %d", label, device.route.paHubChannel);
        return false;
    }

    return true;
}

void disablePaHubIfNeeded(const sf_i2c::I2C& i2c, const sf_i2c::Device& device)
{
    if (sf_i2c::isPaHubRoute(device.route.mode)) {
        i2c.disablePaHubChannels(device.route.mode);
    }
}

void initializeI2cShared()
{
    taskENTER_CRITICAL(&g_i2cInitMux);
    if (g_i2cMutex == nullptr) {
        g_i2cMutex = xSemaphoreCreateMutex();
    }

    if (!g_i2cStarted) {
        g_i2c.beginPortA();
        g_i2cStarted = true;
    }
    taskEXIT_CRITICAL(&g_i2cInitMux);
}

}  // namespace sf_task_i2c
