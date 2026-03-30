#pragma once

#include <Arduino.h>

#include "i2c.h"
#include "level.h"

namespace sf_task_i2c {

constexpr uint32_t kInitRetryMs = 10000UL;

extern sf_i2c::I2C g_i2c;
extern SemaphoreHandle_t g_i2cMutex;

void initializeI2cShared();

bool selectPaHubIfNeeded(const sf_i2c::I2C& i2c, const sf_i2c::Device& device, const char* label);
void disablePaHubIfNeeded(const sf_i2c::I2C& i2c, const sf_i2c::Device& device);

Level::Source levelSourceFromLevelType(sf_i2c::LevelType levelType);

}  // namespace sf_task_i2c
