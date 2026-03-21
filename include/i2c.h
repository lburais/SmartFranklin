// SPDX-License-Identifier: MIT
#pragma once

#include <Arduino.h>

namespace sf_i2c {

enum class RouteMode : uint8_t {
    Unset = 0,
    Internal,
    InternalPaHub,
    Wire,
    WirePaHub,
};

enum class LevelType : uint8_t {
    None = 0,
    InternalM5,
    ExternalMpuUnit,
    ExternalAdxl345,
};

struct Route {
    RouteMode mode = RouteMode::Unset;
    int8_t paHubChannel = -1;
};

struct Device {
    Route route;
    uint8_t address;
    const char* tag;
    const char* deviceName;
    LevelType levelType = LevelType::None;
};

class I2C {

    public:
    explicit I2C(uint32_t clockHz = 100000U);

    void beginPortA() const;
    bool detectRoute(uint8_t deviceAddress, Route& route) const;

    bool selectPaHubChannel(RouteMode mode, uint8_t channel) const;
    void disablePaHubChannels(RouteMode mode) const;

    void publishConfiguration(const Device& device) const;

private:
    bool wireDeviceExists(uint8_t address) const;
    bool exDeviceExists(uint8_t address) const;

    bool wireSelectPaHubChannel(uint8_t channel) const;
    void wireDisablePaHubChannels() const;

    bool exSelectPaHubChannel(uint8_t channel) const;
    void exDisablePaHubChannels() const;

    uint32_t m_clockHz;
};

bool isInternalRoute(RouteMode mode);
bool isPaHubRoute(RouteMode mode);
const char* routeModeToString(RouteMode mode);
const char* levelTypeToString(LevelType type);

}  // namespace sf_i2c
