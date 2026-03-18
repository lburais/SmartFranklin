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

struct Route {
    RouteMode mode = RouteMode::Unset;
    int8_t paHubChannel = -1;
};

struct Device {
    Route route;
    int8_t sda;
    int8_t scl; 
    int32_t clock; 
    uint8_t address; 
    const char* deviceName;   
};

class I2C {

    public:
    explicit I2C(uint32_t clockHz = 400000U);

    void beginPortA(int8_t& sda, int8_t& scl) const;
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

}  // namespace sf_i2c
