// SPDX-License-Identifier: MIT

#include "i2c.h"

#include <M5Unified.h>
#include <Wire.h>

#include "pahub_channels.h"

namespace sf_i2c {

namespace {
constexpr uint8_t PAHUB_CHANNEL_COUNT = 8;
}

I2C::I2C(const uint32_t clockHz)
    : m_clockHz(clockHz)
{
}

void I2C::beginPortA(int8_t& sda, int8_t& scl) const
{
    sda = static_cast<int8_t>(M5.getPin(m5::pin_name_t::port_a_sda));
    scl = static_cast<int8_t>(M5.getPin(m5::pin_name_t::port_a_scl));

    Wire.end();
    Wire.begin(sda, scl, m_clockHz);
    Wire.setPins(sda, scl);
    M5.Ex_I2C.begin();
}

bool I2C::wireDeviceExists(const uint8_t address) const
{
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

bool I2C::exDeviceExists(const uint8_t address) const
{
    return M5.Ex_I2C.scanID(address, m_clockHz);
}

bool I2C::wireSelectPaHubChannel(const uint8_t channel) const
{
    Wire.beginTransmission(PAHUB_ADDRESS);
    Wire.write(static_cast<uint8_t>(1U << channel));
    return Wire.endTransmission() == 0;
}

void I2C::wireDisablePaHubChannels() const
{
    Wire.beginTransmission(PAHUB_ADDRESS);
    Wire.write(static_cast<uint8_t>(0x00));
    Wire.endTransmission();
}

bool I2C::exSelectPaHubChannel(const uint8_t channel) const
{
    if (!M5.Ex_I2C.start(PAHUB_ADDRESS, false, m_clockHz)) {
        return false;
    }

    const bool writeOk = M5.Ex_I2C.write(static_cast<uint8_t>(1U << channel));
    const bool stopOk = M5.Ex_I2C.stop();
    return writeOk && stopOk;
}

void I2C::exDisablePaHubChannels() const
{
    if (!M5.Ex_I2C.start(PAHUB_ADDRESS, false, m_clockHz)) {
        return;
    }

    M5.Ex_I2C.write(static_cast<uint8_t>(0x00));
    M5.Ex_I2C.stop();
}

bool I2C::detectRoute(const uint8_t deviceAddress, Route& route) const
{
    route.mode = RouteMode::Unset;
    route.paHubChannel = -1;

    if (exDeviceExists(deviceAddress)) {
        route.mode = RouteMode::Internal;
        return true;
    }

    if (wireDeviceExists(deviceAddress)) {
        route.mode = RouteMode::Wire;
        return true;
    }

    if (wireDeviceExists(PAHUB_ADDRESS)) {
        for (uint8_t channel = 0; channel < PAHUB_CHANNEL_COUNT; ++channel) {
            if (!wireSelectPaHubChannel(channel)) {
                continue;
            }

            if (wireDeviceExists(deviceAddress)) {
                wireDisablePaHubChannels();
                route.mode = RouteMode::WirePaHub;
                route.paHubChannel = static_cast<int8_t>(channel);
                return true;
            }
        }
        wireDisablePaHubChannels();
    }

    if (exDeviceExists(PAHUB_ADDRESS)) {
        for (uint8_t channel = 0; channel < PAHUB_CHANNEL_COUNT; ++channel) {
            if (!exSelectPaHubChannel(channel)) {
                continue;
            }

            if (exDeviceExists(deviceAddress)) {
                exDisablePaHubChannels();
                route.mode = RouteMode::InternalPaHub;
                route.paHubChannel = static_cast<int8_t>(channel);
                return true;
            }
        }
        exDisablePaHubChannels();
    }

    return false;
}

bool I2C::selectPaHubChannel(const RouteMode mode, const uint8_t channel) const
{
    if (mode == RouteMode::InternalPaHub) {
        return exSelectPaHubChannel(channel);
    }

    return wireSelectPaHubChannel(channel);
}

void I2C::disablePaHubChannels(const RouteMode mode) const
{
    if (mode == RouteMode::InternalPaHub) {
        exDisablePaHubChannels();
        return;
    }

    wireDisablePaHubChannels();
}

bool isInternalRoute(const RouteMode mode)
{
    return mode == RouteMode::Internal || mode == RouteMode::InternalPaHub;
}

bool isPaHubRoute(const RouteMode mode)
{
    return mode == RouteMode::WirePaHub || mode == RouteMode::InternalPaHub;
}

const char* routeModeToString(const RouteMode mode)
{
    switch (mode) {
    case RouteMode::Internal:
        return "internal";
    case RouteMode::InternalPaHub:
        return "internal_pahub";
    case RouteMode::Wire:
        return "wire";
    case RouteMode::WirePaHub:
        return "wire_pahub";
    case RouteMode::Unset:
    default:
        return "unset";
    }
}

}  // namespace sf_i2c
