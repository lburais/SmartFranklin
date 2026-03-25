/**
 * @file i2c.cpp
 * @brief I2C route probing and PAHub channel control implementation.
 *
 * This module detects whether peripherals are reachable over internal I2C,
 * Wire, or PAHub-routed buses and publishes resolved wiring metadata.
 *
 * SPDX-License-Identifier: MIT
 */

#include "i2c.h"

#include <M5Unified.h>
#include <Wire.h>

#include <cstring>

#include "mqtt.h"

#define PAHUB_ADDRESS       0x70

namespace sf_i2c {

namespace {
constexpr uint8_t PAHUB_CHANNEL_COUNT = 8;
}

I2C::I2C(const uint32_t clockHz)
    : m_clockHz(clockHz)
{
}

void I2C::beginPortA() const
{
    int8_t sda = static_cast<int8_t>(M5.getPin(m5::pin_name_t::port_a_sda));
    int8_t scl = static_cast<int8_t>(M5.getPin(m5::pin_name_t::port_a_scl));

    Wire.end();
    Wire.begin(sda, scl, m_clockHz);
    Wire.setPins(sda, scl);
    M5.Ex_I2C.begin();
}

/** Probe one address on Wire. */
bool I2C::wireDeviceExists(const uint8_t address) const
{
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

/** Probe one address on internal Ex_I2C bus. */
bool I2C::exDeviceExists(const uint8_t address) const
{
    return M5.Ex_I2C.scanID(address, m_clockHz);
}

/** Select PAHub channel through Wire bus. */
bool I2C::wireSelectPaHubChannel(const uint8_t channel) const
{
    Wire.beginTransmission(PAHUB_ADDRESS);
    Wire.write(static_cast<uint8_t>(1U << channel));
    return Wire.endTransmission() == 0;
}

/** Disable all PAHub channels through Wire bus. */
void I2C::wireDisablePaHubChannels() const
{
    Wire.beginTransmission(PAHUB_ADDRESS);
    Wire.write(static_cast<uint8_t>(0x00));
    Wire.endTransmission();
}

/** Select PAHub channel through Ex_I2C bus. */
bool I2C::exSelectPaHubChannel(const uint8_t channel) const
{
    if (!M5.Ex_I2C.start(PAHUB_ADDRESS, false, m_clockHz)) {
        return false;
    }

    const bool writeOk = M5.Ex_I2C.write(static_cast<uint8_t>(1U << channel));
    const bool stopOk = M5.Ex_I2C.stop();
    return writeOk && stopOk;
}

/** Disable all PAHub channels through Ex_I2C bus. */
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

void I2C::publishConfiguration(const Device& device) const {
    char pahubChannelBuf[12] = {0};
    char addressBuf[8] = {0};
    const char* tag = device.tag ? device.tag : "unknown";
    char topicBuf[64] = {0};

    snprintf(pahubChannelBuf, sizeof(pahubChannelBuf), "%d", device.route.paHubChannel);
    snprintf(addressBuf, sizeof(addressBuf), "0x%02X", device.address);

    snprintf(topicBuf, sizeof(topicBuf), "smartfranklin/system/i2c/%s/mode", tag);
    sf_mqtt::publish(topicBuf, routeModeToString(device.route.mode), 1, true);
    snprintf(topicBuf, sizeof(topicBuf), "smartfranklin/system/i2c/%s/pahub_channel", tag);
    sf_mqtt::publish(topicBuf, pahubChannelBuf, 1, true);
    snprintf(topicBuf, sizeof(topicBuf), "smartfranklin/system/i2c/%s/address", tag);
    sf_mqtt::publish(topicBuf, addressBuf, 1, true);
    snprintf(topicBuf, sizeof(topicBuf), "smartfranklin/system/i2c/%s/device_name", tag);
    sf_mqtt::publish(topicBuf, device.deviceName, 1, true);
    if (std::strcmp(tag, "level") == 0) {
        snprintf(topicBuf, sizeof(topicBuf), "smartfranklin/system/i2c/%s/level_type", tag);
        sf_mqtt::publish(topicBuf, levelTypeToString(device.levelType), 1, true);
    }
    if (std::strcmp(tag, "rtc") == 0) {
        snprintf(topicBuf, sizeof(topicBuf), "smartfranklin/system/i2c/%s/chip_kind", tag);
        sf_mqtt::publish(topicBuf, chipKindToString(device.chipKind), 1, true);
    }
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

const char* levelTypeToString(const LevelType type)
{
    switch (type) {
    case LevelType::InternalM5:
        return "internal_m5";
    case LevelType::ExternalMpuUnit:
        return "external_mpu_unit";
    case LevelType::ExternalAdxl345:
        return "external_adxl345";
    case LevelType::None:
    default:
        return "none";
    }
}

const char* chipKindToString(const Device::ChipKind kind)
{
    switch (kind) {
    case Device::ChipKind::Bm8563Like:
        return "bm8563_like";
    case Device::ChipKind::Pcd85063Like:
        return "pcd85063_like";
    case Device::ChipKind::Unknown:
    default:
        return "unknown";
    }
}

}  // namespace sf_i2c
