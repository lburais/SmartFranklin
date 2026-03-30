/**
 * @file i2c.cpp
 * @brief I2C route probing and direct bus configuration implementation.
 *
 * This module detects whether peripherals are reachable over internal I2C
 * and Wire buses and publishes resolved wiring metadata.
 *
 * SPDX-License-Identifier: MIT
 */

#include "i2c.h"

#include <M5Unified.h>
#include <Wire.h>

#include <cstring>

#include "mqtt.h"

namespace sf_i2c {

I2C::I2C(const uint32_t clockHz)
    : m_clockHz(clockHz)
{
}

void I2C::beginPortA() const
{
    beginExternalPort(ExternalPort::PortA1);
}

void I2C::beginExternalPort(const ExternalPort port) const
{
    int8_t sda = -1;
    int8_t scl = -1;

    switch (port) {
    case ExternalPort::PortA1:
    case ExternalPort::PortA2:
        sda = static_cast<int8_t>(M5.getPin(m5::pin_name_t::port_a_sda));
        scl = static_cast<int8_t>(M5.getPin(m5::pin_name_t::port_a_scl));
        break;
    case ExternalPort::PortB1:
        sda = static_cast<int8_t>(M5.getPin(m5::pin_name_t::port_b_out));
        scl = static_cast<int8_t>(M5.getPin(m5::pin_name_t::port_b_in));
        break;
    case ExternalPort::PortB2:
        sda = static_cast<int8_t>(M5.getPin(m5::pin_name_t::port_b2_pin2));
        scl = static_cast<int8_t>(M5.getPin(m5::pin_name_t::port_b2_pin1));
        break;
    case ExternalPort::PortC1:
        sda = static_cast<int8_t>(M5.getPin(m5::pin_name_t::port_c_txd));
        scl = static_cast<int8_t>(M5.getPin(m5::pin_name_t::port_c_rxd));
        break;
    case ExternalPort::PortC2:
        sda = static_cast<int8_t>(M5.getPin(m5::pin_name_t::port_c2_pin2));
        scl = static_cast<int8_t>(M5.getPin(m5::pin_name_t::port_c2_pin1));
        break;
    default:
        break;
    }

    if (sda < 0 || scl < 0) {
        return;
    }

    Wire.end();
    Wire.begin(sda, scl, m_clockHz);
    Wire.setPins(sda, scl);
    M5.Ex_I2C.begin();
}

void I2C::beginRoute(const Route& route) const
{
    if (route.mode == RouteMode::Wire) {
        beginExternalPort(route.externalPort);
        return;
    }

    if (route.mode == RouteMode::Internal) {
        M5.Ex_I2C.begin();
    }
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

bool I2C::detectRoute(const uint8_t deviceAddress, Route& route) const
{
    route.mode = RouteMode::Unset;

    if (exDeviceExists(deviceAddress)) {
        route.mode = RouteMode::Internal;
        return true;
    }

    if (wireDeviceExists(deviceAddress)) {
        route.mode = RouteMode::Wire;
        return true;
    }

    return false;
}

bool I2C::deviceExistsOnRoute(const uint8_t deviceAddress, const Route& route) const
{
    if (route.mode == RouteMode::Internal) {
        return exDeviceExists(deviceAddress);
    }
    if (route.mode == RouteMode::Wire) {
        beginExternalPort(route.externalPort);
        return wireDeviceExists(deviceAddress);
    }
    return false;
}

void I2C::publishConfiguration(const Device& device) const {
    char addressBuf[8] = {0};
    char portBuf[16] = {0};
    const char* tag = device.tag ? device.tag : "unknown";
    char topicBuf[64] = {0};

    snprintf(addressBuf, sizeof(addressBuf), "0x%02X", device.address);
    snprintf(portBuf, sizeof(portBuf), "%s", externalPortToString(device.route.externalPort));

    snprintf(topicBuf, sizeof(topicBuf), "smartfranklin/system/i2c/%s/mode", tag);
    sf_mqtt::publish(topicBuf, routeModeToString(device.route.mode), 1, true);
    snprintf(topicBuf, sizeof(topicBuf), "smartfranklin/system/i2c/%s/address", tag);
    sf_mqtt::publish(topicBuf, addressBuf, 1, true);
    snprintf(topicBuf, sizeof(topicBuf), "smartfranklin/system/i2c/%s/port", tag);
    sf_mqtt::publish(topicBuf, portBuf, 1, true);
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
    return mode == RouteMode::Internal;
}

bool resolveRouteFromConfiguredPort(const String& configuredPort, Route& route, const char* label)
{
    String port = configuredPort;
    port.trim();
    port.toUpperCase();

    if (port.isEmpty() || port == "WIRE" || port == "PORTA") {
        route.mode = RouteMode::Wire;
        route.externalPort = ExternalPort::PortA1;
        return true;
    }

    if (port == "INTERNAL" || port == "EX") {
        route.mode = RouteMode::Internal;
        route.externalPort = ExternalPort::PortA1;
        return true;
    }

    if (port == "A1") {
        route.mode = RouteMode::Wire;
        route.externalPort = ExternalPort::PortA1;
        return true;
    }
    if (port == "A2") {
        route.mode = RouteMode::Wire;
        route.externalPort = ExternalPort::PortA2;
        return true;
    }
    if (port == "B1" || port == "PORTB") {
        route.mode = RouteMode::Wire;
        route.externalPort = ExternalPort::PortB1;
        return true;
    }
    if (port == "B2") {
        route.mode = RouteMode::Wire;
        route.externalPort = ExternalPort::PortB2;
        return true;
    }
    if (port == "C1") {
        route.mode = RouteMode::Wire;
        route.externalPort = ExternalPort::PortC1;
        return true;
    }
    if (port == "C2") {
        route.mode = RouteMode::Wire;
        route.externalPort = ExternalPort::PortC2;
        return true;
    }

    M5_LOGW("[%s] invalid configured I2C port '%s' (valid: INTERNAL, EX, A1, A2, B1, B2, C1, C2)",
            label,
            configuredPort.c_str());
    return false;
}

const char* routeModeToString(const RouteMode mode)
{
    switch (mode) {
    case RouteMode::Internal:
        return "internal";
    case RouteMode::Wire:
        return "wire";
    case RouteMode::Unset:
    default:
        return "unset";
    }
}

const char* externalPortToString(const ExternalPort port)
{
    switch (port) {
    case ExternalPort::PortA1:
        return "a1";
    case ExternalPort::PortA2:
        return "a2";
    case ExternalPort::PortB1:
        return "b1";
    case ExternalPort::PortB2:
        return "b2";
    case ExternalPort::PortC1:
        return "c1";
    case ExternalPort::PortC2:
        return "c2";
    default:
        return "a1";
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
