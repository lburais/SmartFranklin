/**
 * @file i2c.cpp
 * @brief I2C direct bus configuration implementation based on configured ports.
 *
 * This module initializes configured ports, probes peripherals, and publishes
 * per-port metadata from CONFIG.
 *
 * SPDX-License-Identifier: MIT
 */

#include "interfaces.h"

#include <M5Unified.h>
#include <Wire.h>

#include <mutex>

#include "mqtt.h"

namespace sf_i2c {

std::mutex g_i2cRouteMutex;
bool g_wireInitialized = false;
uint32_t g_activeWireClockHz = 0;

bool i2cBeginConfiguredPort(const sf_interfaces::InterfaceSensor sensor, const uint32_t clockHz)
{
    sf_interfaces::InterfaceType portType = sf_interfaces::getType(sensor);
    if (portType != sf_interfaces::InterfaceType::I2C) {
        return false;
    }

    std::lock_guard<std::mutex> lock(g_i2cRouteMutex);

    int8_t sda = static_cast<int8_t>(M5.getPin(m5::pin_name_t::port_a_sda));
    int8_t scl = static_cast<int8_t>(M5.getPin(m5::pin_name_t::port_a_scl));

    // M5Station routes external Grove ports on main Wire (default 21/22).
    if (sda < 0) {
        sda = 21;
    }
    if (scl < 0) {
        scl = 22;
    }

    const bool wireNeedsReinit = !g_wireInitialized || g_activeWireClockHz != clockHz;

    if (wireNeedsReinit) {
        Wire.end();
        if (!Wire.begin(sda, scl, clockHz)) {
            g_wireInitialized = false;
            return false;
        }
        g_wireInitialized = true;
        g_activeWireClockHz = clockHz;
    }

    return true;

}

bool i2cDeviceExistsOnConfiguredPort(const sf_interfaces::InterfaceSensor sensor,
                                     uint8_t deviceAddress,
                                     const uint32_t clockHz)
{
    sf_interfaces::InterfaceType portType = sf_interfaces::getType(sensor);
    if (portType != sf_interfaces::InterfaceType::I2C) {
        return false;
    }

    if (!i2cBeginConfiguredPort(sensor, clockHz)) {
        return false;
    }

    Wire.beginTransmission(deviceAddress);
    return Wire.endTransmission() == 0;
}

void i2cPublishConfiguration(const sf_interfaces::InterfaceSensor sensor, const uint8_t address)
{
    char addressBuf[8] = {0};
    char topicBuf[64] = {0};
    const char* topicTag = sf_interfaces::toString(sensor);

    snprintf(addressBuf, sizeof(addressBuf), "0x%02X", address);

    const String name = sf_interfaces::toString(sf_interfaces::getName(sensor));
    const String type = sf_interfaces::toString(sf_interfaces::getType(sensor));
    const String deviceName = sf_interfaces::getDeviceName(sensor);

    snprintf(topicBuf, sizeof(topicBuf), "smartfranklin/system/i2c/%s/address", topicTag);
    sf_mqtt::publish(topicBuf, addressBuf, 1, true);
    snprintf(topicBuf, sizeof(topicBuf), "smartfranklin/system/i2c/%s/port", topicTag);
    sf_mqtt::publish(topicBuf, name.c_str(), 1, true);
    snprintf(topicBuf, sizeof(topicBuf), "smartfranklin/system/i2c/%s/type", topicTag);
    sf_mqtt::publish(topicBuf, type.c_str(), 1, true);
    snprintf(topicBuf, sizeof(topicBuf), "smartfranklin/system/i2c/%s/device_name", topicTag);
    sf_mqtt::publish(topicBuf, deviceName.c_str(), 1, true);
}

}  // namespace sf_i2c
