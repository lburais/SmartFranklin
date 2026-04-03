/**
 * @file interfaces.cpp
 * @brief Centralized interface definitions, enum conversions, and I2C bus helpers.
 *
 * SPDX-License-Identifier: MIT
 */

#include "interfaces.h"

#include "config_store.h"
#include "mqtt.h"

#include <M5Unified.h>
#include <Wire.h>

#include <cstring>
#include <mutex>

namespace sf_interfaces {

namespace {

constexpr InterfaceDefinition kInterfaceDefinitions[] = {
   //name                      type                  yellow  white sesnor                 device
    {InterfaceName::PortA1,    InterfaceType::I2C,   21,     22,   InterfaceSensor::Gaz,  "M5Stack Weight I2C Unit"},
    {InterfaceName::PortA2,    InterfaceType::I2C,   21,     22,   InterfaceSensor::Tank, "M5Stack Unit Ultrasonic I2C (RCWL-9600)"},
    {InterfaceName::PortB1,    InterfaceType::I2C,   21,     22,   InterfaceSensor::Gps,  "DFRobot Gravity GNSS (DFR1103)"},
    {InterfaceName::PortB2,    InterfaceType::UART,  -1,     -1,   InterfaceSensor::Lin,  "LIN Bus"},
    {InterfaceName::PortC1,    InterfaceType::UART,  -1,     -1,   InterfaceSensor::Lte,  "M5Stack NB-IOT2"},
    {InterfaceName::PortC2,    InterfaceType::UART,  -1,     -1,   InterfaceSensor::Lora, "M5Stack C6L"},
    {InterfaceName::Internal,  InterfaceType::I2C,   21,     22,   InterfaceSensor::Imu,  "MPU6886"},
    {InterfaceName::Internal,  InterfaceType::I2C,   21,     22,   InterfaceSensor::Rtc,  "BM8563"},
    {InterfaceName::Internal,  InterfaceType::I2C,   21,     22,   InterfaceSensor::Ina,  "INA3221"},
    {InterfaceName::Internal,  InterfaceType::I2C,   21,     22,   InterfaceSensor::Axp,  "AXP192"},
    {InterfaceName::Bluetooth, InterfaceType::BLE,   -1,     -1,   InterfaceSensor::Bat,  "Battery"},
    {InterfaceName::Bluetooth, InterfaceType::BLE,   -1,     -1,   InterfaceSensor::Obd,  "OBD"},
};

}  // namespace

const char* toString(const InterfaceType type)
{
    switch (type) {
    case InterfaceType::I2C:    return "I2C";
    case InterfaceType::UART:   return "UART";
    case InterfaceType::BLE:    return "BLE";
    case InterfaceType::Unused: return "unused";
    default:                    return "unknown";
    }
}

const char* toString(const InterfaceName id)
{
    switch (id) {
    case InterfaceName::PortA1:     return "a1";
    case InterfaceName::PortA2:     return "a2";
    case InterfaceName::PortB1:     return "b1";
    case InterfaceName::PortB2:     return "b2";
    case InterfaceName::PortC1:     return "c1";
    case InterfaceName::PortC2:     return "c2";
    case InterfaceName::Internal:   return "internal";
    case InterfaceName::Bluetooth:  return "bluetooth";
    case InterfaceName::Unknown:    return "unknown";
    default:                        return "unknown";
    }
}

const char* toString(const InterfaceSensor sensor)
{
    switch (sensor) {
    case InterfaceSensor::None:  return "";
    case InterfaceSensor::Gaz:   return "gaz";
    case InterfaceSensor::Tank:  return "tank";
    case InterfaceSensor::Gps:   return "gps";
    case InterfaceSensor::Lte:   return "lte";
    case InterfaceSensor::Lora:  return "lora";
    case InterfaceSensor::Lin:   return "lin";
    case InterfaceSensor::Imu:   return "imu";
    case InterfaceSensor::Rtc:   return "rtc";
    case InterfaceSensor::Ina:   return "ina";
    case InterfaceSensor::Axp:   return "axp";
    default:                     return "unknown";
    }
}

const InterfaceDefinition* allInterfaceDefinitions(size_t& count)
{
    count = sizeof(kInterfaceDefinitions) / sizeof(kInterfaceDefinitions[0]);
    return kInterfaceDefinitions;
}

const InterfaceDefinition* findBySensor(InterfaceSensor sensor)
{
    size_t count = 0;
    const InterfaceDefinition* defs = allInterfaceDefinitions(count);
    for (size_t i = 0; i < count; ++i) {
        if (defs[i].Sensor == sensor) {
            return &defs[i];
        }
    }
    return nullptr;
}

InterfaceName getName(const InterfaceSensor sensor)
{
    const InterfaceDefinition* iface = findBySensor(sensor);
    return iface ? iface->Name : InterfaceName::Unknown;
}

InterfaceType getType(const InterfaceSensor sensor)
{
    const InterfaceDefinition* iface = findBySensor(sensor);
    return iface ? iface->Type : InterfaceType::Unknown;
}

String getDeviceName(const InterfaceSensor sensor)
{
    const InterfaceDefinition* iface = findBySensor(sensor);
    return iface ? iface->DeviceName : "Unknown";
}

int8_t getSDA(const InterfaceSensor sensor)
{
    const InterfaceDefinition* iface = findBySensor(sensor);
    return (iface && iface->Type == InterfaceType::I2C) ? iface->pinYellow : -1;
}

int8_t getSCL(const InterfaceSensor sensor)
{
    const InterfaceDefinition* iface = findBySensor(sensor);
    return (iface && iface->Type == InterfaceType::I2C) ? iface->pinWhite : -1;
}

int8_t getRX(const InterfaceSensor sensor)
{
    const InterfaceDefinition* iface = findBySensor(sensor);
    return (iface && iface->Type == InterfaceType::UART) ? iface->pinYellow : -1;
}

int8_t getTX(const InterfaceSensor sensor)
{
    const InterfaceDefinition* iface = findBySensor(sensor);
    return (iface && iface->Type == InterfaceType::UART) ? iface->pinWhite : -1;
}

}  // namespace sf_interfaces

namespace sf_i2c {

std::mutex g_i2cRouteMutex;
bool g_wireInitialized = false;
uint32_t g_activeWireClockHz = 0;

bool i2cBeginConfiguredPort(const sf_interfaces::InterfaceSensor sensor, const uint32_t clockHz)
{
    sf_interfaces::InterfaceType portType = sf_interfaces::getType(sensor);
    if (portType != sf_interfaces::InterfaceType::I2C) {
        M5_LOGE( "[I2C] %s is not I2C", sf_interfaces::toString(sensor));
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
            M5_LOGE( "[I2C] %s has wiring issue", sf_interfaces::toString(sensor));
            return false;
        }
        g_wireInitialized = true;
        g_activeWireClockHz = clockHz;
    }

    M5_LOGI( "[I2C] %s on %d/%d", sf_interfaces::toString(sensor), sda, scl);

    return true;
}

bool i2cDeviceExistsOnConfiguredPort(const sf_interfaces::InterfaceSensor sensor,
                                     uint8_t deviceAddress,
                                     const uint32_t clockHz)
{
    sf_interfaces::InterfaceType portType = sf_interfaces::getType(sensor);
    if (portType != sf_interfaces::InterfaceType::I2C) {
        M5_LOGE( "[I2C] %s is not I2C", sf_interfaces::toString(sensor));
        return false;
    }

    if (!i2cBeginConfiguredPort(sensor, clockHz)) {
        M5_LOGE( "[I2C] %s no begin", sf_interfaces::toString(sensor));
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
