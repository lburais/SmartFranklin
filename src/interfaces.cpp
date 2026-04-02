/**
 * @file interfaces.cpp
 * @brief Centralized interface definitions and enum conversions.
 *
 * SPDX-License-Identifier: MIT
 */

#include "interfaces.h"

#include "config_store.h"

#include <cstring>

namespace sf_interfaces {

namespace {

constexpr InterfaceDefinition kInterfaceDefinitions[] = {
    {InterfaceName::PortA1,    InterfaceType::I2C,  InterfaceSensor::Gaz,  "M5Stack Weight I2C Unit"},
    {InterfaceName::PortA2,    InterfaceType::I2C,  InterfaceSensor::Tank, "M5Stack Unit Ultrasonic I2C (RCWL-9600)"},
    {InterfaceName::PortB1,    InterfaceType::I2C,  InterfaceSensor::Gps,  "DFRobot Gravity GNSS (DFR1103)"},
    {InterfaceName::PortB2,    InterfaceType::UART, InterfaceSensor::Lin,  "LIN Bus"},
    {InterfaceName::PortC1,    InterfaceType::UART, InterfaceSensor::Lte,  "M5Stack NB-IOT2"},
    {InterfaceName::PortC2,    InterfaceType::UART, InterfaceSensor::Lora, "M5Stack C6L"},
    {InterfaceName::Internal,  InterfaceType::I2C,  InterfaceSensor::Imu,  "MPU6886"},
    {InterfaceName::Internal,  InterfaceType::I2C,  InterfaceSensor::Rtc,  "BM8563"},
    {InterfaceName::Internal,  InterfaceType::I2C,  InterfaceSensor::Ina,  "INA3221"},
    {InterfaceName::Internal,  InterfaceType::I2C,  InterfaceSensor::Axp,  "AXP192"},
    {InterfaceName::Bluetooth, InterfaceType::BLE,  InterfaceSensor::Bat,  "Battery"},
    {InterfaceName::Bluetooth, InterfaceType::BLE,  InterfaceSensor::Obd,  "OBD"},
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

}  // namespace sf_interfaces
