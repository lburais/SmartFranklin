/**
 * @file ports.cpp
 * @brief Centralized configured-port definitions and enum conversions.
 *
 * SPDX-License-Identifier: MIT
 */

#include "ports.h"

#include "config_store.h"

#include <cstring>

namespace sf_ports {

namespace {

constexpr PortDefinition kPortDefinitions[] = {
    {PortName::PortA1, PortType::I2C, PortSensor::Gaz, "M5Stack Weight I2C Unit"},
    {PortName::PortA2, PortType::I2C, PortSensor::Tank, "M5Stack Unit Ultrasonic I2C (RCWL-9600)"},
    {PortName::PortB1, PortType::I2C, PortSensor::Gps, "DFRobot Gravity GNSS (DFR1103)"},
    {PortName::PortB2, PortType::UART, PortSensor::Lin, "LIN Bus"},
    {PortName::PortC1, PortType::UART, PortSensor::Lte, "M5Stack NB-IOT2"},
    {PortName::PortC2, PortType::UART, PortSensor::Lora, "M5Stack C6L"},
    {PortName::Internal, PortType::I2C, PortSensor::Imu, "MPU6886"},
    {PortName::Internal, PortType::I2C, PortSensor::Rtc, "BM8563"},
    {PortName::Internal, PortType::I2C, PortSensor::Ina, "INA3221"},
    {PortName::Internal, PortType::I2C, PortSensor::Axp, "AXP192"},
    {PortName::Bluetooth, PortType::BLE, PortSensor::Bat, "Battery"},
    {PortName::Bluetooth, PortType::BLE, PortSensor::Obd, "OBD"},
};

}  // namespace

const char* toString(const PortType type)
{
    switch (type) {
    case PortType::I2C:
        return "I2C";
    case PortType::UART:
        return "UART";
    case PortType::BLE:
        return "BLE";
    case PortType::Unused:
        return "unused";
    default:
        return "unknown";
    }
}

const char* toString(const PortName id)
{
    switch (id) {
    case PortName::PortA1:
        return "a1";
    case PortName::PortA2:
        return "a2";
    case PortName::PortB1:
        return "b1";
    case PortName::PortB2:
        return "b2";
    case PortName::PortC1:
        return "c1";
    case PortName::PortC2:
        return "c2";
    case PortName::Internal:
        return "internal";
    case PortName::Bluetooth:
        return "bluetooth";
    case PortName::Unknown:
        return "unknown";
    default:
        return "unknown";
    }
}

const char* toString(const PortSensor sensor)
{
    switch (sensor) {
    case PortSensor::None:
        return "";
    case PortSensor::Gaz:
        return "gaz";
    case PortSensor::Tank:
        return "tank";
    case PortSensor::Gps:
        return "gps";
    case PortSensor::Lte:
        return "lte";
    case PortSensor::Lora:
        return "lora";
    case PortSensor::Lin:
        return "lin";
    case PortSensor::Imu:
        return "imu";
    case PortSensor::Rtc:
        return "rtc";
    case PortSensor::Ina:
        return "ina";
    case PortSensor::Axp:
        return "axp";
    default:
        return "unknown";
    }
}

const PortDefinition* allPortDefinitions(size_t& count)
{
    count = sizeof(kPortDefinitions) / sizeof(kPortDefinitions[0]);
    return kPortDefinitions;
}

const PortDefinition* findPortBySensor(PortSensor sensor)
{
    size_t count = 0;
    const PortDefinition* defs = allPortDefinitions(count);
    for (size_t i = 0; i < count; ++i) {
        if (defs[i].Sensor == sensor) {
            return &defs[i];
        }
    }
    return nullptr;
}

PortName getPortName(const PortSensor sensor){
    const PortDefinition* port = findPortBySensor(sensor);
    return port ? port->Name : PortName::Unknown;
}

PortType getPortType(const PortSensor sensor){
    const PortDefinition* port = findPortBySensor(sensor);
    return port ? port->Type : PortType::Unknown;
}

String getPortDeviceName(const PortSensor sensor){
    const PortDefinition* port = findPortBySensor(sensor);
    return port ? port->DeviceName : "Unknown";
}

}  // namespace sf_ports
