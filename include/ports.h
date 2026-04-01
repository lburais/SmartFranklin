/**
 * @file ports.h
 * @brief Centralized configured-port definitions and enum conversions.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <Arduino.h>

#include <cstddef>
#include <cstdint>

//struct SmartConfig;

namespace sf_ports {

enum class PortName : uint8_t {
    PortA1 = 0,
    PortA2,
    PortB1,
    PortB2,
    PortC1,
    PortC2,
    Internal,
    Bluetooth,
    Unknown,
};

enum class PortType : uint8_t {
    I2C = 0,
    UART,
    BLE,
    Unused,
    Unknown,
};

enum class PortSensor : uint8_t {
    None = 0,
    Gaz,
    Tank,
    Gps,
    Lte,
    Lora,
    Lin,
    Imu,
    Rtc,
    Ina,
    Axp,
    Bat,
    Obd,
    Unknown,
};

struct PortDefinition {
    PortName Name;
    PortType Type;
    PortSensor Sensor;
    const char* DeviceName;
};

const PortDefinition* allPortDefinitions(size_t& count);
const PortDefinition* findPortBySensor(PortSensor sensor);

PortName getPortName(PortSensor sensor);
PortType getPortType(PortSensor sensor);
String getPortDeviceName(PortSensor sensor);

const char* toString(PortName name);
const char* toString(PortType type);
const char* toString(PortSensor sensor);

}  // namespace sf_ports
