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
const PortDefinition* findBySensor(PortSensor sensor);

PortName getName(PortSensor sensor);
PortType getType(PortSensor sensor);
String getDeviceName(PortSensor sensor);

const char* toString(PortName name);
const char* toString(PortType type);
const char* toString(PortSensor sensor);

}  // namespace sf_ports
