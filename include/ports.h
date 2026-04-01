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

struct SmartConfig;

namespace sf_ports {

enum class PortId : uint8_t {
    PortA1 = 0,
    PortA2,
    PortB1,
    PortB2,
    PortC1,
    PortC2,
    Unknown,
};

enum class PortType : uint8_t {
    I2C = 0,
    UART,
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
    Unknown,
};

struct PortDefinition {
    PortId id;
    const char* normalizedName;
    PortType defaultType;
    PortSensor defaultSensor;
    const char* defaultDeviceName;
};

const PortDefinition* allPortDefinitions(size_t& count);
const PortDefinition* findPortById(PortId id);
const PortDefinition* findPortByName(const String& configuredPort);

String normalizePortName(const String& configuredPort);

String defaultPortType(PortId id);
String defaultPortSensor(PortId id);
String defaultPortDeviceName(PortId id);

const String& configuredPortType(const SmartConfig& config, PortId id);
const String& configuredPortSensor(const SmartConfig& config, PortId id);
const String& configuredPortDeviceName(const SmartConfig& config, PortId id);

void setConfiguredPortType(SmartConfig& config, PortId id, const String& value);
void setConfiguredPortSensor(SmartConfig& config, PortId id, const String& value);
void setConfiguredPortDeviceName(SmartConfig& config, PortId id, const String& value);

const char* toString(PortType type);
PortType portTypeFromString(const String& value);

const char* toString(PortSensor sensor);
PortSensor portSensorFromString(const String& value);

}  // namespace sf_ports
