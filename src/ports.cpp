/**
 * @file ports.cpp
 * @brief Centralized configured-port definitions and enum conversions.
 *
 * SPDX-License-Identifier: MIT
 */

#include "ports.h"

#include <cstring>

namespace sf_ports {

namespace {

constexpr PortDefinition kPortDefinitions[] = {
    {PortId::Internal, "internal", PortType::I2C, PortSensor::Internal, "M5 Internal I2C"},
    {PortId::PortA1, "a1", PortType::I2C, PortSensor::Gaz, "M5Stack Weight I2C Unit"},
    {PortId::PortA2, "a2", PortType::I2C, PortSensor::Tank, "M5Stack Unit Ultrasonic I2C (RCWL-9600)"},
    {PortId::PortB1, "b1", PortType::I2C, PortSensor::Gps, "DFRobot Gravity GNSS (DFR1103)"},
    {PortId::PortB2, "b2", PortType::Unused, PortSensor::None, ""},
    {PortId::PortC1, "c1", PortType::Unused, PortSensor::None, ""},
    {PortId::PortC2, "c2", PortType::Unused, PortSensor::None, ""},
};

}  // namespace

const PortDefinition* allPortDefinitions(size_t& count)
{
    count = sizeof(kPortDefinitions) / sizeof(kPortDefinitions[0]);
    return kPortDefinitions;
}

const PortDefinition* findPortById(const PortId id)
{
    size_t count = 0;
    const PortDefinition* defs = allPortDefinitions(count);
    for (size_t i = 0; i < count; ++i) {
        if (defs[i].id == id) {
            return &defs[i];
        }
    }
    return nullptr;
}

String normalizePortName(const String& configuredPort)
{
    String port = configuredPort;
    port.trim();
    port.toUpperCase();

    if (port.isEmpty() || port == "WIRE" || port == "PORTA" || port == "A1") return String("a1");
    if (port == "A2") return String("a2");
    if (port == "B1" || port == "PORTB") return String("b1");
    if (port == "B2") return String("b2");
    if (port == "C1") return String("c1");
    if (port == "C2") return String("c2");
    if (port == "INTERNAL" || port == "EX") return String("internal");

    return String();
}

const PortDefinition* findPortByName(const String& configuredPort)
{
    const String normalized = normalizePortName(configuredPort);
    if (normalized.isEmpty()) {
        return nullptr;
    }

    size_t count = 0;
    const PortDefinition* defs = allPortDefinitions(count);
    for (size_t i = 0; i < count; ++i) {
        if (normalized.equalsIgnoreCase(defs[i].normalizedName)) {
            return &defs[i];
        }
    }
    return nullptr;
}

const char* toString(const PortType type)
{
    switch (type) {
    case PortType::I2C:
        return "I2C";
    case PortType::UART:
        return "UART";
    case PortType::Unused:
        return "unused";
    default:
        return "unknown";
    }
}

PortType portTypeFromString(const String& value)
{
    String normalized = value;
    normalized.trim();
    normalized.toUpperCase();

    if (normalized == "I2C") return PortType::I2C;
    if (normalized == "UART") return PortType::UART;
    if (normalized == "UNUSED" || normalized.isEmpty()) return PortType::Unused;
    return PortType::Unknown;
}

const char* toString(const PortSensor sensor)
{
    switch (sensor) {
    case PortSensor::None:
        return "";
    case PortSensor::Internal:
        return "internal";
    case PortSensor::Gaz:
        return "gaz";
    case PortSensor::Tank:
        return "tank";
    case PortSensor::Gps:
        return "gps";
    default:
        return "unknown";
    }
}

PortSensor portSensorFromString(const String& value)
{
    String normalized = value;
    normalized.trim();
    normalized.toLowerCase();

    if (normalized.isEmpty() || normalized == "none" || normalized == "unused") return PortSensor::None;
    if (normalized == "internal") return PortSensor::Internal;
    if (normalized == "gaz") return PortSensor::Gaz;
    if (normalized == "tank") return PortSensor::Tank;
    if (normalized == "gps") return PortSensor::Gps;
    return PortSensor::Unknown;
}

}  // namespace sf_ports
