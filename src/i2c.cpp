/**
 * @file i2c.cpp
 * @brief I2C direct bus configuration implementation based on configured ports.
 *
 * This module initializes configured ports, probes peripherals, and publishes
 * per-port metadata from CONFIG.
 *
 * SPDX-License-Identifier: MIT
 */

#include "i2c.h"

#include <M5Unified.h>
#include <Wire.h>

#include <mutex>

#include "config_store.h"
#include "mqtt.h"

namespace {

struct ResolvedPort {
    bool valid = false;
    sf_ports::PortName portName = sf_ports::PortName::Unknown;
};

std::mutex g_i2cRouteMutex;
bool g_wireInitialized = false;
uint32_t g_activeWireClockHz = 0;

ResolvedPort resolveConfiguredPort(sf_ports::PortSensor sensor, const char* label, const bool logInvalid)
{
    const sf_ports::PortDefinition* def = sf_ports::findPortByName(configuredPort);
    if (def != nullptr) {
        ResolvedPort port{};
        port.valid = true;
        port.normalized = def->normalizedName;
        port.portId = def->id;
        return port;
    }

    if (logInvalid) {
        M5_LOGW("[%s] invalid configured I2C port '%s' (valid: A1, A2, B1, B2, C1, C2)",
                (label != nullptr) ? label : "I2C",
                configuredPort.c_str());
    }
    return {};
}

bool beginSharedWireBus(const uint32_t clockHz)
{
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

bool wireDeviceExists(const uint8_t address)
{
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

String configuredTypeForNormalizedPort(const char* normalized)
{
    const sf_ports::PortDefinition* def = sf_ports::findPortByName(String(normalized));
    return (def != nullptr) ? sf_ports::configuredPortType(CONFIG, def->id) : String();
}

String configuredSensorForNormalizedPort(const char* normalized)
{
    const sf_ports::PortDefinition* def = sf_ports::findPortByName(String(normalized));
    return (def != nullptr) ? sf_ports::configuredPortSensor(CONFIG, def->id) : String();
}

String configuredDeviceNameForNormalizedPort(const char* normalized)
{
    const sf_ports::PortDefinition* def = sf_ports::findPortByName(String(normalized));
    return (def != nullptr) ? sf_ports::configuredPortDeviceName(CONFIG, def->id) : String();
}

}  // namespace

void i2cBeginPortA(const uint32_t clockHz)
{
    static_cast<void>(beginSharedWireBus(clockHz));
}

bool i2cBeginConfiguredPort(const String& configuredPort, const char* label, const uint32_t clockHz)
{
    const ResolvedPort port = resolveConfiguredPort(configuredPort, label, true);
    if (!port.valid) {
        return false;
    }

    return beginSharedWireBus(clockHz);
}

bool i2cDeviceExistsOnConfiguredPort(const uint8_t deviceAddress,
                                     const String& configuredPort,
                                     const char* label,
                                     const uint32_t clockHz)
{
    const ResolvedPort port = resolveConfiguredPort(configuredPort, label, true);
    if (!port.valid) {
        return false;
    }

    if (!i2cBeginConfiguredPort(configuredPort, label, clockHz)) {
        return false;
    }

    return wireDeviceExists(deviceAddress);
}

String i2cPortName(const String& configuredPort)
{
    const ResolvedPort port = resolveConfiguredPort(configuredPort, nullptr, false);
    return port.valid ? String(port.normalized) : String();
}

String i2cPortType(const String& configuredPort)
{
    const ResolvedPort port = resolveConfiguredPort(configuredPort, nullptr, false);
    return port.valid ? configuredTypeForNormalizedPort(port.normalized) : String();
}

String i2cPortSensor(const String& configuredPort)
{
    const ResolvedPort port = resolveConfiguredPort(configuredPort, nullptr, false);
    return port.valid ? configuredSensorForNormalizedPort(port.normalized) : String();
}

String i2cConfiguredPortForSensor(const sf_ports::PortSensor sensor,
                                  const char* label)
{
    if (sensor == sf_ports::PortSensor::None || sensor == sf_ports::PortSensor::Unknown) {
        M5_LOGW("[%s] invalid sensor enum while resolving configured I2C port",
                (label != nullptr) ? label : "I2C");
        return String();
    }

    size_t count = 0;
    const sf_ports::PortDefinition* defs = sf_ports::allPortDefinitions(count);
    for (size_t i = 0; i < count; ++i) {
        const String portType = configuredTypeForNormalizedPort(defs[i].normalizedName);
        const String portSensor = configuredSensorForNormalizedPort(defs[i].normalizedName);

        if (sf_ports::portTypeFromString(portType) == sf_ports::PortType::I2C &&
            sf_ports::portSensorFromString(portSensor) == sensor) {
            return String(defs[i].normalizedName);
        }
    }

    M5_LOGW("[%s] no I2C configured port found for sensor '%s'",
            (label != nullptr) ? label : "I2C",
            sf_ports::toString(sensor));
    return String();
}

String i2cPortDeviceName(const String& configuredPort)
{
    const ResolvedPort port = resolveConfiguredPort(configuredPort, nullptr, false);
    return port.valid ? configuredDeviceNameForNormalizedPort(port.normalized) : String();
}

void i2cPublishConfiguration(const char* tag, const String& configuredPort, const uint8_t address)
{
    char addressBuf[8] = {0};
    char topicBuf[64] = {0};
    const char* topicTag = (tag != nullptr) ? tag : "unknown";

    snprintf(addressBuf, sizeof(addressBuf), "0x%02X", address);

    const String portName = i2cPortName(configuredPort);
    const String type = i2cPortType(configuredPort);
    const String sensor = i2cPortSensor(configuredPort);
    const String deviceName = i2cPortDeviceName(configuredPort);

    snprintf(topicBuf, sizeof(topicBuf), "smartfranklin/system/i2c/%s/address", topicTag);
    sf_mqtt::publish(topicBuf, addressBuf, 1, true);
    snprintf(topicBuf, sizeof(topicBuf), "smartfranklin/system/i2c/%s/port", topicTag);
    sf_mqtt::publish(topicBuf, portName.c_str(), 1, true);
    snprintf(topicBuf, sizeof(topicBuf), "smartfranklin/system/i2c/%s/type", topicTag);
    sf_mqtt::publish(topicBuf, type.c_str(), 1, true);
    snprintf(topicBuf, sizeof(topicBuf), "smartfranklin/system/i2c/%s/sensor", topicTag);
    sf_mqtt::publish(topicBuf, sensor.c_str(), 1, true);
    snprintf(topicBuf, sizeof(topicBuf), "smartfranklin/system/i2c/%s/device_name", topicTag);
    sf_mqtt::publish(topicBuf, deviceName.c_str(), 1, true);
}
