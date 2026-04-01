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

enum class ExternalPort : uint8_t {
    PortA1 = 0,
    PortA2,
    PortB1,
    PortB2,
    PortC1,
    PortC2,
};

struct ResolvedPort {
    bool valid = false;
    bool internal = false;
    ExternalPort externalPort = ExternalPort::PortA1;
    const char* normalized = "";
};

std::mutex g_i2cRouteMutex;
bool g_wireInitialized = false;
ExternalPort g_activeWirePort = ExternalPort::PortA1;
uint32_t g_activeWireClockHz = 0;
bool g_exInitialized = false;

ExternalPort portIdToExternalPort(const sf_ports::PortId id)
{
    switch (id) {
    case sf_ports::PortId::PortA1:
        return ExternalPort::PortA1;
    case sf_ports::PortId::PortA2:
        return ExternalPort::PortA2;
    case sf_ports::PortId::PortB1:
        return ExternalPort::PortB1;
    case sf_ports::PortId::PortB2:
        return ExternalPort::PortB2;
    case sf_ports::PortId::PortC1:
        return ExternalPort::PortC1;
    case sf_ports::PortId::PortC2:
        return ExternalPort::PortC2;
    default:
        return ExternalPort::PortA1;
    }
}

ResolvedPort resolveConfiguredPort(const String& configuredPort, const char* label, const bool logInvalid)
{
    const sf_ports::PortDefinition* def = sf_ports::findPortByName(configuredPort);
    if (def != nullptr) {
        ResolvedPort port{};
        port.valid = true;
        port.normalized = def->normalizedName;

        if (def->id == sf_ports::PortId::Internal) {
            port.internal = true;
            port.externalPort = ExternalPort::PortA1;
            return port;
        }

        port.internal = false;
        port.externalPort = portIdToExternalPort(def->id);
        return port;
    }

    if (logInvalid) {
        M5_LOGW("[%s] invalid configured I2C port '%s' (valid: INTERNAL, EX, A1, A2, B1, B2, C1, C2)",
                (label != nullptr) ? label : "I2C",
                configuredPort.c_str());
    }
    return {};
}

void beginInternalBus()
{
    std::lock_guard<std::mutex> lock(g_i2cRouteMutex);
    if (!g_exInitialized) {
        M5.Ex_I2C.begin();
        g_exInitialized = true;
    }
}

void beginExternalBus(const ExternalPort port, const uint32_t clockHz)
{
    std::lock_guard<std::mutex> lock(g_i2cRouteMutex);

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
    }

    if (sda < 0 || scl < 0) {
        return;
    }

    const bool wireNeedsReinit = !g_wireInitialized
                              || g_activeWirePort != port
                              || g_activeWireClockHz != clockHz;

    if (wireNeedsReinit) {
        Wire.end();
        Wire.begin(sda, scl, clockHz);
        Wire.setPins(sda, scl);
        g_wireInitialized = true;
        g_activeWirePort = port;
        g_activeWireClockHz = clockHz;
    }

    if (!g_exInitialized) {
        M5.Ex_I2C.begin();
        g_exInitialized = true;
    }
}

bool wireDeviceExists(const uint8_t address)
{
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

bool exDeviceExists(const uint8_t address, const uint32_t clockHz)
{
    return M5.Ex_I2C.scanID(address, clockHz);
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
    beginExternalBus(ExternalPort::PortA1, clockHz);
}

bool i2cBeginConfiguredPort(const String& configuredPort, const char* label, const uint32_t clockHz)
{
    const ResolvedPort port = resolveConfiguredPort(configuredPort, label, true);
    if (!port.valid) {
        return false;
    }

    if (port.internal) {
        beginInternalBus();
    } else {
        beginExternalBus(port.externalPort, clockHz);
    }

    return true;
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

    return port.internal ? exDeviceExists(deviceAddress, clockHz)
                         : wireDeviceExists(deviceAddress);
}

bool i2cIsConfiguredPortInternal(const String& configuredPort)
{
    const ResolvedPort port = resolveConfiguredPort(configuredPort, nullptr, false);
    return port.valid && port.internal;
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
