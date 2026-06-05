/**
 * @file interfaces.cpp
 * @brief Centralized interface definitions, enum conversions, and I2C bus helpers.
 *
 * SPDX-License-Identifier: MIT
 */

#include "interfaces.h"

#include "mqtt.h"
#include "log.h"
#include "hmi.h"

#include <HardwareSerial.h>
#include <Wire.h>

#include <cstddef>
#include <iterator>

namespace sf_interfaces {

namespace {

// =========================================================================
// Connectors
// =========================================================================

enum class InterfacePortType : uint8_t {
    I2C = 0,    
    I2C1,    
    UART1,
    UART2,
    UART3,
    BLE,
    Unused,
    Unknown,
};

struct InterfacePortMap {
    InterfacePortType  portType;
    SemaphoreHandle_t  lock;
    bool               configured;
    int8_t             lastPinYellow;
    int8_t             lastPinWhite;
    uint32_t           lastClock;
    InterfaceConnector connector;      ///< TwoWire or HardwareSerial
};

InterfacePortMap kConnectors[] = {
//                                                       type                      lock                      configured? ye  wh  cl.  connector
    [static_cast<size_t>(InterfacePortType::I2C)]     = {InterfacePortType::I2C,   xSemaphoreCreateMutex(),  false,      -1, -1, 0UL, {}},
    [static_cast<size_t>(InterfacePortType::I2C1)]    = {InterfacePortType::I2C1,                  nullptr,  false,      -1, -1, 0UL, {}},
    [static_cast<size_t>(InterfacePortType::UART1)]   = {InterfacePortType::UART1,                 nullptr,  false,      -1, -1, 0UL, {}},
    [static_cast<size_t>(InterfacePortType::UART2)]   = {InterfacePortType::UART2,                 nullptr,  false,      -1, -1, 0UL, {}},
    [static_cast<size_t>(InterfacePortType::UART3)]   = {InterfacePortType::UART3,                 nullptr,  false,      -1, -1, 0UL, {}},
    [static_cast<size_t>(InterfacePortType::BLE)]     = {InterfacePortType::BLE,                   nullptr,  false,      -1, -1, 0UL, {}},
    [static_cast<size_t>(InterfacePortType::Unused)]  = {InterfacePortType::Unused,                nullptr,  false,      -1, -1, 0UL, {}},
    [static_cast<size_t>(InterfacePortType::Unknown)] = {InterfacePortType::Unknown,               nullptr,  false,      -1, -1, 0UL, {}},
};
static_assert(
    (sizeof(kConnectors) / sizeof(kConnectors[0])) == static_cast<size_t>(InterfacePortType::Unknown) + 1,
    "kConnectors size mismatch"
);

const char* toString(const InterfacePortType type)
{
    switch (type) {
    case InterfacePortType::I2C:    return "I2C";
    case InterfacePortType::I2C1:   return "I2C1";
    case InterfacePortType::UART1:  return "UART1";
    case InterfacePortType::UART2:  return "UART2";
    case InterfacePortType::UART3:  return "UART3";
    case InterfacePortType::BLE:    return "BLE";
    case InterfacePortType::Unused: return "unused";
    default:                        return "unknown";
    }
}

// =========================================================================
// Ports
// =========================================================================

enum class InterfacePortName : uint8_t {
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

const char* toString(const InterfacePortName id)
{
    switch (id) {
    case InterfacePortName::PortA1:    return "a1";
    case InterfacePortName::PortA2:    return "a2";
    case InterfacePortName::PortB1:    return "b1";
    case InterfacePortName::PortB2:    return "b2";
    case InterfacePortName::PortC1:    return "c1";
    case InterfacePortName::PortC2:    return "c2";
    case InterfacePortName::Internal:  return "internal";
    case InterfacePortName::Bluetooth: return "bluetooth";
    case InterfacePortName::Unknown:   return "unknown";
    default:                           return "unknown";
    }
}

// =========================================================================
// Sensors
// =========================================================================

struct InterfaceSensorMap {
    InterfaceSensor   sensor;
    InterfacePortName portName;
    InterfacePortType portType;
    int8_t            pinYellow;     ///< SDA (I2C) or RX (UART); -1 if not applicable
    int8_t            pinWhite;      ///< SCL (I2C) or TX (UART); -1 if not applicable
    int8_t            led;           ///< led number; -1 if not applicable
    uint8_t           address;       ///< I2C device address; 0 if not applicable
    uint32_t          clock;         ///< I2C clock frequency in Hz or UART bauds; 0 if not applicable
    uint32_t          recurrenceMs;  ///< Suggested task recurrence for this sensor
    bool              available;     ///< Last availability state set by configure(sensor)
    InterfacePortMap* portMap;
    const char*       deviceName;
};

InterfaceSensorMap kSensors[] = {
//                                                  sensor                 port                          type                        yellow white led  addr  clock     recur  avail? connection device name
    [static_cast<size_t>(InterfaceSensor::Axp)]  = {InterfaceSensor::Axp,  InterfacePortName::Internal,  InterfacePortType::I2C1,    -1,    -1,   -1,  0x34, 400000UL, 30000, false, nullptr,   "AXP192 power management"},
    [static_cast<size_t>(InterfaceSensor::Gaz)]  = {InterfaceSensor::Gaz,  InterfacePortName::PortA1,    InterfacePortType::I2C,     -1,    -1,   -1,  0x26, 400000UL, 10000, false, nullptr,   "M5Stack Weight I2C Unit"},
    [static_cast<size_t>(InterfaceSensor::Tank)] = {InterfaceSensor::Tank, InterfacePortName::PortA2,    InterfacePortType::I2C,     -1,    -1,   -1,  0x57, 100000UL, 15000, false, nullptr,   "M5Stack Unit Ultrasonic I2C (RCWL-9600)"},
    [static_cast<size_t>(InterfaceSensor::Gps)]  = {InterfaceSensor::Gps,  InterfacePortName::PortC1,    InterfacePortType::I2C,     -1,    -1,   -1,  0x66, 400000UL, 30000, false, nullptr,   "DFRobot Gravity GNSS (DFR1103)"},
    [static_cast<size_t>(InterfaceSensor::Lte)]  = {InterfaceSensor::Lte,  InterfacePortName::PortB1,    InterfacePortType::UART1,   -1,    -1,   -1,  0x00, 115200UL, 10000, false, nullptr,   "M5Stack NB-IOT2"},
    [static_cast<size_t>(InterfaceSensor::Lora)] = {InterfaceSensor::Lora, InterfacePortName::PortC2,    InterfacePortType::UART3,   -1,    -1,   -1,  0x00, 115200UL, 10000, false, nullptr,   "M5Stack C6L"},
    [static_cast<size_t>(InterfaceSensor::Lin)]  = {InterfaceSensor::Lin,  InterfacePortName::PortB2,    InterfacePortType::UART2,   -1,    -1,   -1,  0x00, 115200UL, 10000, false, nullptr,   "LIN Bus"},
    [static_cast<size_t>(InterfaceSensor::Imu)]  = {InterfaceSensor::Imu,  InterfacePortName::Internal,  InterfacePortType::I2C1,    -1,    -1,   -1,  0x68, 400000UL, 10000, false, nullptr,   "MPU6886 6-axis IMU"},
    [static_cast<size_t>(InterfaceSensor::Rtc)]  = {InterfaceSensor::Rtc,  InterfacePortName::Internal,  InterfacePortType::I2C1,    -1,    -1,   -1,  0x51, 400000UL, 60000, false, nullptr,   "BM8563 RTC"},
    [static_cast<size_t>(InterfaceSensor::Ina1)] = {InterfaceSensor::Ina1, InterfacePortName::Internal,  InterfacePortType::I2C1,    -1,    -1,   -1,  0x40, 400000UL, 60000, false, nullptr,   "INA3221 3 channel voltage and current sensor"},
    [static_cast<size_t>(InterfaceSensor::Ina2)] = {InterfaceSensor::Ina2, InterfacePortName::Internal,  InterfacePortType::I2C1,    -1,    -1,   -1,  0x41, 400000UL, 60000, false, nullptr,   "INA3221 3 channel voltage and current sensor"},
    [static_cast<size_t>(InterfaceSensor::Bat)]  = {InterfaceSensor::Bat,  InterfacePortName::Bluetooth, InterfacePortType::BLE,     -1,    -1,   -1,  0x00,      0UL, 10000, false, nullptr,   "Battery"},
    [static_cast<size_t>(InterfaceSensor::Obd)]  = {InterfaceSensor::Obd,  InterfacePortName::Bluetooth, InterfacePortType::BLE,     -1,    -1,   -1,  0x00,      0UL, 10000, false, nullptr,   "OBD"},
    [static_cast<size_t>(InterfaceSensor::None)] = {InterfaceSensor::None, InterfacePortName::Unknown,   InterfacePortType::Unknown, -1,    -1,   -1,  0x00,      0UL, 10000, false, nullptr,   "Unknown"},
};
static_assert(
    (sizeof(kSensors) / sizeof(kSensors[0])) == static_cast<size_t>(InterfaceSensor::None) + 1,
    "kSensors size mismatch"
);

// =========================================================================
// Inline
// =========================================================================

inline InterfaceSensorMap* getSensor(InterfaceSensor sensor) {
    return (sensor != InterfaceSensor::None) ? &kSensors[static_cast<size_t>(sensor)] : nullptr;
}

inline InterfacePortMap* getPortMap(InterfacePortType type) {
    return (type != InterfacePortType::Unknown) ? &kConnectors[static_cast<size_t>(type)] : nullptr;
}

inline InterfacePortMap* getPortMap(InterfaceSensor sensor) {
    InterfaceSensorMap* map = getSensor(sensor);
    return map ? getPortMap(map->portType) : nullptr;
}

// =========================================================================
// Private / Internal
// =========================================================================

void publishConfiguration(const InterfaceSensor sensor)
{
    const InterfaceSensorMap* map = getSensor(sensor);

    char topicBuf[96] = {0};
    const char* topicTag = toString(sensor);

    char addressBuf[8] = {0};
    snprintf(addressBuf, sizeof(addressBuf), "0x%02X", map->address);

    const String name = toString(sensor);

    const String deviceName = map->deviceName;

    const String type = toString(map->portType);

    snprintf(topicBuf, sizeof(topicBuf), "smartfranklin/system/interface/%s/address", topicTag);
    sf_mqtt::publish(topicBuf, addressBuf, 1, true);
    snprintf(topicBuf, sizeof(topicBuf), "smartfranklin/system/interface/%s/port", topicTag);
    sf_mqtt::publish(topicBuf, name.c_str(), 1, true);
    snprintf(topicBuf, sizeof(topicBuf), "smartfranklin/system/interface/%s/type", topicTag);
    sf_mqtt::publish(topicBuf, type.c_str(), 1, true);
    snprintf(topicBuf, sizeof(topicBuf), "smartfranklin/system/interface/%s/device_name", topicTag);
    sf_mqtt::publish(topicBuf, deviceName.c_str(), 1, true);
}

}  // namespace

// =========================================================================
// Strings
// =========================================================================

const char* toString(const InterfaceSensor sensor, bool upper)
{
    if (upper) {
        switch (sensor) {
        case InterfaceSensor::None:  return "";
        case InterfaceSensor::Gaz:   return "GAZ";
        case InterfaceSensor::Tank:  return "TANK";
        case InterfaceSensor::Gps:   return "GPS";
        case InterfaceSensor::Lte:   return "LTE";
        case InterfaceSensor::Lora:  return "LORA";
        case InterfaceSensor::Lin:   return "LIN";
        case InterfaceSensor::Imu:   return "IMU";
        case InterfaceSensor::Rtc:   return "RTC";
        case InterfaceSensor::Ina1:  return "INA1";
        case InterfaceSensor::Ina2:  return "INA2";
        case InterfaceSensor::Axp:   return "AXP";
        case InterfaceSensor::Bat:   return "BAT";
        case InterfaceSensor::Obd:   return "OBD";
        default:                     return "UNKNOWN";
        }
    }

    switch (sensor) {
    case InterfaceSensor::None:   return "";
    case InterfaceSensor::Gaz:    return "gaz";
    case InterfaceSensor::Tank:   return "tank";
    case InterfaceSensor::Gps:    return "gps";
    case InterfaceSensor::Lte:    return "lte";
    case InterfaceSensor::Lora:   return "lora";
    case InterfaceSensor::Lin:    return "lin";
    case InterfaceSensor::Imu:    return "imu";
    case InterfaceSensor::Rtc:    return "rtc";
    case InterfaceSensor::Ina1:   return "ina1";
    case InterfaceSensor::Ina2:   return "ina2";
    case InterfaceSensor::Axp:    return "axp";
    case InterfaceSensor::Bat:    return "bat";
    case InterfaceSensor::Obd:    return "obd";
    default:                      return "unknown";
    }
}

// =========================================================================
// Get
// =========================================================================

const char* getDeviceName(const InterfaceSensor sensor)
{
    const InterfaceSensorMap* map = getSensor(sensor);
    return (map && map->deviceName) ? map->deviceName : "Unknown";
}

uint8_t getAddress(const InterfaceSensor sensor)
{
    const InterfaceSensorMap* map = getSensor(sensor);
    return map ? map->address : 0;
}

int8_t getLed(const InterfaceSensor sensor)
{
    const InterfaceSensorMap* map = getSensor(sensor);
    return map ? map->led : -1;
}

int8_t getSda(const InterfaceSensor sensor)
{
    const InterfaceSensorMap* map = getSensor(sensor);
    return map ? map->pinYellow : -1;
}

int8_t getScl(const InterfaceSensor sensor)
{
    const InterfaceSensorMap* map = getSensor(sensor);
    return map ? map->pinWhite : -1;
}

uint32_t getClock(const InterfaceSensor sensor)
{
    const InterfaceSensorMap* map = getSensor(sensor);
    return map ? map->clock : 0UL;
}

uint32_t getRecurrenceMs(const InterfaceSensor sensor)
{
    const InterfaceSensorMap* map = getSensor(sensor);
    return map ? map->recurrenceMs : 0;
}

const InterfaceConnector getConnector(const InterfaceSensor sensor)
{
    const InterfacePortMap* map = getPortMap(sensor);
    if (map == nullptr) {
        SF_LOGW("[IFACE] getConnector(%s) -> connection not found", toString(sensor));
        HMI::setLed(sensor, PortStatus::Error);
        return InterfaceConnector{};
    }
    return map->connector;
}

const char* getSensor(const InterfaceSensor sensor, const uint8_t channel)
{
    InterfacePortName port = InterfacePortName::Unknown;
    switch (sensor) {
    case (InterfaceSensor::Ina1):
        switch (channel) {
        case 1: port = InterfacePortName::PortA1; break;
        case 2: port = InterfacePortName::PortA2; break;
        case 3: port = InterfacePortName::PortB1; break;
        };
        break;
    case (InterfaceSensor::Ina2):
        switch (channel) {
        case 1: port = InterfacePortName::PortB2; break;
        case 2: port = InterfacePortName::PortC1; break;
        case 3: port = InterfacePortName::PortC2; break;
        };
        break;
    }
    if (port != InterfacePortName::Unknown) {
        const size_t count = sizeof(kSensors) / sizeof(kSensors[0]);
        for (size_t i = 0; i < count; ++i) {
            if (kSensors[i].portName == port) {
                return toString(kSensors[i].sensor);
            }
        }
    }

    return "Unknown";
}

// =========================================================================
// Configure
// =========================================================================

bool configure_all_sensors()
{
    bool allOk = true;
    const size_t count = sizeof(kSensors) / sizeof(kSensors[0]);
    for (size_t i = 0; i < count; ++i) {
        SF_LOGI("[IFACE] configure_all_sensors -> %s", toString(kSensors[i].sensor));
        allOk = configure(kSensors[i].sensor) && allOk;
    }
    return allOk;
}

bool configure(const InterfaceSensor sensor)
{
    SF_LOGI("[IFACE] configure(%s) -> configuring...", toString(sensor));

    InterfaceSensorMap* map = getSensor(sensor);
    if (map == nullptr) {
        SF_LOGW("[IFACE] configure(%s) -> sensor not found", toString(sensor));
        return false;
    }

    map->available = false;

    switch (map->portName) {
        case InterfacePortName::PortA1:
            map->pinYellow = 32;
            map->pinWhite = 33;
            map->led = 0;
            break;
        case InterfacePortName::PortA2:
            map->pinYellow = 32;
            map->pinWhite = 33;
            map->led = 6;
            break;
        case InterfacePortName::PortB1:
            map->pinYellow = 35;
            map->pinWhite = 25;
            map->led = 1;
            break;
        case InterfacePortName::PortB2:
            map->pinYellow = 36;
            map->pinWhite = 26;
            map->led = 5;
            break;
        case InterfacePortName::PortC1:
            map->pinYellow = 14;
            map->pinWhite = 13;
            map->led = 2;
            break;
        case InterfacePortName::PortC2:
            map->pinYellow = 17;
            map->pinWhite = 16;
            map->led = 4;
            break;
        case InterfacePortName::Internal:
            map->pinYellow = 21;
            map->pinWhite = 22;
            map->led = 3;
            break;
        default:
            map->pinYellow = -1;
            map->pinWhite = -1;
            map->led = -1;
            break;
    }

    InterfacePortMap* connection = getPortMap(sensor);

    if (connection == nullptr) {
        SF_LOGE("[IFACE] configure(%s) -> connector not found", toString(sensor));
        HMI::setLed(sensor, PortStatus::Error);
        return false;
    }

    map->portMap = connection;

    switch(connection->portType) {
        case InterfacePortType::I2C:
            connection->connector.ptr.twoWire = &Wire;
            connection->configured = true;
            if (!map->available) {
                // check if sensor is available
                if (seize(sensor)) {

                    map->available = true;

                    release(sensor);
                }
            }
            break;
        case InterfacePortType::I2C1:
            connection->connector.ptr.twoWire = &Wire1;
            connection->configured = true;
            map->available = true;
            break;
        default:
            SF_LOGW("[IFACE] configure(%s) -> %s not yet implemented", toString(sensor), toString(connection->portType));
            HMI::setLed(sensor, PortStatus::Unset);
            connection->configured = false;
    }

    publishConfiguration(sensor);

    SF_LOGI("[IFACE] configure(%s) -> completed", toString(sensor));

    return configured(sensor);
}

bool configured(const InterfaceSensor sensor)
{
    const InterfacePortMap* connection = getPortMap(sensor);
    if (connection == nullptr) {
        SF_LOGW("[IFACE] configured(%s) -> connector not found", toString(sensor));
        return false;
    }

    if (!connection->configured) {
        SF_LOGW("[IFACE] configured(%s) -> port %s not configured", toString(sensor), toString(connection->portType));
        HMI::setLed(sensor, PortStatus::Error);
        return false;
    }

    return isAvailable(sensor);
}

bool isAvailable(const InterfaceSensor sensor)
{
    const InterfaceSensorMap* map = getSensor(sensor);
    if (map == nullptr) {
        SF_LOGW("[IFACE] isAvailable(%s) -> sensor not found", toString(sensor));
        return false;
    }

    return map->available;
}

bool seize(InterfaceSensor sensor) 
{
    const InterfaceSensorMap* map = getSensor(sensor);
    if (map == nullptr) {
        SF_LOGE("[IFACE] seize(%s) -> sensor not found", toString(sensor));
        HMI::setLed(sensor, PortStatus::Error);
        return false;
    }

    if (map->portMap == nullptr) {
        SF_LOGE("[IFACE] seize(%s) -> connector not configured", toString(sensor));
        HMI::setLed(sensor, PortStatus::Error);
        return false;
    }
    
    InterfacePortMap* portMap = map->portMap;
    if (portMap->lock != nullptr) {
        if (!xSemaphoreTake(portMap->lock, pdMS_TO_TICKS(500))) {
            SF_LOGE("[IFACE] seized(%s) -> failed", toString(sensor));
            HMI::setLed(sensor, PortStatus::Error);
            return false;
        }
    }

    if (map->portType == InterfacePortType::I2C) {
        if (map->portMap->connector.ptr.twoWire != nullptr) {
            if (portMap->lastPinYellow != map->pinYellow || portMap->lastPinWhite != map->pinWhite || portMap->lastClock != map->clock) {
                map->portMap->connector.ptr.twoWire->end();
                if (map->portMap->connector.ptr.twoWire->begin(map->pinYellow, map->pinWhite, map->clock)) {
                    portMap->lastPinYellow = map->pinYellow;
                    portMap->lastPinWhite = map->pinWhite;
                    portMap->lastClock = map->clock;
                } else {
                    if (portMap->lock != nullptr) {
                        xSemaphoreGive(map->portMap->lock);
                    }
                    SF_LOGE("[IFACE] seized(%s) -> unable to set pins", toString(sensor));
                    HMI::setLed(sensor, PortStatus::Error);
                    return false;
                }
            }
        }
    }

    return true;
}

void release(InterfaceSensor sensor) 
{
    const InterfaceSensorMap* map = getSensor(sensor);
    if (map == nullptr) {
        SF_LOGW("[IFACE] release(%s) -> sensor not found", toString(sensor));
        HMI::setLed(sensor, PortStatus::Error);
        return;
    }

    if (map->portMap == nullptr) {
        SF_LOGW("[IFACE] release(%s) -> sensor not configured", toString(sensor));
        HMI::setLed(sensor, PortStatus::Error);
        return;
    }
    
    if (map->portMap->lock != nullptr) {
        xSemaphoreGive(map->portMap->lock);
    }
}

}  // namespace sf_interfaces
