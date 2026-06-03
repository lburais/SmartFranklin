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

#include <M5Unified.h>
#include <HardwareSerial.h>
#include <Wire.h>

#include <cstddef>

namespace sf_interfaces {

namespace {

InterfacePortMap kConnections[] = {
//   type                      Lock                      configured? connector
    {InterfacePortType::Wire,   xSemaphoreCreateMutex(),  false,      {}},
    {InterfacePortType::Wire1,                  nullptr,  false,      {}},
    {InterfacePortType::In_I2C,                 nullptr,  false,      {}},
    {InterfacePortType::UART,                   nullptr,  false,      {}},
    {InterfacePortType::UART1,                  nullptr,  false,      {}},
    {InterfacePortType::UART2,                  nullptr,  false,      {}},
    {InterfacePortType::UART3,                  nullptr,  false,      {}},
};

InterfaceSensorMap kSensors[] = {
//   sensor                 port                          type                       yellow white led  addr  clock     recur  avail? map device name
    {InterfaceSensor::Gaz,  InterfacePortName::PortA1,     InterfacePortType::Wire,   -1,    -1,    0,  0x26, 400000UL, 10000, false, nullptr, "M5Stack Weight I2C Unit"},
    {InterfaceSensor::Tank, InterfacePortName::PortA2,     InterfacePortType::Wire,   -1,    -1,    6,  0x57, 100000UL, 15000, false, nullptr, "M5Stack Unit Ultrasonic I2C (RCWL-9600)"},
    {InterfaceSensor::Gps,  InterfacePortName::PortC1,    InterfacePortType::Wire1,  -1,    -1,    2,  0x66, 400000UL, 30000, false, nullptr, "DFRobot Gravity GNSS (DFR1103)"},
    {InterfaceSensor::Lte,  InterfacePortName::PortB1,    InterfacePortType::UART1,  -1,    -1,    1,  0x00, 115200UL, 10000, false, nullptr, "M5Stack NB-IOT2"},
    {InterfaceSensor::Lora, InterfacePortName::PortC2,    InterfacePortType::UART3,  -1,    -1,    4,  0x00, 115200UL, 10000, false, nullptr, "M5Stack C6L"},
    {InterfaceSensor::Lin,  InterfacePortName::PortB2,    InterfacePortType::UART2,  -1,    -1,    5,  0x00, 115200UL, 10000, false, nullptr, "LIN Bus"},
    {InterfaceSensor::Imu,  InterfacePortName::Internal,  InterfacePortType::In_I2C, -1,    -1,   -1,  0x68, 400000UL, 10000, false, nullptr, "MPU6886 6-axis IMU"},
    {InterfaceSensor::Rtc,  InterfacePortName::Internal,  InterfacePortType::In_I2C, -1,    -1,   -1,  0x51, 400000UL, 60000, false, nullptr, "BM8563 RTC"},
    {InterfaceSensor::Ina1, InterfacePortName::Internal,  InterfacePortType::In_I2C, -1,    -1,   -1,  0x40, 400000UL, 60000, false, nullptr, "INA3221 3 channel voltage and current sensor"},
    {InterfaceSensor::Ina2, InterfacePortName::Internal,  InterfacePortType::In_I2C, -1,    -1,   -1,  0x41, 400000UL, 60000, false, nullptr, "INA3221 3 channel voltage and current sensor"},
    {InterfaceSensor::Axp,  InterfacePortName::Internal,  InterfacePortType::In_I2C, -1,    -1,    3,  0x34, 400000UL, 30000, false, nullptr, "AXP192 power management"},
    {InterfaceSensor::Bat,  InterfacePortName::Bluetooth, InterfacePortType::BLE,    -1,    -1,   -1,  0x00,      0UL, 10000, false, nullptr, "Battery"},
    {InterfaceSensor::Obd,  InterfacePortName::Bluetooth, InterfacePortType::BLE,    -1,    -1,   -1,  0x00,      0UL, 10000, false, nullptr, "OBD"},
};

// kSensor

InterfaceSensorMap* findSensorMutable(const InterfaceSensor sensor)
{
    const size_t count = sizeof(kSensors) / sizeof(kSensors[0]);
    for (size_t i = 0; i < count; ++i) {
        if (kSensors[i].Sensor == sensor) {
            return &kSensors[i];
        }
    }
    return nullptr;
}

const InterfaceSensorMap* findSensorConst(const InterfaceSensor sensor)
{
    return findSensorMutable(sensor);
}

// kConnection

InterfacePortMap* findConnectionMutable(const InterfacePortType type)
{
    const size_t count = sizeof(kConnections) / sizeof(kConnections[0]);
    for (size_t i = 0; i < count; ++i) {
        if (kConnections[i].Type == type) {
            return &kConnections[i];
        }
    }
    return nullptr;
}

InterfacePortMap* findConnectionMutable(const InterfaceSensor sensor)
{
    const InterfaceSensorMap* map = findSensorMutable(sensor);
    if (map == nullptr) {
        return nullptr;
    } 
    return findConnectionMutable(map->Type);
}

InterfacePortMap* findConnectionConst(const InterfaceSensor sensor)
{
    return findConnectionMutable(sensor);
}

void publishConfiguration(const InterfaceSensor sensor)
{
    const InterfaceSensorMap* map = findSensorConst(sensor);

    char topicBuf[96] = {0};
    const char* topicTag = toString(sensor);

    char addressBuf[8] = {0};
    snprintf(addressBuf, sizeof(addressBuf), "0x%02X", map->I2cAddress);

    const String name = toString(sensor);

    const String deviceName = map->DeviceName;

    const String type = toString(map->Type);

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

const char* toString(const InterfacePortType type)
{
    switch (type) {
    case InterfacePortType::Wire:         return "I2C";
    case InterfacePortType::Wire1:        return "I2C";
    case InterfacePortType::In_I2C:       return "I2C";
    case InterfacePortType::UART:         return "UART";
    case InterfacePortType::UART1:        return "UART";
    case InterfacePortType::UART2:        return "UART";
    case InterfacePortType::UART3:        return "UART";
    case InterfacePortType::BLE:          return "BLE";
    case InterfacePortType::Unused:       return "unused";
    default:                              return "unknown";
    }
}

const char* toString(const InterfacePortName id)
{
    switch (id) {
    case InterfacePortName::PortA1:     return "a1";
    case InterfacePortName::PortA2:     return "a2";
    case InterfacePortName::PortB1:     return "b1";
    case InterfacePortName::PortB2:     return "b2";
    case InterfacePortName::PortC1:     return "c1";
    case InterfacePortName::PortC2:     return "c2";
    case InterfacePortName::Internal:   return "internal";
    case InterfacePortName::Bluetooth:  return "bluetooth";
    case InterfacePortName::Unknown:    return "unknown";
    default:                            return "unknown";
    }
}

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

const char* getDeviceName(const InterfaceSensor sensor)
{
    const InterfaceSensorMap* iface = findSensorConst(sensor);
    return (iface && iface->DeviceName) ? iface->DeviceName : "Unknown";
}

uint8_t getAddress(const InterfaceSensor sensor)
{
    const InterfaceSensorMap* iface = findSensorConst(sensor);
    const uint8_t result = iface ? iface->I2cAddress : 0;
    return result;
}

int8_t getLed(const InterfaceSensor sensor)
{
    const InterfaceSensorMap* iface = findSensorConst(sensor);
    const int8_t result = iface ? iface->led : -1;
    return result;
}

uint32_t getRecurrenceMs(const InterfaceSensor sensor)
{
    const InterfaceSensorMap* iface = findSensorConst(sensor);
    const uint32_t result = iface ? iface->recurrenceMs : 0;
    return result;
}

bool configure_all_sensors()
{
    bool allOk = true;
    const size_t count = sizeof(kSensors) / sizeof(kSensors[0]);
    for (size_t i = 0; i < count; ++i) {
        SF_LOGI("[IFACE] configure_all_sensors -> %s", toString(kSensors[i].Sensor));
        allOk = configure(kSensors[i].Sensor) && allOk;
    }
    return allOk;
}

bool configure(const InterfaceSensor sensor)
{
    InterfaceSensorMap* map = findSensorMutable(sensor);
    if (map == nullptr) {
        SF_LOGW("[IFACE] configure(%s) -> sensor not found", toString(sensor));
        return false;
    }

    map->available = false;

    switch (map->Port) {
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

    InterfacePortMap* connection = findConnectionMutable(sensor);

    if (connection == nullptr) {
        SF_LOGE("[IFACE] configure(%s) -> connection %s not found", toString(sensor), toString(map->Type));
        HMI::setLed(sensor, PortStatus::Error);
        return false;
    }

    map->portMap = connection;

    switch(connection->Type) {
        case InterfacePortType::Wire:
            if (!connection->configured) {
                if(Wire.begin(map->pinYellow, map->pinWhite, map->Clock)) {
                    connection->connector.ptr.twoWire = &Wire;
                    connection->configured = true;
                }
            }
            break;
        case InterfacePortType::Wire1:
            if (!connection->configured) {
                if(Wire1.begin(map->pinYellow, map->pinWhite, map->Clock)) {
                    connection->connector.ptr.twoWire = &Wire1;
                    connection->configured = true;
                }
            }
            break;
        case InterfacePortType::In_I2C:
            connection->connector.ptr.inI2C = &m5::In_I2C;
            connection->configured = true;
            break;
        case InterfacePortType::UART:
            SF_LOGW("[IFACE] configure(%s) -> %s not yet implemented", toString(sensor), toString(connection->Type));
            HMI::setLed(sensor, PortStatus::Unset);
            connection->configured = false;
            return false;
        default:
            SF_LOGW("[IFACE] configure(%s) -> %s not yet implemented", toString(sensor), toString(connection->Type));
            HMI::setLed(sensor, PortStatus::Unset);
            connection->configured = false;
            return false;
    }

    publishConfiguration(sensor);

    return true;
}

bool configured(const InterfaceSensor sensor)
{
    const InterfacePortMap* map = findConnectionConst(sensor);
    if (map == nullptr) {
        SF_LOGW("[IFACE] configured(%s) -> connector not found", toString(sensor));
        return false;
    }

    if (!map->configured) {
        SF_LOGW("[IFACE] configured(%s) -> port %s not configured", toString(sensor), toString(map->Type));
        HMI::setLed(sensor, PortStatus::Error);
        return false;
    }

    return isAvailable(sensor);
}

bool isAvailable(const InterfaceSensor sensor)
{
    const InterfaceSensorMap* map = findSensorConst(sensor);
    if (map == nullptr) {
        SF_LOGW("[IFACE] configured(%s) -> sensor not found", toString(sensor));
        return false;
    }

    return map->available;
}

void setAvailable(const InterfaceSensor sensor)
{
    InterfaceSensorMap* map = findSensorMutable(sensor);
    if (map == nullptr) {
        SF_LOGW("[IFACE] configured(%s) -> sensor not found", toString(sensor));
    } else {
        map->available = true;
    }
}

const InterfaceConnector getConnector(const InterfaceSensor sensor)
{
    const InterfacePortMap* map = findConnectionConst(sensor);
    if (map == nullptr) {
        SF_LOGW("[IFACE] getConnector(%s) -> connection not found", toString(sensor));
        HMI::setLed(sensor, PortStatus::Error);
        return InterfaceConnector{};
    }
    return map->connector;
}

bool seize(InterfaceSensor sensor) 
{
    const InterfaceSensorMap* map = findSensorConst(sensor);
    if (map == nullptr) {
        SF_LOGE("[IFACE] seize(%s) -> sensor not found", toString(sensor));
        HMI::setLed(sensor, PortStatus::Error);
        return false;
    }

    if (map->portMap == nullptr) {
        SF_LOGE("[IFACE] seize(%s) -> sensor not configured", toString(sensor));
        HMI::setLed(sensor, PortStatus::Error);
        return false;
    }
    
    if (map->portMap->Lock != nullptr) {
        if (!xSemaphoreTake(map->portMap->Lock, pdMS_TO_TICKS(500))) {
            SF_LOGE("[IFACE] seized(%s) -> failed", toString(sensor));
            HMI::setLed(sensor, PortStatus::Error);
            return false;
        }
    }

    if (map->Type == InterfacePortType::Wire || map->Type == InterfacePortType::Wire1) {
        if (map->portMap->connector.ptr.twoWire->getClock() != map->Clock) {
            map->portMap->connector.ptr.twoWire->setClock(map->Clock);
        }
    }

    return true;
}

void release(InterfaceSensor sensor) 
{
    const InterfaceSensorMap* map = findSensorConst(sensor);
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
    
    if (map->portMap->Lock != nullptr) {
        xSemaphoreGive(map->portMap->Lock);
    }
}

}  // namespace sf_interfaces
