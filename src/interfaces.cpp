/**
 * @file interfaces.cpp
 * @brief Centralized interface definitions, enum conversions, and I2C bus helpers.
 *
 * SPDX-License-Identifier: MIT
 */

#include "interfaces.h"

#include "mqtt.h"
#include "log.h"

#include <M5Unified.h>
#include <HardwareSerial.h>
#include <Wire.h>

#include <cctype>
#include <cstddef>
#include <cstring>

namespace sf_interfaces {

namespace {

InterfacePortMap kConnections[] = {
//   type                        Lock                      configured? connector
    {InterfacePortType::I2C,     xSemaphoreCreateMutex(),  false,      {}},
    {InterfacePortType::UART,    xSemaphoreCreateMutex(),  false,      {}},
    {InterfacePortType::UART1,   xSemaphoreCreateMutex(),  false,      {}},
    {InterfacePortType::UART2,   xSemaphoreCreateMutex(),  false,      {}},
    {InterfacePortType::UART3,   xSemaphoreCreateMutex(),  false,      {}},
};

InterfacePort kPorts[] = {
//   name                          type                      yellow white
    {InterfacePortName::PortA,     InterfacePortType::I2C,   32,    33},
    {InterfacePortName::PortB1,    InterfacePortType::UART1, 35,    25},
    {InterfacePortName::PortB2,    InterfacePortType::UART2, 36,    26},
    {InterfacePortName::PortC1,    InterfacePortType::I2C,   14,    13},
    {InterfacePortName::PortC2,    InterfacePortType::UART3, 17,    16},
    {InterfacePortName::Internal,  InterfacePortType::I2C,   21,    22},
    {InterfacePortName::Bluetooth, InterfacePortType::BLE,   -1,    -1},
};

InterfaceSensorMap kSensors[] = {
//   sensor                 port                          led addr  clock     recur  avail? device name
    {InterfaceSensor::Gaz,  InterfacePortName::PortA,      0, 0x26, 400000UL, 20000, false, "M5Stack Weight I2C Unit"},
    {InterfaceSensor::Tank, InterfacePortName::PortA,      6, 0x57, 100000UL, 10000, false, "M5Stack Unit Ultrasonic I2C (RCWL-9600)"},
    {InterfaceSensor::Gps,  InterfacePortName::PortC1,     2, 0x66, 400000UL, 30000, false, "DFRobot Gravity GNSS (DFR1103)"},
    {InterfaceSensor::Lte,  InterfacePortName::PortB1,     1, 0x00, 115200UL, 10000, false, "M5Stack NB-IOT2"},
    {InterfaceSensor::Lora, InterfacePortName::PortC2,     4, 0x00, 115200UL, 10000, false, "M5Stack C6L"},
    {InterfaceSensor::Lin,  InterfacePortName::PortB2,     5, 0x00, 115200UL, 10000, false, "LIN Bus"},
    {InterfaceSensor::Imu,  InterfacePortName::Internal,  -1, 0x68, 400000UL, 10000, false, "MPU6886"},
    {InterfaceSensor::Rtc,  InterfacePortName::Internal,  -1, 0x51, 400000UL, 10000, false, "BM8563"},
    {InterfaceSensor::Ina,  InterfacePortName::Internal,  -1, 0x40, 400000UL, 10000, false, "INA3221"},
    {InterfaceSensor::Axp,  InterfacePortName::Internal,  -1, 0x34, 400000UL, 10000, false, "AXP192"},
    {InterfaceSensor::Bat,  InterfacePortName::Bluetooth, -1, 0x00,      0UL, 10000, false, "Battery"},
    {InterfaceSensor::Obd,  InterfacePortName::Bluetooth, -1, 0x00,      0UL, 10000, false, "OBD"},
};

// kSensor

const InterfaceSensorMap* findSensorConst(const InterfaceSensor sensor)
{
    const size_t count = sizeof(kSensors) / sizeof(kSensors[0]);
    for (size_t i = 0; i < count; ++i) {
        if (kSensors[i].Sensor == sensor) {
            return &kSensors[i];
        }
    }
    return nullptr;
}

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

// kPort

const InterfacePort* findPort(const InterfacePortName name)
{
    const size_t count = sizeof(kPorts) / sizeof(kPorts[0]);
    for (size_t i = 0; i < count; ++i) {
        if (kPorts[i].Name == name) {
            return &kPorts[i];
        }
    }
    return nullptr;
}

const InterfacePort* findPort(const InterfaceSensor sensor)
{
    const InterfaceSensorMap* map = findSensorConst(sensor);
    if (map == nullptr) {
        return nullptr;
    } 
    return findPort(map->Port);
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

const InterfacePortMap* findConnectionConst(const InterfacePortType type)
{
    return findConnectionMutable(type);
}

InterfacePortMap* findConnectionMutable(const InterfaceSensor sensor)
{
    const InterfacePort* port = findPort(sensor);
    if (port == nullptr) {
        return nullptr;
    } 
    return findConnectionMutable(port->Type);
}

InterfacePortMap* findConnectionConst(const InterfaceSensor sensor)
{
    return findConnectionMutable(sensor);
}

void publishConfiguration(const InterfaceSensor sensor)
{
    char addressBuf[8] = {0};
    char topicBuf[96] = {0};
    const char* topicTag = toString(sensor);
    const uint8_t deviceAddress = getAddress(sensor);

    snprintf(addressBuf, sizeof(addressBuf), "0x%02X", deviceAddress);

    const String name = toString(sensor);
    const String deviceName = getDeviceName(sensor);

    const InterfacePort* port = findPort(sensor);
    const InterfacePortType result = port ? port->Type : InterfacePortType::Unknown;
    const String type = toString(result);

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
    case InterfacePortType::I2C:          return "I2C";
    case InterfacePortType::UART:         return "UART";
    case InterfacePortType::BLE:          return "BLE";
    case InterfacePortType::Unused:       return "unused";
    default:                              return "unknown";
    }
}

const char* toString(const InterfacePortName id)
{
    switch (id) {
    case InterfacePortName::PortA:      return "a";
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
        case InterfaceSensor::Ina:   return "INA";
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
    case InterfaceSensor::Ina:    return "ina";
    case InterfaceSensor::Axp:    return "axp";
    case InterfaceSensor::Bat:    return "bat";
    case InterfaceSensor::Obd:    return "obd";
    default:                      return "unknown";
    }
}

// InterfacePortType getType(const InterfaceSensor sensor)
// {
//     const InterfacePort* port = findPort(sensor);
//     const InterfacePortType result = port ? port->Type : InterfacePortType::Unknown;
//     return result;
// }

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

    InterfacePortMap* connection = findConnectionMutable(sensor);

    if (connection == nullptr) {
        SF_LOGE("[IFACE] configure(%s) -> missing connection", toString(sensor));
        return false;
    }

    if (connection->Lock == nullptr) {
        connection->Lock = xSemaphoreCreateMutex();
        SF_LOGI("[IFACE] configure(%s) -> Lock is 0x%X", toString(sensor), connection->Lock);
    }

    if (connection->Type == InterfacePortType::I2C) {

        TwoWire* bus = &Wire;
        if (map->Port == InterfacePortName::Internal) {
            bus = &Wire1;
        }

        connection->connector.ptr.twoWire = bus;
        connection->configured = true;

        if (!seize(sensor)) {
            connection->connector.ptr.twoWire = nullptr;
            connection->configured = false;
            return false;
        }

        bus->beginTransmission(map->I2cAddress);
        const bool ack = bus->endTransmission() == 0;
        map->available = ack;

        release(sensor);

    } else if (connection->Type == InterfacePortType::UART) {

        HardwareSerial* serial = nullptr;
        if (map->Port == InterfacePortName::PortB1) {
            serial = &Serial1;
        } else {
            serial = &Serial2;
        }

        const InterfacePort* port = findPort(sensor);

        const int8_t rx = port->pinYellow;
        const int8_t tx = port->pinWhite;
        const uint32_t baud = map->Clock;

        if (rx < 0 || tx < 0 || baud == 0U) {
            SF_LOGE("[IFACE] configure(%s) -> missing UART pins/baud", toString(sensor));
            return false;
        }

        serial->begin(baud, SERIAL_8N1, rx, tx);
        connection->connector.ptr.hardwareSerial = serial;
        connection->configured = true;

        const bool ok = connection->connector.ptr.hardwareSerial != nullptr;
        map->available = ok;

    } else {
        SF_LOGW("[IFACE] configure(%s) -> %s not yet implemented", toString(sensor), toString(connection->Type));
    }

    if (!map->available) {
        SF_LOGW("[IFACE] configure(%s) -> sensor not available", toString(sensor));
        return false;
    } else {
        SF_LOGI("[IFACE] configure(%s) -> sensor ready", toString(sensor));
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

const InterfaceConnector getConnector(const InterfaceSensor sensor)
{
    const InterfacePortMap* map = findConnectionConst(sensor);
    if (map == nullptr) {
        SF_LOGW("[IFACE] getPort(%s) -> sensor not found", toString(sensor));
        return InterfaceConnector{};
    }
    return map->connector;
}

bool seize(InterfaceSensor sensor) 
{
    InterfacePortMap* connection = findConnectionMutable(sensor);
    if (connection == nullptr) {
        SF_LOGW("[IFACE] seize(%s) -> connector not found", toString(sensor));
        return false;
    }
    
    if (!connection->configured) {
        SF_LOGW("[IFACE] seize(%s) -> port %s not configured", toString(sensor), toString(connection->Type));
        return false;
    }

    if (connection->Lock == nullptr) {
        SF_LOGE("[IFACE] seize(%s) -> missing lock", toString(sensor));
        return false;
    }

    if (!xSemaphoreTake(connection->Lock, pdMS_TO_TICKS(500))) {
        SF_LOGI("[IFACE] seized(0x%X) -> failed", connection->Lock);
        return false;
    }

    if (connection->Type == InterfacePortType::I2C) {
        const InterfacePort* port = findPort(sensor);
        const int8_t white = port->pinWhite;   // SCL
        const int8_t yellow = port->pinYellow; // SDA
        
        const InterfaceSensorMap* map = findSensorConst(sensor);
        const uint32_t clockHz = map->Clock;

        TwoWire* bus = connection->connector.ptr.twoWire;
        if (bus == nullptr) {
            SF_LOGE("[IFACE] seize(%s) -> no connector", toString(connection->Type));
            release(sensor);
            return false;
        }

        bus->end();

        if (!bus->begin(yellow, white, clockHz)) {
            SF_LOGE("[IFACE] seize(%s) -> begin failed", toString(connection->Type));
            release(sensor);
            return false;
        }

        bus->setTimeOut(50);
    }

    return true;
}

void release(InterfaceSensor sensor) 
{
    InterfacePortMap* connection = findConnectionMutable(sensor);
    if (connection == nullptr) {
        SF_LOGW("[IFACE] release(%s) -> connection not found", toString(sensor));
        return;
    }
    
    if (connection->Lock == nullptr) {
        SF_LOGW("[IFACE] release(port=%s) -> missing lock", toString(connection->Type));
        return;
    }

    xSemaphoreGive(connection->Lock);
}

}  // namespace sf_interfaces
