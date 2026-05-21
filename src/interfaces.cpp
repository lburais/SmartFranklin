/**
 * @file interfaces.cpp
 * @brief Centralized interface definitions, enum conversions, and I2C bus helpers.
 *
 * SPDX-License-Identifier: MIT
 */

#include "interfaces.h"

#include "mqtt.h"

#include <M5Unified.h>
#include <HardwareSerial.h>
#include <Wire.h>

#include <cctype>
#include <cstddef>
#include <cstring>

namespace sf_interfaces {

namespace {

InterfacePort kPorts[] = {
//   name                      type                 ch  ye  wh  clock     Lock conf? connector
    {InterfaceName::PortA,     InterfaceType::I2C,   0, 32, 33, 100000UL, {}, false, {}},
    {InterfaceName::PortB1,    InterfaceType::UART,  1, 35, 25, 115200UL, {}, false, {}},
    {InterfaceName::PortB2,    InterfaceType::UART,  2, 36, 26, 115200UL, {}, false, {}},
    {InterfaceName::PortC1,    InterfaceType::I2C,   2, 14, 13, 400000UL, {}, false, {}},
    {InterfaceName::PortC2,    InterfaceType::UART,  3, 17, 16, 115200UL, {}, false, {}},
    {InterfaceName::Internal,  InterfaceType::I2C,   1, 21, 22, 400000UL, {}, false, {}},
    {InterfaceName::Bluetooth, InterfaceType::BLE,   0, -1, -1,      0UL, {}, false, {}},
};

InterfaceSensorMap kSensors[] = {
//   sensor                 port                      led addr  recur  avail? device name
    {InterfaceSensor::Gaz,  InterfaceName::PortA,      0, 0x26,  1000, false, "M5Stack Weight I2C Unit"},
    {InterfaceSensor::Tank, InterfaceName::PortA,      6, 0x57,  1000, false, "M5Stack Unit Ultrasonic I2C (RCWL-9600)"},
    {InterfaceSensor::Gps,  InterfaceName::PortC1,     2, 0x66, 30000, false, "DFRobot Gravity GNSS (DFR1103)"},
    {InterfaceSensor::Lte,  InterfaceName::PortB1,     1, 0x00,  1000, false, "M5Stack NB-IOT2"},
    {InterfaceSensor::Lora, InterfaceName::PortC2,     4, 0x00,  1000, false, "M5Stack C6L"},
    {InterfaceSensor::Lin,  InterfaceName::PortB2,     5, 0x00,  1000, false, "LIN Bus"},
    {InterfaceSensor::Imu,  InterfaceName::Internal,  -1, 0x68,  1000, false, "MPU6886"},
    {InterfaceSensor::Rtc,  InterfaceName::Internal,  -1, 0x51,  1000, false, "BM8563"},
    {InterfaceSensor::Ina,  InterfaceName::Internal,  -1, 0x40,  1000, false, "INA3221"},
    {InterfaceSensor::Axp,  InterfaceName::Internal,  -1, 0x34,  1000, false, "AXP192"},
    {InterfaceSensor::Bat,  InterfaceName::Bluetooth, -1, 0x00,  1000, false, "Battery"},
    {InterfaceSensor::Obd,  InterfaceName::Bluetooth, -1, 0x00,  1000, false, "OBD"},
};

inline bool seizePort(SemaphoreHandle_t lock){
    return xSemaphoreTake(lock, pdMS_TO_TICKS(500));
}

inline void releasePort(SemaphoreHandle_t lock){
    xSemaphoreGive(lock);
}

InterfacePort* findPortMutable(const InterfaceName name)
{
    const size_t count = sizeof(kPorts) / sizeof(kPorts[0]);
    for (size_t i = 0; i < count; ++i) {
        if (kPorts[i].Name == name) {
            return &kPorts[i];
        }
    }
    return nullptr;
}

const InterfacePort* findPortConst(const InterfaceName name)
{
    return findPortMutable(name);
}

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

int findPortIndex(const InterfaceName name)
{
    const size_t count = sizeof(kPorts) / sizeof(kPorts[0]);
    for (size_t i = 0; i < count; ++i) {
        if (kPorts[i].Name == name) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

InterfaceConnector getPort(const InterfaceName name)
{
    const InterfacePort* port = findPortConst(name);
    if (port == nullptr) {
        M5_LOGE("[IFACE] getPort(%s) -> null", toString(name));
        return InterfaceConnector{};
    }
    return port->connector;
}

bool configured(const InterfaceName name)
{
    const InterfacePort* port = findPortConst(name);
    const bool result = (port != nullptr) && port->configured;
    return result;
}

void publishConfiguration(const InterfaceSensor sensor)
{
    char addressBuf[8] = {0};
    char topicBuf[96] = {0};
    const char* topicTag = toString(sensor);
    const uint8_t deviceAddress = getAddress(sensor);

    snprintf(addressBuf, sizeof(addressBuf), "0x%02X", deviceAddress);

    const String name = toString(getName(sensor));
    const String type = toString(getType(sensor));
    const String deviceName = getDeviceName(sensor);

    snprintf(topicBuf, sizeof(topicBuf), "smartfranklin/system/interface/%s/address", topicTag);
    sf_mqtt::publish(topicBuf, addressBuf, 1, true);
    snprintf(topicBuf, sizeof(topicBuf), "smartfranklin/system/interface/%s/port", topicTag);
    sf_mqtt::publish(topicBuf, name.c_str(), 1, true);
    snprintf(topicBuf, sizeof(topicBuf), "smartfranklin/system/interface/%s/type", topicTag);
    sf_mqtt::publish(topicBuf, type.c_str(), 1, true);
    snprintf(topicBuf, sizeof(topicBuf), "smartfranklin/system/interface/%s/device_name", topicTag);
    sf_mqtt::publish(topicBuf, deviceName.c_str(), 1, true);
}

bool configureI2c(InterfacePort& port)
{
    const int8_t sda = port.pinYellow;
    const int8_t scl = port.pinWhite;
    const uint32_t clockHz = port.Clock;
    const uint8_t channel = port.Channel;

    if (sda < 0 || scl < 0 || clockHz == 0U) {
        M5_LOGE("[IFACE] %s: missing I2C pins/clock", toString(port.Name));
        return false;
    }
    
    TwoWire* bus = nullptr;
    if (channel == 0) {
        bus = &Wire;
    }
    else if (channel == 1) {
        bus = &Wire1;
    }
    else {
        M5_LOGE("[IFACE] %s: missing soft I2C implementation", toString(port.Name));
        return false;
    }

    if (seizePort(port.Lock)) {
        bus->end();        
        if (!bus->begin(sda, scl, clockHz)) {
            M5_LOGE("[IFACE] %s: begin failed", toString(port.Name));
            releasePort(port.Lock);
            return false;
        } else {
            M5_LOGI("[IFACE] %s: begin channel %d", toString(port.Name), channel);
        }
        releasePort(port.Lock);
    }

    port.connector.ptr.twoWire = bus;

    return true;
}

bool configureUart(InterfacePort& port)
{
    const InterfaceName name = port.Name;
    HardwareSerial* serial = nullptr;
    if (name == InterfaceName::PortB1) {
        serial = &Serial1;
    } else {
        serial = &Serial2;
    }

    const int8_t rx = port.pinYellow;
    const int8_t tx = port.pinWhite;
    const uint32_t baud = port.Clock;
    if (rx < 0 || tx < 0 || baud == 0U) {
        M5_LOGE("[IFACE] %s: missing UART pins/baud", toString(name));
        return false;
    }

    serial->begin(baud, SERIAL_8N1, rx, tx);
    port.connector.ptr.hardwareSerial = serial;
    return true;
}

bool configure(const InterfaceName name)
{
    InterfacePort* port = findPortMutable(name);
    if (port == nullptr) {
        M5_LOGE("[IFACE] configure(port=%s) -> not found", toString(name));
        return false;
    }
    if (port->configured) {
        M5_LOGW("[IFACE] configure(port=%s) -> already configured", toString(name));
        return true;
    }

    port->Lock = xSemaphoreCreateMutex();

    bool ok = false;
    port->connector.ptr.raw = nullptr; //clearConnector(*port);
    switch (port->Type) {
    case InterfaceType::I2C:
        ok = configureI2c(*port);
        break;
    case InterfaceType::UART:
        ok = configureUart(*port);
        break;
    case InterfaceType::I2CSoftware:
    case InterfaceType::UARTSoftware:
        M5_LOGW("[IFACE] %s software connector not available", toString(name));
        ok = false;
        break;
    case InterfaceType::BLE:
    case InterfaceType::Unused:
        ok = true;
        break;
    default:
        ok = false;
        break;
    }

    port->configured = ok;
    return ok;
}

}  // namespace

const char* toString(const InterfaceType type)
{
    switch (type) {
    case InterfaceType::I2C:          return "I2C";
    case InterfaceType::I2CSoftware:  return "I2CSoftware";
    case InterfaceType::UART:         return "UART";
    case InterfaceType::UARTSoftware: return "UARTSoftware";
    case InterfaceType::BLE:          return "BLE";
    case InterfaceType::Unused:       return "unused";
    default:                          return "unknown";
    }
}

const char* toString(const InterfaceName id)
{
    switch (id) {
    case InterfaceName::PortA:     return "a";
    case InterfaceName::PortB1:     return "b1";
    case InterfaceName::PortB2:     return "b2";
    case InterfaceName::PortC1:     return "c1";
    case InterfaceName::PortC2:     return "c2";
    case InterfaceName::Internal:   return "internal";
    case InterfaceName::Bluetooth:  return "bluetooth";
    case InterfaceName::Unknown:    return "unknown";
    default:                        return "unknown";
    }
}

const char* toString(const InterfaceSensor sensor)
{
    switch (sensor) {
    case InterfaceSensor::None:  return "";
    case InterfaceSensor::Gaz:   return "gaz";
    case InterfaceSensor::Tank:  return "tank";
    case InterfaceSensor::Gps:   return "gps";
    case InterfaceSensor::Lte:   return "lte";
    case InterfaceSensor::Lora:  return "lora";
    case InterfaceSensor::Lin:   return "lin";
    case InterfaceSensor::Imu:   return "imu";
    case InterfaceSensor::Rtc:   return "rtc";
    case InterfaceSensor::Ina:   return "ina";
    case InterfaceSensor::Axp:   return "axp";
    case InterfaceSensor::Bat:   return "bat";
    case InterfaceSensor::Obd:   return "obd";
    default:                     return "unknown";
    }
}

String toUpperString(const InterfaceSensor sensor)
{
    const char* source = sf_interfaces::toString(sensor);
    char buffer[32] = {0};

    if (source != nullptr) {
        strncpy(buffer, source, sizeof(buffer) - 1);
    }

    for (char* p = buffer; *p != '\0'; ++p) {
        *p = static_cast<char>(toupper(static_cast<unsigned char>(*p)));
    }

    return String(buffer);
}


InterfaceName getName(const InterfaceSensor sensor)
{
    const InterfaceSensorMap* iface = findSensorConst(sensor);
    const InterfaceName result = iface ? iface->Port : InterfaceName::Unknown;
    return result;
}

InterfaceType getType(const InterfaceSensor sensor)
{
    const InterfacePort* port = findPortConst(getName(sensor));
    const InterfaceType result = port ? port->Type : InterfaceType::Unknown;
    return result;
}

String getDeviceName(const InterfaceSensor sensor)
{
    const InterfaceSensorMap* iface = findSensorConst(sensor);
    const String result = iface ? iface->DeviceName : "Unknown";
    return result;
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

uint32_t getClock(const InterfaceSensor sensor)
{
    const InterfacePort* port = findPortConst(getName(sensor));
    const uint32_t result = port ? port->Clock : 0;
    return result;
}

int8_t getSDA(const InterfaceSensor sensor)
{
    const InterfacePort* port = findPortConst(getName(sensor));
    const int8_t result = (port && getType(sensor) == InterfaceType::I2C) ? port->pinYellow : -1;
    return result;
}

int8_t getSCL(const InterfaceSensor sensor)
{
    const InterfacePort* port = findPortConst(getName(sensor));
    const int8_t result = (port && getType(sensor) == InterfaceType::I2C) ? port->pinWhite : -1;
    return result;
}

int8_t getRX(const InterfaceSensor sensor)
{
    const InterfacePort* port = findPortConst(getName(sensor));
    const InterfaceType type = getType(sensor);
    const int8_t result = (port && (type == InterfaceType::UART || type == InterfaceType::UARTSoftware)) ? port->pinYellow : -1;
    return result;
}

int8_t getTX(const InterfaceSensor sensor)
{
    const InterfacePort* port = findPortConst(getName(sensor));
    const InterfaceType type = getType(sensor);
    const int8_t result = (port && (type == InterfaceType::UART || type == InterfaceType::UARTSoftware)) ? port->pinWhite : -1;
    return result;
}

bool configure_all_sensors()
{
    bool allOk = true;
    const size_t count = sizeof(kSensors) / sizeof(kSensors[0]);
    for (size_t i = 0; i < count; ++i) {
        allOk = configure(kSensors[i].Sensor) && allOk;
    }
    return allOk;
}

bool configure(const InterfaceSensor sensor)
{
    InterfaceSensorMap* map = findSensorMutable(sensor);
    if (map == nullptr) {
        M5_LOGW("[IFACE] configure(%s) -> sensor not found", toString(sensor));
        return false;
    }

    if (!configure(map->Port)) {
        M5_LOGW("[IFACE] configure(%s) -> port %s failed", toString(sensor), toString(map->Port));
        return false;
    }

    const InterfacePort* port = findPortConst(map->Port);

    if (port->Type == InterfaceType::I2C) {

        TwoWire* bus = port->connector.ptr.twoWire;
        if (bus == nullptr || map->I2cAddress == 0U) {
            M5_LOGE("[IFACE] configure(%s) -> no bus or i2c address", toString(sensor));
            return false;
        }

        if (seize(sensor)) {
            bus->beginTransmission(map->I2cAddress);
            const bool ack = bus->endTransmission() == 0;
            release(sensor);
            map->available = ack;
        } else {
            map->available = false;
        }

    }

    if (port->Type == InterfaceType::UART) {
        const bool ok = port->connector.ptr.hardwareSerial != nullptr;
        map->available = ok;
    }

    if (!map->available) {
        M5_LOGW("[IFACE] configure(%s) -> sensor not available", toString(sensor));
        return false;
    } else {
        M5_LOGI("[IFACE] configure(%s) -> sensor ready", toString(sensor));
    }

    publishConfiguration(sensor);

    return true;
}

bool configured(const InterfaceSensor sensor)
{
    const InterfaceSensorMap* map = findSensorConst(sensor);
    if (map == nullptr) {
        M5_LOGW("[IFACE] configured(%s) -> sensor not found", toString(sensor));
        return false;
    }

    if (!configured(map->Port)) {
        M5_LOGW("[IFACE] configured(%s) -> port %s not configured", toString(sensor), toString(map->Port));
        return false;
    }

    if (!map->available) {
        M5_LOGW("[IFACE] configured(%s) -> sensor not available", toString(sensor));
        return false;
    }

    return true;
}

InterfaceConnector getPort(const InterfaceSensor sensor)
{
    const InterfaceSensorMap* map = findSensorConst(sensor);
    if (map == nullptr) {
        M5_LOGW("[IFACE] getPort(%s) -> sensor not found", toString(sensor));
        return InterfaceConnector{};
    }
    return getPort(map->Port);
}

bool seize(InterfaceSensor sensor){
    const InterfaceSensorMap* map = findSensorConst(sensor);
    if (map == nullptr) {
        M5_LOGW("[IFACE] seize(%s) -> sensor not found", toString(sensor));
        return false;
    }
    
    InterfacePort* port = findPortMutable(map->Port);
    if (port == nullptr) {
        M5_LOGE("[IFACE] seize(port=%s) -> not found", toString(map->Port));
        return false;
    }

    if (!port->configured) {
        M5_LOGW("[IFACE] seize(port=%s) -> not configured", toString(map->Port));
        return false;
    }

    return seizePort(port->Lock);
}

void release(InterfaceSensor sensor){
    const InterfaceSensorMap* map = findSensorConst(sensor);
    if (map == nullptr) {
        M5_LOGW("[IFACE] release(%s) -> sensor not found", toString(sensor));
    }
    
    InterfacePort* port = findPortMutable(map->Port);
    if (port == nullptr) {
        M5_LOGE("[IFACE] release(port=%s) -> not found", toString(map->Port));
    }
    
    if (!port->configured) {
        M5_LOGW("[IFACE] release(port=%s) -> not configured", toString(map->Port));
    }

    releasePort(port->Lock);
}

}  // namespace sf_interfaces
