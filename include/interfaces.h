/**
 * @file interfaces.h
 * @brief Centralized interface (port/sensor) definitions and interface helpers.
 *
 * Provides hardware interface definitions and connector helpers in a single
 * namespace.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <Arduino.h>

#include <cstddef>
#include <cstdint>
#include "freertos/semphr.h"

class TwoWire;
class SoftWire;
class HardwareSerial;
class EspSoftwareSerial;

namespace sf_interfaces {

enum class InterfaceName : uint8_t {
    PortA = 0,
    PortB1,
    PortB2,
    PortC1,
    PortC2,
    Internal,
    Bluetooth,
    Unknown,
};

enum class InterfaceType : uint8_t {
    I2C = 0,
    I2CSoftware,
    UART,
    UARTSoftware,
    BLE,
    Unused,
    Unknown,
};

enum class InterfaceSensor : uint8_t {
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

struct InterfaceConnector {
    union Ptr {
        void* raw;
        TwoWire* twoWire;
        SoftWire* softWire;
        HardwareSerial* hardwareSerial;
        EspSoftwareSerial* softwareSerial;

        constexpr Ptr() : raw(nullptr) {}
    } ptr{};
};

struct InterfacePort {
    InterfaceName      Name;
    InterfaceType      Type;
    uint8_t            Channel;
    int8_t             pinYellow;  ///< SDA (I2C) or RX (UART); -1 if not applicable
    int8_t             pinWhite;   ///< SCL (I2C) or TX (UART); -1 if not applicable
    uint32_t           Clock;      ///< I2C clock frequency in Hz or UART bauds; 0 if not applicable
    SemaphoreHandle_t  Lock;
    bool               configured;
    InterfaceConnector connector;
};

struct InterfaceSensorMap {
    InterfaceSensor Sensor;
    InterfaceName   Port;
    int8_t         led;           ///< led number; -1 if not applicable
    uint8_t         I2cAddress;    ///< I2C device address; 0 if not applicable
    uint32_t        recurrenceMs;  ///< Suggested task recurrence for this sensor
    bool            available;     ///< Last availability state set by configure(sensor)
    const char*     DeviceName;
};

InterfaceName getName(InterfaceSensor sensor);
InterfaceType getType(InterfaceSensor sensor);
String getDeviceName(InterfaceSensor sensor);
uint8_t getAddress(InterfaceSensor sensor);
uint32_t getRecurrenceMs(InterfaceSensor sensor);
int8_t getLed(InterfaceSensor sensor);

uint32_t getClock(InterfaceSensor sensor);  ///< Returns I2C clock frequency in Hz, or 0 if not I2C
int8_t getSDA(InterfaceSensor sensor);      ///< Returns pinYellow if I2C, else -1
int8_t getSCL(InterfaceSensor sensor);      ///< Returns pinWhite  if I2C, else -1
int8_t getRX(InterfaceSensor sensor);       ///< Returns pinYellow if UART, else -1
int8_t getTX(InterfaceSensor sensor);       ///< Returns pinWhite  if UART, else -1

const char* toString(InterfaceName name);
const char* toString(InterfaceType type);
const char* toString(InterfaceSensor sensor);
String toUpperString(InterfaceSensor sensor);

constexpr uint32_t kInterfaceInitRetryMs = 10000UL;

bool configure_all_sensors();
bool configure(InterfaceSensor sensor);
bool configured(InterfaceSensor sensor);

bool seize(InterfaceSensor sensor);
void release(InterfaceSensor sensor);

InterfaceConnector getPort(InterfaceSensor sensor);

}  // namespace sf_interfaces
