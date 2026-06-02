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
#include <M5Unified.h>

#include <cstddef>
#include <cstdint>
#include "freertos/semphr.h"

class TwoWire;
class SoftWire;
class HardwareSerial;
class EspSoftwareSerial;

namespace sf_interfaces {

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

enum class InterfacePortType : uint8_t {
    Wire = 0,
    Wire1,
    In_I2C,
    UART,
    UART1,
    UART2,
    UART3,
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
    Ina1,
    Ina2,
    Axp,
    Bat,
    Obd,
    Unknown,
};

struct InterfaceConnector {
    union Ptr {
        void* raw;
        TwoWire*        twoWire;
        SoftWire*       softWire;
        HardwareSerial* hardwareSerial;
        m5::I2C_Class*  inI2C;

        constexpr Ptr() : raw(nullptr) {}
    } ptr{};
};

struct InterfacePortMap {
    InterfacePortType  Type;
    SemaphoreHandle_t  Lock;
    bool               configured;
    InterfaceConnector connector;      ///< TwoWire or HardwareSerial
};

struct InterfaceSensorMap {
    InterfaceSensor   Sensor;
    InterfacePortName Port;
    InterfacePortType Type;
    int8_t            pinYellow;     ///< SDA (I2C) or RX (UART); -1 if not applicable
    int8_t            pinWhite;      ///< SCL (I2C) or TX (UART); -1 if not applicable
    int8_t            led;           ///< led number; -1 if not applicable
    uint8_t           I2cAddress;    ///< I2C device address; 0 if not applicable
    uint32_t          Clock;         ///< I2C clock frequency in Hz or UART bauds; 0 if not applicable
    uint32_t          recurrenceMs;  ///< Suggested task recurrence for this sensor
    bool              available;     ///< Last availability state set by configure(sensor)
    InterfacePortMap* portMap;
    const char*       DeviceName;
};

// global

const InterfaceConnector getConnector(InterfaceSensor sensor);
const char*              getDeviceName(InterfaceSensor sensor);
uint8_t                  getAddress(InterfaceSensor sensor);
uint32_t                 getRecurrenceMs(InterfaceSensor sensor);
int8_t                   getLed(InterfaceSensor sensor);

// strings
const char* toString(InterfacePortName name);
const char* toString(InterfacePortType type);
const char* toString(InterfaceSensor sensor, bool upper = false);

constexpr uint32_t kInterfaceInitRetryMs = 10000UL;

bool configure_all_sensors();
bool configure(InterfaceSensor sensor);
bool configured(InterfaceSensor sensor);
bool isAvailable(InterfaceSensor sensor);
void setAvailable(InterfaceSensor sensor);

bool seize(InterfaceSensor sensor);
void release(InterfaceSensor sensor);

}  // namespace sf_interfaces
