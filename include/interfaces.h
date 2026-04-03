/**
 * @file interfaces.h
 * @brief Centralized interface (port/sensor) definitions and I2C bus helpers.
 *
 * Unifies the sf_interfaces namespace (hardware interface definitions) and the
 * sf_i2c namespace (I2C bus helpers) into a single header.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <Arduino.h>

#include <cstddef>
#include <cstdint>

namespace sf_interfaces {

enum class InterfaceName : uint8_t {
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

enum class InterfaceType : uint8_t {
    I2C = 0,
    UART,
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

struct InterfaceDefinition {
    InterfaceName Name;
    InterfaceType Type;
    int8_t        pinYellow;  ///< SDA (I2C) or RX (UART); -1 if not applicable
    int8_t        pinWhite;   ///< SCL (I2C) or TX (UART); -1 if not applicable
    InterfaceSensor Sensor;
    const char* DeviceName;
};

const InterfaceDefinition* allInterfaceDefinitions(size_t& count);
const InterfaceDefinition* findBySensor(InterfaceSensor sensor);

InterfaceName getName(InterfaceSensor sensor);
InterfaceType getType(InterfaceSensor sensor);
String getDeviceName(InterfaceSensor sensor);

int8_t getSDA(InterfaceSensor sensor);  ///< Returns pinYellow if I2C, else -1
int8_t getSCL(InterfaceSensor sensor);  ///< Returns pinWhite  if I2C, else -1
int8_t getRX(InterfaceSensor sensor);   ///< Returns pinYellow if UART, else -1
int8_t getTX(InterfaceSensor sensor);   ///< Returns pinWhite  if UART, else -1

const char* toString(InterfaceName name);
const char* toString(InterfaceType type);
const char* toString(InterfaceSensor sensor);

}  // namespace sf_interfaces

namespace sf_i2c {

constexpr uint32_t kI2cInitRetryMs = 10000UL;

/** @brief Initialize the bus selected by a configured interface sensor. */
bool i2cBeginConfiguredPort(sf_interfaces::InterfaceSensor sensor,
                            uint32_t clockHz = 400000U);

/** @brief Check whether a device is reachable on a configured interface. */
bool i2cDeviceExistsOnConfiguredPort(sf_interfaces::InterfaceSensor sensor,
                                     uint8_t deviceAddress,
                                     uint32_t clockHz = 400000U);

/** @brief Publish configuration metadata for one logical I2C device. */
void i2cPublishConfiguration(sf_interfaces::InterfaceSensor sensor,
                             uint8_t address);

}  // namespace sf_i2c
