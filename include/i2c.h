/**
 * @file i2c.h
 * @brief I2C direct bus binding helpers based on configured ports.
 *
 * SmartFranklin uses this module to initialize a configured port,
 * probe peripherals, and publish metadata associated with that port.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <Arduino.h>

#include "ports.h"

constexpr uint32_t kI2cInitRetryMs = 10000UL;

/** @brief Initialize Port A / Wire bus according to runtime settings. */
void i2cBeginPortA(uint32_t clockHz = 100000U);

/** @brief Initialize the bus selected by a configured port string. */
bool i2cBeginConfiguredPort(const String& configuredPort, const char* label, uint32_t clockHz = 100000U);

/** @brief Check whether a device is reachable on a configured port. */
bool i2cDeviceExistsOnConfiguredPort(uint8_t deviceAddress,
                                     const String& configuredPort,
                                     const char* label,
                                     uint32_t clockHz = 100000U);

/** @brief Return normalized configured port name: a1, a2, b1, b2, c1, c2. */
String i2cPortName(const String& configuredPort);

/** @brief Return configured type metadata associated with a port. */
String i2cPortType(const String& configuredPort);

/** @brief Return configured sensor metadata associated with a port. */
String i2cPortSensor(const String& configuredPort);

/**
 * @brief Resolve the configured I2C port matching one sensor tag.
 *
 * Finds the first port where `port_*_type` is `I2C` and `port_*_sensor` matches.
 */
String i2cConfiguredPortForSensor(sf_ports::PortSensor sensor,
                                  const char* label = nullptr);

/** @brief Return configured device name metadata associated with a port. */
String i2cPortDeviceName(const String& configuredPort);

/** @brief Publish configuration metadata for one logical I2C device. */
void i2cPublishConfiguration(const char* tag, const String& configuredPort, uint8_t address);
