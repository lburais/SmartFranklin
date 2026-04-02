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

namespace sf_i2c {

constexpr uint32_t kI2cInitRetryMs = 10000UL;

/** @brief Initialize the bus selected by a configured port string. */
bool i2cBeginConfiguredPort(sf_ports::PortSensor sensor, 
                            uint32_t clockHz = 100000U);

/** @brief Check whether a device is reachable on a configured port. */
bool i2cDeviceExistsOnConfiguredPort(sf_ports::PortSensor sensor,
                                     uint8_t deviceAddress,
                                     uint32_t clockHz = 100000U);

/** @brief Publish configuration metadata for one logical I2C device. */
void i2cPublishConfiguration(sf_ports::PortSensor sensor,
                             uint8_t address);

}  // namespace sf_i2c

