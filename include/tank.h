/**
 * ============================================================================
 * Tank Module Interface - SmartFranklin IoT Device Controller
 * ============================================================================
 *
 * File:        tank.h
 * Project:     SmartFranklin - ESP32 IoT Water Tank Monitor
 * Description: Public API for the M5Stack Unit Ultrasonic I2C water level
 *
 * Hardware:    M5Stack Unit Ultrasonic I2C (RCWL-9600)
 *
 * Features:
 *
 * ============================================================================
 * MIT License - Copyright (c) 2026 Laurent Burais
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * ============================================================================
 */

#pragma once

#include <Arduino.h>

#include "ports.h"

/**
 * @brief Tank ultrasonic water level sensor runtime.
 *
 * This module manages the M5Stack Unit Ultrasonic I2C sensor for measuring
 * water tank depth and computing fill percentage.
 *
 * **Lifecycle:**
 * - `init(configuredPort, i2cAddress)` - Initialize sensor on configured port
 * - `process()` - Perform one measurement cycle (read, compute, publish)
 * - `isInitialized()` - Query initialization state
 *
 * **Thread Safety:** All state is protected by internal mutex.
 *
 * **MQTT Topics:**
 * - `smartfranklin/tank/mm` - Raw distance reading in millimeters
 * - `smartfranklin/tank/fill` - Computed fill percentage (0-100%)
 */
class Tank {
public:
    /**
    * @brief Initialize the tank ultrasonic sensor on the configured port.
     *
    * Activates the RCWL-9600 sensor on the resolved port from configuration.
     *
    * @param configuredPort Normalized port name such as internal, a1, a2, b1...
     * @param i2cAddress The I2C address of the sensor (typically 0x57)
     *
     * @return true if sensor is detected and initialized; false otherwise
     *
     * @note Should be called once during system boot after I2C is ready.
     *       Multiple calls reset the sensor state.
     */
    bool init(const String& configuredPort, uint8_t i2cAddress);

    /**
     * @brief Execute one complete measurement and publication cycle.
     *
     * Reads raw distance from the sensor, validates the measurement,
     * computes fill percentage, updates the shared DATA model,
     * and publishes results to MQTT.
     *
     * @note Should be called periodically (typically every 1000 ms) from
     *       the dedicated sensor task context.
     *
    * @note Logs warnings on measurement failures and exits early when data is invalid.
     */
    void process();

    /**
     * @brief Query whether the tank sensor has been successfully initialized.
     *
     * @return true if init() succeeded; false if not yet initialized or init failed
     */
    bool isInitialized() const;
};

/**
 * @brief Global Tank module singleton instance.
 *
 * This is the canonical instance used throughout SmartFranklin, typically
 * accessed by the sensor task and HMI.
 */
extern Tank TANK_MODULE;


