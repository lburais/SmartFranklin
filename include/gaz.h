/**
 * ============================================================================
 * Gas/Weight Sensor Module Interface - SmartFranklin IoT Device Controller
 * ============================================================================
 *
 * File:        gaz.h
 * Project:     SmartFranklin - ESP32 IoT Device Controller
 * Description: Public API for M5Stack Weight I2C unit integration supporting
 *
 * Hardware:    M5Stack Unit WEIGHT (I2C)
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

#include "i2c.h"

/**
 * @brief Gas bottle weight sensor and fill-level monitor.
 *
 * This module manages the M5Stack Weight I2C sensor for measuring propane/gas
 * bottle weight, computing fill percentage, and supporting weight-based
 * calibration from the HMI.
 *
 * **Lifecycle:**
 * - `init(isInternalRoute, i2cAddress)` - Initialize sensor on detected I2C route
 * - `process()` - Perform one measurement cycle (read, compute, publish)
 * - `tare()` - Zero the scale offset
 * - `applyCalibration(gap)` - Update load cell calibration gap
 * - `readCalibrationSample()` - Get raw sample for calibration workflow
 * - `isInitialized()` - Query initialization state
 *
 * **Thread Safety:** All operations are mutex-protected.
 *
 * **MQTT Topics:**
 * - `smartfranklin/gaz/g` - Weight in grams
 * - `smartfranklin/gaz/fill` - Fill percentage (0-100%)
 * - `smartfranklin/gaz/calibration/gap` - Current calibration gap (diagnostic)
 *
 * **Calibration Flow:**
 * 1. Empty bottle: call `tare()` to zero offset
 * 2. Full bottle: use `readCalibrationSample()` to get weight
 * 3. Apply: call `applyCalibration(computedGap)` to store in sensor
 */
class Gaz {
public:
    /**
     * @brief Initialize the weight sensor on the detected I2C route.
     *
     * Discovers and configures the M5Stack Weight I2C unit on either
     * the M5 internal I2C (Ex_I2C) or the primary Wire interface,
     * with optional PAHub channel routing.
     *
     * @param iSInternalRoute Use M5 internal I2C (Ex_I2C) if true; Wire if false
     * @param i2cAddress The I2C address of the sensor (typically 0x26)
     *
     * @return true if sensor is detected and initialized; false otherwise
     *
     * @note Applies the stored calibration gap from CONFIG.scale_cal_factor.
     */
    bool init(bool iSInternalRoute, uint8_t i2cAddress);

    /**
     * @brief Execute one complete weight measurement and publication cycle.
     *
     * Reads weight from the sensor, validates the measurement,
     * computes fill percentage, updates the shared DATA model,
     * and publishes results to MQTT.
     *
     * Also checks for pending calibration gap updates from CONFIG.
     *
     * @note Should be called periodically (typically every 1000 ms) from
     *       the dedicated sensor task context.
     */
    void process();

    /**
     * @brief Zero the scale offset (tare operation).
     *
     * Sends a tare command to the load cell, resetting the current
     * measurement to zero to compensate for container weight.
     *
     * @return true if tare succeeded; false if sensor not initialized
     *
     * @note Typically called when an empty bottle is placed on the scale.
     */
    bool tare();

    /**
     * @brief Apply a new calibration gap to the load cell.
     *
     * Updates the sensor's internal calibration gap, which compensates
     * for load cell nonlinearity and environmental factors.
     *
     * @param gap The calibration gap factor (typically near 1.0)
     *
     * @return true if calibration applied successfully; false if sensor
     *         not initialized or write failed
     *
     * @note Called during HMI calibration workflow and on process() when
     *       CONFIG.scale_cal_factor changes.
     */
    bool applyCalibration(float gap);

    /**
     * @brief Read a raw weight sample for external calibration computation.
     *
     * Performs one sensor read and returns the raw weight in grams,
     * used by the HMI calibration workflow to compute the gap factor.
     *
     * @return Current weight in grams if read succeeds; last cached weight
     *         if read fails; 0 if sensor not initialized
     *
     * @note This is intentionally blocking (not async) for calibration UX.
     */
    float readCalibrationSample();

    /**
     * @brief Query whether the weight sensor has been successfully initialized.
     *
     * @return true if init() succeeded; false if not yet initialized or init failed
     */
    bool isInitialized() const;
};

/**
 * @brief Global Gaz module singleton instance.
 *
 * This is the canonical instance used throughout SmartFranklin, typically
 * accessed by the sensor task, HMI, and calibration workflows.
 */
extern Gaz GAZ_MODULE;

/**
 * @brief Read one raw calibration sample from the gas/weight module.
 * @return Current weight sample in grams.
 */
float scale_get_raw();

/**
 * @brief Execute tare operation (zero offset capture).
 */
void scale_tare();

/**
 * @brief Apply calibration factor used by scale conversion workflow.
 * @param factor Calibration gap/factor to apply.
 */
void scale_set_cal_factor(float factor);

