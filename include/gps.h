/**
 * @file gps.h
 * @brief Public interface for GNSS acquisition in SmartFranklin.
 *
 * This module abstracts GPS sensor initialization and periodic processing.
 * It exposes source selection and diagnostics helpers so upper layers can
 * report detected hardware and runtime state.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <Arduino.h>

#include "interfaces.h"

class GPS {
public:
    const uint8_t deviceAddress = 0x66;

    /**
    * @brief Initialize GPS runtime with the selected source and configured port.
     * @param source GNSS source implementation to initialize.
    * @param configuredPort Normalized configured port name (a1, a2, b1, b2, c1, c2).
     * @param i2cAddress I2C address of the GPS unit when applicable.
     * @return True when initialization succeeds.
     */
    bool init();

    /**
     * @brief Execute one periodic GPS processing cycle.
     *
     * Intended to be called from a periodic task. Reads samples and publishes
     * updated position/time into the shared runtime model.
     */
    void process();

    /**
     * @brief Check whether GPS is initialized and operational.
     * @return True when module initialization completed successfully.
     */
    bool isInitialized() const;

};

/** Global singleton instance used by runtime tasks. */
extern GPS GPS_MODULE;
