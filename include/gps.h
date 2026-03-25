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

class GPS {
public:
    /**
     * @brief Supported GPS input sources.
     */
    enum class Source : uint8_t {
        /** No valid source selected. */
        None = 0,
        /** External DFRobot Gravity I2C GNSS module. */
        ExternalDfrobotGravity,
    };

    /**
     * @brief Initialize GPS runtime with the selected source and route.
     * @param source GNSS source implementation to initialize.
     * @param isInternalRoute True when using M5 internal route, false for Wire.
     * @param i2cAddress I2C address of the GPS unit when applicable.
     * @return True when initialization succeeds.
     */
    bool init(Source source, bool isInternalRoute, uint8_t i2cAddress);

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

    /**
     * @brief Get currently selected source.
     * @return Active source enum.
     */
    Source source() const;

    /**
     * @brief Get human-readable active source label.
     * @return Source label string.
     */
    const char* sourceName() const;

    /**
     * @brief Convert source enum to a stable text identifier.
     * @param source Source value to convert.
     * @return Constant source name string.
     */
    static const char* sourceToString(Source source);
};

/** Global singleton instance used by runtime tasks. */
extern GPS GPS_MODULE;
