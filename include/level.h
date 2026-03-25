/**
 * @file level.h
 * @brief Public interface for tilt/attitude processing (pitch/roll).
 *
 * The Level module abstracts IMU source selection and periodic pose updates.
 * It is used by local UI, dashboard APIs, and MQTT telemetry publication.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <Arduino.h>

class Level {
public:
    /**
     * @brief Supported IMU sources.
     */
    enum class Source : uint8_t {
        /** No active source. */
        None = 0,
        /** Internal M5 IMU (board integrated). */
        InternalM5,
        /** External MPU6050-compatible M5 unit. */
        ExternalMpuUnit,
        /** External ADXL345 sensor source. */
        ExternalAdxl345,
    };

    /**
     * @brief Initialize level computation backend.
     * @param source Selected IMU source implementation.
     * @param isInternalRoute True for internal route, false for Wire route.
     * @param i2cAddress I2C address for external IMU devices.
     * @return True on successful initialization.
     */
    bool init(Source source, bool isInternalRoute, uint8_t i2cAddress);

    /**
     * @brief Execute one periodic level processing cycle.
     *
     * Computes filtered pitch/roll and derived wheel-height deltas, then
     * updates shared runtime state and optional telemetry outputs.
     */
    void process();

    /**
     * @brief Check module initialization status.
     * @return True when the module is ready to process samples.
     */
    bool isInitialized() const;

    /**
     * @brief Get active source enum value.
     * @return Current source.
     */
    Source source() const;

    /**
     * @brief Get active source as text.
     * @return Pointer to static source label.
     */
    const char* sourceName() const;

    /**
     * @brief Convert source enum to stable string representation.
     * @param source Source enum value.
     * @return Source text identifier.
     */
    static const char* sourceToString(Source source);
};

/** Global singleton instance used by runtime tasks and APIs. */
extern Level LEVEL_MODULE;
