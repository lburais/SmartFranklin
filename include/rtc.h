/**
 * @file rtc.h
 * @brief Public interface for real-time clock acquisition and synchronization.
 *
 * This module exposes RTC source selection and periodic processing hooks used
 * by telemetry and UI layers to maintain valid date/time state.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <Arduino.h>

class RTC {
public:
    /**
     * @brief Supported RTC sources.
     */
    enum class Source : uint8_t {
        /** No RTC source selected. */
        None = 0,
        /** Internal RTC available on supported M5 boards. */
        InternalRtc,
        /** External M5Stack RTC unit. */
        ExternalM5StackRtcUnit,
        /** External Seeed PCD85063TP-compatible RTC source. */
        ExternalSeeedPcd85063tp,
    };

    /**
     * @brief Initialize RTC backend and selected source.
     * @param source Selected RTC source implementation.
     * @param isInternalRoute True for internal route, false for Wire.
     * @param i2cAddress I2C address for external RTC devices.
     * @return True when initialization succeeds.
     */
    bool init(Source source, bool isInternalRoute, uint8_t i2cAddress);

    /**
     * @brief Execute one periodic RTC refresh cycle.
     *
     * Reads current time, validates it, and propagates values into shared
     * runtime state for display and telemetry consumers.
     */
    void process();

    /**
     * @brief Check whether RTC has been initialized successfully.
     * @return True when RTC module is operational.
     */
    bool isInitialized() const;

    /**
     * @brief Get active RTC source enum.
     * @return Current source.
     */
    Source source() const;

    /**
     * @brief Get active source label.
     * @return Human-readable source name.
     */
    const char* sourceName() const;

    /**
     * @brief Convert source enum to stable text identifier.
     * @param source Source enum value.
     * @return Static source name string.
     */
    static const char* sourceToString(Source source);
};

/** Global singleton instance used by system tasks. */
extern RTC RTC_MODULE;
