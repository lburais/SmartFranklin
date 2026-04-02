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

#include <mutex>

#include "interfaces.h"

class GPS {
public:
    const uint8_t deviceAddress = 0x66;

    /**
    * @brief Initialize GPS runtime on the configured I2C interface.
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

private:
    static constexpr uint32_t kProcessPeriodMs = 1000UL;

    bool writeRegister(uint8_t reg, uint8_t value) const;
    bool readRegisters(uint8_t reg, uint8_t* out, size_t len) const;
    bool readPoseAndTimeLocked();
    void publishFix(const char* dateBuf, const char* utcBuf) const;
    static double decodeUnsigned_2_1_100(uint8_t b0, uint8_t b1, uint8_t b2);

    mutable std::mutex m_mutex;
    bool m_initialized = false;
    uint8_t m_i2cAddress = 0x00;

    double m_latitudeDeg = 0.0;
    double m_longitudeDeg = 0.0;
    double m_altitudeM = 0.0;
    double m_speedKnots = 0.0;
    double m_courseDeg = 0.0;
    uint8_t m_satellites = 0;
    bool m_hasFix = false;
    uint16_t m_year = 0;
    uint8_t m_month = 0;
    uint8_t m_day = 0;
    uint8_t m_hour = 0;
    uint8_t m_minute = 0;
    uint8_t m_second = 0;
    uint32_t m_lastProcessMs = 0;
};

/** Global singleton instance used by runtime tasks. */
extern GPS GPS_TASK;
