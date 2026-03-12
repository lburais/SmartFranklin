/*
 * ============================================================================
 * GPS Module Interface - SmartFranklin
 * ============================================================================
 *
 * File:        gps.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Public API for Gravity DFR1103 GNSS/RTC support.
 *              Exposes startup and periodic processing hooks used by the
 *              scheduler-facing GPS task.
 *
 * Author:      Laurent Burais
 * Date:        12 March 2026
 * Version:     1.1
 *
 * Overview:
 *   The GPS class encapsulates all DFR1103 interactions behind a narrow API:
 *   - `init()` discovers and starts the module,
 *   - `process()` runs one read/update/publish cycle.
 *
 *   Internally, this class supports direct Wire access and PA Hub-routed
 *   access and stores compact runtime state used by the implementation.
 *
 * ============================================================================
 * MIT License
 * ============================================================================
 * Copyright (c) 2026 Laurent Burais
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * ============================================================================
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>

#include <DFRobot_GNSSAndRTC.h>

#include "i2c_bus.h"

/**
 * @brief Gravity DFR1103 GNSS/RTC integration class.
 *
 * This class owns one DFR1103 instance and supports either direct Wire access
 * or PA Hub-routed Wire access. It is responsible for module discovery,
 * initialization, periodic reads, DATA model updates, and MQTT publication.
 *
 * Typical lifecycle:
 * 1. `taskGps` calls `init()` once the task starts.
 * 2. On success, `taskGps` calls `process()` on each loop period.
 * 3. `process()` performs one read-and-publish cycle.
 */
class GPS {
public:
    /**
     * @brief Detects and initializes the DFR1103 module.
     *
     * Initializes Wire pins/speed, resolves routing (direct/PA Hub), starts
     * the DFRobot driver, and enables GNSS power.
     *
     * @return true if module initialization succeeded, otherwise false.
     */
    bool init();

    /**
     * @brief Runs one periodic cycle (read + publish) when initialized.
     *
     * This is intentionally lightweight at call site and encapsulates bus
     * selection, GNSS/RTC reads, DATA updates, logging, and MQTT publishing.
     */
    void process();

private:
    /**
     * @brief DFR1103 default I2C address.
     *
     * The Gravity DFR1103 factory default I2C address is 0x66.
     */
    static constexpr uint8_t DFR1103_I2C_ADDRESS = 0x66;

    /**
     * @brief Enables exactly one PA Hub channel.
     * @param channel Channel index in range 0..7.
     * @return true on successful PA Hub write, otherwise false.
     */
    bool selectPaHubChannel(uint8_t channel);

    /**
     * @brief Disables all PA Hub channels.
     */
    void disablePaHubChannels();

    /**
     * @brief Reads GNSS/RTC values, updates DATA, and publishes MQTT payloads.
     *
     * Internal worker used by `process()` after successful initialization.
     */
    void readAndPublish();

    /**
     * @brief DFRobot GNSS/RTC driver instance bound to Wire.
     *
     * All low-level register accesses are performed through this object.
     */
    DFRobot_GNSSAndRTC_I2C m_unit{&Wire, DFR1103_I2C_ADDRESS};

    /**
     * @brief Current detected I2C path to the DFR1103 module.
     */
    I2CBusPathResult m_i2c_path{};

    /**
     * @brief Module initialization state.
     *
     * Set true only when routing and driver startup completed successfully.
     */
    bool m_initialized = false;

    /**
     * @brief Active Wire SDA pin used for DFR1103 access.
     *
     * Resolved from M5 pin mapping during `init()`.
     */
    int8_t m_wireSda = -1;

    /**
     * @brief Active Wire SCL pin used for DFR1103 access.
     *
     * Resolved from M5 pin mapping during init().
     */
    int8_t m_wireScl = -1;
};

/**
 * @brief Global GPS module instance used by the GPS FreeRTOS task.
 */
extern GPS GPS_MODULE;
