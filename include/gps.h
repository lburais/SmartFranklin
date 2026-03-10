/*
 * ============================================================================
 * GPS Module Interface - SmartFranklin
 * ============================================================================
 *
 * File:        gps.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Public interface for Gravity DFR1103 GNSS/RTC support.
 *              Provides initialization, periodic read/publish processing,
 *              and runtime health status helpers.
 *
 * Author:      Laurent Burais
 * Date:        10 March 2026
 * Version:     1.0
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

/**
 * @brief Gravity DFR1103 GNSS/RTC integration class.
 *
 * This class owns one DFR1103 instance and supports either direct Wire access
 * or PA Hub-routed Wire access. It is responsible for module discovery,
 * initialization, periodic reads, DATA model updates, and MQTT publication.
 *
 * Typical lifecycle:
 * 1. taskGps calls init() once the task starts.
 * 2. On success, taskGps calls process() on each period.
 * 3. process() performs one read-and-publish cycle.
 * 4. isHealthy() is used by task status reporting and diagnostics.
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

    /**
     * @brief Returns whether the module was initialized successfully.
     * @return true when init() completed successfully.
     */
    bool isInitialized() const;

    /**
     * @brief Returns health based on latest read success and timeout window.
     * @return true when the latest sample is valid and recent.
     */
    bool isHealthy() const;

private:
    /**
     * @brief Supported routing modes for DFR1103 access.
     */
    enum class BusMode : uint8_t {
        Unset = 0,
        WireDirect,
        WirePaHub,
    };

    /**
     * @brief DFR1103 default I2C address.
        *
        * The Gravity DFR1103 factory default I2C address is 0x66.
     */
    static constexpr uint8_t DFR1103_I2C_ADDRESS = 0x66;

    /**
     * @brief Probes an I2C address on the active Wire bus.
     * @param address 7-bit I2C address.
        * @return true when device ACKs, otherwise false.
     */
    bool probeAddressOnWire(uint8_t address);

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
     * @brief Detects whether DFR1103 is reachable directly or through PA Hub.
        *
        * On success, this function updates m_busMode and m_pahubChannel.
        * On failure, routing state is reset to defaults.
        *
     * @return true when a supported path is found, otherwise false.
     */
    bool detectBusPath();

    /**
     * @brief Reads GNSS/RTC values, updates DATA, and publishes MQTT payloads.
        *
        * Internal worker used by process() after successful initialization.
     */
    void readAndPublish();

    /**
     * @brief DFRobot GNSS/RTC driver instance bound to Wire.
        *
        * All low-level register accesses are performed through this object.
     */
    DFRobot_GNSSAndRTC_I2C m_unit{&Wire, DFR1103_I2C_ADDRESS};

    /**
     * @brief Current detected bus routing mode.
        *
        * Determines whether reads are done directly on Wire or via PA Hub
        * channel selection.
     */
    BusMode m_busMode = BusMode::Unset;

    /**
     * @brief PA Hub channel used for module access when routed by PA Hub.
        *
        * Value is -1 when route is not PA Hub based.
     */
    int8_t m_pahubChannel = -1;

    /**
     * @brief Module initialization state.
        *
        * Set true only when routing and driver startup completed successfully.
     */
    bool m_initialized = false;

    /**
     * @brief Status of the latest read/publish cycle.
        *
        * Set false when bus selection/read/parsing fails.
     */
    bool m_lastReadOk = false;

    /**
     * @brief Millisecond timestamp of latest successful read cycle.
        *
        * Compared against a timeout window in isHealthy().
     */
    uint32_t m_lastReadMs = 0;

    /**
     * @brief Active Wire SDA pin used for DFR1103 access.
        *
        * Resolved from M5 pin mapping during init().
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
