/*
 * SmartFranklin - GPS module interface
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <Arduino.h>

/**
 * @brief Gravity DFR1103 GNSS/RTC sensor integration module.
 *
 * This class manages a DFR1103 instance for satellite positioning, altitude,
 * and timekeeping. It supports both direct Wire access and PAHub-routed I2C.
 *
 * **Lifecycle:**
 * - `init()` - Detect I2C route, initialize sensor, configure channels
 * - `process()` - Read position/altitude/time, update DATA, publish via MQTT
 *
 * **Thread Safety:** All state is protected by internal mutex.
 *
 * MQTT topics include fix, coordinates, altitude, satellites, UTC date/time,
 * RTC time, and I2C diagnostic publication paths.
 */
class GPS {
public:
    /**
     * @brief Detects and initializes the DFR1103 module.
     * @return true when the sensor is reachable and configured.
     */
    bool init();

    /**
     * @brief Executes one sensor update cycle.
     */
    void process();
};

/**
 * @brief Global GPS module instance used by the GPS FreeRTOS task.
 */
extern GPS GPS_MODULE;
