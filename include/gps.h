/*
 * SmartFranklin - GPS module interface
 * SPDX-License-Identifier: MIT
 *
 * Public API for the Gravity DFR1103 GNSS/RTC runtime module.
 */

#pragma once

#include <Arduino.h>
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
