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
     * @brief Initialize level computation backend.
     * @return True on successful initialization.
     */
    bool init();

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


};

/** Global singleton instance used by runtime tasks and APIs. */
extern Level LEVEL_TASK;
