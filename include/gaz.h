#pragma once

#include <Arduino.h>

#include "i2c.h"

/**
 * @brief Gas bottle weight runtime built on M5Stack Weight I2C.
 *
 * This module mirrors the narrow lifecycle used by the GPS runtime:
 * - `init()` discovers and starts the sensor,
 * - `process()` performs one read/update/publish cycle,
 * - calibration helpers are exposed for the HMI workflow.
 */
class Gaz {
public:
    /**
     * @brief Initializes the M5 Weight I2C unit.
     * @param iSInternalRoute True when the sensor is on Ex_I2C, false for Wire.
     * @param i2cAddress Detected sensor address on the selected I2C bus.
     * @return true when the sensor is reachable and configured.
     */
    bool init(bool iSInternalRoute, uint8_t i2cAddress);

    /**
     * @brief Executes one sensor update cycle.
     */
    void process();

    /**
     * @brief Resets the current offset on the sensor.
     * @return true on success.
     */
    bool tare();

    /**
     * @brief Applies the sensor calibration gap.
     * @param gap Calibration gap as defined by M5Unit-WEIGHT.
     * @return true on success.
     */
    bool applyCalibration(float gap);

    /**
     * @brief Reads the current unscaled sample used by the calibration flow.
     * @return Current sample in kilograms when available, otherwise 0.
     */
    float readCalibrationSample();

    /**
     * @brief Returns module initialization state.
     */
    bool isInitialized() const;
};

/**
 * @brief Global Gaz module singleton used by taskGaz and HMI calibration.
 */
extern Gaz GAZ_MODULE;