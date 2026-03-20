#pragma once

#include <Arduino.h>

/**
 * @brief Tank ultrasonic runtime built on M5Stack Unit Ultrasonic I2C.
 *
 * Lifecycle mirrors other SmartFranklin sensor modules:
 * - `init()` discovers the unit on supported I2C routes,
 * - `process()` performs one read/update/publish cycle.
 */
class Tank {
public:
    /**
    * @brief Initializes the M5Stack ultrasonic I2C unit.
    * @param isInternalRoute True when the sensor is on Ex_I2C, false for Wire.
    * @param i2cAddress Detected sensor address on the selected I2C bus.
     * @return true when the sensor is reachable.
     */
    bool init(bool isInternalRoute, uint8_t i2cAddress);

    /**
     * @brief Executes one sensor update cycle.
     */
    void process();

    /**
     * @brief Returns module initialization state.
     */
    bool isInitialized() const;
};

/**
 * @brief Global Tank module singleton used by taskTank.
 */
extern Tank TANK_MODULE;
