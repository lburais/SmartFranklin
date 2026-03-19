#pragma once

#include <Arduino.h>

#include "i2c.h"

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
     * @brief Initializes the M5Stack ultrasonic I2C unit using a pre-detected route.
     * @param device Resolved I2C device route and bus configuration.
     * @return true when the sensor is reachable.
     */
    bool init(const sf_i2c::Device& device);

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
