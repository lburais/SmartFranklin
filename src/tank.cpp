/**
 * @file tank.cpp
 * @brief Tank ultrasonic sensor implementation and fill-level mapping.
 *
 * Reads raw distance samples, clamps invalid values, computes tank fill
 * percentage from installation geometry, updates shared data, and publishes
 * telemetry topics for monitoring.
 *
 * SPDX-License-Identifier: MIT
 */

#include "tank.h"

#include <M5Unified.h>

#include <cmath>
#include <cstdio>
#include <cstdint>
#include <mutex>

#include "data_model.h"
#include "mqtt.h"

namespace {

/** Register used by the ultrasonic unit to trigger/read distance conversion. */
constexpr uint8_t TANK_DISTANCE_REGISTER = 0x01;
/** Sensor conversion latency required between command and readback. */
constexpr uint32_t TANK_CONVERSION_DELAY_MS = 120U;

/** Minimum supported measurement in millimeters (sensor datasheet bound). */
constexpr int32_t TANK_DISTANCE_MIN_MM = 20;
/** Maximum supported measurement in millimeters (sensor datasheet bound). */
constexpr int32_t TANK_DISTANCE_MAX_MM = 4500;

// Tank mapping defaults: tune these values to match the physical installation.
/** Distance corresponding to full tank (100%). */
constexpr int32_t TANK_FULL_DISTANCE_MM = 300;
/** Distance corresponding to empty tank (0%). */
constexpr int32_t TANK_EMPTY_DISTANCE_MM = 1500;

/** Clamp sensor distance to supported RCWL-9600 range. */
int32_t clampDistanceMm(int32_t distanceMm)
{
    if (distanceMm < TANK_DISTANCE_MIN_MM) {
        return TANK_DISTANCE_MIN_MM;
    }
    if (distanceMm > TANK_DISTANCE_MAX_MM) {
        return TANK_DISTANCE_MAX_MM;
    }
    return distanceMm;
}

/** Convert distance in mm to fill percentage using configured mapping. */
int32_t distanceToFillPct(const int32_t distanceMm)
{
    if (distanceMm <= TANK_FULL_DISTANCE_MM) {
        return 100;
    }

    if (distanceMm >= TANK_EMPTY_DISTANCE_MM) {
        return 0;
    }

    const float span = static_cast<float>(TANK_EMPTY_DISTANCE_MM - TANK_FULL_DISTANCE_MM);
    if (span <= 0.0f) {
        return 0;
    }

    const float numerator = static_cast<float>(TANK_EMPTY_DISTANCE_MM - distanceMm);
    const int32_t pct = static_cast<int32_t>(lroundf((numerator / span) * 100.0f));

    if (pct < 0) {
        return 0;
    }
    if (pct > 100) {
        return 100;
    }
    return pct;
}

struct TankState {
    /** Protects all mutable tank runtime state. */
    mutable std::mutex mutex;
    /** Set to true once module initialization succeeds. */
    bool initialized = false;
    /** True when sensor is reached through internal M5 I2C route. */
    bool isInternalRoute = false;
    /** Effective I2C address used for the ultrasonic sensor. */
    uint8_t i2cAddress = 0x57;
    /** Last valid measured distance in millimeters. */
    int32_t lastDistanceMm = 0;
};

TankState TANK_STATE;

}  // namespace

Tank TANK_MODULE;

/** Acquire one tank sample while state mutex is held. */
static bool refreshMeasurementLocked(TankState& state, int32_t& distanceMm)
{
    uint32_t rawDistance = 0;
    bool readOk = false;

    if (state.isInternalRoute) {
        if (!M5.Ex_I2C.start(state.i2cAddress, false, Wire.getClock())) {
            return false;
        }

        if (!M5.Ex_I2C.write(TANK_DISTANCE_REGISTER) || !M5.Ex_I2C.stop()) {
            return false;
        }
    } else {
        Wire.beginTransmission(state.i2cAddress);
        Wire.write(TANK_DISTANCE_REGISTER);
        if (Wire.endTransmission() != 0) {
            return false;
        }
    }

    delay(TANK_CONVERSION_DELAY_MS);

    if (state.isInternalRoute) {

        if (!M5.Ex_I2C.start(state.i2cAddress, true, Wire.getClock())) {
            return false;
        }

        rawDistance = 0;
        for (uint8_t i = 0; i < 3; ++i) {
            uint8_t byte = 0;
            const bool lastNack = (i == 2);
            if (!M5.Ex_I2C.read(&byte, 1U, lastNack)) {
                M5.Ex_I2C.stop();
                return false;
            }

            rawDistance <<= 8;
            rawDistance |= static_cast<uint32_t>(byte);
        }

        readOk = M5.Ex_I2C.stop();
    } else {
        const uint8_t readCount = Wire.requestFrom(state.i2cAddress, static_cast<uint8_t>(3));
        if (readCount < 3) {
            return false;
        }

        rawDistance = 0;
        rawDistance = static_cast<uint32_t>(Wire.read());
        rawDistance <<= 8;
        rawDistance |= static_cast<uint32_t>(Wire.read());
        rawDistance <<= 8;
        rawDistance |= static_cast<uint32_t>(Wire.read());
        readOk = true;
    }

    if (!readOk) {
        return false;
    }

    const float distanceRawMm = static_cast<float>(rawDistance) / 1000.0f;
    if (!std::isfinite(distanceRawMm)) {
        M5_LOGW("[TANK] non-finite distance sample ignored");
        return false;
    }

    distanceMm = clampDistanceMm(static_cast<int32_t>(lroundf(distanceRawMm)));
    state.lastDistanceMm = distanceMm;
    return true;
}

/** Initialize tank state and selected route metadata. */
static bool initState(TankState& state, bool isInternalRoute, uint8_t i2cAddress)
{
    std::lock_guard<std::mutex> lock(state.mutex);

    state.isInternalRoute = isInternalRoute;
    state.i2cAddress = i2cAddress;
    state.initialized = true;

    M5_LOGI("[TANK] Ultrasonic I2C initialization complete");
    return true;
}

/** Publish tank distance and fill percentage to MQTT. */
static void publishDistance(const int32_t distanceMm, const int32_t fillPct)
{
    char mmBuf[24] = {0};
    snprintf(mmBuf, sizeof(mmBuf), "%d", distanceMm);
    sf_mqtt::publish("smartfranklin/tank/mm", mmBuf);

    char pctBuf[16] = {0};
    snprintf(pctBuf, sizeof(pctBuf), "%d", fillPct);
    sf_mqtt::publish("smartfranklin/tank/fill", pctBuf, 1, true);

    M5_LOGI("[TANK] Distance: %d mm     Fill level: %d%%", distanceMm, fillPct);
}

/** Run one full process cycle: measure, convert, persist, publish. */
static void processState(TankState& state)
{
    int32_t distanceMm = 0;
    bool hasMeasurement = false;

    {
        std::lock_guard<std::mutex> lock(state.mutex);
        if (!state.initialized) {
            return;
        }

        hasMeasurement = refreshMeasurementLocked(state, distanceMm);
    }

    if (!hasMeasurement) {
        M5_LOGW("[TANK] No measurement");
        return;
    }

    const int32_t fillPct = distanceToFillPct(distanceMm);

    {
        std::lock_guard<std::mutex> lock(DATA_MUTEX);
        DATA.distance_tank_mm = distanceMm;
        DATA.fill_tank = fillPct;
    }

    publishDistance(distanceMm, fillPct);
}

/** Return current initialization flag from protected state. */
static bool isInitializedState(const TankState& state)
{
    std::lock_guard<std::mutex> lock(state.mutex);
    return state.initialized;
}

/** @brief Initializes the tank module with selected route and address. */
bool Tank::init(bool isInternalRoute, uint8_t i2cAddress)
{
    return initState(TANK_STATE, isInternalRoute, i2cAddress);
}

/** @brief Executes one acquisition cycle and publishes tank telemetry. */
void Tank::process()
{
    processState(TANK_STATE);
}

/** @brief Returns true when tank module initialization has completed. */
bool Tank::isInitialized() const
{
    return isInitializedState(TANK_STATE);
}
