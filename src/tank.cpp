/**
 * @file tank.cpp
 * @brief Tank ultrasonic sensor acquisition and fill-level mapping.
 *
 * This module performs one-shot reads on the external Wire bus, computes
 * fill percentage, updates shared DATA, and publishes telemetry topics.
 *
 * SPDX-License-Identifier: MIT
 */

#include "tank.h"

#include <M5Unified.h>

#include <cmath>
#include <cstdio>
#include <cstdint>
#include <mutex>

#include "config_store.h"
#include "data_model.h"
#include "hmi.h"
#include "interfaces.h"
#include "mqtt.h"

namespace {

constexpr uint8_t TANK_DISTANCE_REGISTER = 0x01;
constexpr uint32_t TANK_CONVERSION_DELAY_MS = 120U;

constexpr int32_t TANK_DISTANCE_MIN_MM = 20;
constexpr int32_t TANK_DISTANCE_MAX_MM = 4500;

constexpr int32_t TANK_FULL_DISTANCE_MM = 300;
constexpr int32_t TANK_EMPTY_DISTANCE_MM = 1500;

int32_t clampDistanceMm(const int32_t distanceMm)
{
    if (distanceMm < TANK_DISTANCE_MIN_MM) {
        return TANK_DISTANCE_MIN_MM;
    }
    if (distanceMm > TANK_DISTANCE_MAX_MM) {
        return TANK_DISTANCE_MAX_MM;
    }
    return distanceMm;
}

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

bool readDistanceMm(const uint8_t i2cAddress, int32_t& distanceMm)
{
    Wire1.beginTransmission(i2cAddress);
    Wire1.write(TANK_DISTANCE_REGISTER);
    if (Wire1.endTransmission() != 0) {
        return false;
    }

    delay(TANK_CONVERSION_DELAY_MS);

    const uint8_t readCount = Wire1.requestFrom(i2cAddress, static_cast<uint8_t>(3));
    if (readCount < 3) {
        return false;
    }

    uint32_t rawDistance = 0;
    rawDistance = static_cast<uint32_t>(Wire1.read());
    rawDistance <<= 8;
    rawDistance |= static_cast<uint32_t>(Wire1.read());
    rawDistance <<= 8;
    rawDistance |= static_cast<uint32_t>(Wire1.read());

    const float distanceRawMm = static_cast<float>(rawDistance) / 1000.0f;
    if (!std::isfinite(distanceRawMm)) {
        M5_LOGW("[TANK] non-finite distance sample ignored");
        return false;
    }

    distanceMm = clampDistanceMm(static_cast<int32_t>(lroundf(distanceRawMm)));
    return true;
}

void publishDistance(const int32_t distanceMm, const int32_t fillPct)
{
    char mmBuf[24] = {0};
    snprintf(mmBuf, sizeof(mmBuf), "%d", distanceMm);
    sf_mqtt::publish("smartfranklin/tank/mm", mmBuf);

    char pctBuf[16] = {0};
    snprintf(pctBuf, sizeof(pctBuf), "%d", fillPct);
    sf_mqtt::publish("smartfranklin/tank/fill", pctBuf, 1, true);

    M5_LOGI("[TANK] Distance: %d mm     Fill level: %d%%", distanceMm, fillPct);
}

}  // namespace

Tank TANK_TASK;

bool Tank::isInitialized() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_initialized;
}

bool Tank::init()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::lock_guard<std::recursive_mutex> i2cLock(sf_i2c::i2cMutex());

    const uint8_t i2cAddress = sf_interfaces::getAddress(sf_interfaces::InterfaceSensor::Tank);
    if (i2cAddress == 0) {
        M5_LOGW("[TANK] address not defined in interfaces");
        return false;
    }

    m_initialized = false;
    m_activeConfiguredPort = "";

    if (!sf_i2c::i2cBeginConfiguredPort(sf_interfaces::InterfaceSensor::Tank) ||
        !sf_i2c::i2cDeviceExistsOnConfiguredPort(sf_interfaces::InterfaceSensor::Tank, i2cAddress)) {
        return false;
    }

    sf_i2c::i2cPublishConfiguration(sf_interfaces::InterfaceSensor::Tank, i2cAddress);
    m_activeConfiguredPort = sf_interfaces::toString(sf_interfaces::getName(sf_interfaces::InterfaceSensor::Tank));
    m_initialized = true;

    M5_LOGI("[TANK] initialized on port '%s' (0x%02X)", m_activeConfiguredPort.c_str(), i2cAddress);
    return true;
}

void Tank::process()
{
    int32_t distanceMm = 0;
    int32_t fillPct = 0;

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        std::lock_guard<std::recursive_mutex> i2cLock(sf_i2c::i2cMutex());
        const uint8_t i2cAddress = sf_interfaces::getAddress(sf_interfaces::InterfaceSensor::Tank);
        if (i2cAddress == 0) {
            M5_LOGW("[TANK] address not defined in interfaces");
            return;
        }

        if (!m_initialized || m_activeConfiguredPort.isEmpty()) {
            return;
        }

        sf_i2c::i2cBeginConfiguredPort(sf_interfaces::InterfaceSensor::Tank);
        if (!readDistanceMm(i2cAddress, distanceMm)) {
            M5_LOGW("[TANK] No measurement");
            return;
        }

        fillPct = distanceToFillPct(distanceMm);
    }

    {
        std::lock_guard<std::mutex> lock(DATA_MUTEX);
        DATA.distance_tank_mm = distanceMm;
        DATA.fill_tank = fillPct;
    }

    publishDistance(distanceMm, fillPct);
}

// ---- FreeRTOS task ----

void taskTank(void* pv)
{
    (void)pv;
    M5_LOGI("[TANK] Task started");

    bool     initialized       = false;
    uint32_t nextInitAttemptMs = 0;

    auto isRetryDue = [](uint32_t nowMs, uint32_t nextAttemptMs) {
        return static_cast<int32_t>(nowMs - nextAttemptMs) >= 0;
    };

    auto scheduleRetry = [](uint32_t& nextAttemptMs, uint32_t nowMs) {
        nextAttemptMs = nowMs + sf_i2c::kI2cInitRetryMs;
    };

    for (;;) {
        const uint32_t nowMs = millis();

        if (!initialized && isRetryDue(nowMs, nextInitAttemptMs)) {
            initialized = TANK_TASK.init();
            hmiSetPortLedStatus(sf_interfaces::InterfaceSensor::Tank, initialized, false);
            if (!initialized) {
                M5_LOGW("[TANK] Init failed");
                scheduleRetry(nextInitAttemptMs, nowMs);
            }
        }

        if (initialized) {
            TANK_TASK.process();
        }

        const uint32_t recurrenceMs = sf_interfaces::getRecurrenceMs(sf_interfaces::InterfaceSensor::Tank);
        const int loopMs = (recurrenceMs > 0) ? static_cast<int>(recurrenceMs) : 1000;
        vTaskDelay(pdMS_TO_TICKS(loopMs));
    }
}
