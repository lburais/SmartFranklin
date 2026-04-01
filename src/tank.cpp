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
#include "i2c.h"
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
    Wire.beginTransmission(i2cAddress);
    Wire.write(TANK_DISTANCE_REGISTER);
    if (Wire.endTransmission() != 0) {
        return false;
    }

    delay(TANK_CONVERSION_DELAY_MS);

    const uint8_t readCount = Wire.requestFrom(i2cAddress, static_cast<uint8_t>(3));
    if (readCount < 3) {
        return false;
    }

    uint32_t rawDistance = 0;
    rawDistance = static_cast<uint32_t>(Wire.read());
    rawDistance <<= 8;
    rawDistance |= static_cast<uint32_t>(Wire.read());
    rawDistance <<= 8;
    rawDistance |= static_cast<uint32_t>(Wire.read());

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

    m_initialized = false;
    m_activeConfiguredPort = String();

    const String configuredPort = i2cConfiguredPortForSensor(sf_ports::PortSensor::Tank, "TANK");

    M5_LOGI("[***************] init: '%s'", configuredPort.c_str());

    if (!i2cBeginConfiguredPort(configuredPort, "TANK") ||
        !i2cDeviceExistsOnConfiguredPort(m_i2cAddress, configuredPort, "TANK")) {
        M5_LOGW("[***************] begin fail: '%s' (0x%02X)", configuredPort.c_str(), m_i2cAddress);
        return false;
    }

    i2cPublishConfiguration("tank", configuredPort, m_i2cAddress);
    m_activeConfiguredPort = configuredPort;
    m_initialized = true;

    M5_LOGI("[TANK] initialized on port '%s' (0x%02X)", configuredPort.c_str(), m_i2cAddress);
    return true;
}

void Tank::process()
{
    int32_t distanceMm = 0;
    int32_t fillPct = 0;

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_initialized || m_activeConfiguredPort.isEmpty()) {
            return;
        }

        i2cBeginConfiguredPort(m_activeConfiguredPort, "TANK");
        if (!readDistanceMm(m_i2cAddress, distanceMm)) {
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
        nextAttemptMs = nowMs + kI2cInitRetryMs;
    };

    for (;;) {
        const uint32_t nowMs = millis();

        if (!initialized && isRetryDue(nowMs, nextInitAttemptMs)) {
            initialized = TANK_TASK.init();
            if (!initialized) {
                M5_LOGW("[TANK] Init failed");
                scheduleRetry(nextInitAttemptMs, nowMs);
            }
        }

        if (initialized) {
            TANK_TASK.process();
        }

        const int loopMs = (CONFIG.task_i2c_loop_ms > 0) ? CONFIG.task_i2c_loop_ms : 1000;
        vTaskDelay(pdMS_TO_TICKS(loopMs));
    }
}
