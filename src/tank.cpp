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
#include <Wire.h>

#include <cmath>
#include <cstdio>
#include <cstdint>
#include <mutex>

#include "config_store.h"
#include "data_model.h"
#include "hmi.h"
#include "interfaces.h"
#include "mqtt.h"

Tank TANK_TASK;

bool Tank::isInitialized() const
{
    return m_initialized;
}

bool Tank::init()
{
    m_initialized = false;

    const bool configured = sf_interfaces::configured(m_sensor);

    if (!configured) {
        if (!sf_interfaces::configure(m_sensor)) {
            M5_LOGW("[%s] configure failed", m_tag);
            return false;
        }
    }

    TwoWire* connector = sf_interfaces::getPort(m_sensor).ptr.twoWire;
    if (connector == nullptr) {
        M5_LOGW("[%s] connector unavailable", m_tag);
        return false;
    }

    if (connector == &Wire1) {
        M5_LOGI("[%s] connector is on Wire1", m_tag);
    } else if (connector == &Wire) {
        M5_LOGI("[%s] connector is on Wire", m_tag);
    } else {
        M5_LOGI("[%s] connector is custom bus ptr=%p", m_tag, connector);
    }

    if (!seize(m_sensor)) {
        M5_LOGW("[%s] unable to lock port", m_tag);
        return false;
    }

    const bool ok = m_units.add(m_unit, *connector) && m_units.begin();

    release(m_sensor);

    if (!ok) {
        M5_LOGW("[%s] %s m_unit not added", m_tag, m_device);
        return false;
    } else {
        M5_LOGI("[%s] %s m_unit added", m_tag, m_device);
    }

    m_initialized = true;
    hmiSetPortLedStatus(m_sensor, m_initialized, false);

    M5_LOGI("[%s] (0x%02X) initialized", m_tag, sf_interfaces::getAddress(m_sensor));
    return true;
}

bool Tank::process()
{
    int32_t distanceMm = 0;
    int32_t fillPct = 0;

    if (!m_initialized) {
        if (!init()) {
            M5_LOGW("[%s] not configured", m_tag);
            hmiSetPortLedStatus(m_sensor, m_initialized, true);
            return false;
        }
    }

    if (!seize(m_sensor)) {
        M5_LOGW("[%s] unable to lock port", m_tag);
        return false;
    }

    m_units.update();

    if (!m_unit.updated()) {
        release(m_sensor);
        return false;
    }

    const float rawDistanceMm = m_unit.distance();

    release(m_sensor);

    if (!std::isfinite(rawDistanceMm)) {
        M5_LOGW("[%s] non-finite distance sample ignored", m_tag);
        return false;
    }

    distanceMm = static_cast<int32_t>(lroundf(rawDistanceMm));

    if (distanceMm < TANK_DISTANCE_MIN_MM) {
        distanceMm = TANK_DISTANCE_MIN_MM;
    }
    if (distanceMm > TANK_DISTANCE_MAX_MM) {
        distanceMm = TANK_DISTANCE_MAX_MM;
    }

    if (distanceMm <= TANK_FULL_DISTANCE_MM) {
        fillPct = 0;
    } else if (distanceMm >= TANK_EMPTY_DISTANCE_MM) {
        fillPct = 100;
    } else {
        const float TANK_SPAN_DISTANCE_MM = static_cast<float>(TANK_EMPTY_DISTANCE_MM - TANK_FULL_DISTANCE_MM);
        const float numerator = static_cast<float>(TANK_EMPTY_DISTANCE_MM - distanceMm);
        
        fillPct = static_cast<int32_t>(lroundf((numerator / TANK_SPAN_DISTANCE_MM) * 100.0f));
    }

    {
        std::lock_guard<std::mutex> lock(DATA_MUTEX);
        DATA.distance_tank_mm = distanceMm;
        DATA.fill_tank = fillPct;
    }

    char mmBuf[24] = {0};
    snprintf(mmBuf, sizeof(mmBuf), "%d", distanceMm);
    sf_mqtt::publish("smartfranklin/tank/mm", mmBuf);

    char pctBuf[16] = {0};
    snprintf(pctBuf, sizeof(pctBuf), "%d", fillPct);
    sf_mqtt::publish("smartfranklin/tank/fill", pctBuf, 1, true);

    M5_LOGI("[%s] Raw: %.2f cm    Distance: %d mm     Fill level: %d%%", m_tag, rawDistanceMm, distanceMm, fillPct);

    return true;
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
        nextAttemptMs = nowMs + 10000UL;  // 10 second retry interval
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
