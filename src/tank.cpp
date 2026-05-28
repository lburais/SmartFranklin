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
#include "log.h"

Tank TANK_TASK;

bool Tank::isInitialized() const
{
    return m_initialized;
}

bool Tank::init()
{
    SF_LOGI("[%s] init", m_tag);

    m_initialized = false;

    const bool configured = sf_interfaces::configured(m_sensor);

    if (!configured) {
        if (!sf_interfaces::configure(m_sensor)) {
            SF_LOGW("[%s] configure failed", m_tag);
            HMI::setLed(m_sensor, PortStatus::Error);
            return false;
        }
    }

    TwoWire* connector = sf_interfaces::getConnector(m_sensor).ptr.twoWire;
    if (connector == nullptr) {
        SF_LOGW("[%s] connector unavailable", m_tag);
        HMI::setLed(m_sensor, PortStatus::Error);
        return false;
    }

    if (!seize(m_sensor)) {
        SF_LOGW("[%s] unable to lock port", m_tag);
        HMI::setLed(m_sensor, PortStatus::Error);
        return false;
    }

    m_units.add(m_unit, *connector);

    const bool ok = m_units.begin();
    if (!ok) {
        SF_LOGW("[%s] %s m_unit not started", m_tag, m_device);
        HMI::setLed(m_sensor, PortStatus::Error);
        release(m_sensor);
        return false;
    } else {
        SF_LOGI("[%s] %s m_unit started", m_tag, m_device);
    }

    m_initialized = true;
    HMI::setLed(m_sensor, PortStatus::Initialized);

    release(m_sensor);

    SF_LOGI("[%s] (0x%02X) initialized", m_tag, sf_interfaces::getAddress(m_sensor));
    return true;
}

bool Tank::process()
{
    int32_t distanceMm = 0;
    int32_t fillPct = 0;

    if (!m_initialized) {
        if (!init()) {
            SF_LOGW("[%s] not configured", m_tag);
            HMI::setLed(m_sensor, PortStatus::Error);
            return false;
        }
    }

    if (!seize(m_sensor)) {
        SF_LOGW("[%s] unable to lock port", m_tag);
        HMI::setLed(m_sensor, PortStatus::Error);
        return false;
    }

    m_units.update();

    if (!m_unit.updated()) {
        release(m_sensor);
        HMI::setLed(m_sensor, PortStatus::NoData);
        return false;
    }

    const float rawDistanceMm = m_unit.distance();

    release(m_sensor);

    if (!std::isfinite(rawDistanceMm)) {
        SF_LOGW("[%s] non-finite distance sample ignored", m_tag);
        HMI::setLed(m_sensor, PortStatus::Error);
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

    SF_LOGI("[%s] Raw: %.2f cm    Distance: %d mm     Fill level: %d%%", m_tag, rawDistanceMm, distanceMm, fillPct);

    HMI::setLed(m_sensor, PortStatus::Ok);

    return true;
}

// ---- FreeRTOS task ----

void taskTank(void* pv)
{
    (void)pv;
    SF_LOGI("[TANK] Task started");

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
            if (!initialized) {
                SF_LOGW("[TANK] Init failed");
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
