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
    SF_LOGI("[%s] initializing...", m_tag);

    m_initialized        = false;

    if (!sf_interfaces::configured(m_sensor)) {
        if (!sf_interfaces::configure(m_sensor)) {
            return false;
        }
    }

    TwoWire* connector = sf_interfaces::getConnector(m_sensor).ptr.twoWire;
    if (connector == nullptr) {
        return false;
    }

    if (!seize(m_sensor)) {
        return false;
    }

    m_unit.begin( connector,
                  sf_interfaces::getAddress(m_sensor),
                  sf_interfaces::getSda(m_sensor),
                  sf_interfaces::getScl(m_sensor),
                  sf_interfaces::getClock(m_sensor) 
                );

    SF_LOGI("[%s] %s started", m_tag, sf_interfaces::getDeviceName(m_sensor));

    SF_LOGI("[%s] (0x%02X) %s initialized", m_tag, sf_interfaces::getAddress(m_sensor), sf_interfaces::getDeviceName(m_sensor));

    m_initialized = true;
    HMI::setLed(m_sensor, PortStatus::Initialized);

    release(m_sensor);

    return true;
}

bool Tank::process()
{
    int32_t distanceMm = 0;
    int32_t fillPct = 0;

    if (!m_initialized) {
        if (!init()) {
            return false;
        }
    }

    if (!seize(m_sensor)) {
        return false;
    }

    const float rawDistanceMm = m_unit.getDistance();

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

    SF_LOGI("[%s] Raw: %.2f mm    Distance: %d mm     Fill level: %d%%", m_tag, rawDistanceMm, distanceMm, fillPct);

    HMI::setLed(m_sensor, PortStatus::Ok);

    return true;
}

