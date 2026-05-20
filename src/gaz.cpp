/**
 * @file gaz.cpp
 * @brief Poids bouteille gaz : acquisition capteur poids I2C, calibration, publication MQTT.
 * @details
 * Ce module gere le capteur M5Stack Weight I2C connecte sur le port dont CONFIG.port_*_sensor vaut "gaz".
 * Il effectue les mesures periodiques, calcule le taux de remplissage, applique la calibration
 * et publie les donnees via MQTT.
 * 
 * Utilise Wire1 directement avec les parametres (pins SDA/SCL, adresse I2C) de sf_interfaces.
 */

#include "gaz.h"

#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedWEIGHT.h>
#include <M5Utility.h>
#include <Wire.h>

#include <cmath>
#include <cstdio>
#include <mutex>

#include "config_store.h"
#include "data_model.h"
#include "hmi.h"
#include "interfaces.h"
#include "mqtt.h"

Gaz GAZ_TASK;

// ---- static utilities ----

float Gaz::sanitizedGap(const float gap)
{
    if (!std::isfinite(gap) || gap == 0.0f) {
        return 1.0f;
    }
    return gap;
}

// ---- instance methods ----

bool Gaz::refreshMeasurement(int32_t& weightG, int32_t& fillPct)
{
    m_units.update();

    if (!m_unit.updated()) {
        return false;
    }

    const float rawWeight = m_unit.weight();
    if (!std::isfinite(rawWeight)) {
        M5_LOGW("[%s] non-finite weight sample ignored", m_tag);
        return false;
    }

    weightG = static_cast<int32_t>(lroundf(rawWeight));

    if (weightG <= GAZ_BOTTLE_EMPTY_G) { 
        fillPct = 0; 
    } else if (weightG >= GAZ_BOTTLE_FULL_G)  {
        fillPct = 100; 
    } else {
        fillPct = (100.0f * (weightG - GAZ_BOTTLE_EMPTY_G) / (GAZ_BOTTLE_FULL_G - GAZ_BOTTLE_EMPTY_G));
    }

    return true;
}

// ---- free functions for web_dashboard calibration API ----

bool scale_tare()
{
    return GAZ_TASK.tare();
}

bool scale_calibrate(float weightG)
{
    return GAZ_TASK.calibrate(weightG);
}

// ---- specific public interface ----

bool Gaz::calibrate(float weightG)
{
    if (!m_initialized) {
        M5_LOGW("[%s] calibrate called before init", m_tag);
        return false;
    }

    if (!m_calibration_in_progress) {
        M5_LOGW("[%s] calibrate called before tare", m_tag);
        return false;
    }

    int32_t raw = 0;
    if (!m_unit.readRawADC (raw)) {
        M5_LOGE("[%s] unable to readADC", m_tag);
        return false;
    }

    const float factor = raw / weightG;;
    if (!m_unit.writeGap(factor)) {
        M5_LOGW("[%s] failed to reset calibration gap during calibrate", m_tag);
        return false;
    }

    CONFIG.gaz_calibration_factor = factor;
    config_save();

    m_calibration_in_progress = false;

    return true;
}

bool Gaz::tare()
{
    m_calibration_in_progress = false;

    if (!m_initialized) {
        return false;
    }

    if (m_unit.resetOffset()) {

        std::lock_guard<std::mutex> lock(DATA_MUTEX);
        DATA.weight_gaz = 0;
        DATA.fill_gaz   = 0;

        CONFIG.gaz_calibration_factor = 1.0f;
        config_save();   
        
        m_calibration_in_progress = true;

        return true;
    }
    return false;
}

// ---- public interface ----

bool Gaz::isInitialized() const
{
    return m_initialized;
}

bool Gaz::init()
{
    M5_LOGI("[%s] init", m_tag);

    m_initialized        = false;

    if (!sf_interfaces::configured(m_sensor)) {
        M5_LOGW("[%s] configuration required", m_tag);
        if (!sf_interfaces::configure(m_sensor)) {
            M5_LOGW("[%s] configuration failed", m_tag);
            return false;
        }
    }

    TwoWire* connector = sf_interfaces::getPort(m_sensor).ptr.twoWire;
    if (connector == nullptr) {
        M5_LOGW("[%s] connector unavailable", m_tag);
        return false;
    }

    if (connector == &Wire1) {
        M5_LOGI("[%s] connector is Wire1", m_tag);
    } else if (connector == &Wire) {
        M5_LOGI("[%s] connector is Wire", m_tag);
    } else {
        M5_LOGI("[%s] connector is custom bus ptr=%p", m_tag, connector);
    }

    const bool ok = m_units.add(m_unit, *connector) && m_units.begin();
    if (!ok) {
        M5_LOGW("[%s] %s m_unit not added", m_tag, m_device);
        return false;
    } else {
        M5_LOGI("[%s] %s m_unit added", m_tag, m_device);
    }

    const float effectiveGap = sanitizedGap(CONFIG.gaz_calibration_factor);
    if (!m_unit.writeGap(effectiveGap)) {
        M5_LOGW("[%s] failed to apply calibration gap %.6f during init", m_tag, effectiveGap);
    }
    CONFIG.gaz_calibration_factor = effectiveGap;
    config_save();

    m_initialized = true;
    hmiSetPortLedStatus(m_sensor, m_initialized, false);

    M5_LOGI("[%s] (0x%02X) initialized", m_tag, sf_interfaces::getAddress(m_sensor));
    return true;
}

bool Gaz::process()
{
    int32_t weightG        = 0;
    int32_t fillPct        = 0;

    if (!m_initialized) {
        if (!init()) {
            M5_LOGW("[%s] not configured", m_tag);
            hmiSetPortLedStatus(m_sensor, m_initialized, true);
            return false;
        }
    }

    if (!refreshMeasurement(weightG, fillPct)) {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(DATA_MUTEX);
        DATA.weight_gaz = weightG;
        DATA.fill_gaz   = fillPct;
    }

    char gBuf[24] = {0};
    snprintf(gBuf, sizeof(gBuf), "%d", weightG);
    sf_mqtt::publish("smartfranklin/gaz/g", gBuf);

    char pctBuf[16] = {0};
    snprintf(pctBuf, sizeof(pctBuf), "%d", fillPct);
    sf_mqtt::publish("smartfranklin/gaz/fill", pctBuf, 1, true);

    M5_LOGI("[%s] Weight: %d g     Fill: %d%%", m_tag, weightG, fillPct);

    return true;
}

// ---- FreeRTOS task ----

void taskGaz(void* pv)
{
    (void)pv;
    M5_LOGI("[GAZ] Task started");

    bool     initialized       = false;
    bool     processed         = false;
    uint32_t nextInitAttemptMs = 0;

    auto isRetryDue = [](uint32_t nowMs, uint32_t nextAttemptMs) {
        return static_cast<int32_t>(nowMs - nextAttemptMs) >= 0;
    };

    auto scheduleRetry = [](uint32_t& nextAttemptMs, uint32_t nowMs) {
        nextAttemptMs = nowMs + 10000UL;  // 10 second retry interval
    };

    for (;;) {
        const uint32_t nowMs = millis();

        if (!GAZ_TASK.isInitialized() && isRetryDue(nowMs, nextInitAttemptMs)) {
            initialized = GAZ_TASK.init();
            hmiSetPortLedStatus(sf_interfaces::InterfaceSensor::Gaz, initialized, false);
            if (!initialized) {
                M5_LOGW("[GAZ] Init failed");
                scheduleRetry(nextInitAttemptMs, nowMs);
            }
        }

        if (initialized) {
            processed = GAZ_TASK.process();
        }

        const uint32_t recurrenceMs = sf_interfaces::getRecurrenceMs(sf_interfaces::InterfaceSensor::Gaz);
        const int loopMs = (recurrenceMs > 0) ? static_cast<int>(recurrenceMs) : 1000;
        vTaskDelay(pdMS_TO_TICKS(loopMs));

    }
}
