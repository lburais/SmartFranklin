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

#include <cmath>
#include <cstdio>
#include <mutex>

#include "config_store.h"
#include "data_model.h"
#include "hmi.h"
#include "interfaces.h"
#include "mqtt.h"
#include "log.h"

Gaz GAZ_TASK;

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
        SF_LOGW("[%s] calibrate called before init", m_tag);
        HMI::setLed(m_sensor, PortStatus::Error);
        return false;
    }

    if (!m_calibration_in_progress) {
        SF_LOGW("[%s] calibrate called before tare", m_tag);
        HMI::setLed(m_sensor, PortStatus::Error);
        return false;
    }

    if (!seize(m_sensor)) {
        return false;
    }

    int32_t raw = m_unit.getRawADC();
    const float factor = raw / weightG;
    bool ok = false;

    if (raw == 0) {
        SF_LOGE("[%s] unable to readADC", m_tag);
        HMI::setLed(m_sensor, PortStatus::Error);
    } else {
        m_unit.setGapValue(factor);
        if (m_unit.getGapValue() != factor) {
            SF_LOGW("[%s] failed to reset calibration gap during calibrate", m_tag);
            HMI::setLed(m_sensor, PortStatus::Error);
        } else {
            ok = true;
        }

    }

    release(m_sensor);

    if (!ok) {
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

    if (!seize(m_sensor)) {
        return false;
    }

    m_unit.setOffset();

    release(m_sensor);

    {
        std::lock_guard<std::mutex> lock(DATA_MUTEX);
        DATA.weight_gaz = 0;
        DATA.fill_gaz   = 0;
    }

    CONFIG.gaz_calibration_factor = 1.0f;
    config_save();   
    
    m_calibration_in_progress = true;

    return true;
}

// ---- public interface ----

bool Gaz::isInitialized() const
{
    return m_initialized;
}

bool Gaz::init()
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

    bool ok = m_unit.begin( connector,
                            sf_interfaces::getSda(m_sensor),
                            sf_interfaces::getScl(m_sensor),
                            sf_interfaces::getAddress(m_sensor),
                            sf_interfaces::getClock(m_sensor) 
                          );

    if (!ok) {
        SF_LOGE("[%s] failed to start %s", m_tag, sf_interfaces::getDeviceName(m_sensor));
        HMI::setLed(m_sensor, PortStatus::Error);
    } else {
        SF_LOGI("[%s] %s started", m_tag, sf_interfaces::getDeviceName(m_sensor));
        float effectiveGap = CONFIG.gaz_calibration_factor;
        if (!std::isfinite(effectiveGap) || effectiveGap == 0.0f) {
            effectiveGap = 1.0f;
        }

        m_unit.setGapValue(effectiveGap);
        ok = (m_unit.getGapValue() == effectiveGap);
        if (!ok) {
            SF_LOGW("[%s] failed to apply calibration gap %.6f during init", m_tag, effectiveGap);
            HMI::setLed(m_sensor, PortStatus::Error);
        } else {
            SF_LOGI("[%s] (0x%02X) %s initialized", m_tag, sf_interfaces::getAddress(m_sensor), sf_interfaces::getDeviceName(m_sensor));
            m_initialized = true;
            HMI::setLed(m_sensor, PortStatus::Initialized);
        }

    }
    
    release(m_sensor);

    return ok;
}

bool Gaz::process()
{
    int32_t weightG        = 0;
    int32_t fillPct        = 0;

    if (!m_initialized) {
        if (!init()) {
            return false;
        }
    }

    if (!seize(m_sensor)) {
        return false;
    }

    const float rawWeight = m_unit.getWeight();

    release(m_sensor);

    if (!std::isfinite(rawWeight)) {
        SF_LOGW("[%s] non-finite weight sample ignored", m_tag);
        HMI::setLed(m_sensor, PortStatus::Error);
        return false;
    } else {
        HMI::setLed(m_sensor, PortStatus::Ok);
    }

    weightG = static_cast<int32_t>(lroundf(rawWeight));

    if (weightG <= GAZ_BOTTLE_EMPTY_G) { 
        fillPct = 0; 
    } else if (weightG >= GAZ_BOTTLE_FULL_G)  {
        fillPct = 100; 
    } else {
        fillPct = (100.0f * (weightG - GAZ_BOTTLE_EMPTY_G) / (GAZ_BOTTLE_FULL_G - GAZ_BOTTLE_EMPTY_G));
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

    SF_LOGI("[%s] Weight: %d g     Fill: %d%%", m_tag, weightG, fillPct);

    return true;
}

