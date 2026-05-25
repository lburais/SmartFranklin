/**
 * @file gaz.h
 * @brief Mesure du poids de bouteille de gaz : acquisition capteur poids I2C, calibration, publication MQTT.
 *
 * Utilise Wire1 directement avec les parametres (pins SDA/SCL, adresse I2C) provenant de sf_interfaces.
 * Aucune abstraction de routage n'est exposee dans l'API publique.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <Arduino.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedWEIGHT.h>

#include <mutex>

#include "interfaces.h"

class Gaz {
public:
    bool init();
    bool process();
    bool isInitialized() const;

    bool tare();
    bool calibrate(float weightG);

private:
    mutable std::mutex m_mutex;

    m5::unit::UnitUnified    m_units;
    m5::unit::UnitWeightI2C  m_unit;

    bool    m_initialized             = false;
    bool    m_calibration_in_progress = false;
    
	const sf_interfaces::InterfaceSensor m_sensor = sf_interfaces::InterfaceSensor::Gaz;
    const char* const m_tag = sf_interfaces::toString(m_sensor, true);
    const char* const m_device = sf_interfaces::getDeviceName(m_sensor);

    static constexpr int32_t  GAZ_BOTTLE_FULL_G  = 6450;
    static constexpr int32_t  GAZ_BOTTLE_EMPTY_G = 3700;

    static float   sanitizedGap(float gap);
};

/** Instance globale utilisee par les taches runtime et APIs. */
extern Gaz GAZ_TASK;

bool scale_tare();
bool scale_calibrate(float weightG);
