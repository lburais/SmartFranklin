/**
 * @file gaz.h
 * @brief Mesure du poids de bouteille de gaz : acquisition capteur poids I2C, calibration, publication MQTT.
 *
 * Le port I2C utilise est resolu depuis CONFIG via le port dont le capteur est "gaz".
 * Aucune source externe ni RouteMode n'est expose dans l'API publique.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <Arduino.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedWEIGHT.h>

#include <array>
#include <mutex>

class Gaz {
public:
    bool init();
    void process();
    bool isInitialized() const;

    /** Tare le capteur, remet le facteur de calibration a 1.0 et persiste dans CONFIG. */
    bool calibrate();

    bool  tare();
    bool  applyCalibration(float gap);
    float readCalibrationSample();
    bool  readCalibrationGap(float& gap);
    bool  readRawAdc(int32_t& rawAdc);

private:
    static constexpr int32_t  kBottleFullG    = 6450;
    static constexpr int32_t  kBottleEmptyG   = 3700;
    static constexpr float    kGapEpsilon     = 1e-6f;
    static constexpr size_t   kAvgWindowMin   = 1U;
    static constexpr size_t   kAvgWindowMax   = 64U;
    static constexpr uint8_t  kI2CAddress     = 0x26;

    static float   sanitizedGap(float gap);
    static int32_t computeFillPct(int32_t weightG);
    static void    publishWeight(int32_t weightG, int32_t fillPct);
    static void    publishCalibrationGap(float gap);

    size_t  sanitizedAveragingWindow() const;
    int32_t pushAndAverage(int32_t rawWeightG);
    bool    refreshMeasurement(int32_t& weightG, int32_t& fillPct);

    mutable std::mutex m_mutex;

    m5::unit::UnitUnified    m_units;
    m5::unit::UnitWeightI2C  m_unit;

    bool    m_initialized        = false;
    float   m_lastCalibrationGap = 1.0f;
    int32_t m_lastWeightG        = 0;
    int32_t m_lastFillPct        = 0;

    std::array<int32_t, kAvgWindowMax> m_recentWeights{};
    size_t m_recentCount  = 0;
    size_t m_recentHead   = 0;
    size_t m_recentWindow = 0;
};

/** Instance globale utilisee par les taches runtime et APIs. */
extern Gaz GAZ_TASK;

/** Lit un echantillon de poids brut pour le workflow de calibration. */
float scale_get_raw();

/** Effectue la tare (remise a zero de l'offset). */
bool scale_tare();

/** Applique un facteur de calibration au capteur. */
bool scale_set_cal_factor(float factor);

/** Lit le facteur de calibration courant depuis le firmware capteur. */
bool scale_get_cal_factor(float& gap);

/** Lit la valeur ADC brute depuis le firmware capteur. */
bool scale_get_raw_adc(int32_t& rawAdc);
