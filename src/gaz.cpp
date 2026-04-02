/**
 * @file gaz.cpp
 * @brief Poids bouteille gaz : acquisition capteur poids I2C, calibration, publication MQTT.
 * @details
 * Ce module gere le capteur M5Stack Weight I2C connecte sur le port dont CONFIG.port_*_sensor vaut "gaz".
 * Il effectue les mesures periodiques, calcule le taux de remplissage, applique la calibration
 * et publie les donnees via MQTT.
 */

#include "gaz.h"

#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedWEIGHT.h>
#include <M5Utility.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <mutex>

#include "config_store.h"
#include "data_model.h"
#include "hmi.h"
#include "i2c.h"
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

int32_t Gaz::computeFillPct(const int32_t weightG)
{
    if (weightG <= kBottleEmptyG) { return 0; }
    if (weightG >= kBottleFullG)  { return 100; }
    return static_cast<int32_t>(100.0f *
        static_cast<float>(weightG - kBottleEmptyG) /
        static_cast<float>(kBottleFullG - kBottleEmptyG));
}

void Gaz::publishCalibrationGap(const float gap)
{
    char buf[24] = {0};
    snprintf(buf, sizeof(buf), "%.6f", gap);
    sf_mqtt::publish("smartfranklin/gaz/calibration/gap", buf, 1, true);
}

void Gaz::publishWeight(const int32_t weightG, const int32_t fillPct)
{
    char gBuf[24] = {0};
    snprintf(gBuf, sizeof(gBuf), "%d", weightG);
    sf_mqtt::publish("smartfranklin/gaz/g", gBuf);

    char pctBuf[16] = {0};
    snprintf(pctBuf, sizeof(pctBuf), "%d", fillPct);
    sf_mqtt::publish("smartfranklin/gaz/fill", pctBuf, 1, true);

    M5_LOGI("[GAZ] Weight: %d g     Fill: %d%%", weightG, fillPct);
}

// ---- instance methods ----

size_t Gaz::sanitizedAveragingWindow() const
{
    const int configured = CONFIG.gaz_weight_average_window;
    if (configured < static_cast<int>(kAvgWindowMin)) { return kAvgWindowMin; }
    if (configured > static_cast<int>(kAvgWindowMax)) { return kAvgWindowMax; }
    return static_cast<size_t>(configured);
}

int32_t Gaz::pushAndAverage(const int32_t rawWeightG)
{
    const size_t window = sanitizedAveragingWindow();

    if (m_recentWindow != window) {
        m_recentWindow = window;
        m_recentCount  = 0;
        m_recentHead   = 0;
    }

    if (m_recentCount > window) {
        m_recentCount = window;
    }

    m_recentWeights[m_recentHead] = rawWeightG;
    m_recentHead = (m_recentHead + 1U) % window;
    if (m_recentCount < window) {
        ++m_recentCount;
    }

    int64_t sum = 0;
    for (size_t i = 0; i < m_recentCount; ++i) {
        sum += static_cast<int64_t>(m_recentWeights[i]);
    }
    return static_cast<int32_t>(sum / static_cast<int64_t>(m_recentCount));
}

bool Gaz::refreshMeasurement(int32_t& weightG, int32_t& fillPct)
{
    m_units.update();

    if (!m_unit.updated()) {
        return false;
    }

    const float rawWeight = m_unit.weight();
    if (!std::isfinite(rawWeight)) {
        M5_LOGW("[GAZ] non-finite weight sample ignored");
        return false;
    }

    const int32_t rawWeightG = static_cast<int32_t>(lroundf(rawWeight));
    weightG = pushAndAverage(rawWeightG);
    m_lastWeightG = weightG;

    fillPct = computeFillPct(weightG);
    m_lastFillPct = fillPct;
    return true;
}

// ---- public interface ----

bool Gaz::isInitialized() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_initialized;
}

bool Gaz::init()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    m_initialized        = false;
    m_recentCount        = 0;
    m_recentHead         = 0;
    m_recentWindow       = 0;
    m_lastWeightG        = 0;
    m_lastFillPct        = 0;
    m_lastCalibrationGap = 1.0f;

    if (!sf_i2c::i2cBeginConfiguredPort(sf_ports::PortSensor::Gaz)) {
        return false;
    }

    if (!sf_i2c::i2cDeviceExistsOnConfiguredPort(sf_ports::PortSensor::Gaz, kI2CAddress)) {
        M5_LOGW("[GAZ] sensor (0x%02X) not found", kI2CAddress);
        return false;
    }

    sf_i2c::i2cPublishConfiguration(sf_ports::PortSensor::Gaz, kI2CAddress);

    m_units = m5::unit::UnitUnified{};
    const bool ok = m_units.add(m_unit, Wire) && m_units.begin();

    if (!ok) {
        M5_LOGW("[GAZ] Weight I2C unit not detected");
        return false;
    }

    const float effectiveGap = sanitizedGap(CONFIG.gaz_calibration_factor);
    if (!m_unit.writeGap(effectiveGap)) {
        M5_LOGW("[GAZ] failed to apply calibration gap %.6f during init", effectiveGap);
    }
    m_lastCalibrationGap = effectiveGap;
    publishCalibrationGap(effectiveGap);

    m_initialized = true;
    M5_LOGI("[GAZ] (0x%02X) initialized", kI2CAddress);
    return true;
}

bool Gaz::calibrate()
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        if (!m_initialized) {
            M5_LOGW("[GAZ] calibrate called before init");
            return false;
        }

        if (!m_unit.resetOffset()) {
            M5_LOGW("[GAZ] tare (resetOffset) failed during calibrate");
            return false;
        }
        m_lastWeightG = 0;

        const float defaultGap = 1.0f;
        if (!m_unit.writeGap(defaultGap)) {
            M5_LOGW("[GAZ] failed to reset calibration gap during calibrate");
            return false;
        }
        m_lastCalibrationGap          = defaultGap;
        CONFIG.gaz_calibration_factor = defaultGap;
        config_save();

        publishCalibrationGap(defaultGap);
        M5_LOGI("[GAZ] calibrated: tare done, gap reset to %.1f", defaultGap);
    }

    {
        std::lock_guard<std::mutex> lock(DATA_MUTEX);
        DATA.weight_gaz = 0;
        DATA.fill_gaz   = 0;
    }

    return true;
}

void Gaz::process()
{
    int32_t weightG        = 0;
    int32_t fillPct        = 0;
    bool    hasMeasurement = false;

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_initialized) {
            return;
        }

        const float desiredGap = sanitizedGap(CONFIG.gaz_calibration_factor);
        if (fabsf(desiredGap - m_lastCalibrationGap) > kGapEpsilon) {
            if (m_unit.writeGap(desiredGap)) {
                m_lastCalibrationGap = desiredGap;
                publishCalibrationGap(desiredGap);
            } else {
                M5_LOGW("[GAZ] failed to refresh calibration gap %.6f", desiredGap);
            }
        }

        hasMeasurement = refreshMeasurement(weightG, fillPct);
    }

    if (!hasMeasurement) {
        M5_LOGD("[GAZ] no new measurement");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(DATA_MUTEX);
        DATA.weight_gaz = weightG;
        DATA.fill_gaz   = fillPct;
    }

    publishWeight(weightG, fillPct);
}

bool Gaz::tare()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_initialized) {
        return false;
    }
    const bool ok = m_unit.resetOffset();
    if (ok) {
        m_lastWeightG = 0;
    }
    return ok;
}

bool Gaz::applyCalibration(const float gap)
{
    const float effectiveGap = sanitizedGap(gap);
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_initialized) {
        return false;
    }
    if (!m_unit.writeGap(effectiveGap)) {
        return false;
    }
    m_lastCalibrationGap = effectiveGap;
    publishCalibrationGap(effectiveGap);
    return true;
}

float Gaz::readCalibrationSample()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_initialized) {
        return 0.0f;
    }
    int32_t weightG = 0;
    int32_t fillPct = 0;
    if (refreshMeasurement(weightG, fillPct)) {
        return static_cast<float>(weightG);
    }
    return static_cast<float>(m_lastWeightG);
}

bool Gaz::readCalibrationGap(float& gap)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_initialized) {
        return false;
    }
    return m_unit.readGap(gap);
}

bool Gaz::readRawAdc(int32_t& rawAdc)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_initialized) {
        return false;
    }
    return m_unit.readRawADC(rawAdc);
}

// ---- free functions for web_dashboard calibration API ----

float scale_get_raw()
{
    return GAZ_TASK.readCalibrationSample();
}

bool scale_tare()
{
    if (!GAZ_TASK.isInitialized()) {
        return false;
    }
    if (!GAZ_TASK.tare()) {
        return false;
    }
    CONFIG.gaz_calibration_factor = 1.0f;
    if (!GAZ_TASK.applyCalibration(CONFIG.gaz_calibration_factor)) {
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(DATA_MUTEX);
        DATA.weight_gaz = 0;
        DATA.fill_gaz   = 0;
    }
    return true;
}

bool scale_set_cal_factor(float factor)
{
    if (!GAZ_TASK.isInitialized()) {
        return false;
    }
    if (GAZ_TASK.applyCalibration(factor)) {
        float appliedGap = factor;
        if (GAZ_TASK.readCalibrationGap(appliedGap) &&
            std::isfinite(appliedGap) &&
            std::fabs(appliedGap) > 1e-6f)
        {
            CONFIG.gaz_calibration_factor = appliedGap;
        } else {
            CONFIG.gaz_calibration_factor = factor;
        }
        return true;
    }
    return false;
}

bool scale_get_cal_factor(float& gap)
{
    if (!GAZ_TASK.isInitialized()) {
        return false;
    }
    return GAZ_TASK.readCalibrationGap(gap);
}

bool scale_get_raw_adc(int32_t& rawAdc)
{
    if (!GAZ_TASK.isInitialized()) {
        return false;
    }
    return GAZ_TASK.readRawAdc(rawAdc);
}

// ---- FreeRTOS task ----

void taskGaz(void* pv)
{
    (void)pv;
    M5_LOGI("[GAZ] Task started");

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
            initialized = GAZ_TASK.init();
            hmiSetPortLedStatus(sf_ports::PortSensor::Gaz, initialized, false);
            if (!initialized) {
                M5_LOGW("[GAZ] Init failed");
                scheduleRetry(nextInitAttemptMs, nowMs);
            }
        }

        if (initialized) {
            GAZ_TASK.process();
        }

        const int loopMs = (CONFIG.task_i2c_loop_ms > 0) ? CONFIG.task_i2c_loop_ms : 1000;
        vTaskDelay(pdMS_TO_TICKS(loopMs));
    }
}
