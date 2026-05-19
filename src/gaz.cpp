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

namespace {

constexpr uint8_t kWeightFirmwareVersionReg = 0xFE;

bool readRegister8(TwoWire& bus, const uint8_t address, const uint8_t reg, uint8_t& value)
{
    bus.beginTransmission(address);
    bus.write(reg);
    if (bus.endTransmission(false) != 0) {
        return false;
    }

    if (bus.requestFrom(static_cast<int>(address), 1) != 1 || bus.available() <= 0) {
        return false;
    }

    value = static_cast<uint8_t>(bus.read());
    return true;
}

}  // namespace

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

bool Gaz::refreshMeasurement(int32_t& weightG, int32_t& fillPct)
{
    M5_LOGI("[GAZ] refreshMeasurement");

    m_units.update();

    if (!m_unit.updated()) {
        M5_LOGW("[GAZ] not updated");
        return false;
    }

    const float rawWeight = m_unit.weight();
    if (!std::isfinite(rawWeight)) {
        M5_LOGW("[GAZ] non-finite weight sample ignored");
        return false;
    }

    weightG = static_cast<int32_t>(lroundf(rawWeight));
    m_lastWeightG = weightG;

    fillPct = computeFillPct(weightG);
    m_lastFillPct = fillPct;
    return true;
}

// ---- public interface ----

bool Gaz::isInitialized() const
{
    return m_initialized;
}

bool Gaz::calibrate()
{
    M5_LOGI("[GAZ] calibrate");

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

    {
        std::lock_guard<std::mutex> lock(DATA_MUTEX);
        DATA.weight_gaz = 0;
        DATA.fill_gaz   = 0;
    }

    return true;
}

bool Gaz::tare()
{
    M5_LOGI("[GAZ] tare");

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
    M5_LOGI("[GAZ] applyCalibration");

    const float effectiveGap = sanitizedGap(gap);
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
    M5_LOGI("[GAZ] readCalibrationSample");

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
    M5_LOGI("[GAZ] readCalibrationGap");

    if (!m_initialized) {
        return false;
    }
    return m_unit.readGap(gap);
}

bool Gaz::readRawAdc(int32_t& rawAdc)
{
    M5_LOGI("[GAZ] readRawAdc");

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

bool Gaz::init()
{
    M5_LOGI("[GAZ] init");

    m_initialized        = false;
    m_lastWeightG        = 0;
    m_lastFillPct        = 0;
    m_lastCalibrationGap = 1.0f;

    const bool configured = sf_interfaces::configured(sf_interfaces::InterfaceSensor::Gaz);

    if (!configured) {
        if (!sf_interfaces::configure(sf_interfaces::InterfaceSensor::Gaz)) {
            M5_LOGW("[GAZ] configure failed");
            return false;
        }
    }

    TwoWire* connector = sf_interfaces::getPort(sf_interfaces::InterfaceSensor::Gaz).ptr.twoWire;
    if (connector == nullptr) {
        M5_LOGW("[GAZ] connector unavailable");
        return false;
    }

    if (connector == &Wire1) {
        M5_LOGI("[GAZ] connector is on Wire1");
    } else if (connector == &Wire) {
        M5_LOGI("[GAZ] connector is on Wire");
    } else {
        M5_LOGI("[GAZ] connector is custom bus ptr=%p", connector);
    }

    const uint8_t address = sf_interfaces::getAddress(sf_interfaces::InterfaceSensor::Gaz);
    uint8_t firmwareVersion = 0;
    if (readRegister8(*connector, address, kWeightFirmwareVersionReg, firmwareVersion) && firmwareVersion != 0U) {
        M5_LOGI("[GAZ] preflight firmware version=0x%02X addr=0x%02X bus=%p", firmwareVersion, address, connector);
    } else {
        M5_LOGW("[GAZ] preflight firmware read failed addr=0x%02X bus=%p", address, connector);
    }

    M5_LOGI("[GAZ] init connector ptr=%p", connector);
    const bool ok = m_units.add(m_unit, *connector) && m_units.begin();

    M5_LOGI("[GAZ] connector setup");

    if (!ok) {
        M5_LOGW("[GAZ] Weight I2C m_unit not added");
        return false;
    } else {
        M5_LOGI("[GAZ] Weight I2C m_unit added");
    }

    const float effectiveGap = sanitizedGap(CONFIG.gaz_calibration_factor);
    if (!m_unit.writeGap(effectiveGap)) {
        M5_LOGW("[GAZ] failed to apply calibration gap %.6f during init", effectiveGap);
    }
    m_lastCalibrationGap = effectiveGap;
    publishCalibrationGap(effectiveGap);

    m_initialized = true;
    M5_LOGI("[GAZ] (0x%02X) initialized", sf_interfaces::getAddress(sf_interfaces::InterfaceSensor::Gaz));
    return true;
}

void Gaz::process()
{
    int32_t weightG        = 0;
    int32_t fillPct        = 0;
    bool    hasMeasurement = false;

    if (!m_initialized) {
        if (!sf_interfaces::configure(sf_interfaces::InterfaceSensor::Gaz)) {
            M5_LOGW("[GAZ] configure failed during process");
            hmiSetPortLedStatus(sf_interfaces::InterfaceSensor::Gaz, m_initialized, true);
            return;
        }
    }

    const float desiredGap = sanitizedGap(CONFIG.gaz_calibration_factor);
    if (fabsf(desiredGap - m_lastCalibrationGap) > kGapEpsilon) {
        if (m_unit.writeGap(desiredGap)) {
            m_lastCalibrationGap = desiredGap;
            publishCalibrationGap(desiredGap);
        } else {
            M5_LOGW("[GAZ] failed to refresh calibration gap %.6f", desiredGap);
            hmiSetPortLedStatus(sf_interfaces::InterfaceSensor::Gaz, m_initialized, true);
        }
    }

    hasMeasurement = refreshMeasurement(weightG, fillPct);

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
        nextAttemptMs = nowMs + 10000UL;  // 10 second retry interval
    };

    for (;;) {
        const uint32_t nowMs = millis();

        if (!initialized && isRetryDue(nowMs, nextInitAttemptMs)) {
            initialized = GAZ_TASK.init();
            hmiSetPortLedStatus(sf_interfaces::InterfaceSensor::Gaz, initialized, false);
            if (!initialized) {
                M5_LOGW("[GAZ] Init failed");
                scheduleRetry(nextInitAttemptMs, nowMs);
            }
        }

        if (initialized) {
            GAZ_TASK.process();
        }

        const uint32_t recurrenceMs = sf_interfaces::getRecurrenceMs(sf_interfaces::InterfaceSensor::Gaz);
        const int loopMs = (recurrenceMs > 0) ? static_cast<int>(recurrenceMs) : 1000;
        vTaskDelay(pdMS_TO_TICKS(loopMs));

    }
}
