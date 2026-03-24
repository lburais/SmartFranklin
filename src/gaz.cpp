/*
 * SmartFranklin - gas sensor module implementation
 * SPDX-License-Identifier: MIT
 */

#include "gaz.h"

#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedWEIGHT.h>
#include <M5Utility.h>

#include <cmath>
#include <cstdio>
#include <mutex>

#include "config_store.h"
#include "data_model.h"
#include "mqtt.h"

namespace {

constexpr int32_t GAZ_BOTTLE_FULL_G = 6450;
constexpr int32_t GAZ_BOTTLE_EMPTY_G = 3700;

float sanitizedGap(const float gap)
{
    if (!std::isfinite(gap) || gap == 0.0f) {
        return 1.0f;
    }
    return gap;
}

struct GazState {
    mutable std::mutex mutex;
    m5::unit::UnitUnified units;
    m5::unit::UnitWeightI2C unit;

    bool initialized = false;
    uint8_t i2cAddress = 0x26;
    float lastCalibrationGap = 1.0f;
    int32_t lastWeightG = 0;
    int32_t lastFillPct = 0;
};

GazState GAZ_STATE;

}  // namespace

Gaz GAZ_MODULE;

static void publishCalibrationGap(const float gap);

static bool initState(GazState& state, const bool isInternalRoute, const uint8_t i2cAddress)
{
    std::lock_guard<std::mutex> lock(state.mutex);
    state.i2cAddress = i2cAddress;

    state.units = m5::unit::UnitUnified{};

    if (isInternalRoute) {
        state.initialized = state.units.add(state.unit, M5.Ex_I2C) && state.units.begin();
    } else {
        state.initialized = state.units.add(state.unit, Wire) && state.units.begin();
    }

    if (!state.initialized) {
        state.initialized = false;
        M5_LOGW("[GAZ] Weight I2C unit was not detected on supported Wire paths");
        return false;
    }

    const float effectiveGap = sanitizedGap(CONFIG.scale_cal_factor);
    if (!state.unit.writeGap(effectiveGap)) {
        M5_LOGW("[GAZ] failed to apply calibration gap %.6f during init", effectiveGap);
    }
    state.lastCalibrationGap = effectiveGap;
    publishCalibrationGap(effectiveGap);

    M5_LOGI("[GAZ] Weight I2C initialization complete");
    M5_LOGI("[GAZ] address: 0x%02X", state.i2cAddress);
    M5_LOGI("%s", state.units.debugInfo().c_str());
    return true;
}

static void publishCalibrationGap(const float gap)
{
    char gapBuf[24] = {0};
    snprintf(gapBuf, sizeof(gapBuf), "%.6f", gap);
    sf_mqtt::publish("smartfranklin/gaz/calibration/gap", gapBuf, 1, true);
}

static bool refreshMeasurementLocked(GazState& state, int32_t& weightG, int32_t& fillPct)
{
    state.units.update();

    if (!state.unit.updated()) {
        return false;
    }

    weightG = state.unit.weight();
    if (!std::isfinite(weightG)) {
        M5_LOGW("[GAZ] non-finite weight sample ignored");
        return false;
    }

    state.lastWeightG = weightG;

    fillPct = 0;
    if (weightG <= GAZ_BOTTLE_EMPTY_G) {
        fillPct = 0;
    } else if (weightG >= GAZ_BOTTLE_FULL_G) {
        fillPct = 100;
    } else {
        fillPct = 100 * (static_cast<float>(weightG - GAZ_BOTTLE_EMPTY_G) / (GAZ_BOTTLE_FULL_G - GAZ_BOTTLE_EMPTY_G));
    }

    state.lastFillPct = fillPct;

    return true;
}

static void publishWeight(const int32_t weightG, const int32_t fillPct)
{
    char gBuf[24] = {0};
    snprintf(gBuf, sizeof(gBuf), "%d", weightG);
    sf_mqtt::publish("smartfranklin/gaz/g", gBuf);

    char pctBuf[16] = {0};
    snprintf(pctBuf, sizeof(pctBuf), "%d", fillPct);
    sf_mqtt::publish("smartfranklin/gaz/fill", pctBuf, 1, true);

    M5_LOGI("[GAZ] Weight: %d g     Fill level: %d%%", weightG, fillPct);
}

static void processState(GazState& state)
{
    int32_t weightG = 0;
    int32_t fillPct = 0;
    float desiredGap = 1.0f;
    bool hasMeasurement = false;

    {
        std::lock_guard<std::mutex> lock(state.mutex);
        if (!state.initialized) {
            return;
        }

        desiredGap = sanitizedGap(CONFIG.scale_cal_factor);
        if (desiredGap != state.lastCalibrationGap) {
            if (state.unit.writeGap(desiredGap)) {
                state.lastCalibrationGap = desiredGap;
                publishCalibrationGap(desiredGap);
            } else {
                M5_LOGW("[GAZ] failed to refresh calibration gap %.6f", desiredGap);
            }
        }

        hasMeasurement = refreshMeasurementLocked(state, weightG, fillPct);
    }

    if (!hasMeasurement) {
        M5_LOGW("[GAZ] No measurement");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(DATA_MUTEX);
        DATA.weight_gaz = weightG;
        DATA.fill_gaz = fillPct;
    }

    publishWeight(weightG, fillPct);
}

static bool tareState(GazState& state)
{
    std::lock_guard<std::mutex> lock(state.mutex);
    if (!state.initialized) {
        return false;
    }
    const bool ok = state.unit.resetOffset();
    if (ok) {
        state.lastWeightG = 0;
    }
    return ok;
}

static bool applyCalibrationState(GazState& state, const float gap)
{
    const float effectiveGap = sanitizedGap(gap);

    std::lock_guard<std::mutex> lock(state.mutex);
    if (!state.initialized) {
        return false;
    }
    if (!state.unit.writeGap(effectiveGap)) {
        return false;
    }
    state.lastCalibrationGap = effectiveGap;
    publishCalibrationGap(effectiveGap);
    return true;
}

static float readCalibrationSampleState(GazState& state)
{
    std::lock_guard<std::mutex> lock(state.mutex);
    if (!state.initialized) {
        return 0.0f;
    }

    int32_t weightG = 0;
    int32_t fillPct = 0;
    if (refreshMeasurementLocked(state, weightG, fillPct)) {
        return weightG;
    }

    return state.lastWeightG;
}

static bool isInitializedState(const GazState& state)
{
    std::lock_guard<std::mutex> lock(state.mutex);
    return state.initialized;
}

bool Gaz::init(bool isInternalRoute, uint8_t i2cAddress)
{
    return initState(GAZ_STATE, isInternalRoute, i2cAddress);
}

void Gaz::process()
{
    processState(GAZ_STATE);
}

bool Gaz::tare()
{
    return tareState(GAZ_STATE);
}

bool Gaz::applyCalibration(const float gap)
{
    return applyCalibrationState(GAZ_STATE, gap);
}

float Gaz::readCalibrationSample()
{
    return readCalibrationSampleState(GAZ_STATE);
}

bool Gaz::isInitialized() const
{
    return isInitializedState(GAZ_STATE);
}

float scale_get_raw()
{
    return GAZ_MODULE.readCalibrationSample();
}

void scale_tare()
{
    if (!GAZ_MODULE.isInitialized()) {
        return;
    }

    if (GAZ_MODULE.tare()) {
        GAZ_MODULE.applyCalibration(1.0f);
    }
}

void scale_set_cal_factor(float factor)
{
    if (!GAZ_MODULE.isInitialized()) {
        return;
    }

    if (GAZ_MODULE.applyCalibration(factor)) {
        CONFIG.scale_cal_factor = factor;
    }
}