/**
 * @file gaz.cpp
 * @brief Gas bottle weight acquisition and calibration implementation.
 *
 * This module wraps M5 Unit WEIGHT operations, keeps calibration state,
 * computes gas fill percentage from measured weight, and publishes telemetry.
 *
 * SPDX-License-Identifier: MIT
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
#include "mqtt.h"

namespace {

constexpr int32_t GAZ_BOTTLE_FULL_G = 6450;
constexpr int32_t GAZ_BOTTLE_EMPTY_G = 3700;
constexpr float CALIBRATION_GAP_EPSILON = 1e-6f;
constexpr uint8_t PAHUB_ADDRESS = 0x70;
constexpr size_t GAZ_AVG_WINDOW_MIN = 1U;
constexpr size_t GAZ_AVG_WINDOW_MAX = 64U;

/** Normalize invalid calibration gaps to a safe default. */
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
    sf_i2c::RouteMode routeMode = sf_i2c::RouteMode::Unset;
    int8_t paHubChannel = -1;
    float lastCalibrationGap = 1.0f;
    int32_t lastWeightG = 0;
    int32_t lastFillPct = 0;
    std::array<int32_t, GAZ_AVG_WINDOW_MAX> recentWeights{};
    size_t recentCount = 0;
    size_t recentHead = 0;
    size_t recentWindow = 0;
};

GazState GAZ_STATE;

bool selectPaHubChannelLocked(const GazState& state)
{
    if (!sf_i2c::isPaHubRoute(state.routeMode)) {
        return true;
    }

    if (state.paHubChannel < 0 || state.paHubChannel > 7) {
        M5_LOGW("[GAZ] invalid PAHub channel %d", state.paHubChannel);
        return false;
    }

    const uint8_t mask = static_cast<uint8_t>(1U << static_cast<uint8_t>(state.paHubChannel));

    if (state.routeMode == sf_i2c::RouteMode::InternalPaHub) {
        if (!M5.Ex_I2C.start(PAHUB_ADDRESS, false, Wire.getClock())) {
            return false;
        }
        const bool ok = M5.Ex_I2C.write(mask) && M5.Ex_I2C.stop();
        return ok;
    }

    Wire.beginTransmission(PAHUB_ADDRESS);
    Wire.write(mask);
    return Wire.endTransmission() == 0;
}

size_t sanitizedAveragingWindow()
{
    const int configured = CONFIG.gaz_weight_average_window;
    if (configured < static_cast<int>(GAZ_AVG_WINDOW_MIN)) {
        return GAZ_AVG_WINDOW_MIN;
    }
    if (configured > static_cast<int>(GAZ_AVG_WINDOW_MAX)) {
        return GAZ_AVG_WINDOW_MAX;
    }
    return static_cast<size_t>(configured);
}

int32_t pushAndAverageWeightLocked(GazState& state, const int32_t rawWeightG)
{
    const size_t window = sanitizedAveragingWindow();

    if (state.recentWindow != window) {
        state.recentWindow = window;
        state.recentCount = 0;
        state.recentHead = 0;
    }

    if (state.recentCount > window) {
        state.recentCount = window;
    }

    state.recentWeights[state.recentHead] = rawWeightG;
    state.recentHead = (state.recentHead + 1U) % window;
    if (state.recentCount < window) {
        ++state.recentCount;
    }

    int64_t sum = 0;
    for (size_t i = 0; i < state.recentCount; ++i) {
        sum += static_cast<int64_t>(state.recentWeights[i]);
    }
    return static_cast<int32_t>(sum / static_cast<int64_t>(state.recentCount));
}

void disablePaHubChannelLocked(const GazState& state)
{
    if (!sf_i2c::isPaHubRoute(state.routeMode)) {
        return;
    }

    if (state.routeMode == sf_i2c::RouteMode::InternalPaHub) {
        if (M5.Ex_I2C.start(PAHUB_ADDRESS, false, Wire.getClock())) {
            M5.Ex_I2C.write(0x00);
            M5.Ex_I2C.stop();
        }
        return;
    }

    Wire.beginTransmission(PAHUB_ADDRESS);
    Wire.write(0x00);
    Wire.endTransmission();
}

}  // namespace

Gaz GAZ_MODULE;

static void publishCalibrationGap(const float gap);

/** Initialize weight unit and apply persisted calibration. */
static bool initState(GazState& state,
                      const bool isInternalRoute,
                      const uint8_t i2cAddress,
                      const sf_i2c::RouteMode routeMode,
                      const int8_t paHubChannel)
{
    std::lock_guard<std::mutex> lock(state.mutex);
    state.i2cAddress = i2cAddress;
    state.routeMode = routeMode;
    state.paHubChannel = paHubChannel;

    if (!selectPaHubChannelLocked(state)) {
        M5_LOGW("[GAZ] failed to select PAHub channel during init");
        state.initialized = false;
        return false;
    }

    state.units = m5::unit::UnitUnified{};

    if (isInternalRoute) {
        state.initialized = state.units.add(state.unit, M5.Ex_I2C) && state.units.begin();
    } else {
        state.initialized = state.units.add(state.unit, Wire) && state.units.begin();
    }

    if (!state.initialized) {
        state.initialized = false;
        disablePaHubChannelLocked(state);
        M5_LOGW("[GAZ] Weight I2C unit was not detected on supported Wire paths");
        return false;
    }

    const float effectiveGap = sanitizedGap(CONFIG.gaz_calibration_factor);
    if (!state.unit.writeGap(effectiveGap)) {
        M5_LOGW("[GAZ] failed to apply calibration gap %.6f during init", effectiveGap);
    }
    state.lastCalibrationGap = effectiveGap;
    publishCalibrationGap(effectiveGap);
    disablePaHubChannelLocked(state);

    M5_LOGI("[GAZ] Weight I2C initialization complete");
    M5_LOGI("[GAZ] address: 0x%02X", state.i2cAddress);
    M5_LOGI("%s", state.units.debugInfo().c_str());
    return true;
}

/** Publish current calibration gap factor. */
static void publishCalibrationGap(const float gap)
{
    char gapBuf[24] = {0};
    snprintf(gapBuf, sizeof(gapBuf), "%.6f", gap);
    sf_mqtt::publish("smartfranklin/gaz/calibration/gap", gapBuf, 1, true);
}

/** Acquire one weight sample and derive fill percentage under lock. */
static bool refreshMeasurementLocked(GazState& state, int32_t& weightG, int32_t& fillPct)
{
    if (!selectPaHubChannelLocked(state)) {
        return false;
    }

    state.units.update();

    if (!state.unit.updated()) {
        disablePaHubChannelLocked(state);
        return false;
    }

    const float rawWeight = state.unit.weight();
    if (!std::isfinite(rawWeight)) {
        disablePaHubChannelLocked(state);
        M5_LOGW("[GAZ] non-finite weight sample ignored");
        return false;
    }

    const int32_t rawWeightG = static_cast<int32_t>(lroundf(rawWeight));
    weightG = pushAndAverageWeightLocked(state, rawWeightG);
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
    disablePaHubChannelLocked(state);

    return true;
}

/** Publish weight and fill telemetry topics. */
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

/** Execute one process cycle for the gas/weight module. */
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

        desiredGap = sanitizedGap(CONFIG.gaz_calibration_factor);
        if (fabsf(desiredGap - state.lastCalibrationGap) > CALIBRATION_GAP_EPSILON) {
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

/** Perform tare operation through unit API. */
static bool tareState(GazState& state)
{
    std::lock_guard<std::mutex> lock(state.mutex);
    if (!state.initialized) {
        return false;
    }

    if (!selectPaHubChannelLocked(state)) {
        return false;
    }

    const bool ok = state.unit.resetOffset();
    disablePaHubChannelLocked(state);
    if (ok) {
        state.lastWeightG = 0;
    }
    return ok;
}

/** Apply runtime calibration factor to the sensor. */
static bool applyCalibrationState(GazState& state, const float gap)
{
    const float effectiveGap = sanitizedGap(gap);

    std::lock_guard<std::mutex> lock(state.mutex);
    if (!state.initialized) {
        return false;
    }

    if (!selectPaHubChannelLocked(state)) {
        return false;
    }

    if (!state.unit.writeGap(effectiveGap)) {
        disablePaHubChannelLocked(state);
        return false;
    }
    disablePaHubChannelLocked(state);
    state.lastCalibrationGap = effectiveGap;
    publishCalibrationGap(effectiveGap);
    return true;
}

/** Read one sample for calibration workflow, with fallback to cached value. */
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

/** Read current calibration gap directly from sensor firmware. */
static bool readCalibrationGapState(GazState& state, float& gap)
{
    std::lock_guard<std::mutex> lock(state.mutex);
    if (!state.initialized) {
        return false;
    }

    if (!selectPaHubChannelLocked(state)) {
        return false;
    }
    const bool ok = state.unit.readGap(gap);
    disablePaHubChannelLocked(state);
    return ok;
}

/** Read raw ADC count directly from sensor firmware. */
static bool readRawAdcState(GazState& state, int32_t& rawAdc)
{
    std::lock_guard<std::mutex> lock(state.mutex);
    if (!state.initialized) {
        return false;
    }

    if (!selectPaHubChannelLocked(state)) {
        return false;
    }
    const bool ok = state.unit.readRawADC(rawAdc);
    disablePaHubChannelLocked(state);
    return ok;
}

/** Return initialization status from protected state. */
static bool isInitializedState(const GazState& state)
{
    std::lock_guard<std::mutex> lock(state.mutex);
    return state.initialized;
}

bool Gaz::init(bool isInternalRoute,
               uint8_t i2cAddress,
               sf_i2c::RouteMode routeMode,
               int8_t paHubChannel)
{
    return initState(GAZ_STATE, isInternalRoute, i2cAddress, routeMode, paHubChannel);
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

bool Gaz::readCalibrationGap(float& gap)
{
    return readCalibrationGapState(GAZ_STATE, gap);
}

bool Gaz::readRawAdc(int32_t& rawAdc)
{
    return readRawAdcState(GAZ_STATE, rawAdc);
}

bool Gaz::isInitialized() const
{
    return isInitializedState(GAZ_STATE);
}

float scale_get_raw()
{
    return GAZ_MODULE.readCalibrationSample();
}

bool scale_tare()
{
    if (!GAZ_MODULE.isInitialized()) {
        return false;
    }

    if (!GAZ_MODULE.tare()) {
        return false;
    }

    // Keep runtime configuration aligned with tare baseline so process() does not
    // immediately restore an old factor on the next cycle.
    CONFIG.gaz_calibration_factor = 1.0f;
    if (!GAZ_MODULE.applyCalibration(CONFIG.gaz_calibration_factor)) {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(DATA_MUTEX);
        DATA.weight_gaz = 0;
        DATA.fill_gaz = 0;
    }

    return true;
}

bool scale_set_cal_factor(float factor)
{
    if (!GAZ_MODULE.isInitialized()) {
        return false;
    }

    if (GAZ_MODULE.applyCalibration(factor)) {
        float appliedGap = factor;
        if (GAZ_MODULE.readCalibrationGap(appliedGap) && std::isfinite(appliedGap) && std::fabs(appliedGap) > 1e-6f) {
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
    if (!GAZ_MODULE.isInitialized()) {
        return false;
    }
    return GAZ_MODULE.readCalibrationGap(gap);
}

bool scale_get_raw_adc(int32_t& rawAdc)
{
    if (!GAZ_MODULE.isInitialized()) {
        return false;
    }
    return GAZ_MODULE.readRawAdc(rawAdc);
}