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
#include "i2c.h"
#include "mqtt.h"

namespace {

constexpr uint8_t GAZ_I2C_ADDRESS = 0x26;
constexpr uint32_t GAZ_I2C_CLOCK_HZ = 400000U;
constexpr const char* GAZ_DEVICE_FULL_NAME = "M5Stack Weight I2C Unit";

// Constants for fill level calculation
constexpr int32_t GAZ_BOTTLE_FULL_G = 6450;
constexpr int32_t GAZ_BOTTLE_EMPTY_G = 3700;

float sanitizedGap(const float gap)
{
    if (!std::isfinite(gap) || gap == 0.0f) {
        return 1.0f;
    }
    return gap;
}

class GazRuntime {
public:
    bool init();
    void process();
    bool tare();
    bool applyCalibration(float gap);
    float readCalibrationSample();
    bool isInitialized() const;

private:
    void publishCalibrationGap(float gap) const;
    void publishWeight(int32_t weightG, const int32_t fillPct) const;
    bool refreshMeasurementLocked(int32_t& weightG, int32_t& fillPct);

    mutable std::mutex m_mutex;
    sf_i2c::I2C m_i2c{GAZ_I2C_CLOCK_HZ};
    m5::unit::UnitUnified m_units;
    m5::unit::UnitWeightI2C m_unit;

    bool m_initialized = false;
    float m_lastCalibrationGap = 1.0f;
    int32_t m_lastWeightG = 0;
    int32_t m_lastFillPct = 0;

    sf_i2c::Device m_device = { .route = sf_i2c::Route{}, 
                                .sda = -1, 
                                .scl = -1, 
                                .clock = GAZ_I2C_CLOCK_HZ, 
                                .address = GAZ_I2C_ADDRESS, 
                                .deviceName = GAZ_DEVICE_FULL_NAME };
};

GazRuntime GAZ_RUNTIME;

}  // namespace

Gaz GAZ_MODULE;

bool GazRuntime::init()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    m_i2c.beginPortA(m_device.sda, m_device.scl);
    M5_LOGI("[GAZ] using Wire SDA:%d SCL:%d", m_device.sda, m_device.scl);

    bool initialized = false;

    if (!m_i2c.detectRoute(GAZ_I2C_ADDRESS, m_device.route)) {
        m_initialized = false;
        M5_LOGW("[GAZ] Weight I2C unit was not detected on internal/direct/PAHub paths");
        m_i2c.publishConfiguration(m_device);
        return false;
    }

    m_units = m5::unit::UnitUnified{};

    if (sf_i2c::isPaHubRoute(m_device.route.mode)) {
        if (!m_i2c.selectPaHubChannel(m_device.route.mode, static_cast<uint8_t>(m_device.route.paHubChannel))) {
            M5_LOGE("[GAZ] failed to select PAHub channel %d", m_device.route.paHubChannel);
            m_initialized = false;
            m_i2c.publishConfiguration(m_device);
            return false;
        }
    }

    if (sf_i2c::isInternalRoute(m_device.route.mode)) {
        initialized = m_units.add(m_unit, M5.Ex_I2C) && m_units.begin();
    } else {
        initialized = m_units.add(m_unit, Wire) && m_units.begin();
    }
    if (sf_i2c::isPaHubRoute(m_device.route.mode)) {
        m_i2c.disablePaHubChannels(m_device.route.mode);
    }

    if (!initialized) {
        m_initialized = false;
        M5_LOGW("[GAZ] Weight I2C unit was not detected on supported Wire paths");
        m_i2c.publishConfiguration(m_device);
        return false;
    }

    m_initialized = true;
    m_i2c.publishConfiguration(m_device);

    const float effectiveGap = sanitizedGap(CONFIG.scale_cal_factor);
    if (!m_unit.writeGap(effectiveGap)) {
        M5_LOGW("[GAZ] failed to apply calibration gap %.6f during init", effectiveGap);
    }
    m_lastCalibrationGap = effectiveGap;
    publishCalibrationGap(effectiveGap);

    M5_LOGI("[GAZ] Weight I2C initialization complete");
    M5_LOGI("%s", m_units.debugInfo().c_str());
    return true;
}

void GazRuntime::publishCalibrationGap(const float gap) const
{
    char gapBuf[24] = {0};
    snprintf(gapBuf, sizeof(gapBuf), "%.6f", gap);
    sf_mqtt::publish("smartfranklin/gaz/calibration/gap", gapBuf, 1, true);
}

bool GazRuntime::refreshMeasurementLocked(int32_t& weightG, int32_t& fillPct)
{
    if (sf_i2c::isPaHubRoute(m_device.route.mode)) {
        if (!m_i2c.selectPaHubChannel(m_device.route.mode, static_cast<uint8_t>(m_device.route.paHubChannel))) {
            M5_LOGW("[GAZ] failed to select PAHub channel %d", m_device.route.paHubChannel);
            return false;
        }
    }

    m_units.update();

    if (sf_i2c::isPaHubRoute(m_device.route.mode)) {
        m_i2c.disablePaHubChannels(m_device.route.mode);
    }

    if (!m_unit.updated()) {
        return false;
    }

    weightG = m_unit.weight();
    if (!std::isfinite(weightG)) {
        M5_LOGW("[GAZ] non-finite weight sample ignored");
        return false;
    }

    m_lastWeightG = weightG;

    fillPct = 0;
    if (weightG <= GAZ_BOTTLE_EMPTY_G) {
        fillPct = 0;
    } else if (weightG >= GAZ_BOTTLE_FULL_G) {
        fillPct = 100;
    } else {
        fillPct = 100 * (static_cast<float>(weightG - GAZ_BOTTLE_EMPTY_G) / (GAZ_BOTTLE_FULL_G - GAZ_BOTTLE_EMPTY_G));
    }

    m_lastFillPct = fillPct;

    return true;
}

void GazRuntime::publishWeight(const int32_t weightG, const int32_t fillPct) const
{
    char gBuf[24] = {0};
    snprintf(gBuf, sizeof(gBuf), "%d", weightG);
    sf_mqtt::publish("smartfranklin/gaz/g", gBuf);

    char pctBuf[16] = {0};
    snprintf(pctBuf, sizeof(pctBuf), "%d", fillPct);
    sf_mqtt::publish("smartfranklin/gaz/fill", pctBuf, 1, true);

    M5_LOGI("[GAZ] Weight: %d g     Fill level: %d%%", weightG, fillPct);
}

void GazRuntime::process()
{
    int32_t weightG = 0;
    int32_t fillPct = 0;
    float desiredGap = 1.0f;
    bool hasMeasurement = false;

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_initialized) {
            return;
        }

        desiredGap = sanitizedGap(CONFIG.scale_cal_factor);
        if (desiredGap != m_lastCalibrationGap) {
            if (m_unit.writeGap(desiredGap)) {
                m_lastCalibrationGap = desiredGap;
                publishCalibrationGap(desiredGap);
            } else {
                M5_LOGW("[GAZ] failed to refresh calibration gap %.6f", desiredGap);
            }
        }

        hasMeasurement = refreshMeasurementLocked(weightG, fillPct);
    }

    if (!hasMeasurement) {
        M5_LOGW("[GAZ] No measurement");
        return;
    }

    {
    }

    {
        std::lock_guard<std::mutex> lock(DATA_MUTEX);
        DATA.weight_gaz = weightG;
        DATA.fill_gaz = fillPct;
    }

    publishWeight(weightG, fillPct);
}

bool GazRuntime::tare()
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

bool GazRuntime::applyCalibration(const float gap)
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

float GazRuntime::readCalibrationSample()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_initialized) {
        return 0.0f;
    }

    int32_t weightG = 0;
    int32_t fillPct = 0;
    if (refreshMeasurementLocked(weightG, fillPct)) {
        return weightG;
    }

    return m_lastWeightG;
}

bool GazRuntime::isInitialized() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_initialized;
}

bool Gaz::init()
{
    return GAZ_RUNTIME.init();
}

void Gaz::process()
{
    GAZ_RUNTIME.process();
}

bool Gaz::tare()
{
    return GAZ_RUNTIME.tare();
}

bool Gaz::applyCalibration(const float gap)
{
    return GAZ_RUNTIME.applyCalibration(gap);
}

float Gaz::readCalibrationSample()
{
    return GAZ_RUNTIME.readCalibrationSample();
}

bool Gaz::isInitialized() const
{
    return GAZ_RUNTIME.isInitialized();
}