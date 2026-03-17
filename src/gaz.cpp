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

constexpr uint8_t WEIGHT_I2C_ADDRESS = 0x26;
constexpr uint32_t WEIGHT_I2C_CLOCK_HZ = 400000U;
constexpr const char* GAZ_DEVICE_FULL_NAME = "M5Stack Weight I2C Unit";

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
    bool detectRoute();
    void publishI2cConfiguration() const;
    void publishCalibrationGap(float gap) const;
    void publishWeight(float weightKg, int32_t weightG) const;
    bool refreshMeasurementLocked(float& weightKg, int32_t& weightG);

    mutable std::mutex m_mutex;
    sf_i2c::I2C m_i2c{WEIGHT_I2C_CLOCK_HZ};
    m5::unit::UnitUnified m_units;
    m5::unit::UnitWeightI2C m_unit;
    bool m_initialized = false;
    sf_i2c::Route m_route;
    int8_t m_wireSda = -1;
    int8_t m_wireScl = -1;
    float m_lastCalibrationGap = 1.0f;
    float m_lastWeightKg = 0.0f;
    int32_t m_lastWeightG = 0;
};

GazRuntime GAZ_RUNTIME;

}  // namespace

Gaz GAZ_MODULE;

bool GazRuntime::detectRoute()
{
    m_route = sf_i2c::Route{};
    return m_i2c.detectRoute(WEIGHT_I2C_ADDRESS, m_route);
}

bool GazRuntime::init()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    m_i2c.beginPortA(m_wireSda, m_wireScl);
    M5_LOGI("[GAZ] using Wire SDA:%d SCL:%d", m_wireSda, m_wireScl);

    bool initialized = false;

    if (!detectRoute()) {
        m_initialized = false;
        M5_LOGW("[GAZ] Weight I2C unit was not detected on internal/direct/PAHub paths");
        publishI2cConfiguration();
        return false;
    }

    m_units = m5::unit::UnitUnified{};

    if (sf_i2c::isPaHubRoute(m_route.mode)) {
        if (!m_i2c.selectPaHubChannel(m_route.mode, static_cast<uint8_t>(m_route.paHubChannel))) {
            M5_LOGE("[GAZ] failed to select PAHub channel %d", m_route.paHubChannel);
            m_initialized = false;
            publishI2cConfiguration();
            return false;
        }
    }

    if (sf_i2c::isInternalRoute(m_route.mode)) {
        initialized = m_units.add(m_unit, M5.Ex_I2C) && m_units.begin();
    } else {
        initialized = m_units.add(m_unit, Wire) && m_units.begin();
    }
    if (sf_i2c::isPaHubRoute(m_route.mode)) {
        m_i2c.disablePaHubChannels(m_route.mode);
    }

    if (!initialized) {
        m_initialized = false;
        M5_LOGW("[GAZ] Weight I2C unit was not detected on supported Wire paths");
        publishI2cConfiguration();
        return false;
    }

    m_initialized = true;
    publishI2cConfiguration();

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

void GazRuntime::publishI2cConfiguration() const
{
    char pahubChannelBuf[12] = {0};
    char sdaBuf[12] = {0};
    char sclBuf[12] = {0};
    char addressBuf[8] = {0};

    snprintf(pahubChannelBuf, sizeof(pahubChannelBuf), "%d", m_route.paHubChannel);
    snprintf(sdaBuf, sizeof(sdaBuf), "%d", m_wireSda);
    snprintf(sclBuf, sizeof(sclBuf), "%d", m_wireScl);
    snprintf(addressBuf, sizeof(addressBuf), "0x%02X", WEIGHT_I2C_ADDRESS);

    sf_mqtt::publish("smartfranklin/system/gaz/i2c/mode", sf_i2c::routeModeToString(m_route.mode));
    sf_mqtt::publish("smartfranklin/system/gaz/i2c/pahub_channel", pahubChannelBuf);
    sf_mqtt::publish("smartfranklin/system/gaz/i2c/sda", sdaBuf);
    sf_mqtt::publish("smartfranklin/system/gaz/i2c/scl", sclBuf);
    sf_mqtt::publish("smartfranklin/system/gaz/i2c/address", addressBuf);
    sf_mqtt::publish("smartfranklin/system/gaz/i2c/device_name", GAZ_DEVICE_FULL_NAME);
}

void GazRuntime::publishCalibrationGap(const float gap) const
{
    char gapBuf[24] = {0};
    snprintf(gapBuf, sizeof(gapBuf), "%.6f", gap);
    sf_mqtt::publish("smartfranklin/system/gaz/calibration/gap", gapBuf);
}

bool GazRuntime::refreshMeasurementLocked(float& weightKg, int32_t& weightG)
{
    if (sf_i2c::isPaHubRoute(m_route.mode)) {
        if (!m_i2c.selectPaHubChannel(m_route.mode, static_cast<uint8_t>(m_route.paHubChannel))) {
            M5_LOGW("[GAZ] failed to select PAHub channel %d", m_route.paHubChannel);
            return false;
        }
    }

    m_units.update();

    if (sf_i2c::isPaHubRoute(m_route.mode)) {
        m_i2c.disablePaHubChannels(m_route.mode);
    }

    if (!m_unit.updated()) {
        return false;
    }

    weightKg = m_unit.weight();
    if (!std::isfinite(weightKg)) {
        M5_LOGW("[GAZ] non-finite weight sample ignored");
        return false;
    }

    weightG = static_cast<int32_t>(lroundf(weightKg * 1000.0f));
    m_lastWeightKg = weightKg;
    m_lastWeightG = weightG;
    return true;
}

void GazRuntime::publishWeight(const float weightKg, const int32_t weightG) const
{
    char kgBuf[24] = {0};
    char gBuf[24] = {0};
    snprintf(kgBuf, sizeof(kgBuf), "%.3f", weightKg);
    snprintf(gBuf, sizeof(gBuf), "%ld", static_cast<long>(weightG));

    sf_mqtt::publish("smartfranklin/gaz/kg", kgBuf);
    sf_mqtt::publish("smartfranklin/weight/g", gBuf);
}

void GazRuntime::process()
{
    float weightKg = 0.0f;
    int32_t weightG = 0;
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

        hasMeasurement = refreshMeasurementLocked(weightKg, weightG);
    }

    if (!hasMeasurement) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(DATA_MUTEX);
        DATA.weight_g = weightG;
    }

    publishWeight(weightKg, weightG);
    M5_LOGI("[GAZ] Weight: %.3f kg (%ld g)", weightKg, static_cast<long>(weightG));
}

bool GazRuntime::tare()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_initialized) {
        return false;
    }
    const bool ok = m_unit.resetOffset();
    if (ok) {
        m_lastWeightKg = 0.0f;
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

    float weightKg = 0.0f;
    int32_t weightG = 0;
    if (refreshMeasurementLocked(weightKg, weightG)) {
        return weightKg;
    }

    return m_lastWeightKg;
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