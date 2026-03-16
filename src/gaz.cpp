#include "gaz.h"

#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedHUB.h>
#include <M5UnitUnifiedWEIGHT.h>
#include <M5Utility.h>

#include <cmath>
#include <cstdio>
#include <mutex>

#include "config_store.h"
#include "data_model.h"
#include "mqtt.h"
#include "pahub_channels.h"

namespace {

constexpr uint8_t WEIGHT_I2C_ADDRESS = 0x26;
constexpr uint32_t WEIGHT_I2C_CLOCK_HZ = 400000U;
constexpr uint8_t PAHUB_CHANNEL_COUNT = 8;
constexpr const char* GAZ_DEVICE_FULL_NAME = "M5Stack Weight I2C Unit";

enum class GazRouteMode : uint8_t {
    Unset = 0,
    Internal,
    InternalPaHub,
    Wire,
    WirePaHub,
};

const char* routeModeToString(const GazRouteMode mode)
{
    switch (mode) {
    case GazRouteMode::Internal:
        return "internal";
    case GazRouteMode::InternalPaHub:
        return "internal_pahub";
    case GazRouteMode::Wire:
        return "wire";
    case GazRouteMode::WirePaHub:
        return "wire_pahub";
    case GazRouteMode::Unset:
    default:
        return "unset";
    }
}

bool wireDeviceExists(const uint8_t address)
{
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

bool exDeviceExists(const uint8_t address)
{
    return M5.Ex_I2C.scanID(address, WEIGHT_I2C_CLOCK_HZ);
}

bool wireSelectPaHubChannel(const uint8_t channel)
{
    Wire.beginTransmission(PAHUB_ADDRESS);
    Wire.write(static_cast<uint8_t>(1U << channel));
    return Wire.endTransmission() == 0;
}

void wireDisablePaHubChannels()
{
    Wire.beginTransmission(PAHUB_ADDRESS);
    Wire.write(static_cast<uint8_t>(0x00));
    Wire.endTransmission();
}

bool exSelectPaHubChannel(const uint8_t channel)
{
    if (!M5.Ex_I2C.start(PAHUB_ADDRESS, false, WEIGHT_I2C_CLOCK_HZ)) {
        return false;
    }

    const bool writeOk = M5.Ex_I2C.write(static_cast<uint8_t>(1U << channel));
    const bool stopOk = M5.Ex_I2C.stop();
    return writeOk && stopOk;
}

void exDisablePaHubChannels()
{
    if (!M5.Ex_I2C.start(PAHUB_ADDRESS, false, WEIGHT_I2C_CLOCK_HZ)) {
        return;
    }

    M5.Ex_I2C.write(static_cast<uint8_t>(0x00));
    M5.Ex_I2C.stop();
}

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
    m5::unit::UnitUnified m_units;
    m5::unit::UnitWeightI2C m_unit;
    m5::unit::UnitPaHub2 m_pahub{PAHUB_ADDRESS};
    bool m_initialized = false;
    GazRouteMode m_routeMode = GazRouteMode::Unset;
    int8_t m_paHubChannel = -1;
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
    m_routeMode = GazRouteMode::Unset;
    m_paHubChannel = -1;

    if (exDeviceExists(WEIGHT_I2C_ADDRESS)) {
        m_routeMode = GazRouteMode::Internal;
        return true;
    }

    if (wireDeviceExists(WEIGHT_I2C_ADDRESS)) {
        m_routeMode = GazRouteMode::Wire;
        return true;
    }

    if (wireDeviceExists(PAHUB_ADDRESS)) {
        for (uint8_t channel = 0; channel < PAHUB_CHANNEL_COUNT; ++channel) {
            if (!wireSelectPaHubChannel(channel)) {
                continue;
            }

            if (wireDeviceExists(WEIGHT_I2C_ADDRESS)) {
                wireDisablePaHubChannels();
                m_routeMode = GazRouteMode::WirePaHub;
                m_paHubChannel = static_cast<int8_t>(channel);
                return true;
            }
        }
        wireDisablePaHubChannels();
    }

    if (exDeviceExists(PAHUB_ADDRESS)) {
        for (uint8_t channel = 0; channel < PAHUB_CHANNEL_COUNT; ++channel) {
            if (!exSelectPaHubChannel(channel)) {
                continue;
            }

            if (exDeviceExists(WEIGHT_I2C_ADDRESS)) {
                exDisablePaHubChannels();
                m_routeMode = GazRouteMode::InternalPaHub;
                m_paHubChannel = static_cast<int8_t>(channel);
                return true;
            }
        }
        exDisablePaHubChannels();
    }

    return false;
}

bool GazRuntime::init()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    m_wireSda = static_cast<int8_t>(M5.getPin(m5::pin_name_t::port_a_sda));
    m_wireScl = static_cast<int8_t>(M5.getPin(m5::pin_name_t::port_a_scl));
    M5_LOGI("[GAZ] using Wire SDA:%d SCL:%d", m_wireSda, m_wireScl);

    Wire.end();
    Wire.begin(m_wireSda, m_wireScl, WEIGHT_I2C_CLOCK_HZ);
    Wire.setPins(m_wireSda, m_wireScl);
    M5.Ex_I2C.begin();

    bool initialized = false;

    if (!detectRoute()) {
        m_initialized = false;
        M5_LOGW("[GAZ] Weight I2C unit was not detected on internal/direct/PAHub paths");
        publishI2cConfiguration();
        return false;
    }

    if (m_routeMode == GazRouteMode::WirePaHub) {
        initialized = m_pahub.add(m_unit, static_cast<uint8_t>(m_paHubChannel))
            && m_units.add(m_pahub, Wire)
            && m_units.begin();
    } else if (m_routeMode == GazRouteMode::InternalPaHub) {
        initialized = m_pahub.add(m_unit, static_cast<uint8_t>(m_paHubChannel))
            && m_units.add(m_pahub, M5.Ex_I2C)
            && m_units.begin();
    } else if (m_routeMode == GazRouteMode::Wire) {
        initialized = m_units.add(m_unit, Wire) && m_units.begin();
    } else if (m_routeMode == GazRouteMode::Internal) {
        initialized = m_units.add(m_unit, M5.Ex_I2C) && m_units.begin();
    }

    if (!initialized) {
        m_initialized = false;
        M5_LOGW("[GAZ] Weight I2C unit initialization failed on detected path");
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

    snprintf(pahubChannelBuf, sizeof(pahubChannelBuf), "%d", m_paHubChannel);
    snprintf(sdaBuf, sizeof(sdaBuf), "%d", m_wireSda);
    snprintf(sclBuf, sizeof(sclBuf), "%d", m_wireScl);
    snprintf(addressBuf, sizeof(addressBuf), "0x%02X", WEIGHT_I2C_ADDRESS);

    sf_mqtt::publish("smartfranklin/system/gaz/i2c/mode", routeModeToString(m_routeMode));
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
    m_units.update();
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