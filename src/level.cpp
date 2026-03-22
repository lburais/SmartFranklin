/*
 * SmartFranklin - IMU module implementation
 * SPDX-License-Identifier: MIT
 */

#include "level.h"

#include <M5Unified.h>
#include <Wire.h>

#include <cmath>
#include <cstdio>
#include <cstdint>
#include <mutex>

#include "config_store.h"
#include "data_model.h"
#include "mqtt.h"

namespace {

constexpr uint8_t MPU6050_REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t MPU6050_REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t MPU6050_REG_ACCEL_XOUT_H = 0x3B;
constexpr float MPU6050_ACCEL_LSB_PER_G = 16384.0f;

constexpr uint8_t ADXL345_REG_DEVID = 0x00;
constexpr uint8_t ADXL345_REG_POWER_CTL = 0x2D;
constexpr uint8_t ADXL345_REG_DATA_FORMAT = 0x31;
constexpr uint8_t ADXL345_REG_DATAX0 = 0x32;
constexpr uint8_t ADXL345_DEVICE_ID = 0xE5;
constexpr float ADXL345_G_PER_LSB = 0.0039f;

constexpr float PI_F = 3.14159265358979323846f;
constexpr float RAD_TO_DEG_F = 57.2957795f;

struct GeometryConfig {
    float wheelbaseMm = 2200.0f;
    float trackWidthMm = 1600.0f;
    float offsetXmm = 180.0f;
    float offsetYmm = -120.0f;
};

struct LevelState {
    mutable std::mutex mutex;

    bool initialized = false;
    Level::Source source = Level::Source::None;

    bool isInternalRoute = false;
    uint8_t i2cAddress = 0x00;

    float lastPitchDeg = 0.0f;
    float lastRollDeg = 0.0f;
    float lastWheelFlMm = 0.0f;
    float lastWheelFrMm = 0.0f;
    float lastWheelRlMm = 0.0f;
    float lastWheelRrMm = 0.0f;
};

LevelState LEVEL_STATE;

float sanitizePositive(const float value, const float fallback, const float minValue, const float maxValue)
{
    if (!std::isfinite(value) || value < minValue || value > maxValue) {
        return fallback;
    }
    return value;
}

float sanitizeFinite(const float value, const float fallback, const float minValue, const float maxValue)
{
    if (!std::isfinite(value) || value < minValue || value > maxValue) {
        return fallback;
    }
    return value;
}

GeometryConfig geometryFromConfig()
{
    GeometryConfig g;

    g.wheelbaseMm = sanitizePositive(CONFIG.level_wheelbase_mm, 2200.0f, 100.0f, 10000.0f);
    g.trackWidthMm = sanitizePositive(CONFIG.level_track_width_mm, 1600.0f, 100.0f, 10000.0f);
    g.offsetXmm = sanitizeFinite(CONFIG.level_offset_x_mm, 180.0f, -5000.0f, 5000.0f);
    g.offsetYmm = sanitizeFinite(CONFIG.level_offset_y_mm, -120.0f, -5000.0f, 5000.0f);

    return g;
}

bool writeRegister(const LevelState& state, uint8_t reg, uint8_t value)
{
    if (state.isInternalRoute) {
        if (!M5.Ex_I2C.start(state.i2cAddress, false, Wire.getClock())) {
            return false;
        }
        const bool ok = M5.Ex_I2C.write(reg) && M5.Ex_I2C.write(value) && M5.Ex_I2C.stop();
        return ok;
    }

    Wire.beginTransmission(state.i2cAddress);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool readRegisters(const LevelState& state, uint8_t reg, uint8_t* out, size_t len)
{
    if (out == nullptr || len == 0) {
        return false;
    }

    if (state.isInternalRoute) {
        if (!M5.Ex_I2C.start(state.i2cAddress, false, Wire.getClock())) {
            return false;
        }

        if (!M5.Ex_I2C.write(reg) || !M5.Ex_I2C.stop()) {
            return false;
        }

        if (!M5.Ex_I2C.start(state.i2cAddress, true, Wire.getClock())) {
            return false;
        }

        for (size_t i = 0; i < len; ++i) {
            const bool lastNack = (i + 1U == len);
            if (!M5.Ex_I2C.read(&out[i], 1U, lastNack)) {
                M5.Ex_I2C.stop();
                return false;
            }
        }

        return M5.Ex_I2C.stop();
    }

    Wire.beginTransmission(state.i2cAddress);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    const size_t readCount = Wire.requestFrom(state.i2cAddress, static_cast<uint8_t>(len));
    if (readCount < len) {
        return false;
    }

    for (size_t i = 0; i < len; ++i) {
        out[i] = static_cast<uint8_t>(Wire.read());
    }

    return true;
}

bool initMpu6050(LevelState& state)
{
    if (!writeRegister(state, MPU6050_REG_PWR_MGMT_1, 0x00)) {
        return false;
    }

    if (!writeRegister(state, MPU6050_REG_ACCEL_CONFIG, 0x00)) {
        return false;
    }

    return true;
}

bool initAdxl345(LevelState& state)
{
    uint8_t devid = 0;
    if (!readRegisters(state, ADXL345_REG_DEVID, &devid, 1U)) {
        return false;
    }

    if (devid != ADXL345_DEVICE_ID) {
        return false;
    }

    // Full-resolution, +-2g range.
    if (!writeRegister(state, ADXL345_REG_DATA_FORMAT, 0x08)) {
        return false;
    }

    // Measurement mode.
    if (!writeRegister(state, ADXL345_REG_POWER_CTL, 0x08)) {
        return false;
    }

    return true;
}

bool ensureInternalImuReady()
{
    if (M5.Imu.isEnabled() && M5.Imu.getType() != m5::imu_t::imu_none) {
        return true;
    }

    // Fallback: force IMU re-bind in case board boot init skipped or raced.
    if (!M5.Imu.begin()) {
        return false;
    }

    return M5.Imu.isEnabled() && M5.Imu.getType() != m5::imu_t::imu_none;
}

bool readAccelSampleLocked(const LevelState& state, float& ax, float& ay, float& az)
{
    ax = 0.0f;
    ay = 0.0f;
    az = 0.0f;

    switch (state.source) {
    case Level::Source::InternalM5: {
        if (!ensureInternalImuReady()) {
            return false;
        }

        // update() refreshes internal sensor data before reading current sample.
        M5.Imu.update();
        if (!M5.Imu.getAccel(&ax, &ay, &az)) {
            // Retry once after explicit IMU re-init to recover transient failures.
            if (!M5.Imu.begin()) {
                return false;
            }
            M5.Imu.update();
            if (!M5.Imu.getAccel(&ax, &ay, &az)) {
                return false;
            }
        }

        if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az)) {
            return false;
        }

        return true;
    }

    case Level::Source::ExternalMpuUnit: {
        uint8_t raw[6] = {0};
        if (!readRegisters(state, MPU6050_REG_ACCEL_XOUT_H, raw, sizeof(raw))) {
            return false;
        }

        const int16_t xRaw = static_cast<int16_t>((static_cast<uint16_t>(raw[0]) << 8) | raw[1]);
        const int16_t yRaw = static_cast<int16_t>((static_cast<uint16_t>(raw[2]) << 8) | raw[3]);
        const int16_t zRaw = static_cast<int16_t>((static_cast<uint16_t>(raw[4]) << 8) | raw[5]);

        ax = static_cast<float>(xRaw) / MPU6050_ACCEL_LSB_PER_G;
        ay = static_cast<float>(yRaw) / MPU6050_ACCEL_LSB_PER_G;
        az = static_cast<float>(zRaw) / MPU6050_ACCEL_LSB_PER_G;
        return true;
    }

    case Level::Source::ExternalAdxl345: {
        uint8_t raw[6] = {0};
        if (!readRegisters(state, ADXL345_REG_DATAX0, raw, sizeof(raw))) {
            return false;
        }

        // ADXL345 outputs little-endian 16-bit signed values.
        const int16_t xRaw = static_cast<int16_t>((static_cast<uint16_t>(raw[1]) << 8) | raw[0]);
        const int16_t yRaw = static_cast<int16_t>((static_cast<uint16_t>(raw[3]) << 8) | raw[2]);
        const int16_t zRaw = static_cast<int16_t>((static_cast<uint16_t>(raw[5]) << 8) | raw[4]);

        ax = static_cast<float>(xRaw) * ADXL345_G_PER_LSB;
        ay = static_cast<float>(yRaw) * ADXL345_G_PER_LSB;
        az = static_cast<float>(zRaw) * ADXL345_G_PER_LSB;
        return true;
    }

    case Level::Source::None:
    default:
        return false;
    }
}

void computeWheelHeights(const float pitchDeg,
                         const float rollDeg,
                         const GeometryConfig& geometry,
                         float& flMm,
                         float& frMm,
                         float& rlMm,
                         float& rrMm)
{
    const float pitchRad = pitchDeg * PI_F / 180.0f;
    const float rollRad = rollDeg * PI_F / 180.0f;

    const float pitchTan = tanf(pitchRad);
    const float rollTan = tanf(rollRad);

    const float halfWheelbase = geometry.wheelbaseMm * 0.5f;
    const float halfTrack = geometry.trackWidthMm * 0.5f;

    const float xFl = +halfWheelbase;
    const float yFl = -halfTrack;
    const float xFr = +halfWheelbase;
    const float yFr = +halfTrack;
    const float xRl = -halfWheelbase;
    const float yRl = -halfTrack;
    const float xRr = -halfWheelbase;
    const float yRr = +halfTrack;

    auto heightAt = [&](float x, float y) {
        const float dx = x - geometry.offsetXmm;
        const float dy = y - geometry.offsetYmm;
        return dx * pitchTan + dy * rollTan;
    };

    flMm = heightAt(xFl, yFl);
    frMm = heightAt(xFr, yFr);
    rlMm = heightAt(xRl, yRl);
    rrMm = heightAt(xRr, yRr);
}

void publishPose(const float pitchDeg,
                 const float rollDeg,
                 const float flMm,
                 const float frMm,
                 const float rlMm,
                 const float rrMm)
{
    char pitchBuf[24] = {0};
    char rollBuf[24] = {0};
    char flBuf[24] = {0};
    char frBuf[24] = {0};
    char rlBuf[24] = {0};
    char rrBuf[24] = {0};

    snprintf(pitchBuf, sizeof(pitchBuf), "%.3f", pitchDeg);
    snprintf(rollBuf, sizeof(rollBuf), "%.3f", rollDeg);
    snprintf(flBuf, sizeof(flBuf), "%.2f", flMm);
    snprintf(frBuf, sizeof(frBuf), "%.2f", frMm);
    snprintf(rlBuf, sizeof(rlBuf), "%.2f", rlMm);
    snprintf(rrBuf, sizeof(rrBuf), "%.2f", rrMm);

    sf_mqtt::publish("smartfranklin/level/pitch_deg", pitchBuf);
    sf_mqtt::publish("smartfranklin/level/roll_deg", rollBuf);
    sf_mqtt::publish("smartfranklin/level/wheel/fl_mm", flBuf);
    sf_mqtt::publish("smartfranklin/level/wheel/fr_mm", frBuf);
    sf_mqtt::publish("smartfranklin/level/wheel/rl_mm", rlBuf);
    sf_mqtt::publish("smartfranklin/level/wheel/rr_mm", rrBuf);

    M5_LOGI("[LEVEL] pitch:%.3f roll:%.3f wheel_mm FL:%.2f FR:%.2f RL:%.2f RR:%.2f",
            pitchDeg,
            rollDeg,
            flMm,
            frMm,
            rlMm,
            rrMm);
}

bool initState(LevelState& state, Level::Source source, bool isInternalRoute, uint8_t i2cAddress)
{
    std::lock_guard<std::mutex> lock(state.mutex);

    state.source = Level::Source::None;
    state.initialized = false;
    state.isInternalRoute = isInternalRoute;
    state.i2cAddress = i2cAddress;

    switch (source) {
    case Level::Source::InternalM5:
        // Internal IMU: use M5 API fallback init before declaring unavailable.
        if (!ensureInternalImuReady()) {
            return false;
        }
        state.source = source;
        state.initialized = true;
        break;

    case Level::Source::ExternalMpuUnit:
        if (!initMpu6050(state)) {
            return false;
        }
        state.source = source;
        state.initialized = true;
        break;

    case Level::Source::ExternalAdxl345:
        if (!initAdxl345(state)) {
            return false;
        }
        state.source = source;
        state.initialized = true;
        break;

    case Level::Source::None:
    default:
        return false;
    }

    M5_LOGI("[LEVEL] initialized source:%s address:0x%02X route:%s",
            Level::sourceToString(state.source),
            state.i2cAddress,
            state.isInternalRoute ? "internal" : "wire");

    return true;
}

void processState(LevelState& state)
{
    Level::Source source = Level::Source::None;
    const GeometryConfig geometry = geometryFromConfig();
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;

    float pitchDeg = 0.0f;
    float rollDeg = 0.0f;
    float flMm = 0.0f;
    float frMm = 0.0f;
    float rlMm = 0.0f;
    float rrMm = 0.0f;

    {
        std::lock_guard<std::mutex> lock(state.mutex);
        if (!state.initialized) {
            return;
        }

        source = state.source;

        if (!readAccelSampleLocked(state, ax, ay, az)) {
            M5_LOGW("[LEVEL] sample read failed");
            return;
        }

        const float norm = sqrtf((ax * ax) + (ay * ay) + (az * az));
        if (!std::isfinite(norm) || norm < 0.0001f) {
            M5_LOGW("[LEVEL] invalid accel norm");
            return;
        }

        pitchDeg = atan2f(-ax, sqrtf((ay * ay) + (az * az))) * RAD_TO_DEG_F;
        rollDeg = atan2f(ay, az) * RAD_TO_DEG_F;

        if (!std::isfinite(pitchDeg) || !std::isfinite(rollDeg)) {
            M5_LOGW("[LEVEL] non-finite pose");
            return;
        }

        computeWheelHeights(pitchDeg, rollDeg, geometry, flMm, frMm, rlMm, rrMm);

        state.lastPitchDeg = pitchDeg;
        state.lastRollDeg = rollDeg;
        state.lastWheelFlMm = flMm;
        state.lastWheelFrMm = frMm;
        state.lastWheelRlMm = rlMm;
        state.lastWheelRrMm = rrMm;
    }

    {
        std::lock_guard<std::mutex> lock(DATA_MUTEX);
        DATA.level_pitch_deg = pitchDeg;
        DATA.level_roll_deg = rollDeg;
        DATA.level_wheel_fl_mm = flMm;
        DATA.level_wheel_fr_mm = frMm;
        DATA.level_wheel_rl_mm = rlMm;
        DATA.level_wheel_rr_mm = rrMm;
    }

    publishPose(pitchDeg, rollDeg, flMm, frMm, rlMm, rrMm);
}

bool isInitializedState(const LevelState& state)
{
    std::lock_guard<std::mutex> lock(state.mutex);
    return state.initialized;
}

Level::Source sourceState(const LevelState& state)
{
    std::lock_guard<std::mutex> lock(state.mutex);
    return state.source;
}

}  // namespace

Level LEVEL_MODULE;

bool Level::init(Source source, bool isInternalRoute, uint8_t i2cAddress)
{
    return initState(LEVEL_STATE, source, isInternalRoute, i2cAddress);
}

void Level::process()
{
    processState(LEVEL_STATE);
}

bool Level::isInitialized() const
{
    return isInitializedState(LEVEL_STATE);
}

Level::Source Level::source() const
{
    return sourceState(LEVEL_STATE);
}

const char* Level::sourceName() const
{
    return sourceToString(source());
}

const char* Level::sourceToString(Source source)
{
    switch (source) {
    case Source::InternalM5:
        return "internal_m5";
    case Source::ExternalMpuUnit:
        return "external_m5unit_imu";
    case Source::ExternalAdxl345:
        return "external_adxl345";
    case Source::None:
    default:
        return "none";
    }
}
