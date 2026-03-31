/**
 * @file level.cpp
 * @brief Implements inclinometer acquisition, vehicle pose estimation, and corner height projection.
 * @details
 * This module reads acceleration vectors from the integrated M5 IMU,
 * converts them to pitch/roll angles, applies configurable zero offsets, projects the
 * resulting plane onto the four wheel points, updates shared telemetry state, and publishes the
 * measurements through MQTT.
 */

#include "level.h"

#include <M5Unified.h>

#include <cmath>
#include <cstdio>
#include <cstdint>

#include "config_store.h"
#include "data_model.h"
#include "i2c.h"
#include "mqtt.h"

float Level::sanitizePositive(const float value, const float fallback, const float minValue, const float maxValue)
{
    if (!std::isfinite(value) || value < minValue || value > maxValue) {
        return fallback;
    }
    return value;
}

float Level::sanitizeFinite(const float value, const float fallback, const float minValue, const float maxValue)
{
    if (!std::isfinite(value) || value < minValue || value > maxValue) {
        return fallback;
    }
    return value;
}

Level::GeometryConfig Level::geometryFromConfig()
{
    GeometryConfig g;
    g.wheelbaseMm = sanitizePositive(CONFIG.level_wheelbase_mm, 2200.0f, 100.0f, 10000.0f);
    g.trackWidthMm = sanitizePositive(CONFIG.level_track_width_mm, 1600.0f, 100.0f, 10000.0f);
    g.offsetXmm = sanitizeFinite(CONFIG.level_offset_x_mm, 180.0f, -5000.0f, 5000.0f);
    g.offsetYmm = sanitizeFinite(CONFIG.level_offset_y_mm, -120.0f, -5000.0f, 5000.0f);
    return g;
}

void Level::computeWheelHeights(const float pitchDeg,
                                const float rollDeg,
                                const GeometryConfig& geometry,
                                float& flMm,
                                float& frMm,
                                float& rlMm,
                                float& rrMm)
{
    const float pitchRad = pitchDeg * kPi / 180.0f;
    const float rollRad = rollDeg * kPi / 180.0f;
    const float pitchTan = tanf(pitchRad);
    const float rollTan = tanf(rollRad);
    const float halfWheelbase = geometry.wheelbaseMm * 0.5f;
    const float halfTrack = geometry.trackWidthMm * 0.5f;

    auto heightAt = [&](float x, float y) {
        return (x - geometry.offsetXmm) * pitchTan + (y - geometry.offsetYmm) * rollTan;
    };

    flMm = heightAt(+halfWheelbase, -halfTrack);
    frMm = heightAt(+halfWheelbase, +halfTrack);
    rlMm = heightAt(-halfWheelbase, -halfTrack);
    rrMm = heightAt(-halfWheelbase, +halfTrack);
}

void Level::publishLevel(const float pitchDeg,
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

    M5_LOGI("[LEVEL] pitch:%.0f° roll:%.0f° FL:%.0fcm FR:%.0fcm RL:%.0fcm RR:%.0fcm",
            pitchDeg, rollDeg, flMm/10, frMm/10, rlMm/10, rrMm/10);
}

Level LEVEL_TASK;

bool Level::isInitialized() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_initialized;
}

bool Level::calibrate()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (!m_initialized) {
        M5_LOGW("[LEVEL] calibrate called before init");
        return false;
    }

    M5.Imu.update();
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    if (!M5.Imu.getAccel(&ax, &ay, &az)) {
        M5_LOGW("[LEVEL] calibrate: accel read failed");
        return false;
    }

    if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az)) {
        M5_LOGW("[LEVEL] calibrate: non-finite accel");
        return false;
    }

    const float norm = sqrtf((ax * ax) + (ay * ay) + (az * az));
    if (!std::isfinite(norm) || norm < 0.0001f) {
        M5_LOGW("[LEVEL] calibrate: invalid accel norm");
        return false;
    }

    const float rawPitchDeg = atan2f(-ax, sqrtf((ay * ay) + (az * az))) * kRadToDeg;
    const float rawRollDeg  = atan2f(ay, az) * kRadToDeg;

    CONFIG.level_zero_pitch_deg = rawPitchDeg;
    CONFIG.level_zero_roll_deg  = rawRollDeg;
    config_save();

    M5_LOGI("[LEVEL] calibrated: zero pitch=%.3f roll=%.3f", rawPitchDeg, rawRollDeg);
    return true;
}

bool Level::init()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    m_initialized = false;

    bool imuReady = M5.Imu.isEnabled() && M5.Imu.getType() != m5::imu_t::imu_none;
    if (!imuReady) {
        if (!M5.Imu.begin()) {
            return false;
        }
        imuReady = M5.Imu.isEnabled() && M5.Imu.getType() != m5::imu_t::imu_none;
    }

    if (imuReady) {
        m_initialized = true;
        M5_LOGI("[LEVEL] initialized");
    } else {
        M5_LOGE("[LEVEL] unable to initialize");
    }

    return imuReady;
}

void Level::process()
{
    static uint32_t lastProcessMs = 0;
    const uint32_t now = millis();
    if (now - lastProcessMs < kProcessPeriodMs) {
        return;
    }
    lastProcessMs = now;

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
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_initialized) {
            return;
        }

        M5.Imu.update();
        if (!M5.Imu.getAccel(&ax, &ay, &az)) {
            if (!M5.Imu.begin()) {
                M5_LOGW("[LEVEL] sample read failed");
                return;
            }
            M5.Imu.update();
            if (!M5.Imu.getAccel(&ax, &ay, &az)) {
                M5_LOGW("[LEVEL] sample read failed");
                return;
            }
        }

        if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az)) {
            M5_LOGW("[LEVEL] sample read failed");
            return;
        }

        const float norm = sqrtf((ax * ax) + (ay * ay) + (az * az));
        if (!std::isfinite(norm) || norm < 0.0001f) {
            M5_LOGW("[LEVEL] invalid accel norm");
            return;
        }

        pitchDeg = atan2f(-ax, sqrtf((ay * ay) + (az * az))) * kRadToDeg;
        rollDeg = atan2f(ay, az) * kRadToDeg;

        const float zeroPitch = sanitizeFinite(CONFIG.level_zero_pitch_deg, 0.0f, -90.0f, 90.0f);
        const float zeroRoll = sanitizeFinite(CONFIG.level_zero_roll_deg, 0.0f, -90.0f, 90.0f);
        pitchDeg -= zeroPitch;
        rollDeg -= zeroRoll;

        if (!std::isfinite(pitchDeg) || !std::isfinite(rollDeg)) {
            M5_LOGW("[LEVEL] non-finite pose");
            return;
        }

        computeWheelHeights(pitchDeg, rollDeg, geometry, flMm, frMm, rlMm, rrMm);

        m_lastPitchDeg = pitchDeg;
        m_lastRollDeg = rollDeg;
        m_lastWheelFlMm = flMm;
        m_lastWheelFrMm = frMm;
        m_lastWheelRlMm = rlMm;
        m_lastWheelRrMm = rrMm;
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

    publishLevel(pitchDeg, rollDeg, flMm, frMm, rlMm, rrMm);
}

/**
 * @brief FreeRTOS task for level (accelerometer) acquisition and processing.
 */
void taskLevel(void* pv)
{
    (void)pv;
    M5_LOGI("[LEVEL] Task started");

    bool initialized = false;
    uint32_t nextInitAttemptMs = 0;

    auto isRetryDue = [](uint32_t nowMs, uint32_t nextAttemptMs) {
        return static_cast<int32_t>(nowMs - nextAttemptMs) >= 0;
    };

    auto scheduleRetry = [](uint32_t& nextAttemptMs, uint32_t nowMs) {
        nextAttemptMs = nowMs + sf_i2c::kInitRetryMs;
    };

    for (;;) {
        const uint32_t nowMs = millis();

        if (!initialized && isRetryDue(nowMs, nextInitAttemptMs)) {
            initialized = LEVEL_TASK.init();

            if (!initialized) {
                M5_LOGW("[LEVEL] Init failed");
                scheduleRetry(nextInitAttemptMs, nowMs);
            }
        }

        if (initialized) {
            LEVEL_TASK.process();
        }

        const int loopMs = (CONFIG.task_i2c_loop_ms > 0) ? CONFIG.task_i2c_loop_ms : 1000;
        vTaskDelay(pdMS_TO_TICKS(loopMs));
    }
}
