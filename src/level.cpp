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
#include <mutex>

#include "config_store.h"
#include "data_model.h"
#include "i2c.h"
#include "mqtt.h"

namespace {

constexpr float PI_F = 3.14159265358979323846f;
constexpr float RAD_TO_DEG_F = 57.2957795f;

constexpr uint32_t PROCESS_PERIOD_MS = 10000; // Change as needed

struct GeometryConfig {
    float wheelbaseMm = 2200.0f;
    float trackWidthMm = 1600.0f;
    float offsetXmm = 180.0f;
    float offsetYmm = -120.0f;
};

struct LevelState {
    mutable std::mutex mutex;

    bool initialized = false;

    float lastPitchDeg = 0.0f;
    float lastRollDeg = 0.0f;
    float lastWheelFlMm = 0.0f;
    float lastWheelFrMm = 0.0f;
    float lastWheelRlMm = 0.0f;
    float lastWheelRrMm = 0.0f;
};

LevelState LEVEL_STATE;

/**
 * @brief Reads one acceleration sample in g units from the configured source.
 */
bool readAccelSampleLocked(const LevelState& state, float& ax, float& ay, float& az)
{
    ax = 0.0f;
    ay = 0.0f;
    az = 0.0f;

    if (state.initialized) {

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
    } else {
        return false;
    }

}

float sanitizePositive(const float value, const float fallback, const float minValue, const float maxValue)
{
    if (!std::isfinite(value) || value < minValue || value > maxValue) {
        return fallback;
    }
    return value;
}

/**
 * @brief Validates a finite bounded numeric value and returns a fallback when invalid.
 */
float sanitizeFinite(const float value, const float fallback, const float minValue, const float maxValue)
{
    if (!std::isfinite(value) || value < minValue || value > maxValue) {
        return fallback;
    }
    return value;
}

/**
 * @brief Builds normalized geometry parameters from configuration storage.
 */
GeometryConfig geometryFromConfig()
{
    GeometryConfig g;

    g.wheelbaseMm = sanitizePositive(CONFIG.level_wheelbase_mm, 2200.0f, 100.0f, 10000.0f);
    g.trackWidthMm = sanitizePositive(CONFIG.level_track_width_mm, 1600.0f, 100.0f, 10000.0f);
    g.offsetXmm = sanitizeFinite(CONFIG.level_offset_x_mm, 180.0f, -5000.0f, 5000.0f);
    g.offsetYmm = sanitizeFinite(CONFIG.level_offset_y_mm, -120.0f, -5000.0f, 5000.0f);

    return g;
}

/**
 * @brief Projects pitch/roll orientation onto vehicle wheel corners.
 */
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

/**
 * @brief Publishes pose angles and projected wheel heights to MQTT topics.
 */
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

/**
 * @brief Initializes module state and configures the selected IMU backend.
 */
bool initState(LevelState& state)
{
    std::lock_guard<std::mutex> lock(state.mutex);

    state.initialized = false;

    bool ensureInternalImuReady = (M5.Imu.isEnabled() && M5.Imu.getType() != m5::imu_t::imu_none);

    if ( !ensureInternalImuReady ) {
        // Fallback: force IMU re-bind in case board boot init skipped or raced.

        if (!M5.Imu.begin()) {
            return false;
        }

        ensureInternalImuReady =  M5.Imu.isEnabled() && M5.Imu.getType() != m5::imu_t::imu_none;

    }

    if ( ensureInternalImuReady ) {
        state.initialized = true;
        M5_LOGI("[LEVEL] initialized");
    } else {
        M5_LOGE("[LEVEL] unable to initialize");
    }

    return ensureInternalImuReady;
}

/**
 * @brief Executes one periodic processing cycle and updates shared output fields.
 */
void processState(LevelState& state)
{
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

        const float zeroPitch = sanitizeFinite(CONFIG.level_zero_pitch_deg, 0.0f, -90.0f, 90.0f);
        const float zeroRoll = sanitizeFinite(CONFIG.level_zero_roll_deg, 0.0f, -90.0f, 90.0f);
        pitchDeg -= zeroPitch;
        rollDeg -= zeroRoll;

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

/**
 * @brief Reports whether the level module state is initialized.
 */
bool isInitializedState(const LevelState& state)
{
    std::lock_guard<std::mutex> lock(state.mutex);
    return state.initialized;
}

}  // namespace

Level LEVEL_TASK;

/** @brief Returns true when the level module has been successfully initialized. */
bool Level::isInitialized() const
{
    return isInitializedState(LEVEL_STATE);
}


/** @brief Initializes the level module using the selected source and bus route. */
bool Level::init()
{
    return initState(LEVEL_STATE);
}


/** @brief Runs time-gated orientation processing and telemetry publication. */
void Level::process()
{
    static uint32_t lastProcessMs = 0;
    const uint32_t now = millis();
    if (now - lastProcessMs < PROCESS_PERIOD_MS) {
        return;
    }
    lastProcessMs = now;
    processState(LEVEL_STATE);
}

/**
 * @brief FreeRTOS task for level (accelerometer) acquisition and processing.
 *
 * Initializes the internal M5 IMU, reads acceleration samples periodically,
 * computes pitch/roll angles, and publishes measurements to MQTT.
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
            // Initialize internal M5 IMU
            initialized = LEVEL_TASK.init();

            if (!initialized) {
                M5_LOGW("[LEVEL] Init failed");
                scheduleRetry(nextInitAttemptMs, nowMs);
            } else {
                M5_LOGI("[LEVEL] Initialized with internal M5 IMU");
            }
        }

        if (initialized) {
            LEVEL_TASK.process();
        }

        const int loopMs = (CONFIG.task_i2c_loop_ms > 0) ? CONFIG.task_i2c_loop_ms : 1000;
        vTaskDelay(pdMS_TO_TICKS(loopMs));
    }
}
