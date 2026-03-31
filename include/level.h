#pragma once

#include <Arduino.h>
#include <mutex>

class Level {
public:
    bool init();
    void process();
    bool isInitialized() const;

    /** Capture the current raw pitch/roll as the zero-level offsets and persist them. */
    bool calibrate();

private:
    struct GeometryConfig {
        float wheelbaseMm = 2200.0f;
        float trackWidthMm = 1600.0f;
        float offsetXmm = 180.0f;
        float offsetYmm = -120.0f;
    };

    static constexpr float kPi = 3.14159265358979323846f;
    static constexpr float kRadToDeg = 57.2957795f;
    static constexpr uint32_t kProcessPeriodMs = 10000;

    static float sanitizePositive(float value, float fallback, float minValue, float maxValue);
    static float sanitizeFinite(float value, float fallback, float minValue, float maxValue);
    static GeometryConfig geometryFromConfig();
    static void computeWheelHeights(float pitchDeg,
                                    float rollDeg,
                                    const GeometryConfig& geometry,
                                    float& flMm,
                                    float& frMm,
                                    float& rlMm,
                                    float& rrMm);
    static void publishLevel(float pitchDeg,
                             float rollDeg,
                             float flMm,
                             float frMm,
                             float rlMm,
                             float rrMm);

    mutable std::mutex m_mutex;

    bool m_initialized = false;

    float m_lastPitchDeg = 0.0f;
    float m_lastRollDeg = 0.0f;
    float m_lastWheelFlMm = 0.0f;
    float m_lastWheelFrMm = 0.0f;
    float m_lastWheelRlMm = 0.0f;
    float m_lastWheelRrMm = 0.0f;
};

/** Global singleton instance used by runtime tasks and APIs. */
extern Level LEVEL_TASK;
