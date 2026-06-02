#pragma once

#include <Arduino.h>

#include <mutex>

#include "interfaces.h"

class Ina {
public:
    bool init();
    void process();
    bool isInitialized() const;

private:
    static constexpr size_t kChannelCount = 3;

    struct Sample {
        bool ok = false;
        float busVoltageV[kChannelCount] = {0.0f, 0.0f, 0.0f};
        float shuntVoltageMv[kChannelCount] = {0.0f, 0.0f, 0.0f};
        float currentA[kChannelCount] = {0.0f, 0.0f, 0.0f};
    };

    mutable std::mutex m_mutex;

    bool m_initialized = false;
    bool m_ina1Available = false;
    bool m_ina2Available = false;

    const sf_interfaces::InterfaceSensor m_sensor = sf_interfaces::InterfaceSensor::Ina1;
    const char* const m_tag = "INA";
    const char* const m_device = sf_interfaces::getDeviceName(m_sensor);

    static float sanitize(float value);

    bool sampleSensor(sf_interfaces::InterfaceSensor sensor, Sample& out);
    void publishSensor(const char* topicRoot, const Sample& sample);
};

extern Ina INA_TASK;