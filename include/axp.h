#pragma once

#include <Arduino.h>

#include <mutex>

#include "interfaces.h"

class Axp {
public:
    bool init();
    void process();
    bool isInitialized() const;

private:
    mutable std::mutex m_mutex;

    bool m_initialized = false;

    const sf_interfaces::InterfaceSensor m_sensor = sf_interfaces::InterfaceSensor::Axp;
    const char* const m_tag = sf_interfaces::toString(m_sensor, true);
    const char* const m_device = sf_interfaces::getDeviceName(m_sensor);

    float m_lastBatteryVoltageV = 0.0f;
    int m_lastBatteryPercent = 0;
    bool m_lastCharging = false;
    float m_lastTemperatureC = 0.0f;
};

extern Axp AXP_TASK;
