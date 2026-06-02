#include "ina.h"

#include <M5Unified.h>

#include <cmath>
#include <cstdio>

#include "data_model.h"
#include "hmi.h"
#include "interfaces.h"
#include "log.h"
#include "mqtt.h"

Ina INA_TASK;

bool Ina::isInitialized() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_initialized;
}

float Ina::sanitize(const float value)
{
    if (!std::isfinite(value)) {
        return 0.0f;
    }
    return value;
}

bool Ina::init()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    m_initialized = false;
    m_ina1Available = false;
    m_ina2Available = false;

    if (sf_interfaces::configure(sf_interfaces::InterfaceSensor::Ina1)) {
        if (!sf_interfaces::seize(sf_interfaces::InterfaceSensor::Ina1)) {
            SF_LOGW("[%s] INA1 lock failed during init", m_tag);
            HMI::setLed(sf_interfaces::InterfaceSensor::Ina1, PortStatus::Error);
        } else {
            m_ina1Available = M5.Power.Ina3221[0].begin();
            sf_interfaces::release(sf_interfaces::InterfaceSensor::Ina1);
            HMI::setLed(sf_interfaces::InterfaceSensor::Ina1, m_ina1Available ? PortStatus::Initialized : PortStatus::Error);
        }
    } else {
        HMI::setLed(sf_interfaces::InterfaceSensor::Ina1, PortStatus::Error);
    }

    if (sf_interfaces::configure(sf_interfaces::InterfaceSensor::Ina2)) {
        if (!sf_interfaces::seize(sf_interfaces::InterfaceSensor::Ina2)) {
            SF_LOGW("[%s] INA2 lock failed during init", m_tag);
            HMI::setLed(sf_interfaces::InterfaceSensor::Ina2, PortStatus::Error);
        } else {
            m_ina2Available = M5.Power.Ina3221[1].begin();
            sf_interfaces::release(sf_interfaces::InterfaceSensor::Ina2);
            HMI::setLed(sf_interfaces::InterfaceSensor::Ina2, m_ina2Available ? PortStatus::Initialized : PortStatus::Error);
        }
    } else {
        HMI::setLed(sf_interfaces::InterfaceSensor::Ina2, PortStatus::Error);
    }

    m_initialized = (m_ina1Available || m_ina2Available);
    if (!m_initialized) {
        SF_LOGW("[%s] initialization failed (ina1=%d ina2=%d)", m_tag, m_ina1Available, m_ina2Available);
        return false;
    }

    SF_LOGI("[%s] initialized (ina1=%d ina2=%d)", m_tag, m_ina1Available, m_ina2Available);
    return true;
}

bool Ina::sampleSensor(const sf_interfaces::InterfaceSensor sensor, Sample& out)
{
    if (!sf_interfaces::seize(sensor)) {
        return false;
    }

    const uint8_t index = (sensor == sf_interfaces::InterfaceSensor::Ina2) ? 1U : 0U;
    bool ok = true;

    for (size_t i = 0; i < kChannelCount; ++i) {
        const float rawBusV = M5.Power.Ina3221[index].getBusVoltage(i);
        const float rawShuntV = M5.Power.Ina3221[index].getShuntVoltage(i);
        const float rawCurrentA = M5.Power.Ina3221[index].getCurrent(i);

        if (!std::isfinite(rawBusV) || !std::isfinite(rawShuntV) || !std::isfinite(rawCurrentA)) {
            ok = false;
            break;
        }

        const float busV = sanitize(rawBusV);
        const float shuntV = sanitize(rawShuntV);
        const float currentA = sanitize(rawCurrentA);

        out.shuntVoltageMv[i] = shuntV * 1000.0f;
        out.busVoltageV[i] = busV;
        out.currentA[i] = currentA;
    }

    sf_interfaces::release(sensor);

    out.ok = ok;
    return ok;
}

void Ina::publishSensor(const char* topicRoot, const Sample& sample)
{
    sf_mqtt::publish(topicRoot, sample.ok ? "1" : "0", 1, true);

    for (size_t i = 0; i < kChannelCount; ++i) {
        char topic[96] = {0};
        char value[24] = {0};

        snprintf(topic, sizeof(topic), "%s/ch%u/bus_voltage_v", topicRoot, static_cast<unsigned>(i + 1));
        snprintf(value, sizeof(value), "%.3f", sample.busVoltageV[i]);
        sf_mqtt::publish(topic, value, 1, true);

        snprintf(topic, sizeof(topic), "%s/ch%u/shunt_voltage_mv", topicRoot, static_cast<unsigned>(i + 1));
        snprintf(value, sizeof(value), "%.3f", sample.shuntVoltageMv[i]);
        sf_mqtt::publish(topic, value, 1, true);

        snprintf(topic, sizeof(topic), "%s/ch%u/current_a", topicRoot, static_cast<unsigned>(i + 1));
        snprintf(value, sizeof(value), "%.3f", sample.currentA[i]);
        sf_mqtt::publish(topic, value, 1, true);
    }
}

void Ina::process()
{
    bool initialized = false;
    bool ina1Available = false;
    bool ina2Available = false;

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        initialized = m_initialized;
        ina1Available = m_ina1Available;
        ina2Available = m_ina2Available;
    }

    if (!initialized) {
        return;
    }

    Sample ina1{};
    Sample ina2{};

    if (ina1Available) {
        (void)sampleSensor(sf_interfaces::InterfaceSensor::Ina1, ina1);
    }

    if (ina2Available) {
        (void)sampleSensor(sf_interfaces::InterfaceSensor::Ina2, ina2);
    }

    {
        std::lock_guard<std::mutex> lock(DATA_MUTEX);

        DATA.ina1_online = ina1.ok;
        DATA.ina2_online = ina2.ok;

        DATA.ina1_bus_voltage_ch1_v = ina1.busVoltageV[0];
        DATA.ina1_bus_voltage_ch2_v = ina1.busVoltageV[1];
        DATA.ina1_bus_voltage_ch3_v = ina1.busVoltageV[2];
        DATA.ina1_shunt_voltage_ch1_mv = ina1.shuntVoltageMv[0];
        DATA.ina1_shunt_voltage_ch2_mv = ina1.shuntVoltageMv[1];
        DATA.ina1_shunt_voltage_ch3_mv = ina1.shuntVoltageMv[2];
        DATA.ina1_current_ch1_a = ina1.currentA[0];
        DATA.ina1_current_ch2_a = ina1.currentA[1];
        DATA.ina1_current_ch3_a = ina1.currentA[2];

        DATA.ina2_bus_voltage_ch1_v = ina2.busVoltageV[0];
        DATA.ina2_bus_voltage_ch2_v = ina2.busVoltageV[1];
        DATA.ina2_bus_voltage_ch3_v = ina2.busVoltageV[2];
        DATA.ina2_shunt_voltage_ch1_mv = ina2.shuntVoltageMv[0];
        DATA.ina2_shunt_voltage_ch2_mv = ina2.shuntVoltageMv[1];
        DATA.ina2_shunt_voltage_ch3_mv = ina2.shuntVoltageMv[2];
        DATA.ina2_current_ch1_a = ina2.currentA[0];
        DATA.ina2_current_ch2_a = ina2.currentA[1];
        DATA.ina2_current_ch3_a = ina2.currentA[2];
    }

    publishSensor("smartfranklin/ina1", ina1);
    publishSensor("smartfranklin/ina2", ina2);

    HMI::setLed(sf_interfaces::InterfaceSensor::Ina1, ina1.ok ? PortStatus::Ok : PortStatus::Error);
    HMI::setLed(sf_interfaces::InterfaceSensor::Ina2, ina2.ok ? PortStatus::Ok : PortStatus::Error);

    if (ina1.ok ) {
        for (int ch = 0; ch < kChannelCount; ++ch) {
            SF_LOGI(
                "[%s] INA1 CH%d: Voltage = %.3f V current = %.3f A shunt = %.3f mV", 
                m_tag, 
                ch, 
                ina1.busVoltageV[ch], 
                ina1.currentA[ch], 
                ina1.busVoltageV[ch]);
        }
    } else {
        SF_LOGI("[%s] INA1 KO", m_tag);
    }
    if (ina2.ok ) {
        for (int ch = 0; ch < kChannelCount; ++ch) {
            SF_LOGI(
                "[%s] INA2 CH%d: Voltage = %.3f V current = %.3f A shunt = %.3f mV", 
                m_tag, 
                ch, 
                ina2.busVoltageV[ch], 
                ina2.currentA[ch], 
                ina2.busVoltageV[ch]);
        }
    } else {
        SF_LOGI("[%s] INA2 KO", m_tag);
    }
}