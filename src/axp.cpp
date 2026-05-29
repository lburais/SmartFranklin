#include "axp.h"

#include <M5Unified.h>

#include <cmath>
#include <cstdio>

#include "data_model.h"
#include "interfaces.h"
#include "mqtt.h"
#include "hmi.h"
#include "log.h"

Axp AXP_TASK;

bool Axp::isInitialized() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_initialized;
}

bool Axp::init()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    m_initialized = true;
    HMI::setLed(sf_interfaces::InterfaceSensor::Axp, PortStatus::Initialized);
    SF_LOGI("[%s] initialized", m_tag);

    return true;
}

void Axp::process()
{
    float batteryVoltageV = 0.0f;
    int batteryPercent = 0;
    bool charging = false;
    float temperatureC = 0.0f;

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_initialized) {
            return;
        }

        batteryVoltageV = M5.Power.getBatteryVoltage() / 1000.0f;
        batteryPercent = M5.Power.getBatteryLevel();
        charging = M5.Power.isCharging();
        switch (M5.Power.getType()) {
        case m5::Power_Class::pmic_axp192:
            temperatureC = M5.Power.Axp192.getInternalTemperature();
            break;
        case m5::Power_Class::pmic_axp2101:
            temperatureC = M5.Power.Axp2101.getInternalTemperature();
            break;
        default:
            temperatureC = 0.0f;
            break;
        }

        if (!std::isfinite(batteryVoltageV) || batteryVoltageV < 0.0f) {
            batteryVoltageV = 0.0f;
        }
        if (batteryPercent < 0) {
            batteryPercent = 0;
        } else if (batteryPercent > 100) {
            batteryPercent = 100;
        }
        if (!std::isfinite(temperatureC)) {
            temperatureC = 0.0f;
        }

        m_lastBatteryVoltageV = batteryVoltageV;
        m_lastBatteryPercent = batteryPercent;
        m_lastCharging = charging;
        m_lastTemperatureC = temperatureC;
    }

    {
        std::lock_guard<std::mutex> lock(DATA_MUTEX);
        DATA.axp_battery_voltage = batteryVoltageV;
        DATA.axp_battery_percent = batteryPercent;
        DATA.axp_charging = charging;
        DATA.axp_temperature = temperatureC;
    }

    char voltageBuf[24] = {0};
    char percentBuf[16] = {0};
    char tempBuf[24] = {0};

    snprintf(voltageBuf, sizeof(voltageBuf), "%.3f", batteryVoltageV);
    snprintf(percentBuf, sizeof(percentBuf), "%d", batteryPercent);
    snprintf(tempBuf, sizeof(tempBuf), "%.2f", temperatureC);

    sf_mqtt::publish("smartfranklin/axp/battery_voltage", voltageBuf, 1, true);
    sf_mqtt::publish("smartfranklin/axp/battery_percent", percentBuf, 1, true);
    sf_mqtt::publish("smartfranklin/axp/charging", charging ? "1" : "0", 1, true);
    sf_mqtt::publish("smartfranklin/axp/temperature", tempBuf, 1, true);

    if (batteryPercent > 0.1) {
        HMI::setLed(sf_interfaces::InterfaceSensor::Axp, PortStatus::Ok);
    } else {
        HMI::setLed(sf_interfaces::InterfaceSensor::Axp, PortStatus::BatteryLow);
    }

    SF_LOGI("[%s] Voltage: %.3f V     Fill: %d%%    Charging: %d     Temp: %.2f°", m_tag, batteryVoltageV, batteryPercent, charging, temperatureC);
}
