/**
 * @file gps.cpp
 * @brief GNSS module implementation for DFRobot Gravity GPS over I2C.
 *
 * This unit reads date/time and navigation registers from the GPS device,
 * validates fix quality, updates shared runtime state, and publishes GPS
 * telemetry to MQTT topics used by dashboards and external consumers.
 *
 * SPDX-License-Identifier: MIT
 */

#include "gps.h"

#include <M5Unified.h>
#include <M5Utility.h>
#include <Wire.h>

#include <cmath>
#include <cstdio>
#include <cstdint>
#include <mutex>

#include "config_store.h"
#include "data_model.h"
#include "hmi.h"
#include "mqtt.h"
#include "interfaces.h"
#include "log.h"

namespace {

constexpr uint8_t REG_YEAR_H = 0;
constexpr uint8_t REG_HOUR = 4;
constexpr uint8_t REG_LAT_1 = 7;
constexpr uint8_t REG_LON_1 = 13;
constexpr uint8_t REG_USE_STAR = 19;
constexpr uint8_t REG_ALT_H = 20;
constexpr uint8_t REG_SOG_H = 23;
constexpr uint8_t REG_COG_H = 26;
constexpr uint32_t kGpsI2cClockHz = 400000U;

}  // namespace

GPS GPS_TASK;

bool GPS::writeRegister(const uint8_t reg, const uint8_t value) const
{
    const sf_interfaces::InterfaceConnector connection = sf_interfaces::getConnector(m_sensor);

    TwoWire* connector = connection.ptr.twoWire;
    if (connector == nullptr) {
        HMI::setLed(m_sensor, PortStatus::Error);
        return false;
    }

    connector->beginTransmission(m_i2cAddress);
    connector->write(reg);
    connector->write(value);
    bool ok = connector->endTransmission() == 0;

    return ok;
}

bool GPS::readRegisters(const uint8_t reg, uint8_t* out, const size_t len) const
{
    if (out == nullptr || len == 0U) {
        return false;
    }

    const sf_interfaces::InterfaceConnector connection = sf_interfaces::getConnector(m_sensor);

    TwoWire* connector = connection.ptr.twoWire;
    if (connector == nullptr) {
        HMI::setLed(m_sensor, PortStatus::Error);
        return false;
    }

    connector->beginTransmission(m_i2cAddress);
    connector->write(reg);
    if (connector->endTransmission(false) != 0) {
        return false;
    }

    const size_t readCount = connector->requestFrom(m_i2cAddress, static_cast<uint8_t>(len));
    if (readCount < len) {
        return false;
    }

    for (size_t i = 0; i < len; ++i) {
        out[i] = static_cast<uint8_t>(connector->read());
    }

    return true;
}

double GPS::decodeUnsigned_2_1_100(const uint8_t b0, const uint8_t b1, const uint8_t b2)
{
    const uint16_t highLow = static_cast<uint16_t>((static_cast<uint16_t>(b0 & 0x7F) << 8) | b1);
    return static_cast<double>(highLow) + (static_cast<double>(b2) / 100.0);
}

bool GPS::readPoseAndTimeLocked()
{
    uint8_t dateTimeRaw[7] = {0};
    uint8_t latRaw[6] = {0};
    uint8_t lonRaw[6] = {0};
    uint8_t satRaw[1] = {0};
    uint8_t altRaw[3] = {0};
    uint8_t sogRaw[3] = {0};
    uint8_t cogRaw[3] = {0};

    if (!readRegisters(REG_YEAR_H, dateTimeRaw, sizeof(dateTimeRaw)) ||
        !readRegisters(REG_LAT_1, latRaw, sizeof(latRaw)) ||
        !readRegisters(REG_LON_1, lonRaw, sizeof(lonRaw)) ||
        !readRegisters(REG_USE_STAR, satRaw, sizeof(satRaw)) ||
        !readRegisters(REG_ALT_H, altRaw, sizeof(altRaw)) ||
        !readRegisters(REG_SOG_H, sogRaw, sizeof(sogRaw)) ||
        !readRegisters(REG_COG_H, cogRaw, sizeof(cogRaw))) {
        return false;
    }

    m_year = static_cast<uint16_t>((static_cast<uint16_t>(dateTimeRaw[0]) << 8) | dateTimeRaw[1]);
    m_month = dateTimeRaw[2];
    m_day = dateTimeRaw[3];
    m_hour = dateTimeRaw[4];
    m_minute = dateTimeRaw[5];
    m_second = dateTimeRaw[6];

    const uint8_t latDD = latRaw[0];
    const uint8_t latMM = latRaw[1];
    const uint32_t latMMMMM =
        (static_cast<uint32_t>(latRaw[2]) << 16) |
        (static_cast<uint32_t>(latRaw[3]) << 8) |
        static_cast<uint32_t>(latRaw[4]);
    const char latDir = static_cast<char>(latRaw[5]);

    const uint8_t lonDDD = lonRaw[0];
    const uint8_t lonMM = lonRaw[1];
    const uint32_t lonMMMMM =
        (static_cast<uint32_t>(lonRaw[2]) << 16) |
        (static_cast<uint32_t>(lonRaw[3]) << 8) |
        static_cast<uint32_t>(lonRaw[4]);
    const char lonDir = static_cast<char>(lonRaw[5]);

    double latitude = static_cast<double>(latDD) +
                      (static_cast<double>(latMM) / 60.0) +
                      ((static_cast<double>(latMMMMM) / 100000.0) / 60.0);

    double longitude = static_cast<double>(lonDDD) +
                       (static_cast<double>(lonMM) / 60.0) +
                       ((static_cast<double>(lonMMMMM) / 100000.0) / 60.0);

    if (latDir == 'S' || latDir == 's') {
        latitude = -latitude;
    }
    if (lonDir == 'W' || lonDir == 'w') {
        longitude = -longitude;
    }

    m_latitudeDeg = latitude;
    m_longitudeDeg = longitude;
    m_satellites = satRaw[0];
    m_altitudeM = decodeUnsigned_2_1_100(altRaw[0], altRaw[1], altRaw[2]);
    m_speedKnots = decodeUnsigned_2_1_100(sogRaw[0], sogRaw[1], sogRaw[2]);
    m_courseDeg = decodeUnsigned_2_1_100(cogRaw[0], cogRaw[1], cogRaw[2]);

    const bool validDate = (m_year >= 2000U && m_month >= 1U && m_month <= 12U && m_day >= 1U && m_day <= 31U);
    const bool validUtc = (m_hour <= 23U && m_minute <= 59U && m_second <= 59U);
    const bool validCoords = std::isfinite(m_latitudeDeg) && std::isfinite(m_longitudeDeg) &&
                             fabs(m_latitudeDeg) <= 90.0 && fabs(m_longitudeDeg) <= 180.0;

    m_hasFix = (m_satellites > 0U) && validDate && validUtc && validCoords;
    return true;
}

bool GPS::isInitialized() const
{
    return m_initialized;
}

bool GPS::init()
{
    SF_LOGI("[%s] init", m_tag);

    m_initialized        = false;

    if (!sf_interfaces::configured(m_sensor)) {
        SF_LOGW("[%s] configuration required", m_tag);
        if (!sf_interfaces::configure(m_sensor)) {
            SF_LOGW("[%s] configuration failed", m_tag);
            HMI::setLed(m_sensor, PortStatus::Error);
            return false;
        }
    }

    if (!seize(m_sensor)) {
        SF_LOGW("[%s] unable to lock port", m_tag);
        HMI::setLed(m_sensor, PortStatus::Error);
        return false;
    }

    uint8_t probe = 0;
    if (!readRegisters(REG_USE_STAR, &probe, 1U)) {
        SF_LOGE("[%s] cannot read register 0x%02X", m_tag, REG_USE_STAR);
        release(m_sensor);
        return false;
    }

    // 0x07 = GPS + BeiDou + GLONASS in DFRobot firmware.
    static_cast<void>(writeRegister(34U, 0x07));

    m_initialized = true;

    SF_LOGI("[%s] (0x%02X) initialized", m_tag, sf_interfaces::getAddress(m_sensor));

    release(m_sensor);

    return true;
}

bool GPS::process()
{
    char dateBuf[16] = {0};
    char utcBuf[16] = {0};

    if (!m_initialized) {
        if (!init()) {
            SF_LOGW("[%s] not configured", m_tag);
            HMI::setLed(m_sensor, PortStatus::Error);
            return false;
        }
    }

    if (!seize(m_sensor)) {
        SF_LOGW("[%s] unable to lock port", m_tag);
        HMI::setLed(m_sensor, PortStatus::Error);
        return false;
    }

    if (!readPoseAndTimeLocked()) {
        SF_LOGW("[%s] sample read failed", m_tag);
        HMI::setLed(m_sensor, PortStatus::NoData);
        release(m_sensor);
        return false;
    }    

    snprintf(dateBuf, sizeof(dateBuf), "%04u-%02u-%02u",
             static_cast<unsigned>(m_year),
             static_cast<unsigned>(m_month),
             static_cast<unsigned>(m_day));
    snprintf(utcBuf, sizeof(utcBuf), "%02u:%02u:%02u",
             static_cast<unsigned>(m_hour),
             static_cast<unsigned>(m_minute),
             static_cast<unsigned>(m_second));

    {
        std::lock_guard<std::mutex> dataLock(DATA_MUTEX);
        DATA.gps_has_fix = m_hasFix;
        DATA.gps_latitude_deg = m_latitudeDeg;
        DATA.gps_longitude_deg = m_longitudeDeg;
        DATA.gps_altitude_m = m_altitudeM;
        DATA.gps_speed_knots = m_speedKnots;
        DATA.gps_course_deg = m_courseDeg;
        DATA.gps_satellites = m_satellites;
        DATA.gps_date = dateBuf;
        DATA.gps_utc = utcBuf;
    }

    // publishFix(dateBuf, utcBuf);
    char latBuf[24] = {0};
    char lonBuf[24] = {0};
    char altBuf[24] = {0};
    char sogBuf[24] = {0};
    char cogBuf[24] = {0};
    char satsBuf[8] = {0};

    snprintf(latBuf, sizeof(latBuf), "%.7f", m_latitudeDeg);
    snprintf(lonBuf, sizeof(lonBuf), "%.7f", m_longitudeDeg);
    snprintf(altBuf, sizeof(altBuf), "%.2f", m_altitudeM);
    snprintf(sogBuf, sizeof(sogBuf), "%.2f", m_speedKnots);
    snprintf(cogBuf, sizeof(cogBuf), "%.2f", m_courseDeg);
    snprintf(satsBuf, sizeof(satsBuf), "%u", static_cast<unsigned>(m_satellites));

    sf_mqtt::publish("smartfranklin/gps/has_fix", m_hasFix ? "1" : "0");
    sf_mqtt::publish("smartfranklin/gps/latitude_deg", latBuf);
    sf_mqtt::publish("smartfranklin/gps/longitude_deg", lonBuf);
    sf_mqtt::publish("smartfranklin/gps/altitude_m", altBuf);
    sf_mqtt::publish("smartfranklin/gps/speed_knots", sogBuf);
    sf_mqtt::publish("smartfranklin/gps/course_deg", cogBuf);
    sf_mqtt::publish("smartfranklin/gps/satellites", satsBuf);
    sf_mqtt::publish("smartfranklin/gps/date", dateBuf);
    sf_mqtt::publish("smartfranklin/gps/utc", utcBuf);

    if (m_hasFix) {
        SF_LOGI("[%s] sat=%s lat=%s lon=%s alt=%s date=%s utc=%s", m_tag, satsBuf, latBuf, lonBuf, altBuf, dateBuf, utcBuf);
    } else {
        SF_LOGI("[%s] no fix", m_tag);
    }

    HMI::setLed(m_sensor, PortStatus::Ok);

    release(m_sensor);

    return true;
}

void taskGps(void* pv)
{
    (void)pv;
    SF_LOGI("[GPS] Task started");

    uint32_t nextInitAttemptMs = 0;

    auto isRetryDue = [](uint32_t nowMs, uint32_t nextAttemptMs) {
        return static_cast<int32_t>(nowMs - nextAttemptMs) >= 0;
    };

    auto scheduleRetry = [](uint32_t& nextAttemptMs, uint32_t nowMs) {
        nextAttemptMs = nowMs + 10000UL;  // 10 second retry interval
    };

    for (;;) {
        const uint32_t nowMs = millis();

        if (!GPS_TASK.isInitialized() && isRetryDue(nowMs, nextInitAttemptMs)) {
            if (!GPS_TASK.init()) {
                SF_LOGW("[GPS] Init failed");
                scheduleRetry(nextInitAttemptMs, nowMs);
            }
        }

        if (GPS_TASK.isInitialized()) {
            GPS_TASK.process();
        }

        const uint32_t recurrenceMs = sf_interfaces::getRecurrenceMs(sf_interfaces::InterfaceSensor::Gps);
        const int loopMs = (recurrenceMs > 0) ? static_cast<int>(recurrenceMs) : 1000;
        vTaskDelay(pdMS_TO_TICKS(loopMs));
    }
}
