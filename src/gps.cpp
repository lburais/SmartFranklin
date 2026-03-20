/*
 * SmartFranklin - GPS module implementation
 * SPDX-License-Identifier: MIT
 *
 * Gravity DFR1103 GNSS/RTC runtime implementation over Wire, with optional
 * PA Hub channel routing. Reads GNSS + RTC values, updates DATA, and publishes
 * MQTT telemetry.
 */

#include "gps.h"

#include <DFRobot_GNSSAndRTC.h>
#include <M5Unified.h>
#include <Wire.h>

#include <cmath>
#include <cstdio>
#include <mutex>

#include "data_model.h"
#include "i2c.h"
#include "mqtt.h"

namespace {

constexpr uint8_t GPS_I2C_ADDRESS = 0x66;
constexpr uint32_t GPS_I2C_CLOCK_HZ = 400000U;
constexpr const char* GPS_DEVICE_FULL_NAME = "DFRobot Gravity GNSS positioning and timing module (DFR1103)";

class GnssRtcExI2C final : public DFRobot_GNSSAndRTC {
public:
    explicit GnssRtcExI2C(const uint8_t address)
        : m_deviceAddress(address)
    {
    }

protected:
    void writeReg(uint8_t reg, void* pBuf, size_t size) override
    {
        if (!M5.Ex_I2C.start(m_deviceAddress, false, GPS_I2C_CLOCK_HZ)) {
            return;
        }

        if (!M5.Ex_I2C.write(reg)) {
            M5.Ex_I2C.stop();
            return;
        }

        auto* bytes = static_cast<uint8_t*>(pBuf);
        for (size_t i = 0; i < size; ++i) {
            if (!M5.Ex_I2C.write(bytes[i])) {
                M5.Ex_I2C.stop();
                return;
            }
        }

        M5.Ex_I2C.stop();
    }

    uint8_t readReg(uint8_t reg, void* pBuf, size_t size) override
    {
        if (!M5.Ex_I2C.start(m_deviceAddress, false, GPS_I2C_CLOCK_HZ)) {
            return 1U;
        }

        if (!M5.Ex_I2C.write(reg) || !M5.Ex_I2C.stop()) {
            return 1U;
        }

        if (!M5.Ex_I2C.start(m_deviceAddress, true, GPS_I2C_CLOCK_HZ)) {
            return 1U;
        }

        auto* bytes = static_cast<uint8_t*>(pBuf);
        for (size_t i = 0; i < size; ++i) {
            const bool lastNack = (i + 1U == size);
            if (!M5.Ex_I2C.read(&bytes[i], 1U, lastNack)) {
                M5.Ex_I2C.stop();
                return 1U;
            }
        }

        return M5.Ex_I2C.stop() ? 0U : 1U;
    }

private:
    uint8_t m_deviceAddress;
};

double applyDirection(const double value, const char direction)
{
    if (direction == 'S' || direction == 'W') {
        return -value;
    }
    return value;
}

class GpsRuntime {
public:
    bool init();
    void process();

private:
    bool detectRoute();
    DFRobot_GNSSAndRTC* activeUnit();
    void publishI2cConfiguration() const;

    mutable std::mutex m_mutex;
    sf_i2c::I2C m_i2c{GPS_I2C_CLOCK_HZ};
    DFRobot_GNSSAndRTC_I2C m_wireUnit{&Wire, GPS_I2C_ADDRESS};
    GnssRtcExI2C m_exUnit{GPS_I2C_ADDRESS};
    bool m_initialized = false;
    sf_i2c::Route m_route;
    // int8_t m_wireSda = -1;
    // int8_t m_wireScl = -1;
};

GpsRuntime GPS_RUNTIME;

}  // namespace

GPS GPS_MODULE;

DFRobot_GNSSAndRTC* GpsRuntime::activeUnit()
{
    if (sf_i2c::isInternalRoute(m_route.mode)) {
        return &m_exUnit;
    }
    return &m_wireUnit;
}

bool GpsRuntime::detectRoute()
{
    m_route = sf_i2c::Route{};
    return m_i2c.detectRoute(GPS_I2C_ADDRESS, m_route);
}

void GpsRuntime::publishI2cConfiguration() const
{
    char pahubChannelBuf[12] = {0};
    char addressBuf[8] = {0};

    snprintf(pahubChannelBuf, sizeof(pahubChannelBuf), "%d", m_route.paHubChannel);
    snprintf(addressBuf, sizeof(addressBuf), "0x%02X", GPS_I2C_ADDRESS);

    sf_mqtt::publish("smartfranklin/system/i2c/gps/mode", sf_i2c::routeModeToString(m_route.mode), 1, true);
    sf_mqtt::publish("smartfranklin/system/i2c/gps/pahub_channel", pahubChannelBuf, 1, true);
    sf_mqtt::publish("smartfranklin/system/i2c/gps/address", addressBuf, 1, true);
    sf_mqtt::publish("smartfranklin/system/i2c/gps/device_name", GPS_DEVICE_FULL_NAME, 1, true);
}

bool GpsRuntime::init()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    m_i2c.beginPortA();
    M5_LOGI("[GPS] initialized Wire port A");

    if (!detectRoute()) {
        m_initialized = false;
        M5_LOGW("[GPS] DFR1103 was not detected on internal/direct/PAHub paths");
        publishI2cConfiguration();
        return false;
    }

    if (sf_i2c::isPaHubRoute(m_route.mode)) {
        if (!m_i2c.selectPaHubChannel(m_route.mode, static_cast<uint8_t>(m_route.paHubChannel))) {
            M5_LOGE("[GPS] failed to select PAHub channel %d", m_route.paHubChannel);
            m_initialized = false;
            publishI2cConfiguration();
            return false;
        }
    }

    DFRobot_GNSSAndRTC* unit = activeUnit();
    const bool initialized = unit->begin();
    if (sf_i2c::isPaHubRoute(m_route.mode)) {
        m_i2c.disablePaHubChannels(m_route.mode);
    }

    if (!initialized) {
        m_initialized = false;
        M5_LOGE("[GPS] DFR1103 initialization failed");
        publishI2cConfiguration();
        return false;
    }

    unit->enablePower();
    m_initialized = true;
    publishI2cConfiguration();
    M5_LOGI("[GPS] DFR1103 initialization complete");
    return true;
}

void GpsRuntime::process()
{
    bool fix = false;
    double latitude = 0.0;
    double longitude = 0.0;
    double altitudeM = 0.0;
    uint8_t satellites = 0;
    char utcDateBuf[16] = {0};
    char utcTimeBuf[16] = {0};
    char rtcTimeBuf[32] = {0};

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_initialized) {
            return;
        }

        if (sf_i2c::isPaHubRoute(m_route.mode)) {
            if (!m_i2c.selectPaHubChannel(m_route.mode, static_cast<uint8_t>(m_route.paHubChannel))) {
                M5_LOGW("[GPS] failed to select PAHub channel %d", m_route.paHubChannel);
                return;
            }
        }

        DFRobot_GNSSAndRTC* unit = activeUnit();
        const auto date = unit->getDate();
        const auto utc = unit->getUTC();
        const auto latData = unit->getLat();
        const auto lonData = unit->getLon();
        altitudeM = unit->getAlt();
        satellites = unit->getNumSatUsed();
        const auto rtc = unit->getRTCTime();

        if (sf_i2c::isPaHubRoute(m_route.mode)) {
            m_i2c.disablePaHubChannels(m_route.mode);
        }

        latitude = applyDirection(latData.latitudeDegree, latData.latDirection);
        longitude = applyDirection(lonData.lonitudeDegree, lonData.lonDirection);

        const bool validNumbers = std::isfinite(latitude) && std::isfinite(longitude) && std::isfinite(altitudeM);
        fix = validNumbers && satellites > 0;

        snprintf(utcDateBuf, sizeof(utcDateBuf), "%04u-%02u-%02u", date.year, date.month, date.date);
        snprintf(utcTimeBuf, sizeof(utcTimeBuf), "%02u:%02u:%02u", utc.hour, utc.minute, utc.second);
        snprintf(rtcTimeBuf,
                 sizeof(rtcTimeBuf),
                 "%04u-%02u-%02uT%02u:%02u:%02uZ",
                 rtc.year,
                 rtc.month,
                 rtc.day,
                 rtc.hour,
                 rtc.minute,
                 rtc.second);
    }

    {
        std::lock_guard<std::mutex> lock(DATA_MUTEX);
        DATA.gps_fix = fix;
        DATA.gps_lat = latitude;
        DATA.gps_lon = longitude;
        DATA.gps_alt_m = altitudeM;
        DATA.gps_satellites = satellites;
        DATA.gps_utc_date = utcDateBuf;
        DATA.gps_utc_time = utcTimeBuf;
        DATA.gps_rtc_time = rtcTimeBuf;
    }

    char latBuf[24] = {0};
    char lonBuf[24] = {0};
    char altBuf[24] = {0};
    char satBuf[8] = {0};

    snprintf(latBuf, sizeof(latBuf), "%.7f", latitude);
    snprintf(lonBuf, sizeof(lonBuf), "%.7f", longitude);
    snprintf(altBuf, sizeof(altBuf), "%.2f", altitudeM);
    snprintf(satBuf, sizeof(satBuf), "%u", satellites);

    M5_LOGI("[GPS] read fix:%u sat:%s lat:%s lon:%s alt_m:%s utc:%sT%sZ",
            fix ? 1U : 0U,
            satBuf,
            latBuf,
            lonBuf,
            altBuf,
            utcDateBuf,
            utcTimeBuf);
    M5_LOGI("[GPS] read rtc:%s", rtcTimeBuf);

    sf_mqtt::publish("smartfranklin/gps/fix", fix ? "1" : "0");
    sf_mqtt::publish("smartfranklin/gps/lat", latBuf);
    sf_mqtt::publish("smartfranklin/gps/lon", lonBuf);
    sf_mqtt::publish("smartfranklin/gps/alt_m", altBuf);
    sf_mqtt::publish("smartfranklin/gps/satellites", satBuf);
    sf_mqtt::publish("smartfranklin/gps/utc/date", utcDateBuf);
    sf_mqtt::publish("smartfranklin/gps/utc/time", utcTimeBuf);
    sf_mqtt::publish("smartfranklin/gps/rtc/time", rtcTimeBuf);
}

bool GPS::init()
{
    return GPS_RUNTIME.init();
}

void GPS::process()
{
    GPS_RUNTIME.process();
}
