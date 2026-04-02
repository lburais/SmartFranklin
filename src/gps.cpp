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
#include <Wire.h>

#include <cmath>
#include <cstdio>
#include <cstdint>
#include <mutex>

#include "data_model.h"
#include "mqtt.h"
#include "ports.h"

namespace {

constexpr uint8_t REG_YEAR_H = 0;
constexpr uint8_t REG_HOUR = 4;
constexpr uint8_t REG_LAT_1 = 7;
constexpr uint8_t REG_LON_1 = 13;
constexpr uint8_t REG_USE_STAR = 19;
constexpr uint8_t REG_ALT_H = 20;
constexpr uint8_t REG_SOG_H = 23;
constexpr uint8_t REG_COG_H = 26;
constexpr uint32_t PROCESS_PERIOD_MS = 1000UL;

struct GpsState {
    mutable std::mutex mutex;

    bool initialized = false;

    uint8_t i2cAddress = 0x00;

    double latitudeDeg = 0.0;
    double longitudeDeg = 0.0;
    double altitudeM = 0.0;
    double speedKnots = 0.0;
    double courseDeg = 0.0;
    uint8_t satellites = 0;
    bool hasFix = false;
    uint16_t year = 0;
    uint8_t month = 0;
    uint8_t day = 0;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
};

GpsState GPS_STATE;

/** Write a single device register through the configured I2C port. */
bool writeRegister(const GpsState& state, uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(state.i2cAddress);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

/** Read contiguous registers from GPS through the selected configured port. */
bool readRegisters(const GpsState& state, uint8_t reg, uint8_t* out, size_t len)
{
    if (out == nullptr || len == 0U) {
        return false;
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

/** Decode DFRobot fixed-point (2-byte integer + 1-byte hundredth) format. */
double decodeUnsigned_2_1_100(uint8_t b0, uint8_t b1, uint8_t b2)
{
    const uint16_t highLow = static_cast<uint16_t>((static_cast<uint16_t>(b0 & 0x7F) << 8) | b1);
    return static_cast<double>(highLow) + (static_cast<double>(b2) / 100.0);
}

/** Acquire one complete pose/time sample and update locked state fields. */
bool readPoseAndTimeLocked(GpsState& state)
{
    uint8_t dateTimeRaw[7] = {0};
    uint8_t latRaw[6] = {0};
    uint8_t lonRaw[6] = {0};
    uint8_t satRaw[1] = {0};
    uint8_t altRaw[3] = {0};
    uint8_t sogRaw[3] = {0};
    uint8_t cogRaw[3] = {0};

    if (!readRegisters(state, REG_YEAR_H, dateTimeRaw, sizeof(dateTimeRaw)) ||
        !readRegisters(state, REG_LAT_1, latRaw, sizeof(latRaw)) ||
        !readRegisters(state, REG_LON_1, lonRaw, sizeof(lonRaw)) ||
        !readRegisters(state, REG_USE_STAR, satRaw, sizeof(satRaw)) ||
        !readRegisters(state, REG_ALT_H, altRaw, sizeof(altRaw)) ||
        !readRegisters(state, REG_SOG_H, sogRaw, sizeof(sogRaw)) ||
        !readRegisters(state, REG_COG_H, cogRaw, sizeof(cogRaw))) {
        return false;
    }

    state.year = static_cast<uint16_t>((static_cast<uint16_t>(dateTimeRaw[0]) << 8) | dateTimeRaw[1]);
    state.month = dateTimeRaw[2];
    state.day = dateTimeRaw[3];
    state.hour = dateTimeRaw[4];
    state.minute = dateTimeRaw[5];
    state.second = dateTimeRaw[6];

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

    state.latitudeDeg = latitude;
    state.longitudeDeg = longitude;
    state.satellites = satRaw[0];
    state.altitudeM = decodeUnsigned_2_1_100(altRaw[0], altRaw[1], altRaw[2]);
    state.speedKnots = decodeUnsigned_2_1_100(sogRaw[0], sogRaw[1], sogRaw[2]);
    state.courseDeg = decodeUnsigned_2_1_100(cogRaw[0], cogRaw[1], cogRaw[2]);

    const bool validDate = (state.year >= 2000U && state.month >= 1U && state.month <= 12U && state.day >= 1U && state.day <= 31U);
    const bool validUtc = (state.hour <= 23U && state.minute <= 59U && state.second <= 59U);
    const bool validCoords = std::isfinite(state.latitudeDeg) && std::isfinite(state.longitudeDeg) &&
                             fabs(state.latitudeDeg) <= 90.0 && fabs(state.longitudeDeg) <= 180.0;

    state.hasFix = (state.satellites > 0U) && validDate && validUtc && validCoords;
    return true;
}

/** Initialize GPS backend for requested source and configured port. */
bool initState(GpsState& state, uint8_t i2cAddress)
{
    std::lock_guard<std::mutex> lock(state.mutex);

    state.initialized = false;
    state.i2cAddress = i2cAddress;

    uint8_t probe = 0;
    if (!readRegisters(state, REG_USE_STAR, &probe, 1U)) {
        return false;
    }

    // 0x07 = GPS + BeiDou + GLONASS in DFRobot firmware.
    static_cast<void>(writeRegister(state, 34U, 0x07));

    state.initialized = true;

    M5_LOGI("[GPS] initialized address:0x%02X port:%s",
            state.i2cAddress,
            sf_ports::toString(getName(sf_ports::PortSensor::Gps)));

    return true;
}

/** Publish latest GPS sample to MQTT topics. */
void publishFix(const GpsState& state, const char* dateBuf, const char* utcBuf)
{
    char latBuf[24] = {0};
    char lonBuf[24] = {0};
    char altBuf[24] = {0};
    char sogBuf[24] = {0};
    char cogBuf[24] = {0};
    char satsBuf[8] = {0};

    snprintf(latBuf, sizeof(latBuf), "%.7f", state.latitudeDeg);
    snprintf(lonBuf, sizeof(lonBuf), "%.7f", state.longitudeDeg);
    snprintf(altBuf, sizeof(altBuf), "%.2f", state.altitudeM);
    snprintf(sogBuf, sizeof(sogBuf), "%.2f", state.speedKnots);
    snprintf(cogBuf, sizeof(cogBuf), "%.2f", state.courseDeg);
    snprintf(satsBuf, sizeof(satsBuf), "%u", static_cast<unsigned>(state.satellites));

    sf_mqtt::publish("smartfranklin/gps/has_fix", state.hasFix ? "1" : "0");
    sf_mqtt::publish("smartfranklin/gps/latitude_deg", latBuf);
    sf_mqtt::publish("smartfranklin/gps/longitude_deg", lonBuf);
    sf_mqtt::publish("smartfranklin/gps/altitude_m", altBuf);
    sf_mqtt::publish("smartfranklin/gps/speed_knots", sogBuf);
    sf_mqtt::publish("smartfranklin/gps/course_deg", cogBuf);
    sf_mqtt::publish("smartfranklin/gps/satellites", satsBuf);
    sf_mqtt::publish("smartfranklin/gps/date", dateBuf);
    sf_mqtt::publish("smartfranklin/gps/utc", utcBuf);

    if ( state.hasFix ) {
        M5_LOGI("[GPS] sat=%s lat=%s lon=%s alt=%s date=%s utc=%s", satsBuf, latBuf, lonBuf, altBuf, dateBuf, utcBuf);
    } else {
        M5_LOGI("[GPS] no fix");
    }
}

/** Execute one processing cycle: read, validate, persist and publish. */
void processState(GpsState& state)
{
    char dateBuf[16] = {0};
    char utcBuf[16] = {0};

    {
        std::lock_guard<std::mutex> lock(state.mutex);
        if (!state.initialized) {
            return;
        }

        if (!readPoseAndTimeLocked(state)) {
            M5_LOGW("[GPS] sample read failed");
            return;
        }

        snprintf(dateBuf, sizeof(dateBuf), "%04u-%02u-%02u",
                 static_cast<unsigned>(state.year),
                 static_cast<unsigned>(state.month),
                 static_cast<unsigned>(state.day));
        snprintf(utcBuf, sizeof(utcBuf), "%02u:%02u:%02u",
                 static_cast<unsigned>(state.hour),
                 static_cast<unsigned>(state.minute),
                 static_cast<unsigned>(state.second));
    }

    {
        std::lock_guard<std::mutex> lock(DATA_MUTEX);
        DATA.gps_has_fix = GPS_STATE.hasFix;
        DATA.gps_latitude_deg = GPS_STATE.latitudeDeg;
        DATA.gps_longitude_deg = GPS_STATE.longitudeDeg;
        DATA.gps_altitude_m = GPS_STATE.altitudeM;
        DATA.gps_speed_knots = GPS_STATE.speedKnots;
        DATA.gps_course_deg = GPS_STATE.courseDeg;
        DATA.gps_satellites = GPS_STATE.satellites;
        DATA.gps_date = dateBuf;
        DATA.gps_utc = utcBuf;
    }

    publishFix(GPS_STATE, dateBuf, utcBuf);
}

/** Return module initialization flag from protected state. */
bool isInitializedState(const GpsState& state)
{
    std::lock_guard<std::mutex> lock(state.mutex);
    return state.initialized;
}

}  // namespace

GPS GPS_MODULE;

bool GPS::init()
{
    return initState(GPS_STATE, deviceAddress);
}

void GPS::process()
{
    static uint32_t lastProcessMs = 0;
    const uint32_t now = millis();
    if (now - lastProcessMs < PROCESS_PERIOD_MS) {
        return;
    }
    lastProcessMs = now;
    processState(GPS_STATE);
}

bool GPS::isInitialized() const
{
    return isInitializedState(GPS_STATE);
}
