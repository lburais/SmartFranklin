/*
 * SPDX-FileCopyrightText: 2026 Laurent Burais
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_DFRobotGPS.cpp
  @brief DFRobot Gravity GNSS (TEL0157, I2C) Unit for M5UnitUnified
*/

#include "DFRobotGPS.hpp"

#include <M5Utility.hpp>   // m5::utility::millis(), delay()
#include <cstdlib>         // atof, atoi
#include <cstring>         // memset, strncpy
#include <cmath>           // floor

using namespace m5::unit::types;
using namespace m5::unit::dfrobotgps;
using namespace m5::utility::mmh3;

namespace m5 {
namespace unit {

// ---------------------------------------------------------------------------
// Static class identity fields (required by M5_UNIT_COMPONENT_HPP_BUILDER)
// ---------------------------------------------------------------------------
const char          UnitDFRobotGPS::name[] = "UnitDFRobotGPS";
const types::uid_t  UnitDFRobotGPS::uid    { "UnitDFRobotGPS"_mmh3 };
const types::attr_t UnitDFRobotGPS::attr   { attribute::AccessI2C };

// ---------------------------------------------------------------------------
// begin()
// ---------------------------------------------------------------------------
bool UnitDFRobotGPS::begin()
{
    // Resize the circular buffer if stored_size() was changed by the manager
    auto ssize = stored_size();
    assert(ssize && "stored_size must be greater than zero");
    if (ssize != _data->capacity()) {
        _data.reset(new m5::container::CircularBuffer<Data>(ssize));
        if (!_data) {
            M5_LIB_LOGE("Failed to allocate circular buffer");
            return false;
        }
    }

    // Poll firmware-version register until the GNSS module is ready (≤ 2 s).
    // The TEL0157 MCU needs a short boot time before the I2C slave is active.
    uint8_t ver   = 0;
    bool    ready = false;
    auto    timeout_at = m5::utility::millis() + 2000UL;
    do {
        ready = readRegister8(static_cast<uint8_t>(REG_SOFT_VERSION), ver, 0)
                && ver != 0;
        if (ready) break;
        m5::utility::delay(100);
    } while (m5::utility::millis() <= timeout_at);

    if (!ready) {
        M5_LIB_LOGE("TEL0157 not responding (firmware reg = 0x%02X)", ver);
        return false;
    }
    M5_LIB_LOGD("TEL0157 firmware: 0x%02X", ver);

    // Configure RGB status LED
    if (!setRGBLed(_cfg.rgb_led)) {
        M5_LIB_LOGW("Failed to set RGB LED mode");
        // Non-fatal: continue
    }

    return _cfg.start_periodic
               ? startPeriodicMeasurement(_cfg.interval)
               : true;
}

// ---------------------------------------------------------------------------
// update()  – called by UnitUnified::update() on every manager tick
// ---------------------------------------------------------------------------
void UnitDFRobotGPS::update(const bool force)
{
    _updated = false;
    if (!inPeriodic()) return;

    elapsed_time_t now = m5::utility::millis();
    if (!force && _latest && now < _latest + _interval) return;

    Data d{};
    _updated = read_gnss(d);
    if (_updated) {
        _latest = now;
        _data->push_back(d);
    }
}

// ---------------------------------------------------------------------------
// Periodic measurement CRTP hooks
// ---------------------------------------------------------------------------
bool UnitDFRobotGPS::start_periodic_measurement(uint32_t interval)
{
    if (inPeriodic()) {
        M5_LIB_LOGD("Already in periodic measurement");
        return false;
    }
    _interval = interval;
    _latest   = 0;
    _periodic = true;
    return true;
}

bool UnitDFRobotGPS::stop_periodic_measurement()
{
    _periodic = false;
    return true;
}

// ---------------------------------------------------------------------------
// Module control
// ---------------------------------------------------------------------------
bool UnitDFRobotGPS::sleep(bool dormant)
{
    const uint8_t mode = dormant ? 0x02U : 0x01U;
    return writeRegister8(static_cast<uint8_t>(REG_SLEEP_MODE), mode);
}

bool UnitDFRobotGPS::wake()
{
    return writeRegister8(static_cast<uint8_t>(REG_SLEEP_MODE), 0x00U);
}

bool UnitDFRobotGPS::setRGBLed(bool enable)
{
    return writeRegister8(static_cast<uint8_t>(REG_RGB_MODE),
                          enable ? 0x01U : 0x00U);
}

// ---------------------------------------------------------------------------
// Private: read all GNSS fields into a Data struct
// ---------------------------------------------------------------------------
bool UnitDFRobotGPS::read_gnss(Data& out)
{
    // --- Fix status ---
    uint8_t status = 0;
    if (!readRegister8(static_cast<uint8_t>(REG_STATUS), status, 0)) {
        M5_LIB_LOGW("Failed to read fix status");
        return false;
    }
    out.fix = (status != 0);
    if (!out.fix) {
        // No fix – return valid but empty data; update() will still push it
        // so callers can see fixValid() == false rather than stale data.
        return true;
    }

    // --- Latitude  "ddmm.mmmmm\0N\0"  (14 bytes) ---
    uint8_t lat_buf[15] = {};
    if (readRegister(static_cast<uint8_t>(REG_LATITUDE), lat_buf,
                     sizeof(lat_buf) - 1U, 0)) {
        out.latitude = parse_nmea_coord(
            reinterpret_cast<const char*>(lat_buf + 0),  // "ddmm.mmmmm"
            static_cast<char>(lat_buf[11])               // 'N' or 'S'
        );
    }

    // --- Longitude "dddmm.mmmmm\0E\0"  (15 bytes) ---
    uint8_t lon_buf[16] = {};
    if (readRegister(static_cast<uint8_t>(REG_LONGITUDE), lon_buf,
                     sizeof(lon_buf) - 1U, 0)) {
        out.longitude = parse_nmea_coord(
            reinterpret_cast<const char*>(lon_buf + 0),
            static_cast<char>(lon_buf[12])               // 'E' or 'W'
        );
    }

    // --- Altitude "mmm.m\0\0\0\0"  (9 bytes) ---
    uint8_t alt_buf[10] = {};
    if (readRegister(static_cast<uint8_t>(REG_ALTITUDE), alt_buf,
                     sizeof(alt_buf) - 1U, 0)) {
        out.altitude = parse_float_field(
            reinterpret_cast<const char*>(alt_buf));
    }

    // --- Speed knots (5 bytes) → km/h ---
    uint8_t spd_buf[6] = {};
    if (readRegister(static_cast<uint8_t>(REG_SPEED), spd_buf,
                     sizeof(spd_buf) - 1U, 0)) {
        const float knots = parse_float_field(
            reinterpret_cast<const char*>(spd_buf));
        out.speed_kmh = knots * 1.852f;
    }

    // --- Heading (7 bytes) ---
    uint8_t hdg_buf[8] = {};
    if (readRegister(static_cast<uint8_t>(REG_HEADING), hdg_buf,
                     sizeof(hdg_buf) - 1U, 0)) {
        out.heading = parse_float_field(
            reinterpret_cast<const char*>(hdg_buf));
    }

    // --- HDOP (5 bytes) ---
    uint8_t hdop_buf[6] = {};
    if (readRegister(static_cast<uint8_t>(REG_HDOP), hdop_buf,
                     sizeof(hdop_buf) - 1U, 0)) {
        out.hdop = parse_float_field(
            reinterpret_cast<const char*>(hdop_buf));
    }

    // --- Satellites (1 byte) ---
    readRegister8(static_cast<uint8_t>(REG_SATELLITES), out.satellites, 0);

    return true;
}

// ---------------------------------------------------------------------------
// Private: NMEA ddmm.mmmmm → decimal degrees
// ---------------------------------------------------------------------------
double UnitDFRobotGPS::parse_nmea_coord(const char* str, char dir)
{
    if (!str || str[0] == '\0') return 0.0;

    const double raw    = atof(str);            // ddmm.mmmmm or dddmm.mmmmm
    const double deg    = std::floor(raw / 100.0);
    const double mins   = raw - deg * 100.0;
    double       result = deg + mins / 60.0;

    if (dir == 'S' || dir == 'W') result = -result;
    return result;
}

// ---------------------------------------------------------------------------
// Private: safe atof on a possibly non-null-terminated buffer copy
// ---------------------------------------------------------------------------
float UnitDFRobotGPS::parse_float_field(const char* str)
{
    if (!str || str[0] == '\0') return 0.0f;
    return static_cast<float>(atof(str));
}

}  // namespace unit
}  // namespace m5
