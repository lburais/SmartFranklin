/*
 * ============================================================================
 * GPS Module Implementation - SmartFranklin
 * ============================================================================
 *
 * File:        gps.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: Gravity DFR1103 GNSS/RTC implementation over I2C (Wire)
 *              with optional PA Hub channel routing. Reads GNSS + RTC values,
 *              updates the shared DATA model, publishes MQTT topics, and
 *              provides health information used by the GPS FreeRTOS task.
 *
 * Author:      Laurent Burais
 * Date:        10 March 2026
 * Version:     1.0
 *
 * Overview:
 *   This module encapsulates all low-level interaction with the Gravity
 *   DFR1103 unit. It hides I2C path detection, GNSS/RTC sampling, data
 *   conversion, and publication details from the task layer.
 *
 * I2C Routing Strategy:
 *   1. Try direct Wire access to DFR1103 address (0x66).
 *   2. If not found, probe PA Hub (0x70) and scan channels 0..7.
 *   3. Cache discovered route for periodic reads.
 *   4. When PA Hub is used, select channel before each read cycle and
 *      disable channels after reads to avoid side effects on other devices.
 *
 * Read Cycle Outputs:
 *   - GNSS date/time (UTC)
 *   - Latitude/longitude in signed decimal degrees
 *   - Altitude in meters
 *   - Number of satellites used
 *   - DFR1103 RTC timestamp
 *
 * Data Model Integration:
 *   - Writes to DATA.gps_fix
 *   - Writes to DATA.gps_lat / DATA.gps_lon / DATA.gps_alt_m
 *   - Writes to DATA.gps_satellites
 *   - Writes to DATA.gps_utc_date / DATA.gps_utc_time / DATA.gps_rtc_time
 *   - All writes are protected by DATA_MUTEX
 *
 * MQTT Topics Published:
 *   - smartfranklin/gps/fix
 *   - smartfranklin/gps/lat
 *   - smartfranklin/gps/lon
 *   - smartfranklin/gps/alt_m
 *   - smartfranklin/gps/satellites
 *   - smartfranklin/gps/utc/date
 *   - smartfranklin/gps/utc/time
 *   - smartfranklin/gps/rtc/time
 *
 * Health Model:
 *   - m_initialized tracks successful startup.
 *   - m_lastReadOk tracks validity of latest sample.
 *   - m_lastReadMs tracks freshness of latest valid cycle.
 *   - isHealthy() returns true only if all three checks pass.
 *
 * Error Handling:
 *   - Missing device during initialization returns false.
 *   - PA Hub channel selection failures mark read as failed.
 *   - Non-finite coordinates/altitude force invalid fix state.
 *   - Route state is reset when detection fails.
 *
 * Performance Notes:
 *   - process() performs one synchronous sample-and-publish cycle.
 *   - No dynamic allocation in the hot path.
 *   - String formatting uses fixed-size stack buffers.
 *
 * Dependencies:
 *   - M5Unified (logging and pin mapping)
 *   - DFRobot_GNSSAndRTC (DFR1103 driver)
 *   - Wire (I2C transport)
 *   - data_model.h (shared runtime state)
 *   - mqtt_layer.h (topic publishing)
 *   - pahub_channels.h (PA Hub address)
 *   - tasks.h (period constants for health timeout)
 *
 * Limitations:
 *   - Assumes factory DFR1103 I2C address (0x66).
 *   - Single active DFR1103 instance is supported.
 *   - Health is based on sample validity/freshness, not HDOP or geofence quality.
 *
 * ============================================================================
 * MIT License
 * ============================================================================
 * Copyright (c) 2026 Laurent Burais
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * ============================================================================
 */

#include "gps.h"

#include <M5Unified.h>

#include <cmath>

#include "data_model.h"
#include "mqtt_layer.h"
#include "pahub_channels.h"
#include "tasks.h"

namespace {
// ============================================================================
// Module-Private Constants And Helpers
// ============================================================================

/**
 * @brief Number of PA Hub channels to scan for DFR1103.
 */
constexpr uint8_t PAHUB_CHANNEL_COUNT = 8;

/**
 * @brief Health timeout window for GPS task status checks.
 *
 * Health is considered valid only if the latest read cycle succeeded and the
 * elapsed time since that read is within this window.
 */
constexpr uint32_t GPS_HEALTH_TIMEOUT_MS = (PERIOD_GPS * 3UL) + 5000UL;

/**
 * @brief Converts hemisphere-tagged coordinates to signed decimal degrees.
 *
 * DFR1103 latitude/longitude structures provide a numeric degree value and
 * a hemisphere marker. This helper maps southern/western coordinates to
 * negative values so the resulting representation is standard WGS84 signed
 * decimal notation.
 *
 * @param value Coordinate magnitude in decimal degrees.
 * @param direction Hemisphere indicator ('N', 'S', 'E', or 'W').
 * @return Signed decimal degree value.
 */
static double applyDirection(const double value, const char direction)
{
    if (direction == 'S' || direction == 'W') {
        return -value;
    }
    return value;
}
}  // namespace

/**
 * @brief Global GPS module singleton used by taskGps.
 */
GPS GPS_MODULE;

/**
 * @brief Probes a device address on Wire.
 * @param address I2C address to probe.
 * @return true when device responds with ACK.
 */
bool GPS::probeAddressOnWire(const uint8_t address)
{
    // A successful endTransmission() means the device ACKed this address.
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

/**
 * @brief Selects one PA Hub channel for downstream Wire access.
 * @param channel PA Hub channel index.
 * @return true when channel command succeeds.
 */
bool GPS::selectPaHubChannel(const uint8_t channel)
{
    // PA Hub expects one-hot bit mask: bit[channel] = 1 selects that lane.
    Wire.beginTransmission(PAHUB_ADDRESS);
    Wire.write(static_cast<uint8_t>(1U << channel));
    return Wire.endTransmission() == 0;
}

/**
 * @brief Disables all PA Hub channels.
 */
void GPS::disablePaHubChannels()
{
    Wire.beginTransmission(PAHUB_ADDRESS);
    Wire.write(static_cast<uint8_t>(0x00));
    Wire.endTransmission();
}

/**
 * @brief Detects the path used to reach DFR1103 over Wire.
 *
 * Detection order:
 * 1. Direct Wire path (module connected directly to Port A bus).
 * 2. PA Hub path (scan channels 0..7 and probe module address).
 *
 * On success, the function stores route details in m_busMode/m_pahubChannel.
 * On failure, route state is reset to Unset/-1.
 *
 * @return true when direct or PA Hub path is found.
 */
bool GPS::detectBusPath()
{
    // First try direct Wire routing (no PA Hub multiplexing).
    if (probeAddressOnWire(DFR1103_I2C_ADDRESS)) {
        m_busMode = BusMode::WireDirect;
        m_pahubChannel = -1;
        M5_LOGI("[GPS] DFR1103 detected on direct Wire bus");
        return true;
    }

    // If PA Hub itself is unreachable, no multiplexed route is possible.
    if (!probeAddressOnWire(PAHUB_ADDRESS)) {
        m_busMode = BusMode::Unset;
        m_pahubChannel = -1;
        return false;
    }

    // Then scan all PA Hub channels to find where DFR1103 is routed.
    for (uint8_t channel = 0; channel < PAHUB_CHANNEL_COUNT; ++channel) {
        if (!selectPaHubChannel(channel)) {
            continue;
        }

        if (probeAddressOnWire(DFR1103_I2C_ADDRESS)) {
            m_busMode = BusMode::WirePaHub;
            m_pahubChannel = static_cast<int8_t>(channel);
            M5_LOGI("[GPS] DFR1103 detected on PAHub channel %u", channel);
            return true;
        }
    }

    disablePaHubChannels();
    m_busMode = BusMode::Unset;
    m_pahubChannel = -1;
    return false;
}

/**
 * @brief Initializes Wire routing and DFR1103 module state.
 *
 * Initialization flow:
 * 1. Resolve Port A SDA/SCL pins.
 * 2. Configure Wire at 400kHz.
 * 3. Detect module route (direct or PA Hub).
 * 4. If PA Hub route is used, select the detected channel.
 * 5. Call DFRobot begin() and re-apply Wire pin configuration.
 * 6. Enable GNSS power and initialize health timestamps.
 *
 * @return true on successful module startup.
 */
bool GPS::init()
{
    // Resolve physical Port A pins from board-specific M5 mapping.
    m_wireSda = static_cast<int8_t>(M5.getPin(m5::pin_name_t::port_a_sda));
    m_wireScl = static_cast<int8_t>(M5.getPin(m5::pin_name_t::port_a_scl));
    M5_LOGI("[GPS] using Wire SDA:%d SCL:%d", m_wireSda, m_wireScl);

    // Reset and reinitialize Wire to ensure the intended pin/speed setup.
    Wire.end();
    Wire.begin(m_wireSda, m_wireScl, 400000U);
    Wire.setPins(m_wireSda, m_wireScl);

    if (!detectBusPath()) {
        M5_LOGW("[GPS] DFR1103 was not detected on supported Wire paths");
        m_initialized = false;
        return false;
    }

    if (m_busMode == BusMode::WirePaHub && m_pahubChannel >= 0) {
        if (!selectPaHubChannel(static_cast<uint8_t>(m_pahubChannel))) {
            M5_LOGE("[GPS] failed to select PAHub channel %d", m_pahubChannel);
            m_initialized = false;
            return false;
        }
    }

    // Start DFRobot driver and underlying communication sequence.
    m_initialized = m_unit.begin();

    // DFRobot begin() touches Wire state internally, re-apply selected bus pins.
    Wire.begin(m_wireSda, m_wireScl, 400000U);

    if (m_busMode == BusMode::WirePaHub) {
        disablePaHubChannels();
    }

    if (!m_initialized) {
        M5_LOGE("[GPS] DFR1103 initialization failed");
        return false;
    }

    m_unit.enablePower();
    m_lastReadOk = false;
    m_lastReadMs = millis();

    M5_LOGI("[GPS] DFR1103 initialization complete");
    return true;
}

/**
 * @brief Executes one GNSS/RTC sample and publish cycle.
 *
 * This function reads GNSS and RTC values from DFR1103, updates the global
 * DATA model under mutex protection, logs the sampled values, and publishes
 * dedicated MQTT topics for fix/coordinates/altitude/satellites/time.
 *
 * If PA Hub routing is active, the channel is selected before reads and
 * disabled afterward to avoid affecting other devices.
 */
void GPS::readAndPublish()
{
    if (m_busMode == BusMode::WirePaHub && m_pahubChannel >= 0) {
        if (!selectPaHubChannel(static_cast<uint8_t>(m_pahubChannel))) {
            M5_LOGW("[GPS] failed to reselect PAHub channel %d", m_pahubChannel);
            m_lastReadOk = false;
            return;
        }
    }

    // Sample one complete GNSS + RTC frame from the module.
    const auto date = m_unit.getDate();
    const auto utc = m_unit.getUTC();
    const auto latData = m_unit.getLat();
    const auto lonData = m_unit.getLon();
    const double altitudeM = m_unit.getAlt();
    const uint8_t satellites = m_unit.getNumSatUsed();
    const auto rtc = m_unit.getRTCTime();

    if (m_busMode == BusMode::WirePaHub) {
        disablePaHubChannels();
    }

    // Normalize coordinates to signed decimal notation.
    const double latitude = applyDirection(latData.latitudeDegree, latData.latDirection);
    const double longitude = applyDirection(lonData.lonitudeDegree, lonData.lonDirection);

    // Consider fix valid only when numeric values are finite and satellites exist.
    const bool validNumbers = std::isfinite(latitude) && std::isfinite(longitude) && std::isfinite(altitudeM);
    const bool fix = validNumbers && satellites > 0;

    char utcDateBuf[16] = {0};
    char utcTimeBuf[16] = {0};
    char rtcTimeBuf[32] = {0};

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

    // Update shared runtime model consumed by display, telemetry, and handlers.
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

    // Publish granular MQTT topics to keep subscribers simple and decoupled.
    sf_mqtt::publish("smartfranklin/gps/fix", fix ? "1" : "0");
    sf_mqtt::publish("smartfranklin/gps/lat", latBuf);
    sf_mqtt::publish("smartfranklin/gps/lon", lonBuf);
    sf_mqtt::publish("smartfranklin/gps/alt_m", altBuf);
    sf_mqtt::publish("smartfranklin/gps/satellites", satBuf);
    sf_mqtt::publish("smartfranklin/gps/utc/date", utcDateBuf);
    sf_mqtt::publish("smartfranklin/gps/utc/time", utcTimeBuf);
    sf_mqtt::publish("smartfranklin/gps/rtc/time", rtcTimeBuf);

    m_lastReadOk = validNumbers;
    m_lastReadMs = millis();
}

/**
 * @brief Public periodic entrypoint used by the GPS task loop.
 *
 * No-op until init() succeeds. After initialization, each call performs one
 * full read/publish cycle.
 */
void GPS::process()
{
    // Task calls process() periodically; skip work until init() succeeds.
    if (!m_initialized) {
        return;
    }

    readAndPublish();
}

/**
 * @brief Returns module initialization state.
 */
bool GPS::isInitialized() const
{
    return m_initialized;
}

/**
 * @brief Returns module health based on read success and freshness.
 *
 * Health is true only when:
 * - module initialization succeeded,
 * - the latest read cycle was valid,
 * - and that read is still within GPS_HEALTH_TIMEOUT_MS.
 */
bool GPS::isHealthy() const
{
    if (!m_initialized || !m_lastReadOk) {
        return false;
    }

    // Read is considered stale once timeout window is exceeded.
    return (millis() - m_lastReadMs) <= GPS_HEALTH_TIMEOUT_MS;
}
