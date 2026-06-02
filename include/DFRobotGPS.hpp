/*
 * SPDX-FileCopyrightText: 2026 Laurent Burais
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_DFRobotGPS.hpp
  @brief DFRobot Gravity GNSS (TEL0157, I2C) Unit for M5UnitUnified

  Wraps the DFRobot Gravity GPS module behind the standard
  m5::unit::Component + PeriodicMeasurementAdapter interface so it can be
  used identically to any first-party M5 Unit:

      m5::unit::UnitUnified   m_units;
      m5::unit::UnitDFRobotGPS m_gps;

      m_units.add(m_gps, *wire);
      m_units.begin();
      // --- in loop ---
      m_units.update();
      if (m_gps.updated()) {
          double lat = m_gps.latitude();
          double lon = m_gps.longitude();
      }

  Register map: DFRobot TEL0157 firmware ≥ 0x01 (I2C default addr 0x20).
*/

#pragma once

#include <M5UnitComponent.hpp>
#include <m5_utility/container/circular_buffer.hpp>
#include <array>
#include <cstring>
#include <memory>

namespace m5 {
namespace unit {

// ---------------------------------------------------------------------------
// Namespace for TEL0157 register map and measurement data
// ---------------------------------------------------------------------------
namespace dfrobotgps {

// I2C register addresses (TEL0157 datasheet)
enum Reg : uint8_t {
    REG_I2C_ADDR     = 0x00,  // R/W  change I2C address
    REG_SOFT_VERSION = 0x01,  // R    firmware version (nonzero when ready)
    REG_STATUS       = 0x02,  // R    0 = no fix, 1 = GNSS fix acquired
    REG_LATITUDE     = 0x03,  // R    ddmm.mmmmm string, 14 bytes
    REG_LATITUDE_DIR = 0x04,  // part of latitude block (N/S)
    REG_LONGITUDE    = 0x05,  // R    dddmm.mmmmm string, 15 bytes
    REG_LONGITUDE_DIR= 0x06,
    REG_UTC_TIME     = 0x07,  // R    hhmmss.ss  string, 9 bytes
    REG_SPEED        = 0x09,  // R    knots string, 5 bytes
    REG_SATELLITES   = 0x0B,  // R    uint8  number in view
    REG_ALTITUDE     = 0x0C,  // R    metres string, 9 bytes  (ASL)
    REG_HEADING      = 0x0E,  // R    degrees string, 7 bytes (true course)
    REG_HDOP         = 0x10,  // R    hdop string, 5 bytes
    REG_PDOP         = 0x12,  // R    pdop string, 5 bytes
    REG_VDOP         = 0x14,  // R    vdop string, 5 bytes
    REG_GNSS_MODE    = 0xA9,  // R/W  GNSS constellation mask
    REG_SLEEP_MODE   = 0xAA,  // W    0x00 = wake, 0x01 = standby, 0x02 = dormant
    REG_RGB_MODE     = 0xAB,  // W    0x00 = off, 0x01 = on (status LED)
};

/*!
  @struct Data
  @brief One complete GNSS measurement snapshot
*/
struct Data {
    bool    fix       = false;   ///< true when a GNSS fix is available
    double  latitude  = 0.0;     ///< decimal degrees, WGS-84 (+N/-S)
    double  longitude = 0.0;     ///< decimal degrees, WGS-84 (+E/-W)
    float   altitude  = 0.0f;   ///< metres above sea level
    float   speed_kmh = 0.0f;   ///< ground speed km/h
    float   heading   = 0.0f;   ///< true course degrees [0, 360)
    float   hdop      = 99.0f;  ///< horizontal dilution of precision
    uint8_t satellites = 0;     ///< number of satellites used
};

}  // namespace dfrobotgps

// ---------------------------------------------------------------------------

/*!
  @class UnitDFRobotGPS
  @brief DFRobot Gravity GNSS (TEL0157) driver as an M5UnitUnified Component.

  Inherits:
    - m5::unit::Component   – adapter/manager integration, readRegister* helpers
    - PeriodicMeasurementAdapter – updated(), oldest(), latest(), available()…
*/
class UnitDFRobotGPS
    : public Component
    , public PeriodicMeasurementAdapter<UnitDFRobotGPS, dfrobotgps::Data>
{
    // Injects DEFAULT_ADDRESS, uid, attr, name, and copy-delete boilerplate.
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitDFRobotGPS, 0x20);
    // Injects oldest_periodic_data(), latest_periodic_data(), and the five
    // pure-virtual PeriodicMeasurementAdapter implementations.
    M5_UNIT_COMPONENT_PERIODIC_MEASUREMENT_ADAPTER_HPP_BUILDER(
        UnitDFRobotGPS, dfrobotgps::Data);

public:
    /*!
      @struct config_t
      @brief Runtime configuration applied in begin()
    */
    struct config_t {
        //! Start periodic measurement immediately in begin()?
        bool start_periodic = true;
        //! Measurement interval in ms (minimum ~200 ms for TEL0157)
        uint32_t interval = 1000U;
        //! Enable the RGB status LED on the module
        bool rgb_led = false;
    };

    explicit UnitDFRobotGPS(const uint8_t addr = DEFAULT_ADDRESS)
        : Component(addr)
        , _data{new m5::container::CircularBuffer<dfrobotgps::Data>(1)}
    {
        auto ccfg  = component_config();
        ccfg.clock = 100 * 1000U;   // 100 kHz – safe for long Grove cables
        component_config(ccfg);
    }

    virtual ~UnitDFRobotGPS() = default;

    // -----------------------------------------------------------------------
    // Configuration
    // -----------------------------------------------------------------------

    inline config_t config() const         { return _cfg; }
    inline void     config(const config_t& c) { _cfg = c; }

    // -----------------------------------------------------------------------
    // M5UnitUnified lifecycle (override Component virtuals)
    // -----------------------------------------------------------------------

    /*!
      @brief Initialise the unit.
      Polls the firmware-version register until the module is ready (up to
      2 s), configures the RGB LED, and optionally starts periodic measurement.
      @return True if the module responded and setup succeeded.
    */
    virtual bool begin() override;

    /*!
      @brief Drive the periodic measurement engine.
      Called by UnitUnified::update().  Reads all position/kinematic registers
      when the configured interval has elapsed and sets _updated accordingly.
      @param force Read immediately regardless of the interval timer.
    */
    virtual void update(const bool force = false) override;

    // -----------------------------------------------------------------------
    // Periodic measurement control
    // -----------------------------------------------------------------------

    /*!
      @brief Start the periodic measurement engine.
      @param interval Sampling interval in ms (minimum ~200 ms).
      @return True if successful (fails if already running).
    */
    inline bool startPeriodicMeasurement(const uint32_t interval = 1000U)
    {
        return PeriodicMeasurementAdapter<UnitDFRobotGPS, dfrobotgps::Data>
            ::startPeriodicMeasurement(interval);
    }

    /*!
      @brief Stop the periodic measurement engine.
      @return True if successful.
    */
    inline bool stopPeriodicMeasurement()
    {
        return PeriodicMeasurementAdapter<UnitDFRobotGPS, dfrobotgps::Data>
            ::stopPeriodicMeasurement();
    }

    // -----------------------------------------------------------------------
    // Measurement accessors (mirror UnitWeightI2C naming convention)
    // Read the *latest* value stored in the circular buffer.
    // -----------------------------------------------------------------------

    //! @brief GNSS fix valid?
    inline bool   fixValid()    const { return !empty() ? oldest().fix        : false;  }
    //! @brief Latitude in decimal degrees (+N / -S)
    inline double latitude()    const { return !empty() ? oldest().latitude   : 0.0;   }
    //! @brief Longitude in decimal degrees (+E / -W)
    inline double longitude()   const { return !empty() ? oldest().longitude  : 0.0;   }
    //! @brief Altitude in metres (above sea level)
    inline float  altitude()    const { return !empty() ? oldest().altitude   : 0.0f;  }
    //! @brief Ground speed in km/h
    inline float  speedKmh()    const { return !empty() ? oldest().speed_kmh  : 0.0f;  }
    //! @brief True course in degrees
    inline float  heading()     const { return !empty() ? oldest().heading    : 0.0f;  }
    //! @brief Horizontal dilution of precision
    inline float  hdop()        const { return !empty() ? oldest().hdop       : 99.0f; }
    //! @brief Satellites used for the fix
    inline uint8_t satellites() const { return !empty() ? oldest().satellites : 0;     }

    // -----------------------------------------------------------------------
    // Module control
    // -----------------------------------------------------------------------

    /*!
      @brief Put the module into low-power sleep mode.
      @param dormant  true = dormant (GNSS off), false = standby (lower power)
      @return True if the I2C write succeeded.
    */
    bool sleep(bool dormant = false);

    /*!
      @brief Wake the module from sleep.
      @return True if the I2C write succeeded.
    */
    bool wake();

    /*!
      @brief Enable or disable the on-module RGB status LED.
      @return True if the I2C write succeeded.
    */
    bool setRGBLed(bool enable);

private:
    // Internal helpers called by begin()/update()
    bool     read_gnss(dfrobotgps::Data& out);
    double   parse_nmea_coord(const char* str, char dir);
    float    parse_float_field(const char* str);

    config_t _cfg{};
    bool     _mode_started{};
    std::unique_ptr<m5::container::CircularBuffer<dfrobotgps::Data>> _data;

    // Required by start/stop_periodic_measurement called via CRTP
    friend class PeriodicMeasurementAdapter<UnitDFRobotGPS, dfrobotgps::Data>;
    bool start_periodic_measurement(uint32_t interval);
    bool stop_periodic_measurement();
};

}  // namespace unit
}  // namespace m5
