/**
 * @file i2c.h
 * @brief I2C route discovery and direct bus binding helpers.
 *
 * SmartFranklin uses this module to probe internal and Wire buses and
 * publish resulting route metadata.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <Arduino.h>

namespace sf_i2c {

/**
 * @brief Physical/logical route used to reach a peripheral.
 */
enum class RouteMode : uint8_t {
    /** Route not resolved yet. */
    Unset = 0,
    /** Internal M5 I2C bus. */
    Internal,
    /** Primary Wire bus. */
    Wire,
};

/**
 * @brief Physical external connector used when route mode is Wire.
 */
enum class ExternalPort : uint8_t {
    PortA1 = 0,
    PortA2,
    PortB1,
    PortB2,
    PortC1,
    PortC2,
};

/**
 * @brief Supported level sensor categories.
 */
enum class LevelType : uint8_t {
    /** No level sensor type declared. */
    None = 0,
    /** Internal M5 IMU type. */
    InternalM5,
};

/**
 * @brief Resolved bus route information for one device.
 */
struct Route {
    /** Selected route mode. */
    RouteMode mode = RouteMode::Unset;
    /** Selected external connector for Wire mode. */
    ExternalPort externalPort = ExternalPort::PortA1;
};

/**
 * @brief Device descriptor used during route discovery and publication.
 */
struct Device {
    /**
     * @brief Generic chip family classification.
     */
    enum class ChipKind : uint8_t {
        /** Unknown or not yet identified. */
        Unknown = 0,
        /** BM8563-like RTC family. */
        Bm8563Like,
        /** PCD85063-like RTC family. */
        Pcd85063Like,
    };

    /** Route used to reach this device. */
    Route route;
    /** I2C address of the peripheral. */
    uint8_t address;
    /** Short stable tag used in telemetry/config topics. */
    const char* tag;
    /** Human-readable device name for logs/UI. */
    const char* deviceName;
    /** Optional level sensor subtype metadata. */
    LevelType levelType = LevelType::None;
    /** Optional chip family metadata for RTC-compatible devices. */
    ChipKind chipKind = ChipKind::Unknown;
};

/**
 * @brief Route detection helper.
 */
class I2C {

    public:
    /**
     * @brief Construct helper with desired bus clock.
     * @param clockHz Target I2C clock frequency in Hz.
     */
    explicit I2C(uint32_t clockHz = 100000U);

    /**
     * @brief Initialize Port A / Wire bus according to runtime settings.
     */
    void beginPortA() const;

    /**
     * @brief Initialize Wire bus on a specific external connector.
     * @param port External connector to use.
     */
    void beginExternalPort(ExternalPort port) const;

    /**
     * @brief Initialize bus for a resolved route.
     * @param route Route to activate.
     */
    void beginRoute(const Route& route) const;

    /**
     * @brief Detect a reachable route for a given device address.
     * @param deviceAddress Target I2C address.
     * @param route Output resolved route data.
     * @return True if the device is detected on any supported route.
     */
    bool detectRoute(uint8_t deviceAddress, Route& route) const;

    /**
     * @brief Check if a device is reachable on a specific configured route.
     * @param deviceAddress Target I2C address.
     * @param mode Configured route mode.
     * @return True when address is reachable on the selected route.
     */
    bool deviceExistsOnRoute(uint8_t deviceAddress, const Route& route) const;

    /**
     * @brief Publish detected configuration metadata for one device.
     * @param device Device descriptor to publish.
     */
    void publishConfiguration(const Device& device) const;

private:
    /** @brief Check address presence on Wire bus. */
    bool wireDeviceExists(uint8_t address) const;
    /** @brief Check address presence on internal Ex bus. */
    bool exDeviceExists(uint8_t address) const;

    /** Configured I2C clock frequency in Hz. */
    uint32_t m_clockHz;
};

constexpr uint32_t kInitRetryMs = 10000UL;
bool resolveRouteFromConfiguredPort(const String& configuredPort, Route& route, const char* label);

/**
 * @brief Check whether route uses internal M5 path.
 * @param mode Route mode to inspect.
 * @return True for internal routes.
 */
bool isInternalRoute(RouteMode mode);

/** @brief Convert route mode to stable text value. */
const char* routeModeToString(RouteMode mode);
const char* externalPortToString(ExternalPort port);
/** @brief Convert level type to stable text value. */
const char* levelTypeToString(LevelType type);
/** @brief Convert chip kind to stable text value. */
const char* chipKindToString(Device::ChipKind kind);

}  // namespace sf_i2c
