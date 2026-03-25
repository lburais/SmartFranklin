/**
 * @file i2c.h
 * @brief I2C route discovery and PAHub-aware device binding helpers.
 *
 * SmartFranklin uses this module to probe both internal and Wire buses,
 * optionally through PAHub channels, and publish resulting route metadata.
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
    /** Internal M5 I2C bus routed through PAHub. */
    InternalPaHub,
    /** Primary Wire bus. */
    Wire,
    /** Primary Wire bus routed through PAHub. */
    WirePaHub,
};

/**
 * @brief Supported external level sensor categories.
 */
enum class LevelType : uint8_t {
    /** No level sensor type declared. */
    None = 0,
    /** Internal M5 IMU type. */
    InternalM5,
    /** External MPU-style level source. */
    ExternalMpuUnit,
    /** External ADXL345 level source. */
    ExternalAdxl345,
};

/**
 * @brief Resolved bus route information for one device.
 */
struct Route {
    /** Selected route mode. */
    RouteMode mode = RouteMode::Unset;
    /** PAHub channel index when mode is a PAHub route, otherwise -1. */
    int8_t paHubChannel = -1;
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
 * @brief Route detection and PAHub channel control helper.
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
     * @brief Detect a reachable route for a given device address.
     * @param deviceAddress Target I2C address.
     * @param route Output resolved route data.
     * @return True if the device is detected on any supported route.
     */
    bool detectRoute(uint8_t deviceAddress, Route& route) const;

    /**
     * @brief Select PAHub channel for a specific routed bus.
     * @param mode Route mode that uses PAHub.
     * @param channel PAHub channel index.
     * @return True on successful channel selection.
     */
    bool selectPaHubChannel(RouteMode mode, uint8_t channel) const;

    /**
     * @brief Disable all PAHub channel selections for a routed bus.
     * @param mode Route mode that uses PAHub.
     */
    void disablePaHubChannels(RouteMode mode) const;

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

    /** @brief Select PAHub channel via Wire bus. */
    bool wireSelectPaHubChannel(uint8_t channel) const;
    /** @brief Disable PAHub channel selection on Wire bus. */
    void wireDisablePaHubChannels() const;

    /** @brief Select PAHub channel via internal Ex bus. */
    bool exSelectPaHubChannel(uint8_t channel) const;
    /** @brief Disable PAHub channel selection on internal Ex bus. */
    void exDisablePaHubChannels() const;

    /** Configured I2C clock frequency in Hz. */
    uint32_t m_clockHz;
};

/**
 * @brief Check whether route uses internal M5 path.
 * @param mode Route mode to inspect.
 * @return True for internal routes.
 */
bool isInternalRoute(RouteMode mode);

/**
 * @brief Check whether route is PAHub-based.
 * @param mode Route mode to inspect.
 * @return True for PAHub routes.
 */
bool isPaHubRoute(RouteMode mode);

/** @brief Convert route mode to stable text value. */
const char* routeModeToString(RouteMode mode);
/** @brief Convert level type to stable text value. */
const char* levelTypeToString(LevelType type);
/** @brief Convert chip kind to stable text value. */
const char* chipKindToString(Device::ChipKind kind);

}  // namespace sf_i2c
