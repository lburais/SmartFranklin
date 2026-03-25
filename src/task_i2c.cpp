/**
 * @file task_i2c.cpp
 * @brief Unified I2C task orchestrating Gaz, Tank, Level, RTC, and GPS modules.
 *
 * Detects reachable I2C routes for each supported peripheral, applies PAHub
 * channel selection when needed, initializes module backends with retries,
 * and executes periodic processing loops for all enabled sensors.
 *
 * SPDX-License-Identifier: MIT
 */

#include <Arduino.h>
#include <M5Unified.h>

#include "config_store.h"
#include "gps.h"
#include "i2c.h"
#include "gaz.h"
#include "level.h"
#include "rtc.h"
#include "tank.h"

namespace {

static constexpr uint32_t I2C_SENSORS_INIT_RETRY_MS = 10000UL;

/** Map detected I2C level sensor type to Level module source enum. */
Level::Source levelSourceFromLevelType(const sf_i2c::LevelType levelType)
{
	switch (levelType) {
	case sf_i2c::LevelType::InternalM5:
		return Level::Source::InternalM5;
	case sf_i2c::LevelType::ExternalMpuUnit:
		return Level::Source::ExternalMpuUnit;
	case sf_i2c::LevelType::ExternalAdxl345:
		return Level::Source::ExternalAdxl345;
	case sf_i2c::LevelType::None:
	default:
		return Level::Source::None;
	}
}

/** Select PAHub channel when a device route requires it. */
bool selectPaHubIfNeeded(const sf_i2c::I2C& i2c, const sf_i2c::Device& device, const char* label)
{
	if (!sf_i2c::isPaHubRoute(device.route.mode)) {
		return true;
	}

	if (device.route.paHubChannel < 0) {
		M5_LOGW("[%s] invalid PAHub channel %d", label, device.route.paHubChannel);
		return false;
	}

	if (!i2c.selectPaHubChannel(device.route.mode, static_cast<uint8_t>(device.route.paHubChannel))) {
		M5_LOGW("[%s] failed to select PAHub channel %d", label, device.route.paHubChannel);
		return false;
	}

	return true;
}

/** Disable PAHub channel selection after routed transaction. */
void disablePaHubIfNeeded(const sf_i2c::I2C& i2c, const sf_i2c::Device& device)
{
	if (sf_i2c::isPaHubRoute(device.route.mode)) {
		i2c.disablePaHubChannels(device.route.mode);
	}
}

}  // namespace

/**
 * @brief FreeRTOS entrypoint for unified I2C sensor orchestration.
 * @param pv Unused task parameter.
 */
void taskI2c(void *pv)
{
	(void)pv;
	M5_LOGI("[I2C_SENSORS] Task started");

	bool gazInitialized = false;
	bool tankInitialized = false;
	bool levelInitialized = false;
	bool rtcInitialized = false;
	bool gpsInitialized = false;
	sf_i2c::I2C i2c{};

	sf_i2c::Device gazDevice{};
	gazDevice.route = sf_i2c::Route{};
	gazDevice.address = 0x26;
	gazDevice.tag = "gaz";
	gazDevice.deviceName = "M5Stack Weight I2C Unit";

	sf_i2c::Device tankDevice{};
	tankDevice.route = sf_i2c::Route{};
	tankDevice.address = 0x57;
	tankDevice.tag = "tank";
	tankDevice.deviceName = "M5Stack Unit Ultrasonic I2C (RCWL-9600)";

	sf_i2c::Device internalLevelDevice{};
	internalLevelDevice.route = sf_i2c::Route{};
	internalLevelDevice.route.mode = sf_i2c::RouteMode::Internal;
	internalLevelDevice.route.paHubChannel = -1;
	internalLevelDevice.address = 0x00;
	internalLevelDevice.tag = "level";
	internalLevelDevice.deviceName = "M5 internal Level sensor";
	internalLevelDevice.levelType = sf_i2c::LevelType::InternalM5;

	sf_i2c::Device levelMpuDevice{};
	levelMpuDevice.route = sf_i2c::Route{};
	levelMpuDevice.address = 0x68;
	levelMpuDevice.tag = "level";
	levelMpuDevice.deviceName = "M5Stack External Level Unit (MPU-compatible)";
	levelMpuDevice.levelType = sf_i2c::LevelType::ExternalMpuUnit;

	sf_i2c::Device levelAdxlDevice{};
	levelAdxlDevice.route = sf_i2c::Route{};
	levelAdxlDevice.address = 0x53;
	levelAdxlDevice.tag = "level";
	levelAdxlDevice.deviceName = "SeeedStudio ADXL345 Level Sensor";
	levelAdxlDevice.levelType = sf_i2c::LevelType::ExternalAdxl345;

	sf_i2c::Device activeLevelDevice{};
	activeLevelDevice.route = sf_i2c::Route{};
	activeLevelDevice.address = 0x00;
	activeLevelDevice.tag = "level";
	activeLevelDevice.deviceName = "Level";
	activeLevelDevice.levelType = sf_i2c::LevelType::None;

	sf_i2c::Device internalRtcDevice{};
	internalRtcDevice.route = sf_i2c::Route{};
	internalRtcDevice.route.mode = sf_i2c::RouteMode::Internal;
	internalRtcDevice.route.paHubChannel = -1;
	internalRtcDevice.address = 0x51;
	internalRtcDevice.tag = "rtc";
	internalRtcDevice.deviceName = "M5 internal RTC";

	sf_i2c::Device externalRtcDevice{};
	externalRtcDevice.route = sf_i2c::Route{};
	externalRtcDevice.address = 0x51;
	externalRtcDevice.tag = "rtc";
	externalRtcDevice.deviceName = "M5Stack RTC Unit or Seeed PCD85063TP";

	sf_i2c::Device activeRtcDevice{};
	activeRtcDevice.route = sf_i2c::Route{};
	activeRtcDevice.address = 0x51;
	activeRtcDevice.tag = "rtc";
	activeRtcDevice.deviceName = "RTC";

	sf_i2c::Device gpsDevice{};
	gpsDevice.route = sf_i2c::Route{};
	gpsDevice.address = 0x66;
	gpsDevice.tag = "gps";
	gpsDevice.deviceName = "DFRobot Gravity GNSS (DFR1103)";

	uint32_t nextTankInitAttemptMs = 0;
	uint32_t nextGazInitAttemptMs = 0;
	uint32_t nextLevelInitAttemptMs = 0;
	uint32_t nextRtcInitAttemptMs = 0;
	uint32_t nextGpsInitAttemptMs = 0;

	auto isRetryDue = [](const uint32_t nowMs, const uint32_t nextAttemptMs) {
		return static_cast<int32_t>(nowMs - nextAttemptMs) >= 0;
	};

	auto scheduleRetry = [](uint32_t& nextAttemptMs, const uint32_t nowMs) {
		nextAttemptMs = nowMs + I2C_SENSORS_INIT_RETRY_MS;
	};

	i2c.beginPortA();

	for (;;) {
		const uint32_t nowMs = millis();

#ifndef DISABLE_TANK
		if (!tankInitialized && isRetryDue(nowMs, nextTankInitAttemptMs)) {
			if (!i2c.detectRoute(tankDevice.address, tankDevice.route)) {
				M5_LOGW("[I2C_SENSORS] TANK route detection failed");
				i2c.publishConfiguration(tankDevice);
				tankInitialized = false;
				scheduleRetry(nextTankInitAttemptMs, nowMs);
			} else {
				i2c.publishConfiguration(tankDevice);
				bool channelSelected = selectPaHubIfNeeded(i2c, tankDevice, "I2C_SENSORS");
				if (channelSelected) {
					tankInitialized = TANK_MODULE.init(sf_i2c::isInternalRoute(tankDevice.route.mode), tankDevice.address);
					disablePaHubIfNeeded(i2c, tankDevice);
					if (!tankInitialized) {
						scheduleRetry(nextTankInitAttemptMs, nowMs);
					}
				} else {
					tankInitialized = false;
					scheduleRetry(nextTankInitAttemptMs, nowMs);
				}
			}

			if (!tankInitialized) {
				M5_LOGW("[I2C_SENSORS] TANK init failed");
			}
		}
#else
		tankInitialized = true;
#endif

#ifndef DISABLE_GAZ
		if (!gazInitialized && isRetryDue(nowMs, nextGazInitAttemptMs)) {
			if (!i2c.detectRoute(gazDevice.address, gazDevice.route)) {
				M5_LOGW("[I2C_SENSORS] GAZ route detection failed");
				i2c.publishConfiguration(gazDevice);
				gazInitialized = false;
				scheduleRetry(nextGazInitAttemptMs, nowMs);
			} else {
				i2c.publishConfiguration(gazDevice);
				bool channelSelected = selectPaHubIfNeeded(i2c, gazDevice, "I2C_SENSORS");
				if (channelSelected) {
					gazInitialized = GAZ_MODULE.init(sf_i2c::isInternalRoute(gazDevice.route.mode), gazDevice.address);
					disablePaHubIfNeeded(i2c, gazDevice);
					if (!gazInitialized) {
						scheduleRetry(nextGazInitAttemptMs, nowMs);
					}
				} else {
					gazInitialized = false;
					scheduleRetry(nextGazInitAttemptMs, nowMs);
				}
			}

			if (!gazInitialized) {
				M5_LOGW("[I2C_SENSORS] GAZ init failed");
			}
		}
#else
		gazInitialized = true;
#endif

#ifndef DISABLE_LEVEL
		if (!levelInitialized && isRetryDue(nowMs, nextLevelInitAttemptMs)) {

			if (LEVEL_MODULE.init(levelSourceFromLevelType(internalLevelDevice.levelType), true, internalLevelDevice.address)) {
				levelInitialized = true;
				activeLevelDevice = internalLevelDevice;
			} else {
				if (i2c.detectRoute(levelMpuDevice.address, levelMpuDevice.route)) {
					bool channelSelected = selectPaHubIfNeeded(i2c, levelMpuDevice, "I2C_SENSORS");
					if (channelSelected) {
						levelInitialized = LEVEL_MODULE.init(levelSourceFromLevelType(levelMpuDevice.levelType),
															 sf_i2c::isInternalRoute(levelMpuDevice.route.mode),
															 levelMpuDevice.address);
						disablePaHubIfNeeded(i2c, levelMpuDevice);
						if (levelInitialized) {
							activeLevelDevice = levelMpuDevice;
						}
					}
				}

				if (!levelInitialized && i2c.detectRoute(levelAdxlDevice.address, levelAdxlDevice.route)) {
					bool channelSelected = selectPaHubIfNeeded(i2c, levelAdxlDevice, "I2C_SENSORS");
					if (channelSelected) {
						levelInitialized = LEVEL_MODULE.init(levelSourceFromLevelType(levelAdxlDevice.levelType),
															 sf_i2c::isInternalRoute(levelAdxlDevice.route.mode),
															 levelAdxlDevice.address);
						disablePaHubIfNeeded(i2c, levelAdxlDevice);
						if (levelInitialized) {
							activeLevelDevice = levelAdxlDevice;
						}
					}
				}
			}

			if (!levelInitialized) {
				M5_LOGW("[I2C_SENSORS] LEVEL init failed");
				scheduleRetry(nextLevelInitAttemptMs, nowMs);
			} else {
				i2c.publishConfiguration(activeLevelDevice);

			}
		}
#else
		levelInitialized = true;
#endif

#ifndef DISABLE_RTC
		if (!rtcInitialized && isRetryDue(nowMs, nextRtcInitAttemptMs)) {
			if (RTC_MODULE.init(RTC::Source::InternalRtc, true, internalRtcDevice.address)) {
				rtcInitialized = true;
				activeRtcDevice = internalRtcDevice;
			} else if (i2c.detectRoute(externalRtcDevice.address, externalRtcDevice.route)) {
				bool channelSelected = selectPaHubIfNeeded(i2c, externalRtcDevice, "I2C_SENSORS");
				if (channelSelected) {
					const bool isInternalRoute = sf_i2c::isInternalRoute(externalRtcDevice.route.mode);
					rtcInitialized = RTC_MODULE.init(RTC::Source::ExternalM5StackRtcUnit,
													 isInternalRoute,
													 externalRtcDevice.address);
					if (!rtcInitialized) {
						rtcInitialized = RTC_MODULE.init(RTC::Source::ExternalSeeedPcd85063tp,
														 isInternalRoute,
														 externalRtcDevice.address);
					}
					disablePaHubIfNeeded(i2c, externalRtcDevice);
					if (rtcInitialized) {
						activeRtcDevice = externalRtcDevice;
					}
				}
			}

			if (!rtcInitialized) {
				M5_LOGW("[I2C_SENSORS] RTC init failed");
				scheduleRetry(nextRtcInitAttemptMs, nowMs);
			} else {
				i2c.publishConfiguration(activeRtcDevice);
			}
		}
#else
		rtcInitialized = true;
#endif

#ifndef DISABLE_GPS
		if (!gpsInitialized && isRetryDue(nowMs, nextGpsInitAttemptMs)) {
			if (!i2c.detectRoute(gpsDevice.address, gpsDevice.route)) {
				M5_LOGW("[I2C_SENSORS] GPS route detection failed");
				i2c.publishConfiguration(gpsDevice);
				gpsInitialized = false;
				scheduleRetry(nextGpsInitAttemptMs, nowMs);
			} else {
				i2c.publishConfiguration(gpsDevice);
				bool channelSelected = selectPaHubIfNeeded(i2c, gpsDevice, "I2C_SENSORS");
				if (channelSelected) {
					gpsInitialized = GPS_MODULE.init(GPS::Source::ExternalDfrobotGravity,
					                                 sf_i2c::isInternalRoute(gpsDevice.route.mode),
					                                 gpsDevice.address);
					disablePaHubIfNeeded(i2c, gpsDevice);
					if (!gpsInitialized) {
						scheduleRetry(nextGpsInitAttemptMs, nowMs);
					}
				} else {
					gpsInitialized = false;
					scheduleRetry(nextGpsInitAttemptMs, nowMs);
				}
			}

			if (!gpsInitialized) {
				M5_LOGW("[I2C_SENSORS] GPS init failed");
			}
		}
#else
		gpsInitialized = true;
#endif

#ifndef DISABLE_GAZ
		if (gazInitialized && selectPaHubIfNeeded(i2c, gazDevice, "I2C_SENSORS")) {
			GAZ_MODULE.process();
			disablePaHubIfNeeded(i2c, gazDevice);
		}
#endif
#ifndef DISABLE_TANK
		if (tankInitialized && selectPaHubIfNeeded(i2c, tankDevice, "I2C_SENSORS")) {
			TANK_MODULE.process();
			disablePaHubIfNeeded(i2c, tankDevice);
		}
#endif

#ifndef DISABLE_LEVEL
		if (levelInitialized && activeLevelDevice.levelType == sf_i2c::LevelType::InternalM5) {
			LEVEL_MODULE.process();
		} else if (levelInitialized && selectPaHubIfNeeded(i2c, activeLevelDevice, "I2C_SENSORS")) {
			LEVEL_MODULE.process();
			disablePaHubIfNeeded(i2c, activeLevelDevice);
		}
#endif

#ifndef DISABLE_RTC
		if (rtcInitialized && activeRtcDevice.route.mode == sf_i2c::RouteMode::Internal) {
			RTC_MODULE.process();
		} else if (rtcInitialized && selectPaHubIfNeeded(i2c, activeRtcDevice, "I2C_SENSORS")) {
			RTC_MODULE.process();
			disablePaHubIfNeeded(i2c, activeRtcDevice);
		}
#endif

#ifndef DISABLE_GPS
		if (gpsInitialized && selectPaHubIfNeeded(i2c, gpsDevice, "I2C_SENSORS")) {
			GPS_MODULE.process();
			disablePaHubIfNeeded(i2c, gpsDevice);
		}
#endif

		const int i2cLoopMs = (CONFIG.task_i2c_loop_ms > 0) ? CONFIG.task_i2c_loop_ms : 1000;
		vTaskDelay(pdMS_TO_TICKS(i2cLoopMs));
	}
}
