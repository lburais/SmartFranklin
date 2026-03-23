/*
 * SmartFranklin - unified I2C sensors task
 * SPDX-License-Identifier: MIT
 */

#include <Arduino.h>
#include <M5Unified.h>

#include "config_store.h"
#include "i2c.h"
#include "gaz.h"
#include "level.h"
#include "rtc.h"
#include "tank.h"

namespace {

static constexpr uint32_t I2C_SENSORS_INIT_RETRY_MS = 10000UL;
static constexpr uint32_t I2C_SENSORS_LOOP_MS = 1000UL;

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

void disablePaHubIfNeeded(const sf_i2c::I2C& i2c, const sf_i2c::Device& device)
{
	if (sf_i2c::isPaHubRoute(device.route.mode)) {
		i2c.disablePaHubChannels(device.route.mode);
	}
}

}  // namespace

void taskI2c(void *pv)
{
	(void)pv;
	M5_LOGI("[I2C_SENSORS] Task started");

	bool gazInitialized = false;
	bool tankInitialized = false;
	bool levelInitialized = false;
	bool rtcInitialized = false;
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

	i2c.beginPortA();

	for (;;) {

#ifndef DISABLE_TANK
		if (!tankInitialized) {
			if (!i2c.detectRoute(tankDevice.address, tankDevice.route)) {
				M5_LOGW("[I2C_SENSORS] TANK route detection failed");
				i2c.publishConfiguration(tankDevice);
				tankInitialized = false;
			} else {
				i2c.publishConfiguration(tankDevice);
				bool channelSelected = selectPaHubIfNeeded(i2c, tankDevice, "I2C_SENSORS");
				if (channelSelected) {
					tankInitialized = TANK_MODULE.init(sf_i2c::isInternalRoute(tankDevice.route.mode), tankDevice.address);
					disablePaHubIfNeeded(i2c, tankDevice);
				} else {
					tankInitialized = false;
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
		if (!gazInitialized) {
			if (!i2c.detectRoute(gazDevice.address, gazDevice.route)) {
				M5_LOGW("[I2C_SENSORS] GAZ route detection failed");
				i2c.publishConfiguration(gazDevice);
				gazInitialized = false;
			} else {
				i2c.publishConfiguration(gazDevice);
				bool channelSelected = selectPaHubIfNeeded(i2c, gazDevice, "I2C_SENSORS");
				if (channelSelected) {
					gazInitialized = GAZ_MODULE.init(sf_i2c::isInternalRoute(gazDevice.route.mode), gazDevice.address);
					disablePaHubIfNeeded(i2c, gazDevice);
				} else {
					gazInitialized = false;
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
		if (!levelInitialized) {

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
			} else {
				i2c.publishConfiguration(activeLevelDevice);

			}
		}
#else
		levelInitialized = true;
#endif

#ifndef DISABLE_RTC
		if (!rtcInitialized) {
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
			} else {
				i2c.publishConfiguration(activeRtcDevice);
			}
		}
#else
		rtcInitialized = true;
#endif

		if (gazInitialized && tankInitialized && levelInitialized && rtcInitialized) {
			break;
		}

		M5_LOGW("[I2C_SENSORS] init incomplete, retry in 10s");
		vTaskDelay(pdMS_TO_TICKS(I2C_SENSORS_INIT_RETRY_MS));
	}

	for (;;) {
#ifndef DISABLE_GAZ
		if (selectPaHubIfNeeded(i2c, gazDevice, "I2C_SENSORS")) {
			GAZ_MODULE.process();
			disablePaHubIfNeeded(i2c, gazDevice);
		}
#endif
#ifndef DISABLE_TANK
		if (selectPaHubIfNeeded(i2c, tankDevice, "I2C_SENSORS")) {
			TANK_MODULE.process();
			disablePaHubIfNeeded(i2c, tankDevice);
		}
#endif

#ifndef DISABLE_LEVEL
		if (activeLevelDevice.levelType == sf_i2c::LevelType::InternalM5) {
			LEVEL_MODULE.process();
		} else if (selectPaHubIfNeeded(i2c, activeLevelDevice, "I2C_SENSORS")) {
			LEVEL_MODULE.process();
			disablePaHubIfNeeded(i2c, activeLevelDevice);
		}
#endif

#ifndef DISABLE_RTC
		if (activeRtcDevice.route.mode == sf_i2c::RouteMode::Internal) {
			RTC_MODULE.process();
		} else if (selectPaHubIfNeeded(i2c, activeRtcDevice, "I2C_SENSORS")) {
			RTC_MODULE.process();
			disablePaHubIfNeeded(i2c, activeRtcDevice);
		}
#endif

		vTaskDelay(pdMS_TO_TICKS(I2C_SENSORS_LOOP_MS));
	}
}
