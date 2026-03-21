/*
 * SmartFranklin - unified I2C sensors task
 * SPDX-License-Identifier: MIT
 */

#include <Arduino.h>
#include <M5Unified.h>

#include "config_store.h"
#include "i2c.h"
#include "gaz.h"
#include "imu.h"
#include "tank.h"

namespace {

static constexpr uint32_t I2C_SENSORS_INIT_RETRY_MS = 10000UL;
static constexpr uint32_t I2C_SENSORS_LOOP_MS = 1000UL;

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

void taskI2cSensors(void *pv)
{
    (void)pv;
    M5_LOGI("[I2C_SENSORS] Task started");

    bool gazInitialized = false;
    bool tankInitialized = false;
    bool imuInitialized = false;
    sf_i2c::I2C i2c{};

    sf_i2c::Device gazDevice = { .route = sf_i2c::Route{},
                                 .address = 0x26,
                                 .tag = "gaz",
                                 .deviceName = "M5Stack Weight I2C Unit" };

    sf_i2c::Device tankDevice = { .route = sf_i2c::Route{},
                                  .address = 0x57,
                                  .tag = "tank",
                                  .deviceName = "M5Stack Unit Ultrasonic I2C (RCWL-9600)" };

    sf_i2c::Device imuMpuDevice = { .route = sf_i2c::Route{},
                                    .address = 0x68,
                                    .tag = "imu_mpu",
                                    .deviceName = "M5Stack External IMU Unit (MPU-compatible)" };

    sf_i2c::Device imuAdxlDevice = { .route = sf_i2c::Route{},
                                     .address = 0x53,
                                     .tag = "imu_adxl345",
                                     .deviceName = "SeeedStudio ADXL345 Accelerometer" };

    sf_i2c::Device activeImuDevice = { .route = sf_i2c::Route{},
                                       .address = 0x00,
                                       .tag = "imu",
                                       .deviceName = "IMU" };

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

#ifndef DISABLE_IMU
        if (!imuInitialized) {
            if (IMU_MODULE.init(Imu::Source::InternalM5, true, 0x00)) {
                imuInitialized = true;
                activeImuDevice = { .route = sf_i2c::Route{},
                                    .address = 0x00,
                                    .tag = "imu_internal",
                                    .deviceName = "M5 internal IMU" };
            } else {
                if (i2c.detectRoute(imuMpuDevice.address, imuMpuDevice.route)) {
                    i2c.publishConfiguration(imuMpuDevice);
                    bool channelSelected = selectPaHubIfNeeded(i2c, imuMpuDevice, "I2C_SENSORS");
                    if (channelSelected) {
                        imuInitialized = IMU_MODULE.init(Imu::Source::ExternalMpuUnit,
                                                         sf_i2c::isInternalRoute(imuMpuDevice.route.mode),
                                                         imuMpuDevice.address);
                        disablePaHubIfNeeded(i2c, imuMpuDevice);
                        if (imuInitialized) {
                            activeImuDevice = imuMpuDevice;
                        }
                    }
                }

                if (!imuInitialized && i2c.detectRoute(imuAdxlDevice.address, imuAdxlDevice.route)) {
                    i2c.publishConfiguration(imuAdxlDevice);
                    bool channelSelected = selectPaHubIfNeeded(i2c, imuAdxlDevice, "I2C_SENSORS");
                    if (channelSelected) {
                        imuInitialized = IMU_MODULE.init(Imu::Source::ExternalAdxl345,
                                                         sf_i2c::isInternalRoute(imuAdxlDevice.route.mode),
                                                         imuAdxlDevice.address);
                        disablePaHubIfNeeded(i2c, imuAdxlDevice);
                        if (imuInitialized) {
                            activeImuDevice = imuAdxlDevice;
                        }
                    }
                }
            }

            if (!imuInitialized) {
                M5_LOGW("[I2C_SENSORS] IMU init failed");
            }
        }
#else
        imuInitialized = true;
#endif

        if (gazInitialized && tankInitialized && imuInitialized) {
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

#ifndef DISABLE_IMU
        if (IMU_MODULE.source() == Imu::Source::InternalM5) {
            IMU_MODULE.process();
        } else if (selectPaHubIfNeeded(i2c, activeImuDevice, "I2C_SENSORS")) {
            IMU_MODULE.process();
            disablePaHubIfNeeded(i2c, activeImuDevice);
        }
#endif

        vTaskDelay(pdMS_TO_TICKS(I2C_SENSORS_LOOP_MS));
    }
}
