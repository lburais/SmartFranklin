#include <Arduino.h>
#include <M5Unified.h>

#include "config_store.h"
#include "i2c.h"
#include "gaz.h"
#include "tank.h"

namespace {

static constexpr uint32_t I2C_SENSORS_INIT_RETRY_MS = 10000UL;
static constexpr uint32_t I2C_SENSORS_LOOP_MS = 1000UL;
static constexpr uint32_t I2C_SENSORS_CLOCK_HZ = 400000U;

static constexpr uint8_t GAZ_I2C_ADDRESS = 0x26;
static constexpr const char* GAZ_DEVICE_FULL_NAME = "M5Stack Weight I2C Unit";

static constexpr uint8_t TANK_I2C_ADDRESS = 0x57;
static constexpr const char* TANK_DEVICE_FULL_NAME = "M5Stack Unit Ultrasonic I2C (RCWL-9600)";

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
    sf_i2c::I2C i2c{};

    sf_i2c::Device gazDevice = { .route = sf_i2c::Route{},
                                 .sda = -1,
                                 .scl = -1,
                                 .clock = I2C_SENSORS_CLOCK_HZ,
                                 .address = GAZ_I2C_ADDRESS,
                                 .deviceName = GAZ_DEVICE_FULL_NAME };

    sf_i2c::Device tankDevice = { .route = sf_i2c::Route{},
                                  .sda = -1,
                                  .scl = -1,
                                  .clock = I2C_SENSORS_CLOCK_HZ,
                                  .address = TANK_I2C_ADDRESS,
                                  .deviceName = TANK_DEVICE_FULL_NAME };

    i2c.beginPortA(gazDevice.sda, gazDevice.scl);
    tankDevice.sda = gazDevice.sda;
    tankDevice.scl = gazDevice.scl;
    M5_LOGI("[I2C_SENSORS] using Wire SDA:%d SCL:%d", gazDevice.sda, gazDevice.scl);

    for (;;) {
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
                    gazInitialized = GAZ_MODULE.init(sf_i2c::isInternalRoute(gazDevice.route.mode));
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
                    tankInitialized = TANK_MODULE.init(tankDevice);
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

        if (gazInitialized && tankInitialized) {
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

        vTaskDelay(pdMS_TO_TICKS(I2C_SENSORS_LOOP_MS));
    }
}
