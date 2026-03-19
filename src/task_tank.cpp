#include <Arduino.h>
#include <M5Unified.h>

#include "config_store.h"
#include "i2c.h"
#include "tank.h"

namespace {

static constexpr uint32_t TANK_INIT_RETRY_MS = 10000UL;
static constexpr uint32_t TANK_LOOP_FALLBACK_MS = 60000UL;
static constexpr uint8_t TANK_I2C_ADDRESS = 0x57;
static constexpr uint32_t TANK_I2C_CLOCK_HZ = 400000U;
static constexpr const char* TANK_DEVICE_FULL_NAME = "M5Stack Unit Ultrasonic I2C (RCWL-9600)";

bool selectPaHubIfNeeded(const sf_i2c::I2C& i2c, const sf_i2c::Device& device)
{
    if (!sf_i2c::isPaHubRoute(device.route.mode)) {
        return true;
    }

    if (device.route.paHubChannel < 0) {
        M5_LOGW("[TANK] invalid PAHub channel %d", device.route.paHubChannel);
        return false;
    }

    if (!i2c.selectPaHubChannel(device.route.mode, static_cast<uint8_t>(device.route.paHubChannel))) {
        M5_LOGW("[TANK] failed to select PAHub channel %d", device.route.paHubChannel);
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

void taskTank(void *pv)
{
    (void)pv;
    M5_LOGI("[TANK] Task started");

    sf_i2c::I2C i2c{TANK_I2C_CLOCK_HZ};
    sf_i2c::Device device = { .route = sf_i2c::Route{},
                              .sda = -1,
                              .scl = -1,
                              .clock = TANK_I2C_CLOCK_HZ,
                              .address = TANK_I2C_ADDRESS,
                              .deviceName = TANK_DEVICE_FULL_NAME };

    i2c.beginPortA(device.sda, device.scl);
    M5_LOGI("[TANK] using Wire SDA:%d SCL:%d", device.sda, device.scl);

    while (true) {
        if (i2c.detectRoute(device.address, device.route)) {
            i2c.publishConfiguration(device);
            const bool channelSelected = selectPaHubIfNeeded(i2c, device);
            if (channelSelected && TANK_MODULE.init(device)) {
                disablePaHubIfNeeded(i2c, device);
                break;
            }
            disablePaHubIfNeeded(i2c, device);
        } else {
            M5_LOGW("[TANK] route detection failed");
            i2c.publishConfiguration(device);
        }

        M5_LOGW("[TANK] init failed, retry in 10s");
        vTaskDelay(pdMS_TO_TICKS(TANK_INIT_RETRY_MS));
    }

    for (;;) {
        if (selectPaHubIfNeeded(i2c, device)) {
            TANK_MODULE.process();
            disablePaHubIfNeeded(i2c, device);
        }
        vTaskDelay(pdMS_TO_TICKS(TANK_LOOP_FALLBACK_MS));
    }
}
