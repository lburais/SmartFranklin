#include "tank.h"

#include <M5Unified.h>

#include <cmath>
#include <cstdio>
#include <cstdint>
#include <mutex>

#include "data_model.h"
#include "i2c.h"
#include "mqtt.h"

namespace {

constexpr uint8_t TANK_I2C_ADDRESS = 0x57;
constexpr uint8_t TANK_DISTANCE_REGISTER = 0x01;
constexpr uint32_t TANK_I2C_CLOCK_HZ = 400000U;
constexpr uint32_t TANK_CONVERSION_DELAY_MS = 120U;
constexpr const char* TANK_DEVICE_FULL_NAME = "M5Stack Unit Ultrasonic I2C (RCWL-9600)";

constexpr int32_t TANK_DISTANCE_MIN_MM = 20;
constexpr int32_t TANK_DISTANCE_MAX_MM = 4500;

// Tank mapping defaults: tune these values to match the physical installation.
constexpr int32_t TANK_FULL_DISTANCE_MM = 300;
constexpr int32_t TANK_EMPTY_DISTANCE_MM = 1500;

int32_t clampDistanceMm(int32_t distanceMm)
{
    if (distanceMm < TANK_DISTANCE_MIN_MM) {
        return TANK_DISTANCE_MIN_MM;
    }
    if (distanceMm > TANK_DISTANCE_MAX_MM) {
        return TANK_DISTANCE_MAX_MM;
    }
    return distanceMm;
}

int32_t distanceToFillPct(const int32_t distanceMm)
{
    if (distanceMm <= TANK_FULL_DISTANCE_MM) {
        return 100;
    }

    if (distanceMm >= TANK_EMPTY_DISTANCE_MM) {
        return 0;
    }

    const float span = static_cast<float>(TANK_EMPTY_DISTANCE_MM - TANK_FULL_DISTANCE_MM);
    if (span <= 0.0f) {
        return 0;
    }

    const float numerator = static_cast<float>(TANK_EMPTY_DISTANCE_MM - distanceMm);
    const int32_t pct = static_cast<int32_t>(lroundf((numerator / span) * 100.0f));

    if (pct < 0) {
        return 0;
    }
    if (pct > 100) {
        return 100;
    }
    return pct;
}

class TankRuntime {
public:
    bool init(const sf_i2c::Device& device);
    void process();
    bool isInitialized() const;

private:
    void publishDistance(int32_t distanceMm, int32_t fillPct) const;
    bool refreshMeasurementLocked(int32_t& distanceMm);
    bool readDistanceWireLocked(uint32_t& rawDistance) const;
    bool readDistanceExLocked(uint32_t& rawDistance) const;

    mutable std::mutex m_mutex;

    bool m_initialized = false;
    int32_t m_lastDistanceMm = 0;

    sf_i2c::Device m_device = { .route = sf_i2c::Route{}, 
                                .sda = -1, 
                                .scl = -1, 
                                .clock = TANK_I2C_CLOCK_HZ, 
                                .address = TANK_I2C_ADDRESS, 
                                .deviceName = TANK_DEVICE_FULL_NAME };
};

TankRuntime TANK_RUNTIME;

}  // namespace

Tank TANK_MODULE;


bool TankRuntime::readDistanceWireLocked(uint32_t& rawDistance) const
{
    Wire.beginTransmission(TANK_I2C_ADDRESS);
    Wire.write(TANK_DISTANCE_REGISTER);
    if (Wire.endTransmission() != 0) {
        return false;
    }

    delay(TANK_CONVERSION_DELAY_MS);

    const uint8_t readCount = Wire.requestFrom(TANK_I2C_ADDRESS, static_cast<uint8_t>(3));
    if (readCount < 3) {
        return false;
    }

    rawDistance = 0;
    rawDistance = static_cast<uint32_t>(Wire.read());
    rawDistance <<= 8;
    rawDistance |= static_cast<uint32_t>(Wire.read());
    rawDistance <<= 8;
    rawDistance |= static_cast<uint32_t>(Wire.read());
    return true;
}

bool TankRuntime::readDistanceExLocked(uint32_t& rawDistance) const
{
    if (!M5.Ex_I2C.start(TANK_I2C_ADDRESS, false, TANK_I2C_CLOCK_HZ)) {
        return false;
    }

    if (!M5.Ex_I2C.write(TANK_DISTANCE_REGISTER) || !M5.Ex_I2C.stop()) {
        return false;
    }

    delay(TANK_CONVERSION_DELAY_MS);

    if (!M5.Ex_I2C.start(TANK_I2C_ADDRESS, true, TANK_I2C_CLOCK_HZ)) {
        return false;
    }

    rawDistance = 0;
    for (uint8_t i = 0; i < 3; ++i) {
        uint8_t byte = 0;
        const bool lastNack = (i == 2);
        if (!M5.Ex_I2C.read(&byte, 1U, lastNack)) {
            M5.Ex_I2C.stop();
            return false;
        }

        rawDistance <<= 8;
        rawDistance |= static_cast<uint32_t>(byte);
    }

    return M5.Ex_I2C.stop();
}

bool TankRuntime::refreshMeasurementLocked(int32_t& distanceMm)
{
    uint32_t rawDistance = 0;
    const bool readOk = sf_i2c::isInternalRoute(m_device.route.mode)
        ? readDistanceExLocked(rawDistance)
        : readDistanceWireLocked(rawDistance);

    if (!readOk) {
        return false;
    }

    const float distanceRawMm = static_cast<float>(rawDistance) / 1000.0f;
    if (!std::isfinite(distanceRawMm)) {
        M5_LOGW("[TANK] non-finite distance sample ignored");
        return false;
    }

    distanceMm = clampDistanceMm(static_cast<int32_t>(lroundf(distanceRawMm)));
    m_lastDistanceMm = distanceMm;
    return true;
}

bool TankRuntime::init(const sf_i2c::Device& device)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    m_device = device;
    if (m_device.route.mode == sf_i2c::RouteMode::Unset) {
        m_initialized = false;
        M5_LOGW("[TANK] invalid I2C route (unset)");
        return false;
    }

    m_initialized = true;
    M5_LOGI("[TANK] using route=%s channel=%d SDA=%d SCL=%d",
            sf_i2c::routeModeToString(m_device.route.mode),
            m_device.route.paHubChannel,
            m_device.sda,
            m_device.scl);

    M5_LOGI("[TANK] Ultrasonic I2C initialization complete");
    return true;
}

void TankRuntime::publishDistance(const int32_t distanceMm, const int32_t fillPct) const
{
    char mmBuf[24] = {0};
    snprintf(mmBuf, sizeof(mmBuf), "%d", distanceMm);
    sf_mqtt::publish("smartfranklin/tank/mm", mmBuf);

    char pctBuf[16] = {0};
    snprintf(pctBuf, sizeof(pctBuf), "%d", fillPct);
    sf_mqtt::publish("smartfranklin/tank/fill", pctBuf, 1, true);

    M5_LOGI("[TANK] Distance: %d mm     Fill level: %d%%", distanceMm, fillPct);
}

void TankRuntime::process()
{
    int32_t distanceMm = 0;
    bool hasMeasurement = false;

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_initialized) {
            return;
        }

        hasMeasurement = refreshMeasurementLocked(distanceMm);
    }

    if (!hasMeasurement) {
        M5_LOGW("[TANK] No measurement");
        return;
    }

    const int32_t fillPct = distanceToFillPct(distanceMm);

    {
        std::lock_guard<std::mutex> lock(DATA_MUTEX);
        DATA.distance_tank_mm = distanceMm;
        DATA.fill_tank = fillPct;
    }

    publishDistance(distanceMm, fillPct);
}

bool TankRuntime::isInitialized() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_initialized;
}

bool Tank::init(const sf_i2c::Device& device)
{
    return TANK_RUNTIME.init(device);
}

void Tank::process()
{
    TANK_RUNTIME.process();
}

bool Tank::isInitialized() const
{
    return TANK_RUNTIME.isInitialized();
}
