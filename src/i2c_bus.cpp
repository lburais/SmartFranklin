/*
 * ============================================================================
 * I2C Bus Implementation - SmartFranklin
 * ============================================================================
 *
 * File:        i2c_bus.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: Implements I2C topology discovery and targeted route detection
 *              across Wire, Ex_I2C, and PAHub-multiplexed segments.
 *
 * Author:      Laurent Burais
 * Date:        12 March 2026
 * Version:     1.2
 *
 * Overview:
 *   This module is responsible for:
 *   - scanning and reporting the full I2C topology,
 *   - identifying known device addresses with readable labels,
 *   - probing Gravity dual-UART bridge candidates,
 *   - resolving direct-vs-PAHub routing for a single target address.
 *
 * Dependencies:
 *   - i2c_bus.h (public API)
 *   - m5_hw.h/M5Unified (board pins and Ex_I2C access)
 *   - Wire (base I2C bus operations)
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

#include "i2c_bus.h"

#include <cstring>

namespace {

struct I2CAddressLabel {
    uint8_t address;
    const char* label;
};

constexpr uint32_t I2C_SCAN_SPEED = 400000U;

// M5 I2C addresses
constexpr uint8_t PAHUB_ADDRESS = 0x70;
constexpr uint8_t DISTANCE_I2C_ADDRESS = 0x57;
constexpr uint8_t WEIGHT_I2C_ADDRESS = 0x26;
constexpr uint8_t RTC_ADDR_PCF8563 = 0x51;
constexpr uint8_t RTC_ADDR_RX8130 = 0x32;
constexpr uint8_t RTC_ADDR_POWERHUB = 0x50;

// Common Seeed Studio / Grove I2C addresses.
constexpr uint8_t SEEED_GROVE_OLED_ADDR = 0x3C;
constexpr uint8_t SEEED_GROVE_OLED_ALT_ADDR = 0x3D;
constexpr uint8_t SEEED_GROVE_BH1750_ADDR = 0x23;
constexpr uint8_t SEEED_GROVE_SHT31_ADDR = 0x44;
constexpr uint8_t SEEED_GROVE_SHT31_ALT_ADDR = 0x45;
constexpr uint8_t SEEED_GROVE_SI7021_ADDR = 0x40;
constexpr uint8_t SEEED_GROVE_VL53L0X_ADDR = 0x29;
constexpr uint8_t SEEED_GROVE_BME280_ADDR = 0x76;
constexpr uint8_t SEEED_GROVE_BME280_ALT_ADDR = 0x77;
constexpr uint8_t SEEED_GROVE_APDS9960_ADDR = 0x39;
constexpr uint8_t SEEED_GROVE_SCD30_ADDR = 0x61;
constexpr uint8_t SEEED_GROVE_COMPASS_QMC5883L_ADDR = 0x0D;
constexpr uint8_t SEEED_GROVE_COMPASS_HMC5883L_ADDR = 0x1E;

// Common DFRobot Gravity I2C addresses.
constexpr uint8_t GRAVITY_URM09_ADDR = 0x11;

constexpr I2CAddressLabel M5_I2C_ADDRESS_LABELS[] = {
    {0x08, "M5 Faces keyboard/gamepad family (MEGA328)"},
    {0x0C, "M5Unified IMU magnetometer (AK8963)"},
    {0x10, "M5Unified IMU magnetometer (BMM150)"},
    {0x10, "M5Unified audio codec (ES8388)"},
    {0x11, "M5 Unit MQ (STM32G030)"},
    {0x14, "M5 touch controller (GT911 family)"},
    {0x15, "M5GFX touch controller (CST816S)"},
    {0x18, "M5Unified audio codec (ES8311 addr0)"},
    {0x19, "M5Unified audio codec (ES8311 addr1)"},
    {0x1A, "M5GFX touch controller (CST226 alt)"},
    {0x20, "M5 Unit HBridge family (STM32F030 range 0x20-0x2F)"},
    {0x21, "M5 Unit QRCode controller (STM32F030)"},
    {0x21, "M5Unified/M5GFX camera sensor (GC0308)"},
    {0x22, "M5 Base X (STM32F030)"},
    {0x23, "M5 Core light sensor family (LTR533/BH1750 overlap)"},
    {0x24, "M5 Module 4EncoderMotor family"},
    {0x25, "M5 Unit/Hat 8Servos family"},
    {WEIGHT_I2C_ADDRESS, "M5Unit-WEIGHT UnitWeightI2C / UnitMiniScales"},
    {0x27, "M5 EXT.IO / Module13.2 stepmotor driver family"},
    {0x28, "M5 RFID2/Faces RFID/Dial family"},
    {0x29, "M5 ToF family (VL53L0X)"},
    {0x2E, "M5GFX touch controller (M5Tough profile)"},
    {0x30, "M5 Unit DigiClock bridge"},
    {0x31, "M5 Unit DDS"},
    {RTC_ADDR_RX8130, "M5Unified RTC (RX8130)"},
    {0x33, "M5 thermal/audio bridge family"},
    {0x34, "M5Unified power PMIC (AXP192 / AXP2101)"},
    {0x35, "M5 Unit ID secure element (ATECC608)"},
    {0x36, "M5Unified audio amplifier (AW88298)"},
    {0x38, "M5GFX touch controller (FT5x06 profile)"},
    {0x38, "M5 touch/HMI/motion controller family"},
    {0x39, "M5AtomDisplay/M5ModuleDisplay HDMI transmitter profile"},
    {0x3C, "M5Unified camera sensor profile (OV3660)"},
    {0x3C, "M5GFX generic I2C display/touch bus profile"},
    {0x3E, "M5 Unit LCD bridge (ST7789V2)"},
    {0x40, "M5Unified power monitor (INA226)"},
    {0x40, "M5Unified power monitor (INA3221)"},
    {0x40, "M5Unified audio ADC (ES7210)"},
    {0x41, "M5 power monitor secondary address (INA226/INA3221)"},
    {0x42, "M5 Mini EncoderC"},
    {0x43, "M5Unified IO expander (PI4IOE5V6408)"},
    {0x44, "M5GFX IO expander profile (PI4IOE secondary)"},
    {0x45, "M5 IO expander family (PI4IOE5V6408 possible alt strap)"},
    {0x46, "M5 IO expander family (PI4IOE5V6408 possible alt strap)"},
    {0x49, "M5Unified power controller (AW32001)"},
    {0x50, "M5Unified PowerHub / RTC_PowerHub / LED_PowerHub"},
    {RTC_ADDR_PCF8563, "M5Unified RTC (PCF8563/BM8563)"},
    {0x53, "M5 Module Servo"},
    {0x54, "M5 Mini JoyC"},
    {0x55, "M5Unified fuel gauge (BQ27220)"},
    {0x56, "M5 Module DCMotor"},
    {DISTANCE_I2C_ADDRESS, "M5Unit-DISTANCE UnitRCWL9620 / UnitUltraSonicI2C"},
    {0x58, "M5Unified IO expander (AW9523)"},
    {0x59, "M5 IO expander family (AW9523 possible alt address)"},
    {0x5A, "M5 IO expander family (AW9523 possible alt address)"},
    {0x5A, "M5GFX touch controller (CST226SE)"},
    {0x5B, "M5 IO expander family (AW9523 possible alt address)"},
    {0x5E, "M5 Faces Encoder/Joystick family"},
    {0x5F, "M5 Hat CardKB"},
    {0x61, "M5Unit-HUB UnitPbHub"},
    {0x65, "M5 Unit BLDC driver"},
    {0x68, "M5Unified IMU (MPU6886)"},
    {0x69, "M5Unified IMU (BMI270)"},
    {0x6C, "M5Unified IMU (SH200Q)"},
    {0x6E, "M5Unified power PMIC (PY32PMIC / M5PM1)"},
    {PAHUB_ADDRESS, "M5Unit-HUB UnitPCA9548AP / PAHUB (default)"},
    {0x71, "M5Unit-HUB UnitPCA9548AP / PAHUB (alt strap A0)"},
    {0x72, "M5Unit-HUB UnitPCA9548AP / PAHUB (alt strap A1)"},
    {0x73, "M5Unit-HUB UnitPCA9548AP / PAHUB (alt strap A0+A1)"},
    {0x74, "M5Unit-HUB UnitPCA9548AP / PAHUB (alt strap A2)"},
    {0x75, "M5Unit-HUB UnitPCA9548AP / PAHUB (alt strap A2+A0)"},
    {0x75, "M5Unified power controller (IP5306)"},
    {0x76, "M5Unit-HUB UnitPCA9548AP / PAHUB (alt strap A2+A1)"},
    {0x77, "M5Unit-HUB UnitPCA9548AP / PAHUB (alt strap A2+A1+A0)"},
};

constexpr I2CAddressLabel THIRD_PARTY_I2C_ADDRESS_LABELS[] = {
    {0x08, "DFRobot Gravity programmable controller family"},
    {SEEED_GROVE_COMPASS_QMC5883L_ADDR, "Seeed Grove 3-axis compass (QMC5883L)"},
    {GRAVITY_URM09_ADDR, "DFRobot Gravity URM09 ultrasonic ranger"},
    {SEEED_GROVE_COMPASS_HMC5883L_ADDR, "Seeed Grove 3-axis compass (HMC5883L)"},
    {0x1E, "DFRobot Gravity compass family (HMC5883L/LIS2MDL overlap)"},
    {0x20, "DFRobot Gravity IO expander / actuator family"},
    {0x20, "DFRobot Gravity GNSS GPS BeiDou receiver (TEL0157 / L76K)"},
    {0x21, "DFRobot Gravity QR/vision bridge family"},
    {SEEED_GROVE_BH1750_ADDR, "Seeed Grove / DFRobot Gravity BH1750 light sensor"},
    {0x23, "DFRobot Gravity BH1750 light sensor"},
    {0x26, "DFRobot Gravity relay/actuator controller family"},
    {0x27, "DFRobot Gravity digital IO expander family"},
    {0x28, "DFRobot Gravity RFID / touch-key family"},
    {SEEED_GROVE_VL53L0X_ADDR, "Seeed Grove / DFRobot Gravity VL53L0X ToF ranger"},
    {0x29, "DFRobot Gravity ToF / color sensor family (VL53L0X/TCS34725 overlap)"},
    {0x30, "DFRobot Gravity display/clock bridge family"},
    {0x31, "DFRobot Gravity waveform/DDS controller family"},
    {0x32, "DFRobot Gravity HuskyLens / UART-bridge family"},
    {0x33, "DFRobot Gravity thermal/audio bridge family"},
    {0x38, "DFRobot Gravity AHT20 / touch-controller family"},
    {SEEED_GROVE_APDS9960_ADDR, "Seeed Grove / DFRobot Gravity APDS9960 gesture/proximity"},
    {0x39, "Seeed Grove / DFRobot Gravity APDS9960/AS7341 overlap"},
    {0x3A, "DFRobot Gravity display bridge family"},
    {0x3B, "DFRobot Gravity display bridge family (alt)"},
    {SEEED_GROVE_OLED_ADDR, "Seeed Grove / DFRobot Gravity OLED display (0x3C)"},
    {0x3C, "DFRobot Gravity OLED / camera overlap"},
    {SEEED_GROVE_OLED_ALT_ADDR, "Seeed Grove / DFRobot Gravity OLED display alt (0x3D)"},
    {0x3E, "DFRobot Gravity LCD bridge family"},
    {SEEED_GROVE_SI7021_ADDR, "Seeed Grove / DFRobot Gravity SI7021 temp/humidity"},
    {0x40, "DFRobot Gravity current/power sensor family (INA219/INA226 overlap)"},
    {0x41, "DFRobot Gravity current/power sensor family (alt)"},
    {0x42, "DFRobot Gravity encoder/IO controller family"},
    {0x43, "DFRobot Gravity IO expander family"},
    {SEEED_GROVE_SHT31_ADDR, "Seeed Grove / DFRobot Gravity SHT31 temp/humidity"},
    {SEEED_GROVE_SHT31_ALT_ADDR, "Seeed Grove / DFRobot Gravity SHT31 temp/humidity"},
    {0x46, "DFRobot Gravity IO expander family (alt)"},
    {0x47, "DFRobot Gravity IO expander family (alt)"},
    {0x48, "DFRobot Gravity ADC / dual-UART overlap"},
    {0x49, "DFRobot Gravity ADC / PMIC overlap"},
    {0x4A, "DFRobot Gravity ADC family"},
    {0x4B, "DFRobot Gravity ADC family"},
    {0x50, "DFRobot Gravity ACSSR/DCSSR controller family"},
    {0x51, "DFRobot Gravity RTC family (BM8563/PCF8563 overlap)"},
    {0x52, "DFRobot Gravity gas sensor family"},
    {0x53, "DFRobot Gravity gas/servo controller family"},
    {0x54, "DFRobot Gravity joystick/controller family"},
    {0x55, "DFRobot Gravity battery/fuel-gauge family"},
    {0x56, "DFRobot Gravity motor controller family"},
    {0x57, "DFRobot Gravity MAX30102/ultrasonic overlap"},
    {0x58, "DFRobot Gravity SGP30 / Dual-UART overlap"},
    {0x59, "DFRobot Gravity SGP40 / IO-expander overlap"},
    {0x5A, "DFRobot Gravity MLX90614 / capacitive-touch overlap"},
    {0x5B, "DFRobot Gravity CCS811 / IO-expander overlap"},
    {0x5C, "DFRobot Gravity environmental sensor family"},
    {0x5D, "DFRobot Gravity environmental sensor family (alt)"},
    {0x5F, "DFRobot Gravity keyboard/controller family"},
    {SEEED_GROVE_SCD30_ADDR, "Seeed Grove / DFRobot Gravity SCD30 CO2 sensor"},
    {0x62, "DFRobot Gravity ORP/water-quality family"},
    {0x63, "DFRobot Gravity pH sensor family"},
    {0x64, "DFRobot Gravity EC/TDS sensor family"},
    {0x65, "DFRobot Gravity BLDC/motor driver family"},
    {0x66, "DFRobot Gravity GNSS positioning and timing module (DFR1103)"},
    {0x68, "DFRobot Gravity IMU family (MPU6050/MPU6886 overlap)"},
    {0x69, "DFRobot Gravity IMU family (ICM/BMI overlap)"},
    {0x6A, "DFRobot Gravity IMU family (alt)"},
    {0x6B, "DFRobot Gravity IMU family (alt)"},
    {0x6C, "DFRobot Gravity IMU family (SH200Q overlap)"},
    {0x6E, "DFRobot Gravity PMIC/power family"},
    {0x6F, "DFRobot Gravity PMIC/power family (alt)"},
    {0x70, "DFRobot Gravity I2C multiplexer family (TCA9548/PCA9548 window)"},
    {0x71, "DFRobot Gravity I2C multiplexer family (window)"},
    {0x72, "DFRobot Gravity I2C multiplexer family (window)"},
    {0x73, "DFRobot Gravity dissolved-oxygen / multiplexer overlap"},
    {0x74, "DFRobot Gravity I2C multiplexer family (window)"},
    {0x75, "DFRobot Gravity power/multiplexer overlap"},
    {SEEED_GROVE_BME280_ADDR, "Seeed Grove / DFRobot Gravity BME280 env sensor"},
    {0x76, "DFRobot Gravity BME/BMP family (0x76)"},
    {SEEED_GROVE_BME280_ALT_ADDR, "Seeed Grove / DFRobot Gravity BME280 env sensor"},
    {0x77, "DFRobot Gravity BME/BMP family (0x77)"},
};

void append_label(char* buffer, const size_t buffer_size, const char* label)
{
    if (buffer == nullptr || label == nullptr || buffer_size == 0 || label[0] == '\0') {
        return;
    }

    const size_t used = std::strlen(buffer);
    if (used >= buffer_size - 1U) {
        return;
    }

    size_t remaining = buffer_size - used - 1U;
    if (used > 0 && remaining > 0U) {
        std::strncat(buffer, " | ", remaining);
    }

    remaining = buffer_size - std::strlen(buffer) - 1U;
    if (remaining > 0U) {
        std::strncat(buffer, label, remaining);
    }
}

bool wire_device_exists(const uint8_t address)
{
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

bool ex_i2c_device_exists(const uint8_t address)
{
    return M5.Ex_I2C.scanID(address, I2C_SCAN_SPEED);
}

bool wire_pahub_select_channel(const uint8_t channel)
{
    Wire.beginTransmission(PAHUB_ADDRESS);
    Wire.write(static_cast<uint8_t>(1U << channel));
    return Wire.endTransmission() == 0;
}

void wire_pahub_disable_all_channels()
{
    Wire.beginTransmission(PAHUB_ADDRESS);
    Wire.write(static_cast<uint8_t>(0x00));
    Wire.endTransmission();
}

bool ex_pahub_select_channel(const uint8_t channel)
{
    if (!M5.Ex_I2C.start(PAHUB_ADDRESS, false, I2C_SCAN_SPEED)) {
        return false;
    }

    const bool write_ok = M5.Ex_I2C.write(static_cast<uint8_t>(1U << channel));
    const bool stop_ok = M5.Ex_I2C.stop();
    return write_ok && stop_ok;
}

void ex_pahub_disable_all_channels()
{
    if (!M5.Ex_I2C.start(PAHUB_ADDRESS, false, I2C_SCAN_SPEED)) {
        return;
    }

    M5.Ex_I2C.write(static_cast<uint8_t>(0x00));
    M5.Ex_I2C.stop();
}

bool is_gravity_dual_uart_candidate(const uint8_t address)
{
    // Common SC16IS752/SC16IS762 I2C address windows used by dual UART bridges.
    return (address >= 0x48 && address <= 0x4F)
        || (address >= 0x58 && address <= 0x5F);
}

const char* i2c_unit_name(const uint8_t address)
{
    static char resolved[640];
    resolved[0] = '\0';

    for (const auto& entry : M5_I2C_ADDRESS_LABELS) {
        if (entry.address == address) {
            append_label(resolved, sizeof(resolved), entry.label);
        }
    }

    if (address >= 0x20 && address <= 0x2F) {
        append_label(resolved, sizeof(resolved), "M5 programmable device window (HBridge family 0x20-0x2F)");
    }
    if (address >= 0x40 && address <= 0x47) {
        append_label(resolved, sizeof(resolved), "M5 programmable device window (PCA9685/IO expander family 0x40-0x47)");
    }

    if (is_gravity_dual_uart_candidate(address)) {
        append_label(resolved, sizeof(resolved), "DFRobot Gravity I2C Dual UART (SC16IS752/762 candidate)");
    }

    for (const auto& entry : THIRD_PARTY_I2C_ADDRESS_LABELS) {
        if (entry.address == address) {
            append_label(resolved, sizeof(resolved), entry.label);
        }
    }

    if (resolved[0] != '\0') {
        return resolved;
    }

    return "Unknown/unsupported I2C device";
}

constexpr uint8_t I2C_MIN_ADDRESS = 0x08;
constexpr uint8_t I2C_MAX_ADDRESS = 0x77;
constexpr uint8_t PAHUB_CHANNEL_COUNT = 8;
constexpr uint8_t I2C_MAX_DISCOVERED_PER_SEGMENT = (I2C_MAX_ADDRESS - I2C_MIN_ADDRESS + 1);

struct BusScanSnapshot {
    uint8_t direct[I2C_MAX_DISCOVERED_PER_SEGMENT] = {0};
    uint8_t direct_count = 0;

    bool pahub_found = false;
    uint8_t pahub_devices[PAHUB_CHANNEL_COUNT][I2C_MAX_DISCOVERED_PER_SEGMENT] = {{0}};
    uint8_t pahub_device_counts[PAHUB_CHANNEL_COUNT] = {0};
};

bool append_discovered_address(uint8_t* list, uint8_t& count, const uint8_t address)
{
    if (count >= I2C_MAX_DISCOVERED_PER_SEGMENT) {
        return false;
    }

    list[count++] = address;
    return true;
}

void scan_direct_wire(BusScanSnapshot& snapshot)
{
    for (uint8_t address = I2C_MIN_ADDRESS; address <= I2C_MAX_ADDRESS; ++address) {
        if (wire_device_exists(address)) {
            append_discovered_address(snapshot.direct, snapshot.direct_count, address);
        }
    }
}

void scan_direct_ex(BusScanSnapshot& snapshot)
{
    for (uint8_t address = I2C_MIN_ADDRESS; address <= I2C_MAX_ADDRESS; ++address) {
        if (ex_i2c_device_exists(address)) {
            append_discovered_address(snapshot.direct, snapshot.direct_count, address);
        }
    }
}

void scan_wire_pahub(BusScanSnapshot& snapshot)
{
    snapshot.pahub_found = wire_device_exists(PAHUB_ADDRESS);
    if (!snapshot.pahub_found) {
        return;
    }

    for (uint8_t channel = 0; channel < PAHUB_CHANNEL_COUNT; ++channel) {
        if (!wire_pahub_select_channel(channel)) {
            continue;
        }

        for (uint8_t address = I2C_MIN_ADDRESS; address <= I2C_MAX_ADDRESS; ++address) {
            if (address == PAHUB_ADDRESS || !wire_device_exists(address)) {
                continue;
            }
            append_discovered_address(snapshot.pahub_devices[channel], snapshot.pahub_device_counts[channel], address);
        }
    }

    wire_pahub_disable_all_channels();
}

void scan_ex_pahub(BusScanSnapshot& snapshot)
{
    snapshot.pahub_found = ex_i2c_device_exists(PAHUB_ADDRESS);
    if (!snapshot.pahub_found) {
        return;
    }

    for (uint8_t channel = 0; channel < PAHUB_CHANNEL_COUNT; ++channel) {
        if (!ex_pahub_select_channel(channel)) {
            continue;
        }

        for (uint8_t address = I2C_MIN_ADDRESS; address <= I2C_MAX_ADDRESS; ++address) {
            if (address == PAHUB_ADDRESS || !ex_i2c_device_exists(address)) {
                continue;
            }
            append_discovered_address(snapshot.pahub_devices[channel], snapshot.pahub_device_counts[channel], address);
        }
    }

    ex_pahub_disable_all_channels();
}

void log_bus_hierarchy(const char* bus_tag, const BusScanSnapshot& snapshot)
{
    M5_LOGI("[I2C][REPORT] %s", bus_tag);

    if (snapshot.direct_count == 0) {
        M5_LOGI("[I2C][REPORT]   direct: none");
    } else {
        M5_LOGI("[I2C][REPORT]   direct:");
        for (uint8_t i = 0; i < snapshot.direct_count; ++i) {
            const uint8_t address = snapshot.direct[i];
            M5_LOGI("[I2C][REPORT]     - 0x%02X (%s)", address, i2c_unit_name(address));
        }
    }

    if (!snapshot.pahub_found) {
        M5_LOGI("[I2C][REPORT]   pahub: not found");
        return;
    }

    M5_LOGI("[I2C][REPORT]   pahub @0x%02X (%s)", PAHUB_ADDRESS, i2c_unit_name(PAHUB_ADDRESS));

    bool downstream_found = false;
    for (uint8_t channel = 0; channel < PAHUB_CHANNEL_COUNT; ++channel) {
        const uint8_t channel_count = snapshot.pahub_device_counts[channel];
        if (channel_count == 0) {
            continue;
        }

        downstream_found = true;
        M5_LOGI("[I2C][REPORT]     channel %u:", channel);
        for (uint8_t i = 0; i < channel_count; ++i) {
            const uint8_t address = snapshot.pahub_devices[channel][i];
            M5_LOGI("[I2C][REPORT]       - 0x%02X (%s)", address, i2c_unit_name(address));
        }
    }

    if (!downstream_found) {
        M5_LOGI("[I2C][REPORT]     no downstream devices detected");
    }
}

constexpr uint8_t SC16_REG_RHR = 0x00;
constexpr uint8_t SC16_REG_THR = 0x00;
constexpr uint8_t SC16_REG_LCR = 0x03;
constexpr uint8_t SC16_REG_LSR = 0x05;
constexpr uint8_t SC16_REG_SPR = 0x07;

constexpr uint8_t SC16_CH_A = 0;
constexpr uint8_t SC16_CH_B = 1;

struct GravityProbeResult {
    bool ran = false;
    bool nb_iot2_confirmed = false;
    bool c6l_activity_detected = false;
};

uint8_t sc16_reg_addr(const uint8_t reg, const uint8_t channel)
{
    return static_cast<uint8_t>((reg << 3U) | (channel << 1U));
}

bool wire_write_raw(const uint8_t address, const uint8_t* data, const size_t len)
{
    Wire.beginTransmission(address);
    for (size_t i = 0; i < len; ++i) {
        Wire.write(data[i]);
    }
    return Wire.endTransmission() == 0;
}

bool wire_read_raw(const uint8_t address, uint8_t* data, const size_t len)
{
    const uint8_t requested = Wire.requestFrom(static_cast<int>(address), static_cast<int>(len));
    if (requested != len) {
        return false;
    }

    for (size_t i = 0; i < len; ++i) {
        if (!Wire.available()) {
            return false;
        }
        data[i] = static_cast<uint8_t>(Wire.read());
    }
    return true;
}

bool ex_write_raw(const uint8_t address, const uint8_t* data, const size_t len)
{
    if (!M5.Ex_I2C.start(address, false, I2C_SCAN_SPEED)) {
        return false;
    }

    for (size_t i = 0; i < len; ++i) {
        if (!M5.Ex_I2C.write(data[i])) {
            M5.Ex_I2C.stop();
            return false;
        }
    }

    return M5.Ex_I2C.stop();
}

bool ex_read_raw(const uint8_t address, uint8_t* data, const size_t len)
{
    if (data == nullptr || len == 0) {
        return false;
    }

    if (!M5.Ex_I2C.start(address, true, I2C_SCAN_SPEED)) {
        return false;
    }

    for (size_t i = 0; i < len; ++i) {
        const bool last_nack = (i + 1U >= len);
        if (!M5.Ex_I2C.read(&data[i], 1U, last_nack)) {
            M5.Ex_I2C.stop();
            return false;
        }
    }

    return M5.Ex_I2C.stop();
}

bool sc16_write_reg_wire(const uint8_t address, const uint8_t channel, const uint8_t reg, const uint8_t value)
{
    const uint8_t payload[] = {sc16_reg_addr(reg, channel), value};
    return wire_write_raw(address, payload, sizeof(payload));
}

bool sc16_read_reg_wire(const uint8_t address, const uint8_t channel, const uint8_t reg, uint8_t& out)
{
    const uint8_t reg_addr = sc16_reg_addr(reg, channel);
    if (!wire_write_raw(address, &reg_addr, 1U)) {
        return false;
    }
    return wire_read_raw(address, &out, 1U);
}

bool sc16_write_reg_ex(const uint8_t address, const uint8_t channel, const uint8_t reg, const uint8_t value)
{
    const uint8_t payload[] = {sc16_reg_addr(reg, channel), value};
    return ex_write_raw(address, payload, sizeof(payload));
}

bool sc16_read_reg_ex(const uint8_t address, const uint8_t channel, const uint8_t reg, uint8_t& out)
{
    const uint8_t reg_addr = sc16_reg_addr(reg, channel);
    if (!ex_write_raw(address, &reg_addr, 1U)) {
        return false;
    }
    return ex_read_raw(address, &out, 1U);
}

void sc16_flush_rx_wire(const uint8_t address, const uint8_t channel)
{
    for (uint8_t i = 0; i < 64; ++i) {
        uint8_t lsr = 0;
        if (!sc16_read_reg_wire(address, channel, SC16_REG_LSR, lsr) || (lsr & 0x01U) == 0) {
            return;
        }

        uint8_t discard = 0;
        if (!sc16_read_reg_wire(address, channel, SC16_REG_RHR, discard)) {
            return;
        }
    }
}

void sc16_flush_rx_ex(const uint8_t address, const uint8_t channel)
{
    for (uint8_t i = 0; i < 64; ++i) {
        uint8_t lsr = 0;
        if (!sc16_read_reg_ex(address, channel, SC16_REG_LSR, lsr) || (lsr & 0x01U) == 0) {
            return;
        }

        uint8_t discard = 0;
        if (!sc16_read_reg_ex(address, channel, SC16_REG_RHR, discard)) {
            return;
        }
    }
}

bool sc16_send_bytes_wire(const uint8_t address, const uint8_t channel, const char* text)
{
    if (text == nullptr) {
        return false;
    }

    for (const char* p = text; *p != '\0'; ++p) {
        const uint32_t start = millis();
        uint8_t lsr = 0;

        while (true) {
            if (!sc16_read_reg_wire(address, channel, SC16_REG_LSR, lsr)) {
                return false;
            }
            if ((lsr & 0x20U) != 0) {
                break;
            }
            if (millis() - start > 100U) {
                return false;
            }
            delay(1);
        }

        if (!sc16_write_reg_wire(address, channel, SC16_REG_THR, static_cast<uint8_t>(*p))) {
            return false;
        }
    }

    return true;
}

bool sc16_send_bytes_ex(const uint8_t address, const uint8_t channel, const char* text)
{
    if (text == nullptr) {
        return false;
    }

    for (const char* p = text; *p != '\0'; ++p) {
        const uint32_t start = millis();
        uint8_t lsr = 0;

        while (true) {
            if (!sc16_read_reg_ex(address, channel, SC16_REG_LSR, lsr)) {
                return false;
            }
            if ((lsr & 0x20U) != 0) {
                break;
            }
            if (millis() - start > 100U) {
                return false;
            }
            delay(1);
        }

        if (!sc16_write_reg_ex(address, channel, SC16_REG_THR, static_cast<uint8_t>(*p))) {
            return false;
        }
    }

    return true;
}

size_t sc16_read_text_wire(const uint8_t address,
                          const uint8_t channel,
                          char* buffer,
                          const size_t buffer_len,
                          const uint32_t timeout_ms)
{
    if (buffer == nullptr || buffer_len == 0) {
        return 0;
    }

    size_t written = 0;
    const uint32_t start = millis();

    while (millis() - start < timeout_ms) {
        uint8_t lsr = 0;
        if (!sc16_read_reg_wire(address, channel, SC16_REG_LSR, lsr)) {
            break;
        }

        if ((lsr & 0x01U) == 0) {
            delay(2);
            continue;
        }

        uint8_t byte_value = 0;
        if (!sc16_read_reg_wire(address, channel, SC16_REG_RHR, byte_value)) {
            break;
        }

        if (written + 1U < buffer_len) {
            if ((byte_value >= 32U && byte_value <= 126U) || byte_value == '\r' || byte_value == '\n') {
                buffer[written++] = static_cast<char>(byte_value);
            }
        }
    }

    buffer[written] = '\0';
    return written;
}

size_t sc16_read_text_ex(const uint8_t address,
                        const uint8_t channel,
                        char* buffer,
                        const size_t buffer_len,
                        const uint32_t timeout_ms)
{
    if (buffer == nullptr || buffer_len == 0) {
        return 0;
    }

    size_t written = 0;
    const uint32_t start = millis();

    while (millis() - start < timeout_ms) {
        uint8_t lsr = 0;
        if (!sc16_read_reg_ex(address, channel, SC16_REG_LSR, lsr)) {
            break;
        }

        if ((lsr & 0x01U) == 0) {
            delay(2);
            continue;
        }

        uint8_t byte_value = 0;
        if (!sc16_read_reg_ex(address, channel, SC16_REG_RHR, byte_value)) {
            break;
        }

        if (written + 1U < buffer_len) {
            if ((byte_value >= 32U && byte_value <= 126U) || byte_value == '\r' || byte_value == '\n') {
                buffer[written++] = static_cast<char>(byte_value);
            }
        }
    }

    buffer[written] = '\0';
    return written;
}

bool sc16_channel_present_wire(const uint8_t address, const uint8_t channel)
{
    // Use scratch register round-trip to validate channel access.
    const uint8_t pattern = static_cast<uint8_t>(0x5AU + channel);
    uint8_t observed = 0;

    if (!sc16_write_reg_wire(address, channel, SC16_REG_SPR, pattern)) {
        return false;
    }
    if (!sc16_read_reg_wire(address, channel, SC16_REG_SPR, observed)) {
        return false;
    }

    return observed == pattern;
}

bool sc16_channel_present_ex(const uint8_t address, const uint8_t channel)
{
    // Use scratch register round-trip to validate channel access.
    const uint8_t pattern = static_cast<uint8_t>(0x5AU + channel);
    uint8_t observed = 0;

    if (!sc16_write_reg_ex(address, channel, SC16_REG_SPR, pattern)) {
        return false;
    }
    if (!sc16_read_reg_ex(address, channel, SC16_REG_SPR, observed)) {
        return false;
    }

    return observed == pattern;
}

GravityProbeResult probe_gravity_dual_uart_wire(const uint8_t address, const char* path_tag)
{
    GravityProbeResult result;
    result.ran = true;

    M5_LOGD("[I2C][%s] active probe for Gravity Dual UART bridge @0x%02X", path_tag, address);

    bool nb_iot_detected = false;
    bool c6l_activity_detected = false;

    for (uint8_t channel = SC16_CH_A; channel <= SC16_CH_B; ++channel) {
        if (!sc16_channel_present_wire(address, channel)) {
            M5_LOGD("[I2C][%s][DUAL_UART:CH%u] channel access failed", path_tag, channel);
            continue;
        }

        sc16_write_reg_wire(address, channel, SC16_REG_LCR, 0x03U);  // 8N1
        sc16_flush_rx_wire(address, channel);

        const bool sent = sc16_send_bytes_wire(address, channel, "AT\r\n");
        char response[128] = {0};
        const size_t bytes = sent ? sc16_read_text_wire(address, channel, response, sizeof(response), 500U) : 0U;

        const bool got_ok = std::strstr(response, "OK") != nullptr;
        const bool got_any = bytes > 0U;

        if (got_ok) {
            nb_iot_detected = true;
            M5_LOGD("[I2C][%s][DUAL_UART:CH%u] NB-IoT2 probe success (AT->OK)", path_tag, channel);
        } else if (got_any) {
            c6l_activity_detected = true;
            M5_LOGD("[I2C][%s][DUAL_UART:CH%u] serial activity detected (possible C6L)", path_tag, channel);
        } else {
            M5_LOGD("[I2C][%s][DUAL_UART:CH%u] no UART response", path_tag, channel);
        }
    }

    if (!nb_iot_detected) {
        M5_LOGD("[I2C][%s][DUAL_UART] NB-IoT2 not confirmed by AT probe", path_tag);
    }
    if (!c6l_activity_detected) {
        M5_LOGD("[I2C][%s][DUAL_UART] C6L activity not observed during probe window", path_tag);
    }

    result.nb_iot2_confirmed = nb_iot_detected;
    result.c6l_activity_detected = c6l_activity_detected;
    return result;
}

GravityProbeResult probe_gravity_dual_uart_ex(const uint8_t address, const char* path_tag)
{
    GravityProbeResult result;
    result.ran = true;

    M5_LOGD("[I2C][%s] active probe for Gravity Dual UART bridge @0x%02X", path_tag, address);

    bool nb_iot_detected = false;
    bool c6l_activity_detected = false;

    for (uint8_t channel = SC16_CH_A; channel <= SC16_CH_B; ++channel) {
        if (!sc16_channel_present_ex(address, channel)) {
            M5_LOGD("[I2C][%s][DUAL_UART:CH%u] channel access failed", path_tag, channel);
            continue;
        }

        sc16_write_reg_ex(address, channel, SC16_REG_LCR, 0x03U);  // 8N1
        sc16_flush_rx_ex(address, channel);

        const bool sent = sc16_send_bytes_ex(address, channel, "AT\r\n");
        char response[128] = {0};
        const size_t bytes = sent ? sc16_read_text_ex(address, channel, response, sizeof(response), 500U) : 0U;

        const bool got_ok = std::strstr(response, "OK") != nullptr;
        const bool got_any = bytes > 0U;

        if (got_ok) {
            nb_iot_detected = true;
            M5_LOGD("[I2C][%s][DUAL_UART:CH%u] NB-IoT2 probe success (AT->OK)", path_tag, channel);
        } else if (got_any) {
            c6l_activity_detected = true;
            M5_LOGD("[I2C][%s][DUAL_UART:CH%u] serial activity detected (possible C6L)", path_tag, channel);
        } else {
            M5_LOGD("[I2C][%s][DUAL_UART:CH%u] no UART response", path_tag, channel);
        }
    }

    if (!nb_iot_detected) {
        M5_LOGD("[I2C][%s][DUAL_UART] NB-IoT2 not confirmed by AT probe", path_tag);
    }
    if (!c6l_activity_detected) {
        M5_LOGD("[I2C][%s][DUAL_UART] C6L activity not observed during probe window", path_tag);
    }

    result.nb_iot2_confirmed = nb_iot_detected;
    result.c6l_activity_detected = c6l_activity_detected;
    return result;
}

}  // namespace

/**
 * @brief Enumerates I2C devices and fills a structured topology report.
 *
 * The scan covers:
 * - direct Wire,
 * - Wire + PAHub channels,
 * - direct Ex_I2C,
 * - Ex_I2C + PAHub channels.
 *
 * @param i2c_report Output report structure populated with discoveries.
 * @return Number of discovered device entries across all scanned segments.
 */
uint16_t enumerate_i2c_units(I2CEnumerationReport& i2c_report)
{
    uint16_t discovered_entries = 0;
    uint16_t gravity_bridge_hits = 0;
    bool gravity_candidate_on_wire = false;
    bool gravity_candidate_on_wire_pahub = false;
    bool gravity_candidate_on_ex = false;
    bool gravity_candidate_on_ex_pahub = false;

    bool pahub_found = false;
    bool distance_found = false;
    bool weight_found = false;
    bool rtc_addr_found = false;
    bool distance_on_wire = false;
    bool distance_on_wire_pahub = false;
    int8_t distance_pahub_channel = -1;
    bool distance_on_ex = false;
    bool distance_on_ex_pahub = false;
    int8_t distance_ex_pahub_channel = -1;
    bool weight_on_wire = false;
    bool weight_on_wire_pahub = false;
    int8_t weight_pahub_channel = -1;
    bool gravity_probe_ran = false;
    bool nb_iot2_confirmed = false;
    bool c6l_activity_detected = false;

    BusScanSnapshot wire_snapshot;
    BusScanSnapshot ex_snapshot;

    M5_LOGI("[I2C] starting bus enumeration (order: WIRE, WIRE/PAHUB, EX, EX/PAHUB)");

    auto pin_num_sda = M5.getPin(m5::pin_name_t::port_a_sda);
    auto pin_num_scl = M5.getPin(m5::pin_name_t::port_a_scl);
    Wire.end();
    Wire.begin(pin_num_sda, pin_num_scl, I2C_SCAN_SPEED);
    M5.Ex_I2C.begin();

    scan_direct_wire(wire_snapshot);
    scan_wire_pahub(wire_snapshot);
    scan_direct_ex(ex_snapshot);
    scan_ex_pahub(ex_snapshot);

    auto process_discovery = [&](const uint8_t address,
                                const bool on_ex_bus,
                                const bool via_pahub,
                                const int8_t pahub_channel) {
        ++discovered_entries;

        if (address == DISTANCE_I2C_ADDRESS) {
            distance_found = true;
            if (on_ex_bus) {
                if (via_pahub) {
                    distance_on_ex_pahub = true;
                    if (distance_ex_pahub_channel < 0) {
                        distance_ex_pahub_channel = pahub_channel;
                    }
                } else {
                    distance_on_ex = true;
                }
            } else if (via_pahub) {
                distance_on_wire_pahub = true;
                if (distance_pahub_channel < 0) {
                    distance_pahub_channel = pahub_channel;
                }
            } else {
                distance_on_wire = true;
            }
        }

        if (address == WEIGHT_I2C_ADDRESS) {
            weight_found = true;
            if (!on_ex_bus) {
                if (via_pahub) {
                    weight_on_wire_pahub = true;
                    if (weight_pahub_channel < 0) {
                        weight_pahub_channel = pahub_channel;
                    }
                } else {
                    weight_on_wire = true;
                }
            }
        }

        if (address == RTC_ADDR_PCF8563 || address == RTC_ADDR_RX8130 || address == RTC_ADDR_POWERHUB) {
            rtc_addr_found = true;
        }

        if (!is_gravity_dual_uart_candidate(address)) {
            return;
        }

        ++gravity_bridge_hits;
        if (on_ex_bus) {
            if (via_pahub) {
                gravity_candidate_on_ex_pahub = true;
            } else {
                gravity_candidate_on_ex = true;
            }
        } else if (via_pahub) {
            gravity_candidate_on_wire_pahub = true;
        } else {
            gravity_candidate_on_wire = true;
        }

        const char* probe_tag = on_ex_bus
            ? (via_pahub ? "EX/PAHUB" : "EX")
            : (via_pahub ? "WIRE/PAHUB" : "WIRE");

        const GravityProbeResult probe = on_ex_bus
            ? probe_gravity_dual_uart_ex(address, probe_tag)
            : probe_gravity_dual_uart_wire(address, probe_tag);

        gravity_probe_ran = gravity_probe_ran || probe.ran;
        nb_iot2_confirmed = nb_iot2_confirmed || probe.nb_iot2_confirmed;
        c6l_activity_detected = c6l_activity_detected || probe.c6l_activity_detected;
    };

    for (uint8_t i = 0; i < wire_snapshot.direct_count; ++i) {
        process_discovery(wire_snapshot.direct[i], false, false, -1);
    }
    for (uint8_t channel = 0; channel < PAHUB_CHANNEL_COUNT; ++channel) {
        for (uint8_t i = 0; i < wire_snapshot.pahub_device_counts[channel]; ++i) {
            process_discovery(wire_snapshot.pahub_devices[channel][i], false, true, static_cast<int8_t>(channel));
        }
    }
    for (uint8_t i = 0; i < ex_snapshot.direct_count; ++i) {
        process_discovery(ex_snapshot.direct[i], true, false, -1);
    }
    for (uint8_t channel = 0; channel < PAHUB_CHANNEL_COUNT; ++channel) {
        for (uint8_t i = 0; i < ex_snapshot.pahub_device_counts[channel]; ++i) {
            process_discovery(ex_snapshot.pahub_devices[channel][i], true, true, static_cast<int8_t>(channel));
        }
    }

    pahub_found = wire_snapshot.pahub_found || ex_snapshot.pahub_found;

    M5_LOGI("[I2C][REPORT] ========================================");
    M5_LOGI("[I2C][REPORT] Hierarchical connection report");
    log_bus_hierarchy("WIRE bus", wire_snapshot);
    log_bus_hierarchy("EX bus", ex_snapshot);

    if (gravity_bridge_hits == 0) {
        M5_LOGI("[I2C][REPORT] Gravity bridge: not detected");
    } else {
        M5_LOGI("[I2C][REPORT] Gravity bridge candidates: %u", gravity_bridge_hits);
        if (gravity_candidate_on_wire) {
            M5_LOGI("[I2C][REPORT]   path: WIRE");
        }
        if (gravity_candidate_on_wire_pahub) {
            M5_LOGI("[I2C][REPORT]   path: WIRE/PAHUB");
        }
        if (gravity_candidate_on_ex) {
            M5_LOGI("[I2C][REPORT]   path: EX");
        }
        if (gravity_candidate_on_ex_pahub) {
            M5_LOGI("[I2C][REPORT]   path: EX/PAHUB");
        }

        M5_LOGI("[I2C][REPORT]   downstream NB-IoT2: %s", nb_iot2_confirmed ? "detected" : "not detected");
        M5_LOGI("[I2C][REPORT]   downstream C6L: %s", c6l_activity_detected ? "activity detected" : "not detected");
    }

    const bool imu_enabled = M5.Imu.isEnabled();
    M5_LOGI("[I2C][REPORT] Internal sensors");
    M5_LOGI("[I2C][REPORT]   IMU (accelerometer/gyroscope): %s", imu_enabled ? "enabled" : "not detected");
    M5_LOGI("[I2C][REPORT]   Tilt: %s", imu_enabled ? "available (computed from IMU)" : "unavailable (IMU not enabled)");

    M5_LOGI("[I2C][REPORT] totals: entries=%u pahub=%s distance=%s weight=%s rtc=%s",
            discovered_entries,
            pahub_found ? "yes" : "no",
            distance_found ? "yes" : "no",
            weight_found ? "yes" : "no",
            rtc_addr_found ? "yes" : "no");
    M5_LOGI("[I2C][REPORT] ========================================");

    i2c_report.discovered_entries = discovered_entries;
    i2c_report.gravity_bridge_hits = gravity_bridge_hits;
    i2c_report.pahub_found = pahub_found;
    i2c_report.distance_found = distance_found;
    i2c_report.weight_found = weight_found;
    i2c_report.rtc_addr_found = rtc_addr_found;
    i2c_report.distance_on_wire = distance_on_wire;
    i2c_report.distance_on_wire_pahub = distance_on_wire_pahub;
    i2c_report.distance_pahub_channel = distance_pahub_channel;
    i2c_report.distance_on_ex = distance_on_ex;
    i2c_report.distance_on_ex_pahub = distance_on_ex_pahub;
    i2c_report.distance_ex_pahub_channel = distance_ex_pahub_channel;
    i2c_report.weight_on_wire = weight_on_wire;
    i2c_report.weight_on_wire_pahub = weight_on_wire_pahub;
    i2c_report.weight_pahub_channel = weight_pahub_channel;
    i2c_report.gravity_bridge_detected = gravity_bridge_hits > 0;
    i2c_report.gravity_on_wire = gravity_candidate_on_wire;
    i2c_report.gravity_on_wire_pahub = gravity_candidate_on_wire_pahub;
    i2c_report.gravity_on_ex = gravity_candidate_on_ex;
    i2c_report.gravity_on_ex_pahub = gravity_candidate_on_ex_pahub;
    i2c_report.gravity_probe_ran = gravity_probe_ran;
    i2c_report.nb_iot2_confirmed = nb_iot2_confirmed;
    i2c_report.c6l_activity_detected = c6l_activity_detected;

    return discovered_entries;
}

/**
 * @brief Resolves the Wire route for one target I2C address.
 *
 * Detection order:
 * 1. Direct Wire probe.
 * 2. PAHub presence probe.
 * 3. PAHub channel scan (0..7).
 *
 * @param target_address 7-bit I2C address to locate.
 * @param result Output with resolved bus mode and PAHub channel.
 * @return true when target is reachable on direct Wire or via PAHub.
 */
bool detect_wire_bus_path(const uint8_t target_address, I2CBusPathResult& result)
{
    result.bus_mode = I2CBusMode::Unset;
    result.pahub_channel = -1;

    // First try direct Wire routing without any PAHub multiplexing.
    if (wire_device_exists(target_address)) {
        result.bus_mode = I2CBusMode::WireDirect;
        return true;
    }

    // If PAHub is absent, no multiplexed route can be resolved.
    if (!wire_device_exists(PAHUB_ADDRESS)) {
        return false;
    }

    // Scan all PAHub channels for the requested target address.
    for (uint8_t channel = 0; channel < PAHUB_CHANNEL_COUNT; ++channel) {
        if (!wire_pahub_select_channel(channel)) {
            continue;
        }

        if (wire_device_exists(target_address)) {
            wire_pahub_disable_all_channels();
            result.bus_mode = I2CBusMode::WirePaHub;
            result.pahub_channel = static_cast<int8_t>(channel);
            return true;
        }
    }

    wire_pahub_disable_all_channels();
    return false;
}
