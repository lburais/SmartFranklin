/*
 * ============================================================================
 * RTC Task Module - SmartFranklin
 * ============================================================================
 * 
 * File:        task_rtc.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: FreeRTOS task for Real-Time Clock (RTC) management and time
 *              synchronization. Reads current date/time from M5Stack RTC,
 *              updates global data model, and publishes timestamp to MQTT.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   SmartFranklin maintains accurate timekeeping using the internal M5 RTC
 *   when available, with automatic fallback to an external RTC on Port A.
 *   The task can discover an external RTC behind a PAHub channel, then
 *   continuously reads date/time, formats it as an ISO string, updates the
 *   global data model, and publishes timestamps to MQTT.
 * 
 * RTC Hardware:
 *   - BM8563 RTC chip integrated in M5Stack Core devices
 *   - Battery-backed timekeeping (CR1220 coin cell battery)
 *   - I2C interface communication
 *   - Accuracy: ±20ppm (approximately 1 minute per month)
 *   - Time format: Gregorian calendar with leap year support
 *   - Alarm functionality available (not used in this task)
 * 
 * Time Format:
 *   - String Format: "YYYY-MM-DDTHH:MM:SSZ" (ISO 8601 UTC)
 *   - Components: 4-digit year, 2-digit month/day, 2-digit hour/minute/second
 *   - Zero-padding: All numeric components padded with leading zeros
 *   - Buffer Size: 32 characters (sufficient for format + null terminator)
 *   - Thread Safety: String formatting completed before global update
 * 
 * Data Flow:
 *   1. Read current date/time from RTC hardware
 *   2. Format as string using snprintf for consistency
 *   3. Update global DATA.rtc_time with thread-safe mutex lock
 *   4. Publish timestamp to MQTT topic "smartfranklin/rtc/time"
 *   5. Log timestamp to serial console for debugging
 * 
 * MQTT Publishing:
 *   - Topic: "smartfranklin/rtc/time"
 *   - Payload: Timestamp string (e.g., "2026-03-05T14:30:25Z")
 *   - QoS: Default (0, at most once delivery)
 *   - Retention: Not retained (current time only)
 *   - Frequency: Every PERIOD_RTC milliseconds (continuous updates)
 * 
 * Task Configuration:
 *   - Update Period: PERIOD_RTC milliseconds (defined in tasks.h)
 *   - Priority: tskIDLE_PRIORITY + 1 (standard priority)
 *   - Stack Size: 4096 bytes (sufficient for RTC operations)
 *   - Core Affinity: No restriction (runs on any core)
 * 
 * Error Handling:
 *   - RTC Not Found: Task enters infinite loop with error logging
 *   - I2C Communication: Handled by M5Unified library
 *   - Formatting Errors: snprintf provides bounds checking
 *   - Mutex Errors: std::lock_guard provides exception safety
 *   - MQTT Failures: Publishing fails gracefully, task continues
 * 
 * Performance Considerations:
 *   - CPU Usage: Low (RTC reads are fast I2C operations)
 *   - Memory Usage: Minimal (local buffer and string operations)
 *   - I2C Traffic: Brief communication during each read cycle
 *   - Update Frequency: Configurable via PERIOD_RTC (balance accuracy vs. overhead)
 *   - Power Impact: Minimal additional power consumption
 * 
 * Dependencies:
 *   - M5Unified.h (M5Stack unified sensor interface)
 *   - M5UnitUnified.h (M5Stack unit interface - not directly used)
 *   - M5Utility.h (M5Stack utility functions)
 *   - tasks.h (Task definitions and PERIOD_RTC constant)
 *   - data_model.h (Global DATA structure and mutex)
 *   - pahub_channels.h (PAHub address/channel helpers for external RTC fallback)
 *   - mqtt_layer.h (MQTT publishing interface)
 * 
 * Limitations:
 *   - No NTP Synchronization: RTC relies on manual setting or battery backup
 *   - Fixed Format: ISO-like string format (not configurable)
 *   - No Timezone Support: UTC time only (no timezone conversion)
 *   - Battery Dependency: Time lost if CR1220 battery depleted
 *   - No Alarm Usage: RTC alarm features not implemented
 *   - Auto-discovery Scope: Fallback scans PAHub channels for supported RTC addresses
 * 
 * Best Practices:
 *   - Set RTC time during device commissioning
 *   - Monitor battery voltage to prevent time loss
 *   - Use appropriate update intervals for application needs
 *   - Consider NTP integration for network-connected deployments
 *   - Validate time accuracy periodically
 *   - Use RTC for timestamping sensor data
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

#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5Utility.h>

#include "tasks.h"
#include "data_model.h"
#include "pahub_channels.h"
#include "mqtt_layer.h"

namespace {
constexpr uint8_t RTC_ADDR_PCF8563 = 0x51;
constexpr uint8_t RTC_ADDR_RX8130 = 0x32;
constexpr uint8_t RTC_ADDR_POWERHUB = 0x50;

bool g_use_external_rtc = false;
int g_external_rtc_pahub_channel = -1;

bool ex_i2c_device_exists(const uint8_t address)
{
    return M5.Ex_I2C.scanID(address, 400000);
}

bool pahub_select_channel(const uint8_t channel)
{
    if (!M5.Ex_I2C.start(PAHUB_ADDRESS, false, 400000)) {
        return false;
    }
    const bool write_ok = M5.Ex_I2C.write(static_cast<uint8_t>(1U << channel));
    const bool stop_ok = M5.Ex_I2C.stop();
    return write_ok && stop_ok;
}

void pahub_disable_all_channels()
{
    if (!M5.Ex_I2C.start(PAHUB_ADDRESS, false, 400000)) {
        return;
    }
    M5.Ex_I2C.write(static_cast<uint8_t>(0x00));
    M5.Ex_I2C.stop();
}

bool rtc_address_visible_on_bus()
{
    return ex_i2c_device_exists(RTC_ADDR_PCF8563)
        || ex_i2c_device_exists(RTC_ADDR_RX8130)
        || ex_i2c_device_exists(RTC_ADDR_POWERHUB);
}

bool enable_external_rtc_via_pahub()
{
    if (!ex_i2c_device_exists(PAHUB_ADDRESS)) {
        return false;
    }

    for (uint8_t channel = 0; channel < 8; ++channel) {
        if (!pahub_select_channel(channel)) {
            continue;
        }

        if (!rtc_address_visible_on_bus()) {
            continue;
        }

        if (M5.Rtc.begin(&M5.Ex_I2C)) {
            g_use_external_rtc = true;
            g_external_rtc_pahub_channel = static_cast<int>(channel);
            M5_LOGI("[RTC] external RTC found via PAHub channel %d", g_external_rtc_pahub_channel);
            return true;
        }
    }

    pahub_disable_all_channels();
    return false;
}
}  // namespace

// ============================================================================
// RTC Initialization Function
// ============================================================================

/**
 * @brief Initializes the Real-Time Clock hardware and validates functionality.
 * 
 * Checks if the M5Stack RTC module is available and enabled. If the RTC
 * is not found or not functioning, logs an error and enters an infinite
 * loop to prevent task continuation with invalid time data.
 * 
 * Initialization Process:
 *   1. Query M5Stack RTC interface for availability
 *   2. Check M5.Rtc.isEnabled() for hardware status
 *   3. Log success or failure with appropriate message level
 *   4. Enter error loop on failure to halt task execution
 * 
 * Hardware Validation:
 *   - I2C Communication: Verified by M5Unified library
 *   - RTC Chip Presence: BM8563 detection and initialization
 *   - Battery Backup: Assumed functional (not explicitly checked)
 *   - Time Validity: Not checked (assumes previously set time)
 * 
 * Error Handling:
 *   - RTC Disabled: Comprehensive error logging with M5_LOGE
 *   - Task Termination: Infinite loop prevents invalid time operations
 *   - Recovery: Manual hardware check required to resolve
 *   - Logging: Debug information available for troubleshooting
 * 
 * Success Indicators:
 *   - "[RTC] found." logged to serial console
 *   - Task continues to measurement loop
 *   - RTC time available for system use
 * 
 * @return void
 * 
 * @note This function blocks indefinitely on RTC initialization failure.
 *       Ensure M5Stack RTC is properly connected and battery is installed.
 *       RTC time should be set during device commissioning.
 * 
 * @see M5.Rtc.isEnabled() - RTC hardware availability check
 * @see M5.Rtc.getDateTime() - RTC time reading function
 */
static void rtc_setup()
{
    if (M5.Rtc.isEnabled()) {
        M5_LOGI("[RTC] found (internal).");
        return;
    }

    M5_LOGW("[RTC] internal RTC not found, searching external RTC...");
    M5.Ex_I2C.begin();

    bool found = enable_external_rtc_via_pahub();

    if (!found && M5.Rtc.begin(&M5.Ex_I2C)) {
        g_use_external_rtc = true;
        g_external_rtc_pahub_channel = -1;
        found = true;
        M5_LOGI("[RTC] external RTC found on direct Port A I2C");
    }

    if (!found) {
        M5_LOGE("[RTC] not found.");
        for (;;) {
            vTaskDelay(500);
        }
    }

    M5.Rtc.setSystemTimeFromRtc();
    M5.Rtc.disableIRQ();

    M5_LOGI("[RTC] found (external).");
}

// ============================================================================
// RTC Measurement Loop
// ============================================================================

/**
 * @brief Main measurement loop for reading and publishing RTC time.
 * 
 * Retrieves current date and time from the RTC, formats it as a string,
 * updates the global data model, publishes to MQTT, and logs the timestamp.
 * Designed to be called repeatedly from the FreeRTOS task loop.
 * 
 * Processing Steps:
 *   1. Read current date/time from RTC hardware
 *   2. Format as "YYYY-MM-DDTHH:MM:SSZ" string using snprintf
 *   3. Update global DATA.rtc_time with thread-safe mutex lock
 *   4. Publish timestamp to MQTT topic "smartfranklin/rtc/time"
 *   5. Log formatted timestamp to serial console
 * 
 * String Formatting:
 *   - Buffer Size: 32 characters (adequate for format)
 *   - Format Specifiers: %04d (year), %02d (month/day/hour/minute/second)
 *   - Zero Padding: Ensures consistent field widths
 *   - Null Termination: Automatic with snprintf bounds checking
 * 
 * Thread Safety:
 *   - DATA_MUTEX lock held during global data update
 *   - Prevents race conditions with other tasks reading DATA
 *   - Lock scope limited to data update only
 * 
 * MQTT Publishing:
 *   - Topic: "smartfranklin/rtc/time"
 *   - Payload: Formatted timestamp string
 *   - QoS: Default MQTT settings
 *   - Synchronous: Publishing completes within function call
 * 
 * Logging:
 *   - Serial Output: Formatted log message with M5.Log.printf()
 *   - Debug Level: Informational logging for monitoring
 *   - Format: "[RTC] time:YYYY-MM-DDTHH:MM:SSZ"
 * 
 * Performance:
 *   - Execution Time: < 10ms (I2C read + formatting + MQTT)
 *   - Memory Usage: Minimal (local buffer only)
 *   - I2C Traffic: Brief communication during getDateTime()
 *   - CPU Usage: Low (formatting is lightweight)
 * 
 * @return void
 * 
 * @note This function is designed to be called in a loop from the FreeRTOS task.
 *       It provides continuous time updates for system synchronization.
 *       MQTT publishing occurs synchronously within this function.
 * 
 * @see M5.Rtc.getDateTime() - RTC time reading
 * @see sf_mqtt::publish() - MQTT message publishing
 */
static void rtc_loop()
{
    if (g_use_external_rtc && g_external_rtc_pahub_channel >= 0) {
        if (!pahub_select_channel(static_cast<uint8_t>(g_external_rtc_pahub_channel))) {
            M5_LOGW("[RTC] failed to select PAHub channel %d", g_external_rtc_pahub_channel);
            return;
        }
    }

    m5::rtc_datetime_t dt{};
    const bool rtc_ok = M5.Rtc.getDateTime(&dt);

    if (g_use_external_rtc && g_external_rtc_pahub_channel >= 0) {
        pahub_disable_all_channels();
    }

    if (!rtc_ok) {
        M5_LOGW("[RTC] read failed");
        return;
    }

    char buf[32];

    snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02dZ",
                dt.date.year, dt.date.month, dt.date.date,
                dt.time.hours, dt.time.minutes, dt.time.seconds);

    // Update shared data model
    {
        std::lock_guard<std::mutex> lock(DATA_MUTEX);
        DATA.rtc_time = buf;
    }

    // Publish to MQTT
    sf_mqtt::publish("smartfranklin/rtc/time", std::string(String(buf).c_str()));

    // Log current value
    M5.Log.printf("[RTC] time:%s\r\n", buf);

}

// ============================================================================
// FreeRTOS Task Implementation
// ============================================================================

/**
 * @brief FreeRTOS task for RTC time management and publishing.
 * 
 * Main task function that initializes the RTC hardware and runs the
 * continuous time reading and publishing loop. Provides accurate
 * timestamping for the entire SmartFranklin system.
 * 
 * Task Lifecycle:
 *   1. Log task startup to serial console
 *   2. Call setup() for RTC hardware initialization
 *   3. Enter infinite measurement loop
 *   4. Call loop() for time reading and publishing
 *   5. Delay for PERIOD_RTC milliseconds
 *   6. Repeat time update cycle
 * 
 * Task Configuration:
 *   - Update Period: PERIOD_RTC milliseconds (defined in tasks.h)
 *   - Priority: tskIDLE_PRIORITY + 1 (standard priority)
 *   - Stack Size: 4096 bytes (sufficient for RTC operations)
 *   - Core Affinity: No restriction (runs on any core)
 * 
 * Error Recovery:
 *   - Initialization Failures: setup() enters infinite loop
 *   - Runtime Errors: Task continues running (RTC library handles)
 *   - I2C Failures: May cause time read errors but task survives
 *   - MQTT Failures: Publishing fails gracefully, task continues
 * 
 * Performance:
 *   - CPU Usage: Low (mostly sleeping in vTaskDelay)
 *   - Memory Usage: Fixed after initialization
 *   - I2C Activity: Brief bursts during time reads
 *   - MQTT Traffic: Small timestamp messages periodically
 * 
 * Integration:
 *   - Provides system-wide time synchronization
 *   - Enables timestamped sensor data logging
 *   - Supports MQTT time publishing for remote monitoring
 *   - Complements other timing-dependent tasks
 * 
 * @param pv - FreeRTOS task parameter (unused, nullptr)
 * 
 * @return void (task runs indefinitely)
 * 
 * @note Task function name is taskRtc and supports internal/external RTC sources.
 *       RTC time should be set during initial device setup.
 *       External fallback currently supports direct Port A or PAHub-routed RTC.
 * 
 * @see rtc_setup() - RTC hardware initialization
 * @see rtc_loop() - Time reading and publishing
 * @see PERIOD_RTC - Update interval configuration
 */
void taskRtc(void *pv)
{
    M5_LOGI("[RTC] Task started");

    rtc_setup();

    for (;;) {
        rtc_loop();
            vTaskDelay(pdMS_TO_TICKS(PERIOD_RTC));
    }

}