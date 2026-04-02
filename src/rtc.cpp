/**
 * @file rtc.cpp
 * @brief Implémente la gestion de l'horloge temps réel interne (RTC), la synchronisation NTP/système, et la publication MQTT.
 */

#include "rtc.h"

#include <M5Unified.h>
#include <WiFi.h>

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>

#include "config_store.h"
#include "data_model.h"
#include "interfaces.h"
#include "mqtt.h"

bool RTC::isSystemTimeValid(const time_t t)
{
    return t >= kMinValidEpoch;
}

bool RTC::datetimeToEpoch(const DateTime& dt, time_t& outEpoch)
{
    struct tm tmUtc{};
    tmUtc.tm_year = dt.year - 1900;
    tmUtc.tm_mon = dt.month - 1;
    tmUtc.tm_mday = dt.day;
    tmUtc.tm_hour = dt.hour;
    tmUtc.tm_min = dt.minute;
    tmUtc.tm_sec = dt.second;
    tmUtc.tm_isdst = 0;

    const String oldTz = String(getenv("TZ") ? getenv("TZ") : "");
    setenv("TZ", "UTC0", 1);
    tzset();
    const time_t epoch = mktime(&tmUtc);
    if (!oldTz.isEmpty()) {
        setenv("TZ", oldTz.c_str(), 1);
    } else {
        unsetenv("TZ");
    }
    tzset();

    if (!isSystemTimeValid(epoch)) {
        return false;
    }

    outEpoch = epoch;
    return true;
}

bool RTC::epochToDateTime(const time_t epoch, DateTime& out)
{
    struct tm tmUtc;
    if (gmtime_r(&epoch, &tmUtc) == nullptr) {
        return false;
    }

    out.year = tmUtc.tm_year + 1900;
    out.month = tmUtc.tm_mon + 1;
    out.day = tmUtc.tm_mday;
    out.hour = tmUtc.tm_hour;
    out.minute = tmUtc.tm_min;
    out.second = tmUtc.tm_sec;
    return true;
}

bool RTC::epochToLocalIso(const time_t epoch, char* out, const size_t outLen)
{
    if (out == nullptr || outLen == 0) {
        return false;
    }

    const String tzSpec = timezoneToTzString(timezoneConfigValue());
    setenv("TZ", tzSpec.c_str(), 1);
    tzset();

    struct tm tmLocal;
    if (localtime_r(&epoch, &tmLocal) == nullptr) {
        return false;
    }

    char baseBuf[32] = {0};
    char zoneBuf[8] = {0};
    if (strftime(baseBuf, sizeof(baseBuf), "%Y-%m-%dT%H:%M:%S", &tmLocal) == 0) {
        return false;
    }
    if (strftime(zoneBuf, sizeof(zoneBuf), "%z", &tmLocal) == 0) {
        return false;
    }

    if (strlen(zoneBuf) == 5) {
        snprintf(out, outLen, "%s%c%c%c:%c%c",
                 baseBuf,
                 zoneBuf[0], zoneBuf[1], zoneBuf[2], zoneBuf[3], zoneBuf[4]);
    } else {
        snprintf(out, outLen, "%s%s", baseBuf, zoneBuf);
    }

    return true;
}

void RTC::formatIsoUtc(const DateTime& dt, char* out, const size_t outLen)
{
    if (out == nullptr || outLen == 0) {
        return;
    }

    snprintf(out,
             outLen,
             "%04d-%02d-%02dT%02d:%02d:%02dZ",
             dt.year,
             dt.month,
             dt.day,
             dt.hour,
             dt.minute,
             dt.second);
}

bool RTC::readDateTimeFromInternalApi(DateTime& out)
{
    if (!M5.Rtc.isEnabled()) {
        return false;
    }

    m5::rtc_datetime_t dt{};
    if (!M5.Rtc.getDateTime(&dt)) {
        return false;
    }

    out.year = dt.date.year;
    out.month = dt.date.month;
    out.day = dt.date.date;
    out.hour = dt.time.hours;
    out.minute = dt.time.minutes;
    out.second = dt.time.seconds;
    return true;
}

bool RTC::writeDateTimeToInternalApi(const DateTime& dt)
{
    if (!M5.Rtc.isEnabled()) {
        return false;
    }

    m5::rtc_date_t date{};
    date.year = static_cast<int16_t>(dt.year);
    date.month = static_cast<int8_t>(dt.month);
    date.date = static_cast<int8_t>(dt.day);
    date.weekDay = -1;

    m5::rtc_time_t time{};
    time.hours = static_cast<int8_t>(dt.hour);
    time.minutes = static_cast<int8_t>(dt.minute);
    time.seconds = static_cast<int8_t>(dt.second);

    M5.Rtc.setDateTime(&date, &time);

    DateTime verify{};
    return readDateTimeFromInternalApi(verify);
}

const char* RTC::syncSourceToString(const SyncSource src)
{
    switch (src) {
    case SyncSource::Ntp:
        return "ntp";
    case SyncSource::Rtc:
        return "rtc";
    case SyncSource::None:
    default:
        return "none";
    }
}

String RTC::timezoneConfigValue()
{
    String timezone = CONFIG.rtc_timezone;
    timezone.trim();
    if (timezone.isEmpty()) {
        return "Europe/Paris";
    }
    return timezone;
}

String RTC::timezoneToTzString(const String& value)
{
    if (value.equalsIgnoreCase("Europe/Paris")) {
        return "CET-1CEST,M3.5.0/2,M10.5.0/3";
    }
    if (value.equalsIgnoreCase("UTC") || value.equalsIgnoreCase("Etc/UTC")) {
        return "UTC0";
    }
    return value;
}

void RTC::publishTimezone()
{
    const String timezone = timezoneConfigValue();
    sf_mqtt::publish("smartfranklin/rtc/timezone", timezone.c_str(), 1, true);
}

void RTC::publishSyncSource(const SyncSource src)
{
    sf_mqtt::publish("smartfranklin/rtc/sync_source", syncSourceToString(src), 1, true);

    std::lock_guard<std::mutex> lock(DATA_MUTEX);
    DATA.rtc_sync_source = syncSourceToString(src);
}

void RTC::publishRtcTime(const DateTime& dt)
{
    char timeBuf[32] = {0};
    time_t epoch = 0;
    if (!datetimeToEpoch(dt, epoch) || !epochToLocalIso(epoch, timeBuf, sizeof(timeBuf))) {
        formatIsoUtc(dt, timeBuf, sizeof(timeBuf));
    }

    sf_mqtt::publish("smartfranklin/rtc/time", timeBuf, 1, true);

    M5_LOGI("[RTC] %s", timeBuf);

    std::lock_guard<std::mutex> lock(DATA_MUTEX);
    DATA.rtc_time = timeBuf;
}

bool RTC::syncSystemFromNtp()
{
    if (WiFi.status() != WL_CONNECTED) {
        return false;
    }

    const String tzSpec = timezoneToTzString(timezoneConfigValue());
    configTzTime(tzSpec.c_str(), "pool.ntp.org", "time.google.com", "time.nist.gov");

    struct tm nowTm;
    if (!getLocalTime(&nowTm, kNtpTimeoutMs)) {
        return false;
    }

    const time_t nowEpoch = time(nullptr);
    return isSystemTimeValid(nowEpoch);
}

bool RTC::readDateTime(DateTime& out)
{
    return readDateTimeFromInternalApi(out);
}

bool RTC::writeDateTime(const DateTime& dt)
{
    return writeDateTimeToInternalApi(dt);
}

bool RTC::syncSystemFromRtcHardware()
{
    DateTime rtcDateTime;
    if (!readDateTime(rtcDateTime)) {
        return false;
    }

    time_t epoch = 0;
    if (!datetimeToEpoch(rtcDateTime, epoch)) {
        return false;
    }

    struct timeval tv;
    tv.tv_sec = epoch;
    tv.tv_usec = 0;
    return settimeofday(&tv, nullptr) == 0;
}

bool RTC::syncRtcHardwareFromSystem()
{
    const time_t nowEpoch = time(nullptr);
    if (!isSystemTimeValid(nowEpoch)) {
        return false;
    }

    DateTime nowDt;
    if (!epochToDateTime(nowEpoch, nowDt)) {
        return false;
    }

    return writeDateTime(nowDt);
}

RTC RTC_TASK;

bool RTC::isInitialized() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_initialized;
}

bool RTC::init()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    m_initialized = false;
    m_lastProcessMs = 0;
    m_lastPublishMs = 0;
    m_lastRtcWriteMs = 0;
    m_lastSyncSource = SyncSource::None;

    DateTime internalDt{};
    if (!readDateTimeFromInternalApi(internalDt)) {
        M5_LOGW("[RTC] internal RTC API unavailable");
        return false;
    }

    m_initialized = true;

    publishTimezone();
    publishSyncSource(m_lastSyncSource);
    M5_LOGI("[RTC] initialized with internal RTC");
    return true;
}

void RTC::process()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_initialized) {
        return;
    }

    const uint32_t nowMs = millis();
    if ((nowMs - m_lastProcessMs) < kProcessPeriodMs) {
        return;
    }
    m_lastProcessMs = nowMs;

    SyncSource syncSource = SyncSource::None;

    if (syncSystemFromNtp()) {
        syncSource = SyncSource::Ntp;
        if (syncRtcHardwareFromSystem()) {
            m_lastRtcWriteMs = nowMs;
        }
    } else {
        const time_t nowEpoch = time(nullptr);
        if (!isSystemTimeValid(nowEpoch) && syncSystemFromRtcHardware()) {
            syncSource = SyncSource::Rtc;
        }
    }

    if (syncSource != SyncSource::None) {
        m_lastSyncSource = syncSource;
        publishSyncSource(syncSource);
    }

    if (syncSource != SyncSource::Ntp && (nowMs - m_lastRtcWriteMs) >= kWritePeriodMs) {
        if (syncRtcHardwareFromSystem()) {
            m_lastRtcWriteMs = nowMs;
        }
    }

    if ((nowMs - m_lastPublishMs) >= kPublishPeriodMs) {
        publishTimezone();

        DateTime rtcDt;
        if (readDateTime(rtcDt)) {
            publishRtcTime(rtcDt);
        }
        m_lastPublishMs = nowMs;
    }
}

/**
 * @brief FreeRTOS task for RTC acquisition and NTP synchronization.
 */
void taskRtc(void* pv)
{
    (void)pv;
    M5_LOGI("[RTC] Task started");

    bool initialized = false;
    uint32_t nextInitAttemptMs = 0;

    auto isRetryDue = [](uint32_t nowMs, uint32_t nextAttemptMs) {
        return static_cast<int32_t>(nowMs - nextAttemptMs) >= 0;
    };

    auto scheduleRetry = [](uint32_t& nextAttemptMs, uint32_t nowMs) {
        nextAttemptMs = nowMs + sf_i2c::kI2cInitRetryMs;
    };

    for (;;) {
        const uint32_t nowMs = millis();

        if (!initialized && isRetryDue(nowMs, nextInitAttemptMs)) {
            initialized = RTC_TASK.init();
            if (!initialized) {
                M5_LOGW("[RTC] Init failed");
                scheduleRetry(nextInitAttemptMs, nowMs);
            }
        }

        if (initialized) {
            RTC_TASK.process();
        }

        const int loopMs = (CONFIG.task_i2c_loop_ms > 0) ? CONFIG.task_i2c_loop_ms : 1000;
        vTaskDelay(pdMS_TO_TICKS(loopMs));
    }
}
