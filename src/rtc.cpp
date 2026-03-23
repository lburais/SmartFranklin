/*
 * SmartFranklin - RTC module implementation
 * SPDX-License-Identifier: MIT
 */

#include "rtc.h"

#include <M5Unified.h>
#include <WiFi.h>
#include <Wire.h>

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <sys/time.h>
#include <time.h>

#include "data_model.h"
#include "i2c.h"
#include "mqtt.h"

namespace {

constexpr uint32_t RTC_PROCESS_PERIOD_MS = 5000UL;
constexpr uint32_t RTC_PUBLISH_PERIOD_MS = 30000UL;
constexpr uint32_t RTC_WRITE_PERIOD_MS = 60000UL;
constexpr uint32_t RTC_NTP_TIMEOUT_MS = 1500UL;
constexpr time_t MIN_VALID_EPOCH = 1704067200;  // 2024-01-01T00:00:00Z

enum class SyncSource : uint8_t {
    None = 0,
    Ntp,
    Rtc,
};

struct DateTime {
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    int second = 0;
};

struct RtcState {
    mutable std::mutex mutex;

    bool initialized = false;
    RTC::Source source = RTC::Source::None;
    sf_i2c::Device::ChipKind chipKind = sf_i2c::Device::ChipKind::Unknown;
    bool isInternalRoute = false;
    uint8_t i2cAddress = 0x51;

    uint32_t lastProcessMs = 0;
    uint32_t lastPublishMs = 0;
    uint32_t lastRtcWriteMs = 0;
    SyncSource lastSyncSource = SyncSource::None;
};

RtcState RTC_STATE;

uint8_t toBcd(const int value)
{
    const int tens = value / 10;
    const int ones = value % 10;
    return static_cast<uint8_t>((tens << 4) | ones);
}

int fromBcd(const uint8_t value)
{
    return ((value >> 4) * 10) + (value & 0x0F);
}

bool isValidBcd(const uint8_t value, const int maxDec)
{
    const int high = (value >> 4) & 0x0F;
    const int low = value & 0x0F;
    if (high > 9 || low > 9) {
        return false;
    }
    return ((high * 10) + low) <= maxDec;
}

bool writeBurst(const RtcState& state, const uint8_t reg, const uint8_t* data, const size_t len)
{
    if (data == nullptr || len == 0) {
        return false;
    }

    if (state.isInternalRoute) {
        if (!M5.Ex_I2C.start(state.i2cAddress, false, Wire.getClock())) {
            return false;
        }

        if (!M5.Ex_I2C.write(reg)) {
            M5.Ex_I2C.stop();
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

    Wire.beginTransmission(state.i2cAddress);
    Wire.write(reg);
    for (size_t i = 0; i < len; ++i) {
        Wire.write(data[i]);
    }
    return Wire.endTransmission() == 0;
}

bool readBurst(const RtcState& state, const uint8_t reg, uint8_t* out, const size_t len)
{
    if (out == nullptr || len == 0) {
        return false;
    }

    if (state.isInternalRoute) {
        if (!M5.Ex_I2C.start(state.i2cAddress, false, Wire.getClock())) {
            return false;
        }

        if (!M5.Ex_I2C.write(reg) || !M5.Ex_I2C.stop()) {
            return false;
        }

        if (!M5.Ex_I2C.start(state.i2cAddress, true, Wire.getClock())) {
            return false;
        }

        for (size_t i = 0; i < len; ++i) {
            const bool lastNack = (i + 1U == len);
            if (!M5.Ex_I2C.read(&out[i], 1U, lastNack)) {
                M5.Ex_I2C.stop();
                return false;
            }
        }

        return M5.Ex_I2C.stop();
    }

    Wire.beginTransmission(state.i2cAddress);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    const size_t readCount = Wire.requestFrom(state.i2cAddress, static_cast<uint8_t>(len));
    if (readCount < len) {
        return false;
    }

    for (size_t i = 0; i < len; ++i) {
        out[i] = static_cast<uint8_t>(Wire.read());
    }

    return true;
}

bool isSystemTimeValid(const time_t t)
{
    return t >= MIN_VALID_EPOCH;
}

int64_t daysFromCivil(int year, unsigned month, unsigned day)
{
    year -= (month <= 2U) ? 1 : 0;
    const int era = (year >= 0 ? year : year - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(year - era * 400);
    const unsigned doy = (153U * (month + (month > 2U ? static_cast<unsigned>(-3) : 9U)) + 2U) / 5U + day - 1U;
    const unsigned doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;
    return static_cast<int64_t>(era) * 146097LL + static_cast<int64_t>(doe) - 719468LL;
}

bool datetimeToEpoch(const DateTime& dt, time_t& outEpoch)
{
    const int64_t days = daysFromCivil(dt.year, static_cast<unsigned>(dt.month), static_cast<unsigned>(dt.day));
    const int64_t seconds = days * 86400LL +
                            static_cast<int64_t>(dt.hour) * 3600LL +
                            static_cast<int64_t>(dt.minute) * 60LL +
                            static_cast<int64_t>(dt.second);
    const time_t epoch = static_cast<time_t>(seconds);
    if (!isSystemTimeValid(epoch)) {
        return false;
    }

    outEpoch = epoch;
    return true;
}

bool epochToDateTime(const time_t epoch, DateTime& out)
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

bool parseIsoUtc(const String& value, DateTime& out)
{
    if (value.length() < 19) {
        return false;
    }

    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    int second = 0;

    const int parsed = sscanf(value.c_str(),
                              "%d-%d-%dT%d:%d:%d",
                              &year,
                              &month,
                              &day,
                              &hour,
                              &minute,
                              &second);

    if (parsed != 6) {
        return false;
    }

    if (year < 2000 || month < 1 || month > 12 || day < 1 || day > 31 ||
        hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 59) {
        return false;
    }

    out.year = year;
    out.month = month;
    out.day = day;
    out.hour = hour;
    out.minute = minute;
    out.second = second;
    return true;
}

void formatIsoUtc(const DateTime& dt, char* out, const size_t outLen)
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

bool readDateTimeFromInternalApi(DateTime& out)
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

bool writeDateTimeToInternalApi(const DateTime& dt)
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

    // M5Unified setDateTime() is void; verify by reading back one sample.
    DateTime verify{};
    return readDateTimeFromInternalApi(verify);
}

const char* syncSourceToString(const SyncSource source)
{
    switch (source) {
    case SyncSource::Ntp:
        return "ntp";
    case SyncSource::Rtc:
        return "rtc";
    case SyncSource::None:
    default:
        return "none";
    }
}

void publishSource(const RTC::Source source)
{
    sf_mqtt::publish("smartfranklin/rtc/source", RTC::sourceToString(source), 1, true);
}

void publishSyncSource(const SyncSource source)
{
    sf_mqtt::publish("smartfranklin/rtc/sync_source", syncSourceToString(source), 1, true);

    std::lock_guard<std::mutex> lock(DATA_MUTEX);
    DATA.rtc_sync_source = syncSourceToString(source);
}

void publishRtcTime(const DateTime& dt)
{
    char timeBuf[32] = {0};
    formatIsoUtc(dt, timeBuf, sizeof(timeBuf));
    sf_mqtt::publish("smartfranklin/rtc/time", timeBuf, 1, true);

    M5_LOGI("[RTC] %s", timeBuf);

    std::lock_guard<std::mutex> lock(DATA_MUTEX);
    DATA.rtc_time = timeBuf;
}

bool probeBm8563Like(const uint8_t* regs, const size_t len)
{
    if (regs == nullptr || len < 9U) {
        return false;
    }

    return isValidBcd(static_cast<uint8_t>(regs[2] & 0x7F), 59) &&
           isValidBcd(static_cast<uint8_t>(regs[3] & 0x7F), 59) &&
           isValidBcd(static_cast<uint8_t>(regs[4] & 0x3F), 23) &&
           isValidBcd(static_cast<uint8_t>(regs[5] & 0x3F), 31) &&
           isValidBcd(static_cast<uint8_t>(regs[7] & 0x1F), 12) &&
           isValidBcd(regs[8], 99);
}

bool probePcd85063Like(const uint8_t* regs, const size_t len)
{
    if (regs == nullptr || len < 11U) {
        return false;
    }

    return isValidBcd(static_cast<uint8_t>(regs[4] & 0x7F), 59) &&
           isValidBcd(static_cast<uint8_t>(regs[5] & 0x7F), 59) &&
           isValidBcd(static_cast<uint8_t>(regs[6] & 0x3F), 23) &&
           isValidBcd(static_cast<uint8_t>(regs[7] & 0x3F), 31) &&
           isValidBcd(static_cast<uint8_t>(regs[9] & 0x1F), 12) &&
           isValidBcd(regs[10], 99);
}

bool readDateTimeFromRtc(const RtcState& state, DateTime& out)
{
    if (state.source == RTC::Source::InternalRtc && readDateTimeFromInternalApi(out)) {
        return true;
    }

    uint8_t regs[11] = {0};
    if (!readBurst(state, 0x00, regs, sizeof(regs))) {
        return false;
    }

    switch (state.chipKind) {
    case sf_i2c::Device::ChipKind::Bm8563Like:
        out.second = fromBcd(static_cast<uint8_t>(regs[2] & 0x7F));
        out.minute = fromBcd(static_cast<uint8_t>(regs[3] & 0x7F));
        out.hour = fromBcd(static_cast<uint8_t>(regs[4] & 0x3F));
        out.day = fromBcd(static_cast<uint8_t>(regs[5] & 0x3F));
        out.month = fromBcd(static_cast<uint8_t>(regs[7] & 0x1F));
        out.year = 2000 + fromBcd(regs[8]);
        return true;

    case sf_i2c::Device::ChipKind::Pcd85063Like:
        out.second = fromBcd(static_cast<uint8_t>(regs[4] & 0x7F));
        out.minute = fromBcd(static_cast<uint8_t>(regs[5] & 0x7F));
        out.hour = fromBcd(static_cast<uint8_t>(regs[6] & 0x3F));
        out.day = fromBcd(static_cast<uint8_t>(regs[7] & 0x3F));
        out.month = fromBcd(static_cast<uint8_t>(regs[9] & 0x1F));
        out.year = 2000 + fromBcd(regs[10]);
        return true;

    case sf_i2c::Device::ChipKind::Unknown:
    default:
        return false;
    }
}

bool writeDateTimeToRtc(const RtcState& state, const DateTime& dt)
{
    if (state.source == RTC::Source::InternalRtc) {
        return writeDateTimeToInternalApi(dt);
    }

    if (state.chipKind == sf_i2c::Device::ChipKind::Bm8563Like) {
        uint8_t payload[7] = {0};
        payload[0] = toBcd(dt.second);
        payload[1] = toBcd(dt.minute);
        payload[2] = toBcd(dt.hour);
        payload[3] = toBcd(dt.day);
        payload[4] = 0x00;
        payload[5] = toBcd(dt.month);
        payload[6] = toBcd(dt.year % 100);
        return writeBurst(state, 0x02, payload, sizeof(payload));
    }

    if (state.chipKind == sf_i2c::Device::ChipKind::Pcd85063Like) {
        uint8_t payload[7] = {0};
        payload[0] = 0x00;
        payload[1] = toBcd(dt.second);
        payload[2] = toBcd(dt.minute);
        payload[3] = toBcd(dt.hour);
        payload[4] = toBcd(dt.day);
        payload[5] = 0x00;
        payload[6] = toBcd(dt.month);
        if (!writeBurst(state, 0x03, payload, sizeof(payload))) {
            return false;
        }

        const uint8_t yearBcd = toBcd(dt.year % 100);
        return writeBurst(state, 0x0A, &yearBcd, 1U);
    }

    return false;
}

bool selectChipKind(const RTC::Source requestedSource,
                    const uint8_t* regs,
                    const size_t len,
                    sf_i2c::Device::ChipKind& out)
{
    const bool bmOk = probeBm8563Like(regs, len);
    const bool pcdOk = probePcd85063Like(regs, len);

    switch (requestedSource) {
    case RTC::Source::InternalRtc:
    case RTC::Source::ExternalM5StackRtcUnit:
        if (bmOk) {
            out = sf_i2c::Device::ChipKind::Bm8563Like;
            return true;
        }
        return false;

    case RTC::Source::ExternalSeeedPcd85063tp:
        if (pcdOk) {
            out = sf_i2c::Device::ChipKind::Pcd85063Like;
            return true;
        }
        return false;

    case RTC::Source::None:
    default:
        return false;
    }
}

bool syncSystemFromNtp()
{
    if (WiFi.status() != WL_CONNECTED) {
        return false;
    }

    configTzTime("UTC0", "pool.ntp.org", "time.google.com", "time.nist.gov");
    struct tm nowTm;
    if (!getLocalTime(&nowTm, RTC_NTP_TIMEOUT_MS)) {
        return false;
    }

    const time_t nowEpoch = time(nullptr);
    return isSystemTimeValid(nowEpoch);
}

bool syncSystemFromRtc(const RtcState& state)
{
    DateTime rtcDateTime;
    if (!readDateTimeFromRtc(state, rtcDateTime)) {
        return false;
    }

    time_t epoch = 0;
    if (!datetimeToEpoch(rtcDateTime, epoch)) {
        return false;
    }

    struct timeval tv;
    tv.tv_sec = epoch;
    tv.tv_usec = 0;
    if (settimeofday(&tv, nullptr) != 0) {
        return false;
    }

    return true;
}

bool syncRtcFromSystem(const RtcState& state)
{
    const time_t nowEpoch = time(nullptr);
    if (!isSystemTimeValid(nowEpoch)) {
        return false;
    }

    DateTime nowDt;
    if (!epochToDateTime(nowEpoch, nowDt)) {
        return false;
    }

    return writeDateTimeToRtc(state, nowDt);
}

bool initState(RtcState& state, RTC::Source source, const bool isInternalRoute, const uint8_t i2cAddress)
{
    std::lock_guard<std::mutex> lock(state.mutex);

    state.initialized = false;
    state.source = RTC::Source::None;
    state.chipKind = sf_i2c::Device::ChipKind::Unknown;
    state.isInternalRoute = isInternalRoute;
    state.i2cAddress = i2cAddress;
    state.lastProcessMs = 0;
    state.lastPublishMs = 0;
    state.lastRtcWriteMs = 0;
    state.lastSyncSource = SyncSource::None;

    if (source == RTC::Source::InternalRtc) {
        DateTime internalDt{};
        if (!readDateTimeFromInternalApi(internalDt)) {
            M5_LOGW("[RTC] internal RTC API unavailable");
            return false;
        }

        state.source = source;
        state.chipKind = sf_i2c::Device::ChipKind::Bm8563Like;
        state.initialized = true;
        publishSource(state.source);
        publishSyncSource(state.lastSyncSource);

        M5_LOGI("[RTC] initialized source:%s via M5.Rtc API", RTC::sourceToString(state.source));
        return true;
    }

    uint8_t regs[11] = {0};
    if (!readBurst(state, 0x00, regs, sizeof(regs))) {
        M5_LOGW("[RTC] probe read failed source:%s addr:0x%02X route:%s",
                RTC::sourceToString(source),
                state.i2cAddress,
                state.isInternalRoute ? "internal" : "wire");
        return false;
    }

    sf_i2c::Device::ChipKind kind = sf_i2c::Device::ChipKind::Unknown;
    if (!selectChipKind(source, regs, sizeof(regs), kind)) {
        M5_LOGW("[RTC] unsupported register map for requested source:%s", RTC::sourceToString(source));
        return false;
    }

    state.source = source;
    state.chipKind = kind;
    state.initialized = true;

    publishSource(state.source);
    publishSyncSource(state.lastSyncSource);

    M5_LOGI("[RTC] initialized source:%s chip:%s address:0x%02X route:%s",
            RTC::sourceToString(state.source),
            sf_i2c::chipKindToString(state.chipKind),
            state.i2cAddress,
            state.isInternalRoute ? "internal" : "wire");

    return true;
}

void processState(RtcState& state)
{
    std::lock_guard<std::mutex> lock(state.mutex);
    if (!state.initialized) {
        return;
    }

    const uint32_t nowMs = millis();
    if ((nowMs - state.lastProcessMs) < RTC_PROCESS_PERIOD_MS) {
        return;
    }
    state.lastProcessMs = nowMs;

    SyncSource syncSource = SyncSource::None;

    if (syncSystemFromNtp()) {
        syncSource = SyncSource::Ntp;
    } else {
        const time_t nowEpoch = time(nullptr);
        if (!isSystemTimeValid(nowEpoch) && syncSystemFromRtc(state)) {
            syncSource = SyncSource::Rtc;
        }
    }

    if (syncSource != SyncSource::None) {
        state.lastSyncSource = syncSource;
        publishSyncSource(syncSource);
    }

    if ((nowMs - state.lastRtcWriteMs) >= RTC_WRITE_PERIOD_MS) {
        if (syncRtcFromSystem(state)) {
            state.lastRtcWriteMs = nowMs;
        }
    }

    if ((nowMs - state.lastPublishMs) >= RTC_PUBLISH_PERIOD_MS) {
        DateTime rtcDt;
        if (readDateTimeFromRtc(state, rtcDt)) {
            publishRtcTime(rtcDt);
        }
        state.lastPublishMs = nowMs;
    }
}

bool isInitializedState(const RtcState& state)
{
    std::lock_guard<std::mutex> lock(state.mutex);
    return state.initialized;
}

RTC::Source sourceState(const RtcState& state)
{
    std::lock_guard<std::mutex> lock(state.mutex);
    return state.source;
}

}  // namespace

RTC RTC_MODULE;

bool RTC::init(Source source, bool isInternalRoute, uint8_t i2cAddress)
{
    return initState(RTC_STATE, source, isInternalRoute, i2cAddress);
}

void RTC::process()
{
    processState(RTC_STATE);
}

bool RTC::isInitialized() const
{
    return isInitializedState(RTC_STATE);
}

RTC::Source RTC::source() const
{
    return sourceState(RTC_STATE);
}

const char* RTC::sourceName() const
{
    return sourceToString(source());
}

const char* RTC::sourceToString(Source source)
{
    switch (source) {
    case Source::InternalRtc:
        return "internal_rtc";
    case Source::ExternalM5StackRtcUnit:
        return "external_m5stack_rtc_unit";
    case Source::ExternalSeeedPcd85063tp:
        return "external_seeed_pcd85063tp";
    case Source::None:
    default:
        return "none";
    }
}
