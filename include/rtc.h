/**
 * @file rtc.h
 * @brief Interface publique pour l'horloge temps réel interne (RTC) : acquisition, synchronisation NTP, publication MQTT.
 *
 * Ce module gère uniquement l'horloge interne du M5Stack (plus aucune gestion de source RTC externe).
 * Il fournit la synchronisation NTP, la gestion de fuseau horaire, et la publication des données via MQTT.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <Arduino.h>
#include <cstdint>
#include <ctime>
#include <mutex>

class RTC {
public:
    /**
     * @brief Initialize internal M5 RTC backend.
     * @return True when initialization succeeds.
     */
    bool init();

    /**
     * @brief Execute one periodic RTC refresh cycle.
     *
     * Reads current time, validates it, and propagates values into shared
     * runtime state for display and telemetry consumers.
     */
    void process();

    /**
     * @brief Check whether RTC has been initialized successfully.
     * @return True when RTC module is operational.
     */
    bool isInitialized() const;

private:
    /** Normalized date-time tuple. */
    struct DateTime {
        int year   = 0;
        int month  = 0;
        int day    = 0;
        int hour   = 0;
        int minute = 0;
        int second = 0;
    };

    /** Origine de synchronisation temporelle (NTP ou RTC interne uniquement). */
    enum class SyncSource : uint8_t { None = 0, Ntp, Rtc };

    // ---- constants ----
    static constexpr uint32_t kProcessPeriodMs = 5000UL;
    static constexpr uint32_t kPublishPeriodMs = 30000UL;
    static constexpr uint32_t kWritePeriodMs   = 60000UL;
    static constexpr uint32_t kNtpTimeoutMs    = 1500UL;
    static constexpr time_t   kMinValidEpoch   = 1704067200; // 2024-01-01T00:00:00Z

    // ---- pure static utilities ----
    static bool    isSystemTimeValid(time_t t);
    static bool    datetimeToEpoch(const DateTime& dt, time_t& outEpoch);
    static bool    epochToDateTime(time_t epoch, DateTime& out);
    static bool    epochToLocalIso(time_t epoch, char* out, size_t outLen);
    static void    formatIsoUtc(const DateTime& dt, char* out, size_t outLen);
    static bool    readDateTimeFromInternalApi(DateTime& out);
    static bool    writeDateTimeToInternalApi(const DateTime& dt);
    static const char* syncSourceToString(SyncSource src);
    static String  timezoneConfigValue();
    static String  timezoneToTzString(const String& value);
    static void    publishTimezone();
    static void    publishSyncSource(SyncSource src);
    static void    publishRtcTime(const DateTime& dt);
    static bool    syncSystemFromNtp();

    // ---- instance methods (need member state) ----
    bool readDateTime(DateTime& out);
    bool writeDateTime(const DateTime& dt);

    // ---- member state ----
    mutable std::mutex m_mutex;
    bool m_initialized = false;
    uint32_t m_lastProcessMs = 0;
    uint32_t m_lastPublishMs = 0;
    uint32_t m_lastRtcWriteMs = 0;
    SyncSource m_lastSyncSource = SyncSource::None;
};

/** Instance globale utilisée par les tâches système. */
extern RTC RTC_TASK;
