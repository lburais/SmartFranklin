#pragma once

#include <Arduino.h>
#include <M5Unified.h>

#include <cstdarg>
#include <cstddef>
#include <cstdint>

namespace sf_log {

enum class Level : uint8_t {
    Verbose = 0,
    Debug,
    Info,
    Warn,
    Error,
};

void write(Level level, const char* fmt, ...);
void vwrite(Level level, const char* fmt, va_list args);

// Returns a JSON payload: {"next_seq":N,"entries":[...]}
String getLogsJson(uint32_t afterSeq, size_t maxEntries = 100);

}  // namespace sf_log

#define SF_LOGV(...)                                                          \
    do {                                                                      \
        M5_LOGV(__VA_ARGS__);                                                 \
        ::sf_log::write(::sf_log::Level::Verbose, __VA_ARGS__);               \
    } while (0)
#define SF_LOGD(...)                                                          \
    do {                                                                      \
        M5_LOGD(__VA_ARGS__);                                                 \
        ::sf_log::write(::sf_log::Level::Debug, __VA_ARGS__);                 \
    } while (0)
#define SF_LOGI(...)                                                          \
    do {                                                                      \
        M5_LOGI(__VA_ARGS__);                                                 \
        ::sf_log::write(::sf_log::Level::Info, __VA_ARGS__);                  \
    } while (0)
#define SF_LOGW(...)                                                          \
    do {                                                                      \
        M5_LOGW(__VA_ARGS__);                                                 \
        ::sf_log::write(::sf_log::Level::Warn, __VA_ARGS__);                  \
    } while (0)
#define SF_LOGE(...)                                                          \
    do {                                                                      \
        M5_LOGE(__VA_ARGS__);                                                 \
        ::sf_log::write(::sf_log::Level::Error, __VA_ARGS__);                 \
    } while (0)
