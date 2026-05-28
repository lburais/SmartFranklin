#include "log.h"

#include <ArduinoJson.h>
#include <M5Unified.h>

#include <cstdarg>
#include <ctime>
#include <cstdio>
#include <cstring>
#include <mutex>

namespace sf_log {
namespace {

struct LogEntry {
    uint32_t seq = 0;
    char time[20] = {0};
    char level = 'I';
    char msg[192] = {0};
};

constexpr size_t kLogCapacity = 240;
LogEntry g_entries[kLogCapacity];
size_t g_head = 0;    // Next write position.
size_t g_count = 0;   // Number of valid entries.
uint32_t g_nextSeq = 1;
std::mutex g_mutex;

char levelToChar(const Level level)
{
    switch (level) {
    case Level::Verbose: return 'V';
    case Level::Debug:   return 'D';
    case Level::Info:    return 'I';
    case Level::Warn:    return 'W';
    case Level::Error:   return 'E';
    default:             return 'I';
    }
}

void formatCurrentTime(char* out, const size_t outSize)
{
    if (out == nullptr || outSize == 0) {
        return;
    }

    out[0] = '\0';

    const time_t now = time(nullptr);
    struct tm tmLocal = {};
    if (localtime_r(&now, &tmLocal) == nullptr) {
        strncpy(out, "00-00-00 00:00:00", outSize - 1);
        out[outSize - 1] = '\0';
        return;
    }

    if (strftime(out, outSize, "%d-%m-%y %H:%M:%S", &tmLocal) == 0) {
        strncpy(out, "00-00-00 00:00:00", outSize - 1);
        out[outSize - 1] = '\0';
    }
}

}  // namespace

void vwrite(const Level level, const char* fmt, va_list args)
{
    if (fmt == nullptr) {
        return;
    }

    char body[192] = {0};
    vsnprintf(body, sizeof(body), fmt, args);

    // If message starts with "[tag] ", normalize to "[tag       ] ".
    if (body[0] == '[') {
        const char* close = strchr(body + 1, ']');
        if (close != nullptr && close[1] == ' ') {
            const size_t tagLen = static_cast<size_t>(close - (body + 1));
            if (tagLen > 0U) {
                char tag[64] = {0};
                const size_t copyLen = (tagLen < (sizeof(tag) - 1U)) ? tagLen : (sizeof(tag) - 1U);
                memcpy(tag, body + 1, copyLen);
                tag[copyLen] = '\0';

                const char* rest = close + 2;
                char normalized[sizeof(body)] = {0};
                snprintf(normalized, sizeof(normalized), "[%-6.6s] %s", tag, rest);
                strncpy(body, normalized, sizeof(body) - 1);
                body[sizeof(body) - 1] = '\0';
            }
        }
    }
    
    const char lvl = levelToChar(level);
    char currentTime[20] = {0};
    formatCurrentTime(currentTime, sizeof(currentTime));

    {
        std::lock_guard<std::mutex> lock(g_mutex);

        LogEntry& entry = g_entries[g_head];
        entry.seq = g_nextSeq++;
        strncpy(entry.time, currentTime, sizeof(entry.time) - 1);
        entry.time[sizeof(entry.time) - 1] = '\0';
        entry.level = lvl;
        strncpy(entry.msg, body, sizeof(entry.msg) - 1);
        entry.msg[sizeof(entry.msg) - 1] = '\0';

        g_head = (g_head + 1) % kLogCapacity;
        if (g_count < kLogCapacity) {
            ++g_count;
        }
    }

}

void write(const Level level, const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vwrite(level, fmt, args);
    va_end(args);
}

String getLogsJson(const uint32_t afterSeq, const size_t maxEntries)
{
    JsonDocument doc;
    JsonArray entries = doc["entries"].to<JsonArray>();

    {
        std::lock_guard<std::mutex> lock(g_mutex);

        const uint32_t nextSeq = g_nextSeq;
        doc["next_seq"] = nextSeq;

        if (g_count == 0 || maxEntries == 0) {
            String out;
            serializeJson(doc, out);
            return out;
        }

        const size_t start = (g_count < kLogCapacity) ? 0 : g_head;
        size_t added = 0;

        for (size_t i = 0; i < g_count; ++i) {
            const size_t idx = (start + i) % kLogCapacity;
            const LogEntry& e = g_entries[idx];
            if (e.seq <= afterSeq) {
                continue;
            }
            if (added >= maxEntries) {
                break;
            }

            JsonObject item = entries.add<JsonObject>();
            item["seq"] = e.seq;
            item["time"] = e.time;
            char lvl[2] = {e.level, '\0'};
            item["level"] = lvl;
            item["msg"] = e.msg;
            ++added;
        }
    }

    String out;
    serializeJson(doc, out);
    return out;
}

}  // namespace sf_log
