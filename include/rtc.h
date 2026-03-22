/*
 * SmartFranklin - RTC module interface
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <Arduino.h>

class RTC {
public:
    enum class Source : uint8_t {
        None = 0,
        InternalRtc,
        ExternalM5StackRtcUnit,
        ExternalSeeedPcd85063tp,
    };

    bool init(Source source, bool isInternalRoute, uint8_t i2cAddress);
    void process();
    bool isInitialized() const;

    Source source() const;
    const char* sourceName() const;
    static const char* sourceToString(Source source);
};

extern RTC RTC_MODULE;
