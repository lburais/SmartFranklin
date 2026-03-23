/*
 * SmartFranklin - GPS module interface
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <Arduino.h>

class GPS {
public:
    enum class Source : uint8_t {
        None = 0,
        ExternalDfrobotGravity,
    };

    bool init(Source source, bool isInternalRoute, uint8_t i2cAddress);
    void process();
    bool isInitialized() const;

    Source source() const;
    const char* sourceName() const;
    static const char* sourceToString(Source source);
};

extern GPS GPS_MODULE;
