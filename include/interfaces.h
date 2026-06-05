/**
 * @file interfaces.h
 * @brief Centralized interface (port/sensor) definitions and interface helpers.
 *
 * Provides hardware interface definitions and connector helpers in a single
 * namespace.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <Arduino.h>
#include <M5Unified.h>

#include <cstddef>
#include <cstdint>

#include "freertos/semphr.h"

class TwoWire;
class SoftWire;
class HardwareSerial;
class EspSoftwareSerial;

namespace sf_interfaces {

enum class InterfaceSensor : uint8_t {
    Axp = 0,   /// start first to ensure power on grove
    Gaz,
    Tank,
    Gps,
    Lte,
    Lora,
    Lin,
    Imu,
    Rtc,
    Ina1,
    Ina2,
    Bat,
    Obd,
    None,
};

struct InterfaceConnector {
    union Ptr {
        void* raw;
        TwoWire*        twoWire;
        SoftWire*       softWire;
        HardwareSerial* hardwareSerial;

        constexpr Ptr() : raw(nullptr) {}
    } ptr{};
};

// global

const InterfaceConnector getConnector(InterfaceSensor sensor);

const char* getSensor(const InterfaceSensor sensor, const uint8_t channel);

const char* getDeviceName(InterfaceSensor sensor);
uint8_t getAddress(InterfaceSensor sensor);
uint32_t getRecurrenceMs(InterfaceSensor sensor);
int8_t getLed(InterfaceSensor sensor);
int8_t getSda(InterfaceSensor sensor);
int8_t getScl(InterfaceSensor sensor);
uint32_t getClock(InterfaceSensor sensor);

bool isI2C(InterfaceSensor sensor);

const char* toString(InterfaceSensor sensor, bool upper = false);

bool configure_all_sensors();
bool configure(InterfaceSensor sensor);
bool configured(InterfaceSensor sensor);
bool isAvailable(InterfaceSensor sensor);

bool seize(InterfaceSensor sensor);
void release(InterfaceSensor sensor);

constexpr uint32_t kInterfaceInitRetryMs = 10000UL;


}  // namespace sf_interfaces
