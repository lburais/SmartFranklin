/**
 * ============================================================================
 * Tank Module Interface - SmartFranklin IoT Device Controller
 * ============================================================================
 *
 * File:        tank.h
 * Project:     SmartFranklin - ESP32 IoT Water Tank Monitor
 * Description: Public API for the M5Stack Unit Ultrasonic I2C water level
 *              Uses Wire1 directly with SDA/SCL pins and I2C address from
 *              sf_interfaces.
 *
 * Hardware:    M5Stack Unit Ultrasonic I2C (RCWL-9600)
 *
 * Features:
 *
 * ============================================================================
 * MIT License - Copyright (c) 2026 Laurent Burais
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * ============================================================================
 */

#pragma once

#include <Arduino.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedDISTANCE.h>

#include <mutex>

#include "interfaces.h"


/**
 * @brief Tank ultrasonic water level runtime.
 */
class Tank {
public:
	/** Initialize tank sensor from configured tank port metadata. */
	bool init();

	/** Run one acquisition and publication cycle. */
	bool process();

	/** Return true when module initialization is complete. */
	bool isInitialized() const;

private:
	mutable std::mutex m_mutex;

    m5::unit::UnitUnified       m_units;
    m5::unit::UnitUltraSonicI2C m_unit;

	bool    m_initialized = false;

	const sf_interfaces::InterfaceSensor m_sensor = sf_interfaces::InterfaceSensor::Tank;
	const String m_tag = sf_interfaces::toUpperString(m_sensor);
	const String m_device = sf_interfaces::getDeviceName(m_sensor);

	const uint8_t TANK_DISTANCE_REGISTER = 0x01;
	const uint32_t TANK_CONVERSION_DELAY_MS = 120U;

	const int32_t TANK_DISTANCE_MIN_MM = 20;
	const int32_t TANK_DISTANCE_MAX_MM = 4500;

	const int32_t TANK_FULL_DISTANCE_MM = 300;
	const int32_t TANK_EMPTY_DISTANCE_MM = 1500;

};

/** Global singleton used by taskTank, mirroring LEVEL_TASK pattern. */
extern Tank TANK_TASK;


