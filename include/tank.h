/**
 * ============================================================================
 * Tank Module Interface - SmartFranklin IoT Device Controller
 * ============================================================================
 *
 * File:        tank.h
 * Project:     SmartFranklin - ESP32 IoT Water Tank Monitor
 * Description: Public API for the M5Stack Unit Ultrasonic I2C water level
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

#include <mutex>

/**
 * @brief Tank ultrasonic water level runtime.
 */
class Tank {
public:
	/** Initialize tank sensor from configured tank port metadata. */
	bool init();

	/** Run one acquisition and publication cycle. */
	void process();

	/** Return true when module initialization is complete. */
	bool isInitialized() const;

private:
	mutable std::mutex m_mutex;
	bool m_initialized = false;
	String m_activeConfiguredPort;
	uint8_t m_i2cAddress = 0x57;
};

/** Global singleton used by taskTank, mirroring LEVEL_TASK pattern. */
extern Tank TANK_TASK;


