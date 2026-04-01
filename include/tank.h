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

/**
 * @brief Acquire one tank measurement, update DATA, and publish MQTT topics.
 *
 * This helper expects that the task already selected and initialized the
 * external I2C port (A1/A2/B1/B2/C1/C2) before calling it.
 *
 * @param i2cAddress Ultrasonic sensor I2C address (typically 0x57)
 * @return true when one valid measurement was processed and published
 */
bool tankReadAndPublish(uint8_t i2cAddress);


