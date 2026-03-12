/*
 * ============================================================================
 * I2C Bus Interface - SmartFranklin
 * ============================================================================
 *
 * File:        i2c_bus.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Public API for I2C topology discovery and route detection on
 *              the primary Wire bus. Used by startup diagnostics and modules
 *              that must resolve whether a device is direct or PAHub-routed.
 *
 * Author:      Laurent Burais
 * Date:        12 March 2026
 * Version:     1.2
 *
 * Overview:
 *   This module provides two core capabilities:
 *   - full I2C topology enumeration (`enumerate_i2c_units`),
 *   - targeted route detection for one address (`detect_wire_bus_path`).
 *
 *   Route detection returns both routing mode and PAHub channel when
 *   applicable, allowing callers to configure access logic without duplicating
 *   probe code.
 *
 * ============================================================================
 * MIT License
 * ============================================================================
 * Copyright (c) 2026 Laurent Burais
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * ============================================================================
 */

#pragma once

#include <cstdint>

#include "m5_hw.h"

/**
 * @brief Wire bus route used to reach a target I2C device.
 */
enum class I2CBusMode : uint8_t {
	/** Target is not reachable on known routes. */
	Unset = 0,
	/** Target is directly reachable on the base Wire bus. */
	WireDirect,
	/** Target is reachable through PAHub channel multiplexing. */
	WirePaHub,
};

/**
 * @brief Result of route detection for a target I2C address.
 */
struct I2CBusPathResult {
    I2CBusMode bus_mode = I2CBusMode::Unset;
    int8_t pahub_channel = -1;
};

/**
 * @brief Enumerate I2C topology and update the provided report structure.
 *
 * Scans direct WIRE, WIRE/PAHUB, direct EX, and EX/PAHUB paths, then probes
 * Gravity Dual UART candidates for NB-IoT2/C6L activity and prints a
 * hierarchical connection report.
 *
 * @param i2c_report Report structure populated with detected topology data.
 * @return Number of discovered bus entries.
 */
uint16_t enumerate_i2c_units(I2CEnumerationReport& i2c_report);

/**
 * @brief Detects how a target address is reachable on the Wire bus.
 *
 * Detection order:
 * 1. Probe direct Wire path.
 * 2. Probe PAHub presence and then scan channels 0..7.
 *
 * @param target_address 7-bit I2C address to resolve.
 * @param result Output structure receiving detected mode and PAHub channel.
 * @return true when a supported path is found, otherwise false.
 */
bool detect_wire_bus_path(uint8_t target_address, I2CBusPathResult& result);
