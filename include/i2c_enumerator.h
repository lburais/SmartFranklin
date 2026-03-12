/*
 * ============================================================================
 * I2C Enumerator Interface - SmartFranklin
 * ============================================================================
 *
 * File:        i2c_enumerator.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Declares I2C topology scan entry point used during startup
 *              hardware discovery and reporting.
 *
 * Author:      Laurent Burais
 * Date:        10 March 2026
 * Version:     1.1
 *
 * ============================================================================
 */

#pragma once

#include <cstdint>

#include "m5_hw.h"

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
