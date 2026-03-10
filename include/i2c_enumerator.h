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
