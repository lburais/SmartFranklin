/**
 * ============================================================================
 * JBD BMS Module - SmartFranklin
 * ============================================================================
 * 
 * File:        jbd_bms.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Header file for JBD Battery Management System protocol parsing.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 * 
 * JBD Protocol Overview:
 * 
 * Frame Structure:
 * 
 * Battery Parameters:
 * 
 * Integration:
 * 
 * Error Handling:
 * 
 * Performance Considerations:
 * 
 * Dependencies:
 * 
 * Limitations:
 * 
 * Best Practices:
 * 
 * Safety Considerations:
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
#include <Arduino.h>

/**
 * @brief Data structure for parsed JBD BMS frame information.
 * 
 * Contains the essential battery parameters extracted from a JBD BMS
 * communication frame. Provides a clean interface for accessing
 * voltage, current, and state of charge measurements.
 */
struct JbdFrame {
    /**
     * @brief Total battery pack voltage.
     * 
     * The sum of all cell voltages in the battery pack.
     * Critical for battery monitoring and protection systems.
     * 
     * Units: Volts (V)
     * Range: Typically 0.0 to 50.0V (depends on pack configuration)
     * Precision: 2-3 decimal places (protocol dependent)
     * Update Rate: Matches BMS communication rate
     */
    float voltage;

    /**
     * @brief Battery charge/discharge current.
     * 
     * Current flowing through the battery. Positive values indicate
     * charging, negative values indicate discharging.
     * 
     * Units: Amperes (A)
     * Range: Negative (discharge) to positive (charge)
     * Precision: 2-3 decimal places (protocol dependent)
     * Sign Convention: Positive = charging, Negative = discharging
     */
    float current;

    /**
     * @brief Battery state of charge percentage.
     * 
     * Estimated percentage of battery capacity remaining.
     * Calculated by BMS using coulomb counting and voltage measurements.
     * 
     * Units: Percentage (%)
     * Range: 0.0 to 100.0
     * Precision: 1 decimal place typical
     * Accuracy: Depends on BMS calibration and battery age
     */
    float soc;
};

/**
 * @brief Parses a JBD BMS protocol frame from raw data.
 * 
 * Takes a buffer of raw bytes received from the BMS and parses it
 * according to the JBD protocol specification. Extracts voltage,
 * current, and state of charge values into the output structure.
 * 
 * @param data - Pointer to raw frame data buffer
 * @param len - Length of the data buffer in bytes
 * @param out - Reference to JbdFrame structure to fill with parsed data
 * @return true if frame parsed successfully, false on errors
 * 
 * Parsing Process:
 * 
 * Frame Validation:
 * 
 * Error Conditions:
 * 
 * Performance:
 * 
 * Usage Example:
 * 
 * @note Function assumes data buffer contains complete BMS frame.
 * 
 * @see JbdFrame - Output data structure
 * @see HardwareSerial::readBytes() - Typical data source
 */
bool jbd_parse_frame(const uint8_t *data, size_t len, JbdFrame &out);