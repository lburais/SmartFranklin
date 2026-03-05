/*
 * ============================================================================
 * JBD BMS Module - SmartFranklin
 * ============================================================================
 * 
 * File:        jbd_bms.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Header file for JBD Battery Management System protocol parsing.
 *              Provides data structures and functions for decoding JBD BMS
 *              communication frames containing battery voltage, current, and
 *              state of charge information.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   The JBD BMS module handles communication with JBD (Jianxing Battery Device)
 *   battery management systems commonly used in lithium-ion battery packs.
 *   The module parses binary protocol frames received over serial or I2C
 *   interfaces, extracting critical battery parameters for monitoring and
 *   control in SmartFranklin's power management system.
 * 
 * JBD Protocol Overview:
 *   - Manufacturer: JBD (Shenzhen Jianxing Battery Technology Co.)
 *   - Application: Lithium-ion battery pack management
 *   - Communication: Serial protocol (UART) or I2C
 *   - Frame Format: Binary protocol with header, data, checksum
 *   - Parameters: Voltage, current, temperature, cell voltages, protection status
 *   - Update Rate: Configurable (typically 100ms-1s intervals)
 * 
 * Frame Structure:
 *   - Header: Protocol identification and frame type
 *   - Data Payload: Battery parameters in binary format
 *   - Checksum: CRC or simple checksum for data integrity
 *   - Length: Variable frame length depending on data included
 * 
 * Battery Parameters:
 *   - Voltage: Total pack voltage (sum of all cells)
 *   - Current: Charge/discharge current (positive/negative)
 *   - State of Charge: Battery capacity percentage remaining
 *   - Temperature: Battery and/or ambient temperature
 *   - Cell Voltages: Individual cell voltage measurements
 *   - Protection Status: Over/under voltage, over current, temperature alerts
 * 
 * Integration:
 *   - Serial Interface: Connected via UART pins on ESP32
 *   - I2C Interface: Alternative connection through I2C bus
 *   - Data Model: Parsed values stored in global DATA structure
 *   - MQTT Publishing: Battery data published for remote monitoring
 *   - Safety Systems: Critical for battery protection and management
 * 
 * Error Handling:
 *   - Frame Validation: Checksum verification and format checking
 *   - Data Sanity: Range checking for voltage/current values
 *   - Communication Errors: Timeout and retry mechanisms
 *   - Fault Detection: BMS protection status monitoring
 * 
 * Performance Considerations:
 *   - Parsing Speed: Fast binary processing (< 1ms per frame)
 *   - Memory Usage: Minimal (small frame buffers)
 *   - CPU Load: Low impact on system performance
 *   - Real-time: Suitable for real-time battery monitoring
 * 
 * Dependencies:
 *   - Arduino.h: Basic types and utilities
 *   - HardwareSerial: For UART communication (if used)
 *   - Wire.h: For I2C communication (if used)
 * 
 * Limitations:
 *   - Protocol Specific: Only supports JBD BMS protocol
 *   - Frame Size: Limited to maximum expected frame length
 *   - No Encryption: Protocol data transmitted in plain binary
 *   - Single BMS: No support for multiple BMS units
 *   - Firmware Dependent: May require updates for new JBD firmware
 * 
 * Best Practices:
 *   - Validate checksums on all received frames
 *   - Monitor for communication timeouts
 *   - Implement battery protection logic based on BMS data
 *   - Regularly calibrate BMS parameters
 *   - Use appropriate update rates for battery monitoring
 * 
 * Safety Considerations:
 *   - Battery Voltage: Critical for over/under voltage protection
 *   - Current Monitoring: Prevents over-current damage
 *   - Temperature: Thermal protection for battery safety
 *   - SOC Accuracy: Important for battery life and capacity management
 *   - Fault Handling: Proper response to BMS fault conditions
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
 *   1. Validate frame header and protocol identification
 *   2. Check frame length and data integrity
 *   3. Verify checksum or CRC for data corruption
 *   4. Extract voltage value from designated bytes
 *   5. Extract current value with proper sign handling
 *   6. Extract state of charge percentage
 *   7. Perform range and sanity checking
 *   8. Populate output structure with parsed values
 * 
 * Frame Validation:
 *   - Header Check: Verify JBD protocol identifier
 *   - Length Check: Ensure sufficient data for all parameters
 *   - Checksum: Validate data integrity
 *   - Range Check: Verify values are within expected ranges
 *   - Sanity Check: Detect obviously invalid measurements
 * 
 * Error Conditions:
 *   - Invalid header: Wrong protocol or corrupted frame
 *   - Insufficient length: Frame truncated or incomplete
 *   - Checksum failure: Data corruption during transmission
 *   - Invalid values: Measurements outside expected ranges
 *   - Buffer errors: Null pointer or invalid buffer access
 * 
 * Performance:
 *   - Execution Time: < 1ms for typical frame sizes
 *   - Memory Usage: Minimal (no dynamic allocation)
 *   - CPU Load: Low (simple byte manipulation)
 *   - Real-time: Suitable for high-frequency BMS updates
 * 
 * Usage Example:
 *   uint8_t buffer[64];
 *   size_t received = serial.readBytes(buffer, sizeof(buffer));
 *   JbdFrame frame;
 *   if (jbd_parse_frame(buffer, received, frame)) {
 *       // Use frame.voltage, frame.current, frame.soc
 *   }
 * 
 * @note Function assumes data buffer contains complete BMS frame.
 *       Partial frames or concatenated frames may cause parsing failures.
 *       Check return value to ensure successful parsing before using results.
 * 
 * @see JbdFrame - Output data structure
 * @see HardwareSerial::readBytes() - Typical data source
 */
bool jbd_parse_frame(const uint8_t *data, size_t len, JbdFrame &out);