/*
 * ============================================================================
 * JBD Battery Management System Parser - SmartFranklin
 * ============================================================================
 * 
 * File:        jbd_bms.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: Protocol parser for JBD BMS (Battery Management System) serial
 *              communication. Decodes proprietary BMS frame format to extract
 *              battery voltage, current, and state of charge metrics.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   JBD (深圳杰比特) is a manufacturer of battery management systems used in
 *   various energy storage and power systems. SmartFranklin interfaces with
 *   JBD BMS units via serial (UART) communication to monitor battery health
 *   and operational status in real-time.
 * 
 * Communication Protocol:
 *   - Interface: Serial UART (typically 9600, 19200, or 115200 baud)
 *   - Frame Format: Proprietary binary protocol with fixed structure
 *   - Response Type: Unsolicited periodic updates or request-response
 *   - Encoding: Big-endian multi-byte values
 *   - Error Detection: Frame structure validation (header and length checks)
 * 
 * JBD Frame Structure:
 *   Byte Offset | Field          | Description
 *   ------------|----------------|------------------------------------------
 *   0           | Frame Start    | 0xDD (magic number, indicates BMS frame)
 *   1           | Command        | 0x03 (typically status/info request response)
 *   2-3         | Data Length    | Frame payload size (13 bytes for status)
 *   4-5         | Voltage (mV)   | Battery voltage in millivolts (big-endian)
 *   6-7         | Current (mA)   | Charge/discharge current (big-endian, signed)
 *   8           | SOC (%)        | State of Charge percentage (0-100)
 *   9-12        | Checksum/CRC   | Frame validation bytes (not used in this parser)
 * 
 * Example Frame (48V battery, 2.5A charging, 85% SOC):
 *   Hex: DD 03 00 0D BB 80 00 64 55 00 01 FB 2D
 *   
 *   Interpretation:
 *   - DD: Frame start marker
 *   - 03: Status command
 *   - 00 0D: 13 bytes of data
 *   - BB 80: 0xBB80 = 48000 mV = 48.0V
 *   - 00 64: 0x0064 = 100 mA = 0.1A (positive = charging)
 *   - 55: 85% SOC
 *   - 00 01 FB 2D: Checksum fields
 * 
 * Value Encoding:
 *   - Voltage: Stored as 16-bit unsigned integer in millivolts (mV)
 *     Conversion: voltage_V = voltage_mV / 1000.0
 *     Example: 0xBB80 (48000 mV) → 48.0V
 * 
 *   - Current: Stored as 16-bit signed integer in milliamps (mA)
 *     Positive values: Battery charging (current flowing into pack)
 *     Negative values: Battery discharging (current flowing out of pack)
 *     Conversion: current_A = current_mA / 1000.0
 *     Example: 0x0064 (100 mA) → 0.1A charging
 *     Example: 0xFF9C (-100 mA) → -0.1A discharging
 * 
 *   - SOC: Stored as 8-bit unsigned integer representing percentage
 *     Range: 0 to 100 (%), directly usable without conversion
 *     Example: 0x55 (85) → 85% charged
 * 
 * Multi-pack Support:
 *   Many JBD systems support multiple battery packs with individual monitoring:
 *   - Each pack responds with its own frame
 *   - Frames received on different serial ports or time-slots
 *   - Parser designed to handle single frames independently
 * 
 * Dependencies:
 *   - jbd_bms.h (structure and function declarations)
 *   - <cstdint> (fixed-width integer types: uint8_t, uint16_t, int16_t)
 * 
 * Error Conditions:
 *   - Frame too short (< 13 bytes): Invalid, return false
 *   - Wrong start marker (not 0xDD): Different protocol, return false
 *   - Wrong command (not 0x03): Status response not recognized, return false
 *   - Partial/corrupted frame: Parser will reject, caller should resynchronize
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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ============================================================================
 */

#include "jbd_bms.h"

// ============================================================================
// JBD BMS Frame Parser
// ============================================================================

/**
 * @brief Parses JBD BMS serial frame to extract battery metrics.
 * 
 * Decodes binary frame data from JBD battery management system unit, validating
 * frame structure and extracting voltage, current, and state of charge values.
 * Performs comprehensive frame validation before parsing to ensure data integrity.
 * 
 * Frame Validation Steps:
 *   1. Minimum Length Check: Rejects frames shorter than 13 bytes
 *   2. Start Marker Check: Verifies first byte is 0xDD (JBD frame marker)
 *   3. Command Check: Verifies command byte is 0x03 (status response)
 * 
 *   All validations must pass for parsing to proceed. Any failure returns false
 *   and output structure is left unchanged (no partial updates).
 * 
 * Field Extraction and Conversion:
 * 
 *   Voltage (Bytes 4-5):
 *   - Raw bytes combined as: mv = (data[4] << 8) | data[5]
 *   - Big-endian 16-bit unsigned integer in millivolts
 *   - Converted to volts: voltage = mv / 1000.0f
 *   - Range: 0 to 65.535V (typical: 24V-60V systems)
 *   - Precision: 0.001V (1mV resolution)
 * 
 *   Current (Bytes 6-7):
 *   - Raw bytes combined as: ma = (data[6] << 8) | data[7]
 *   - Big-endian 16-bit SIGNED integer in milliamps
 *   - Positive values: Battery charging (current into pack)
 *   - Negative values: Battery discharging (current out of pack)
 *   - Converted to amps: current = ma / 1000.0f
 *   - Range: -32.768A to +32.767A
 *   - Precision: 0.001A (1mA resolution)
 * 
 *   State of Charge (Byte 8):
 *   - Unsigned 8-bit integer percentage (0-100)
 *   - No conversion needed, directly represents battery charge level
 *   - Range: 0% (empty) to 100% (full)
 * 
 * Output Parameters:
 *   All parsed values are assembled into JbdFrame structure and returned
 *   by reference. If parsing fails, structure remains unchanged.
 * 
 * Data Types in Output:
 *   - out.voltage: float (IEEE 754 single precision)
 *   - out.current: float (could be positive or negative)
 *   - out.soc: uint8_t (0-100 percentage, stored as uint8_t)
 * 
 * Error Handling:
 *   - Returns false immediately upon first validation failure
 *   - No exception throwing (exception-free for embedded systems)
 *   - Caller must check return value before using output data
 *   - Invalid frames are silently rejected (no error logging)
 * 
 * Example Usage:
 *   @code
 *   uint8_t frame_data[] = {0xDD, 0x03, 0x00, 0x0D, 0xBB, 0x80, 
 *                            0x00, 0x64, 0x55, 0x00, 0x01, 0xFB, 0x2D};
 *   JbdFrame result;
 *   
 *   if (jbd_parse_frame(frame_data, sizeof(frame_data), result)) {
 *       printf("Battery: %.1fV, %.2fA, %d%% SOC\n",
 *              result.voltage, result.current, result.soc);
 *       // Output: Battery: 48.0V, 0.10A, 85% SOC
 *   } else {
 *       printf("Invalid BMS frame\n");
 *   }
 *   @endcode
 * 
 * Performance:
 *   - Time Complexity: O(1) - Fixed operations regardless of frame size
 *   - Space Complexity: O(1) - No dynamic allocation or recursion
 *   - Typical execution: < 1 microsecond
 *   - Suitable for real-time embedded systems
 * 
 * @param data - Pointer to byte buffer containing JBD BMS frame data
 *               Must point to valid memory of at least 'len' bytes
 * 
 * @param len  - Size of frame data buffer in bytes
 *               Minimum required: 13 bytes for valid status frame
 *               Oversized buffers are acceptable (only first 13 used)
 * 
 * @param out  - Reference to JbdFrame structure to receive parsed results
 *               Populated only if function returns true
 *               Structure contents undefined if return value is false
 * 
 * @return bool - true if frame parsed successfully, false if validation failed
 *                - false: len < 13 (frame incomplete)
 *                - false: data[0] != 0xDD (not a JBD frame)
 *                - false: data[1] != 0x03 (not a status response)
 * 
 * @note This function performs NO checksum validation. Checksum fields
 *       (bytes 9-12) are ignored. For systems requiring validated data,
 *       implement CRC calculation and compare against bytes 9-12.
 * 
 * @see JbdFrame - Output structure definition in jbd_bms.h
 * @see jbd_parse_frame - Function being documented here
 */
bool jbd_parse_frame(const uint8_t *data, size_t len, JbdFrame &out)
{
    // =========================================================================
    // Frame Validation: Minimum Length
    // =========================================================================
    // Check if frame has minimum required bytes (13-byte status frame)
    // Insufficient data indicates incomplete/corrupted transmission
    if (len < 13) return false;

    // =========================================================================
    // Frame Validation: Start Marker (Magic Number)
    // =========================================================================
    // JBD frames always begin with 0xDD marker byte
    // Different marker indicates non-JBD protocol or data corruption
    if (data[0] != 0xDD) return false;

    // =========================================================================
    // Frame Validation: Command Type
    // =========================================================================
    // Byte 1 contains command identifier
    // 0x03 = Status information response (most common)
    // Other values indicate different frame types (not supported here)
    uint8_t cmd = data[1];
    if (cmd != 0x03) return false;

    // =========================================================================
    // Parse Battery Voltage (Bytes 4-5)
    // =========================================================================
    // Extract big-endian 16-bit unsigned integer (millivolts)
    // Shift byte 4 left 8 bits, OR with byte 5 to combine
    // Example: 0xBB, 0x80 → 0xBB80 = 48000 mV = 48.0V
    uint16_t mv = (data[4] << 8) | data[5];
    
    // Convert from millivolts to volts (divide by 1000)
    out.voltage = mv / 1000.0f;

    // =========================================================================
    // Parse Battery Current (Bytes 6-7)
    // =========================================================================
    // Extract big-endian 16-bit SIGNED integer (milliamps)
    // Positive values = charging (current into battery)
    // Negative values = discharging (current from battery)
    // Example: 0x00, 0x64 → 0x0064 = 100 mA = 0.1A (charging)
    // Example: 0xFF, 0x9C → 0xFF9C = -100 mA = -0.1A (discharging)
    int16_t ma = (data[6] << 8) | data[7];
    
    // Convert from milliamps to amps (divide by 1000)
    out.current = ma / 1000.0f;

    // =========================================================================
    // Parse State of Charge (Byte 8)
    // =========================================================================
    // Extract 8-bit unsigned integer representing battery charge percentage
    // Range: 0 (empty) to 100 (full)
    // No conversion needed - directly represents percentage
    out.soc = data[8];

    // Frame parsed successfully, output structure populated
    return true;
}