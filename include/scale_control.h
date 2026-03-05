/*
 * ============================================================================
 * Scale Control Module - SmartFranklin
 * ============================================================================
 * 
 * File:        scale_control.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Header file for weight scale control functionality. Provides
 *              interface for load cell weight measurement, calibration, and
 *              taring operations using HX711 or compatible load cell amplifier.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   The scale control module provides high-level interface for weight measurement
 *   using load cell sensors. It handles raw ADC readings from the HX711 24-bit
 *   analog-to-digital converter, applies calibration factors, and provides
 *   tare (zeroing) functionality. The module abstracts the complexities of
 *   load cell signal conditioning and provides reliable weight measurements
 *   for SmartFranklin's monitoring applications.
 * 
 * Load Cell Technology:
 *   - Sensor: Strain gauge load cell (typically 50kg or 100kg capacity)
 *   - Amplifier: HX711 24-bit ADC with programmable gain
 *   - Interface: Two-wire serial communication (DOUT, SCK)
 *   - Resolution: 24-bit ADC (up to 16,777,216 counts)
 *   - Gain Settings: 128x or 64x amplification
 *   - Power: 2.6V to 5.5V operation
 *   - Sample Rate: Up to 80 samples per second
 * 
 * Calibration Process:
 *   - Known Weight: Place calibrated weight on scale
 *   - Raw Reading: Obtain raw ADC value with weight
 *   - Zero Reading: Obtain raw ADC value without weight (tare)
 *   - Factor Calculation: (weight / (raw_with - raw_without))
 *   - Application: Multiply raw readings by calibration factor
 *   - Units: Typically grams or kilograms
 * 
 * Taring Operation:
 *   - Zero Point: Establish baseline ADC reading
 *   - Current Load: Measure current weight on scale
 *   - Offset Storage: Store tare offset for subtraction
 *   - Dynamic Tare: Can be performed with weight on scale
 *   - Reset: Tare offset can be cleared to restore absolute readings
 * 
 * Measurement Accuracy:
 *   - Resolution: Depends on load cell capacity and ADC bits
 *   - Linearity: ±0.05% typical for quality load cells
 *   - Temperature: ±0.02% per °C temperature coefficient
 *   - Creep: <0.03% for 30 minutes under load
 *   - Hysteresis: <0.02% for quality sensors
 * 
 * Integration:
 *   - PA Hub: Connected through PA Hub channel for I2C multiplexing
 *   - Data Model: Weight readings stored in global DATA structure
 *   - Tasks: Periodic sampling in dedicated FreeRTOS task
 *   - Calibration: Persistent storage of calibration factors
 *   - MQTT Publishing: Weight data published for remote monitoring
 * 
 * Performance Considerations:
 *   - Conversion Time: 100µs per reading (HX711)
 *   - Sample Rate: Up to 80 SPS with 10Hz filter
 *   - Power Consumption: 1.6mA active, <1µA sleep
 *   - Noise Immunity: 50/60Hz power supply noise rejection
 *   - Settling Time: 500ms after load changes for stable readings
 * 
 * Error Sources:
 *   - Mechanical: Load cell mounting and alignment
 *   - Electrical: Power supply noise and grounding
 *   - Environmental: Temperature variations and vibration
 *   - Calibration: Incorrect calibration weight or procedure
 *   - Overload: Exceeding load cell capacity
 * 
 * Dependencies:
 *   - HX711 Library: Arduino library for HX711 communication
 *   - Arduino.h: Basic types and utilities
 *   - PA Hub: For I2C channel multiplexing
 * 
 * Limitations:
 *   - Single Load Cell: Designed for single load cell configuration
 *   - Capacity Limited: Maximum weight depends on load cell rating
 *   - No Multi-Point: Simple linear calibration only
 *   - Temperature Sensitive: Performance varies with temperature
 *   - Mechanical Setup: Requires proper load cell mounting
 * 
 * Best Practices:
 *   - Calibrate regularly with known weights
 *   - Allow settling time after load changes
 *   - Use shielded cables to reduce noise
 *   - Mount load cell securely to prevent movement
 *   - Monitor for drift and recalibrate as needed
 *   - Protect load cell from overload and shock
 *   - Use appropriate power supply filtering
 * 
 * Safety Considerations:
 *   - Overload Protection: Prevent exceeding load cell capacity
 *   - Mechanical Safety: Secure mounting prevents falling loads
 *   - Electrical Safety: Proper power supply and grounding
 *   - Calibration Verification: Regular checks with known weights
 *   - Failure Monitoring: Detect sensor faults and alert users
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

/**
 * @brief Retrieves raw weight measurement from load cell.
 * 
 * Reads the current weight measurement from the load cell sensor,
 * applies calibration factor and tare offset, and returns the
 * calibrated weight value. This is the primary function for
 * obtaining weight measurements.
 * 
 * Measurement Process:
 *   1. Select PA Hub channel for weight sensor
 *   2. Read raw ADC value from HX711
 *   3. Apply tare offset subtraction
 *   4. Apply calibration factor multiplication
 *   5. Return calibrated weight value
 * 
 * Calibration Application:
 *   - Raw ADC: Uncalibrated 24-bit integer from HX711
 *   - Tare Offset: Zero point offset for taring
 *   - Calibration Factor: Grams per ADC unit
 *   - Final Weight: (raw - tare_offset) * calibration_factor
 * 
 * Units:
 *   - Return Value: Grams (g) or configured weight unit
 *   - Precision: Floating point for fractional grams
 *   - Range: Depends on load cell capacity and calibration
 * 
 * Performance:
 *   - Execution Time: ~100µs for ADC conversion
 *   - Sample Rate: Limited by HX711 conversion rate
 *   - Power Usage: Minimal during measurement
 *   - Blocking: Synchronous ADC read operation
 * 
 * Error Handling:
 *   - Sensor Failure: Returns 0.0 or error value
 *   - Out of Range: Clamped to valid measurement range
 *   - Noise: Digital filtering may be applied
 *   - Calibration: Uses stored calibration factor
 * 
 * Usage Examples:
 *   float weight = scale_get_raw();
 *   if (weight > 0) {
 *       Serial.printf("Weight: %.2f grams\n", weight);
 *   }
 * 
 * Integration:
 *   - Data Model: Updates DATA.weight_g field
 *   - MQTT: Published as sensor data
 *   - Display: Shown on M5Stack LCD
 *   - Tasks: Called periodically by sensor task
 * 
 * @note Function applies calibration and tare automatically.
 *       Requires proper calibration before accurate readings.
 *       HX711 must be properly initialized and connected.
 * 
 * @see scale_tare() - Zero the scale
 * @see scale_set_cal_factor() - Set calibration factor
 */
float scale_get_raw();

/**
 * @brief Performs tare (zeroing) operation on the scale.
 * 
 * Establishes the current load cell reading as the zero point
 * for future measurements. This allows weighing objects by
 * measuring the difference from the tare point.
 * 
 * Taring Process:
 *   1. Read current raw ADC value from load cell
 *   2. Store this value as tare offset
 *   3. Future readings subtract this offset
 *   4. Calibration factor remains unchanged
 * 
 * Use Cases:
 *   - Container Taring: Zero scale with container on it
 *   - Baseline Reset: Reset to zero after measurements
 *   - Dynamic Zeroing: Adjust for environmental changes
 *   - Multiple Measurements: Zero between different loads
 * 
 * Implementation:
 *   - Offset Storage: Internal variable stores tare value
 *   - Persistence: Tare offset maintained until reset
 *   - Reset Method: Call with no load to clear tare
 *   - Thread Safety: Access controlled for concurrent use
 * 
 * Performance:
 *   - Execution Time: < 1ms (simple calculation)
 *   - Memory Usage: Stores single float value
 *   - No ADC Read: Uses existing calibration infrastructure
 * 
 * Usage Examples:
 *   // Tare with container
 *   scale_tare();  // Container weight becomes zero
 *   // Add material and measure
 *   float net_weight = scale_get_raw();
 * 
 * Integration:
 *   - User Interface: Button-triggered taring
 *   - Commands: MQTT command for remote taring
 *   - Automation: Automatic taring in processes
 *   - Data Model: Affects weight readings in DATA
 * 
 * @note Taring should be done with stable load on scale.
 *       Allow time for readings to stabilize before taring.
 *       Tare offset persists until explicitly reset.
 * 
 * @see scale_get_raw() - Get weight with tare applied
 * @see scale_set_cal_factor() - Calibration affects tare behavior
 */
void scale_tare();

/**
 * @brief Sets the calibration factor for weight measurements.
 * 
 * Configures the calibration factor that converts raw ADC readings
 * to weight units. The factor is calculated as (known_weight / raw_reading)
 * and should be determined through proper calibration procedure.
 * 
 * Calibration Procedure:
 *   1. Place known weight on scale
 *   2. Read raw ADC value (without tare)
 *   3. Calculate factor = known_weight / raw_reading
 *   4. Set factor using this function
 *   5. Verify calibration with test weights
 * 
 * Factor Characteristics:
 *   - Units: weight_units / ADC_counts (e.g., grams/ADC_unit)
 *   - Typical Range: 0.001 to 1.0 depending on load cell
 *   - Precision: Float for accurate calibration
 *   - Persistence: Should be stored in configuration
 * 
 * Implementation:
 *   - Factor Storage: Internal variable for calibration
 *   - Application: Multiplied with (raw - tare) in get_raw()
 *   - Validation: No validation (caller responsibility)
 *   - Thread Safety: Access controlled for concurrent updates
 * 
 * Performance:
 *   - Execution Time: < 1ms (simple assignment)
 *   - Memory Usage: Stores single float value
 *   - No ADC Access: Pure configuration function
 * 
 * Usage Examples:
 *   // Calibrate with 100g weight
 *   float raw_with_100g = scale_get_raw();  // Before calibration
 *   scale_set_cal_factor(100.0f / raw_with_100g);
 *   // Now scale_get_raw() returns grams
 * 
 * Integration:
 *   - Configuration Store: Factor saved in CONFIG
 *   - Setup Process: Calibrated during device setup
 *   - Maintenance: Recalibrated periodically
 *   - Diagnostics: Factor checked for validity
 * 
 * @note Factor must be determined through proper calibration.
 *       Incorrect factor leads to inaccurate measurements.
 *       Factor should be recalibrated if load cell changes.
 * 
 * @see scale_get_raw() - Uses calibration factor
 * @see scale_tare() - Independent of calibration factor
 */
void scale_set_cal_factor(float factor);