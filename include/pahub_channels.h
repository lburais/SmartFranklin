/*
 * ============================================================================
 * PA Hub Channels Module - SmartFranklin
 * ============================================================================
 * 
 * File:        pahub_channels.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Header file defining PA Hub (Port Address Hub) channel
 *              assignments and timing constants for I2C device multiplexing.
 *              Configures channel allocation for distance sensor, weight sensor,
 *              and other peripherals connected through the PA Hub multiplexer.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   The PA Hub channels module provides configuration for the M5Stack PA Hub,
 *   an I2C multiplexer that allows multiple I2C devices to share the same bus
 *   by assigning them to different channels. This enables SmartFranklin to
 *   connect multiple sensors and peripherals that might have conflicting I2C
 *   addresses. The module defines channel assignments, device mappings, and
 *   sampling periods for each connected device.
 * 
 * PA Hub Technology:
 *   - Device: TCA9548A I2C multiplexer or compatible
 *   - Function: 8-channel I2C bus multiplexer
 *   - Addressing: I2C address 0x70 (configurable)
 *   - Channels: 8 independent I2C buses (0-7)
 *   - Control: Channel selection via I2C write commands
 *   - Isolation: Each channel electrically isolated
 *   - Power: 2.7V to 5.5V operation
 * 
 * Channel Discovery Strategy:
 *   - Channels are discovered dynamically during startup enumeration
 *   - Distance/weight units are bound to detected channels at runtime
 *   - Channels 0-7 are scanned when PA Hub is present
 *   - Internal devices (IMU/RTC) use dedicated buses
 *   - Dynamic binding avoids hardcoded channel assumptions
 * 
 * Device Integration:
 *   - Distance Sensor: Discovered on direct I2C or PA Hub channel
 *   - Weight Sensor: Discovered on direct I2C or PA Hub channel
 *   - IMU Sensor: Internal MPU6886, no PA Hub required
 *   - RTC Module: Internal BM8563, no PA Hub required
 *   - Future Devices: Compatible units can be detected on remaining channels
 * 
 * Sampling Periods:
 *   - Weight Sensor: 1000ms (1Hz) - stable readings for load cells
 *   - Tilt Sensor: 1000ms (1Hz) - sufficient for orientation changes
 *   - RTC Update: 1000ms (1Hz) - time synchronization frequency
 *   - Distance Sensor: 1000ms (1Hz) - balance responsiveness vs. power
 *   - Rationale: Conservative periods to minimize power consumption
 *   - Adjustability: Periods can be modified based on application needs
 * 
 * Configuration Management:
 *   - Runtime Discovery: Startup scan determines active channel mappings
 *   - Documentation: Header documents PA Hub address and timing constants
 *   - Expansion: Unused channels remain available for new devices
 *   - Compatibility: Dynamic mapping adapts to wiring changes
 * 
 * Hardware Considerations:
 *   - Wiring: Each device connected to appropriate PA Hub channel pins
 *   - Pull-ups: I2C pull-up resistors required on each channel
 *   - Power: Devices powered from PA Hub power pins
 *   - Isolation: Channel switching prevents bus conflicts
 *   - Debugging: Channel selection can be monitored for troubleshooting
 * 
 * Performance Impact:
 *   - Switching Overhead: Small delay when changing channels
 *   - Bus Speed: Standard 100kHz I2C speed maintained
 *   - Power Consumption: Minimal additional power for multiplexer
 *   - Latency: Channel switching adds microseconds to transactions
 *   - Throughput: Sequential access to multiple devices
 * 
 * Integration Points:
 *   - Sensor Tasks: Use channel constants for device access
 *   - I2C Library: Channel selection before device communication
 *   - Power Management: PA Hub power control for low-power modes
 *   - Diagnostics: Channel status monitoring for fault detection
 * 
 * Dependencies:
 *   - Arduino.h: Basic defines and types
 *   - Wire.h: I2C communication library
 *   - PA Hub Hardware: Physical multiplexer device
 * 
 * Limitations:
 *   - Channel Count: Maximum 8 devices per PA Hub
 *   - I2C Speed: Limited to 400kHz maximum
 *   - Address Conflicts: Still possible within same channel
 *   - Power Budget: Total device power must not exceed PA Hub limits
 *   - Single Hub: No cascading for more than 8 devices
 * 
 * Best Practices:
 *   - Document channel assignments clearly
 *   - Test device connections on each channel
 *   - Use appropriate pull-up resistors
 *   - Monitor for I2C bus errors
 *   - Keep channel switching to minimum
 *   - Plan for future device additions
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

// ============================================================================
// PA Hub I2C Address Configuration
// ============================================================================

/**
 * @brief I2C address for the PA Hub multiplexer device.
 * 
 * The 7-bit I2C address used to communicate with the TCA9548A
 * or compatible I2C multiplexer. This address must match the
 * hardware configuration (A0, A1, A2 pins) of the PA Hub.
 * 
 * Default: 0x70 (standard address with A0-A2 = GND)
 * Range: 0x70 to 0x77 depending on address pin configuration
 * Hardware: Set by A0-A2 pins on TCA9548A device
 * Fixed: Should not be changed without hardware modification
 */
#define PAHUB_ADDRESS       0x70

// ============================================================================
// PA Hub Channel Discovery
// ============================================================================

/**
 * @brief PA Hub channel mapping is discovered dynamically at runtime.
 *
 * SmartFranklin now uses HW I2C enumeration to detect the active PA Hub
 * channels for connected units (distance, weight, etc.) instead of fixed
 * compile-time channel constants.
 */

// ============================================================================
// Sensor Sampling Periods (milliseconds)
// ============================================================================

/**
 * @brief Sampling period for weight sensor measurements.
 * 
 * Time interval between weight sensor readings in milliseconds.
 * Conservative 1-second interval allows load cell stabilization
 * and provides stable weight measurements.
 * 
 * Value: 1000ms (1 second)
 * Frequency: 1 Hz
 * Rationale: Load cells require settling time for accurate readings
 * Power Impact: Lower frequency reduces ADC and processing load
 * Adjustability: Can be decreased for faster response if needed
 */
#define PERIOD_WEIGHT       60000

/**
 * @brief Sampling period for tilt/orientation measurements.
 * 
 * Time interval between IMU orientation readings in milliseconds.
 * 1-second interval sufficient for most orientation monitoring
 * applications with good balance of responsiveness and power efficiency.
 * 
 * Value: 1000ms (1 second)
 * Frequency: 1 Hz
 * Rationale: Orientation changes typically gradual, not requiring high frequency
 * Power Impact: Conservative sampling reduces IMU processing load
 * Adjustability: Can be increased for slower applications or decreased for dynamic use
 */
#define PERIOD_TILT         60000

/**
 * @brief Sampling period for RTC time synchronization.
 * 
 * Time interval between RTC time reads in milliseconds.
 * 1-second updates provide good time synchronization without
 * excessive I2C bus traffic.
 * 
 * Value: 1000ms (1 second)
 * Frequency: 1 Hz
 * Rationale: Time changes slowly, high precision not required
 * Power Impact: Minimal impact as RTC reads are low power
 * Adjustability: Can be increased for less frequent time checks
 */
#define PERIOD_RTC          60000

/**
 * @brief Sampling period for distance sensor measurements.
 * 
 * Time interval between distance/proximity readings in milliseconds.
 * 1-second interval balances measurement accuracy with power consumption
 * for typical proximity monitoring applications.
 * 
 * Value: 1000ms (1 second)
 * Frequency: 1 Hz
 * Rationale: Distance changes relatively slowly in most applications
 * Power Impact: Ultrasonic sensors can be power-hungry, so conservative sampling
 * Adjustability: Can be decreased for dynamic distance tracking applications
 */
#define PERIOD_DISTANCE     60000