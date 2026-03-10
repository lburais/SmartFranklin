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

