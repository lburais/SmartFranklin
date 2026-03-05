/*
 * ============================================================================
 * Command Handler Module - SmartFranklin
 * ============================================================================
 * 
 * File:        command_handler.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Header file for MQTT command processing and display screen
 *              management. Provides functions for initializing command handling,
 *              processing incoming MQTT messages, and managing display screen
 *              selection for the M5Stack interface.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   The command handler module processes MQTT messages received by SmartFranklin
 *   and translates them into system actions. It handles configuration commands,
 *   sensor control, display management, and other remote control operations.
 *   The module also manages display screen selection for the M5Stack LCD,
 *   allowing remote switching between different information displays.
 * 
 * MQTT Command Processing:
 *   - Topic Subscription: Listens to smartfranklin/commands topic
 *   - Payload Parsing: JSON or simple string command interpretation
 *   - Action Execution: Triggers appropriate system responses
 *   - Error Handling: Invalid command logging and graceful failure
 *   - Security: Command validation and authorization checks
 * 
 * Display Screen Management:
 *   - Screen Selection: Numeric screen ID management (0-9 range)
 *   - Display Switching: LCD content changes based on screen ID
 *   - Persistent State: Screen selection maintained across reboots
 *   - Remote Control: MQTT commands can change display screens
 *   - Default Screen: Fallback screen when invalid ID specified
 * 
 * Command Types Supported:
 *   - Configuration: System settings and parameter updates
 *   - Sensor Control: Enable/disable sensors and adjust parameters
 *   - Display Control: Screen switching and display settings
 *   - System Control: Restart, reset, and maintenance operations
 *   - Status Requests: Information queries and status reporting
 * 
 * Integration:
 *   - MQTT Layer: Receives messages from sf_mqtt::subscribe callbacks
 *   - Data Model: Updates global DATA structure for system state
 *   - Display System: Controls M5Stack LCD content and layout
 *   - Task Coordination: Signals other tasks for configuration changes
 * 
 * Dependencies:
 *   - Arduino.h: String class and basic types
 *   - mqtt_layer.h: MQTT subscription and message handling
 *   - data_model.h: Global data structure access
 *   - M5Unified.h: Display and UI management
 * 
 * Limitations:
 *   - Single Command Processing: Commands processed sequentially
 *   - No Command Queuing: Real-time execution without buffering
 *   - Limited Validation: Basic command format checking
 *   - No Authentication: Commands accepted from any MQTT source
 *   - Screen Range: Limited to 10 screens (0-9)
 * 
 * Best Practices:
 *   - Use descriptive command names and payloads
 *   - Implement command validation before execution
 *   - Provide feedback through MQTT status topics
 *   - Document command formats for API consumers
 *   - Test commands thoroughly before production deployment
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
 * @brief Initializes the command handler system.
 * 
 * Sets up MQTT subscriptions for command processing and initializes
 * display screen management. Must be called during system startup
 * before command processing can begin.
 * 
 * Initialization Tasks:
 *   - Subscribe to MQTT command topics
 *   - Set up command callback functions
 *   - Initialize display screen state
 *   - Load saved screen preferences
 *   - Register command processing handlers
 * 
 * MQTT Subscriptions:
 *   - smartfranklin/commands: Main command topic
 *   - smartfranklin/display/screen: Screen control topic
 *   - Additional topics as needed for specific commands
 * 
 * Display Setup:
 *   - Load default screen from configuration
 *   - Initialize screen switching mechanism
 *   - Set up display update callbacks
 * 
 * Error Handling:
 *   - Logs subscription failures
 *   - Continues with reduced functionality
 *   - Provides startup status indication
 * 
 * @note This function should be called once during system initialization.
 *       Command processing is not available until this function completes.
 * 
 * @see command_handle() - Command processing function
 * @see command_get_display_screen() - Screen management function
 */
void command_handler_init();

/**
 * @brief Processes incoming MQTT command messages.
 * 
 * Main command processing function called by MQTT callback when
 * messages are received on subscribed command topics. Parses
 * command payloads and executes appropriate system actions.
 * 
 * @param topic - MQTT topic string where command was received
 * @param payload - Command payload string containing instructions
 * 
 * Command Processing Flow:
 *   1. Validate topic and payload format
 *   2. Parse command type and parameters
 *   3. Execute appropriate action handler
 *   4. Log command execution result
 *   5. Send acknowledgment if required
 * 
 * Supported Command Types:
 *   - "screen X": Switch to display screen X (0-9)
 *   - "restart": System restart command
 *   - "config": Configuration update commands
 *   - "sensor": Sensor control commands
 *   - "status": Status query commands
 * 
 * Error Handling:
 *   - Invalid commands logged and ignored
 *   - Malformed payloads handled gracefully
 *   - System state preserved on errors
 *   - MQTT acknowledgments sent for feedback
 * 
 * Security Considerations:
 *   - No authentication in current implementation
 *   - Commands accepted from any MQTT source
 *   - Consider adding authorization for production use
 * 
 * @note This function is called asynchronously by MQTT callbacks.
 *       Keep processing lightweight to avoid blocking other operations.
 * 
 * @see command_handler_init() - Initialization function
 */
void command_handle(const String &topic, const String &payload);

/**
 * @brief Returns the current display screen identifier.
 * 
 * Provides access to the currently selected display screen number.
 * Used by display rendering code to determine which screen content
 * to show on the M5Stack LCD.
 * 
 * @return int - Current display screen ID (0-9 range)
 * 
 * Screen ID Usage:
 *   - 0: Default/main screen (system overview)
 *   - 1: Sensor data screen
 *   - 2: Network status screen
 *   - 3: Configuration screen
 *   - 4-9: Reserved for future screens
 * 
 * State Management:
 *   - Screen ID persists across function calls
 *   - Changed by command_handle() for remote control
 *   - Used by display rendering tasks
 *   - Thread-safe access to shared state
 * 
 * Display Integration:
 *   - Called by UI rendering code
 *   - Determines screen layout and content
 *   - Supports dynamic screen switching
 *   - Enables remote display control
 * 
 * @note Screen IDs are limited to 0-9 range.
 *       Invalid screen selections default to screen 0.
 * 
 * @see command_handle() - Screen switching function
 */
int command_get_display_screen();