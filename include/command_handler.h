/**
 * ============================================================================
 * Command Handler Module - SmartFranklin
 * ============================================================================
 * 
 * File:        command_handler.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Header file for MQTT command processing and display screen
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 * 
 * MQTT Command Processing:
 * 
 * Display Screen Management:
 * 
 * Command Types Supported:
 * 
 * Integration:
 * 
 * Dependencies:
 * 
 * Limitations:
 * 
 * Best Practices:
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
 * 
 * MQTT Subscriptions:
 * 
 * Display Setup:
 * 
 * Error Handling:
 * 
 * @note This function should be called once during system initialization.
 * 
 * @see command_handle() - Command processing function
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
 * 
 * Supported Command Types:
 * 
 * Error Handling:
 * 
 * Security Considerations:
 * 
 * @note This function is called asynchronously by MQTT callbacks.
 * 
 * @see command_handler_init() - Initialization function
 */
void command_handle(const String &topic, const String &payload);