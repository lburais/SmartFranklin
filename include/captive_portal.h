/**
 * ============================================================================
 * Captive Portal Module - SmartFranklin
 * ============================================================================
 * 
 * File:        captive_portal.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Header file for WiFi captive portal functionality. Provides
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 * 
 * WiFi Access Point:
 * 
 * Web Server Features:
 * 
 * Configuration Options:
 * 
 * Portal Activation:
 * 
 * User Experience:
 * 
 * Security Considerations:
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

/**
 * @brief Starts the WiFi captive portal for device configuration.
 * 
 * Initializes and starts the captive portal access point and web server.
 * Creates an open WiFi network that allows users to connect and configure
 * the device through a web interface. Automatically handles DNS redirection
 * and web page serving for a seamless configuration experience.
 * 
 * Portal Setup Process:
 * 
 * Access Point Configuration:
 * 
 * Web Server Features:
 * 
 * DNS Server:
 * 
 * Configuration Handling:
 * 
 * User Interaction:
 * 
 * Timeout and Exit:
 * 
 * Error Handling:
 * 
 * Security Notes:
 * 
 * @note This function blocks until configuration is complete or timeout.
 * 
 * @see WiFiManager - WiFi connection management
 * @see config_store - Configuration persistence
 */
void captive_portal_start();