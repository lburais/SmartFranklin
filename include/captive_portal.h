/**
 * ============================================================================
 * Captive Portal Module - SmartFranklin
 * ============================================================================
 * 
 * File:        captive_portal.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Declaration for the captive-portal DNS helper used when the
 *              SmartFranklin access point is active.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   `captive_portal_start()` starts the wildcard DNS responder that points
 *   every hostname at the AP interface IP. The web dashboard provides the
 *   HTTP content; this header only exposes the DNS start hook.
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
 * @brief Starts the wildcard DNS responder for captive-portal behavior.
 *
 * This function does not create the AP or serve HTML by itself; it only makes
 * DNS queries resolve to `WiFi.softAPIP()` so browsers land on the device.
 *
 * @note Non-blocking helper intended to be called after AP startup.
 */
void captive_portal_start();