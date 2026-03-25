/**
 * ============================================================================
 * Web Dashboard Module - SmartFranklin
 * ============================================================================
 * 
 * File:        web_dashboard.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Header file for the embedded web-server dashboard.  Declares
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 * 
 * Features:
 * 
 * Integration:
 * 
 * Dependencies:
 * 
 * Limitations:
 * 
 * Best practices:
 * 
 * Usage:
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
 * @brief Initialize the HTTP(S) web dashboard.
 *
 * Sets up the internal web server, mounts the filesystem containing the
 * frontend assets, registers URI handlers for status/configuration APIs
 * and, if requested, enables authentication/TLS.  After this call the
 * dashboard will respond to incoming browser requests without further
 * intervention; it runs inside its own task or the Arduino loop depending
 * on the underlying web-server library.
 *
 * Call once during setup() after the network interface has been started.
 * The function is re-entrant; calling it a second time has no effect.
 */
void web_dashboard_init();
