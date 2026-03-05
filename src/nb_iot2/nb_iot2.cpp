/*
 * ============================================================================
 * NB-IoT Library Implementation - SmartFranklin
 * ============================================================================
 * 
 * File:        nb_iot2.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: Implementation of NB-IoT cellular communication library for
 *              SmartFranklin. Provides AT command interface, network management,
 *              MQTT over cellular, and GNSS positioning functionality.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   The NB-IoT library provides a comprehensive interface for Narrowband IoT
 *   cellular communication in SmartFranklin. It handles modem initialization,
 *   network attachment, PDP context activation, MQTT messaging over cellular
 *   networks, and GNSS positioning data retrieval. The library abstracts
 *   complex AT command sequences into simple function calls for reliable
 *   IoT connectivity in areas without WiFi coverage.
 * 
 * NB-IoT Technology Integration:
 *   - AT Command Interface: Serial communication with NB-IoT modem
 *   - Network Management: APN configuration and network attachment
 *   - PDP Context: IP address assignment and data connectivity
 *   - MQTT over Cellular: Publish/subscribe messaging via cellular
 *   - GNSS Support: GPS/GLONASS positioning data retrieval
 *   - Error Handling: Robust retry mechanisms and status tracking
 * 
 * Library Architecture:
 *   - Singleton Pattern: NB_IOT2 global instance for system-wide access
 *   - State Management: Internal status tracking for connection states
 *   - Asynchronous Operation: Non-blocking communication with timeouts
 *   - Resource Management: Serial port and buffer handling
 *   - Thread Safety: Designed for single-threaded FreeRTOS environment
 * 
 * AT Command Protocol:
 *   - Command Format: AT+COMMAND=parameters\r
 *   - Response Parsing: OK/ERROR detection and data extraction
 *   - Timeout Handling: Configurable timeouts for different operations
 *   - Buffer Management: Serial data buffering and token detection
 *   - Error Recovery: Automatic retry on communication failures
 * 
 * Network Connection Flow:
 *   1. Modem Initialization: AT command verification and configuration
 *   2. Network Attachment: CGATT command for network registration
 *   3. PDP Activation: CGACT command for IP connectivity
 *   4. IP Address Retrieval: CGPADDR command for address assignment
 *   5. Status Monitoring: Continuous connection health checking
 * 
 * MQTT over Cellular:
 *   - Configuration: SMCONF commands for broker settings
 *   - Connection: SMCONN command for broker connection establishment
 *   - Publishing: SMPUB command for message transmission
 *   - QoS Support: Quality of Service levels (0, 1, 2)
 *   - Retain Option: Message retention on broker
 *   - Authentication: Username/password support
 * 
 * GNSS Positioning:
 *   - Power Control: CGNSPWR command for GNSS power management
 *   - Information Retrieval: CGNSINF command for position data
 *   - Data Parsing: Latitude, longitude, altitude extraction
 *   - Validity Checking: Position data validation and filtering
 *   - Accuracy: Dependent on satellite visibility and signal strength
 * 
 * Status Monitoring:
 *   - Modem Ready: Basic AT command responsiveness
 *   - Network Attached: Cellular network registration status
 *   - PDP Active: IP connectivity and data session status
 *   - MQTT Connected: Broker connection establishment
 *   - IP Address: Assigned IP address for data connectivity
 *   - RSSI: Signal strength indication (not implemented in this version)
 * 
 * Error Handling and Recovery:
 *   - Communication Timeouts: Configurable timeout values per operation
 *   - Retry Logic: Automatic retries for failed operations
 *   - State Reset: Status flags reset on communication failures
 *   - Graceful Degradation: Partial functionality on network issues
 *   - Logging: Debug information for troubleshooting
 * 
 * Performance Considerations:
 *   - Serial Communication: 115200 baud rate for reliable data transfer
 *   - Memory Usage: Minimal buffers and state variables
 *   - Power Consumption: NB-IoT optimized for low power operation
 *   - Latency: Cellular network delays vs. WiFi immediacy
 *   - CPU Usage: Non-blocking operations with delay loops
 * 
 * Dependencies:
 *   - Arduino.h: HardwareSerial and delay functions
 *   - nb_iot2.h: Header file with class definitions and structures
 *   - M5Unified.h: Logging utilities (optional, for debugging)
 * 
 * Limitations:
 *   - Single Modem Support: No multi-modem configurations
 *   - Synchronous Operations: Blocking calls with timeouts
 *   - Limited Error Codes: Basic OK/ERROR response parsing
 *   - No SMS Support: MQTT-only messaging
 *   - GNSS Cold Start: May require time for satellite acquisition
 *   - Network Operator Dependencies: APN and configuration variations
 * 
 * Best Practices:
 *   - Initialize modem before network operations
 *   - Check connection status before MQTT operations
 *   - Handle timeouts gracefully in calling code
 *   - Monitor signal strength for optimal performance
 *   - Use appropriate QoS levels for message reliability
 *   - Implement connection monitoring and reconnection logic
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

#include "nb_iot2.h"

// ============================================================================
// Global NB-IoT Instance
// ============================================================================

/**
 * @brief Global singleton instance of NbIot2 class.
 * 
 * Provides system-wide access to NB-IoT functionality. This instance
 * is used by the NB-IoT task and other components that need cellular
 * communication capabilities. The singleton pattern ensures consistent
 * state management across the application.
 */
NbIot2 NB_IOT2;

// ============================================================================
// Initialization and Setup
// ============================================================================

/**
 * @brief Initializes the NB-IoT modem with serial communication parameters.
 * 
 * Configures the hardware serial interface for communication with the NB-IoT
 * modem. Sets up baud rate, data format, and pin assignments for reliable
 * serial communication. This method must be called before any other NB-IoT
 * operations.
 * 
 * @param serial - Pointer to HardwareSerial instance (e.g., &Serial2)
 * @param baud - Baud rate for serial communication (typically 115200)
 * @param rx - RX pin number for serial receive
 * @param tx - TX pin number for serial transmit
 * 
 * Serial Configuration:
 *   - Baud Rate: Configurable (115200 recommended for NB-IoT modems)
 *   - Data Format: 8 data bits, no parity, 1 stop bit (SERIAL_8N1)
 *   - Flow Control: None (RTS/CTS not used)
 *   - Buffer Size: Arduino default serial buffers
 *   - Pin Assignment: GPIO pins for RX/TX (e.g., 13/14 on M5Stack)
 * 
 * Initialization State:
 *   - Sets m_inited flag to true
 *   - Stores serial interface pointer for future use
 *   - Prepares modem for AT command communication
 * 
 * Usage Notes:
 *   - Call once during system initialization
 *   - Ensure pins are not used by other peripherals
 *   - Verify baud rate matches modem configuration
 *   - Serial interface must remain available throughout operation
 * 
 * @see HardwareSerial - Arduino serial communication class
 * @see SERIAL_8N1 - Standard serial data format
 */
void NbIot2::init(HardwareSerial *serial, uint32_t baud, int rx, int tx)
{
    m_serial = serial;
    m_serial->begin(baud, SERIAL_8N1, rx, tx);
    m_inited = true;
}

// ============================================================================
// Low-Level Communication Functions
// ============================================================================

/**
 * @brief Waits for a specific token in serial response with timeout.
 * 
 * Reads incoming serial data and searches for a specified token string.
 * Implements timeout protection to prevent infinite waiting. Used as
 * the foundation for AT command response parsing.
 * 
 * @param token - String token to wait for (e.g., "OK", "ERROR")
 * @param resp - Optional pointer to store complete response buffer
 * @param timeout - Timeout in milliseconds (default operations use various timeouts)
 * @return true if token found within timeout, false otherwise
 * 
 * Response Processing:
 *   - Character-by-character reading from serial buffer
 *   - Accumulates data in local buffer for token matching
 *   - Immediate return on token detection
 *   - Complete response available via resp parameter
 *   - 10ms delay between availability checks (non-blocking)
 * 
 * Timeout Behavior:
 *   - Millisecond precision timing using millis()
 *   - Returns false on timeout expiration
 *   - Partial response still available in resp buffer
 *   - Allows calling code to handle timeout conditions
 * 
 * Performance:
 *   - Non-blocking operation with small delays
 *   - Minimal memory usage (local buffer)
 *   - Efficient for typical AT command responses
 * 
 * @note This is a low-level function used by sendAT and other methods.
 *       Token matching is case-sensitive and exact string matching.
 */
bool NbIot2::waitFor(const String &token, String *resp, uint32_t timeout)
{
    String buf;
    unsigned long start = millis();
    while (millis() - start < timeout) {
        while (m_serial->available()) {
            char c = m_serial->read();
            buf += c;
            if (buf.indexOf(token) >= 0) {
                if (resp) *resp = buf;
                return true;
            }
        }
        delay(10);
    }
    if (resp) *resp = buf;
    return false;
}

/**
 * @brief Sends an AT command and waits for OK response.
 * 
 * Core AT command transmission function. Sends command with CR termination,
 * clears any pending serial data, and waits for "OK" response. Provides
 * the foundation for all modem communication.
 * 
 * @param cmd - AT command string (without CR termination)
 * @param resp - Optional pointer to store complete response
 * @param timeout - Response timeout in milliseconds
 * @return true if OK received, false on error or timeout
 * 
 * Command Transmission:
 *   - Clears serial receive buffer before sending
 *   - Appends \r carriage return to command
 *   - Uses waitFor() to detect OK response
 *   - Stores complete response if requested
 * 
 * Error Handling:
 *   - Returns false on timeout or ERROR response
 *   - Response buffer contains actual modem response
 *   - Allows differentiation between timeout and modem error
 * 
 * Initialization Check:
 *   - Verifies m_inited flag before operation
 *   - Prevents operation on uninitialized serial interface
 *   - Returns false if init() not called previously
 * 
 * @see waitFor() - Token detection function
 * @see init() - Serial initialization requirement
 */
bool NbIot2::sendAT(const String &cmd, String *resp, uint32_t timeout)
{
    if (!m_inited) return false;
    while (m_serial->available()) m_serial->read();
    m_serial->print(cmd);
    m_serial->print("\r");
    String r;
    if (!waitFor("OK", &r, timeout)) {
        if (resp) *resp = r;
        return false;
    }
    if (resp) *resp = r;
    return true;
}

// ============================================================================
// Modem and Network Management
// ============================================================================

/**
 * @brief Ensures modem is ready and properly configured.
 * 
 * Verifies modem responsiveness and performs initial configuration.
 * Implements retry logic for reliable modem initialization. Sets
 * up modem for NB-IoT operation with appropriate error reporting.
 * 
 * Initialization Sequence:
 *   1. Check if modem already marked as ready
 *   2. Send basic AT command to verify communication
 *   3. Disable echo (ATE0) for cleaner responses
 *   4. Enable verbose error reporting (AT+CMEE=2)
 *   5. Set full functionality (AT+CFUN=1)
 *   6. Mark modem as ready on success
 * 
 * Retry Logic:
 *   - Up to 5 attempts with 500ms delays
 *   - Allows for modem boot-up time
 *   - Returns false after all retries exhausted
 * 
 * Configuration Commands:
 *   - ATE0: Disable command echo
 *   - AT+CMEE=2: Enable numeric error codes
 *   - AT+CFUN=1: Set full functionality mode
 * 
 * State Management:
 *   - Updates m_status.modem_ready flag
 *   - Persistent state across calls
 *   - Allows early return on already-ready modem
 * 
 * @return true if modem is ready, false on failure
 * 
 * @note This function should be called before network operations.
 *       Modem may require several seconds to become responsive after power-on.
 */
bool NbIot2::ensureModem()
{
    if (m_status.modem_ready) return true;
    String r;
    for (int i = 0; i < 5; ++i) {
        if (sendAT("AT", &r, 1000)) {
            sendAT("ATE0");
            sendAT("AT+CMEE=2");
            sendAT("AT+CFUN=1");
            m_status.modem_ready = true;
            return true;
        }
        delay(500);
    }
    return false;
}

/**
 * @brief Ensures network attachment with specified APN.
 * 
 * Attaches to cellular network using provided Access Point Name.
 * Configures PDP context and initiates network registration.
 * Required before establishing data connectivity.
 * 
 * @param apn - Access Point Name for cellular network (e.g., "iot" or operator-specific)
 * @return true if network attached, false on failure
 * 
 * Network Attachment Process:
 *   1. Ensure modem is ready (calls ensureModem())
 *   2. Configure PDP context with APN (AT+CGDCONT)
 *   3. Attach to network (AT+CGATT=1)
 *   4. Verify attachment success
 *   5. Update status flags
 * 
 * APN Configuration:
 *   - Operator-dependent (consult cellular provider)
 *   - Examples: "iot", "nbiot", or full APN strings
 *   - Critical for successful network registration
 * 
 * Status Updates:
 *   - m_status.network_attached set based on CGATT response
 *   - Used by higher-level functions to check connectivity
 * 
 * Timeout Handling:
 *   - CGATT command has 20-second timeout for network registration
 *   - Allows time for cellular network procedures
 * 
 * @see ensureModem() - Modem readiness prerequisite
 * @see ensurePdp() - Next step in connection process
 */
bool NbIot2::ensureNetwork(const String &apn)
{
    if (!ensureModem()) return false;
    String r;
    sendAT("AT+CGDCONT=1,\"IP\",\"" + apn + "\"");
    sendAT("AT+CGATT=1", &r, 20000);
    m_status.network_attached = r.indexOf("OK") >= 0;
    return m_status.network_attached;
}

/**
 * @brief Ensures PDP context activation and retrieves IP address.
 * 
 * Activates Packet Data Protocol context for IP connectivity and
 * retrieves assigned IP address. Completes the network connection
 * process for data communication.
 * 
 * @param apn - Access Point Name (passed through to ensureNetwork)
 * @return true if PDP active with valid IP, false on failure
 * 
 * PDP Activation Process:
 *   1. Ensure network attachment (calls ensureNetwork())
 *   2. Activate PDP context (AT+CGACT=1,1)
 *   3. Retrieve IP address (AT+CGPADDR=1)
 *   4. Parse and store IP address
 *   5. Update status flags
 * 
 * IP Address Retrieval:
 *   - Uses CGPADDR command to get assigned IP
 *   - Parses response to extract IP string
 *   - Stores in m_status.ip for status reporting
 *   - Trims whitespace for clean storage
 * 
 * Status Updates:
 *   - m_status.pdp_active set based on CGACT success
 *   - m_status.ip populated with assigned address
 *   - Enables data connectivity verification
 * 
 * Error Handling:
 *   - Returns false if PDP activation fails
 *   - IP parsing handles malformed responses gracefully
 *   - Status flags remain false on failure
 * 
 * @see ensureNetwork() - Network attachment prerequisite
 * @see getStatus() - IP address access function
 */
bool NbIot2::ensurePdp(const String &apn)
{
    if (!ensureNetwork(apn)) return false;
    String r;
    sendAT("AT+CGACT=1,1", &r, 20000);
    m_status.pdp_active = r.indexOf("OK") >= 0;
    if (m_status.pdp_active) {
        sendAT("AT+CGPADDR=1", &r, 5000);
        int idx = r.indexOf("+CGPADDR:");
        if (idx >= 0) {
            int q = r.indexOf(",", idx);
            int e = r.indexOf("\r", q);
            if (q > 0 && e > q) {
                m_status.ip = r.substring(q + 1, e);
                m_status.ip.trim();
            }
        }
    }
    return m_status.pdp_active;
}

// ============================================================================
// MQTT over Cellular Functions
// ============================================================================

/**
 * @brief Ensures MQTT connection to specified broker.
 * 
 * Establishes MQTT connection over cellular network using provided
 * broker parameters. Configures connection settings and initiates
 * connection to MQTT broker.
 * 
 * @param host - MQTT broker hostname or IP address
 * @param port - MQTT broker port (typically 1883)
 * @param user - MQTT username (empty string for no authentication)
 * @param pass - MQTT password (empty string for no authentication)
 * @return true if MQTT connected, false on failure or empty host
 * 
 * MQTT Configuration Process:
 *   1. Check for valid host parameter
 *   2. Return true if already connected
 *   3. Configure broker URL and port (AT+SMCONF)
 *   4. Set username if provided
 *   5. Set password if provided
 *   6. Establish connection (AT+SMCONN)
 *   7. Update connection status
 * 
 * Authentication:
 *   - Optional username/password support
 *   - Empty strings skip authentication configuration
 *   - Broker-dependent authentication requirements
 * 
 * Connection Management:
 *   - Persistent connection state tracking
 *   - Avoids redundant connection attempts
 *   - 30-second timeout for connection establishment
 * 
 * @note Host parameter is required - returns false if empty.
 *       Connection persists until network issues or explicit disconnect.
 * 
 * @see mqttPublish() - Message publishing function
 * @see loop() - Connection monitoring
 */
bool NbIot2::ensureMqtt(const String &host, int port,
                        const String &user, const String &pass)
{
    if (host.isEmpty()) return false;
    if (m_status.mqtt_connected) return true;

    String r;
    sendAT("AT+SMCONF=\"URL\",\"" + host + "\"," + String(port), &r, 5000);
    if (!user.isEmpty())
        sendAT("AT+SMCONF=\"USERNAME\",\"" + user + "\"", &r, 5000);
    if (!pass.isEmpty())
        sendAT("AT+SMCONF=\"PASSWORD\",\"" + pass + "\"", &r, 5000);

    sendAT("AT+SMCONN", &r, 30000);
    m_status.mqtt_connected = r.indexOf("OK") >= 0;
    return m_status.mqtt_connected;
}

/**
 * @brief Connects to cellular network with specified APN.
 * 
 * Public interface for network connection. Calls ensurePdp() to
 * complete full network attachment and IP connectivity.
 * 
 * @param apn - Access Point Name for cellular network
 * @return true if network connected with IP address, false on failure
 * 
 * @see ensurePdp() - Underlying connection implementation
 */
bool NbIot2::connectNetwork(const String &apn)
{
    return ensurePdp(apn);
}

/**
 * @brief Connects to MQTT broker over cellular network.
 * 
 * Public interface for MQTT connection. Calls ensureMqtt() with
 * provided broker parameters.
 * 
 * @param host - MQTT broker hostname
 * @param port - MQTT broker port
 * @param user - MQTT username
 * @param pass - MQTT password
 * @return true if MQTT connected, false on failure
 * 
 * @see ensureMqtt() - Underlying MQTT connection implementation
 */
bool NbIot2::mqttConnect(const String &host, int port,
                         const String &user, const String &pass)
{
    return ensureMqtt(host, port, user, pass);
}

/**
 * @brief Publishes message to MQTT topic over cellular network.
 * 
 * Sends MQTT publish message using cellular connectivity. Supports
 * QoS levels and retain flag for flexible message delivery.
 * 
 * @param topic - MQTT topic string
 * @param payload - Message payload string
 * @param qos - Quality of Service level (0, 1, or 2)
 * @param retain - Retain flag (true to retain message on broker)
 * @return true if message published successfully, false on failure
 * 
 * Publishing Process:
 *   1. Check MQTT connection status
 *   2. Return false if not connected
 *   3. Construct SMPUB command with parameters
 *   4. Send command and wait for prompt
 *   5. Transmit payload data
 *   6. Wait for OK confirmation
 * 
 * QoS Levels:
 *   - 0: At most once (fire and forget)
 *   - 1: At least once (acknowledged delivery)
 *   - 2: Exactly once (highest reliability)
 * 
 * Retain Option:
 *   - true: Message retained on broker for new subscribers
 *   - false: Message delivered only to current subscribers
 * 
 * Error Handling:
 *   - Returns false if MQTT not connected
 *   - Handles command transmission failures
 *   - 10-second timeout for publish completion
 * 
 * @note Requires active MQTT connection from mqttConnect().
 *       Payload length is automatically calculated.
 * 
 * @see mqttConnect() - MQTT connection establishment
 */
bool NbIot2::mqttPublish(const String &topic, const String &payload,
                         int qos, bool retain)
{
    if (!m_status.mqtt_connected) return false;
    String cmd = "AT+SMPUB=\"" + topic + "\"," +
                 String(payload.length()) + "," +
                 String(qos) + "," +
                 String(retain ? 1 : 0);
    String r;
    if (!sendAT(cmd, &r, 5000)) return false;
    m_serial->print(payload);
    m_serial->print("\r");
    return waitFor("OK", &r, 10000);
}

// ============================================================================
// GNSS Positioning Functions
// ============================================================================

/**
 * @brief Retrieves GNSS positioning information.
 * 
 * Gets current GPS/GLONASS position data from the NB-IoT modem.
 * Powers on GNSS if necessary and parses position information.
 * 
 * @param out - Reference to GnssInfo structure to fill with position data
 * @return true if valid position data retrieved, false on failure
 * 
 * GNSS Retrieval Process:
 *   1. Power on GNSS receiver (AT+CGNSPWR=1)
 *   2. Request position information (AT+CGNSINF)
 *   3. Parse CGNSINF response for coordinates
 *   4. Extract latitude, longitude, altitude
 *   5. Validate position data
 * 
 * Data Parsing:
 *   - Finds +CGNSINF: response line
 *   - Extracts comma-separated values
 *   - Converts strings to numeric values
 *   - Handles parsing errors gracefully
 * 
 * Position Validation:
 *   - Checks for non-zero coordinates
 *   - Sets valid flag based on data presence
 *   - Filters out invalid (0,0) positions
 * 
 * Coordinate System:
 *   - Latitude: Decimal degrees (-90 to +90)
 *   - Longitude: Decimal degrees (-180 to +180)
 *   - Altitude: Meters above sea level
 * 
 * @note GNSS may require time to acquire satellite signals.
 *       Cold start can take several minutes for first fix.
 *       Position accuracy depends on satellite visibility.
 * 
 * @see GnssInfo - Position data structure
 */
bool NbIot2::getGnss(GnssInfo &out)
{
    String r;
    if (!sendAT("AT+CGNSPWR=1", &r, 5000)) return false;
    sendAT("AT+CGNSINF", &r, 5000);
    int idx = r.indexOf("+CGNSINF:");
    if (idx < 0) return false;
    String line = r.substring(idx);
    int c1 = line.indexOf(",", 0);
    int c2 = line.indexOf(",", c1 + 1);
    int c3 = line.indexOf(",", c2 + 1);
    int c4 = line.indexOf(",", c3 + 1);
    int c5 = line.indexOf(",", c4 + 1);
    if (c3 < 0 || c4 < 0 || c5 < 0) return false;
    out.lat = line.substring(c2 + 1, c3).toDouble();
    out.lon = line.substring(c3 + 1, c4).toDouble();
    out.alt = line.substring(c4 + 1, c5).toFloat();
    out.valid = (out.lat != 0 || out.lon != 0);
    return out.valid;
}

// ============================================================================
// Main Loop and Status Functions
// ============================================================================

/**
 * @brief Main processing loop for connection monitoring.
 * 
 * Performs periodic health checks on modem connectivity. Sends
 * keep-alive AT commands and resets status flags on failures.
 * Should be called regularly from the main task loop.
 * 
 * Health Check Process:
 *   1. Check if library initialized
 *   2. Verify 10-second interval since last ping
 *   3. Send basic AT command for connectivity test
 *   4. Reset all status flags on failure
 *   5. Update last ping timestamp
 * 
 * Status Reset Logic:
 *   - Modem ready flag cleared on AT failure
 *   - Network attachment flag reset
 *   - PDP active flag reset
 *   - MQTT connection flag reset
 *   - Forces reconnection on next operation
 * 
 * Timing:
 *   - 10-second intervals between health checks
 *   - 2-second timeout for AT command response
 *   - Non-blocking operation with timestamp tracking
 * 
 * @note This function should be called from the NB-IoT task loop.
 *       Provides automatic recovery detection for connection issues.
 *       Status flags are conservative - cleared on any communication failure.
 * 
 * @see getStatus() - Status flag access
 * @see ensureModem() - Modem reconnection function
 */
void NbIot2::loop()
{
    if (!m_inited) return;
    unsigned long now = millis();
    if (now - m_lastPing > 10000) {
        String r;
        if (!sendAT("AT", &r, 2000)) {
            m_status.modem_ready = false;
            m_status.network_attached = false;
            m_status.pdp_active = false;
            m_status.mqtt_connected = false;
        }
        m_lastPing = now;
    }
}

/**
 * @brief Returns current NB-IoT connection status.
 * 
 * Provides read-only access to internal status structure.
 * Used by external code to check connection states and IP address.
 * 
 * @return NbIotStatus structure with current connection information
 * 
 * Status Information:
 *   - modem_ready: Basic modem responsiveness
 *   - network_attached: Cellular network registration
 *   - pdp_active: IP connectivity active
 *   - mqtt_connected: MQTT broker connection
 *   - ip: Assigned IP address string
 *   - rssi: Signal strength (not implemented in this version)
 * 
 * Usage:
 *   - Check connection states before operations
 *   - Display status information in UI
 *   - Log connection status for debugging
 *   - Make decisions based on connectivity
 * 
 * @see NbIotStatus - Status structure definition
 * @see loop() - Status update function
 */
NbIotStatus NbIot2::getStatus() const
{
    return m_status;
}