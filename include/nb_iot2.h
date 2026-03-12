/*
 * ============================================================================
 * NB-IoT Module - SmartFranklin
 * ============================================================================
 * 
 * File:        nb_iot2.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Header file for NB-IoT (Narrowband IoT) cellular connectivity.
 *              Provides LTE-M/NB-IoT modem control, network attachment, MQTT
 *              communication over cellular networks, and GNSS positioning.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   The NB-IoT module enables SmartFranklin to communicate over cellular
 *   networks using NB-IoT/LTE-M technology. This provides reliable, low-power
 *   connectivity for remote IoT applications where WiFi or Ethernet is
 *   unavailable. The module handles modem initialization, network registration,
 *   MQTT messaging over cellular, and optional GNSS positioning for location
 *   services.
 * 
 * NB-IoT Technology:
 *   - Standard: 3GPP Release 13+ NB-IoT specifications
 *   - Frequency Bands: 700MHz, 800MHz, 900MHz (region dependent)
 *   - Data Rates: Up to 26kbps downlink, 66kbps uplink
 *   - Power Consumption: Ultra-low power for battery operation
 *   - Coverage: Excellent indoor penetration and rural coverage
 *   - Latency: 1.5-10 seconds typical (depending on network)
 *   - Battery Life: Years on single battery for low-data applications
 * 
 * Module Architecture:
 *   - Modem Control: AT command interface to cellular modem
 *   - Network Management: APN configuration and PDP context setup
 *   - MQTT over Cellular: Direct MQTT communication over cellular data
 *   - GNSS Integration: GPS/GLONASS positioning capabilities
 *   - Status Monitoring: Real-time connection and network status
 *   - Error Recovery: Automatic reconnection and fault handling
 * 
 * Cellular Connectivity:
 *   - Modem Types: Quectel BG96, SIMCom SIM7000, or compatible
 *   - SIM Card: Standard nano SIM with cellular data plan
 *   - APN Settings: Operator-specific Access Point Name configuration
 *   - Network Registration: Automatic network attachment and roaming
 *   - Signal Quality: RSSI monitoring and network selection
 *   - Data Connection: PDP context establishment for IP connectivity
 * 
 * MQTT over Cellular:
 *   - Direct MQTT: Native MQTT protocol over TCP/IP
 *   - Broker Connection: Standard MQTT brokers accessible via cellular
 *   - QoS Support: Quality of Service levels 0, 1, 2
 *   - Authentication: Username/password support
 *   - Keep-Alive: Connection maintenance with ping/pong
 *   - Message Publishing: Send sensor data and status updates
 * 
 * GNSS Positioning:
 *   - GPS/GLONASS: Satellite positioning system integration
 *   - Accuracy: 2.5-5 meters typical (open sky)
 *   - Update Rate: Configurable position update frequency
 *   - Power Management: GNSS can be powered on/off for power saving
 *   - Location Data: Latitude, longitude, altitude reporting
 *   - Cold Start: Initial position acquisition time
 * 
 * Power Management:
 *   - PSM (Power Saving Mode): Deep sleep with network registration maintained
 *   - eDRX (Extended Discontinuous Reception): Reduced paging cycle
 *   - Connected Mode: Active data transmission with higher power
 *   - Idle Mode: Registered but not transmitting, moderate power
 *   - Power Off: Complete modem shutdown for maximum power saving
 * 
 * Integration:
 *   - Configuration Store: Cellular settings from CONFIG structure
 *   - Data Model: GNSS position updates to global DATA
 *   - MQTT Integration: Cellular MQTT as backup to WiFi MQTT
 *   - Status Display: Network status in M5Stack interface
 *   - Error Handling: Cellular failures logged and handled gracefully
 * 
 * Dependencies:
 *   - HardwareSerial: Arduino serial communication for AT commands
 *   - Arduino.h: String class and basic utilities
 *   - Modem Firmware: Compatible NB-IoT modem with MQTT support
 * 
 * Limitations:
 *   - Regional Coverage: NB-IoT availability varies by country/operator
 *   - Data Volume: Limited data plans suitable for sensor applications
 *   - Speed: Low bandwidth compared to WiFi/LTE
 *   - SIM Requirements: Data-enabled SIM card and active cellular plan
 *   - Hardware Cost: Additional cost for NB-IoT modem and SIM
 *   - Cold Start Time: GNSS position acquisition can take 30+ seconds
 * 
 * Best Practices:
 *   - Use appropriate data plans for your usage patterns
 *   - Implement power-saving modes for battery-powered applications
 *   - Monitor signal quality and handle poor coverage gracefully
 *   - Test with your specific cellular operator and SIM card
 *   - Implement retry logic for network operations
 *   - Use GNSS only when needed to conserve power
 *   - Keep firmware updated for security and feature improvements
 * 
 * Security Considerations:
 *   - SIM Card Security: Protect physical SIM card access
 *   - Network Security: Cellular networks provide transport security
 *   - MQTT Security: Use TLS when available for MQTT connections
 *   - Data Encryption: Encrypt sensitive data before transmission
 *   - Access Control: Implement proper MQTT topic authorization
 *   - Firmware Security: Keep modem firmware updated
 * 
 * ============================================================================
 * MIT License
 * ============================================================================
 * Copyright (c) 2026 Laurent Burais
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in this software and associated documentation files (the "Software"), to deal
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
 * @brief Status structure for NB-IoT modem and network state.
 * 
 * Contains comprehensive status information about the NB-IoT modem,
 * network connection, and MQTT connectivity. Updated by the NbIot2
 * class and used for monitoring cellular connectivity health.
 */
struct NbIotStatus {
    /**
     * @brief Modem readiness flag.
     * 
     * Indicates whether the NB-IoT modem is powered on, initialized,
     * and responding to AT commands. False indicates modem failure
     * or communication issues.
     * 
     * Default: false (modem not ready)
     * Update: Set by modem initialization and health checks
     * Critical: Must be true for any cellular functionality
     */
    bool modem_ready = false;

    /**
     * @brief Network attachment status.
     * 
     * Shows whether the modem is successfully attached to the cellular
     * network and can make/receive calls and data connections.
     * 
     * Default: false (not attached)
     * Update: Set during network registration process
     * Network States: Attached allows data connectivity
     */
    bool network_attached = false;

    /**
     * @brief PDP context activation status.
     * 
     * Indicates whether the Packet Data Protocol context is active,
     * providing IP connectivity for data transmission. Required for
     * MQTT and internet connectivity.
     * 
     * Default: false (PDP not active)
     * Update: Set when PDP context is established with APN
     * Data Ready: True enables IP-based communication
     */
    bool pdp_active = false;

    /**
     * @brief MQTT connection status over cellular.
     * 
     * Shows whether MQTT client is connected to the configured broker
     * using cellular data connection. Independent of WiFi MQTT status.
     * 
     * Default: false (MQTT not connected)
     * Update: Set by MQTT connection establishment
     * Messaging: True enables publish/subscribe over cellular
     */
    bool mqtt_connected = false;

    /**
     * @brief Cellular network operator name.
     * 
     * Human-readable name of the cellular network operator currently
     * registered with the modem. Useful for network identification
     * and troubleshooting.
     * 
     * Format: Operator name string (e.g., "Vodafone", "AT&T")
     * Default: Empty string (no operator identified)
     * Update: Set during network registration
     */
    String operator_name;

    /**
     * @brief Assigned IP address from cellular network.
     * 
     * IP address assigned by the cellular network through the PDP context.
     * Used for network diagnostics and connectivity verification.
     * 
     * Format: IPv4 dotted decimal (e.g., "10.123.45.67")
     * Default: Empty string (no IP assigned)
     * Update: Set when PDP context is activated
     */
    String ip;

    /**
     * @brief Received Signal Strength Indicator.
     * 
     * Signal quality measurement in dBm, indicating cellular signal strength.
     * Lower (more negative) values indicate weaker signals. Used for
     * network quality assessment and troubleshooting.
     * 
     * Range: Typically -50 dBm (excellent) to -120 dBm (poor)
     * Units: dBm (decibels relative to 1 milliwatt)
     * Default: 0 (no measurement)
     * Update: Periodic signal strength queries
     */
    int rssi = 0;
};

/**
 * @brief GNSS (Global Navigation Satellite System) information structure.
 * 
 * Contains positioning data from the integrated GNSS receiver.
 * Provides latitude, longitude, and altitude for location services.
 * Used when GPS/GLONASS positioning is required.
 */
struct GnssInfo {
    /**
     * @brief Position validity flag.
     * 
     * Indicates whether the GNSS position data is valid and current.
     * False indicates no satellite fix or position unavailable.
     * 
     * Default: false (position not valid)
     * Update: Set when valid position is acquired
     * Validity: True guarantees lat/lon/alt are accurate
     */
    bool valid = false;

    /**
     * @brief Latitude coordinate in decimal degrees.
     * 
     * North-south position coordinate from GNSS receiver.
     * Positive values indicate north latitude, negative south.
     * 
     * Range: -90.0 to +90.0 degrees
     * Precision: Typically 6-8 decimal places
     * Default: 0.0 (invalid position)
     * Update: Set when GNSS position is acquired
     */
    double lat = 0;

    /**
     * @brief Longitude coordinate in decimal degrees.
     * 
     * East-west position coordinate from GNSS receiver.
     * Positive values indicate east longitude, negative west.
     * 
     * Range: -180.0 to +180.0 degrees
     * Precision: Typically 6-8 decimal places
     * Default: 0.0 (invalid position)
     * Update: Set when GNSS position is acquired
     */
    double lon = 0;

    /**
     * @brief Altitude above mean sea level.
     * 
     * Height above sea level in meters from GNSS receiver.
     * Positive values indicate above sea level.
     * 
     * Units: Meters (m)
     * Range: -500 to +10000 meters (typical)
     * Precision: Typically 1-2 meters
     * Default: 0.0 (invalid altitude)
     * Update: Set when GNSS position is acquired
     */
    float alt = 0;
};

/**
 * @brief NB-IoT cellular connectivity class.
 * 
 * Provides complete NB-IoT modem control, network management, MQTT
 * communication, and GNSS positioning. Handles all aspects of cellular
 * connectivity for SmartFranklin's remote communication needs.
 */
class NbIot2 {
public:
    /**
     * @brief Initializes the NB-IoT modem with serial communication.
     * 
     * Sets up serial communication with the NB-IoT modem and performs
     * initial modem configuration. Must be called before using other
     * cellular functions.
     * 
     * @param serial - HardwareSerial instance for modem communication
     * @param baud - Baud rate for serial communication (typically 115200)
     * @param rx - RX pin number for serial communication
     * @param tx - TX pin number for serial communication
     * 
     * Initialization Process:
     *   1. Configure serial pins and baud rate
     *   2. Power on and reset modem if necessary
     *   3. Send basic AT commands to verify communication
     *   4. Configure modem for NB-IoT operation
     *   5. Set up error handling and timeouts
     *   6. Mark modem as initialized
     * 
     * Hardware Requirements:
     *   - Compatible NB-IoT modem (Quectel BG96, SIMCom SIM7000, etc.)
     *   - Proper power supply (3.3V-5V depending on modem)
     *   - Antenna connection for cellular and GNSS
     *   - SIM card inserted with valid cellular plan
     * 
     * Error Handling:
     *   - Serial communication failures logged
     *   - Modem not responding handled gracefully
     *   - Invalid pin configurations detected
     * 
     * Performance:
     *   - Initialization Time: 5-15 seconds (modem boot time)
     *   - Power Consumption: Modem power-on current draw
     *   - Memory Usage: Serial buffer allocation
     * 
     * Usage Notes:
     *   - Call once during system initialization
     *   - Ensure modem hardware is properly connected
     *   - SIM card must be inserted before calling
     *   - Antenna connection required for functionality
     * 
     * @note Blocking operation during modem initialization.
     *       Hardware connections must be correct for success.
     *       Modem may require several seconds to become responsive.
     * 
     * @see connectNetwork() - Establish network connection after init
     */
    void init(HardwareSerial *serial, uint32_t baud, int rx, int tx);

    /**
     * @brief Connects to cellular network with specified APN.
     * 
     * Establishes cellular network connection using the provided Access
     * Point Name. Handles network registration and PDP context activation
     * for data connectivity.
     * 
     * @param apn - Access Point Name for cellular network connection
     * @return true if network connection successful, false on error
     * 
     * Connection Process:
     *   1. Verify modem is ready and initialized
     *   2. Set APN configuration for network operator
     *   3. Register with cellular network
     *   4. Activate PDP context for data connectivity
     *   5. Verify IP address assignment
     *   6. Update connection status
     * 
     * APN Configuration:
     *   - Operator Specific: Contact cellular provider for correct APN
     *   - Format: String like "iot" or "company.apn.operator.com"
     *   - Authentication: May require username/password (not implemented)
     *   - Examples: "iot", "nbiot", "internet"
     * 
     * Network Registration:
     *   - Automatic: Modem searches for available networks
     *   - Operator Selection: May prefer specific operators
     *   - Roaming: Can connect to roaming networks if allowed
     *   - Signal Requirements: Minimum signal strength required
     * 
     * Error Handling:
     *   - Network registration failures with retry logic
     *   - Invalid APN configurations detected
     *   - Poor signal conditions handled gracefully
     *   - SIM card issues reported
     * 
     * Performance:
     *   - Connection Time: 30-120 seconds (network dependent)
     *   - Power Usage: Higher during network search and registration
     *   - Reliability: Depends on cellular coverage and signal quality
     * 
     * Usage Notes:
     *   - Requires valid SIM card with data plan
     *   - APN must match cellular provider requirements
     *   - Good cellular signal recommended for reliable connection
     *   - Function may take significant time to complete
     * 
     * @note Blocking operation during network connection.
     *       Requires good cellular coverage for success.
     *       APN settings are operator-specific and critical.
     * 
     * @see init() - Required initialization before connection
     * @see mqttConnect() - Establish MQTT connection after network
     */
    bool connectNetwork(const String &apn);

    /**
     * @brief Connects MQTT client to broker over cellular network.
     * 
     * Establishes MQTT connection to the specified broker using cellular
     * data connectivity. Enables publish/subscribe messaging over NB-IoT.
     * 
     * @param host - MQTT broker hostname or IP address
     * @param port - MQTT broker port number (typically 1883)
     * @param user - Username for broker authentication (optional)
     * @param pass - Password for broker authentication (optional)
     * @return true if MQTT connection successful, false on error
     * 
     * Connection Process:
     *   1. Verify cellular network connection is active
     *   2. Configure MQTT client parameters
     *   3. Establish TCP connection to MQTT broker
     *   4. Perform MQTT CONNECT handshake
     *   5. Handle authentication if credentials provided
     *   6. Update MQTT connection status
     * 
     * Broker Configuration:
     *   - Host: Domain name or IP address of MQTT broker
     *   - Port: Standard MQTT (1883) or secure MQTT (8883)
     *   - Authentication: Username/password if required
     *   - Client ID: Auto-generated for session management
     * 
     * MQTT Features:
     *   - QoS Support: Quality of Service levels 0, 1, 2
     *   - Keep-Alive: Automatic ping/pong for connection maintenance
     *   - Clean Session: Session state management
     *   - Last Will: Death certificates (not implemented)
     * 
     * Error Handling:
     *   - Network connectivity issues handled
     *   - Broker unreachable conditions detected
     *   - Authentication failures reported
     *   - Connection timeouts with retry capability
     * 
     * Performance:
     *   - Connection Time: 5-30 seconds (network dependent)
     *   - Data Usage: Minimal for connection establishment
     *   - Reliability: Depends on cellular network stability
     * 
     * Usage Notes:
     *   - Requires active cellular network connection
     *   - Broker must be accessible from cellular network
     *   - Credentials required if broker has authentication
     *   - Function may block during connection attempts
     * 
     * @note Requires cellular network connection from connectNetwork().
     *       Blocking operation during MQTT connection establishment.
     *       Broker must be reachable through cellular data network.
     * 
     * @see connectNetwork() - Required network connection
     * @see mqttPublish() - Send messages after connection
     */
    bool mqttConnect(const String &host, int port,
                     const String &user, const String &pass);

    /**
     * @brief Publishes message to MQTT topic over cellular.
     * 
     * Sends MQTT message to specified topic using cellular connectivity.
     * Provides QoS and retain options for message delivery guarantees.
     * 
     * @param topic - MQTT topic string for message publication
     * @param payload - Message payload as string data
     * @param qos - Quality of Service level (0, 1, or 2)
     * @param retain - Retain flag for message persistence
     * @return true if message published successfully, false on error
     * 
     * Publishing Process:
     *   1. Verify MQTT connection is active
     *   2. Format MQTT PUBLISH packet
     *   3. Send message over cellular data connection
     *   4. Wait for acknowledgment based on QoS level
     *   5. Handle delivery confirmations and retries
     * 
     * QoS Levels:
     *   - 0: At most once (fire and forget)
     *   - 1: At least once (acknowledged delivery)
     *   - 2: Exactly once (guaranteed single delivery)
     * 
     * Error Conditions:
     *   - No MQTT connection established
     *   - Network connectivity issues
     *   - Invalid topic or payload format
     *   - QoS level not supported
     * 
     * Performance:
     *   - Transmission Time: 1-10 seconds (cellular latency)
     *   - Data Usage: Payload size + MQTT overhead
     *   - Reliability: Depends on QoS level and network conditions
     * 
     * Usage Examples:
     *   mqttPublish("smartfranklin/sensor/temp", "23.5", 1, false);
     *   mqttPublish("status", "online", 0, true);
     * 
     * @note Requires MQTT connection from mqttConnect().
     *       Cellular latency affects delivery time.
     *       QoS 2 has highest reliability but slowest delivery.
     * 
     * @see mqttConnect() - Establish MQTT connection
     * @see loop() - Maintain connection in main loop
     */
    bool mqttPublish(const String &topic, const String &payload,
                     int qos = 0, bool retain = false);

    /**
     * @brief Main processing loop for NB-IoT maintenance.
     * 
     * Performs periodic maintenance tasks for cellular connectivity,
     * including connection health checks, ping/pong for MQTT keep-alive,
     * and status updates. Should be called regularly from main program loop.
     * 
     * Maintenance Tasks:
     *   1. Check modem responsiveness with periodic AT commands
     *   2. Monitor network registration status
     *   3. Maintain MQTT connection with keep-alive pings
     *   4. Update signal strength and network information
     *   5. Handle reconnection logic for failed connections
     *   6. Process any pending modem responses
     * 
     * Timing Considerations:
     *   - Call Frequency: Every 1-10 seconds depending on requirements
     *   - MQTT Keep-Alive: Typically every 60 seconds
     *   - Network Checks: Periodic but not too frequent
     *   - Balance: Performance vs. power consumption
     * 
     * Error Recovery:
     *   - Automatic reconnection on connection loss
     *   - Modem reset on unresponsiveness
     *   - Network re-registration on detachment
     *   - MQTT reconnection on broker disconnection
     * 
     * Performance:
     *   - Execution Time: < 100ms per call (typically)
     *   - CPU Usage: Minimal when no actions needed
     *   - Power Impact: Small periodic transmissions
     *   - Memory Usage: Minimal additional overhead
     * 
     * Usage Pattern:
     *   void loop() {
     *       NB_IOT2.loop();
     *       // Other processing...
     *   }
     * 
     * Integration:
     *   - Main Loop: Call from Arduino loop() function
     *   - Task Scheduling: Can be called from FreeRTOS task
     *   - Event Driven: Non-blocking operation
     *   - Status Updates: Updates internal status structure
     * 
     * @note Should be called regularly for connection maintenance.
     *       Non-blocking operation suitable for main loop integration.
     *       Handles all automatic reconnection and health monitoring.
     * 
     * @see getStatus() - Retrieve current status information
     */
    void loop();

    /**
     * @brief Retrieves current GNSS position information.
     * 
     * Gets the latest position data from the integrated GNSS receiver.
     * Returns latitude, longitude, and altitude if position is available.
     * GNSS must be enabled and have satellite fix for valid data.
     * 
     * @param out - GnssInfo structure to fill with position data
     * @return true if valid position retrieved, false if no position available
     * 
     * GNSS Operation:
     *   1. Check if GNSS is powered on and operational
     *   2. Query current position from GNSS receiver
     *   3. Parse NMEA or binary position data
     *   4. Validate position accuracy and fix status
     *   5. Populate output structure with position data
     *   6. Update validity flag based on fix status
     * 
     * Position Data:
     *   - Latitude/Longitude: WGS84 coordinate system
     *   - Altitude: Height above mean sea level
     *   - Accuracy: HDOP/VDOP dilution of precision
     *   - Satellites: Number of satellites used for fix
     * 
     * GNSS States:
     *   - No Fix: Invalid position, searching for satellites
     *   - 2D Fix: Latitude/longitude valid, altitude invalid
     *   - 3D Fix: Full position data valid
     *   - Cold Start: Initial satellite acquisition (longer time)
     * 
     * Performance:
     *   - Acquisition Time: 30-300 seconds for cold start
     *   - Update Rate: 1-10 Hz depending on configuration
     *   - Accuracy: 2.5-5 meters typical (open sky)
     *   - Power Usage: 20-50mA during active tracking
     * 
     * Error Handling:
     *   - No satellite fix returns invalid position
     *   - Poor signal conditions handled gracefully
     *   - Antenna issues detected and reported
     *   - Invalid data parsing errors logged
     * 
     * Usage Notes:
     *   - GNSS may need clear sky view for best performance
     *   - Cold start takes significant time after power-on
     *   - Position accuracy varies with satellite geometry
     *   - Function is non-blocking and returns current data
     * 
     * @note GNSS receiver must be enabled in modem configuration.
     *       Position availability depends on satellite visibility.
     *       Cold start can take several minutes in poor conditions.
     * 
     * @see GnssInfo - Position data structure
     * @see loop() - GNSS status updates in main loop
     */
    bool getGnss(GnssInfo &out);

    /**
     * @brief Returns current NB-IoT status information.
     * 
     * Provides read-only access to comprehensive cellular connectivity
     * status including modem state, network attachment, and MQTT connection.
     * Used for monitoring and diagnostics.
     * 
     * @return NbIotStatus structure with current status
     * 
     * Status Information:
     *   - Modem Ready: Hardware initialization status
     *   - Network Attached: Cellular network registration
     *   - PDP Active: Data connectivity status
     *   - MQTT Connected: MQTT broker connection status
     *   - Operator Name: Current network operator
     *   - IP Address: Assigned cellular IP address
     *   - RSSI: Signal strength measurement
     * 
     * Status Updates:
     *   - Real-time: Current state when called
     *   - Background: Updated by loop() function
     *   - Persistent: Status maintained across calls
     *   - Thread Safe: Read-only access safe for concurrent use
     * 
     * Usage Examples:
     *   NbIotStatus status = NB_IOT2.getStatus();
     *   if (status.mqtt_connected) {
     *       // Send data over cellular MQTT
     *   }
     * 
     * Performance:
     *   - Execution Time: < 1ms
     *   - No Network I/O: Local status retrieval only
     *   - Memory Usage: Minimal (structure copy)
     * 
     * @note Returns snapshot of current status.
     *       Status is updated by loop() function calls.
     *       All fields are read-only through this interface.
     * 
     * @see NbIotStatus - Status structure definition
     * @see loop() - Status update function
     */
    NbIotStatus getStatus() const;

private:
    /**
     * @brief Sends AT command to modem and waits for response.
     * 
     * Internal utility function for sending AT commands to the modem
     * and optionally waiting for specific responses. Handles command
     * formatting, transmission, and response parsing.
     * 
     * @param cmd - AT command string to send
     * @param resp - Optional pointer to store response string
     * @param timeout - Timeout in milliseconds for response
     * @return true if command sent successfully, false on error
     */
    bool sendAT(const String &cmd, String *resp = nullptr,
                uint32_t timeout = 5000);

    /**
     * @brief Waits for specific token in modem response.
     * 
     * Internal function that waits for a specific response token
     * from the modem, useful for parsing multi-line responses.
     * 
     * @param token - Response token to wait for
     * @param resp - Pointer to store complete response
     * @param timeout - Timeout in milliseconds
     * @return true if token received, false on timeout/error
     */
    bool waitFor(const String &token, String *resp,
                 uint32_t timeout);

    /**
     * @brief Ensures modem is ready and responsive.
     * 
     * Internal function that verifies modem is powered on and
     * responding to basic AT commands. Performs initialization
     * if modem is not ready.
     * 
     * @return true if modem is ready, false on failure
     */
    bool ensureModem();

    /**
     * @brief Ensures network attachment with specified APN.
     * 
     * Internal function that handles network registration and
     * attachment using the provided APN configuration.
     * 
     * @param apn - Access Point Name for network connection
     * @return true if network attached, false on failure
     */
    bool ensureNetwork(const String &apn);

    /**
     * @brief Ensures PDP context is active.
     * 
     * Internal function that activates the Packet Data Protocol
     * context for IP connectivity using the specified APN.
     * 
     * @param apn - Access Point Name for PDP activation
     * @return true if PDP active, false on failure
     */
    bool ensurePdp(const String &apn);

    /**
     * @brief Ensures MQTT connection to broker.
     * 
     * Internal function that establishes and maintains MQTT
     * connection to the specified broker over cellular network.
     * 
     * @param host - MQTT broker hostname
     * @param port - MQTT broker port
     * @param user - MQTT username
     * @param pass - MQTT password
     * @return true if MQTT connected, false on failure
     */
    bool ensureMqtt(const String &host, int port,
                    const String &user, const String &pass);

private:
    /**
     * @brief Hardware serial interface for modem communication.
     * 
     * Pointer to HardwareSerial instance used for AT command
     * communication with the NB-IoT modem.
     */
    HardwareSerial *m_serial = nullptr;

    /**
     * @brief Current NB-IoT status information.
     * 
     * Internal storage for modem and network status.
     * Updated by various functions and returned by getStatus().
     */
    NbIotStatus m_status;

    /**
     * @brief Timestamp of last MQTT ping.
     * 
     * Tracks when the last MQTT keep-alive ping was sent.
     * Used to maintain MQTT connection health.
     */
    unsigned long m_lastPing = 0;

    /**
     * @brief Initialization flag.
     * 
     * Indicates whether the modem has been properly initialized
     * through the init() function.
     */
    bool m_inited = false;
};

// ============================================================================
// Global NB-IoT Instance
// ============================================================================

/**
 * @brief Global NB-IoT instance for system-wide cellular access.
 * 
 * Singleton instance providing cellular connectivity functionality
 * throughout the SmartFranklin application. All cellular operations
 * go through this global instance.
 * 
 * Usage Pattern:
 *   NB_IOT2.init(&Serial2, 115200, 16, 17);  // Initialize
 *   NB_IOT2.connectNetwork("iot.apn");       // Connect network
 *   NB_IOT2.mqttConnect("broker.com", 1883); // Connect MQTT
 *   NB_IOT2.loop();                          // Maintain connection
 * 
 * Access Control:
 *   - Global Scope: Accessible from any source file
 *   - Single Instance: One cellular interface per device
 *   - Thread Safety: Functions are generally synchronous
 *   - Lifetime: Available throughout program execution
 * 
 * @see NbIot2 - Cellular connectivity class
 * @see NbIotStatus - Status structure
 * @see GnssInfo - GNSS position structure
 */
extern NbIot2 NB_IOT2;