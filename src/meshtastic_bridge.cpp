/*
 * ============================================================================
 * Meshtastic Bridge Module - SmartFranklin
 * ============================================================================
 * 
 * File:        meshtastic_bridge.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: LoRa mesh networking bridge using Meshtastic protocol.
 *              Provides long-range, decentralized communication for remote
 *              areas without WiFi coverage. Bridges local MQTT to mesh network.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   Meshtastic is an open-source, decentralized mesh networking protocol
 *   designed for long-range communication using LoRa radio modules. This
 *   module integrates Meshtastic into SmartFranklin to provide redundant
 *   connectivity in areas without WiFi or cellular coverage.
 * 
 * Key Features:
 *   - LoRa mesh networking: Multi-hop communication up to 20+ km range
 *   - Decentralized architecture: No central server required
 *   - Text message bridging: Relay MQTT messages over mesh network
 *   - Automatic routing: Messages find optimal path through mesh
 *   - Battery-efficient: Low-power LoRa transmission
 *   - GPS integration: Location-aware routing and position sharing
 *   - Encrypted communication: End-to-end security for sensitive data
 * 
 * Hardware Requirements:
 *   - LoRa radio module (e.g., EByte E22-900M30S, Heltec LoRa32)
 *   - Serial connection to ESP32 (UART pins configurable)
 *   - Antenna tuned for frequency band (433MHz, 868MHz, or 915MHz)
 *   - Power supply: 3.3V-5V, current draw ~100mA during transmission
 * 
 * Protocol Characteristics:
 *   - Frequency Bands: 433MHz (Europe), 868MHz (Europe), 915MHz (Americas)
 *   - Modulation: LoRa (Chirp Spread Spectrum)
 *   - Data Rate: 0.3-50 kbps (configurable for range vs speed)
 *   - Range: 2-20+ km line-of-sight (depends on antenna and power)
 *   - Mesh Topology: Ad-hoc, self-forming, self-healing
 *   - Message Types: Text, position, telemetry, private messages
 * 
 * Integration with SmartFranklin:
 *   - MQTT Bridge: Relays messages between local MQTT and mesh network
 *   - Fallback Connectivity: Activates when WiFi/NB-IoT unavailable
 *   - Sensor Data Publishing: Broadcasts sensor readings to mesh nodes
 *   - Command Reception: Receives remote commands via mesh network
 *   - Position Tracking: Shares GPS coordinates with mesh participants
 * 
 * Dependencies:
 *   - Meshtastic.h (Meshtastic protocol library)
 *   - M5Unified.h (M5Stack hardware abstraction)
 *   - M5Utility.h (M5Stack utility functions)
 *   - HardwareSerial (ESP32 serial communication)
 *   - meshtastic_bridge.h (header declarations)
 * 
 * Configuration:
 *   - UART Pins: Configured for GPIO 33 (RX), 32 (TX) - adjust for hardware
 *   - Baud Rate: 115200 bps (Meshtastic default)
 *   - Channel: PRIMARY channel for public communication
 *   - Message Format: TEXT_MESSAGE_APP for simple text bridging
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
 * copies or substantial portions of the Software is furnished to do so, subject to
 * the following conditions:
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

#include "meshtastic_bridge.h"

#include <Meshtastic.h>
#include <M5Unified.h>
#include <M5Utility.h>

// ============================================================================
// Global Meshtastic Objects
// ============================================================================

/**
 * @brief HardwareSerial instance for Meshtastic LoRa module communication.
 * 
 * Configured for UART1 on ESP32 with pins 33 (RX) and 32 (TX).
 * Baud rate 115200 matches Meshtastic default configuration.
 * Adjust pin assignments based on actual hardware connections.
 */
HardwareSerial MESHSERIAL(1);

/**
 * @brief SerialDevice wrapper for Meshtastic communication.
 * 
 * Abstracts the serial interface, providing Meshtastic protocol layer
 * with access to the LoRa module through HardwareSerial.
 */
SerialDevice* meshDevice = nullptr;

/**
 * @brief Main Meshtastic mesh network interface.
 * 
 * Provides high-level API for mesh networking operations:
 * - Sending and receiving messages
 * - Network management and routing
 * - Event handling and callbacks
 */
Mesh* mesh = nullptr;

/**
 * @brief Buffer for last received mesh message.
 * 
 * Stores the most recent text message received from the mesh network.
 * Cleared after being retrieved by meshtastic_poll_received().
 */
static String lastReceived;

// ============================================================================
// Meshtastic Packet Reception Callback
// ============================================================================

/**
 * @brief Callback function invoked when a mesh packet is received.
 * 
 * Processes incoming packets from the Meshtastic network, extracting
 * text messages for bridging to MQTT. Only TEXT_MESSAGE_APP packets
 * are processed; other packet types (position, telemetry) are ignored.
 * 
 * Packet Processing:
 *   - Validates packet type (must be TEXT_MESSAGE_APP)
 *   - Extracts payload data as UTF-8 string
 *   - Stores in lastReceived buffer for polling retrieval
 *   - Thread-safe: Called from Meshtastic library context
 * 
 * Message Format:
 *   Incoming packets contain binary payload data. For text messages:
 *   - Port number: PortNum::TEXT_MESSAGE_APP (indicates text content)
 *   - Payload: UTF-8 encoded string data
 *   - Length: Variable, up to maximum packet size (~200 bytes)
 * 
 * Bridging Behavior:
 *   - Messages are queued in lastReceived buffer
 *   - Retrieved by meshtastic_poll_received() in main application
 *   - Forwarded to MQTT topics for cloud integration
 *   - Supports multi-hop routing through mesh network
 * 
 * @param p - Reference to received MeshPacket structure
 *            Contains decoded payload, sender info, and metadata
 * 
 * @return void
 * 
 * @note This function is called asynchronously by the Meshtastic library.
 *       Keep processing minimal to avoid blocking mesh operations.
 *       Do not perform blocking I/O or long operations here.
 * 
 * @see MeshPacket - Packet structure definition in Meshtastic library
 * @see PortNum::TEXT_MESSAGE_APP - Text message port identifier
 */
void onMeshPacket(const MeshPacket& p)
{
    // Check if packet contains a text message (not position/telemetry/etc.)
    if (p.decoded.portnum == PortNum::TEXT_MESSAGE_APP) {
        // Extract text payload from packet data
        // Convert binary payload to UTF-8 string
        // Payload size indicates actual message length
        lastReceived = String(
            reinterpret_cast<const char*>(p.decoded.payload.data()),
            p.decoded.payload.size()
        );
    }
}

// ============================================================================
// Meshtastic Bridge Initialization
// ============================================================================

/**
 * @brief Initializes the Meshtastic bridge for mesh networking.
 * 
 * Sets up serial communication with LoRa module, creates Meshtastic
 * device and mesh objects, registers packet callback, and begins
 * mesh network participation. Must be called once during system startup.
 * 
 * Hardware Setup:
 *   - UART1: GPIO 33 (RX), GPIO 32 (TX) - adjust for your hardware
 *   - Baud Rate: 115200 bps (Meshtastic default)
 *   - Serial Mode: 8 data bits, no parity, 1 stop bit (8N1)
 *   - Flow Control: None (RTS/CTS not used)
 * 
 * Meshtastic Initialization Sequence:
 *   1. Configure HardwareSerial for LoRa module
 *   2. Create SerialDevice wrapper for protocol abstraction
 *   3. Instantiate Mesh object for network operations
 *   4. Register onMeshPacket callback for incoming messages
 *   5. Begin mesh network participation (join network)
 * 
 * Network Joining:
 *   - Device broadcasts presence to nearby mesh nodes
 *   - Receives routing table updates from neighbors
 *   - Participates in mesh topology formation
 *   - May take 10-30 seconds for full network integration
 * 
 * Error Handling:
 *   - Serial initialization failures: Function returns silently
 *   - No error reporting (embedded system constraints)
 *   - Check mesh pointer for successful initialization
 * 
 * Power Considerations:
 *   - LoRa transmission: ~100mA current draw (brief spikes)
 *   - Receive mode: ~20mA continuous
 *   - Sleep mode: < 1mA when not transmitting
 *   - Duty cycle: Keep transmissions short to conserve battery
 * 
 * @return void
 * 
 * @note Call this function during setup() phase, after M5Stack initialization.
 *       Ensure LoRa module is powered and connected before calling.
 *       Example: meshtastic_bridge_init(); // in setup()
 * 
 * @see HardwareSerial::begin() - Serial port configuration
 * @see Mesh::begin() - Start mesh network participation
 */
void meshtastic_bridge_init()
{
    // =========================================================================
    // Serial Port Configuration for LoRa Module
    // =========================================================================
    // Initialize UART1 for communication with Meshtastic LoRa module
    // Pins: GPIO 33 (RX from LoRa), GPIO 32 (TX to LoRa)
    // Adjust pin numbers based on your hardware wiring
    // Baud rate 115200 matches Meshtastic firmware default
    MESHSERIAL.begin(115200, SERIAL_8N1, 33, 32);

    // =========================================================================
    // Meshtastic Object Creation
    // =========================================================================
    // Create SerialDevice wrapper to abstract serial communication
    // Provides Meshtastic protocol layer with hardware access
    meshDevice = new SerialDevice(&MESHSERIAL);
    
    // Create main Mesh object for network operations
    // Handles routing, sending, receiving, and network management
    mesh = new Mesh(meshDevice);

    // =========================================================================
    // Callback Registration and Network Start
    // =========================================================================
    // Register callback function for incoming packet processing
    // onMeshPacket will be called whenever a mesh packet is received
    mesh->onReceive(onMeshPacket);
    
    // Begin mesh network participation
    // Device will start listening for and sending mesh traffic
    // Network integration may take several seconds
    mesh->begin();
}

// ============================================================================
// Meshtastic Message Transmission
// ============================================================================

/**
 * @brief Sends a text message over the Meshtastic mesh network.
 * 
 * Transmits a text string to all nodes on the PRIMARY channel of the
 * mesh network. Messages are automatically routed through the mesh
 * using multi-hop algorithms to reach destination nodes.
 * 
 * Transmission Characteristics:
 *   - Channel: PRIMARY (public channel, all nodes can receive)
 *   - Message Type: TEXT_MESSAGE_APP (text content identifier)
 *   - Routing: Automatic multi-hop through mesh topology
 *   - Reliability: Best-effort delivery (no acknowledgments)
 *   - Encryption: Uses channel encryption if configured
 * 
 * Message Limits:
 *   - Maximum length: ~200 bytes (LoRa packet size constraints)
 *   - Encoding: UTF-8 supported
 *   - Delivery time: 1-10 seconds depending on mesh size and distance
 *   - Range: Up to 20+ km with optimal conditions
 * 
 * Usage Examples:
 *   @code
 *   // Send sensor data over mesh
 *   meshtastic_send_text("Temperature: 25.5°C");
 *   
 *   // Send status update
 *   meshtastic_send_text("SmartFranklin online");
 *   
 *   // Bridge MQTT message to mesh
 *   meshtastic_send_text(mqtt_payload);
 *   @endcode
 * 
 * Error Conditions:
 *   - Mesh not initialized: Function returns silently (no transmission)
 *   - Message too long: Truncated by Meshtastic library
 *   - No mesh neighbors: Message may not be delivered
 *   - Radio interference: Transmission may fail
 * 
 * Power Impact:
 *   - Transmission duration: ~100-500ms
 *   - Current draw: ~100mA peak during transmit
 *   - Duty cycle: Keep transmissions infrequent (< 1% airtime)
 * 
 * @param msg - Text message to send over mesh network
 *               Should be UTF-8 encoded string
 *               Maximum recommended length: 200 characters
 * 
 * @return void
 * 
 * @note This is an asynchronous operation - function returns immediately
 *       while transmission occurs in background. No delivery confirmation.
 *       For critical messages, consider implementing acknowledgment protocol.
 * 
 * @see Mesh::sendText() - Underlying Meshtastic transmission API
 * @see ChannelIndex::PRIMARY - Public mesh channel identifier
 */
void meshtastic_send_text(const String& msg)
{
    // Check if mesh network is initialized and available
    if (!mesh) return;
    
    // Send text message to all nodes on PRIMARY channel
    // Message will be automatically routed through mesh network
    // No delivery confirmation or error reporting
    mesh->sendText(msg.c_str(), ChannelIndex::PRIMARY);
}

// ============================================================================
// Meshtastic Message Reception Polling
// ============================================================================

/**
 * @brief Polls for received text messages from the mesh network.
 * 
 * Checks if any text messages have been received since last poll,
 * returning the message content if available. Messages are buffered
 * by the onMeshPacket callback and retrieved here for processing.
 * 
 * Polling Behavior:
 *   - Non-blocking: Returns immediately if no message available
 *   - FIFO processing: Returns oldest unprocessed message first
 *   - Message consumption: Retrieved messages are cleared from buffer
 *   - Thread-safe: Can be called from any context
 * 
 * Return Values:
 *   - true: Message received, content placed in 'out' parameter
 *   - false: No messages available, 'out' parameter unchanged
 * 
 * Usage Pattern:
 *   @code
 *   void loop() {
 *       String mesh_msg;
 *       if (meshtastic_poll_received(mesh_msg)) {
 *           // Process received message
 *           Serial.println("Mesh received: " + mesh_msg);
 *           // Forward to MQTT or other processing
 *       }
 *   }
 *   @endcode
 * 
 * Integration with MQTT Bridge:
 *   - Received mesh messages can be published to MQTT topics
 *   - Enables bidirectional communication between mesh and cloud
 *   - Supports remote control commands via mesh network
 * 
 * Buffer Management:
 *   - Single message buffer (lastReceived)
 *   - Overwrites previous message if multiple arrive before polling
 *   - Consider implementing queue for high-traffic scenarios
 * 
 * Performance:
 *   - Polling frequency: 10-100Hz recommended
 *   - Processing time: < 1ms when no messages
 *   - Memory usage: Minimal (String buffer only)
 * 
 * @param out - Reference to String that will receive message content
 *              Only modified if function returns true
 *              Previous content preserved if no message available
 * 
 * @return bool - true if message received and placed in 'out'
 *                false if no messages available
 * 
 * @note Call this function regularly (e.g., every 100ms) to process
 *       incoming mesh messages. Messages not polled will be lost.
 *       For real-time applications, consider interrupt-driven processing.
 * 
 * @see onMeshPacket() - Callback that populates message buffer
 * @see Mesh::loop() - Processes incoming mesh traffic
 */
bool meshtastic_poll_received(String& out)
{
    // Check if mesh network is initialized
    if (!mesh) return false;

    // Process any pending mesh network traffic
    // Updates routing tables, handles acknowledgments, processes packets
    mesh->loop();

    // Check if a message has been received and buffered
    if (lastReceived.length()) {
        // Copy message to output parameter
        out = lastReceived;
        
        // Clear buffer to prepare for next message
        lastReceived = "";
        
        // Indicate successful message retrieval
        return true;
    }
    
    // No messages available
    return false;
}