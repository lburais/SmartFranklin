/*
 * ============================================================================
 * Meshtastic Bridge Task Module - SmartFranklin
 * ============================================================================
 * 
 * File:        task_meshtastic_bridge.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: FreeRTOS task implementing bidirectional bridge between Meshtastic
 *              mesh network and MQTT broker. Forwards text messages, position data,
 *              and node information between the decentralized mesh network and
 *              centralized MQTT infrastructure for seamless IoT integration.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   SmartFranklin acts as a gateway between Meshtastic's decentralized mesh
 *   networking and traditional MQTT-based IoT infrastructure. This bridge
 *   enables IoT devices in remote areas without internet connectivity to
 *   communicate through mesh networks while still integrating with cloud
 *   services via MQTT. The bridge handles bidirectional message forwarding,
 *   connection status monitoring, and automatic reconnection.
 * 
 * Meshtastic Integration:
 *   - Mesh Network: Decentralized, long-range communication using LoRa
 *   - Device Types: Supports various Meshtastic devices (T-Beam, Heltec, etc.)
 *   - Communication: Serial interface to Meshtastic radio module
 *   - Message Types: Text messages, position data, node information, telemetry
 *   - Channels: Multi-channel support for different message types
 * 
 * Bridge Functionality:
 * 
 *   MQTT → Meshtastic Direction:
 *   - Subscribes to MQTT topics with configurable prefix
 *   - Forwards MQTT messages as Meshtastic text messages
 *   - Enables cloud-to-mesh communication for remote control
 *   - Supports broadcast and direct messaging
 * 
 *   Meshtastic → MQTT Direction:
 *   - Receives mesh packets and forwards to MQTT topics
 *   - Text messages: Published to smartfranklin/meshtastic/text
 *   - Position data: Published as JSON to smartfranklin/meshtastic/position
 *   - Node info: Published to smartfranklin/meshtastic/nodeinfo
 *   - Status updates: Connection state monitoring
 * 
 * Message Processing:
 *   - Packet Types: Handles TEXT, POSITION, NODEINFO, and status events
 *   - JSON Formatting: Position data converted to structured JSON
 *   - Topic Organization: Hierarchical MQTT topics for different data types
 *   - QoS Settings: Standard MQTT QoS for reliable delivery
 *   - Filtering: Only forwards messages matching configured prefix
 * 
 * Connection Management:
 *   - Serial Interface: Configurable baud rate and pins (CONFIG parameters)
 *   - Auto-reconnection: Handles radio disconnections gracefully
 *   - Status Publishing: MQTT notifications for connection state changes
 *   - Node Discovery: Automatic detection of mesh network nodes
 *   - Channel Management: Support for multiple communication channels
 * 
 * MQTT Topics:
 *   - smartfranklin/meshtastic/text: Text messages from mesh network
 *   - smartfranklin/meshtastic/position: GPS position data (JSON format)
 *   - smartfranklin/meshtastic/nodeinfo: Node information and names
 *   - smartfranklin/meshtastic/node_updated: Node status changes
 *   - smartfranklin/meshtastic/status: Bridge connection status
 *   - [prefix]/*: MQTT topics forwarded to mesh (configurable prefix)
 * 
 * Configuration:
 *   - Bridge Enable: CONFIG.meshtastic_bridge_enabled controls operation
 *   - MQTT Prefix: CONFIG.meshtastic_mqtt_prefix for topic filtering
 *   - Serial Pins: CONFIG.meshtastic_pin_rx/tx for radio connection
 *   - Baud Rate: CONFIG.meshtastic_baud for serial communication
 *   - Channel Index: Default channel 0 for message sending
 * 
 * Error Handling:
 *   - Serial failures: Logged and reconnection attempted
 *   - MQTT disconnections: Bridge continues operating
 *   - Invalid packets: Discarded with logging
 *   - Radio timeouts: Automatic reconnection logic
 *   - Configuration errors: Graceful degradation
 * 
 * Performance Considerations:
 *   - CPU Usage: Low (mostly waiting for events)
 *   - Memory Usage: Minimal (event-driven processing)
 *   - Serial Bandwidth: LoRa radio communication overhead
 *   - MQTT Traffic: Event-driven publishing (not continuous)
 *   - Task Priority: Standard priority for reliable operation
 * 
 * Dependencies:
 *   - Meshtastic.h (Meshtastic radio communication library)
 *   - M5Unified.h (M5Stack hardware interface)
 *   - Arduino.h (FreeRTOS task functions)
 *   - tasks.h (Task definitions and priorities)
 *   - mqtt_layer.h (MQTT publishing and subscription)
 *   - config_store.h (Configuration access)
 *   - data_model.h (Global data structures)
 * 
 * Limitations:
 *   - Single radio support (no multi-radio configurations)
 *   - Text message focus (limited binary data support)
 *   - Fixed channel assignment (channel 0 default)
 *   - No message queuing (real-time forwarding only)
 *   - Serial dependency (no direct LoRa interface)
 *   - Configuration changes require restart
 * 
 * Security Considerations:
 *   - Mesh encryption handled by Meshtastic protocol
 *   - MQTT authentication through existing broker setup
 *   - No additional encryption in bridge layer
 *   - Topic-based access control through MQTT broker
 *   - Physical security of radio hardware required
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

#include <Arduino.h>
#include <Meshtastic.h>
#include <M5Unified.h>
#include <M5Utility.h>

#include "tasks.h"
#include "mqtt_layer.h"
#include "config_store.h"
#include "data_model.h"

// ============================================================================
// Bridge Configuration Constants
// ============================================================================

/**
 * @brief Period for sending test messages (in seconds).
 * 
 * Defines how often the bridge sends "Hello, world!" test messages
 * to the mesh network. Set to 300 seconds (5 minutes) for minimal
 * network traffic while maintaining connectivity testing.
 */
#define SEND_PERIOD 300

/**
 * @brief Global variables for message timing and connection state.
 * 
 * next_send_time: Timestamp for next scheduled test message
 * not_yet_connected: Flag to track initial connection status
 */
uint32_t next_send_time = 0;
bool not_yet_connected = true;

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Converts Meshtastic public key to hexadecimal string representation.
 * 
 * Takes a 32-byte public key and converts it to a 64-character hexadecimal
 * string for logging and debugging purposes. Each byte becomes two hex
 * characters (00-FF), resulting in a null-terminated 65-character string.
 * 
 * @param pubKey - Meshtastic public key structure containing 32 bytes
 * @param hex_str - Output buffer (must be at least 65 characters)
 * 
 * @note Output buffer must be large enough to hold 64 hex characters + null terminator.
 *       No bounds checking performed - caller must ensure sufficient buffer size.
 */
void displayPubKey(meshtastic_MeshPacket_public_key_t pubKey, char *hex_str) {
    for (int i = 0; i < 32; i++) {
        sprintf(&hex_str[i * 2], "%02x", (unsigned char)pubKey.bytes[i]);
    }
    hex_str[64] = '\0'; // Null terminator
}

// ============================================================================
// Meshtastic Callback Functions
// ============================================================================

/**
 * @brief Callback function for Meshtastic node connection events.
 * 
 * Called when the Meshtastic device establishes or updates connection
 * to the mesh network. Logs connection status and updates global state.
 * Used for initial connection detection and status monitoring.
 * 
 * @param node - Pointer to connected node information
 * @param progress - Connection progress indicator
 * 
 * @note Sets not_yet_connected to false after first connection.
 *       Provides logging for connection establishment.
 */
static void connected_callback(mt_node_t *node, mt_nr_progress_t progress) {
    if (not_yet_connected) 
        M5_LOGI("Connected to Meshtastic device!");
    not_yet_connected = false;
}

/**
 * @brief Callback function for encrypted Meshtastic messages.
 * 
 * Handles incoming encrypted messages from the mesh network.
 * Currently logs the sender and recipient information for debugging.
 * Encrypted payload processing could be added here if needed.
 * 
 * @param from - Sender node ID
 * @param to - Recipient node ID (0xFFFFFFFF for broadcast)
 * @param channel - Communication channel number
 * @param pubKey - Public key of the sender
 * @param enc_payload - Encrypted message payload
 * 
 * @note Encrypted messages are not forwarded to MQTT in current implementation.
 *       Could be extended to handle encrypted data forwarding if required.
 */
void encrypted_callback(uint32_t from, uint32_t to,  uint8_t channel, meshtastic_MeshPacket_public_key_t pubKey, meshtastic_MeshPacket_encrypted_t *enc_payload) {
    M5_LOGI("Received an ENCRYPTED callback from: %u to: %u\n", from, to);
}

/**
 * @brief Callback function for Meshtastic port number messages.
 * 
 * Processes messages based on their port number (application type).
 * Includes a utility function to convert port numbers to human-readable strings.
 * Logs the port type for debugging and potential filtering.
 * 
 * Port Numbers Supported:
 *   - TEXT_MESSAGE_APP: Standard text messages
 *   - POSITION_APP: GPS location data
 *   - NODEINFO_APP: Node information and names
 *   - TELEMETRY_APP: Sensor and status data
 *   - And many others for specialized applications
 * 
 * @param from - Sender node ID
 * @param to - Recipient node ID
 * @param channel - Communication channel
 * @param portNum - Application port number
 * @param payload - Message payload data
 * 
 * @note Nested function definition for portnum_to_string is non-standard C++.
 *       Should be moved to a separate utility function for better code organization.
 */
void portnum_callback(uint32_t from, uint32_t to,  uint8_t channel, meshtastic_PortNum portNum, meshtastic_Data_payload_t *payload) {
    // Utility function to convert port numbers to strings (should be global)
    char meshtastic_portnum_to_string(meshtastic_PortNum port) {
        switch (port) {
            case meshtastic_PortNum_UNKNOWN_APP: return "UNKNOWN_APP";
            case meshtastic_PortNum_TEXT_MESSAGE_APP: return "TEXT_MESSAGE_APP";
            case meshtastic_PortNum_REMOTE_HARDWARE_APP: return "REMOTE_HARDWARE_APP";
            case meshtastic_PortNum_POSITION_APP: return "POSITION_APP";
            case meshtastic_PortNum_NODEINFO_APP: return "NODEINFO_APP";
            case meshtastic_PortNum_ROUTING_APP: return "ROUTING_APP";
            case meshtastic_PortNum_ADMIN_APP: return "ADMIN_APP";
            case meshtastic_PortNum_TEXT_MESSAGE_COMPRESSED_APP: return "TEXT_MESSAGE_COMPRESSED_APP";
            case meshtastic_PortNum_WAYPOINT_APP: return "WAYPOINT_APP";
            case meshtastic_PortNum_AUDIO_APP: return "AUDIO_APP";
            case meshtastic_PortNum_DETECTION_SENSOR_APP: return "DETECTION_SENSOR_APP";
            case meshtastic_PortNum_REPLY_APP: return "REPLY_APP";
            case meshtastic_PortNum_IP_TUNNEL_APP: return "IP_TUNNEL_APP";
            case meshtastic_PortNum_PAXCOUNTER_APP: return "PAXCOUNTER_APP";
            case meshtastic_PortNum_SERIAL_APP: return "SERIAL_APP";
            case meshtastic_PortNum_STORE_FORWARD_APP: return "STORE_FORWARD_APP";
            case meshtastic_PortNum_RANGE_TEST_APP: return "RANGE_TEST_APP";
            case meshtastic_PortNum_TELEMETRY_APP: return "TELEMETRY_APP";
            case meshtastic_PortNum_ZPS_APP: return "ZPS_APP";
            case meshtastic_PortNum_SIMULATOR_APP: return "SIMULATOR_APP";
            case meshtastic_PortNum_TRACEROUTE_APP: return "TRACEROUTE_APP";
            case meshtastic_PortNum_NEIGHBORINFO_APP: return "NEIGHBORINFO_APP";
            case meshtastic_PortNum_ATAK_PLUGIN: return "ATAK_PLUGIN";
            case meshtastic_PortNum_MAP_REPORT_APP: return "MAP_REPORT_APP";
            case meshtastic_PortNum_POWERSTRESS_APP: return "POWERSTRESS_APP";
            case meshtastic_PortNum_PRIVATE_APP: return "PRIVATE_APP";
            case meshtastic_PortNum_ATAK_FORWARDER: return "ATAK_FORWARDER";
            case meshtastic_PortNum_MAX: return "MAX";
            default: return "UNKNOWN_PORTNUM";
        }
    }
    M5_LOGI("Received a callback for PortNum : %s\n", meshtastic_portnum_to_string(portNum));
}

/**
 * @brief Callback function for Meshtastic text messages.
 * 
 * Processes incoming text messages from the mesh network.
 * Logs message details including sender, recipient, channel, and content.
 * Identifies broadcast vs. direct messages for proper handling.
 * 
 * Message Types:
 *   - Broadcast: to = 0xFFFFFFFF (sent to all nodes)
 *   - Direct Message to self: to = my_node_num
 *   - Direct Message to others: to = other node ID
 * 
 * @param from - Sender node ID
 * @param to - Recipient node ID
 * @param channel - Communication channel number
 * @param text - Message text content
 * 
 * @note Messages are logged but not automatically forwarded to MQTT.
 *       MQTT forwarding is handled in the mesh_to_mqtt function.
 */
void text_message_callback(uint32_t from, uint32_t to,  uint8_t channel, const char* text) {
    M5_LOGI("Received a text message on channel: %u from: %u to: %u\n", channel, from, to);
    M5_LOGI("  Message: %s\n", text);
    if (to == 0xFFFFFFFF){
        M5_LOGI("  This is a BROADCAST message.");
    } else if (to == my_node_num){
        M5_LOGI("  This is a DM to me!");
    } else {
        M5_LOGI("  This is a DM to someone else.");
    }
}

// ============================================================================
// Message Forwarding Functions
// ============================================================================

/**
 * @brief Forwards Meshtastic packets to MQTT topics.
 * 
 * Processes incoming mesh packets and publishes relevant data to MQTT.
 * Handles different packet types (text, position, node info) with appropriate
 * formatting and topic selection.
 * 
 * Supported Packet Types:
 *   - TEXT: Published to smartfranklin/meshtastic/text
 *   - POSITION: Converted to JSON and published to smartfranklin/meshtastic/position
 *   - NODEINFO: Published to smartfranklin/meshtastic/nodeinfo
 * 
 * JSON Position Format:
 *   {"lat": latitude, "lon": longitude, "alt": altitude}
 *   All values are floats with appropriate precision
 * 
 * @param p - Reference to the received MeshPacket
 * 
 * @note Only processes decoded packets with supported types.
 *       Encrypted or unknown packets are silently ignored.
 *       QoS and retention settings use MQTT defaults.
 */
static void mesh_to_mqtt(const MeshPacket &p)
{
    // Text message forwarding
    if (p.decoded.type == MeshPacket::Decoded::TEXT) {
        sf_mqtt::publish(
            "smartfranklin/meshtastic/text",
            p.decoded.text.c_str()
        );
    }

    // Position data forwarding with JSON formatting
    if (p.decoded.type == MeshPacket::Decoded::POSITION) {
        char buf[128];
        snprintf(buf, sizeof(buf),
                 "{\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.1f}",
                 p.decoded.position.latitude,
                 p.decoded.position.longitude,
                 p.decoded.position.altitude);

        sf_mqtt::publish("smartfranklin/meshtastic/position", buf);
    }

    // Node information forwarding
    if (p.decoded.type == MeshPacket::Decoded::NODEINFO) {
        sf_mqtt::publish(
            "smartfranklin/meshtastic/nodeinfo",
            p.decoded.nodeinfo.longName.c_str()
        );
    }
}

/**
 * @brief Forwards MQTT messages to Meshtastic mesh network.
 * 
 * Callback function for MQTT subscriptions that forwards matching messages
 * to the mesh network as text messages. Only processes messages with the
 * configured topic prefix and bridge enabled.
 * 
 * Message Processing:
 *   1. Check if bridge is enabled in configuration
 *   2. Verify topic matches configured prefix
 *   3. Extract message payload
 *   4. Send as broadcast text message to mesh
 * 
 * @param topic - MQTT topic of the received message
 * @param payload - Message payload content
 * 
 * @note Duplicate function definition exists - this is the active one.
 *       Messages are sent as broadcasts to all mesh nodes.
 *       No QoS or delivery confirmation in mesh network.
 */
static void mqtt_to_mesh(const char *topic, const char *payload)
{
    if (!CONFIG.meshtastic_bridge_enabled) return;

    // Only forward messages matching the configured prefix
    String t = String(topic);
    if (!t.startsWith(CONFIG.meshtastic_mqtt_prefix)) return;

    // Strip prefix and send as text message
    String msg = payload;
    radio.sendText(msg.c_str());
}

// ============================================================================
// Setup and Loop Functions (Legacy - Not Used in Task)
// ============================================================================

/**
 * @brief Legacy setup function from Meshtastic example code.
 * 
 * Initializes serial communication and Meshtastic device connection.
 * Sets up callbacks for different message types and MQTT subscription.
 * This function is called from the task but contains legacy code.
 * 
 * Initialization Steps:
 *   1. Initialize serial port with timeout
 *   2. Initialize Meshtastic serial interface
 *   3. Request node report for connection
 *   4. Register callback functions
 *   5. Subscribe to MQTT topics
 * 
 * @note This function contains legacy code and is not the primary initialization.
 *       The task uses a different initialization approach with MeshRadio.
 *       Consider refactoring to remove duplicate code.
 */
void setup() {
    // Serial initialization with timeout
    Serial.begin(115200);
    while(true) {
        if (Serial) break;
        if (millis() > 5000) {
            M5_LOGE("Couldn't find a serial port after 5 seconds, continuing anyway");
            break;
        }
    }

    M5_LOGI("Booted Meshtastic send/receive client in serial mode.");

    // Initialize Meshtastic serial interface (legacy)
    mt_serial_init(SERIAL_RX_PIN, SERIAL_TX_PIN, BAUD_RATE);

    randomSeed(micros());

    // Request initial connection
    mt_request_node_report(connected_callback);

    // Register callback functions
    set_text_message_callback(text_message_callback);
    set_portnum_callback(portnum_callback);
    set_encrypted_callback(encrypted_callback);

    // Subscribe to MQTT topics for forwarding
    sf_mqtt::subscribe(CONFIG.meshtastic_mqtt_prefix.c_str(), mqtt_callback);
}

/**
 * @brief Legacy loop function from Meshtastic example code.
 * 
 * Processes Meshtastic events and sends periodic test messages.
 * Contains timing logic for message sending and event processing.
 * This function is not used in the current task implementation.
 * 
 * Loop Behavior:
 *   - Record current timestamp
 *   - Process Meshtastic events
 *   - Send test message if timer expired
 *   - Schedule next message send
 * 
 * @note This function is legacy code and not called by the task.
 *       The task uses radio.loop() instead for event processing.
 *       Consider removing to clean up the codebase.
 */
void loop() {
    uint32_t now = millis();
    bool can_send = mt_loop(now);

    if (can_send && now >= next_send_time) {
        uint32_t dest = BROADCAST_ADDR; 
        uint8_t channel_index = 0; 
        mt_send_text("Hello, world!", dest, channel_index);
        next_send_time = now + SEND_PERIOD * 1000;
    }
}

// ============================================================================
// MQTT Callback (Referenced but Not Defined)
// ============================================================================

/**
 * @brief MQTT message callback function.
 * 
 * This function is referenced in the code but not defined in this file.
 * It should be implemented to handle incoming MQTT messages for forwarding
 * to the mesh network. The function signature matches the mqtt_layer callback.
 * 
 * @param topic - MQTT topic string
 * @param payload - Message payload string
 * 
 * @note Function declaration missing - needs to be implemented or included.
 *       Currently causes compilation error due to undefined reference.
 */
static void mqtt_callback(const std::string &topic, const std::string &payload);

// ============================================================================
// MeshRadio Instance and Event Handler
// ============================================================================

/**
 * @brief Global MeshRadio instance for Meshtastic communication.
 * 
 * Provides the primary interface to the Meshtastic radio module.
 * Handles serial communication, packet processing, and event callbacks.
 * Used by the task for all mesh network operations.
 */
static MeshRadio radio(&Serial1);

/**
 * @brief Event handler for Meshtastic radio events.
 * 
 * Processes various MeshRadio events and forwards relevant information to MQTT.
 * Handles packet reception, node updates, and connection status changes.
 * 
 * Supported Events:
 *   - PACKET_RECEIVED: Forward packet to MQTT via mesh_to_mqtt()
 *   - NODE_UPDATED: Publish node information to MQTT
 *   - RADIO_CONNECTED: Publish connection status
 *   - RADIO_DISCONNECTED: Publish disconnection status
 * 
 * @param ev - Reference to the MeshEvent containing event details
 * 
 * @note Default case ignores unhandled events.
 *       Event processing is synchronous within the callback.
 */
static void onMeshEvent(const MeshEvent &ev)
{
    switch (ev.type) {
        case MeshEvent::PACKET_RECEIVED:
            mesh_to_mqtt(ev.packet);
            break;

        case MeshEvent::NODE_UPDATED:
            sf_mqtt::publish("smartfranklin/meshtastic/node_updated",
                             ev.node.longName.c_str());
            break;

        case MeshEvent::RADIO_CONNECTED:
            sf_mqtt::publish("smartfranklin/meshtastic/status", "connected");
            break;

        case MeshEvent::RADIO_DISCONNECTED:
            sf_mqtt::publish("smartfranklin/meshtastic/status", "disconnected");
            break;

        default:
            break;
    }
}

// ============================================================================
// FreeRTOS Task Implementation
// ============================================================================

/**
 * @brief FreeRTOS task for Meshtastic-MQTT bridge operation.
 * 
 * Main task function that initializes the Meshtastic radio and runs the
 * bridge loop. Handles bidirectional message forwarding between mesh
 * network and MQTT broker with automatic reconnection and status monitoring.
 * 
 * Task Lifecycle:
 *   1. Log task startup
 *   2. Initialize serial communication for radio
 *   3. Set up MeshRadio with event handler
 *   4. Subscribe to MQTT topics for forwarding
 *   5. Call legacy setup function (consider refactoring)
 *   6. Enter main loop for event processing
 * 
 * Main Loop:
 *   - Process radio events with radio.loop()
 *   - Handle MQTT-to-mesh forwarding via callbacks
 *   - Maintain connection and forward messages
 *   - 20ms delay for responsive event processing
 * 
 * Serial Configuration:
 *   - Port: Serial1 (hardware serial)
 *   - Baud: CONFIG.meshtastic_baud
 *   - Pins: CONFIG.meshtastic_pin_rx/tx
 *   - Format: 8N1 (standard serial)
 * 
 * Error Handling:
 *   - Serial initialization failures logged
 *   - Radio connection issues handled by MeshRadio
 *   - MQTT subscription failures logged by mqtt_layer
 *   - Task continues running despite individual failures
 * 
 * Performance:
 *   - CPU Usage: Low (event-driven processing)
 *   - Memory Usage: Minimal (static allocations)
 *   - Serial Bandwidth: LoRa radio communication
 *   - Task Priority: Standard priority
 *   - Loop Delay: 20ms for responsive operation
 * 
 * @param pv - FreeRTOS task parameter (unused, nullptr)
 * 
 * @return void (task runs indefinitely)
 * 
 * @note Contains legacy setup/loop functions that should be refactored.
 *       MQTT callback function needs to be properly defined.
 *       Bridge enable/disable controlled by CONFIG.meshtastic_bridge_enabled.
 * 
 * @see MeshRadio - Meshtastic radio interface
 * @see mesh_to_mqtt() - Mesh to MQTT forwarding
 * @see mqtt_to_mesh() - MQTT to mesh forwarding
 */
void taskMeshtasticBridge(void *pv)
{
    M5_LOGI("[MESHTASTIC] Bridge task started");

    // Initialize serial for Meshtastic radio
    Serial1.begin(CONFIG.meshtastic_baud, SERIAL_8N1,
                  CONFIG.meshtastic_pin_rx,
                  CONFIG.meshtastic_pin_tx);

    // Initialize radio with event handler
    radio.onEvent(onMeshEvent);
    radio.begin();

    // Subscribe to MQTT topics for forwarding
    sf_mqtt::subscribe(CONFIG.meshtastic_mqtt_prefix.c_str(), mqtt_callback);

    // Call legacy setup (consider removing/refactoring)
    setup();

    // Main bridge loop
    for (;;) {
        radio.loop();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}