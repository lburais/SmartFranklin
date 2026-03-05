/*
 * ============================================================================
 * Meshtastic Bridge Module - SmartFranklin
 * ============================================================================
 * 
 * File:        meshtastic_bridge.h
 * Project:     SmartFranklin IoT Device Controller
 * Description: Header file for Meshtastic mesh networking bridge functionality.
 *              Provides interface for LoRa mesh network communication using
 *              the Meshtastic protocol, enabling long-range, decentralized
 *              messaging between SmartFranklin devices.
 * 
 * Author:      Laurent Burais
 * Date:        5 March 2026
 * Version:     1.0
 * 
 * Overview:
 *   The Meshtastic bridge module integrates SmartFranklin with the Meshtastic
 *   ecosystem, providing mesh networking capabilities for IoT applications.
 *   Meshtastic uses LoRa radio technology to create decentralized mesh networks
 *   that can operate over long distances without cellular or WiFi infrastructure.
 *   This bridge enables SmartFranklin to send and receive text messages through
 *   the mesh network for remote monitoring, control, and data sharing.
 * 
 * Meshtastic Technology:
 *   - Protocol: Open-source mesh networking for LoRa devices
 *   - Hardware: LoRa radio modules (typically ESP32 + SX1262/SX1276)
 *   - Range: 1-20km depending on terrain, antenna, and power
 *   - Topology: Decentralized mesh with automatic routing
 *   - Security: End-to-end encryption with public/private keys
 *   - Power: Low power consumption for battery operation
 *   - Frequency: ISM bands (868MHz EU, 915MHz US, 433MHz Asia)
 * 
 * Bridge Functionality:
 *   - Text Messaging: Send and receive text messages via mesh
 *   - Node Discovery: Automatic detection of nearby mesh nodes
 *   - Message Routing: Multi-hop routing through intermediate nodes
 *   - Status Monitoring: Network connectivity and node information
 *   - Configuration: Radio parameters and network settings
 * 
 * Integration with SmartFranklin:
 *   - MQTT Bridge: Mesh messages can be forwarded to MQTT brokers
 *   - Data Sharing: Sensor data distributed through mesh network
 *   - Remote Control: Commands received via mesh messaging
 *   - Backup Communication: Alternative to WiFi/cellular when unavailable
 *   - Emergency Communication: Critical messages during network outages
 * 
 * Message Types:
 *   - Text Messages: Human-readable messages for status and alerts
 *   - Data Messages: Structured data (JSON) for sensor readings
 *   - Command Messages: Remote control commands and configuration
 *   - Acknowledgment: Delivery confirmation and status responses
 * 
 * Network Management:
 *   - Node ID: Unique identifier for each mesh node
 *   - Channel: Frequency channel for network segmentation
 *   - Region: Regulatory region settings (EU868, US915, etc.)
 *   - Power: Transmission power level (affects range and battery life)
 *   - Role: Router, client, or repeater node configuration
 * 
 * Performance Characteristics:
 *   - Latency: Variable (seconds to minutes depending on hops)
 *   - Throughput: Low (text messages only, ~10-50 bytes/second)
 *   - Reliability: High (mesh routing with acknowledgments)
 *   - Power Usage: Moderate (LoRa transmission bursts)
 *   - Range: Long (line-of-sight up to 20km)
 * 
 * Dependencies:
 *   - Meshtastic Library: Arduino library for Meshtastic protocol
 *   - LoRa Hardware: Compatible LoRa radio module
 *   - Arduino.h: Basic types and String class
 * 
 * Limitations:
 *   - Text Only: No binary data or large file transfer
 *   - Speed: Low bandwidth compared to WiFi/cellular
 *   - Regulatory: Must comply with local radio regulations
 *   - Hardware: Requires LoRa radio module and antenna
 *   - Single Channel: One mesh network per device
 *   - No Internet: Mesh-only, no direct internet connectivity
 * 
 * Best Practices:
 *   - Use appropriate transmission power for your use case
 *   - Implement message acknowledgments for critical communications
 *   - Monitor mesh network health and connectivity
 *   - Use encryption for sensitive data transmission
 *   - Consider duty cycle limitations for battery-powered operation
 *   - Test range and reliability in target deployment environment
 * 
 * Security Considerations:
 *   - Encryption: Messages encrypted end-to-end by default
 *   - Authentication: Node authentication through public keys
 *   - Channel Security: Private channels for sensitive networks
 *   - Physical Security: Protect LoRa modules from tampering
 *   - Regulatory Compliance: Use licensed frequencies where required
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
 * @brief Initializes the Meshtastic bridge and mesh networking.
 * 
 * Sets up the LoRa radio module and initializes the Meshtastic protocol
 * stack. Configures radio parameters, network settings, and establishes
 * mesh network connectivity. Must be called once during system startup.
 * 
 * Initialization Process:
 *   1. Initialize LoRa radio hardware and SPI interface
 *   2. Load Meshtastic configuration (region, channel, power)
 *   3. Start Meshtastic protocol stack and node services
 *   4. Join configured mesh network and channel
 *   5. Begin listening for incoming mesh messages
 *   6. Register with mesh network (node announcement)
 * 
 * Hardware Requirements:
 *   - LoRa radio module (SX1262, SX1276, or compatible)
 *   - Proper antenna connection and tuning
 *   - SPI interface pins correctly configured
 *   - Power supply meeting radio module requirements
 * 
 * Configuration Parameters:
 *   - Region: Regulatory region (EU868, US915, CN470, etc.)
 *   - Channel: Network channel number for segmentation
 *   - TX Power: Transmission power level (dBm)
 *   - Node Name: Human-readable device identifier
 *   - Role: Device role (router, client, repeater)
 * 
 * Error Handling:
 *   - Radio initialization failures logged and retried
 *   - Network join failures with fallback configurations
 *   - Antenna detection warnings for troubleshooting
 *   - Configuration validation before network join
 * 
 * Performance:
 *   - Initialization Time: 5-15 seconds (radio calibration)
 *   - Memory Usage: Meshtastic library overhead (~20-50KB)
 *   - Power Consumption: Base current + periodic transmissions
 * 
 * Usage Notes:
 *   - Call during system setup after hardware initialization
 *   - Ensure antenna is connected before calling
 *   - Configuration should be set before initialization
 *   - Network connectivity may take time to establish
 * 
 * @note Blocking operation that may take several seconds.
 *       Ensure called early in system initialization.
 *       Radio operation requires proper licensing in some regions.
 * 
 * @see meshtastic_send_text() - Send messages after initialization
 * @see meshtastic_poll_received() - Receive messages after initialization
 */
void meshtastic_bridge_init();

/**
 * @brief Sends a text message through the Meshtastic mesh network.
 * 
 * Transmits a text message to all nodes in the mesh network or to a
 * specific destination. Message is encrypted and routed through the
 * mesh using Meshtastic protocol. Provides reliable delivery with
 * acknowledgments when requested.
 * 
 * @param msg - Text message to send (String object)
 * 
 * Message Transmission:
 *   1. Validate message length and content
 *   2. Encrypt message using network keys
 *   3. Add routing headers and metadata
 *   4. Transmit via LoRa radio with appropriate power
 *   5. Wait for acknowledgments if requested
 *   6. Retry transmission on failures
 * 
 * Message Characteristics:
 *   - Format: Plain text (UTF-8 encoded)
 *   - Length: Limited by LoRa packet size (~200-250 bytes)
 *   - Delivery: Best-effort or acknowledged
 *   - Routing: Automatic mesh routing to destination
 *   - Encryption: End-to-end encryption by default
 * 
 * Destination Options:
 *   - Broadcast: Empty destination sends to all nodes
 *   - Unicast: Specific node ID for direct messaging
 *   - Group: Channel-based group messaging
 * 
 * Reliability:
 *   - Acknowledgments: Optional delivery confirmation
 *   - Retry Logic: Automatic retransmission on failure
 *   - Timeout: Configurable delivery timeout
 *   - Status: Success/failure indication
 * 
 * Performance:
 *   - Transmission Time: 100-500ms per message
 *   - Power Usage: Burst transmission current
 *   - Latency: Variable (depends on mesh topology)
 *   - Throughput: Limited by LoRa duty cycle
 * 
 * Error Handling:
 *   - Message too long: Truncation or rejection
 *   - Radio busy: Queued for later transmission
 *   - No route: Message dropped with error logging
 *   - Encryption failure: Message not sent
 * 
 * Usage Examples:
 *   meshtastic_send_text("System started successfully");
 *   meshtastic_send_text("Alert: Low battery - " + String(battery_level));
 * 
 * @note Function is asynchronous - returns immediately after queuing.
 *       Large messages may be fragmented or rejected.
 *       Duty cycle limitations apply to transmission frequency.
 * 
 * @see meshtastic_poll_received() - Receive response messages
 * @see meshtastic_bridge_init() - Required initialization
 */
void meshtastic_send_text(const String &msg);

/**
 * @brief Polls for received Meshtastic messages from the mesh network.
 * 
 * Checks for incoming messages from the mesh network and returns the
 * most recent message if available. Messages are buffered internally
 * and retrieved on demand. Non-blocking operation for integration
 * with event loops.
 * 
 * @param out - String reference to store received message
 * @return true if message received and stored in out, false if no message
 * 
 * Message Reception:
 *   1. Check internal message buffer for pending messages
 *   2. Decrypt message using network keys
 *   3. Validate message integrity and sender
 *   4. Extract text payload from protocol frame
 *   5. Store message in output parameter
 *   6. Remove message from buffer (one-time delivery)
 * 
 * Message Processing:
 *   - Decryption: Automatic decryption of received messages
 *   - Validation: Checksum and authenticity verification
 *   - Filtering: Optional sender or content filtering
 *   - Buffering: Queue messages for processing
 *   - Acknowledgment: Automatic ACK sending for reliable messages
 * 
 * Buffer Management:
 *   - Queue Size: Limited buffer for incoming messages
 *   - FIFO Order: Oldest messages processed first
 *   - Overflow: Old messages discarded on buffer full
 *   - Thread Safety: Safe for concurrent access
 * 
 * Performance:
 *   - Execution Time: < 1ms when no messages
 *   - Memory Usage: Message buffer storage
 *   - CPU Load: Minimal polling operation
 *   - Real-time: Suitable for frequent polling
 * 
 * Error Handling:
 *   - Decryption failures: Message discarded
 *   - Invalid messages: Logged and ignored
 *   - Buffer overflow: Oldest messages lost
 *   - No messages: Returns false without error
 * 
 * Usage Pattern:
 *   String received_msg;
 *   if (meshtastic_poll_received(received_msg)) {
 *       process_message(received_msg);
 *   }
 * 
 * Integration:
 *   - Event Loop: Call regularly in main loop or task
 *   - MQTT Bridge: Forward messages to MQTT broker
 *   - Command Processing: Parse commands from mesh messages
 *   - Logging: Store received messages for debugging
 * 
 * @note Function removes message from buffer after retrieval.
 *       Call frequently to avoid buffer overflow.
 *       Messages are delivered in reception order.
 * 
 * @see meshtastic_send_text() - Send messages to mesh network
 * @see meshtastic_bridge_init() - Required initialization
 */
bool meshtastic_poll_received(String &out);