#include "mqtt_bridge.h"
#include "config_store.h"

#include <PubSubClient.h>
#include <WiFi.h>

// -----------------------------------------------------------------------------
// MQTT clients
// -----------------------------------------------------------------------------
static WiFiClient extClient;
static PubSubClient extMqtt(extClient);

static WiFiClient intClient;
static PubSubClient intMqtt(intClient);

static bool extConnected = false;
static bool intConnected = false;

static unsigned long lastIntToExt = 0;
static unsigned long lastExtToInt = 0;

static uint32_t countIntToExt = 0;
static uint32_t countExtToInt = 0;

// -----------------------------------------------------------------------------
// Loop detection
// -----------------------------------------------------------------------------
static bool isLoopMessage(const String &topic)
{
    if (!CONFIG.mqtt_bridge_loop_detection) return false;

    return topic.startsWith(CONFIG.mqtt_bridge_prefix_internal) ||
           topic.startsWith(CONFIG.mqtt_bridge_prefix_external);
}

// -----------------------------------------------------------------------------
// Forward internal → external
// -----------------------------------------------------------------------------
static void forwardToExternal(char *topic, byte *payload, unsigned int len)
{
    if (!CONFIG.mqtt_bridge_enabled || !extConnected) return;

    String t = String(topic);
    if (isLoopMessage(t)) return;

    String outTopic = CONFIG.mqtt_bridge_prefix_external + t;

    // PubSubClient publish signature:
    // publish(const char* topic, const uint8_t* payload, unsigned int length, bool retained)
    extMqtt.publish(outTopic.c_str(), payload, len, CONFIG.mqtt_bridge_retain);

    lastIntToExt = millis();
    countIntToExt++;
}

// -----------------------------------------------------------------------------
// Forward external → internal
// -----------------------------------------------------------------------------
static void forwardToInternal(char *topic, byte *payload, unsigned int len)
{
    if (!CONFIG.mqtt_bridge_enabled || !intConnected) return;

    String t = String(topic);
    if (isLoopMessage(t)) return;

    String outTopic = CONFIG.mqtt_bridge_prefix_internal + t;

    intMqtt.publish(outTopic.c_str(), payload, len, CONFIG.mqtt_bridge_retain);

    lastExtToInt = millis();
    countExtToInt++;
}

// -----------------------------------------------------------------------------
// Init
// -----------------------------------------------------------------------------
void mqtt_bridge_init()
{
    // Internal broker (local Mosquitto)
    intMqtt.setServer("127.0.0.1", 1883);
    intMqtt.setCallback(forwardToExternal);

    // External broker (optional)
    if (CONFIG.ext_mqtt_enabled) {
        extMqtt.setServer(CONFIG.ext_mqtt_host.c_str(), CONFIG.ext_mqtt_port);
        extMqtt.setCallback(forwardToInternal);
    }
}

// -----------------------------------------------------------------------------
// Ensure internal broker connection
// -----------------------------------------------------------------------------
static void ensureInternal()
{
    if (!intMqtt.connected()) {
        intConnected = intMqtt.connect(
            "SmartFranklinBridgeInternal",
            CONFIG.admin_user.c_str(),
            CONFIG.admin_pass.c_str()
        );

        if (intConnected) {
            // PubSubClient supports QoS 0 or 1 only
            uint8_t qos = CONFIG.mqtt_bridge_qos > 1 ? 1 : CONFIG.mqtt_bridge_qos;
            intMqtt.subscribe("#", qos);
        }
    }
}

// -----------------------------------------------------------------------------
// Ensure external broker connection
// -----------------------------------------------------------------------------
static void ensureExternal()
{
    if (!CONFIG.ext_mqtt_enabled) return;

    if (!extMqtt.connected()) {
        extConnected = extMqtt.connect(
            "SmartFranklinBridgeExternal",
            CONFIG.ext_mqtt_user.c_str(),
            CONFIG.ext_mqtt_pass.c_str()
        );

        if (extConnected) {
            uint8_t qos = CONFIG.mqtt_bridge_qos > 1 ? 1 : CONFIG.mqtt_bridge_qos;
            extMqtt.subscribe("#", qos);
        }
    }
}

// -----------------------------------------------------------------------------
// Loop
// -----------------------------------------------------------------------------
void mqtt_bridge_loop()
{
    ensureInternal();
    ensureExternal();

    if (intConnected) intMqtt.loop();
    if (extConnected) extMqtt.loop();
}

// -----------------------------------------------------------------------------
// Status
// -----------------------------------------------------------------------------
void mqtt_bridge_status(JsonDocument &doc)
{
    doc["internal_connected"] = intConnected;
    doc["external_connected"] = extConnected;
    doc["last_int_to_ext_ms"] = lastIntToExt;
    doc["last_ext_to_int_ms"] = lastExtToInt;
    doc["count_int_to_ext"] = countIntToExt;
    doc["count_ext_to_int"] = countExtToInt;
}
