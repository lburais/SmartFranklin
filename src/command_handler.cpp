#include "command_handler.h"
#include "meshtastic_bridge.h"
#include "config_store.h"
#include <ArduinoJson.h>

static int g_screen = 0;

void command_handler_init()
{
    g_screen = 0;
}

int command_get_display_screen()
{
    return g_screen;
}

static void handleDisplayCmd(const String &payload)
{
    JsonDocument doc;
    if (deserializeJson(doc, payload)) return;
    if (doc.containsKey("screen")) {
        g_screen = doc["screen"].as<int>();
    }
}

static void handleMeshCmd(const String &payload)
{
    JsonDocument doc;
    if (deserializeJson(doc, payload)) return;
    if (doc.containsKey("text")) {
        meshtastic_send_text(doc["text"].as<String>());
    }
}

static void handleScaleCmd(const String &payload)
{
    JsonDocument doc;
    if (deserializeJson(doc, payload)) return;
    String action = doc["action"] | "";
    if (action == "tare") {
        // TODO: scale_tare();
    } else if (action == "calibrate") {
        float v = doc["value"] | 0.0f;
        // TODO: scale_set_cal_factor(v);
    }
}

void command_handle(const String &topic, const String &payload)
{
    if (topic == "smartfranklin/cmd/display") {
        handleDisplayCmd(payload);
    } else if (topic == "smartfranklin/cmd/mesh") {
        handleMeshCmd(payload);
    } else if (topic == "smartfranklin/cmd/scale") {
        handleScaleCmd(payload);
    }
}
