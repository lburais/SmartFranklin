#include "command_handler.h"
#include "data_model.h"
#include "mqtt_layer.h"
#include "tasks.h"

#include <ArduinoJson.h>

static void handle_set(JsonDocument& doc) {
    if (doc["led"].is<bool>()) {
        bool state = doc["led"];
        DATA.led_state = state;
        sf_mqtt::publish("smartfranklin/led/state", state ? "1" : "0");
    }

    if (doc["buzzer"].is<bool>()) {
        bool state = doc["buzzer"];
        DATA.buzzer_state = state;
        sf_mqtt::publish("smartfranklin/buzzer/state", state ? "1" : "0");
    }

    if (doc["target_soc"].is<int>()) {
        int soc = doc["target_soc"];
        DATA.target_soc = soc;
        sf_mqtt::publish("smartfranklin/bms/target_soc", String(soc).c_str());
    }
}

static void handle_get(JsonDocument& doc) {
    if (doc["what"].is<String>()) {
        String what = doc["what"].as<String>();

        if (what == "bms") {
            JsonDocument out;
            out["voltage"] = DATA.bms_voltage;
            out["current"] = DATA.bms_current;
            out["soc"]     = DATA.bms_soc;

            String payload;
            serializeJson(out, payload);
            sf_mqtt::publish("smartfranklin/bms/status", payload.c_str());
        }

        if (what == "system") {
            JsonDocument out;
            out["uptime"] = millis();
            out["free_heap"] = ESP.getFreeHeap();

            String payload;
            serializeJson(out, payload);
            sf_mqtt::publish("smartfranklin/system/status", payload.c_str());
        }
    }
}

void command_handler_process(const String& json) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, json);

    if (err) {
        sf_mqtt::publish("smartfranklin/error", "invalid_json");
        return;
    }

    if (doc["cmd"].is<String>()) {
        String cmd = doc["cmd"].as<String>();

        if (cmd == "set") {
            handle_set(doc);
            return;
        }

        if (cmd == "get") {
            handle_get(doc);
            return;
        }

        sf_mqtt::publish("smartfranklin/error", "unknown_cmd");
        return;
    }

    sf_mqtt::publish("smartfranklin/error", "missing_cmd");
}
