/*
 * ============================================================================
 * Meshtastic Bridge Task - SmartFranklin
 * ============================================================================
 *
 * File:        task_meshtastic_bridge.cpp
 * Project:     SmartFranklin IoT Device Controller
 * Description: FreeRTOS task that drives Meshtastic connectivity, publishes
 *              node reports to MQTT, and forwards inbound mesh text messages.
 *
 * Author:      Laurent Burais
 * Date:        10 March 2026
 * Version:     1.1
 *
 * ============================================================================
 */

#include <Arduino.h>
#include <M5Unified.h>
#include <string>
#include <mutex>
#include <ArduinoJson.h>

#include "tasks.h"
#include "config_store.h"
#include "meshtastic_bridge.h"
#include "mqtt.h"
#include "data_model.h"

namespace {

/**
 * @brief Convert Meshtastic node report callbacks into MQTT telemetry payloads.
 */
void publish_node_report(mt_node_t *node, mt_nr_progress_t progress)
{
    if (progress == MT_NR_IN_PROGRESS && node != nullptr) {
        JsonDocument doc;
        doc["report_ts_ms"] = millis();
        {
            std::lock_guard<std::mutex> lock(DATA_MUTEX);
            if (!DATA.rtc_time.isEmpty()) {
                doc["report_iso"] = DATA.rtc_time;
            }
        }
        doc["node_num"] = node->node_num;
        doc["is_mine"] = node->is_mine;
        doc["has_user"] = node->has_user;
        doc["user_id"] = node->user_id;
        doc["long_name"] = node->long_name;
        doc["short_name"] = node->short_name;
        doc["latitude"] = node->latitude;
        doc["longitude"] = node->longitude;
        doc["altitude"] = node->altitude;
        doc["ground_speed"] = node->ground_speed;
        doc["battery_level"] = node->battery_level;
        doc["last_heard_from"] = node->last_heard_from;
        doc["last_heard_position"] = node->last_heard_position;
        doc["time_of_last_position"] = node->time_of_last_position;
        doc["voltage"] = node->voltage;
        doc["channel_utilization"] = node->channel_utilization;
        doc["air_util_tx"] = node->air_util_tx;

        String payload;
        serializeJson(doc, payload);
        sf_mqtt::publish("smartfranklin/meshtastic/node_report",
                         std::string(payload.c_str()));
        return;
    }

    if (progress == MT_NR_DONE) {
        sf_mqtt::publish("smartfranklin/meshtastic/node_report_status",
                         std::string("done"));
    } else if (progress == MT_NR_INVALID) {
        sf_mqtt::publish("smartfranklin/meshtastic/node_report_status",
                         std::string("invalid"));
    }
}

} // namespace

/**
 * @brief Meshtastic bridge task loop.
 *
 * Maintains mesh link state, periodically requests node reports, stores last
 * received text in the shared data model, and republishes key events to MQTT.
 */
void taskMeshtasticBridge(void *pv)
{
    (void)pv;

    if (!CONFIG.meshtastic_bridge_enabled) {
        M5_LOGI("[MESHTASTIC] Bridge disabled in config");
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    M5_LOGI("[MESHTASTIC] Bridge task started");
    meshtastic_bridge_init();
    sf_mqtt::publish("smartfranklin/meshtastic/status", std::string("starting"));

    std::string ingressTopic = std::string(CONFIG.meshtastic_mqtt_prefix.c_str()) + "#";
    sf_mqtt::subscribe(ingressTopic, 1);

    bool lastReady = false;
    bool firstReadySample = true;
    uint32_t lastNodeReportRequestMs = 0;
    const uint32_t nodeReportPeriodMs = 300000;

    for (;;) {
        meshtastic_bridge_tick();

        bool ready = meshtastic_bridge_is_ready();
        if (firstReadySample || ready != lastReady) {
            sf_mqtt::publish("smartfranklin/meshtastic/status",
                             std::string(ready ? "connected" : "disconnected"));
            lastReady = ready;
            firstReadySample = false;
        }

        uint32_t nowMs = millis();
        if (ready && (lastNodeReportRequestMs == 0 || (nowMs - lastNodeReportRequestMs) >= nodeReportPeriodMs)) {
            bool requested = meshtastic_bridge_request_node_report(publish_node_report);
            sf_mqtt::publish("smartfranklin/meshtastic/node_report_status",
                             std::string(requested ? "requested" : "request_failed"));
            lastNodeReportRequestMs = nowMs;
        }

        String msg;
        if (meshtastic_poll_received(msg)) {
            {
                std::lock_guard<std::mutex> lock(DATA_MUTEX);
                DATA.last_mesh_msg = msg;
            }

            sf_mqtt::publish("smartfranklin/meshtastic/text",
                             std::string(msg.c_str()));
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
