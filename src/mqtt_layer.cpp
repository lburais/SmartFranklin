#include "mqtt_layer.h"

#include <cstring>
#include <esp_log.h>
#include <mqtt_client.h>

namespace sf_mqtt {

static const char *TAG = "SF_MQTT";

static esp_mqtt_client_handle_t s_client = nullptr;
static MessageCallback          s_msg_cb = nullptr;
static bool                     s_connected = false;

static esp_err_t event_handler_cb(esp_mqtt_event_handle_t event)
{
    switch (event->event_id) {

    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        s_connected = true;
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        s_connected = false;
        break;

    case MQTT_EVENT_DATA: {
        std::string topic(event->topic, event->topic_len);
        std::string payload(event->data, event->data_len);

        ESP_LOGD(TAG, "Incoming: %s => %s", topic.c_str(), payload.c_str());

        if (s_msg_cb) {
            s_msg_cb(topic, payload);
        }
        break;
    }

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
        break;

    default:
        ESP_LOGD(TAG, "Unhandled MQTT event id: %d", event->event_id);
        break;
    }

    return ESP_OK;
}

bool init(const Config &cfg, MessageCallback cb)
{
    if (s_client != nullptr) {
        ESP_LOGW(TAG, "MQTT already initialized");
        return true;
    }

    s_msg_cb = cb;

    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.uri = cfg.uri.c_str();

    if (!cfg.username.empty()) {
        mqtt_cfg.username = cfg.username.c_str();
    }
    if (!cfg.password.empty()) {
        mqtt_cfg.password = cfg.password.c_str();
    }
    if (!cfg.client_id.empty()) {
        mqtt_cfg.client_id = cfg.client_id.c_str();
    }

    mqtt_cfg.keepalive = cfg.keepalive_sec;
    mqtt_cfg.disable_clean_session = !cfg.clean_session;

    mqtt_cfg.event_handle = event_handler_cb;

    s_client = esp_mqtt_client_init(&mqtt_cfg);
    if (!s_client) {
        ESP_LOGE(TAG, "Failed to init MQTT client");
        return false;
    }

    esp_err_t err = esp_mqtt_client_start(s_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %d", err);
        s_client = nullptr;
        return false;
    }

    ESP_LOGI(TAG, "MQTT client started with URI: %s", cfg.uri.c_str());
    return true;
}

bool is_connected()
{
    return s_connected;
}

bool publish(const std::string &topic,
             const std::string &payload,
             int qos,
             bool retain)
{
    if (!s_client) {
        ESP_LOGW(TAG, "publish() called but MQTT client not initialized");
        return false;
    }

    int msg_id = esp_mqtt_client_publish(
        s_client,
        topic.c_str(),
        payload.c_str(),
        payload.size(),
        qos,
        retain ? 1 : 0
    );

    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to publish to %s", topic.c_str());
        return false;
    }

    ESP_LOGD(TAG, "Publish queued to %s (msg_id=%d)", topic.c_str(), msg_id);
    return true;
}

bool subscribe(const std::string &topic, int qos)
{
    if (!s_client) {
        ESP_LOGW(TAG, "subscribe() called but MQTT client not initialized");
        return false;
    }

    int msg_id = esp_mqtt_client_subscribe(
        s_client,
        topic.c_str(),
        qos
    );

    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to subscribe to %s", topic.c_str());
        return false;
    }

    ESP_LOGI(TAG, "Subscribe requested for %s (msg_id=%d)", topic.c_str(), msg_id);
    return true;
}

} // namespace sf_mqtt
