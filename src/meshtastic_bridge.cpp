#include "meshtastic_bridge.h"

#include <Arduino.h>
#include <Meshtastic.h>
#include <mutex>

#include "config_store.h"

namespace {

std::mutex s_api_mutex;
std::mutex s_rx_mutex;
bool s_initialized = false;
bool s_ready = false;
bool s_has_message = false;
String s_last_message;

void on_mesh_text(uint32_t from, uint32_t to, uint8_t channel, const char *text)
{
    (void)from;
    (void)to;
    (void)channel;

    std::lock_guard<std::mutex> lock(s_rx_mutex);
    s_last_message = text ? String(text) : String();
    s_has_message = true;
}

} // namespace

void meshtastic_bridge_init()
{
    if (!CONFIG.meshtastic_bridge_enabled || s_initialized) {
        return;
    }

    std::lock_guard<std::mutex> lock(s_api_mutex);
    mt_serial_init(CONFIG.meshtastic_pin_rx,
                   CONFIG.meshtastic_pin_tx,
                   static_cast<uint32_t>(CONFIG.meshtastic_baud));
    set_text_message_callback(on_mesh_text);
    s_ready = false;
    s_initialized = true;
}

void meshtastic_bridge_tick()
{
    if (!CONFIG.meshtastic_bridge_enabled || !s_initialized) {
        return;
    }

    std::lock_guard<std::mutex> lock(s_api_mutex);
    s_ready = mt_loop(millis());
}

bool meshtastic_bridge_is_ready()
{
    if (!CONFIG.meshtastic_bridge_enabled || !s_initialized) {
        return false;
    }

    std::lock_guard<std::mutex> lock(s_api_mutex);
    return s_ready;
}

void meshtastic_send_text(const String &msg)
{
    if (!CONFIG.meshtastic_bridge_enabled || !s_initialized || msg.isEmpty()) {
        return;
    }

    std::lock_guard<std::mutex> lock(s_api_mutex);
    mt_send_text(msg.c_str(), BROADCAST_ADDR, 0);
}

bool meshtastic_poll_received(String &out)
{
    if (!CONFIG.meshtastic_bridge_enabled || !s_initialized) {
        return false;
    }

    std::lock_guard<std::mutex> lock(s_rx_mutex);
    if (!s_has_message) {
        return false;
    }

    out = s_last_message;
    s_last_message = "";
    s_has_message = false;
    return true;
}

void meshtastic_bridge_handle_mqtt(const String &topic, const String &payload)
{
    if (!CONFIG.meshtastic_bridge_enabled || !s_initialized) {
        return;
    }

    if (!topic.startsWith(CONFIG.meshtastic_mqtt_prefix)) {
        return;
    }

    meshtastic_send_text(payload);
}

bool meshtastic_bridge_request_node_report(void (*callback)(mt_node_t *, mt_nr_progress_t))
{
    if (!CONFIG.meshtastic_bridge_enabled || !s_initialized || callback == nullptr) {
        return false;
    }

    std::lock_guard<std::mutex> lock(s_api_mutex);
    if (!s_ready) {
        return false;
    }

    return mt_request_node_report(callback);
}
