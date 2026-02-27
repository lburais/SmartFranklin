#pragma once

#include <string>
#include <functional>

extern "C" {
#include "mqtt_client.h"
}

namespace sf_mqtt {

using MessageCallback = std::function<void(const std::string &topic,
                                           const std::string &payload)>;

struct Config {
    std::string uri;          // e.g. "mqtt://broker.hivemq.com"
    std::string username;     // optional
    std::string password;     // optional
    std::string client_id;    // optional, can be empty
    bool        clean_session = true;
    int         keepalive_sec = 60;
    bool        use_tls       = false;
};

bool init(const Config &cfg, MessageCallback cb);
bool is_connected();
bool publish(const std::string &topic,
             const std::string &payload,
             int qos = 1,
             bool retain = false);
bool subscribe(const std::string &topic, int qos = 1);

// kept for compatibility if you had a loop() call somewhere
inline void loop() {}

} // namespace sf_mqtt
