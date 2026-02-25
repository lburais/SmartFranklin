#pragma once
#include <Arduino.h>

struct NbIotStatus {
    bool   modem_ready = false;
    bool   network_attached = false;
    bool   pdp_active = false;
    bool   mqtt_connected = false;
    String operator_name;
    String ip;
    int    rssi = 0;
};

struct GnssInfo {
    bool   valid = false;
    double lat = 0;
    double lon = 0;
    float  alt = 0;
};

class NbIot2 {
public:
    void init(HardwareSerial *serial, uint32_t baud, int rx, int tx);
    bool connectNetwork(const String &apn);
    bool mqttConnect(const String &host, int port,
                     const String &user, const String &pass);
    bool mqttPublish(const String &topic, const String &payload,
                     int qos = 0, bool retain = false);
    void loop();
    bool getGnss(GnssInfo &out);
    NbIotStatus getStatus() const;

private:
    HardwareSerial *m_serial = nullptr;
    NbIotStatus m_status;
    unsigned long m_lastPing = 0;
    bool m_inited = false;

    bool sendAT(const String &cmd, String *resp = nullptr,
                uint32_t timeout = 5000);
    bool waitFor(const String &token, String *resp,
                 uint32_t timeout);
    bool ensureModem();
    bool ensureNetwork(const String &apn);
    bool ensurePdp(const String &apn);
    bool ensureMqtt(const String &host, int port,
                    const String &user, const String &pass);
};

extern NbIot2 NB_IOT2;
