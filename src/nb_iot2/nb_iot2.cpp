#include "nb_iot2.h"

NbIot2 NB_IOT2;

void NbIot2::init(HardwareSerial *serial, uint32_t baud, int rx, int tx)
{
    m_serial = serial;
    m_serial->begin(baud, SERIAL_8N1, rx, tx);
    m_inited = true;
}

bool NbIot2::waitFor(const String &token, String *resp, uint32_t timeout)
{
    String buf;
    unsigned long start = millis();
    while (millis() - start < timeout) {
        while (m_serial->available()) {
            char c = m_serial->read();
            buf += c;
            if (buf.indexOf(token) >= 0) {
                if (resp) *resp = buf;
                return true;
            }
        }
        delay(10);
    }
    if (resp) *resp = buf;
    return false;
}

bool NbIot2::sendAT(const String &cmd, String *resp, uint32_t timeout)
{
    if (!m_inited) return false;
    while (m_serial->available()) m_serial->read();
    m_serial->print(cmd);
    m_serial->print("\r");
    String r;
    if (!waitFor("OK", &r, timeout)) {
        if (resp) *resp = r;
        return false;
    }
    if (resp) *resp = r;
    return true;
}

bool NbIot2::ensureModem()
{
    if (m_status.modem_ready) return true;
    String r;
    for (int i = 0; i < 5; ++i) {
        if (sendAT("AT", &r, 1000)) {
            sendAT("ATE0");
            sendAT("AT+CMEE=2");
            sendAT("AT+CFUN=1");
            m_status.modem_ready = true;
            return true;
        }
        delay(500);
    }
    return false;
}

bool NbIot2::ensureNetwork(const String &apn)
{
    if (!ensureModem()) return false;
    String r;
    sendAT("AT+CGDCONT=1,\"IP\",\"" + apn + "\"");
    sendAT("AT+CGATT=1", &r, 20000);
    m_status.network_attached = r.indexOf("OK") >= 0;
    return m_status.network_attached;
}

bool NbIot2::ensurePdp(const String &apn)
{
    if (!ensureNetwork(apn)) return false;
    String r;
    sendAT("AT+CGACT=1,1", &r, 20000);
    m_status.pdp_active = r.indexOf("OK") >= 0;
    if (m_status.pdp_active) {
        sendAT("AT+CGPADDR=1", &r, 5000);
        int idx = r.indexOf("+CGPADDR:");
        if (idx >= 0) {
            int q = r.indexOf(",", idx);
            int e = r.indexOf("\r", q);
            if (q > 0 && e > q) {
                m_status.ip = r.substring(q + 1, e);
                m_status.ip.trim();
            }
        }
    }
    return m_status.pdp_active;
}

bool NbIot2::ensureMqtt(const String &host, int port,
                        const String &user, const String &pass)
{
    if (host.isEmpty()) return false;
    if (m_status.mqtt_connected) return true;

    String r;
    sendAT("AT+SMCONF=\"URL\",\"" + host + "\"," + String(port), &r, 5000);
    if (!user.isEmpty())
        sendAT("AT+SMCONF=\"USERNAME\",\"" + user + "\"", &r, 5000);
    if (!pass.isEmpty())
        sendAT("AT+SMCONF=\"PASSWORD\",\"" + pass + "\"", &r, 5000);

    sendAT("AT+SMCONN", &r, 30000);
    m_status.mqtt_connected = r.indexOf("OK") >= 0;
    return m_status.mqtt_connected;
}

bool NbIot2::connectNetwork(const String &apn)
{
    return ensurePdp(apn);
}

bool NbIot2::mqttConnect(const String &host, int port,
                         const String &user, const String &pass)
{
    return ensureMqtt(host, port, user, pass);
}

bool NbIot2::mqttPublish(const String &topic, const String &payload,
                         int qos, bool retain)
{
    if (!m_status.mqtt_connected) return false;
    String cmd = "AT+SMPUB=\"" + topic + "\"," +
                 String(payload.length()) + "," +
                 String(qos) + "," +
                 String(retain ? 1 : 0);
    String r;
    if (!sendAT(cmd, &r, 5000)) return false;
    m_serial->print(payload);
    m_serial->print("\r");
    return waitFor("OK", &r, 10000);
}

bool NbIot2::getGnss(GnssInfo &out)
{
    String r;
    if (!sendAT("AT+CGNSPWR=1", &r, 5000)) return false;
    sendAT("AT+CGNSINF", &r, 5000);
    int idx = r.indexOf("+CGNSINF:");
    if (idx < 0) return false;
    String line = r.substring(idx);
    int c1 = line.indexOf(",", 0);
    int c2 = line.indexOf(",", c1 + 1);
    int c3 = line.indexOf(",", c2 + 1);
    int c4 = line.indexOf(",", c3 + 1);
    int c5 = line.indexOf(",", c4 + 1);
    if (c3 < 0 || c4 < 0 || c5 < 0) return false;
    out.lat = line.substring(c2 + 1, c3).toDouble();
    out.lon = line.substring(c3 + 1, c4).toDouble();
    out.alt = line.substring(c4 + 1, c5).toFloat();
    out.valid = (out.lat != 0 || out.lon != 0);
    return out.valid;
}

void NbIot2::loop()
{
    if (!m_inited) return;
    unsigned long now = millis();
    if (now - m_lastPing > 10000) {
        String r;
        if (!sendAT("AT", &r, 2000)) {
            m_status.modem_ready = false;
            m_status.network_attached = false;
            m_status.pdp_active = false;
            m_status.mqtt_connected = false;
        }
        m_lastPing = now;
    }
}

NbIotStatus NbIot2::getStatus() const
{
    return m_status;
}
