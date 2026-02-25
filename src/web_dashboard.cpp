#include "m5_hw.h"
#include "m5_hw.h"
#include "nb_iot2.h"
#include "web_dashboard.h"
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <AsyncElegantOTA.h>
#include <FS.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include "data_model.h"
#include "config_store.h"
#include "mqtt_bridge.h"

static AsyncWebServer server(80);

static const char MAIN_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SmartFranklin</title>
<style>
body { font-family: sans-serif; margin: 10px; }
.card { background: #f4f4f4; padding: 15px; border-radius: 10px; margin-bottom: 15px; }
pre { white-space: pre-wrap; word-wrap: break-word; }
a.button { display: inline-block; padding: 10px 15px; background: #0078ff; color: white; border-radius: 6px; text-decoration: none; margin-right: 5px; }
</style>
</head>
<body>
<h1>SmartFranklin</h1>
<div class="card">
  <a class="button" href="/config">Configuration</a>
  <a class="button" href="/bridge">MQTT Bridge</a>
  <a class="button" href="/update">Firmware Update</a>
  <a class="button" href="/diagnostics">Diagnostics</a>
</div>
<div class="card">
  <h3>Live Data</h3>
  <pre id="data">Loading...</pre>
</div>
<script>
async function refresh() {
  const r = await fetch('/api/status');
    server.on("/api/hw", HTTP_GET, [](AsyncWebServerRequest *request){ 
        DynamicJsonDocument doc(256); 
        HwStatus st = HW.read(); 
        doc["battery_voltage"] = st.battery_voltage; 
        doc["battery_percent"] = st.battery_percent; 
        doc["charging"]        = st.charging; 
        doc["temperature"]     = st.temperature; 
        doc["button_a"]        = st.button_a; 
        doc["button_b"]        = st.button_b; 
        JsonObject accel = doc.createNestedObject("accel"); 
        accel["x"] = st.accel_x; 
        accel["y"] = st.accel_y; 
        accel["z"] = st.accel_z; 
        String out; 
        serializeJson(doc, out); 
        request->send(200, "application/json", out); 
    });
    server.on("/api/nbiot", HTTP_GET, [](AsyncWebServerRequest *request){ 
        DynamicJsonDocument doc(256); 
        NbIotStatus st = NB_IOT2.getStatus(); 
        doc["modem_ready"]      = st.modem_ready; 
        doc["network_attached"] = st.network_attached; 
        doc["pdp_active"]       = st.pdp_active; 
        doc["mqtt_connected"]   = st.mqtt_connected; 
        doc["ip"]               = st.ip; 
        doc["rssi"]             = st.rssi; 
        doc["operator"]         = st.operator_name; 
        String out; 
        serializeJson(doc, out); 
        request->send(200, "application/json", out); 
    });
  const j = await r.json();
  document.getElementById('data').textContent = JSON.stringify(j, null, 2);
}
setInterval(refresh, 1000);
refresh();
</script>
</body>
</html>
)HTML";

static bool checkAuth(AsyncWebServerRequest *request) {
    if (!request->authenticate(CONFIG.admin_user.c_str(),
                               CONFIG.admin_pass.c_str())) {
        request->requestAuthentication("SmartFranklin Config");
        return false;
    }
    return true;
}

void web_dashboard_init()
{
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", MAIN_PAGE);
    });

    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request){
    server.on("/api/hw", HTTP_GET, [](AsyncWebServerRequest *request){ 
        DynamicJsonDocument doc(256); 
        HwStatus st = HW.read(); 
        doc["battery_voltage"] = st.battery_voltage; 
        doc["battery_percent"] = st.battery_percent; 
        doc["charging"]        = st.charging; 
        doc["temperature"]     = st.temperature; 
        doc["button_a"]        = st.button_a; 
        doc["button_b"]        = st.button_b; 
        JsonObject accel = doc.createNestedObject("accel"); 
        accel["x"] = st.accel_x; 
        accel["y"] = st.accel_y; 
        accel["z"] = st.accel_z; 
        String out; 
        serializeJson(doc, out); 
        request->send(200, "application/json", out); 
    });
    server.on("/api/nbiot", HTTP_GET, [](AsyncWebServerRequest *request){ 
        DynamicJsonDocument doc(256); 
        NbIotStatus st = NB_IOT2.getStatus(); 
        doc["modem_ready"]      = st.modem_ready; 
        doc["network_attached"] = st.network_attached; 
        doc["pdp_active"]       = st.pdp_active; 
        doc["mqtt_connected"]   = st.mqtt_connected; 
        doc["ip"]               = st.ip; 
        doc["rssi"]             = st.rssi; 
        doc["operator"]         = st.operator_name; 
        String out; 
        serializeJson(doc, out); 
        request->send(200, "application/json", out); 
    });
        DynamicJsonDocument doc(512);
        {
            std::lock_guard<std::mutex> lock(DATA_MUTEX);
            doc["distance_cm"] = DATA.distance_cm;
            doc["weight_kg"] = DATA.weight_kg;
            doc["pitch"] = DATA.pitch;
            doc["roll"] = DATA.roll;
            doc["rtc_time"] = DATA.rtc_time;
            doc["bms_voltage"] = DATA.bms_voltage;
            doc["bms_current"] = DATA.bms_current;
            doc["bms_soc"] = DATA.bms_soc;
            doc["last_mesh_msg"] = DATA.last_mesh_msg;
        }
        String out;
        serializeJson(doc, out);
        request->send(200, "application/json", out);
    });

    AsyncElegantOTA.begin(&server, CONFIG.admin_user.c_str(), CONFIG.admin_pass.c_str());
    server.on("/hw", HTTP_GET, [](AsyncWebServerRequest *req){ req->send(SPIFFS, "/hw.html", "text/html"); }); 
    server.on("/nbiot", HTTP_GET, [](AsyncWebServerRequest *req){ req->send(SPIFFS, "/nbiot.html", "text/html"); }); 
    server.on("/meshtastic", HTTP_GET, [](AsyncWebServerRequest *req){ req->send(SPIFFS, "/meshtastic.html", "text/html"); }); 
    server.on("/sensors", HTTP_GET, [](AsyncWebServerRequest *req){ req->send(SPIFFS, "/sensors.html", "text/html"); }); 
    server.on("/theme.js", HTTP_GET, [](AsyncWebServerRequest *req){ req->send(SPIFFS, "/theme.js", "application/javascript"); }); 
    
    server.on("/api/set_brightness", HTTP_GET, [](AsyncWebServerRequest *req){ 
        if (req->hasParam("value")) { int v = req->getParam("value")->value().toInt(); HW.setBrightness(v); } 
        req->send(200, "text/plain", "OK"); 
    }); 
    server.on("/api/reboot", HTTP_GET, [](AsyncWebServerRequest *req){ 
        req->send(200, "text/plain", "Rebooting"); delay(200); ESP.restart(); 
    }); 
    server.on("/api/sleep", HTTP_GET, [](AsyncWebServerRequest *req){ 
        req->send(200, "text/plain", "Sleeping"); delay(200); HW.deepSleep(); 
    });
    server.begin();
}
