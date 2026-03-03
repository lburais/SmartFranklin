#include <Arduino.h>
#include <NimBLEDevice.h>
//#include <NimBLERemoteCharacteristic.h>

#include "tasks.h"
#include "jbd_bms.h"
#include "data_model.h"
#include "mqtt_layer.h"

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------
static NimBLEClient *client = nullptr;
static NimBLERemoteCharacteristic *notifyChar = nullptr;

static const char *TARGET_NAME   = "JBD-BMS";
static const char *UUID_SERVICE  = "0000ff00-0000-1000-8000-00805f9b34fb";
static const char *UUID_NOTIFY   = "0000ff01-0000-1000-8000-00805f9b34fb";

// -----------------------------------------------------------------------------
// Notification callback
// -----------------------------------------------------------------------------
class BmsNotifyCallback : public NimBLERemoteCharacteristicCallbacks {
    void onNotify(NimBLERemoteCharacteristic *c,
                  uint8_t *data,
                  size_t len,
                  bool isNotify) override
    {
        JbdFrame f;
        if (!jbd_parse_frame(data, len, f)) {
            Serial.println("[BMS_BLE] Invalid frame");
            return;
        }

        {
            std::lock_guard<std::mutex> lock(DATA_MUTEX);
            DATA.bms_voltage = f.voltage;
            DATA.bms_current = f.current;
            DATA.bms_soc     = f.soc;
        }

        sf_mqtt::publish("smartfranklin/bms/voltage",
                         String(f.voltage, 2).c_str());
        sf_mqtt::publish("smartfranklin/bms/current",
                         String(f.current, 2).c_str());
        sf_mqtt::publish("smartfranklin/bms/soc",
                         String(f.soc, 1).c_str());
    }
};

static BmsNotifyCallback notifyCb;

// -----------------------------------------------------------------------------
// BLE connect logic
// -----------------------------------------------------------------------------
static bool connectToBms()
{
    Serial.println("[BMS_BLE] Scanning...");

    NimBLEScan *scan = NimBLEDevice::getScan();
    scan->setActiveScan(true);
    NimBLEScanResults results = scan->start(5, false);

    NimBLEAdvertisedDevice *target = nullptr;

    for (int i = 0; i < results.getCount(); ++i) {
        auto &dev = results.getDevice(i);
        if (dev.getName() == TARGET_NAME) {
            target = &dev;
            break;
        }
    }

    if (!target) {
        Serial.println("[BMS_BLE] Device not found");
        return false;
    }

    client = NimBLEDevice::createClient();
    if (!client->connect(target)) {
        Serial.println("[BMS_BLE] Connect failed");
        return false;
    }

    NimBLERemoteService *svc = client->getService(UUID_SERVICE);
    if (!svc) {
        Serial.println("[BMS_BLE] Service missing");
        return false;
    }

    notifyChar = svc->getCharacteristic(UUID_NOTIFY);
    if (!notifyChar || !notifyChar->canNotify()) {
        Serial.println("[BMS_BLE] Notify characteristic missing");
        return false;
    }

    if (!notifyChar->subscribe(true, &notifyCb)) {
        Serial.println("[BMS_BLE] Subscribe failed");
        return false;
    }

    Serial.println("[BMS_BLE] Subscribed to notifications");
    return true;
}

// -----------------------------------------------------------------------------
// Task
// -----------------------------------------------------------------------------
void taskBmsBle(void *pv)
{
    Serial.println("[BMS_BLE] Task started");

    NimBLEDevice::init("SmartFranklin");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    for (;;) {
        if (!client || !client->isConnected()) {
            Serial.println("[BMS_BLE] Connecting...");
            if (!connectToBms()) {
                Serial.println("[BMS_BLE] Retry in 5s");
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }
            Serial.println("[BMS_BLE] Connected");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
