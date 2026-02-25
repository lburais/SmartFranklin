#include <Arduino.h>
#include <NimBLEDevice.h>
#include "tasks.h"
#include "jbd_bms.h"
#include "data_model.h"
#include "mqtt_layer.h"

static NimBLEClient *client = nullptr;
static NimBLERemoteCharacteristic *notifyChar = nullptr;
static const char *TARGET_NAME = "JBD-BMS";

class BmsNotifyCallback : public NimBLERemoteCharacteristicCallbacks {
    void onNotify(NimBLERemoteCharacteristic *c, uint8_t *data, size_t len, bool) override {
        JbdFrame f;
        if (jbd_parse_frame(data, len, f)) {
            {
                std::lock_guard<std::mutex> lock(DATA_MUTEX);
                DATA.bms_voltage = f.voltage;
                DATA.bms_current = f.current;
                DATA.bms_soc = f.soc;
            }
            mqtt_publish("smartfranklin/bms/voltage", String(f.voltage, 2));
            mqtt_publish("smartfranklin/bms/current", String(f.current, 2));
            mqtt_publish("smartfranklin/bms/soc", String(f.soc, 1));
        }
    }
};

static BmsNotifyCallback notifyCb;

static bool connectToBms()
{
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
    if (!target) return false;
    client = NimBLEDevice::createClient();
    if (!client->connect(target)) return false;
    NimBLERemoteService *svc = client->getService("0000ff00-0000-1000-8000-00805f9b34fb");
    if (!svc) return false;
    notifyChar = svc->getCharacteristic("0000ff01-0000-1000-8000-00805f9b34fb");
    if (!notifyChar || !notifyChar->canNotify()) return false;
    notifyChar->subscribe(true, &notifyCb);
    return true;
}

void taskBmsBle(void *pv)
{
    Serial.println("[BMS_BLE] Task started");
    NimBLEDevice::init("SmartFranklin");
    for (;;) {
        if (!client || !client->isConnected()) {
            Serial.println("[BMS_BLE] Connecting...");
            if (!connectToBms()) {
                Serial.println("[BMS_BLE] Connect failed, retry in 5s");
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }
            Serial.println("[BMS_BLE] Connected.");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
