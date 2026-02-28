#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLERemoteCharacteristic.h>

#include "tasks.h"
#include "jbd_bms.h"
#include "data_model.h"
#include "mqtt_layer.h"

static NimBLEClient *client = nullptr;
static NimBLERemoteCharacteristic *notifyChar = nullptr;
static const char *TARGET_NAME = "JBD-BMS";

// ------------------------------------------------------------
// Correct callback class for your version of NimBLE
// ------------------------------------------------------------
class BmsNotifyCallback : public NimBLERemoteCharacteristicCallbacks {
    void onNotify(NimBLERemoteCharacteristic* c,
                  uint8_t* data,
                  size_t len,
                  bool isNotify) override
    {
        JbdFrame f;
        if (jbd_parse_frame(data, len, f)) {

            {
                std::lock_guard<std::mutex> lock(DATA_MUTEX);
                DATA.bms_voltage = f.voltage;
                DATA.bms_current = f.current;
                DATA.bms_soc     = f.soc;
            }

            sf_mqtt::publish("smartfranklin/bms/voltage",
                             std::string(String(f.voltage, 2).c_str()));

            sf_mqtt::publish("smartfranklin/bms/current",
                             std::string(String(f.current, 2).c_str()));

            sf_mqtt::publish("smartfranklin/bms/soc",
                             std::string(String(f.soc, 1).c_str()));
        }
    }
};

static BmsNotifyCallback notifyCb;

// ------------------------------------------------------------
// BLE connection logic
// ------------------------------------------------------------
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

    NimBLERemoteService *svc =
        client->getService("0000ff00-0000-1000-8000-00805f9b34fb");
    if (!svc) return false;

    notifyChar =
        svc->getCharacteristic("0000ff01-0000-1000-8000-00805f9b34fb");
    if (!notifyChar || !notifyChar->canNotify()) return false;

    notifyChar->subscribe(true, &notifyCb);
    return true;
}

// ------------------------------------------------------------
// Task
// ------------------------------------------------------------
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
