#include <unity.h>
#include "config_store.h"

void test_config_roundtrip() {
    CONFIG.sta_ssid = "TestSSID";
    CONFIG.sta_pass = "TestPass";
    CONFIG.scale_cal_factor = 2.5f;
    CONFIG.ext_mqtt_host = "broker";
    CONFIG.ext_mqtt_port = 1883;
    CONFIG.ext_mqtt_enabled = true;
    CONFIG.mqtt_bridge_enabled = true;
    CONFIG.mqtt_bridge_prefix_internal = "local/";
    CONFIG.mqtt_bridge_prefix_external = "cloud/";
    CONFIG.mqtt_bridge_qos = 1;
    CONFIG.mqtt_bridge_retain = false;
    CONFIG.mqtt_bridge_loop_detection = true;

    TEST_ASSERT_TRUE(config_save());
    TEST_ASSERT_TRUE(config_load());
    TEST_ASSERT_EQUAL_STRING("TestSSID", CONFIG.sta_ssid.c_str());
    TEST_ASSERT_EQUAL_STRING("TestPass", CONFIG.sta_pass.c_str());
    TEST_ASSERT_EQUAL_FLOAT(2.5f, CONFIG.scale_cal_factor);
    TEST_ASSERT_EQUAL_STRING("broker", CONFIG.ext_mqtt_host.c_str());
    TEST_ASSERT_EQUAL(1883, CONFIG.ext_mqtt_port);
    TEST_ASSERT_TRUE(CONFIG.ext_mqtt_enabled);
    TEST_ASSERT_TRUE(CONFIG.mqtt_bridge_enabled);
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_config_roundtrip);
    UNITY_END();
}

void loop() {}
