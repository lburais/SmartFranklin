#include <unity.h>
#include "plugin_manager.h"

class DummyPlugin : public Plugin {
public:
    bool setupCalled = false;
    bool loopCalled = false;
    const char* name() override { return "Dummy"; }
    void setup() override { setupCalled = true; }
    void loop() override { loopCalled = true; }
};

DummyPlugin plugin;

void test_plugin_registration() {
    plugin_register(&plugin);
    plugin_setup_all();
    plugin_loop_all();
    TEST_ASSERT_TRUE(plugin.setupCalled);
    TEST_ASSERT_TRUE(plugin.loopCalled);
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_plugin_registration);
    UNITY_END();
}

void loop() {}
