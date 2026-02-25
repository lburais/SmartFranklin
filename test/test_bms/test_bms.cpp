#include <unity.h>
#include "jbd_bms.h"

void test_jbd_parse() {
    uint8_t frame[] = {
        0xDD, 0x03, 0x00, 0x00,
        0x0F, 0xA0,
        0xFF, 0x38,
        0x64,
        0x00, 0x00, 0x77
    };
    JbdFrame f;
    TEST_ASSERT_TRUE(jbd_parse_frame(frame, sizeof(frame), f));
    TEST_ASSERT_FLOAT_WITHIN(0.01, 4.0, f.voltage);
    TEST_ASSERT_FLOAT_WITHIN(0.01, -0.2, f.current);
    TEST_ASSERT_EQUAL_FLOAT(100, f.soc);
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_jbd_parse);
    UNITY_END();
}

void loop() {}
