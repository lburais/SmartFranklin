#pragma once
#include <Arduino.h>

struct JbdFrame {
    float voltage;
    float current;
    float soc;
};

bool jbd_parse_frame(const uint8_t *data, size_t len, JbdFrame &out);
