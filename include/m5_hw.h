#pragma once
#include <Arduino.h>
#include <M5Unified.h>

struct HwStatus {
    float battery_voltage = 0;
    float battery_percent = 0;
    bool  charging = false;
    float temperature = 0;
    bool  button_a = false;
    bool  button_b = false;
    float accel_x = 0;
    float accel_y = 0;
    float accel_z = 0;
};

class M5Hardware {
public:
    void init();
    HwStatus read();
    void setBrightness(uint8_t level);
    void deepSleep();

private:
    HwStatus status;
};

extern M5Hardware HW;
