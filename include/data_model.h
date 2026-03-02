#pragma once
#include <Arduino.h>
#include <mutex>

struct SmartData {
    float distance_cm = 0;
    float pitch = 0;
    float roll = 0;
    String rtc_time = "";
    String last_mesh_msg = "";

    // Weight
    int32_t weight_g = 0;
    int32_t gap = 0;

    // BMS
    float bms_voltage = 0.0f;
    float bms_current = 0.0f;
    float bms_soc     = 0.0f;

    // Actuators
    bool led_state    = false;
    bool buzzer_state = false;

    // Settings
    int target_soc    = 80;
};

extern SmartData DATA;
extern std::mutex DATA_MUTEX;

