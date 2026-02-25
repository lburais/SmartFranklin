#pragma once
#include <Arduino.h>
#include <mutex>

struct SmartData {
    float distance_cm = 0;
    float weight_kg = 0;
    float pitch = 0;
    float roll = 0;
    String rtc_time = "";
    float bms_voltage = 0;
    float bms_current = 0;
    float bms_soc = 0;
    String last_mesh_msg = "";
};

extern SmartData DATA;
extern std::mutex DATA_MUTEX;
