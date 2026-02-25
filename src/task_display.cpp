#include <Arduino.h>
#include <M5Unified.h>
#include "tasks.h"
#include "data_model.h"
#include "command_handler.h"
#include "config_store.h"
#include "scale_control.h"

static int screen = 0;
static bool calib_in_progress = false;
static float calib_known_weight = 1.0f;

void draw()
{
    M5.Display.clear();
    std::lock_guard<std::mutex> lock(DATA_MUTEX);
    if (screen == 0) {
        M5.Display.printf("Distance: %.1f cm\n", DATA.distance_cm);
        M5.Display.printf("Weight: %.3f kg\n", DATA.weight_kg);
    } else if (screen == 1) {
        M5.Display.printf("Pitch: %.1f\nRoll: %.1f\n", DATA.pitch, DATA.roll);
    } else if (screen == 2) {
        M5.Display.printf("BMS V: %.2f\nI: %.2f\nSOC: %.1f\n",
                          DATA.bms_voltage, DATA.bms_current, DATA.bms_soc);
    } else if (screen == 3) {
        if (!calib_in_progress) {
            M5.Display.println("Scale Calib");
            M5.Display.println("Put known weight");
            M5.Display.println("BtnB: start");
        } else {
            M5.Display.println("Calibrating...");
            M5.Display.printf("Known: %.2f kg\n", calib_known_weight);
            M5.Display.printf("Raw:   %.3f kg\n", DATA.weight_kg);
            M5.Display.println("BtnB: finish");
        }
    }
}

void taskDisplay(void *pv)
{
    Serial.println("[DISPLAY] Task started");
    int lastScreen = -1;
    for (;;) {
        M5.update();
        int s = command_get_display_screen();
        if (M5.BtnA.wasPressed()) {
            s = (s + 1) % 4;
        }
        if (s != lastScreen) {
            lastScreen = s;
            screen = s;
            draw();
        }
        if (screen == 3) {
            if (!calib_in_progress && M5.BtnB.wasPressed()) {
                calib_in_progress = true;
                scale_tare();
                draw();
            } else if (calib_in_progress && M5.BtnB.wasPressed()) {
                float raw = scale_get_raw();
                if (raw != 0) {
                    float factor = calib_known_weight / raw;
                    scale_set_cal_factor(factor);
                    CONFIG.scale_cal_factor = factor;
                    config_save();
                }
                calib_in_progress = false;
                draw();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
