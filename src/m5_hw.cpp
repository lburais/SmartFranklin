#include "m5_hw.h"

M5Hardware HW;

void M5Hardware::init()
{
    M5.begin();
    M5.Display.setBrightness(128);
}

HwStatus M5Hardware::read()
{
    M5.update();

    status.battery_voltage = M5.Power.getBatteryVoltage() / 1000.0f;
    status.battery_percent = M5.Power.getBatteryLevel();
    status.charging        = M5.Power.isCharging();

    status.button_a = M5.BtnA.isPressed();
    status.button_b = M5.BtnB.isPressed();

    M5.Imu.getAccel(&status.accel_x, &status.accel_y, &status.accel_z);

    return status;
}

void M5Hardware::setBrightness(uint8_t level)
{
    M5.Display.setBrightness(level);
}

void M5Hardware::deepSleep()
{
    M5.Power.deepSleep();
}
