#pragma once
#include <Arduino.h>

void setupWiFiApSta(const char *apSsid,
                    const char *apPass,
                    const char *staSsid,
                    const char *staPass);
