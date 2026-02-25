#pragma once
#include <Arduino.h>

class Plugin {
public:
    virtual const char* name() = 0;
    virtual void setup() = 0;
    virtual void loop() = 0;
};
