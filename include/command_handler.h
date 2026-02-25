#pragma once
#include <Arduino.h>

void command_handler_init();
void command_handle(const String &topic, const String &payload);
int  command_get_display_screen();
