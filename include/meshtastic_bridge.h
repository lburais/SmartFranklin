#pragma once
#include <Arduino.h>

void meshtastic_bridge_init();
void meshtastic_send_text(const String &msg);
bool meshtastic_poll_received(String &out);
