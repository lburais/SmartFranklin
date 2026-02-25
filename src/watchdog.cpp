#include "watchdog.h"
#include <esp_task_wdt.h>

static const int NUM_TASKS = 9;
static uint32_t lastBeat[NUM_TASKS];

void watchdog_init()
{
    esp_task_wdt_init(10, true);
    for (int i = 0; i < NUM_TASKS; ++i) lastBeat[i] = millis();
}

void watchdog_beat(int id)
{
    if (id < 0 || id >= NUM_TASKS) return;
    lastBeat[id] = millis();
}
