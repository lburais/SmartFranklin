#include <Arduino.h>
#include "tasks.h"
#include "watchdog.h"
#include <esp_task_wdt.h>

void taskWatchdog(void *pv)
{
    Serial.println("[WATCHDOG] Task started");
    watchdog_init();
    esp_task_wdt_add(NULL);
    for (;;) {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
