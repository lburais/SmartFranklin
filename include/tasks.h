#pragma once
#include <Arduino.h>

extern TaskHandle_t taskMqttBrokerHandle;
extern TaskHandle_t taskDistanceHandle;
extern TaskHandle_t taskWeightHandle;
extern TaskHandle_t taskTiltHandle;
extern TaskHandle_t taskRtcHandle;
extern TaskHandle_t taskBmsBleHandle;
extern TaskHandle_t taskDisplayHandle;
extern TaskHandle_t taskMeshtasticBridgeHandle;
extern TaskHandle_t taskNbiotHandle;

void taskMqttBroker(void *pvParameters);
void taskDistance(void *pvParameters);
void taskWeight(void *pvParameters);
void taskTilt(void *pvParameters);
void taskRtc(void *pvParameters);
void taskBmsBle(void *pvParameters);
void taskDisplay(void *pvParameters);
void taskMeshtasticBridge(void *pvParameters);
void taskNbiot(void *pvParameters);
void taskWatchdog(void *pvParameters);
