#pragma once

#include <stdint.h>

#define EXTERNAL_NUM_INTERRUPTS 16
#define NUM_DIGITAL_PINS        34
#define NUM_ANALOG_INPUTS       18

// I2C (PaHub, sensors)
#define PIN_WIRE_SDA 32
#define PIN_WIRE_SCL 33

// UART1 (NB-IoT2)
#define PIN_SERIAL1_TX 13
#define PIN_SERIAL1_RX 14

// UART2 (C6L Meshtastic)
#define PIN_SERIAL2_TX 0
#define PIN_SERIAL2_RX 26

// Display (handled by M5Unified)
#define TFT_MISO -1
#define TFT_MOSI -1
#define TFT_SCLK -1
#define TFT_CS   -1
#define TFT_DC   -1
#define TFT_RST  -1

// Buttons
#define BUTTON_A_PIN 37
#define BUTTON_B_PIN 39
