# SmartFranklin

[![PlatformIO](https://img.shields.io/badge/PlatformIO-ESP32-F5822A?logo=platformio&logoColor=white)](https://platformio.org/)
[![Framework](https://img.shields.io/badge/framework-Arduino-00979D?logo=arduino&logoColor=white)](https://www.arduino.cc/)
[![Board](https://img.shields.io/badge/board-M5StickC%20Plus2-1f6feb)](https://docs.m5stack.com/en/core/m5stickc_plus2)
[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

SmartFranklin is an ESP32 IoT controller project for **M5Stick C Plus2**, focused on sensing, connectivity, and resilient remote operation.

---

## Key Features

- Wi-Fi dual mode (**AP + STA**)
- MQTT client layer + MQTT bridge
- Web dashboard (status/config)
- Sensor support (weight, distance, tilt/IMU, RTC)
- PA Hub channel mapping (I2C multiplexer)
- Optional **NB-IoT** (cellular MQTT + GNSS)
- Optional **Meshtastic** bridge
- Watchdog-based task health monitoring

---

## Architecture

```mermaid
flowchart LR
    SENSORS[Weight / Distance / IMU / RTC] --> PAHUB[PA Hub Channels]
    PAHUB --> CORE[SmartFranklin Core]

    CORE --> WIFI[Wi-Fi AP+STA]
    CORE --> WEB[Web Dashboard]
    CORE --> WDG[Watchdog]

    CORE --> MQTTL[MQTT Layer]
    MQTTL --> MQTTB[MQTT Bridge]
    MQTTB --> BROKER[External MQTT Broker]

    CORE --> NBIOT[NB-IoT2 Module]
    NBIOT --> CELL[Cellular Network]

    CORE --> MESH[Meshtastic Bridge]
    MESH --> LORA[LoRa Mesh Network]
```

---

## Project Layout

- `include/` — module headers
- `src/` — module implementations
- `boards/` — custom board definitions
- `platformio.ini` — build/upload/deps configuration

---

## Build & Flash (macOS)

```bash
cd /Volumes/Ra/Development/SmartFranklin
pio run -e m5stick-c-plus2
pio run -e m5stick-c-plus2 -t upload
pio device monitor -b 115200
```

---

## Documentation (Doxygen)

```bash
cd /Volumes/Ra/Development/SmartFranklin
brew install doxygen graphviz
doxygen -g Doxyfile   # first time only
doxygen Doxyfile
open docs/html/index.html
```

Use:
- `INPUT = include src`
- `RECURSIVE = YES`

---

## Notes

- If using custom board config, confirm `platformio.ini` board selection matches `boards/m5stick-c-plus2.json`.
- For remote deployments, validate fallback paths (AP mode / NB-IoT / Meshtastic) before field use.

---

## License

MIT License  
Copyright (c) 2026 Laurent Burais