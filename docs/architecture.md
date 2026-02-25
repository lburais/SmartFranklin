# Architecture Overview

Core:
- main.cpp
- tasks/*
- data_model
- mqtt_layer
- web_dashboard
- config_store
- watchdog
- plugin_manager
- mqtt_bridge

Data flow:
Sensors -> DataModel -> MQTT -> Web -> Commands -> Tasks

Bridge:
Internal broker <-> MQTT bridge <-> External broker
