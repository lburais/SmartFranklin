# MQTT API

## Commands

Topic: smartfranklin/cmd/display
Payload: {"screen":2}

Topic: smartfranklin/cmd/mesh
Payload: {"text":"hello"}

Topic: smartfranklin/cmd/scale
Payload: {"action":"tare"}

Topic: smartfranklin/cmd/scale
Payload: {"action":"calibrate","value":1.234}

## Telemetry

- smartfranklin/distance/cm
- smartfranklin/weight/kg
- smartfranklin/tilt/pitch
- smartfranklin/tilt/roll
- smartfranklin/time
- smartfranklin/bms/voltage
- smartfranklin/bms/current
- smartfranklin/bms/soc
- smartfranklin/mesh/in
