# mcp-drone-example
## Install lib
```
python>=3.10
uv
mcp
```

## Create project and install lib
### Client
```
uv init client
uv venv
.venv\Scripts\activate

uv add mcp python-dotenv google-genai
```

### Server
```
uv init server
uv venv
.venv\Scripts\activate

uv add mcp python-dotenv google-genai pymavlink
```

## Run
### Server
```
uv run server.py
```
### Client
```
uv run client.py http://localhost:8081/sse
```

## Log Example
```
Initialized SSE Client Session.
Listing tools...
Connected to server with tools: ['show', 'get_weather', 'add_number', 'add_connection', 'remove_connection', 'arm', 'disarm', 'takeoff', 'landing', 'get_drone_status', 'change_flight_mode', 'goto_location']
MCP Client started! Type 'quit' to exit.

Query: get drone infor
[Gemini requested tool call: get_drone_status with args: {}]
Tool raw result: meta=None content=[TextContent(type='text', text='{\n  "status": "No active connection"\n}', annotations=None, meta=None)] structuredContent={'result': '{\n  "status": "No active connection"\n}'} isError=False
Gemini: OK. The drone status is: "No active connection".

Query: add connection with ip 192.168.1.91, port 14550 
[Gemini requested tool call: add_connection with args: {'port': 14550, 'connection_ip': '192.168.1.91'}]
Tool raw result: meta=None content=[TextContent(type='text', text='Connecting to udp:192.168.1.91:14550', annotations=None, meta=None)] structuredContent={'result': 'Connecting to udp:192.168.1.91:14550'} isError=False
Gemini: OK.

Query: get drone infor
[Gemini requested tool call: get_drone_status with args: {}]
Tool raw result: meta=None content=[TextContent(type='text', text='{\n  "status": "Connected",\n  "imu_roll_deg": -0.014809072710236069,\n  "imu_pitch_deg": 0.04246878074785927,\n  "imu_yaw_deg": 140.8556675046444,\n  "angular_velocity": {\n    "rollspeed_rad_s": 0.0013123913668096066,\n    "pitchspeed_rad_s": 0.000613159965723753,\n    "yawspeed_rad_s": -0.00036382442340254784\n  },\n  "latitude": -35.3642619,\n  "longitude": 149.1655429,\n  "altitude_m": 20.001,\n  "battery_voltage_V": 12.6,\n  "battery_remaining_%": 0,\n  "acceleration_raw": {\n    "x": 0.0,\n    "y": 0.0,\n    "z": -1.001\n  },\n  "gyro_raw": {\n    "x": 0.005,\n    "y": 0.004,\n    "z": 0.003\n  },\n  "mag_raw": {\n    "x": -146,\n    "y": -187,\n    "z": -529\n  },\n  "mode": "GUIDED",\n  "system_status": 4,\n  "type": 2,\n  "system_id": 1,\n  "component_id": 0\n}', annotations=None, meta=None)] structuredContent={'result': '{\n  "status": "Connected",\n  "imu_roll_deg": -0.014809072710236069,\n  "imu_pitch_deg": 0.04246878074785927,\n  "imu_yaw_deg": 140.8556675046444,\n  "angular_velocity": {\n    "rollspeed_rad_s": 0.0013123913668096066,\n    "pitchspeed_rad_s": 0.000613159965723753,\n    "yawspeed_rad_s": -0.00036382442340254784\n  },\n  "latitude": -35.3642619,\n  "longitude": 149.1655429,\n  "altitude_m": 20.001,\n  "battery_voltage_V": 12.6,\n  "battery_remaining_%": 0,\n  "acceleration_raw": {\n    "x": 0.0,\n    "y": 0.0,\n    "z": -1.001\n  },\n  "gyro_raw": {\n    "x": 0.005,\n    "y": 0.004,\n    "z": 0.003\n  },\n  "mag_raw": {\n    "x": -146,\n    "y": -187,\n    "z": -529\n  },\n  "mode": "GUIDED",\n  "system_status": 4,\n  "type": 2,\n  "system_id": 1,\n  "component_id": 0\n}'} isError=False
Gemini: OK. I have retrieved the drone information. The drone status is Connected. The latitude is -35.3642619, longitude is 149.1655429, and altitude is 20.001 meters. The battery voltage is 12.6V. The flight mode is GUIDED.

Query: go to position with lat: -35.360000 lon: 149.1640000, alt:10
[Gemini requested tool call: goto_location with args: {'longitude': 149.164, 'altitude': 10, 'latitude': -35.36}]
Tool raw result: meta=None content=[TextContent(type='text', text='Drone navigating to (-35.360000, 149.164000, 10.0m)', annotations=None, meta=None)] structuredContent={'result': 'Drone navigating to (-35.360000, 149.164000, 10.0m)'} isError=False
Gemini: OK. I've sent the drone to latitude -35.36, longitude 149.164, and altitude 10.

```

