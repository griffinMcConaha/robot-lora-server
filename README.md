# Robot LoRa Server (Starter)

This is a starter backend for:
- input area setup (base station + boundary)
- live telemetry ingestion over LoRa bridge
- live coverage map updates
- path planning for app updates

## Run

```bash
npm install
npm run start
```

Server default: `http://localhost:8080`

Open `http://localhost:8080` to confirm the server is running.

## API

### 0) Server root/status
`GET /`

Returns a basic status payload and a list of available endpoints.

### 0.1) App-compatible status
`GET /status`

Compatibility endpoint for the mobile app / base-station style polling.

### 0.2) App-compatible command
`POST /command`

Accepts either JSON or text body:

```json
{ "cmd": "forward" }
```

Or plain text body such as:

```text
forward
```

Also handles the STM32 LoRa protocol `CMD:` prefix (e.g. `CMD:AUTO,SALT:25,BRINE:75` is normalized to `AUTO`).

Supported commands: `FORWARD`, `BACKWARD`, `LEFT`, `RIGHT`, `STOP`, `PAUSE`, `AUTO`, `MANUAL`, `ESTOP`, `RESET` (clears ESTOP/ERROR latch).

Responds with plain text `OK`.

Phase E arbitration rules now apply:
- `AUTO` is rejected unless waypoints are already committed to the robot.
- `WPCLEAR`, `WP:...`, and `WPLOAD:...` are accepted only while the mission is `CONFIGURING` or `PAUSED`.
- `RESET` is accepted only while the mission is `PAUSED`, `ABORTED`, or `ERROR`.
- Server mission state is synchronized with direct operator commands: `AUTO` starts/resumes, `PAUSE` pauses, `ESTOP` aborts, and manual/drive commands pause a running mission.

### 0.3) Supervision summary
`GET /api/supervision/summary`

Returns a single app-facing supervision payload with:
- mission snapshot
- LoRa bridge status
- latest robot position and telemetry freshness
- coverage stats
- actionable alerts
- allowed operator actions
- operator workflows and notes

### 1) Initialize area
`POST /api/input-area`

```json
{
  "baseStation": { "lat": 41.0762, "lon": -81.5139 },
  "boundary": [
    { "lat": 41.0762, "lon": -81.5142 },
    { "lat": 41.0762, "lon": -81.5136 },
    { "lat": 41.0758, "lon": -81.5136 },
    { "lat": 41.0758, "lon": -81.5142 }
  ],
  "cellSizeM": 2.0
}
```

### 2) Push telemetry
`POST /api/telemetry`

Supports both existing server payloads and LoRa firmware-style telemetry.

Existing payload:

```json
{
  "robot": { "lat": 41.0761, "lon": -81.51395, "heading": 45, "speed": 0.3 },
  "source": "lora"
}
```

LoRa firmware-style payload (from STM32/gateway chain):

```json
{
  "state": "AUTO",
  "gps": { "lat": 41.0761, "lon": -81.51395, "fix": 1, "sat": 8, "hdop": 0.9 },
  "motor": { "m1": 35, "m2": 34 },
  "heading": { "yaw": 42.1, "pitch": 0.5 },
  "disp": { "salt": 20, "brine": 80 },
  "temp": 2.3,
  "prox": { "left": 90, "right": 102 }
}
```

Fault notification (forwarded from STM32 when a fault triggers an action):

```json
{ "fault": "IMU_TIMEOUT", "action": "ESTOP" }
```

Fault codes: `NONE`, `IMU_TIMEOUT`, `GPS_LOSS`, `PROXIMITY_WARN`, `PROXIMITY_CRIT`, `BATTERY_COLD`, `DISPERSION_CLOG`, `GENERIC`.  
Actions: `PAUSE`, `ESTOP`, `LOG_ONLY`.

A fault payload does **not** overwrite robot position — it publishes a `fault.received` WebSocket event instead.

Base-station passthrough (when the base station ’s `/status` JSON is forwarded here with a `last_lora` field, the inner telemetry string is auto-unwrapped):

```json
{ "battery": 85, "state": "IDLE", "mode": "LORA", "last_lora": "{\"state\":\"AUTO\",...}" }
```

### 3) Get current state
`GET /api/state`

### 4) Get coverage grid
`GET /api/coverage`

### 5) Plan path
`POST /api/path/plan`

```json
{
  "goal": { "lat": 41.0759, "lon": -81.5137 }
}
```

### 6) Mission lifecycle

`POST /api/mission/start`

- Requires mission state `CONFIGURING`
- Requires committed waypoints; if a cached path exists, the server will push it first
- Sends `AUTO` to the robot and only then transitions the mission to `RUNNING`

`POST /api/mission/pause`

- Requires mission state `RUNNING`
- Sends `PAUSE` to the robot and then transitions to `PAUSED`

`POST /api/mission/resume`

- Requires mission state `PAUSED`
- Requires committed waypoints
- Sends `AUTO` to the robot and then transitions to `RUNNING`

`POST /api/mission/abort`

- Requires mission state `RUNNING` or `PAUSED`
- Sends `ESTOP` to the robot and then transitions to `ABORTED`

`POST /api/mission/complete`

- Requires mission state `RUNNING`
- Sends `PAUSE` to the robot and then transitions to `COMPLETED`

### 7) Operator workflows

`GET /api/operator/workflows`

Returns operator workflow state for:
- `preflight`
- `recovery`
- `shutdown`

Each workflow step is either:
- `derived` from server state, or
- `manual`, which an operator can acknowledge

`POST /api/operator/workflows/:workflowId/steps/:stepId`

Example body:

```json
{
  "checked": true,
  "actor": "field-op",
  "note": "Radio check completed"
}
```

`GET /api/operator/notes`

Returns the current in-memory operator note list.

`POST /api/operator/notes`

Example body:

```json
{
  "text": "Obstacle removed from lane 2",
  "category": "recovery",
  "actor": "field-op"
}
```

Notes are also appended to the active mission record as free-form mission notes.

## Live updates for app

A websocket is exposed at the same host/port. Messages are JSON packets:
- `state.snapshot`
- `area.updated`
- `telemetry.updated`
- `path.updated`
- `command.received`
- `fault.received` — payload: `{ fault: string, action: string, at: number }`
- `mission.updated`
- `supervision.updated`
- `operator.updated`

## HIL tests

Run the hardware-in-the-loop style server validation suite with:

```bash
npm run test:hil
```

The test harness runs the server against a local mock base station and validates fail-safe scenarios such as:
- stale telemetry while mission is `RUNNING`
- operator manual override pausing a mission
- ESTOP fault notifications aborting a mission
- operator workflow acknowledgements and note capture

## Node-RED starter flow

Import [flows/node-red-lora-starter.json](flows/node-red-lora-starter.json).
It includes a sample inject -> POST `/api/telemetry` flow.

For production, replace the `inject` node with your LoRa source node (serial/MQTT) and map incoming payload to this shape:

```json
{
  "robot": { "lat": number, "lon": number, "heading": number, "speed": number },
  "source": "lora"
}
```

## Notes

- This is an MVP starter for your old app baseline (controller + area map).
- Path planning currently runs grid-based A* inside the drawn boundary.
- Coverage marks cells visited by robot position updates.
- **Base station Wi-Fi**: the base station (`jakep2377/base_station`) first tries STA mode (SSID `42Fortress`) and falls back to AP mode (SSID `SaltRobot_Base`, IP `192.168.4.1`). In STA mode the IP is DHCP-assigned. The mobile app hard-codes `192.168.4.1`, so use AP mode for app connectivity.
- **LoRa telemetry interval**: the STM32 transmits telemetry every **10 seconds**.
- **SB ESP32 dispersion commands** (sent from STM32 over UART4 @ 9600 baud): `SALT:<pct>,BRINE:<pct>`, `PCT:<pct>` (applies same % to both). Responses: `STATUS:OK`, `STATUS:ERROR,BAD_CMD`, `STATUS:ERROR,OVERFLOW`, `FLOW:SALT:<ml_min>,BRINE:<ml_min>,RPM:<rpm>`.
