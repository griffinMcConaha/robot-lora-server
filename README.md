# Robot LoRa Server

Backend server for the robot stack. Responsibilities include:
- Input area setup (base station + boundary)
- Telemetry ingestion (via LoRa bridge / base station)
- Coverage grid updates
- Path planning
- Mission supervision + operator workflows
- Bridge health/status APIs
- WebSocket live updates

## Run locally

```bash
npm install
npm run start
```

Default: `http://localhost:8080`  
Open `http://localhost:8080` to confirm the server is running.

## Deploy (Render)

This repo includes [`render.yaml`](render.yaml) for one-click setup.

### Quick steps
1. Push this repo to GitHub.
2. In Render, create a **Blueprint** from the repo.
3. Set secret env vars:
   - `APP_API_KEY` (mobile app/operator requests)
   - `BOARD_API_KEY` (base station / bridge requests)
4. Keep the persistent disk mounted at `/var/data`.
5. Deploy and use your Render URL as the app server endpoint.

Notes:
- Use the Render URL/domain, not a static IP.
- SQLite persistence uses `ROBOT_LORA_DATA_DIR=/var/data`.

---

## API Authentication (optional; recommended for hosted use)

If either `APP_API_KEY` or `BOARD_API_KEY` is set, API key auth is enabled.

For key rotation / multi-key deployments, you can also set comma-separated lists:
- `APP_API_KEYS`
- `BOARD_API_KEYS`

Send the key via either:
- `x-api-key: <key>`
- `Authorization: Bearer <key>`

Role behavior:
- **App key** (`APP_API_KEY` / `APP_API_KEYS`): operator/app actions (area setup, path planning, mission/operator workflows)
- **Board key** (`BOARD_API_KEY` / `BOARD_API_KEYS`): telemetry writes (`POST /api/telemetry`)
- **Both keys** may read shared state endpoints (status, mission snapshots, supervision, bridge sync)

---

## API overview

### Health / status

- `GET /`  
  Basic status payload + endpoint list.

- `GET /status`  
  Compatibility endpoint for mobile app / base-station style polling.

- `GET /api/health`  
  Readiness checks (`db`, `bridge`, `telemetry`). Returns `503` when not ready.

- `GET /api/metrics`  
  App-role endpoint for operational counters (uptime, command/telemetry counts, auth/rate-limit denials, safety action count, bridge reliability state).

### Commands (app-compatible)

- `POST /command`  
  Accepts JSON or text:

```json
{ "cmd": "forward" }
```

```text
forward
```

Also handles the STM32 LoRa protocol `CMD:` prefix (e.g. `CMD:AUTO,SALT:25,BRINE:75` is normalized to `AUTO`).

Supported commands:
`FORWARD`, `BACKWARD`, `LEFT`, `RIGHT`, `STOP`, `PAUSE`, `AUTO`, `MANUAL`, `ESTOP`, `RESET`

Responds with plain text: `OK`.

**Phase E arbitration rules:**
- `AUTO` is rejected unless waypoints are already committed to the robot.
- `WPCLEAR`, `WP:...`, `WPLOAD:...` are accepted only while the mission is `CONFIGURING` or `PAUSED`.
- `RESET` is accepted only while the mission is `PAUSED`, `ABORTED`, or `ERROR`.
- Mission state is synchronized with direct operator commands:
  - `AUTO` starts/resumes
  - `PAUSE` pauses
  - `ESTOP` aborts
  - manual/drive commands pause a running mission

### Supervision

- `GET /api/supervision/summary`  
  Single app-facing payload with:
  - mission snapshot
  - LoRa bridge status
  - latest robot position + telemetry freshness
  - coverage stats
  - actionable alerts
  - allowed operator actions
  - operator workflows + notes

### Input area / boundary

- `POST /api/input-area`

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

Notes:
- `cellSizeM` supports down to `0.5` meters (recommended for 0.5m coverage passes).
- Coverage progress is updated from telemetry using `COVERAGE_MARK_RADIUS_M` (default `0.5`).

### Telemetry ingestion

- `POST /api/telemetry`

Supports both existing server payloads and LoRa firmware-style telemetry.

Existing payload:

```json
{
  "robot": { "lat": 41.0761, "lon": -81.51395, "heading": 45, "speed": 0.3 },
  "source": "lora"
}
```

LoRa firmware-style payload:

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

Fault codes:
`NONE`, `IMU_TIMEOUT`, `GPS_LOSS`, `PROXIMITY_WARN`, `PROXIMITY_CRIT`, `BATTERY_COLD`, `DISPERSION_CLOG`, `GENERIC`

Actions:
`PAUSE`, `ESTOP`, `LOG_ONLY`

A fault payload does **not** overwrite robot position; it publishes a `fault.received` WebSocket event instead.

Base-station passthrough:
If base station `/status` JSON is forwarded here with a `last_lora` field, the inner telemetry string is auto-unwrapped.

```json
{ "battery": 85, "state": "IDLE", "mode": "LORA", "last_lora": "{\"state\":\"AUTO\",...}" }
```

### State / coverage / planning

- `GET /api/state`
- `GET /api/bridge/sync`  
  Single pull endpoint returning mission + bridge + boundary/path + robot/fault state (suitable for both app and board polling loops).
- `GET /api/coverage`
- `POST /api/path/plan`

Coverage plan mode (default when `goal` is omitted):

```json
{
  "mode": "coverage",
  "coverageWidthM": 0.5
}
```

Goal-to-goal mode (A* route):

```json
{
  "mode": "goal",
  "goal": { "lat": 41.0759, "lon": -81.5137 }
}
```

Response waypoints include `headingDeg` on each point (except final point) for direction arrows in UI.

### Mission lifecycle

- `POST /api/mission/start`
  - Requires mission state `CONFIGURING`
  - Requires committed waypoints (server may push cached path first)
  - Sends `AUTO` then transitions to `RUNNING`

- `POST /api/mission/pause`
  - Requires mission state `RUNNING`
  - Sends `PAUSE` then transitions to `PAUSED`

- `POST /api/mission/resume`
  - Requires mission state `PAUSED`
  - Requires committed waypoints
  - Sends `AUTO` then transitions to `RUNNING`

- `POST /api/mission/abort`
  - Requires mission state `RUNNING` or `PAUSED`
  - Sends `ESTOP` then transitions to `ABORTED`

- `POST /api/mission/complete`
  - Requires mission state `RUNNING`
  - Sends `PAUSE` then transitions to `COMPLETED`

### Operator workflows + notes

- `GET /api/operator/workflows`
  - Workflows: `preflight`, `recovery`, `shutdown`
  - Steps are either `derived` (from server state) or `manual` (operator acknowledges)

- `POST /api/operator/workflows/:workflowId/steps/:stepId`

```json
{
  "checked": true,
  "actor": "field-op",
  "note": "Radio check completed"
}
```

- `GET /api/operator/notes`
- `POST /api/operator/notes`

```json
{
  "text": "Obstacle removed from lane 2",
  "category": "recovery",
  "actor": "field-op"
}
```

Notes are appended to the active mission record as mission notes.

---

## WebSocket events

WebSocket is exposed at the same host/port.

Events:
- `state.snapshot`
- `area.updated`
- `telemetry.updated`
- `path.updated`
- `command.received`
- `fault.received` — `{ fault: string, action: string, at: number }`
- `mission.updated`
- `supervision.updated`
- `operator.updated`

---

## Integration model (app + board)

- App loop:
  - Write: `/api/input-area`, `/api/path/plan`, mission/operator endpoints
  - Read: `/api/supervision/summary`, `/api/state`, `/api/bridge/sync`, WebSocket events

- Board/bridge loop:
  - Write: `/api/telemetry`
  - Read: `/api/bridge/sync` (or `/api/lora/status` + `/api/mission/current`)

Both clients can operate independently if they target the same base URL and send the appropriate API key.

---

## HIL tests

Run the hardware-in-the-loop validation suite:

```bash
npm run test:hil
```

The harness runs the server against a local mock base station and validates fail-safe scenarios such as:
- stale telemetry while mission is `RUNNING`
- operator manual override pausing a mission
- ESTOP fault notifications aborting a mission
- operator workflow acknowledgements + note capture

---

## Production runbook (P0–P2)

### Environment + key setup

Required:
- `APP_API_KEY` or `APP_API_KEYS`
- `BOARD_API_KEY` or `BOARD_API_KEYS`

Recommended safety/reliability:
- `TELEMETRY_FAILSAFE_ENABLED=1`
- `TELEMETRY_FAILSAFE_ACTION=ESTOP`
- `GEOFENCE_FAILSAFE_ENABLED=1`
- `GEOFENCE_FAILSAFE_ACTION=ESTOP`
- `LORA_TRANSIENT_RETRY_MAX=2`
- `BRIDGE_DEGRADED_FAILURE_THRESHOLD=3`
- `REQUEST_RATE_LIMIT_WINDOW_MS=60000`
- `COMMAND_RATE_LIMIT_PER_WINDOW=120`
- `TELEMETRY_RATE_LIMIT_PER_WINDOW=600`
- `EVENT_RETENTION_DAYS=14`
- `EVENT_RETENTION_PRUNE_INTERVAL_MS=3600000`

Mobile app (`robot-app-main`):
- `EXPO_PUBLIC_APP_API_KEY=<active app key>`

### Pre-release checks

From `robot-lora-server`:
- `node --check src/server.js`
- `node --check src/lora_bridge.js`
- `node --check src/coverage.js`
- `npm run test:hil`

From `robot-app-main`:
- `npx expo export`

Acceptance criteria:
- `/api/health` returns `200` when idle/ready and includes `db`, `bridge`, `telemetry` checks
- `/api/metrics` reachable with app credentials
- no syntax errors in changed files
- HIL tests pass

### Key rotation procedure (zero downtime)

1. Add new key to `APP_API_KEYS` / `BOARD_API_KEYS` while keeping old key.
2. Deploy server.
3. Update app/base-station clients to send new keys.
4. Verify traffic and auth-denied counters remain stable.
5. Remove old keys and redeploy.

### Hardware validation matrix (field gate)

Record pass/fail with timestamps:
- Telemetry stale test (during `RUNNING`)
- Geofence breach test
- LoRa link-loss test (STM32 AUTO)
- Bridge degraded test (repeat base-station TX failures)
- Operator recovery test (post fail-safe; `RESET`/resume flow)

### Backup and rollback

- Ensure persistent SQLite volume is mounted and writable.
- Snapshot DB before production deploy.
- Keep previous server image/config ready for rollback.
- On rollback, verify `/api/health`, `/api/supervision/summary`, and `/command` path.

---

## Node-RED starter flow

Import [`flows/node-red-lora-starter.json`](flows/node-red-lora-starter.json).

For production, replace the `inject` node with your LoRa source node (serial/MQTT) and map incoming payload to:

```json
{
  "robot": { "lat": number, "lon": number, "heading": number, "speed": number },
  "source": "lora"
}
```

---

## Notes

- MVP starter for older app baseline (controller + area map).
- Path planning runs grid-based A* inside the configured boundary.
- Coverage marks visited cells from robot position updates.
- Base station Wi-Fi: base station firmware first tries STA (SSID `42Fortress`) then falls back to AP (SSID `SaltRobot_Base`, typically `192.168.4.1`).
- LoRa telemetry interval: STM32 transmits telemetry every ~10 seconds.
