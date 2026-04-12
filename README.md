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

## Hosting on Render

This repo includes [render.yaml](render.yaml) for one-click service setup.

### Quick steps

1. Push this repo to GitHub.
2. In Render, create a **Blueprint** from that repo.
3. Set secret env vars in Render:
  - `APP_API_KEY` (used by mobile app requests)
  - `BOARD_API_KEY` (used by LoRa bridge/board requests)
4. Keep the attached persistent disk mounted at `/var/data`.
5. Deploy and use your Render URL as the app server endpoint.

Notes:
- Use the Render URL/domain, not a static IP.
- SQLite persistence is kept via `ROBOT_LORA_DATA_DIR=/var/data`.

## API

## API Authentication (optional, recommended for hosted use)

If either `APP_API_KEY` or `BOARD_API_KEY` is set, API key auth is enabled.

For key rotation / multi-key deployments, you can also set comma-separated lists:
- `APP_API_KEYS`
- `BOARD_API_KEYS`

- Send key as header `x-api-key: <key>`
- Or `Authorization: Bearer <key>`

Role behavior:
- app key (`APP_API_KEY`): operator/app actions (input area, path planning, mission actions, operator workflows/notes)
- board key (`BOARD_API_KEY`): telemetry writes (`POST /api/telemetry`)
- both keys can read shared state endpoints (status, state, supervision, mission snapshots, bridge sync)

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

### 0.4) Metrics
`GET /api/metrics`

App-role endpoint for operational counters (uptime, command/telemetry counts, auth/rate-limit denials, safety action count, bridge reliability state).

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

`cellSizeM` now supports down to `0.5` meters (default minimum), which is recommended when planning 0.5m coverage passes.

Coverage progress is updated from telemetry using a configurable mark radius (`COVERAGE_MARK_RADIUS_M`, default `0.5`).

P0 fail-safe policy (production default):
- If mission is `RUNNING` and telemetry becomes stale, the server enforces a safety action automatically.
- `TELEMETRY_FAILSAFE_ENABLED` (default `1`) enables this policy.
- `TELEMETRY_FAILSAFE_ACTION` (default `ESTOP`, allowed `ESTOP` or `PAUSE`) controls the action.
- `TELEMETRY_FAILSAFE_COOLDOWN_MS` (default `max(SUPERVISION_TELEMETRY_STALE_MS, 5000)`) prevents repeated rapid triggers.
- `SAFETY_MONITOR_INTERVAL_MS` (default `500`) controls how often fail-safe checks run.

Geofence fail-safe (P0):
- If mission is `RUNNING` and robot telemetry is outside the configured boundary grid, the server enforces a safety action.
- `GEOFENCE_FAILSAFE_ENABLED` (default `1`) enables this policy.
- `GEOFENCE_FAILSAFE_ACTION` (default `ESTOP`, allowed `ESTOP` or `PAUSE`) controls the action.
- `GEOFENCE_FAILSAFE_COOLDOWN_MS` (default `5000`) prevents repeated rapid triggers.

Bridge reliability and health/readiness (P0):
- LoRa bridge command sends now include transient retries (`LORA_TRANSIENT_RETRY_MAX`, default `2`; `LORA_TRANSIENT_RETRY_MS`, default `200`).
- Bridge status reports degraded mode after repeated failures (`BRIDGE_DEGRADED_FAILURE_THRESHOLD`, default `3`).
- `/api/health` now returns readiness checks (`db`, `bridge`, `telemetry`) and responds `503` when not ready.

P1/P2 hardening controls:
- Request throttling (`REQUEST_RATE_LIMIT_WINDOW_MS`, `COMMAND_RATE_LIMIT_PER_WINDOW`, `TELEMETRY_RATE_LIMIT_PER_WINDOW`).
- Event retention pruning (`EVENT_RETENTION_DAYS`, `EVENT_RETENTION_PRUNE_INTERVAL_MS`).
- Key rotation support (`APP_API_KEYS`, `BOARD_API_KEYS`) while keeping legacy single-key vars.

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

### 3.1) Bridge sync (single pull endpoint for board/app)
`GET /api/bridge/sync`

Returns mission + LoRa bridge + boundary/path + robot/fault state in one payload, suitable for independent polling loops from both app and board-side integrations.

### 4) Get coverage grid
`GET /api/coverage`

### 5) Plan path
`POST /api/path/plan`

Coverage plan mode (interior sweep; default when `goal` is omitted):

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

Response waypoints include `headingDeg` on each point (except final point), which can be used to draw direction arrows in the map UI.

`mode` must be either `coverage` or `goal`.

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

## Independent app + board integration model

- App loop:
  - Write: `/api/input-area`, `/api/path/plan`, mission/operator endpoints
  - Read: `/api/supervision/summary`, `/api/state`, `/api/bridge/sync`, websocket events
- Board/bridge loop:
  - Write: `/api/telemetry`
  - Read: `/api/bridge/sync` (or `/api/lora/status` + `/api/mission/current`)

Both clients can operate independently as long as they target the same hosted base URL and send the appropriate API key.

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

## Production runbook (P0-P2)

Use this as the minimum release gate before field deployment.

### 1) Environment and key setup

Server (`robot-lora-server`) required vars:
- `APP_API_KEY` or `APP_API_KEYS` (comma-separated list for rotation)
- `BOARD_API_KEY` or `BOARD_API_KEYS` (comma-separated list for rotation)

Server recommended safety/reliability vars:
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

Mobile app (`robot-app-main`) expected var:
- `EXPO_PUBLIC_APP_API_KEY=<active app key>`

### 2) Pre-release checks (must pass)

From `robot-lora-server`:
- `node --check src/server.js`
- `node --check src/lora_bridge.js`
- `node --check src/coverage.js`
- `npm run test:hil`

From `robot-app-main`:
- `npx expo export`

Acceptance criteria:
- `/api/health` returns `200` when idle/ready and includes checks (`db`, `bridge`, `telemetry`)
- `/api/metrics` reachable with app credentials
- No TypeScript or syntax errors in changed server/app files

### 3) Key rotation procedure (zero downtime)

1. Add new key to `APP_API_KEYS`/`BOARD_API_KEYS` while keeping old key present.
2. Deploy server.
3. Update app/base-station clients to send new keys.
4. Verify traffic and auth-denied counters remain stable.
5. Remove old keys from key lists and redeploy.

### 4) Hardware validation matrix (field gate)

Run these on real hardware and record pass/fail with timestamps:

- **Telemetry stale test:** stop telemetry stream during `RUNNING`; verify configured safety action triggers and mission leaves `RUNNING`.
- **Geofence breach test:** drive/send position outside boundary; verify configured safety action triggers.
- **LoRa link-loss test (STM32 AUTO):** simulate link timeout; verify STM32 safe-state action and server mission safety alignment.
- **Bridge degraded test:** induce repeated base-station TX failures; verify degraded signal in status/health and recovery after link restoration.
- **Operator recovery test:** after fail-safe trigger, validate `PAUSED/ABORTED` handling and `RESET`/resume flow per policy.

### 5) Backup and rollback

- Ensure persistent SQLite volume is mounted and writable.
- Take DB snapshot before production deploy.
- Keep previous server image/config ready for immediate rollback.
- On rollback, verify `/api/health`, `/api/supervision/summary`, and command path (`/command`) before resuming operations.

## Remaining work after P2 (sorted)

### A) Hardware-in-loop verification on real devices (not complete in this repo)

Status: **pending hardware runtime validation**

Required to close:
- Build and flash the ESP-IDF base-station firmware that includes `/health` + degraded counters.
- Induce transmit failures and verify degraded counters increment/reset as designed.
- Verify server `/api/health` and app supervision views reflect base-station degraded/recovery transitions.

Evidence to capture:
- Firmware serial logs (timestamped)
- Server `/api/metrics` snapshots before/after fault injection
- App screenshots of degraded/healthy transitions

### B) End-to-end policy alignment constants (partially complete)

Status: **documented for server/app; pending full cross-target sign-off**

Use this alignment matrix as the canonical policy source:

- **Telemetry staleness reaction**
  - Server env: `TELEMETRY_FAILSAFE_ENABLED`, `TELEMETRY_FAILSAFE_ACTION`, `SUPERVISION_TELEMETRY_STALE_MS`, `TELEMETRY_FAILSAFE_COOLDOWN_MS`
  - STM32 compile-time: link-loss watchdog timeout/cooldown macros in robot firmware
  - App env/UI: `EXPO_PUBLIC_APP_API_KEY`; supervision UI must show stale/degraded state from server payload

- **Boundary/Geofence reaction**
  - Server env: `GEOFENCE_FAILSAFE_ENABLED`, `GEOFENCE_FAILSAFE_ACTION`, `GEOFENCE_FAILSAFE_COOLDOWN_MS`
  - STM32: no independent geofence authority unless explicitly implemented (server remains source of truth)
  - App: operator UX must treat server fail-safe state as authoritative

- **Bridge reliability/degraded policy**
  - Server env: `LORA_TRANSIENT_RETRY_MAX`, `LORA_TRANSIENT_RETRY_MS`, `BRIDGE_DEGRADED_FAILURE_THRESHOLD`
  - ESP-IDF base station: TX success/fail streak + degraded counters in status/health handlers
  - App: render degraded signal and recovery clearly in controller screen

### C) P3 production operations

Status: **partially complete**

1. CI pipelines across all targets
  - Server CI (this repo): implemented via `.github/workflows/server-ci.yml` (syntax checks + `npm run test:hil`)
  - App CI (`robot-app-main`): type check + `npx expo export`
  - Firmware CI (STM32 + ESP-IDF repos): compile jobs and artifact retention

2. Release gating
  - Server release gate script implemented: `npm run release:gate`
  - Optional live checks: set `RELEASE_GATE_BASE_URL` and `RELEASE_GATE_APP_API_KEY` to enforce `/api/health` + `/api/metrics` checks.
  - Still required: enforce this gate together with app/firmware CI before release tag.

3. Key rotation automation
  - Validation script implemented: `npm run ops:key-rotation:check`
  - Inputs: `ROTATION_BASE_URL`, `NEW_APP_KEY`, optional `OLD_APP_KEY`.
  - Post-cutover validation: run with `-- --expect-old-revoked`.
  - Still required: pipeline wiring that updates secrets and records an audit trail.

4. Backup/restore drill
  - Drill script implemented: `npm run ops:sqlite:drill`
  - Optional overrides: `ROBOT_LORA_DATA_DIR`, `SQLITE_DB_FILE`, `SQLITE_BACKUP_DIR`.
  - Still required: scheduled staging restore cadence and RTO/RPO sign-off.

5. On-device soak tests
  - 4-8 hour stability run with periodic telemetry dropouts and link disturbance scenarios.

### D) Suggested execution order

1. Close A (hardware runtime verification).
2. Complete B sign-off (constants freeze across server/STM32/app).
3. Implement app + firmware CI lanes and make `release:gate` mandatory in promotion workflow.
4. Schedule recurring key-rotation and SQLite drill jobs; complete C5 soak validation.

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
- **SB ESP32 dispersion commands** (sent from STM32 over UART4 @ 115200 baud): `SALT:<pct>,BRINE:<pct>`, `PCT:<pct>` (applies same % to both). Responses: `STATUS:OK`, `STATUS:ERROR,BAD_CMD`, `STATUS:ERROR,OVERFLOW`, `FLOW:SALT:<ml_min>,BRINE:<ml_min>,RPM:<rpm>`.
