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

```json
{
  "robot": { "lat": 41.0761, "lon": -81.51395, "heading": 45, "speed": 0.3 },
  "source": "lora"
}
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

## Live updates for app

A websocket is exposed at the same host/port. Messages are JSON packets:
- `state.snapshot`
- `area.updated`
- `telemetry.updated`
- `path.updated`

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
