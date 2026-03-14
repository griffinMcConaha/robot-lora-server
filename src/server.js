const express = require('express');
const cors = require('cors');
const http = require('http');
const { WebSocketServer } = require('ws');
const {
  buildCoverageMap,
  markCoverage,
  coverageStats,
} = require('./coverage');
const { findPath } = require('./pathing');

const app = express();
app.use(cors());
app.use(express.json({ limit: '1mb' }));
app.use(express.text({ type: 'text/*', limit: '256kb' }));

const server = http.createServer(app);
const wss = new WebSocketServer({ server });

const state = {
  baseStation: null,
  boundary: null,
  coverage: null,
  robot: null,
  trail: [],
  lastPath: [],
  lastCommand: null,
  lastCommandAt: 0,
  lastFault: null,
};

function parseMaybeJson(value) {
  if (typeof value !== 'string') return value;
  const trimmed = value.trim();
  if (!trimmed) return value;
  if (!trimmed.startsWith('{') && !trimmed.startsWith('[')) return value;
  try {
    return JSON.parse(trimmed);
  } catch {
    return value;
  }
}

function normalizeCommand(raw) {
  if (typeof raw !== 'string') return null;
  // Strip optional CMD: prefix (STM32 LoRa protocol uses CMD:AUTO, CMD:MANUAL, etc.)
  let token = raw.trim().toUpperCase();
  if (token.startsWith('CMD:')) token = token.slice(4).trim();
  // Strip SALT/BRINE params (e.g. AUTO,SALT:25,BRINE:75 -> AUTO)
  const commaIdx = token.indexOf(',');
  if (commaIdx !== -1) token = token.slice(0, commaIdx).trim();
  if (!token) return null;

  const aliases = {
    FWD: 'FORWARD',
    FORWARD: 'FORWARD',
    BACK: 'BACKWARD',
    BACKWARD: 'BACKWARD',
    LEFT: 'LEFT',
    RIGHT: 'RIGHT',
    STOP: 'STOP',
    PAUSE: 'PAUSE',
    AUTO: 'AUTO',
    MANUAL: 'MANUAL',
    ESTOP: 'ESTOP',
    RESET: 'RESET',   // clears ESTOP/ERROR latch → PAUSE
  };

  return aliases[token] ?? token;
}

function parseIncomingTelemetry(payload) {
  let body = parseMaybeJson(payload) ?? {};

  // Base station wraps last LoRa RX into a string field: {"last_lora":"<json>"}
  // Unwrap and re-parse so we treat it like direct telemetry.
  if (typeof body?.last_lora === 'string' && body.last_lora.trim().startsWith('{')) {
    const inner = parseMaybeJson(body.last_lora);
    if (inner && typeof inner === 'object') body = inner;
  }

  // Fault notification: {"fault":"IMU_TIMEOUT","action":"ESTOP"}
  if (typeof body?.fault === 'string') {
    return {
      robot: null,
      isFault: true,
      fault: String(body.fault),
      action: String(body.action ?? 'UNKNOWN'),
      source: 'lora-fault',
      raw: body,
    };
  }

  if (body?.robot && typeof body.robot.lat === 'number' && typeof body.robot.lon === 'number') {
    return {
      robot: {
        lat: body.robot.lat,
        lon: body.robot.lon,
        heading: Number(body.robot.heading ?? body.heading?.yaw ?? 0),
        speed: Number(body.robot.speed ?? 0),
      },
      source: String(body.source ?? 'unknown'),
      stateName: typeof body.state === 'string' ? body.state : null,
      raw: body,
    };
  }

  if (body?.gps && typeof body.gps.lat === 'number' && typeof body.gps.lon === 'number') {
    const motorM1 = Number(body.motor?.m1 ?? 0);
    const motorM2 = Number(body.motor?.m2 ?? 0);
    const approxSpeed = Math.abs((motorM1 + motorM2) / 2) / 100;

    return {
      robot: {
        lat: body.gps.lat,
        lon: body.gps.lon,
        heading: Number(body.heading?.yaw ?? 0),
        speed: Number(body.speed ?? approxSpeed),
      },
      source: String(body.source ?? 'lora'),
      stateName: typeof body.state === 'string' ? body.state : null,
      raw: body,
    };
  }

  return null;
}

function publish(event, payload) {
  const packet = JSON.stringify({ event, payload, at: Date.now() });
  for (const client of wss.clients) {
    if (client.readyState === 1) {
      client.send(packet);
    }
  }
}

function publicState() {
  return {
    baseStation: state.baseStation,
    boundary: state.boundary,
    robot: state.robot,
    trail: state.trail,
    coverage: state.coverage
      ? {
          stats: coverageStats(state.coverage),
          grid: {
            width: state.coverage.width,
            height: state.coverage.height,
            cellSizeM: state.coverage.cellSizeM,
          },
        }
      : null,
    lastPath: state.lastPath,
  };
}

app.get('/api/health', (_req, res) => {
  res.json({ ok: true, service: 'robot-lora-server' });
});

app.get('/status', (_req, res) => {
  res.json({
    battery: 85,
    state: state.robot?.state ?? 'IDLE',
    mode: 'SERVER',
    last_cmd: state.lastCommand,
    last_fault: state.lastFault ?? null,
    robot: state.robot
      ? {
          lat: state.robot.lat,
          lon: state.robot.lon,
          heading: state.robot.heading,
          speed: state.robot.speed,
          source: state.robot.source,
          at: state.robot.timestampMs,
        }
      : null,
  });
});

app.post('/command', (req, res) => {
  const parsedBody = parseMaybeJson(req.body);
  const rawCmd =
    (typeof parsedBody === 'object' && parsedBody !== null && typeof parsedBody.cmd === 'string' && parsedBody.cmd) ||
    (typeof parsedBody === 'string' && parsedBody) ||
    (typeof req.body === 'string' && req.body) ||
    null;

  const cmd = normalizeCommand(rawCmd);
  if (!cmd) {
    return res.status(400).send('Missing cmd');
  }

  state.lastCommand = cmd;
  state.lastCommandAt = Date.now();
  publish('command.received', { cmd, at: state.lastCommandAt });

  return res.type('text/plain').send('OK');
});

app.get('/', (_req, res) => {
  res.json({
    ok: true,
    service: 'robot-lora-server',
    endpoints: [
      'GET /status',
      'POST /command',
      'GET /api/health',
      'POST /api/input-area',
      'POST /api/telemetry',
      'GET /api/state',
      'GET /api/coverage',
      'POST /api/path/plan',
    ],
  });
});

app.post('/api/input-area', (req, res) => {
  const { baseStation, boundary, cellSizeM = 2.0 } = req.body ?? {};

  if (!baseStation || typeof baseStation.lat !== 'number' || typeof baseStation.lon !== 'number') {
    return res.status(400).json({ ok: false, error: 'baseStation.lat/lon required' });
  }

  if (!Array.isArray(boundary) || boundary.length < 3) {
    return res.status(400).json({ ok: false, error: 'boundary requires at least 3 points' });
  }

  state.baseStation = baseStation;
  state.boundary = boundary;
  state.coverage = buildCoverageMap(boundary, cellSizeM);
  state.robot = null;
  state.trail = [];
  state.lastPath = [];

  const stats = coverageStats(state.coverage);
  publish('area.updated', { baseStation, boundary, stats });

  return res.json({ ok: true, stats });
});

app.post('/api/telemetry', (req, res) => {
  const parsed = parseIncomingTelemetry(req.body);
  if (!parsed) {
    return res.status(400).json({ ok: false, error: 'Unsupported telemetry payload' });
  }

  // Handle fault notification separately — don't overwrite robot position.
  if (parsed.isFault) {
    state.lastFault = { fault: parsed.fault, action: parsed.action, at: Date.now() };
    publish('fault.received', { fault: parsed.fault, action: parsed.action, at: state.lastFault.at });
    return res.json({ ok: true, fault: parsed.fault, action: parsed.action });
  }

  state.robot = {
    lat: parsed.robot.lat,
    lon: parsed.robot.lon,
    heading: parsed.robot.heading,
    speed: parsed.robot.speed,
    source: parsed.source,
    state: parsed.stateName ?? state.robot?.state ?? 'IDLE',
    timestampMs: Date.now(),
    raw: parsed.raw,
  };

  state.trail.push({ lat: state.robot.lat, lon: state.robot.lon, t: state.robot.timestampMs });
  if (state.trail.length > 5000) {
    state.trail = state.trail.slice(-5000);
  }

  let stats = null;
  if (state.coverage) {
    markCoverage(state.coverage, state.robot, 1.5, state.robot.timestampMs);
    stats = coverageStats(state.coverage);
  }

  publish('telemetry.updated', { robot: state.robot, coverage: stats });

  return res.json({ ok: true, coverage: stats });
});

app.get('/api/state', (_req, res) => {
  res.json({ ok: true, state: publicState() });
});

app.get('/api/coverage', (_req, res) => {
  if (!state.coverage) {
    return res.status(400).json({ ok: false, error: 'input area is not initialized' });
  }

  const cells = [];
  for (let row = 0; row < state.coverage.height; row++) {
    for (let col = 0; col < state.coverage.width; col++) {
      const cell = state.coverage.cells[row][col];
      if (!cell.inside) continue;
      cells.push({ row, col, covered: cell.covered, hits: cell.hits, lastSeenMs: cell.lastSeenMs });
    }
  }

  res.json({
    ok: true,
    stats: coverageStats(state.coverage),
    grid: {
      width: state.coverage.width,
      height: state.coverage.height,
      cellSizeM: state.coverage.cellSizeM,
      cells,
    },
  });
});

app.post('/api/path/plan', (req, res) => {
  if (!state.coverage) {
    return res.status(400).json({ ok: false, error: 'input area is not initialized' });
  }

  const { goal, start } = req.body ?? {};
  if (!goal || typeof goal.lat !== 'number' || typeof goal.lon !== 'number') {
    return res.status(400).json({ ok: false, error: 'goal.lat/lon required' });
  }

  const startPoint = start ?? state.robot ?? state.baseStation;
  if (!startPoint) {
    return res.status(400).json({ ok: false, error: 'no start point available' });
  }

  const path = findPath(state.coverage, startPoint, goal);
  if (!path.ok) {
    state.lastPath = [];
    return res.status(400).json({ ok: false, error: path.reason });
  }

  state.lastPath = path.points;
  publish('path.updated', { points: state.lastPath });
  res.json({ ok: true, points: state.lastPath });
});

wss.on('connection', (socket) => {
  socket.send(JSON.stringify({ event: 'state.snapshot', payload: publicState(), at: Date.now() }));
});

const port = Number(process.env.PORT ?? 8080);
server.listen(port, () => {
  console.log(`[robot-lora-server] listening on http://localhost:${port}`);
});
