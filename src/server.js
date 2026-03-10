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

const server = http.createServer(app);
const wss = new WebSocketServer({ server });

const state = {
  baseStation: null,
  boundary: null,
  coverage: null,
  robot: null,
  trail: [],
  lastPath: [],
};

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

app.get('/', (_req, res) => {
  res.json({
    ok: true,
    service: 'robot-lora-server',
    endpoints: [
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
  if (!state.coverage) {
    return res.status(400).json({ ok: false, error: 'input area is not initialized' });
  }

  const { robot, source = 'unknown' } = req.body ?? {};
  if (!robot || typeof robot.lat !== 'number' || typeof robot.lon !== 'number') {
    return res.status(400).json({ ok: false, error: 'robot.lat/lon required' });
  }

  state.robot = {
    lat: robot.lat,
    lon: robot.lon,
    heading: Number(robot.heading ?? 0),
    speed: Number(robot.speed ?? 0),
    source,
    timestampMs: Date.now(),
  };

  state.trail.push({ lat: state.robot.lat, lon: state.robot.lon, t: state.robot.timestampMs });
  if (state.trail.length > 5000) {
    state.trail = state.trail.slice(-5000);
  }

  markCoverage(state.coverage, state.robot, 1.5, state.robot.timestampMs);
  const stats = coverageStats(state.coverage);

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
