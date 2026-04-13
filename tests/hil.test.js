const test = require('node:test');
const assert = require('node:assert/strict');
const http = require('http');
const fs = require('fs');
const os = require('os');
const path = require('path');
const { spawn } = require('child_process');

const SERVER_PORT = 18180;
const BASE_PORT = 18181;
const SERVER_URL = `http://127.0.0.1:${SERVER_PORT}`;
const BASE_URL = `http://127.0.0.1:${BASE_PORT}`;

function wait(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

async function waitForCondition(predicate, {
  timeoutMs = 5000,
  intervalMs = 25,
  label = 'condition',
} = {}) {
  const start = Date.now();
  let lastError = null;

  while (Date.now() - start < timeoutMs) {
    try {
      const result = await predicate();
      if (result) return result;
    } catch (error) {
      lastError = error;
    }
    await wait(intervalMs);
  }

  const detail = lastError?.message ? ` (last error: ${lastError.message})` : '';
  throw new Error(`Timed out waiting for ${label}${detail}`);
}

async function waitForHealthy(url, timeoutMs = 5000) {
  await waitForCondition(async () => {
    const res = await fetch(`${url}/api/health`);
    return res.ok;
  }, {
    timeoutMs,
    intervalMs: 50,
    label: `health at ${url}`,
  });
}

async function waitForMissionState(url, expectedState, options = {}) {
  return waitForCondition(async () => {
    const { res, json } = await getJson(`${url}/api/mission/current`);
    if (res.status !== 200) return false;
    return json?.mission?.state === expectedState ? json : false;
  }, {
    label: `mission state ${expectedState}`,
    ...options,
  });
}

async function waitForSummaryAlert(url, alertCode, options = {}) {
  return waitForCondition(async () => {
    const { res, json } = await getJson(`${url}/api/supervision/summary`);
    if (res.status !== 200 || !json?.summary) return false;
    return json.summary.alerts.some((alert) => alert.code === alertCode) ? json : false;
  }, {
    label: `summary alert ${alertCode}`,
    ...options,
  });
}

async function postJson(url, body) {
  const res = await fetch(url, {
    method: 'POST',
    headers: { 'content-type': 'application/json' },
    body: JSON.stringify(body),
  });
  const text = await res.text();
  return { res, json: text ? JSON.parse(text) : null, text };
}

async function postText(url, body) {
  const res = await fetch(url, {
    method: 'POST',
    headers: { 'content-type': 'text/plain' },
    body,
  });
  return { res, text: await res.text() };
}

async function getJson(url) {
  const res = await fetch(url);
  const text = await res.text();
  return { res, json: text ? JSON.parse(text) : null };
}

function createMockBaseStation() {
  const commands = [];
  const sockets = new Set();
  const server = http.createServer((req, res) => {
    if (req.method === 'POST' && req.url === '/command') {
      const chunks = [];
      req.on('data', (chunk) => chunks.push(chunk));
      req.on('end', () => {
        const body = Buffer.concat(chunks).toString('utf8');
        commands.push(body);
        res.writeHead(200, { 'content-type': 'text/plain' });
        res.end('OK');
      });
      return;
    }

    if (req.method === 'GET' && req.url === '/status') {
      res.writeHead(200, { 'content-type': 'application/json' });
      res.end(JSON.stringify({ ok: true, queue_depth: 0 }));
      return;
    }

    res.writeHead(404);
    res.end();
  });

  server.keepAliveTimeout = 1;
  server.headersTimeout = 1000;
  server.on('connection', (socket) => {
    sockets.add(socket);
    socket.on('close', () => sockets.delete(socket));
  });

  return {
    commands,
    async start() {
      await new Promise((resolve) => server.listen(BASE_PORT, '127.0.0.1', resolve));
    },
    async stop() {
      if (!server.listening) return;
      for (const socket of sockets) socket.destroy();
      if (typeof server.closeAllConnections === 'function') server.closeAllConnections();
      await new Promise((resolve) => server.close(resolve));
    },
  };
}

function startServer(dataDir, envOverrides = {}) {
  const child = spawn(process.execPath, ['src/server.js'], {
    cwd: path.resolve(__dirname, '..'),
    env: {
      ...process.env,
      PORT: String(SERVER_PORT),
      BASE_STATION_URL: BASE_URL,
      ROBOT_LORA_DATA_DIR: dataDir,
      SUPERVISION_TELEMETRY_STALE_MS: '200',
      TELEMETRY_FAILSAFE_ENABLED: '0',
      SAFETY_MONITOR_INTERVAL_MS: '100',
      ...envOverrides,
    },
    stdio: ['ignore', 'pipe', 'pipe'],
  });

  let stdout = '';
  let stderr = '';
  child.stdout.on('data', (chunk) => { stdout += chunk.toString('utf8'); });
  child.stderr.on('data', (chunk) => { stderr += chunk.toString('utf8'); });

  return {
    child,
    get stdout() { return stdout; },
    get stderr() { return stderr; },
    async stop() {
      if (child.exitCode !== null || child.signalCode !== null) return;

      await new Promise((resolve) => {
        let settled = false;
        let forceKillTimer = null;
        const finish = () => {
          if (settled) return;
          settled = true;
          if (forceKillTimer) clearTimeout(forceKillTimer);
          resolve();
        };

        child.once('exit', finish);
        child.once('close', finish);

        forceKillTimer = setTimeout(() => {
          if (child.exitCode === null && child.signalCode === null) {
            try {
              child.kill('SIGKILL');
            } catch {
              finish();
            }
          }
        }, 250);
        if (typeof forceKillTimer.unref === 'function') forceKillTimer.unref();

        try {
          child.kill();
        } catch {
          finish();
          return;
        }

        if (child.exitCode !== null || child.signalCode !== null) {
          finish();
        }
      });
    },
  };
}

test('Comprehensive /command acceptance for app/server virtual commands', async () => {
  const dataDir = fs.mkdtempSync(path.join(os.tmpdir(), 'robot-lora-server-hil-command-matrix-'));
  const base = createMockBaseStation();
  await base.start();

  const server = startServer(dataDir, {
    COMMAND_RATE_LIMIT_PER_WINDOW: '200000',
    DRIVE_DUPLICATE_SUPPRESS_MS: '0',
  });

  try {
    await waitForHealthy(SERVER_URL);

    const commandMatrix = [
      { body: 'MANUAL', expected: 'MANUAL' },
      { body: 'PAUSE', expected: 'PAUSE' },
      { body: 'AUTO', expected: 'AUTO' },
      { body: 'ESTOP', expected: 'ESTOP' },
      { body: 'RESET', expected: 'RESET' },
      { body: 'FORWARD', expected: 'FORWARD' },
      { body: 'BACKWARD', expected: 'BACKWARD' },
      { body: 'LEFT', expected: 'LEFT' },
      { body: 'RIGHT', expected: 'RIGHT' },
      { body: 'STOP', expected: 'STOP' },
      { body: 'D:100,0', expected: 'D:100,0' },
      { body: 'D:-100,0', expected: 'D:-100,0' },
      { body: 'D:0,100', expected: 'D:0,100' },
      { body: 'D:0,-100', expected: 'D:0,-100' },
      { body: 'DRIVE,THROTTLE:35,TURN:-20', expected: 'D:35,-20' },
    ];

    for (const item of commandMatrix) {
      const { res, text } = await postText(`${SERVER_URL}/command`, item.body);
      assert.equal(res.status, 200, `expected 200 for ${item.body}, got ${res.status}: ${text}`);
      await waitForCondition(() => base.commands.includes(item.expected), {
        timeoutMs: 600,
        intervalMs: 20,
        label: `base command ${item.expected}`,
      });
    }

    {
      const { res, text } = await postJson(`${SERVER_URL}/command`, {
        cmd: 'DRIVE',
        throttle: 72,
        turn: -31,
      });
      assert.equal(res.status, 200, `expected 200 for JSON DRIVE, got ${res.status}: ${text}`);
      await waitForCondition(() => base.commands.includes('D:72,-31'), {
        timeoutMs: 600,
        intervalMs: 20,
        label: 'base command D:72,-31',
      });
    }
  } finally {
    await server.stop();
    await base.stop();
    fs.rmSync(dataDir, { recursive: true, force: true });
  }
});

test('Phase E supervision summary and Phase F fail-safe scenarios', async () => {
  const dataDir = fs.mkdtempSync(path.join(os.tmpdir(), 'robot-lora-server-hil-'));
  const base = createMockBaseStation();
  await base.start();

  const server = startServer(dataDir);
  try {
    await waitForHealthy(SERVER_URL);

    const areaBody = {
      baseStation: { lat: 41.0, lon: -81.0 },
      boundary: [
        { lat: 41.0, lon: -81.0 },
        { lat: 41.0, lon: -80.9995 },
        { lat: 40.9995, lon: -80.9995 },
        { lat: 40.9995, lon: -81.0 },
      ],
      cellSizeM: 2.0,
    };

    {
      const { res, json } = await postJson(`${SERVER_URL}/api/input-area`, areaBody);
      assert.equal(res.status, 200);
      assert.equal(json.ok, true);
    }

    {
      const { res, json } = await getJson(`${SERVER_URL}/api/supervision/summary`);
      assert.equal(res.status, 200);
      assert.equal(json.ok, true);
      assert.equal(json.summary.mission.state, 'CONFIGURING');
      assert.ok(json.summary.alerts.some((alert) => alert.code === 'PATH_MISSING'));
      assert.ok(json.summary.workflows.some((workflow) => workflow.id === 'preflight'));
    }

    {
      const { res, json } = await getJson(`${SERVER_URL}/api/bridge/sync`);
      assert.equal(res.status, 200);
      assert.equal(json.ok, true);
      assert.equal(json.mission.state, 'CONFIGURING');
      assert.ok(Array.isArray(json.lastPath));
      assert.ok(json.lora);
    }

    {
      const { res } = await postJson(`${SERVER_URL}/api/path/plan`, {
        goal: { lat: 40.9997, lon: -80.9997 },
      });
      assert.equal(res.status, 200);
    }

    {
      const { res } = await postJson(`${SERVER_URL}/api/operator/workflows/preflight/steps/radio-check`, {
        checked: true,
        actor: 'test-op',
        note: 'Radio link verified',
      });
      assert.equal(res.status, 200);
    }

    {
      const { res, json } = await postJson(`${SERVER_URL}/api/operator/notes`, {
        text: 'Preflight note recorded',
        category: 'preflight',
        actor: 'test-op',
      });
      assert.equal(res.status, 200);
      assert.equal(json.note.category, 'preflight');
    }

    {
      const { res } = await postJson(`${SERVER_URL}/api/mission/start`, {});
      assert.equal(res.status, 200);
      assert.ok(base.commands.includes('AUTO'));
    }

    {
      const summary = await waitForSummaryAlert(SERVER_URL, 'TELEMETRY_STALE', {
        timeoutMs: 800,
        intervalMs: 25,
      });
      assert.ok(summary.summary.alerts.some((alert) => alert.code === 'TELEMETRY_STALE'));
    }

    {
      const { res } = await postJson(`${SERVER_URL}/api/telemetry`, {
        state: 'AUTO',
        gps: { lat: 40.9997, lon: -80.9997, fix: 1, sat: 8, hdop: 0.9 },
        motor: { m1: 35, m2: 34 },
        heading: { yaw: 45.0, pitch: 0.0 },
        disp: { salt: 50, brine: 50 },
        temp: 1.2,
        prox: { left: 120, right: 118 },
      });
      assert.equal(res.status, 200);
    }

    {
      const { res } = await postText(`${SERVER_URL}/command`, 'FORWARD');
      assert.equal(res.status, 200);
      const { json } = await getJson(`${SERVER_URL}/api/mission/current`);
      assert.equal(json.mission.state, 'PAUSED');
    }

    {
      const { res } = await postJson(`${SERVER_URL}/api/mission/resume`, {});
      assert.equal(res.status, 200);
    }

    {
      const { res } = await postJson(`${SERVER_URL}/api/telemetry`, {
        fault: 'IMU_TIMEOUT',
        action: 'ESTOP',
      });
      assert.equal(res.status, 200);
      const { json } = await getJson(`${SERVER_URL}/api/mission/current`);
      assert.equal(json.mission.state, 'ABORTED');
    }

    {
      const { res } = await postText(`${SERVER_URL}/command`, 'RESET');
      assert.equal(res.status, 200);
    }

    {
      const { res, json } = await getJson(`${SERVER_URL}/api/operator/workflows`);
      assert.equal(res.status, 200);
      assert.ok(json.notes.some((note) => note.text === 'Preflight note recorded'));
    }
  } finally {
    await server.stop();
    await base.stop();
    fs.rmSync(dataDir, { recursive: true, force: true });
  }
});

test('P0 telemetry stale fail-safe issues ESTOP and aborts mission', async () => {
  const dataDir = fs.mkdtempSync(path.join(os.tmpdir(), 'robot-lora-server-hil-failsafe-'));
  const base = createMockBaseStation();
  await base.start();

  const server = startServer(dataDir, {
    TELEMETRY_FAILSAFE_ENABLED: '1',
    TELEMETRY_FAILSAFE_ACTION: 'ESTOP',
    TELEMETRY_FAILSAFE_COOLDOWN_MS: '200',
    SUPERVISION_TELEMETRY_STALE_MS: '200',
    SAFETY_MONITOR_INTERVAL_MS: '50',
  });

  try {
    await waitForHealthy(SERVER_URL);

    const areaBody = {
      baseStation: { lat: 41.0, lon: -81.0 },
      boundary: [
        { lat: 41.0, lon: -81.0 },
        { lat: 41.0, lon: -80.9995 },
        { lat: 40.9995, lon: -80.9995 },
        { lat: 40.9995, lon: -81.0 },
      ],
      cellSizeM: 2.0,
    };

    {
      const { res } = await postJson(`${SERVER_URL}/api/input-area`, areaBody);
      assert.equal(res.status, 200);
    }

    {
      const { res } = await postJson(`${SERVER_URL}/api/path/plan`, {
        goal: { lat: 40.9997, lon: -80.9997 },
      });
      assert.equal(res.status, 200);
    }

    {
      const { res } = await postJson(`${SERVER_URL}/api/mission/start`, {});
      assert.equal(res.status, 200);
      assert.ok(base.commands.includes('AUTO'));
    }

    await waitForCondition(() => base.commands.includes('ESTOP'), {
      timeoutMs: 800,
      intervalMs: 25,
      label: 'ESTOP command',
    });

    {
      const json = await waitForMissionState(SERVER_URL, 'ABORTED', {
        timeoutMs: 800,
        intervalMs: 25,
      });
      assert.equal(json.mission.state, 'ABORTED');
    }

    {
      const { res, json } = await getJson(`${SERVER_URL}/api/supervision/summary`);
      assert.equal(res.status, 200);
      assert.equal(json.summary.safety.telemetryFailsafeEnabled, true);
      assert.equal(json.summary.safety.telemetryFailsafeAction, 'ESTOP');
      assert.ok(json.summary.safety.telemetryFailsafeAt);
      assert.ok(json.summary.alerts.some((alert) => alert.code === 'TELEMETRY_FAILSAFE_TRIGGERED'));
    }
  } finally {
    await server.stop();
    await base.stop();
    fs.rmSync(dataDir, { recursive: true, force: true });
  }
});

test('RESET stays usable during telemetry outage and clears failsafe after recovery telemetry', async () => {
  const dataDir = fs.mkdtempSync(path.join(os.tmpdir(), 'robot-lora-server-hil-reset-recovery-'));
  const base = createMockBaseStation();
  await base.start();

  const server = startServer(dataDir, {
    TELEMETRY_FAILSAFE_ENABLED: '1',
    TELEMETRY_FAILSAFE_ACTION: 'ESTOP',
    TELEMETRY_FAILSAFE_COOLDOWN_MS: '200',
    SUPERVISION_TELEMETRY_STALE_MS: '200',
    SAFETY_MONITOR_INTERVAL_MS: '50',
    COMMAND_STATE_CONFIRM_TIMEOUT_MS: '200',
    COMMAND_STATE_CONFIRM_POLL_MS: '25',
  });

  try {
    await waitForHealthy(SERVER_URL);

    const areaBody = {
      baseStation: { lat: 41.0, lon: -81.0 },
      boundary: [
        { lat: 41.0, lon: -81.0 },
        { lat: 41.0, lon: -80.9995 },
        { lat: 40.9995, lon: -80.9995 },
        { lat: 40.9995, lon: -81.0 },
      ],
      cellSizeM: 2.0,
    };

    {
      const { res } = await postJson(`${SERVER_URL}/api/input-area`, areaBody);
      assert.equal(res.status, 200);
    }

    {
      const { res } = await postJson(`${SERVER_URL}/api/path/plan`, {
        goal: { lat: 40.9997, lon: -80.9997 },
      });
      assert.equal(res.status, 200);
    }

    {
      const { res } = await postJson(`${SERVER_URL}/api/mission/start`, {});
      assert.equal(res.status, 200);
      assert.ok(base.commands.includes('AUTO'));
    }

    await waitForCondition(() => base.commands.includes('ESTOP'), {
      timeoutMs: 800,
      intervalMs: 25,
      label: 'ESTOP command before recovery reset',
    });

    {
      const { res, text } = await postText(`${SERVER_URL}/command`, 'RESET');
      assert.ok([200, 202].includes(res.status), `expected RESET to be forwarded during telemetry outage, got ${res.status}: ${text}`);
      assert.ok(base.commands.includes('RESET'));
    }

    {
      const { res } = await postJson(`${SERVER_URL}/api/telemetry`, {
        state: 'PAUSE',
        gps: { lat: 40.9997, lon: -80.9997, fix: 1, sat: 8, hdop: 0.9 },
        motor: { m1: 0, m2: 0 },
        heading: { yaw: 45.0, pitch: 0.0 },
        disp: { salt: 0, brine: 0 },
        temp: 1.2,
        prox: { left: 120, right: 118 },
      });
      assert.equal(res.status, 200);
    }

    {
      const summary = await waitForCondition(async () => {
        const { res, json } = await getJson(`${SERVER_URL}/api/supervision/summary`);
        if (res.status !== 200 || !json?.summary) return false;
        if (json.summary.robot.state !== 'PAUSE') return false;
        return json.summary.safety.telemetryFailsafeAt == null ? json : false;
      }, {
        timeoutMs: 600,
        intervalMs: 25,
        label: 'failsafe recovery summary',
      });
      assert.equal(summary.summary.robot.state, 'PAUSE');
      assert.equal(summary.summary.safety.telemetryFailsafeAt, null);
      assert.equal(summary.summary.safety.telemetryFailsafeReason, null);
    }
  } finally {
    await server.stop();
    await base.stop();
    fs.rmSync(dataDir, { recursive: true, force: true });
  }
});

test('P0 geofence fail-safe issues ESTOP and aborts mission when robot exits boundary', async () => {
  const dataDir = fs.mkdtempSync(path.join(os.tmpdir(), 'robot-lora-server-hil-geofence-'));
  const base = createMockBaseStation();
  await base.start();

  const server = startServer(dataDir, {
    TELEMETRY_FAILSAFE_ENABLED: '0',
    GEOFENCE_FAILSAFE_ENABLED: '1',
    GEOFENCE_FAILSAFE_ACTION: 'ESTOP',
    GEOFENCE_FAILSAFE_COOLDOWN_MS: '100',
    SAFETY_MONITOR_INTERVAL_MS: '50',
  });

  try {
    await waitForHealthy(SERVER_URL);

    const areaBody = {
      baseStation: { lat: 41.0, lon: -81.0 },
      boundary: [
        { lat: 41.0, lon: -81.0 },
        { lat: 41.0, lon: -80.9995 },
        { lat: 40.9995, lon: -80.9995 },
        { lat: 40.9995, lon: -81.0 },
      ],
      cellSizeM: 2.0,
    };

    {
      const { res } = await postJson(`${SERVER_URL}/api/input-area`, areaBody);
      assert.equal(res.status, 200);
    }

    {
      const { res } = await postJson(`${SERVER_URL}/api/path/plan`, {
        goal: { lat: 40.9997, lon: -80.9997 },
      });
      assert.equal(res.status, 200);
    }

    {
      const { res } = await postJson(`${SERVER_URL}/api/mission/start`, {});
      assert.equal(res.status, 200);
    }

    {
      const { res } = await postJson(`${SERVER_URL}/api/telemetry`, {
        state: 'AUTO',
        gps: { lat: 41.01, lon: -81.01, fix: 1, sat: 8, hdop: 0.9 },
        motor: { m1: 35, m2: 34 },
        heading: { yaw: 45.0, pitch: 0.0 },
        disp: { salt: 50, brine: 50 },
        temp: 1.2,
        prox: { left: 120, right: 118 },
      });
      assert.equal(res.status, 200);
    }

    await waitForCondition(() => base.commands.includes('ESTOP'), {
      timeoutMs: 600,
      intervalMs: 25,
      label: 'geofence ESTOP command',
    });

    {
      const json = await waitForMissionState(SERVER_URL, 'ABORTED', {
        timeoutMs: 600,
        intervalMs: 25,
      });
      assert.equal(json.mission.state, 'ABORTED');
    }

    {
      const { res, json } = await getJson(`${SERVER_URL}/api/supervision/summary`);
      assert.equal(res.status, 200);
      assert.equal(json.summary.safety.geofenceFailsafeEnabled, true);
      assert.equal(json.summary.safety.geofenceFailsafeAction, 'ESTOP');
      assert.ok(json.summary.safety.geofenceFailsafeAt);
      assert.ok(json.summary.alerts.some((alert) => alert.code === 'GEOFENCE_FAILSAFE_TRIGGERED'));
    }
  } finally {
    await server.stop();
    await base.stop();
    fs.rmSync(dataDir, { recursive: true, force: true });
  }
});

test('P0 health endpoint reports degraded when telemetry is stale during running mission', async () => {
  const dataDir = fs.mkdtempSync(path.join(os.tmpdir(), 'robot-lora-server-hil-health-'));
  const base = createMockBaseStation();
  await base.start();

  const server = startServer(dataDir, {
    TELEMETRY_FAILSAFE_ENABLED: '0',
    GEOFENCE_FAILSAFE_ENABLED: '0',
    SUPERVISION_TELEMETRY_STALE_MS: '200',
  });

  try {
    await waitForHealthy(SERVER_URL);

    const areaBody = {
      baseStation: { lat: 41.0, lon: -81.0 },
      boundary: [
        { lat: 41.0, lon: -81.0 },
        { lat: 41.0, lon: -80.9995 },
        { lat: 40.9995, lon: -80.9995 },
        { lat: 40.9995, lon: -81.0 },
      ],
      cellSizeM: 2.0,
    };

    {
      const { res } = await postJson(`${SERVER_URL}/api/input-area`, areaBody);
      assert.equal(res.status, 200);
    }

    {
      const { res } = await postJson(`${SERVER_URL}/api/path/plan`, {
        goal: { lat: 40.9997, lon: -80.9997 },
      });
      assert.equal(res.status, 200);
    }

    {
      const { res } = await postJson(`${SERVER_URL}/api/mission/start`, {});
      assert.equal(res.status, 200);
    }

    {
      const health = await waitForCondition(async () => {
        const res = await fetch(`${SERVER_URL}/api/health`);
        const json = await res.json();
        return res.status === 503 ? { res, json } : false;
      }, {
        timeoutMs: 700,
        intervalMs: 25,
        label: 'degraded health status',
      });
      assert.equal(health.res.status, 503);
      assert.equal(health.json.ok, false);
      assert.equal(health.json.checks.telemetry, false);
      assert.equal(health.json.missionState, 'RUNNING');
    }
  } finally {
    await server.stop();
    await base.stop();
    fs.rmSync(dataDir, { recursive: true, force: true });
  }
});

test('path planning honors explicit app start point over stale robot telemetry', async () => {
  const dataDir = fs.mkdtempSync(path.join(os.tmpdir(), 'robot-lora-server-hil-path-start-'));
  const base = createMockBaseStation();
  await base.start();

  const server = startServer(dataDir, {
    TELEMETRY_FAILSAFE_ENABLED: '0',
    GEOFENCE_FAILSAFE_ENABLED: '0',
  });

  try {
    await waitForHealthy(SERVER_URL);

    const areaBody = {
      baseStation: { lat: 41.0, lon: -81.0 },
      boundary: [
        { lat: 41.0, lon: -81.0 },
        { lat: 41.0, lon: -80.9995 },
        { lat: 40.9995, lon: -80.9995 },
        { lat: 40.9995, lon: -81.0 },
      ],
      cellSizeM: 2.0,
    };

    {
      const { res } = await postJson(`${SERVER_URL}/api/input-area`, areaBody);
      assert.equal(res.status, 200);
    }

    {
      const { res } = await postJson(`${SERVER_URL}/api/telemetry`, {
        state: 'IDLE',
        gps: { lat: 40.99, lon: -80.99, fix: 1, sat: 8, hdop: 0.9 },
        motor: { m1: 0, m2: 0 },
        heading: { yaw: 0.0, pitch: 0.0 },
        disp: { salt: 50, brine: 50 },
        temp: 1.2,
        prox: { left: 120, right: 118 },
      });
      assert.equal(res.status, 200);
    }

    {
      const { res, json } = await postJson(`${SERVER_URL}/api/path/plan`, {
        mode: 'coverage',
        start: { lat: 41.0, lon: -81.0 },
        homePoint: { lat: 41.0, lon: -81.0 },
        coverageWidthM: 0.5,
        returnToBase: true,
        saltPct: 100,
        brinePct: 0,
      });
      assert.equal(res.status, 200);
      assert.equal(json.ok, true);
      assert.ok(Array.isArray(json.points));
      assert.ok(json.points.length > 1);
      assert.ok(Math.abs(json.points[0].lat - 41.0) < 0.0003);
      assert.ok(Math.abs(json.points[0].lon - (-81.0)) < 0.0003);
    }
  } finally {
    await server.stop();
    await base.stop();
    fs.rmSync(dataDir, { recursive: true, force: true });
  }
});
