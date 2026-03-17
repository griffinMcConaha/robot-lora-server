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

async function waitForHealthy(url, timeoutMs = 10000) {
  const start = Date.now();
  while (Date.now() - start < timeoutMs) {
    try {
      const res = await fetch(`${url}/api/health`);
      if (res.ok) return;
    } catch {
      // retry
    }
    await wait(100);
  }
  throw new Error(`Timed out waiting for health at ${url}`);
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

  return {
    commands,
    async start() {
      await new Promise((resolve) => server.listen(BASE_PORT, '127.0.0.1', resolve));
    },
    async stop() {
      await new Promise((resolve) => server.close(resolve));
    },
  };
}

function startServer(dataDir) {
  const child = spawn(process.execPath, ['src/server.js'], {
    cwd: path.resolve(__dirname, '..'),
    env: {
      ...process.env,
      PORT: String(SERVER_PORT),
      BASE_STATION_URL: BASE_URL,
      ROBOT_LORA_DATA_DIR: dataDir,
      SUPERVISION_TELEMETRY_STALE_MS: '200',
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
      if (child.killed) return;
      child.kill();
      await new Promise((resolve) => child.once('exit', resolve));
    },
  };
}

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

    await wait(300);

    {
      const { json } = await getJson(`${SERVER_URL}/api/supervision/summary`);
      assert.ok(json.summary.alerts.some((alert) => alert.code === 'TELEMETRY_STALE'));
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