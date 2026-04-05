/**
 * lora_bridge.js  —  Phase C
 *
 * Outbound command path: server → base station (ESP32) → LoRa → STM32.
 *
 * Responsibilities
 * ─────────────────
 * 1. sendCommand(cmd)   POST a single text command to the base station's
 *                       POST /command endpoint.  Returns { ok, status, body }.
 *
 * 2. pushWaypoints(points)
 *                       Push a full waypoint set to the STM32 using the
 *                       LORA_WIRE text protocol:
 *                         WPCLEAR
 *                         WP:0:<lat>,<lon>,<salt>,<brine>
 *                         …
 *                         WPLOAD:<n>
 *                       Each line is sent sequentially with a small delay so
 *                       the gateway/base-station queue does not overflow.
 *
 * 3. getStatus()        Return a status snapshot for GET /api/lora/status.
 *
 * 4. resetWpState()     Reset wpPushState to 'none' (called on mission abort/
 *                       complete/reset so the next mission starts clean).
 *
 * Base station URL
 * ─────────────────
 * Set BASE_STATION_URL env-var to override the default (ESP32 AP address).
 *
 * points schema (array elements)
 * ───────────────────────────────
 * {
 *   lat:      number   // decimal degrees
 *   lon:      number   // decimal degrees
 *   salt:     number   // 0–100 integer percent
 *   brine:    number   // 0–100 integer percent
 * }
 *
 * Security note
 * ─────────────
 * BASE_STATION_URL is expected to be a local LAN address.  User-supplied
 * waypoints are validated server-side before reaching this module.
 */

'use strict';

const http  = require('http');
const https = require('https');
const { LORA_WIRE, ARBITRATION } = require('./contracts');

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------
const BASE_STATION_URL = (process.env.BASE_STATION_URL ?? 'http://192.168.4.1').replace(/\/$/, '');
const REQUEST_TIMEOUT_MS = Number(process.env.LORA_REQUEST_TIMEOUT_MS ?? 3000);
const TRANSIENT_RETRY_MAX = Number(process.env.LORA_TRANSIENT_RETRY_MAX ?? 2);
const TRANSIENT_RETRY_MS = Number(process.env.LORA_TRANSIENT_RETRY_MS ?? 200);
const BRIDGE_DEGRADED_FAILURE_THRESHOLD = Number(process.env.BRIDGE_DEGRADED_FAILURE_THRESHOLD ?? 3);
const ACK_WAIT_TIMEOUT_MS = Number(process.env.LORA_ACK_WAIT_TIMEOUT_MS ?? 3000);
const ACK_POLL_MS = Number(process.env.LORA_ACK_POLL_MS ?? 120);

// ---------------------------------------------------------------------------
// Module state
// ---------------------------------------------------------------------------
let _wpPushState    = ARBITRATION.WP_NONE;   // none | pending | committed | failed
let _lastCmd        = null;
let _lastCmdAt      = null;
let _lastCmdError   = null;
let _wpPushError    = null;
let _wpPushCount    = 0;       // number of WP lines sent in last push
let _wpPushAt       = null;    // timestamp of last committed push
let _lastSuccessAt  = null;
let _lastErrorAt    = null;
let _consecutiveFailures = 0;
let _degradedSince  = null;
let _baseStationSnapshot = {
  statusOk: false,
  statusCode: null,
  statusError: null,
  state: null,
  mode: null,
  queueDepth: null,
  ackCount: null,
  lastAck: null,
  lastLoRa: null,
  refreshedAt: null,
};

// ---------------------------------------------------------------------------
// Low-level HTTP helper
// ---------------------------------------------------------------------------

/**
 * POST text/plain body to BASE_STATION_URL/command.
 * Returns { ok: boolean, status: number|null, body: string, error: string|null }.
 * Never throws.
 */
async function _post(path, body) {
  return new Promise((resolve) => {
    const bodyBuf = Buffer.from(body, 'utf8');
    const urlStr  = `${BASE_STATION_URL}${path}`;

    let parsedUrl;
    try {
      parsedUrl = new URL(urlStr);
    } catch {
      resolve({ ok: false, status: null, body: '', error: `Invalid URL: ${urlStr}` });
      return;
    }

    const lib     = parsedUrl.protocol === 'https:' ? https : http;
    const options = {
      hostname: parsedUrl.hostname,
      port:     parsedUrl.port || (parsedUrl.protocol === 'https:' ? 443 : 80),
      path:     parsedUrl.pathname + parsedUrl.search,
      method:   'POST',
      headers:  {
        'Content-Type':   'text/plain',
        'Content-Length': bodyBuf.length,
      },
    };

    const req = lib.request(options, (res) => {
      const chunks = [];
      res.on('data', (c) => chunks.push(c));
      res.on('end', () => {
        const respBody = Buffer.concat(chunks).toString('utf8');
        resolve({ ok: res.statusCode >= 200 && res.statusCode < 300, status: res.statusCode, body: respBody, error: null });
      });
    });

    req.setTimeout(REQUEST_TIMEOUT_MS, () => {
      req.destroy();
      resolve({ ok: false, status: null, body: '', error: `Request timeout (${REQUEST_TIMEOUT_MS} ms)` });
    });

    req.on('error', (err) => {
      resolve({ ok: false, status: null, body: '', error: err.message });
    });

    req.write(bodyBuf);
    req.end();
  });
}

/**
 * GET helper for base-station endpoints.
 * Returns { ok, status, body, error } and never throws.
 */
async function _get(path) {
  return new Promise((resolve) => {
    const urlStr = `${BASE_STATION_URL}${path}`;

    let parsedUrl;
    try {
      parsedUrl = new URL(urlStr);
    } catch {
      resolve({ ok: false, status: null, body: '', error: `Invalid URL: ${urlStr}` });
      return;
    }

    const lib = parsedUrl.protocol === 'https:' ? https : http;
    const options = {
      hostname: parsedUrl.hostname,
      port: parsedUrl.port || (parsedUrl.protocol === 'https:' ? 443 : 80),
      path: parsedUrl.pathname + parsedUrl.search,
      method: 'GET',
    };

    const req = lib.request(options, (res) => {
      const chunks = [];
      res.on('data', (c) => chunks.push(c));
      res.on('end', () => {
        const respBody = Buffer.concat(chunks).toString('utf8');
        resolve({ ok: res.statusCode >= 200 && res.statusCode < 300, status: res.statusCode, body: respBody, error: null });
      });
    });

    req.setTimeout(REQUEST_TIMEOUT_MS, () => {
      req.destroy();
      resolve({ ok: false, status: null, body: '', error: `Request timeout (${REQUEST_TIMEOUT_MS} ms)` });
    });

    req.on('error', (err) => {
      resolve({ ok: false, status: null, body: '', error: err.message });
    });

    req.end();
  });
}

function _applyBaseStatusSnapshot(payload, meta = {}) {
  if (!payload || typeof payload !== 'object') {
    _baseStationSnapshot = {
      ..._baseStationSnapshot,
      statusOk: false,
      statusCode: meta.status ?? null,
      statusError: meta.error ?? 'Invalid base station status payload',
      refreshedAt: new Date().toISOString(),
    };
    return _baseStationSnapshot;
  }

  _baseStationSnapshot = {
    ..._baseStationSnapshot,
    statusOk: Boolean(meta.ok ?? true),
    statusCode: meta.status ?? null,
    statusError: meta.error ?? null,
    state: typeof payload.state === 'string' ? payload.state : _baseStationSnapshot.state,
    mode: typeof payload.mode === 'string' ? payload.mode : _baseStationSnapshot.mode,
    queueDepth: Number.isFinite(Number(payload.queue_depth)) ? Number(payload.queue_depth) : _baseStationSnapshot.queueDepth,
    ackCount: Number.isFinite(Number(payload.ack_count)) ? Number(payload.ack_count) : _baseStationSnapshot.ackCount,
    lastAck: typeof payload.last_ack === 'string' ? payload.last_ack.trim() : _baseStationSnapshot.lastAck,
    lastLoRa: typeof payload.last_lora === 'string' ? payload.last_lora.trim() : _baseStationSnapshot.lastLoRa,
    refreshedAt: new Date().toISOString(),
  };
  return _baseStationSnapshot;
}

async function _readBaseStatus() {
  const result = await _get('/status');
  if (!result.ok) {
    _baseStationSnapshot = {
      ..._baseStationSnapshot,
      statusOk: false,
      statusCode: result.status ?? null,
      statusError: result.error ?? `HTTP ${result.status}`,
      refreshedAt: new Date().toISOString(),
    };
    return { ok: false, error: _baseStationSnapshot.statusError, status: result.status ?? null, snapshot: _baseStationSnapshot };
  }

  let parsed;
  try {
    parsed = JSON.parse(result.body);
  } catch {
    _baseStationSnapshot = {
      ..._baseStationSnapshot,
      statusOk: false,
      statusCode: result.status ?? null,
      statusError: 'Base station /status returned invalid JSON',
      refreshedAt: new Date().toISOString(),
    };
    return { ok: false, error: _baseStationSnapshot.statusError, status: result.status ?? null, snapshot: _baseStationSnapshot };
  }

  return {
    ok: true,
    status: result.status ?? null,
    snapshot: _applyBaseStatusSnapshot(parsed, { ok: true, status: result.status ?? null }),
  };
}

/**
 * Sleep helper for inter-line delay between WP commands.
 * @param {number} ms
 */
function _sleep(ms) {
  return new Promise((r) => setTimeout(r, ms));
}

function _normalizeFrameText(text) {
  return String(text ?? '').trim();
}

function _unwrapAckFrame(text) {
  const raw = _normalizeFrameText(text);
  if (!raw) return null;

  const wrapped = raw.match(/^S:\d+:(?:M|E):(.*)$/);
  if (wrapped) {
    const payload = _normalizeFrameText(wrapped[1]);
    return {
      raw,
      wrapped: true,
      payload,
      ack: payload.startsWith('ACK:') ? payload : null,
    };
  }

  return {
    raw,
    wrapped: false,
    payload: raw,
    ack: raw.startsWith('ACK:') ? raw : null,
  };
}

function _expectedAckForCommand(cmd) {
  switch (cmd) {
    case 'AUTO':
      return 'ACK:STATE:AUTO';
    case 'MANUAL':
      return 'ACK:STATE:MANUAL';
    case 'PAUSE':
      return 'ACK:STATE:PAUSE';
    case 'ESTOP':
      return 'ACK:STATE:ESTOP';
    case 'RESET':
      return 'ACK:STATE:PAUSE';
    default:
      return null;
  }
}

async function _readLastLoRa() {
  const result = await _get('/last_lora');
  if (!result.ok) return { ok: false, error: result.error ?? `HTTP ${result.status}` };
  const frame = _unwrapAckFrame(result.body);
  if (frame?.raw) {
    _baseStationSnapshot = {
      ..._baseStationSnapshot,
      lastLoRa: frame.raw,
      refreshedAt: new Date().toISOString(),
    };
  }
  return { ok: true, frame };
}

async function _waitForAck(expectedAck, options = {}) {
  const { baselineRaw = null, wrappedOnly = true, timeoutMs = ACK_WAIT_TIMEOUT_MS } = options;
  const startedAt = Date.now();
  let lastSeenRaw = baselineRaw;
  let lastSeenAck = null;

  while ((Date.now() - startedAt) < timeoutMs) {
    const statusSnapshot = await _readBaseStatus();
    if (statusSnapshot.ok && typeof statusSnapshot.snapshot?.lastAck === 'string' && statusSnapshot.snapshot.lastAck) {
      const frame = _unwrapAckFrame(statusSnapshot.snapshot.lastAck);
      if (frame?.ack) {
        lastSeenRaw = frame.raw;
        lastSeenAck = frame.ack;
        if ((!wrappedOnly || frame.wrapped) && frame.ack === expectedAck && frame.raw !== baselineRaw) {
          return { ok: true, ack: frame.ack, raw: frame.raw, source: 'status' };
        }
      }
    }

    const snapshot = await _readLastLoRa();
    if (snapshot.ok && snapshot.frame?.ack) {
      const { frame } = snapshot;
      lastSeenRaw = frame.raw;
      lastSeenAck = frame.ack;

      if ((!wrappedOnly || frame.wrapped) && frame.ack === expectedAck && frame.raw !== baselineRaw) {
        return { ok: true, ack: frame.ack, raw: frame.raw, source: 'last_lora' };
      }
    }

    await _sleep(ACK_POLL_MS);
  }

  return {
    ok: false,
    error: `Timed out waiting for ${expectedAck}`,
    lastSeenRaw,
    lastSeenAck,
  };
}

// Phase D: retry budget and delay when the base station returns 503 queue_full.
// The base station now returns 503 with body 'queue_full' instead of silently
// dropping commands.  We back off and retry so no WP frame is lost.
const QUEUE_FULL_RETRY_MAX = 6;
const QUEUE_FULL_RETRY_MS  = 250;

/**
 * Like _post() but retries up to QUEUE_FULL_RETRY_MAX times when the base
 * station responds 503 "queue_full" (Phase D back-pressure).
 * All other errors are returned immediately without retrying.
 *
 * @param {string} path
 * @param {string} body
 * @returns {Promise<{ ok, status, body, error }>}
 */
async function _postWithRetry(path, body) {
  for (let attempt = 0; attempt <= QUEUE_FULL_RETRY_MAX; attempt++) {
    const r = await _post(path, body);
    if (r.ok) return r;
    const isQueueFull = r.status === 503 && r.body === 'queue_full';
    const isTransient =
      isQueueFull
      || r.status == null
      || r.status >= 500
      || (typeof r.error === 'string' && r.error.toLowerCase().includes('timeout'));

    const maxRetries = isQueueFull ? QUEUE_FULL_RETRY_MAX : TRANSIENT_RETRY_MAX;
    const retryDelayMs = isQueueFull ? QUEUE_FULL_RETRY_MS : TRANSIENT_RETRY_MS;

    if (!isTransient || attempt === maxRetries) return r;
    await _sleep(retryDelayMs);
  }
  // unreachable
  return { ok: false, status: 503, body: 'queue_full', error: 'Max queue-full retries exceeded' };
}

function _recordResult(result) {
  if (result?.ok) {
    _lastSuccessAt = new Date().toISOString();
    _consecutiveFailures = 0;
    _degradedSince = null;
    return;
  }

  _lastErrorAt = new Date().toISOString();
  _consecutiveFailures += 1;
  if (_consecutiveFailures >= BRIDGE_DEGRADED_FAILURE_THRESHOLD && !_degradedSince) {
    _degradedSince = _lastErrorAt;
  }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/**
 * Send a single command string to the base station's POST /command endpoint.
 *
 * @param {string} cmd  Plain-text command token (e.g. "ESTOP", "PAUSE", "AUTO")
 * @returns {Promise<{ ok: boolean, status: number|null, body: string, error: string|null }>}
 */
async function sendCommand(cmd, options = {}) {
  if (!cmd || typeof cmd !== 'string') {
    return { ok: false, status: null, body: '', error: 'cmd must be a non-empty string' };
  }
  const cmdStr = cmd.trim().toUpperCase();
  const { waitForAck = false } = options;
  const expectedAck = waitForAck ? _expectedAckForCommand(cmdStr) : null;
  const baseline = expectedAck ? await _readLastLoRa() : null;
  const result = await _postWithRetry('/command', cmdStr);

  _lastCmd      = cmdStr;
  _lastCmdAt    = new Date().toISOString();
  _lastCmdError = result.ok ? null : (result.error ?? `HTTP ${result.status}`);
  _recordResult(result);

  if (!result.ok) {
    return result;
  }

  if (expectedAck) {
    const ack = await _waitForAck(expectedAck, {
      baselineRaw: baseline?.ok ? baseline.frame?.raw ?? null : null,
      wrappedOnly: true,
    });
    if (!ack.ok) {
      _lastCmdError = ack.error;
      _recordResult({ ok: false, error: ack.error });
      return { ok: false, status: 504, body: '', error: ack.error, ack };
    }
    return { ...result, ack };
  }

  return result;
}

/**
 * Push a full waypoint set to the STM32 via the LORA_WIRE text protocol.
 *
 * Validates:
 *   - points is a non-empty array
 *   - each point has numeric lat/lon/salt/brine
 *   - salt/brine are 0–100 integers
 *   - length ≤ LORA_WIRE.MAX_WAYPOINTS
 *
 * Sequence sent to base station POST /command:
 *   WPCLEAR
 *   WP:0:<lat>,<lon>,<salt>,<brine>   (LORA_WIRE.WP_INTERLINE_MS delay between each)
 *   …
 *   WPLOAD:<count>
 *
 * Sets wpPushState to 'committed' on full success, 'failed' on any error.
 *
 * @param {Array<{lat:number, lon:number, salt:number, brine:number}>} points
 * @returns {Promise<{ ok: boolean, sent: number, error: string|null }>}
 */
async function pushWaypoints(points) {
  // Validate
  if (!Array.isArray(points) || points.length === 0) {
    return _wpFail(0, 'points must be a non-empty array');
  }
  if (points.length > LORA_WIRE.MAX_WAYPOINTS) {
    return _wpFail(0, `Too many waypoints: ${points.length} > ${LORA_WIRE.MAX_WAYPOINTS}`);
  }
  for (let i = 0; i < points.length; i++) {
    const p = points[i];
    if (typeof p.lat !== 'number' || typeof p.lon !== 'number') {
      return _wpFail(0, `points[${i}]: lat and lon must be numbers`);
    }
    const salt  = Math.round(p.salt);
    const brine = Math.round(p.brine);
    if (salt < 0 || salt > 100 || brine < 0 || brine > 100) {
      return _wpFail(0, `points[${i}]: salt/brine must be 0–100`);
    }
  }

  _wpPushState = ARBITRATION.WP_PENDING;
  _wpPushError = null;
  const baselineClear = await _readLastLoRa();

  // 1. WPCLEAR
  {
    const r = await _postWithRetry('/command', LORA_WIRE.WP_CLEAR);
    _recordResult(r);
    if (!r.ok) return _wpFail(0, r.error ?? `WPCLEAR HTTP ${r.status}`);
    const ack = await _waitForAck(LORA_WIRE.WP_ACK_CLEAR, {
      baselineRaw: baselineClear.ok ? baselineClear.frame?.raw ?? null : null,
      wrappedOnly: true,
    });
    if (!ack.ok) return _wpFail(0, ack.error);
  }
  await _sleep(LORA_WIRE.WP_INTERLINE_MS);

  // 2. WP:<idx>:<lat>,<lon>,<salt>,<brine>
  for (let i = 0; i < points.length; i++) {
    const p     = points[i];
    const salt  = Math.round(p.salt);
    const brine = Math.round(p.brine);
    const line  = `${LORA_WIRE.WP_ADD}:${i}:${p.lat.toFixed(6)},${p.lon.toFixed(6)},${salt},${brine}`;
    const baseline = await _readLastLoRa();
    const r     = await _postWithRetry('/command', line);
    _recordResult(r);
    if (!r.ok) return _wpFail(i, r.error ?? `WP:${i} HTTP ${r.status}`);
    const ack = await _waitForAck(`${LORA_WIRE.WP_ACK_ADD}:${i}`, {
      baselineRaw: baseline.ok ? baseline.frame?.raw ?? null : null,
      wrappedOnly: true,
    });
    if (!ack.ok) return _wpFail(i, ack.error);
    if (i < points.length - 1) await _sleep(LORA_WIRE.WP_INTERLINE_MS);
  }
  await _sleep(LORA_WIRE.WP_INTERLINE_MS);

  // 3. WPLOAD:<count>
  {
    const baseline = await _readLastLoRa();
    const r = await _postWithRetry('/command', `${LORA_WIRE.WP_LOAD}:${points.length}`);
    _recordResult(r);
    if (!r.ok) return _wpFail(points.length, r.error ?? `WPLOAD HTTP ${r.status}`);
    const ack = await _waitForAck(`${LORA_WIRE.WP_ACK_LOAD}:${points.length}`, {
      baselineRaw: baseline.ok ? baseline.frame?.raw ?? null : null,
      wrappedOnly: true,
    });
    if (!ack.ok) return _wpFail(points.length, ack.error);
  }

  _wpPushState = ARBITRATION.WP_COMMITTED;
  _wpPushCount = points.length;
  _wpPushAt    = new Date().toISOString();
  _wpPushError = null;
  _recordResult({ ok: true });
  return { ok: true, sent: points.length, error: null };
}

/**
 * Reset WP push state to 'none'.  Call on mission abort/complete/reset.
 */
function resetWpState() {
  _wpPushState = ARBITRATION.WP_NONE;
  _wpPushError = null;
  _wpPushCount = 0;
  _wpPushAt    = null;
}

/**
 * Return a status snapshot suitable for GET /api/lora/status.
 */
function getStatus() {
  return {
    baseStationUrl: BASE_STATION_URL,
    wpPushState:    _wpPushState,
    wpPushCount:    _wpPushCount,
    wpPushAt:       _wpPushAt,
    wpPushError:    _wpPushError,
    lastCmd:        _lastCmd,
    lastCmdAt:      _lastCmdAt,
    lastCmdError:   _lastCmdError,
    lastSuccessAt:  _lastSuccessAt,
    lastErrorAt:    _lastErrorAt,
    consecutiveFailures: _consecutiveFailures,
    degraded: _consecutiveFailures >= BRIDGE_DEGRADED_FAILURE_THRESHOLD,
    degradedSince: _degradedSince,
    requestTimeoutMs: REQUEST_TIMEOUT_MS,
    ackWaitTimeoutMs: ACK_WAIT_TIMEOUT_MS,
    ackPollMs: ACK_POLL_MS,
    baseStation: {
      statusOk: _baseStationSnapshot.statusOk,
      statusCode: _baseStationSnapshot.statusCode,
      statusError: _baseStationSnapshot.statusError,
      state: _baseStationSnapshot.state,
      mode: _baseStationSnapshot.mode,
      queueDepth: _baseStationSnapshot.queueDepth,
      ackCount: _baseStationSnapshot.ackCount,
      lastAck: _baseStationSnapshot.lastAck,
      lastLoRa: _baseStationSnapshot.lastLoRa,
      refreshedAt: _baseStationSnapshot.refreshedAt,
    },
  };
}

async function refreshStatus() {
  await _readBaseStatus();
  return getStatus();
}

/**
 * Observe a successfully forwarded waypoint command so bridge state remains
 * consistent even when commands are sent individually via /command.
 *
 * @param {string} cmd
 */
function observeCommand(cmd) {
  if (!cmd || typeof cmd !== 'string') return;

  const normalized = cmd.trim().toUpperCase();
  if (!normalized) return;

  if (normalized === LORA_WIRE.WP_CLEAR) {
    _wpPushState = ARBITRATION.WP_PENDING;
    _wpPushCount = 0;
    _wpPushAt = null;
    _wpPushError = null;
    return;
  }

  if (normalized.startsWith(`${LORA_WIRE.WP_ADD}:`)) {
    const parts = normalized.split(':');
    const idx = Number(parts[1]);
    _wpPushState = ARBITRATION.WP_PENDING;
    _wpPushError = null;
    if (Number.isInteger(idx) && idx >= 0) {
      _wpPushCount = Math.max(_wpPushCount, idx + 1);
    }
    return;
  }

  if (normalized.startsWith(`${LORA_WIRE.WP_LOAD}:`)) {
    const count = Number(normalized.slice(`${LORA_WIRE.WP_LOAD}:`.length));
    _wpPushState = ARBITRATION.WP_COMMITTED;
    _wpPushError = null;
    _wpPushAt = new Date().toISOString();
    if (Number.isInteger(count) && count >= 0) {
      _wpPushCount = count;
    }
  }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------
function _wpFail(sent, error) {
  _wpPushState = ARBITRATION.WP_FAILED;
  _wpPushError = error;
  _recordResult({ ok: false, error });
  return { ok: false, sent, error };
}

// ---------------------------------------------------------------------------
// Exports
// ---------------------------------------------------------------------------
module.exports = { sendCommand, pushWaypoints, resetWpState, getStatus, refreshStatus, observeCommand };
