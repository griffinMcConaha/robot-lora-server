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
 * Sleep helper for inter-line delay between WP commands.
 * @param {number} ms
 */
function _sleep(ms) {
  return new Promise((r) => setTimeout(r, ms));
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
    if (!isQueueFull || attempt === QUEUE_FULL_RETRY_MAX) return r;
    await _sleep(QUEUE_FULL_RETRY_MS);
  }
  // unreachable
  return { ok: false, status: 503, body: 'queue_full', error: 'Max queue-full retries exceeded' };
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
async function sendCommand(cmd) {
  if (!cmd || typeof cmd !== 'string') {
    return { ok: false, status: null, body: '', error: 'cmd must be a non-empty string' };
  }
  const cmdStr = cmd.trim().toUpperCase();
  const result = await _post('/command', cmdStr);

  _lastCmd      = cmdStr;
  _lastCmdAt    = new Date().toISOString();
  _lastCmdError = result.ok ? null : (result.error ?? `HTTP ${result.status}`);

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

  // 1. WPCLEAR
  {
    const r = await _postWithRetry('/command', LORA_WIRE.WP_CLEAR);
    if (!r.ok) return _wpFail(0, r.error ?? `WPCLEAR HTTP ${r.status}`);
  }
  await _sleep(LORA_WIRE.WP_INTERLINE_MS);

  // 2. WP:<idx>:<lat>,<lon>,<salt>,<brine>
  for (let i = 0; i < points.length; i++) {
    const p     = points[i];
    const salt  = Math.round(p.salt);
    const brine = Math.round(p.brine);
    const line  = `${LORA_WIRE.WP_ADD}:${i}:${p.lat.toFixed(6)},${p.lon.toFixed(6)},${salt},${brine}`;
    const r     = await _postWithRetry('/command', line);
    if (!r.ok) return _wpFail(i, r.error ?? `WP:${i} HTTP ${r.status}`);
    if (i < points.length - 1) await _sleep(LORA_WIRE.WP_INTERLINE_MS);
  }
  await _sleep(LORA_WIRE.WP_INTERLINE_MS);

  // 3. WPLOAD:<count>
  {
    const r = await _postWithRetry('/command', `${LORA_WIRE.WP_LOAD}:${points.length}`);
    if (!r.ok) return _wpFail(points.length, r.error ?? `WPLOAD HTTP ${r.status}`);
  }

  _wpPushState = ARBITRATION.WP_COMMITTED;
  _wpPushCount = points.length;
  _wpPushAt    = new Date().toISOString();
  _wpPushError = null;
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
  };
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
  return { ok: false, sent, error };
}

// ---------------------------------------------------------------------------
// Exports
// ---------------------------------------------------------------------------
module.exports = { sendCommand, pushWaypoints, resetWpState, getStatus, observeCommand };
