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
const fs = require('fs');
const path = require('path');
const { Bonjour } = require('bonjour-service');
const { LORA_WIRE, ARBITRATION } = require('./contracts');
const { DATA_DIR } = require('./runtime_state');

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------
const BASE_STATION_PERSIST_PATH = path.join(DATA_DIR, 'base_station.json');
const DEFAULT_BASE_STATION_URL = _normalizeBaseStationUrl(process.env.BASE_STATION_URL ?? 'http://192.168.4.1');
const BASE_STATION_CANDIDATES = _buildBaseStationCandidates();
const BASE_STATION_MDNS_ENABLED = String(process.env.BASE_STATION_MDNS_ENABLED ?? '1') !== '0';
const BASE_STATION_MDNS_TYPE = String(process.env.BASE_STATION_MDNS_TYPE ?? 'http').trim() || 'http';
const BASE_STATION_MDNS_PROTOCOL = String(process.env.BASE_STATION_MDNS_PROTOCOL ?? 'tcp').trim() || 'tcp';
const BASE_STATION_MDNS_NAMES = _buildMdnsNameList();
const REQUEST_TIMEOUT_MS = Number(process.env.LORA_REQUEST_TIMEOUT_MS ?? 1500);
const TRANSIENT_RETRY_MAX = Number(process.env.LORA_TRANSIENT_RETRY_MAX ?? 2);
const TRANSIENT_RETRY_MS = Number(process.env.LORA_TRANSIENT_RETRY_MS ?? 100);
const BRIDGE_DEGRADED_FAILURE_THRESHOLD = Number(process.env.BRIDGE_DEGRADED_FAILURE_THRESHOLD ?? 3);
const ACK_WAIT_TIMEOUT_MS = Number(process.env.LORA_ACK_WAIT_TIMEOUT_MS ?? 3500);
const ACK_POLL_MS = Number(process.env.LORA_ACK_POLL_MS ?? 40);
const ACK_REQUIRED = String(process.env.LORA_ACK_REQUIRED ?? '0') === '1';
const BASE_STATUS_REFRESH_INTERVAL_MS = Number(process.env.BASE_STATUS_REFRESH_INTERVAL_MS ?? 750);

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
let _selectedBaseStationUrl = _loadPreferredBaseStationUrl();
let _statusRefreshInFlight = false;
let _bonjour = null;
let _mdnsBrowser = null;
let _mdnsServices = new Map();
let _baseStationSnapshot = {
  statusOk: false,
  statusCode: null,
  statusError: null,
  statusVersion: null,
  state: null,
  mode: null,
  radioMode: null,
  wifiLinkState: null,
  loraLinkState: null,
  queueDepth: null,
  lastCmdId: null,
  lastCmdStatus: null,
  ackCount: null,
  lastAck: null,
  lastAckParsed: null,
  lastLoRa: null,
  discoverySource: 'configured',
  mdnsEnabled: BASE_STATION_MDNS_ENABLED,
  mdnsNames: BASE_STATION_MDNS_NAMES,
  mdnsServices: [],
  refreshedAt: null,
  selectedUrl: _selectedBaseStationUrl,
  attemptedUrls: [],
};

function _normalizeBaseStationUrl(value) {
  return String(value ?? '').trim().replace(/\/$/, '');
}

function _buildBaseStationCandidates() {
  const raw = [
    process.env.BASE_STATION_URL,
    ...(typeof process.env.BASE_STATION_CANDIDATES === 'string'
      ? process.env.BASE_STATION_CANDIDATES.split(',')
      : []),
    'http://192.168.4.1',
    'http://base-station.local',
  ];

  const normalized = raw
    .map(_normalizeBaseStationUrl)
    .filter(Boolean);

  return Array.from(new Set(normalized));
}

function _buildMdnsNameList() {
  const raw = [
    ...(typeof process.env.BASE_STATION_MDNS_NAMES === 'string'
      ? process.env.BASE_STATION_MDNS_NAMES.split(',')
      : []),
    'base-station',
    'base-station.local',
    'saltrobot-base',
    'saltrobot-base.local',
    'salt-robot-base',
    'salt-robot-base.local',
  ];

  return Array.from(new Set(
    raw
      .map((value) => String(value ?? '').trim().toLowerCase())
      .filter(Boolean),
  ));
}

function _loadPreferredBaseStationUrl() {
  try {
    if (!fs.existsSync(BASE_STATION_PERSIST_PATH)) {
      return DEFAULT_BASE_STATION_URL;
    }
    const raw = JSON.parse(fs.readFileSync(BASE_STATION_PERSIST_PATH, 'utf8'));
    const preferred = _normalizeBaseStationUrl(raw?.selectedUrl);
    return preferred || DEFAULT_BASE_STATION_URL;
  } catch {
    return DEFAULT_BASE_STATION_URL;
  }
}

function _savePreferredBaseStationUrl(url) {
  try {
    fs.writeFileSync(
      BASE_STATION_PERSIST_PATH,
      `${JSON.stringify({ selectedUrl: url, savedAt: new Date().toISOString() }, null, 2)}\n`,
      'utf8',
    );
  } catch {
    // Best-effort persistence only.
  }
}

function _setSelectedBaseStationUrl(url) {
  const normalized = _normalizeBaseStationUrl(url);
  if (!normalized || normalized === _selectedBaseStationUrl) {
    return;
  }
  _selectedBaseStationUrl = normalized;
  _baseStationSnapshot = {
    ..._baseStationSnapshot,
    selectedUrl: normalized,
  };
  _savePreferredBaseStationUrl(normalized);
}

function _normalizeHostToken(value) {
  return String(value ?? '')
    .trim()
    .toLowerCase()
    .replace(/\.$/, '');
}

function _hostMatchesMdnsNames(host) {
  const normalizedHost = _normalizeHostToken(host);
  if (!normalizedHost) return false;
  return BASE_STATION_MDNS_NAMES.some((candidate) => {
    const normalizedCandidate = _normalizeHostToken(candidate);
    if (!normalizedCandidate) return false;
    return normalizedHost === normalizedCandidate
      || normalizedHost === `${normalizedCandidate}.local`
      || normalizedHost.startsWith(`${normalizedCandidate}.`);
  });
}

function _serviceMatchesMdns(service) {
  if (!service || typeof service !== 'object') return false;
  if (_hostMatchesMdnsNames(service.host)) return true;
  if (_hostMatchesMdnsNames(service.name)) return true;
  if (Array.isArray(service.addresses) && service.addresses.some(_hostMatchesMdnsNames)) return true;
  return false;
}

function _serviceToBaseUrls(service) {
  if (!service || typeof service !== 'object') return [];

  const protocol = String(service.protocol || 'http').toLowerCase() === 'https' ? 'https' : 'http';
  const port = Number(service.port);
  const hosts = [
    service.host,
    ...(Array.isArray(service.addresses) ? service.addresses : []),
  ]
    .map((value) => String(value ?? '').trim())
    .filter(Boolean);

  return Array.from(new Set(hosts.map((host) => {
    const normalizedHost = host.replace(/\.$/, '');
    if (Number.isFinite(port) && port > 0 && port !== 80 && port !== 443) {
      return _normalizeBaseStationUrl(`${protocol}://${normalizedHost}:${port}`);
    }
    return _normalizeBaseStationUrl(`${protocol}://${normalizedHost}`);
  })));
}

function _recordMdnsService(service) {
  if (!_serviceMatchesMdns(service)) return;
  const urls = _serviceToBaseUrls(service);
  if (!urls.length) return;

  _mdnsServices.set(urls[0], {
    name: service.name ?? null,
    host: service.host ?? null,
    port: Number.isFinite(Number(service.port)) ? Number(service.port) : null,
    type: service.type ?? null,
    protocol: service.protocol ?? null,
    addresses: Array.isArray(service.addresses) ? service.addresses.slice(0, 4) : [],
    urls,
    updatedAt: new Date().toISOString(),
  });

  _baseStationSnapshot = {
    ..._baseStationSnapshot,
    mdnsServices: Array.from(_mdnsServices.values()),
  };
}

function _removeMdnsService(service) {
  const urls = _serviceToBaseUrls(service);
  for (const url of urls) {
    _mdnsServices.delete(url);
  }
  _baseStationSnapshot = {
    ..._baseStationSnapshot,
    mdnsServices: Array.from(_mdnsServices.values()),
  };
}

function _startMdnsDiscovery() {
  if (!BASE_STATION_MDNS_ENABLED || _bonjour || typeof Bonjour !== 'function') {
    return;
  }

  try {
    _bonjour = new Bonjour();
    _mdnsBrowser = _bonjour.find(
      { type: BASE_STATION_MDNS_TYPE, protocol: BASE_STATION_MDNS_PROTOCOL },
      (service) => _recordMdnsService(service),
    );
    if (_mdnsBrowser?.on) {
      _mdnsBrowser.on('up', _recordMdnsService);
      _mdnsBrowser.on('down', _removeMdnsService);
    }
    if (_mdnsBrowser?.start) {
      _mdnsBrowser.start();
    }
  } catch (error) {
    _baseStationSnapshot = {
      ..._baseStationSnapshot,
      mdnsEnabled: false,
      statusError: _baseStationSnapshot.statusError ?? `mDNS unavailable: ${error.message}`,
    };
  }
}

function _candidateUrls() {
  const urls = [
    _selectedBaseStationUrl,
    ...Array.from(_mdnsServices.values()).flatMap((service) => service.urls ?? []),
    ...BASE_STATION_CANDIDATES,
    DEFAULT_BASE_STATION_URL,
  ]
    .map(_normalizeBaseStationUrl)
    .filter(Boolean);

  return Array.from(new Set(urls));
}

function _discoverySourceForUrl(url) {
  if (_mdnsServices.has(url)) {
    return 'mdns';
  }
  if (url === DEFAULT_BASE_STATION_URL) {
    return 'default';
  }
  return 'configured';
}

// ---------------------------------------------------------------------------
// Low-level HTTP helper
// ---------------------------------------------------------------------------

/**
 * POST text/plain body to BASE_STATION_URL/command.
 * Returns { ok: boolean, status: number|null, body: string, error: string|null }.
 * Never throws.
 */
async function _post(path, body, extraHeaders = {}, baseUrl = _selectedBaseStationUrl) {
  return new Promise((resolve) => {
    const bodyBuf = Buffer.from(body, 'utf8');
    const urlStr  = `${baseUrl}${path}`;

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
        'x-discovery-source': _baseStationSnapshot.discoverySource ?? 'configured',
        ...extraHeaders,
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
async function _get(path, baseUrl = _selectedBaseStationUrl) {
  return new Promise((resolve) => {
    const urlStr = `${baseUrl}${path}`;

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

async function _postWithFailover(path, body, extraHeaders = {}) {
  const attemptedUrls = [];
  let lastResult = { ok: false, status: null, body: '', error: 'No base station candidates available' };

  for (const candidateUrl of _candidateUrls()) {
    attemptedUrls.push(candidateUrl);
    const result = await _post(path, body, extraHeaders, candidateUrl);
    lastResult = result;
    if (!result.ok) {
      continue;
    }

    _setSelectedBaseStationUrl(candidateUrl);
    _baseStationSnapshot = {
      ..._baseStationSnapshot,
      selectedUrl: candidateUrl,
      attemptedUrls,
      discoverySource: _discoverySourceForUrl(candidateUrl),
      statusError: null,
      refreshedAt: new Date().toISOString(),
    };
    return {
      ...result,
      baseUrl: candidateUrl,
      attemptedUrls,
    };
  }

  return {
    ...lastResult,
    attemptedUrls,
  };
}

async function _getWithFailover(path) {
  const attemptedUrls = [];
  let lastResult = { ok: false, status: null, body: '', error: 'No base station candidates available' };

  for (const candidateUrl of _candidateUrls()) {
    attemptedUrls.push(candidateUrl);
    const result = await _get(path, candidateUrl);
    lastResult = result;
    if (!result.ok) {
      continue;
    }

    _setSelectedBaseStationUrl(candidateUrl);
    _baseStationSnapshot = {
      ..._baseStationSnapshot,
      selectedUrl: candidateUrl,
      attemptedUrls,
      discoverySource: _discoverySourceForUrl(candidateUrl),
      statusError: null,
      refreshedAt: new Date().toISOString(),
    };
    return {
      ...result,
      baseUrl: candidateUrl,
      attemptedUrls,
    };
  }

  return {
    ...lastResult,
    attemptedUrls,
  };
}

function _applyBaseStatusSnapshot(payload, meta = {}) {
  if (!payload || typeof payload !== 'object') {
    _baseStationSnapshot = {
      ..._baseStationSnapshot,
      statusOk: false,
      statusCode: meta.status ?? null,
      statusError: meta.error ?? 'Invalid base station status payload',
      discoverySource: meta.discoverySource ?? _baseStationSnapshot.discoverySource,
      mdnsEnabled: BASE_STATION_MDNS_ENABLED,
      mdnsNames: BASE_STATION_MDNS_NAMES,
      mdnsServices: Array.from(_mdnsServices.values()),
      refreshedAt: new Date().toISOString(),
      attemptedUrls: Array.isArray(meta.attemptedUrls) ? meta.attemptedUrls : _baseStationSnapshot.attemptedUrls,
    };
    return _baseStationSnapshot;
  }

  const payloadLastLoRa = typeof payload.last_lora === 'string' ? payload.last_lora.trim() : null;
  const payloadLastAck = typeof payload.last_ack === 'string' ? payload.last_ack.trim() : null;
  const derivedLastAck = payloadLastAck || (_unwrapAckFrame(payloadLastLoRa)?.ack ? payloadLastLoRa : null);
  const parsedAck = derivedLastAck ? _parseAckDetails(derivedLastAck) : null;
  const derivedLoraLinkState = typeof payload.lora_link_state === 'string'
    ? payload.lora_link_state
    : (payloadLastLoRa ? 'rx' : _baseStationSnapshot.loraLinkState);

  _baseStationSnapshot = {
    ..._baseStationSnapshot,
    statusOk: Boolean(meta.ok ?? true),
    statusCode: meta.status ?? null,
    statusError: meta.error ?? null,
    statusVersion: Number.isFinite(Number(payload.status_version)) ? Number(payload.status_version) : _baseStationSnapshot.statusVersion,
    state: typeof payload.state === 'string' ? payload.state : _baseStationSnapshot.state,
    mode: typeof payload.mode === 'string' ? payload.mode : _baseStationSnapshot.mode,
    radioMode: typeof payload.radio_mode === 'string'
      ? payload.radio_mode
      : (typeof payload.radioMode === 'string' ? payload.radioMode : _baseStationSnapshot.radioMode),
    wifiLinkState: typeof payload.wifi_link_state === 'string' ? payload.wifi_link_state : _baseStationSnapshot.wifiLinkState,
    loraLinkState: derivedLoraLinkState,
    queueDepth: Number.isFinite(Number(payload.queue_depth)) ? Number(payload.queue_depth) : _baseStationSnapshot.queueDepth,
    lastCmdId: typeof payload.last_cmd_id === 'string' ? payload.last_cmd_id.trim() : _baseStationSnapshot.lastCmdId,
    lastCmdStatus: typeof payload.last_cmd_status === 'string' ? payload.last_cmd_status.trim() : _baseStationSnapshot.lastCmdStatus,
    ackCount: Number.isFinite(Number(payload.ack_count)) ? Number(payload.ack_count) : _baseStationSnapshot.ackCount,
    lastAck: derivedLastAck ?? _baseStationSnapshot.lastAck,
    lastAckParsed: parsedAck ?? _baseStationSnapshot.lastAckParsed,
    lastLoRa: payloadLastLoRa ?? _baseStationSnapshot.lastLoRa,
    discoverySource: meta.discoverySource ?? _baseStationSnapshot.discoverySource,
    mdnsEnabled: BASE_STATION_MDNS_ENABLED,
    mdnsNames: BASE_STATION_MDNS_NAMES,
    mdnsServices: Array.from(_mdnsServices.values()),
    refreshedAt: new Date().toISOString(),
    selectedUrl: meta.selectedUrl ?? _selectedBaseStationUrl,
    attemptedUrls: Array.isArray(meta.attemptedUrls) ? meta.attemptedUrls : _baseStationSnapshot.attemptedUrls,
  };
  return _baseStationSnapshot;
}

async function _readBaseStatus() {
  const attemptedUrls = [];

  for (const candidateUrl of _candidateUrls()) {
    attemptedUrls.push(candidateUrl);
    const result = await _get('/status', candidateUrl);
    if (!result.ok) {
      continue;
    }

    let parsed;
    try {
      parsed = JSON.parse(result.body);
    } catch {
      continue;
    }

    _setSelectedBaseStationUrl(candidateUrl);
    const discoverySource = _discoverySourceForUrl(candidateUrl);
    return {
      ok: true,
      status: result.status ?? null,
      snapshot: _applyBaseStatusSnapshot(parsed, {
        ok: true,
        status: result.status ?? null,
        selectedUrl: candidateUrl,
        attemptedUrls,
        discoverySource,
      }),
    };
  }

  _baseStationSnapshot = {
    ..._baseStationSnapshot,
    statusOk: false,
    statusCode: null,
    statusError: 'Unable to reach any configured base station endpoint',
    mdnsEnabled: BASE_STATION_MDNS_ENABLED,
    mdnsNames: BASE_STATION_MDNS_NAMES,
    mdnsServices: Array.from(_mdnsServices.values()),
    refreshedAt: new Date().toISOString(),
    attemptedUrls,
    selectedUrl: _selectedBaseStationUrl,
  };
  return { ok: false, error: _baseStationSnapshot.statusError, status: null, snapshot: _baseStationSnapshot };
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

function _parseAckDetails(text) {
  const frame = _unwrapAckFrame(text);
  if (!frame?.ack) return null;

  const ack = frame.ack;
  const details = {
    raw: frame.raw,
    wrapped: frame.wrapped,
    ack,
    category: 'unknown',
    source: 'unknown',
    command: null,
    waypointIndex: null,
    waypointCount: null,
    robotState: null,
  };

  if (ack.startsWith('ACK:S:')) {
    details.category = 'gateway_frame';
    details.source = 'gateway';
    return details;
  }

  if (ack.startsWith('ACK:STATE:')) {
    details.category = 'robot_state';
    details.source = 'robot';
    details.robotState = ack.slice('ACK:STATE:'.length) || null;
    details.command = details.robotState;
    return details;
  }

  if (ack.startsWith('ACK:CMD:')) {
    details.category = 'robot_command';
    details.source = 'robot';
    details.command = ack.slice('ACK:CMD:'.length) || null;
    return details;
  }

  if (ack === LORA_WIRE.WP_ACK_CLEAR) {
    details.category = 'waypoint_clear';
    details.source = 'robot';
    details.command = LORA_WIRE.WP_CLEAR;
    return details;
  }

  if (ack.startsWith(`${LORA_WIRE.WP_ACK_ADD}:`)) {
    details.category = 'waypoint_add';
    details.source = 'robot';
    details.command = LORA_WIRE.WP_ADD;
    const idx = Number(ack.slice(`${LORA_WIRE.WP_ACK_ADD}:`.length));
    details.waypointIndex = Number.isFinite(idx) ? idx : null;
    return details;
  }

  if (ack.startsWith(`${LORA_WIRE.WP_ACK_LOAD}:`)) {
    details.category = 'waypoint_load';
    details.source = 'robot';
    details.command = LORA_WIRE.WP_LOAD;
    const count = Number(ack.slice(`${LORA_WIRE.WP_ACK_LOAD}:`.length));
    details.waypointCount = Number.isFinite(count) ? count : null;
    return details;
  }

  return details;
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
  const result = await _getWithFailover('/last_lora');
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
  const { baselineRaw = null, wrappedOnly = true, timeoutMs = ACK_WAIT_TIMEOUT_MS, commandId = null } = options;
  const startedAt = Date.now();
  let lastSeenRaw = baselineRaw;
  let lastSeenAck = null;
  let lastSeenCommandStatus = null;

  while ((Date.now() - startedAt) < timeoutMs) {
    const statusSnapshot = await _readBaseStatus();
    if (statusSnapshot.ok) {
      const snapshot = statusSnapshot.snapshot ?? {};
      lastSeenCommandStatus = snapshot.lastCmdStatus ?? lastSeenCommandStatus;
      if (
        commandId &&
        snapshot.lastCmdId === commandId &&
        snapshot.lastCmdStatus === 'acknowledged'
      ) {
        return {
          ok: true,
          ack: expectedAck,
          raw: snapshot.lastAck ?? snapshot.lastLoRa ?? null,
          source: 'status.command',
        };
      }
    }
    if (statusSnapshot.ok && typeof statusSnapshot.snapshot?.lastAck === 'string' && statusSnapshot.snapshot.lastAck) {
      const frame = _unwrapAckFrame(statusSnapshot.snapshot.lastAck);
      if (frame?.ack) {
        lastSeenRaw = frame.raw;
        lastSeenAck = frame.ack;
        if (frame.ack === expectedAck && frame.raw !== baselineRaw) {
          return { ok: true, ack: frame.ack, raw: frame.raw, source: 'status' };
        }
      }
    }

    const snapshot = await _readLastLoRa();
    if (snapshot.ok && snapshot.frame?.ack) {
      const { frame } = snapshot;
      lastSeenRaw = frame.raw;
      lastSeenAck = frame.ack;

      if (frame.ack === expectedAck && frame.raw !== baselineRaw) {
        return { ok: true, ack: frame.ack, raw: frame.raw, source: 'last_lora' };
      }
    }

    await _sleep(ACK_POLL_MS);
  }

  if (!ACK_REQUIRED) {
    return {
      ok: true,
      ack: expectedAck,
      raw: lastSeenRaw,
      source: 'timeout-soft',
      warning: `Timed out waiting for ${expectedAck}`,
      lastSeenAck,
      lastSeenCommandStatus,
    };
  }

  return {
    ok: false,
    error: `Timed out waiting for ${expectedAck}`,
    lastSeenRaw,
    lastSeenAck,
    lastSeenCommandStatus,
  };
}

// Phase D: retry budget and delay when the base station returns 503 queue_full.
// The base station now returns 503 with body 'queue_full' instead of silently
// dropping commands.  We back off and retry so no WP frame is lost.
const QUEUE_FULL_RETRY_MAX = 6;
const QUEUE_FULL_RETRY_MS  = 120;

/**
 * Like _post() but retries up to QUEUE_FULL_RETRY_MAX times when the base
 * station responds 503 "queue_full" (Phase D back-pressure).
 * All other errors are returned immediately without retrying.
 *
 * @param {string} path
 * @param {string} body
 * @returns {Promise<{ ok, status, body, error }>}
 */
async function _postWithRetry(path, body, extraHeaders = {}) {
  for (let attempt = 0; attempt <= QUEUE_FULL_RETRY_MAX; attempt++) {
    const r = await _postWithFailover(path, body, extraHeaders);
    if (r.ok) return r;
    const isQueueFull = r.status === 503 && r.body === 'queue_full';
    const isTransient =
      isQueueFull
      || r.status == null
      || r.status >= 500
      || (typeof r.error === 'string' && r.error.toLowerCase().includes('timeout'));

    const maxRetries = isQueueFull ? QUEUE_FULL_RETRY_MAX : TRANSIENT_RETRY_MAX;
    const retryDelayMs = isQueueFull ? QUEUE_FULL_RETRY_MS : TRANSIENT_RETRY_MS;

    if (isTransient && r.status == null) {
      await _readBaseStatus();
    }

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
  const { waitForAck = false, commandId = null, commandSource = null } = options;
  const expectedAck = waitForAck ? _expectedAckForCommand(cmdStr) : null;
  const baseline = expectedAck ? await _readLastLoRa() : null;
  const headers = {};
  if (typeof commandId === 'string' && commandId.trim()) {
    headers['x-command-id'] = commandId.trim();
  }
  if (typeof commandSource === 'string' && commandSource.trim()) {
    headers['x-command-source'] = commandSource.trim();
  }
  const result = await _postWithRetry('/command', cmdStr, headers);

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
      commandId: typeof commandId === 'string' && commandId.trim() ? commandId.trim() : null,
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
    baseStationUrl: _selectedBaseStationUrl,
    baseStationCandidates: _candidateUrls(),
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
      statusVersion: _baseStationSnapshot.statusVersion,
      state: _baseStationSnapshot.state,
      mode: _baseStationSnapshot.mode,
      radioMode: _baseStationSnapshot.radioMode,
      wifiLinkState: _baseStationSnapshot.wifiLinkState,
      loraLinkState: _baseStationSnapshot.loraLinkState,
      queueDepth: _baseStationSnapshot.queueDepth,
      lastCmdId: _baseStationSnapshot.lastCmdId,
      lastCmdStatus: _baseStationSnapshot.lastCmdStatus,
      ackCount: _baseStationSnapshot.ackCount,
      lastAck: _baseStationSnapshot.lastAck,
      lastAckParsed: _baseStationSnapshot.lastAckParsed,
      lastLoRa: _baseStationSnapshot.lastLoRa,
      discoverySource: _baseStationSnapshot.discoverySource,
      mdnsEnabled: _baseStationSnapshot.mdnsEnabled,
      mdnsNames: _baseStationSnapshot.mdnsNames,
      mdnsServices: _baseStationSnapshot.mdnsServices,
      refreshedAt: _baseStationSnapshot.refreshedAt,
      selectedUrl: _baseStationSnapshot.selectedUrl,
      attemptedUrls: _baseStationSnapshot.attemptedUrls,
    },
  };
}

async function refreshStatus() {
  await _readBaseStatus();
  return getStatus();
}

async function _refreshLoop() {
  if (_statusRefreshInFlight) return;
  _statusRefreshInFlight = true;
  try {
    await _readBaseStatus();
  } finally {
    _statusRefreshInFlight = false;
  }
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

function restoreStatus(snapshot = {}) {
  if (!snapshot || typeof snapshot !== 'object') return;

  if (typeof snapshot.wpPushState === 'string') _wpPushState = snapshot.wpPushState;
  if (Number.isFinite(Number(snapshot.wpPushCount))) _wpPushCount = Number(snapshot.wpPushCount);
  if (typeof snapshot.wpPushAt === 'string' || snapshot.wpPushAt === null) _wpPushAt = snapshot.wpPushAt ?? null;
  if (typeof snapshot.wpPushError === 'string' || snapshot.wpPushError === null) _wpPushError = snapshot.wpPushError ?? null;
  if (typeof snapshot.lastCmd === 'string' || snapshot.lastCmd === null) _lastCmd = snapshot.lastCmd ?? null;
  if (typeof snapshot.lastCmdAt === 'string' || snapshot.lastCmdAt === null) _lastCmdAt = snapshot.lastCmdAt ?? null;
  if (typeof snapshot.lastCmdError === 'string' || snapshot.lastCmdError === null) _lastCmdError = snapshot.lastCmdError ?? null;
  if (typeof snapshot.lastSuccessAt === 'string' || snapshot.lastSuccessAt === null) _lastSuccessAt = snapshot.lastSuccessAt ?? null;
  if (typeof snapshot.lastErrorAt === 'string' || snapshot.lastErrorAt === null) _lastErrorAt = snapshot.lastErrorAt ?? null;
  if (Number.isFinite(Number(snapshot.consecutiveFailures))) _consecutiveFailures = Number(snapshot.consecutiveFailures);
  if (typeof snapshot.degradedSince === 'string' || snapshot.degradedSince === null) _degradedSince = snapshot.degradedSince ?? null;

  if (snapshot.baseStation && typeof snapshot.baseStation === 'object') {
    _baseStationSnapshot = {
      ..._baseStationSnapshot,
      ...snapshot.baseStation,
    };
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
module.exports = { sendCommand, pushWaypoints, resetWpState, getStatus, refreshStatus, observeCommand, restoreStatus, parseAckDetails: _parseAckDetails };

_startMdnsDiscovery();

if (BASE_STATUS_REFRESH_INTERVAL_MS > 0) {
  const timer = setInterval(() => {
    _refreshLoop().catch(() => {
      // Keep background reconnect best-effort.
    });
  }, BASE_STATUS_REFRESH_INTERVAL_MS);
  if (typeof timer.unref === 'function') {
    timer.unref();
  }
  _refreshLoop().catch(() => {
    // Startup probe is best-effort.
  });
}
