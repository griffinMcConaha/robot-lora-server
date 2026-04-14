const express = require('express');
const cors = require('cors');
const http = require('http');
const fs = require('fs');
const path = require('path');
const { WebSocketServer } = require('ws');
const {
  buildCoverageMap,
  resolveCoverageFrame,
  markCoverage,
  coverageStats,
  worldToGrid,
  withinGrid,
  gridCellPolygon,
} = require('./coverage');
const { latLonToLocal, localToLatLon } = require('./geo');
const { findPath, findCoveragePath, buildCoverageArrows } = require('./pathing');
const {
  createOperatorState,
  telemetryAgeMs: supervisionTelemetryAgeMs,
  isTelemetryStale: supervisionIsTelemetryStale,
  buildAllowedActions: buildSupervisionAllowedActions,
  buildAlerts: buildSupervisionAlerts,
  buildOperatorWorkflows: buildSupervisionWorkflows,
} = require('./supervision');
const {
  ROBOT_STATE,
  CONNECTION_STATE,
  COMMAND_STATUS,
  COMMAND_TRANSPORT_STAGE,
  CMD,
  CMD_ALIASES,
  FAULT_CODE,
  FAULT_ACTION,
  MISSION_STATE,
  EVENT_TYPE,
  WS_EVENT,
  API,
  LORA_WIRE,
} = require('./contracts');
const db      = require('./db');
const mission = require('./mission');
const bridge  = require('./lora_bridge');
const {
  RUNTIME_STATE_PATH,
  loadRuntimeState,
  saveRuntimeState,
} = require('./runtime_state');

// Publish mission transitions over WebSocket
mission.onTransition((m) => publish(WS_EVENT.MISSION_UPDATED, m));

const app = express();
app.use(cors());
app.use(express.json({ limit: '1mb' }));
app.use(express.text({ type: 'text/*', limit: '256kb' }));
app.use((err, req, res, next) => {
  if (err?.type === 'entity.parse.failed' && req?.path === API.TELEMETRY) {
    metrics.telemetryRejected += 1;
    const rawSnippet = typeof err.body === 'string' ? err.body.slice(0, 240) : String(err.body ?? '').slice(0, 240);
    console.warn(`[telemetry] malformed json rejected by parser: ${rawSnippet}`);
    return res.status(202).json({ ok: false, ignored: true, error: 'Malformed telemetry JSON' });
  }
  return next(err);
});
// Serve dashboard UI
app.use(express.static(path.join(__dirname, '..', 'public')));

const server = http.createServer(app);
const wss = new WebSocketServer({ server });

const SUPERVISION_TELEMETRY_STALE_MS = Number(process.env.SUPERVISION_TELEMETRY_STALE_MS ?? 30000);
const TELEMETRY_FAILSAFE_ENABLED = String(process.env.TELEMETRY_FAILSAFE_ENABLED ?? '1') !== '0';
const TELEMETRY_FAILSAFE_ACTION = String(process.env.TELEMETRY_FAILSAFE_ACTION ?? CMD.ESTOP).trim().toUpperCase();
const TELEMETRY_FAILSAFE_COOLDOWN_MS = Number(process.env.TELEMETRY_FAILSAFE_COOLDOWN_MS ?? Math.max(SUPERVISION_TELEMETRY_STALE_MS, 5000));
const SAFETY_MONITOR_INTERVAL_MS = Number(process.env.SAFETY_MONITOR_INTERVAL_MS ?? 500);
const GEOFENCE_FAILSAFE_ENABLED = String(process.env.GEOFENCE_FAILSAFE_ENABLED ?? '1') !== '0';
const GEOFENCE_FAILSAFE_ACTION = String(process.env.GEOFENCE_FAILSAFE_ACTION ?? CMD.ESTOP).trim().toUpperCase();
const GEOFENCE_FAILSAFE_COOLDOWN_MS = Number(process.env.GEOFENCE_FAILSAFE_COOLDOWN_MS ?? 5000);
const GEOFENCE_TOLERANCE_M = Number(process.env.GEOFENCE_TOLERANCE_M ?? 2.0);
const GEOFENCE_DEMO_TOLERANCE_M = Number(process.env.GEOFENCE_DEMO_TOLERANCE_M ?? 6.0);
const GEOFENCE_DEMO_SOFT_ACTION = String(process.env.GEOFENCE_DEMO_SOFT_ACTION ?? '1') !== '0';
const EVENT_RETENTION_DAYS = Number(process.env.EVENT_RETENTION_DAYS ?? 30);
const EVENT_RETENTION_PRUNE_INTERVAL_MS = Number(process.env.EVENT_RETENTION_PRUNE_INTERVAL_MS ?? 3600000);
const MAX_INPUT_AREA_CELLS = Number(process.env.MAX_INPUT_AREA_CELLS ?? 60000);
const MIN_INPUT_AREA_CELL_SIZE_M = Number(process.env.MIN_INPUT_AREA_CELL_SIZE_M ?? 0.5);
const MAX_INPUT_AREA_CELL_SIZE_M = Number(process.env.MAX_INPUT_AREA_CELL_SIZE_M ?? 8.0);
const COVERAGE_MARK_RADIUS_M = Number(process.env.COVERAGE_MARK_RADIUS_M ?? 0.5);
const REQUEST_RATE_LIMIT_WINDOW_MS = Number(process.env.REQUEST_RATE_LIMIT_WINDOW_MS ?? 60000);
const COMMAND_RATE_LIMIT_PER_WINDOW = Number(process.env.COMMAND_RATE_LIMIT_PER_WINDOW ?? 20000);
const TELEMETRY_RATE_LIMIT_PER_WINDOW = Number(process.env.TELEMETRY_RATE_LIMIT_PER_WINDOW ?? 6000);
const COMMAND_STATE_CONFIRM_TIMEOUT_MS = Number(process.env.COMMAND_STATE_CONFIRM_TIMEOUT_MS ?? 15000);
const COMMAND_STATE_CONFIRM_POLL_MS = Number(process.env.COMMAND_STATE_CONFIRM_POLL_MS ?? 150);
const MISSION_AUTO_CONFIRM_TIMEOUT_MS = Number(process.env.MISSION_AUTO_CONFIRM_TIMEOUT_MS ?? 18000);
const GPS_READY_MIN_SAT = Number(process.env.GPS_READY_MIN_SAT ?? 5);
const GPS_READY_MAX_HDOP = Number(process.env.GPS_READY_MAX_HDOP ?? 3.0);
const GPS_FAILSAFE_ENABLED = String(process.env.GPS_FAILSAFE_ENABLED ?? '1') !== '0';
const GPS_FAILSAFE_ACTION = String(process.env.GPS_FAILSAFE_ACTION ?? CMD.PAUSE).trim().toUpperCase();
const GPS_FAILSAFE_COOLDOWN_MS = Number(process.env.GPS_FAILSAFE_COOLDOWN_MS ?? 5000);
const DEMO_MIN_SPOT_DISTANCE_M = Number(process.env.DEMO_MIN_SPOT_DISTANCE_M ?? 0.20);
const DEMO_MIN_LANE_WIDTH_M = Number(process.env.DEMO_MIN_LANE_WIDTH_M ?? 0.90);
const DEMO_OBSTACLE_POLICY_ENABLED = String(process.env.DEMO_OBSTACLE_POLICY_ENABLED ?? '1') !== '0';
const DEMO_OBSTACLE_STOP_CM = Number(process.env.DEMO_OBSTACLE_STOP_CM ?? 70);
const DEMO_OBSTACLE_SIDESTEP_CM = Number(process.env.DEMO_OBSTACLE_SIDESTEP_CM ?? 120);
const DEMO_OBSTACLE_COOLDOWN_MS = Number(process.env.DEMO_OBSTACLE_COOLDOWN_MS ?? 1800);
const DEMO_OBSTACLE_SIDESTEP_MS = Number(process.env.DEMO_OBSTACLE_SIDESTEP_MS ?? 320);
const RUNTIME_STATE_SAVE_DEBOUNCE_MS = Number(process.env.RUNTIME_STATE_SAVE_DEBOUNCE_MS ?? 750);
const RUNTIME_STATE_TRAIL_LIMIT = Number(process.env.RUNTIME_STATE_TRAIL_LIMIT ?? 300);
const COMMAND_HISTORY_LIMIT = Number(process.env.COMMAND_HISTORY_LIMIT ?? 50);
const REMOTE_BASE_STATION_STALE_MS = Number(process.env.REMOTE_BASE_STATION_STALE_MS ?? 15000);
const REMOTE_COMMAND_MAX_QUEUE = Number(process.env.REMOTE_COMMAND_MAX_QUEUE ?? Math.max(LORA_WIRE.MAX_WAYPOINTS + 8, 1024));
const REMOTE_COMMAND_STALE_MS = Number(process.env.REMOTE_COMMAND_STALE_MS ?? 300000);
const REMOTE_COMMAND_ACK_RETENTION_MS = Number(process.env.REMOTE_COMMAND_ACK_RETENTION_MS ?? 15000);
const REMOTE_COMMAND_LEASE_RETRY_MS = Number(process.env.REMOTE_COMMAND_LEASE_RETRY_MS ?? 180);
const REMOTE_COMMAND_MAX_MOTION_LEASES = Number(process.env.REMOTE_COMMAND_MAX_MOTION_LEASES ?? 12);
const REMOTE_COMMAND_MAX_MOTION_LEASE_AGE_MS = Number(process.env.REMOTE_COMMAND_MAX_MOTION_LEASE_AGE_MS ?? 3000);
const BASE_STATION_LORA_POLL_MS = Number(process.env.BASE_STATION_LORA_POLL_MS ?? 120);
const DRIVE_DUPLICATE_SUPPRESS_MS = Number(process.env.DRIVE_DUPLICATE_SUPPRESS_MS ?? 35);
const MANUAL_DRIVE_WS_FLUSH_MS = Number(process.env.MANUAL_DRIVE_WS_FLUSH_MS ?? 30);

function parseKeyList(value) {
  if (typeof value !== 'string') return [];
  return value
    .split(',')
    .map((token) => token.trim())
    .filter(Boolean);
}

const APP_API_KEYS = new Set();
const BOARD_API_KEYS = new Set();
const API_AUTH_ENABLED = false;

const rateLimitStore = new Map();
const remoteCommandQueue = [];

const metrics = {
  startedAt: Date.now(),
  authDenied: 0,
  rateLimitDenied: 0,
  commandsDispatched: 0,
  commandsFailed: 0,
  telemetryAccepted: 0,
  telemetryRejected: 0,
  safetyActions: 0,
  eventPruneRuns: 0,
  eventPrunedRows: 0,
  runtimeStateWrites: 0,
  runtimeStateWriteFailures: 0,
  wsPublishes: 0,
  wsDeliveredMessages: 0,
  wsConnectionsOpened: 0,
  wsConnectionsClosed: 0,
  wsPeakClients: 0,
  wsLastTestAt: null,
  remoteBaseStationAccepted: 0,
  remoteBaseStationRejected: 0,
  remoteBaseStationLastAt: null,
};

const state = {
  baseStation: null,
  homePoint: null,
  boundary: null,
  coverage: null,
  robot: null,
  remoteBaseStation: null,
  trail: [],
  lastPath: [],
  lastArrows: [],
  lastCommand: null,
  lastCommandId: null,
  lastCommandStatus: null,
  lastCommandAt: 0,
  commandHistory: [],
  lastFault: null,
  safety: {
    telemetryFailsafeAt: 0,
    telemetryFailsafeAction: null,
    telemetryFailsafeReason: null,
    gpsFailsafeAt: 0,
    gpsFailsafeAction: null,
    gpsFailsafeReason: null,
    geofenceFailsafeAt: 0,
    geofenceFailsafeAction: null,
    geofenceFailsafeReason: null,
  },
  demo: {
    enabled: false,
    updatedAt: 0,
    source: 'default',
    config: {
      laneWidthM: 3.0,
      cellSizeM: 0.75,
      coverageWidthM: 0.75,
      allowWeakGps: true,
      geofenceToleranceM: GEOFENCE_DEMO_TOLERANCE_M,
      minSpotDistanceM: DEMO_MIN_SPOT_DISTANCE_M,
      passes: 1,
      obstaclePolicyEnabled: DEMO_OBSTACLE_POLICY_ENABLED,
      obstacleStopCm: DEMO_OBSTACLE_STOP_CM,
      obstacleSidestepCm: DEMO_OBSTACLE_SIDESTEP_CM,
      obstacleCooldownMs: DEMO_OBSTACLE_COOLDOWN_MS,
      obstacleSidestepMs: DEMO_OBSTACLE_SIDESTEP_MS,
    },
    obstacle: {
      active: false,
      mode: null,
      side: null,
      nearestCm: null,
      at: 0,
      cooldownUntil: 0,
      note: null,
    },
    spots: {
      start: null,
      end: null,
    },
  },
  automation: {
    enabled: false,
    scheduledRunAt: null,
    label: null,
    notify: true,
    armedAt: 0,
    lastTriggeredAt: 0,
    lastResult: null,
    lastError: null,
  },
  operator: createOperatorState(),
  persistence: {
    restoredAt: null,
    lastSavedAt: null,
    lastSaveReason: null,
    lastSaveError: null,
    source: RUNTIME_STATE_PATH,
  },
};

let safetyMonitorInFlight = false;
let runtimeStateSaveTimer = null;
let scheduledMissionInFlight = false;
let lastPolledLoRaFrame = null;
let lastDriveWireCommand = null;
let lastDriveWireAt = 0;
let latestWsManualDrive = null;
let latestWsManualDriveQueuedAt = 0;
let manualDriveWsDispatchInFlight = false;
let lastMotorTelemetryLogMs = 0;

const DRIVE_COMMANDS = new Set([
  CMD.FORWARD,
  CMD.BACKWARD,
  CMD.LEFT,
  CMD.RIGHT,
  CMD.STOP,
  CMD.DRIVE,
]);

function pathHasZeroDispersion(points = state.lastPath) {
  return Array.isArray(points)
    && points.length > 0
    && points.every((point) => Number(point?.salt ?? 0) <= 0 && Number(point?.brine ?? 0) <= 0);
}

function getAutomationSnapshot(now = Date.now()) {
  const scheduledRunAt = Number(state.automation?.scheduledRunAt ?? 0) || 0;
  return {
    enabled: Boolean(state.automation?.enabled) && scheduledRunAt > 0,
    scheduledRunAt: scheduledRunAt || null,
    label: state.automation?.label ?? null,
    notify: state.automation?.notify !== false,
    armedAt: Number(state.automation?.armedAt ?? 0) || null,
    lastTriggeredAt: Number(state.automation?.lastTriggeredAt ?? 0) || null,
    lastResult: state.automation?.lastResult ?? null,
    lastError: state.automation?.lastError ?? null,
    dueInMs: scheduledRunAt > 0 ? Math.max(0, scheduledRunAt - now) : null,
  };
}

function clearAutomationSchedule(reason = 'cleared', error = null) {
  state.automation = {
    ...state.automation,
    enabled: false,
    scheduledRunAt: null,
    label: null,
    lastResult: reason,
    lastError: error,
  };
  return getAutomationSnapshot();
}

function armAutomationSchedule({ atMs, label, notify = true }) {
  state.automation = {
    ...state.automation,
    enabled: true,
    scheduledRunAt: atMs,
    label: typeof label === 'string' && label.trim() ? label.trim() : new Date(atMs).toLocaleString(),
    notify: Boolean(notify),
    armedAt: Date.now(),
    lastResult: 'armed',
    lastError: null,
  };
  return getAutomationSnapshot();
}

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

function coerceFiniteNumber(...values) {
  for (const value of values) {
    const number = Number(value);
    if (Number.isFinite(number)) {
      return number;
    }
  }
  return null;
}

function coerceBooleanLike(...values) {
  for (const value of values) {
    if (typeof value === 'boolean') return value;
    if (typeof value === 'number' && Number.isFinite(value)) return value !== 0;
    if (typeof value === 'string') {
      const normalized = value.trim().toLowerCase();
      if (['1', 'true', 'yes', 'y', 'ok', 'fix', 'fixed'].includes(normalized)) return true;
      if (['0', 'false', 'no', 'n', 'off', 'none', 'nofix', 'no fix'].includes(normalized)) return false;
    }
  }
  return null;
}

function normalizeCommand(raw) {
  if (typeof raw !== 'string') return null;
  // Strip optional CMD: prefix (STM32 LoRa protocol uses CMD:AUTO, CMD:MANUAL, etc.)
  let token = raw.trim().toUpperCase();
  if (/^D\s*:\s*-?\d+\s*,\s*-?\d+(\s*,\s*S\s*:\s*\d+)?$/.test(token)) return CMD.DRIVE;
  if (token.startsWith('CMD:')) token = token.slice(4).trim();
  // Strip SALT/BRINE params (e.g. AUTO,SALT:25,BRINE:75 -> AUTO)
  const commaIdx = token.indexOf(',');
  if (commaIdx !== -1) token = token.slice(0, commaIdx).trim();
  if (!token) return null;

  const aliases = {
    ...CMD_ALIASES,
    ...Object.fromEntries(Object.values(CMD).map((v) => [v, v])),
  };

  return aliases[token] ?? token;
}

function compactWireCommandForCmd(cmd) {
  switch (cmd) {
    case CMD.FORWARD: return 'F';
    case CMD.BACKWARD: return 'B';
    case CMD.LEFT: return 'L';
    case CMD.RIGHT: return 'R';
    case CMD.STOP: return 'S';
    case CMD.AUTO: return 'A';
    case CMD.MANUAL: return 'M';
    case CMD.PAUSE: return 'P';
    case CMD.ESTOP: return 'E';
    case CMD.RESET: return 'X';
    default: return cmd;
  }
}

function coerceManualDrivePayload(payload) {
  if (!payload || typeof payload !== 'object') {
    return null;
  }

  const drive = clampNumber(Math.round(coerceFiniteNumber(payload.drive, payload.throttle, 0)), -100, 100);
  const turn = clampNumber(Math.round(coerceFiniteNumber(payload.turn, payload.steer, 0)), -100, 100);
  const seq = Number.isFinite(Number(payload.seq)) ? Math.max(0, Math.floor(Number(payload.seq))) : null;
  const wireCommand = seq == null ? `D:${drive},${turn}` : `D:${drive},${turn},S:${seq}`;
  return { drive, turn, seq, wireCommand };
}

function queueWsManualDrive(payload) {
  const normalized = coerceManualDrivePayload(payload);
  if (!normalized) {
    return false;
  }
  latestWsManualDrive = normalized;
  latestWsManualDriveQueuedAt = Date.now();
  return true;
}

async function flushQueuedWsManualDrive() {
  if (manualDriveWsDispatchInFlight || !latestWsManualDrive) {
    return;
  }

  const pending = latestWsManualDrive;
  latestWsManualDrive = null;
  manualDriveWsDispatchInFlight = true;

  try {
    await dispatchCommand(CMD.DRIVE, {
      wireCommand: pending.wireCommand,
      waitForAck: false,
      waitForState: false,
      source: 'app.ws.manual-drive',
    });
  } finally {
    manualDriveWsDispatchInFlight = false;
  }
}

function currentMissionState() {
  return mission.publicMission()?.state ?? MISSION_STATE.IDLE;
}

function isWaypointCommand(cmd) {
  return cmd === LORA_WIRE.WP_CLEAR ||
    cmd.startsWith(`${LORA_WIRE.WP_ADD}:`) ||
    cmd.startsWith(`${LORA_WIRE.WP_LOAD}:`);
}

function getCoveragePct(stats) {
  return Number(stats?.coveredPct ?? stats?.coveragePercent ?? 0);
}

function clampNumber(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function cloneJsonSafe(value, fallback) {
  if (value == null) return fallback;
  try {
    return JSON.parse(JSON.stringify(value));
  } catch {
    return fallback;
  }
}

function normalizeLatLonPoint(point, fallback = null) {
  if (!point || typeof point !== 'object') return fallback;

  const lat = Number(point.lat ?? point.latitude);
  const lon = Number(point.lon ?? point.longitude);
  if (!Number.isFinite(lat) || !Number.isFinite(lon)) return fallback;

  const heading = Number(point.heading ?? point.headingDeg);
  const capturedAt = Number(point.capturedAt ?? point.timestampMs);
  return {
    lat,
    lon,
    heading: Number.isFinite(heading) ? heading : null,
    source: typeof point.source === 'string' && point.source.trim() ? point.source.trim() : null,
    capturedAt: Number.isFinite(capturedAt) ? capturedAt : Date.now(),
  };
}

function pointsRoughlyEqual(a, b, tolerance = 1e-6) {
  const pointA = normalizeLatLonPoint(a, null);
  const pointB = normalizeLatLonPoint(b, null);
  if (!pointA || !pointB) return false;

  return Math.abs(pointA.lat - pointB.lat) <= tolerance
    && Math.abs(pointA.lon - pointB.lon) <= tolerance;
}

function resolveHomePoint(preferred = null) {
  return normalizeLatLonPoint(preferred, null)
    ?? normalizeLatLonPoint(state.homePoint, null)
    ?? normalizeLatLonPoint(state.robot, null)
    ?? normalizeLatLonPoint(state.baseStation, null);
}

function mergePlannedSegments(...segments) {
  const merged = [];
  for (const segment of segments) {
    if (!Array.isArray(segment)) continue;
    for (const point of segment) {
      if (merged.length > 0 && pointsRoughlyEqual(merged[merged.length - 1], point)) {
        continue;
      }
      merged.push(point);
    }
  }
  return merged;
}

function expandPathWithPasses(points, passes = 1) {
  const normalizedPasses = Number.isFinite(Number(passes)) ? Math.max(1, Math.min(5, Math.floor(Number(passes)))) : 1;
  if (!Array.isArray(points) || points.length === 0 || normalizedPasses <= 1) {
    return Array.isArray(points) ? points : [];
  }

  let expanded = points.slice();
  for (let i = 1; i < normalizedPasses; i += 1) {
    expanded = mergePlannedSegments(expanded, points);
  }
  return expanded;
}

function serializeCoverageRuntime(map) {
  if (!map) return null;

  const cells = [];
  for (let row = 0; row < map.height; row++) {
    for (let col = 0; col < map.width; col++) {
      const cell = map.cells[row][col];
      if (!cell?.inside) continue;
      if (!cell.covered && !cell.hits && !cell.lastSeenMs) continue;
      cells.push({
        row,
        col,
        covered: Boolean(cell.covered),
        hits: Number(cell.hits ?? 0),
        lastSeenMs: Number(cell.lastSeenMs ?? 0),
      });
    }
  }

  return {
    cellSizeM: map.cellSizeM,
    cells,
    stats: coverageStats(map),
  };
}

function applyCoverageRuntime(map, snapshot) {
  if (!map || !snapshot || !Array.isArray(snapshot.cells)) return;
  for (const saved of snapshot.cells) {
    const row = Number(saved?.row);
    const col = Number(saved?.col);
    if (!withinGrid(map, row, col)) continue;
    const cell = map.cells[row][col];
    if (!cell?.inside) continue;
    cell.covered = Boolean(saved.covered);
    cell.hits = Number(saved.hits ?? 0);
    cell.lastSeenMs = Number(saved.lastSeenMs ?? 0);
  }
}

function buildRuntimeSnapshot(reason = 'unspecified') {
  return {
    version: 1,
    savedAt: Date.now(),
    reason,
    state: {
      baseStation: cloneJsonSafe(state.baseStation, null),
      homePoint: cloneJsonSafe(state.homePoint, null),
      remoteBaseStation: cloneJsonSafe(state.remoteBaseStation, null),
      boundary: cloneJsonSafe(state.boundary, null),
      coverage: serializeCoverageRuntime(state.coverage),
      robot: cloneJsonSafe(state.robot, null),
      trail: Array.isArray(state.trail)
        ? cloneJsonSafe(state.trail.slice(-Math.max(1, RUNTIME_STATE_TRAIL_LIMIT)), [])
        : [],
      lastPath: cloneJsonSafe(state.lastPath, []),
      lastArrows: cloneJsonSafe(state.lastArrows, []),
      commandHistory: Array.isArray(state.commandHistory)
        ? cloneJsonSafe(state.commandHistory.slice(0, Math.max(1, COMMAND_HISTORY_LIMIT)), [])
        : [],
      lastFault: cloneJsonSafe(state.lastFault, null),
      safety: cloneJsonSafe(state.safety, {}),
      demo: cloneJsonSafe(state.demo, { enabled: false, updatedAt: 0, source: 'default', spots: { start: null, end: null } }),
      automation: cloneJsonSafe(state.automation, { enabled: false, scheduledRunAt: null, label: null, notify: true, armedAt: 0, lastTriggeredAt: 0, lastResult: null, lastError: null }),
      operator: cloneJsonSafe({
        workflowAcks: state.operator.workflowAcks,
        notes: state.operator.notes,
      }, createOperatorState()),
      bridge: cloneJsonSafe(bridge.getStatus(), null),
      mission: cloneJsonSafe(mission.publicMission(), null),
    },
  };
}

function flushRuntimeState(reason = 'unspecified') {
  if (runtimeStateSaveTimer) {
    clearTimeout(runtimeStateSaveTimer);
    runtimeStateSaveTimer = null;
  }

  try {
    saveRuntimeState(buildRuntimeSnapshot(reason));
    metrics.runtimeStateWrites += 1;
    state.persistence.lastSavedAt = Date.now();
    state.persistence.lastSaveReason = reason;
    state.persistence.lastSaveError = null;
  } catch (error) {
    metrics.runtimeStateWriteFailures += 1;
    state.persistence.lastSaveError = error.message;
    console.error('[runtime] failed to save runtime state:', error.message);
  }
}

function scheduleRuntimeStateSave(reason = 'unspecified', options = {}) {
  const { immediate = false } = options;
  if (immediate) {
    flushRuntimeState(reason);
    return;
  }
  if (runtimeStateSaveTimer) {
    return;
  }
  runtimeStateSaveTimer = setTimeout(() => {
    flushRuntimeState(reason);
  }, Math.max(0, RUNTIME_STATE_SAVE_DEBOUNCE_MS));
}

function restoreRuntimeState() {
  let snapshot = null;
  try {
    snapshot = loadRuntimeState();
  } catch (error) {
    state.persistence.lastSaveError = `restore failed: ${error.message}`;
    console.error('[runtime] failed to load runtime state:', error.message);
  }

  const saved = snapshot?.state ?? null;
  const missionSnapshot = mission.publicMission();
  const boundary = Array.isArray(saved?.boundary) && saved.boundary.length >= 3
    ? saved.boundary
    : (Array.isArray(missionSnapshot?.boundary) && missionSnapshot.boundary.length >= 3 ? missionSnapshot.boundary : null);
  const baseStation = saved?.baseStation ?? missionSnapshot?.baseStation ?? null;
  const remoteBaseStation = cloneJsonSafe(saved?.remoteBaseStation, null);
  const homePoint = cloneJsonSafe(saved?.homePoint, null) ?? cloneJsonSafe(saved?.baseStation, null) ?? baseStation;
  const cellSizeM = Number(saved?.coverage?.cellSizeM ?? missionSnapshot?.cellSizeM ?? 2.0);

  state.baseStation = baseStation;
  state.homePoint = homePoint;
  state.remoteBaseStation = remoteBaseStation;
  state.boundary = boundary;
  state.robot = cloneJsonSafe(saved?.robot, null);
  state.trail = Array.isArray(saved?.trail) ? cloneJsonSafe(saved.trail, []) : [];
  state.lastPath = Array.isArray(saved?.lastPath) ? cloneJsonSafe(saved.lastPath, []) : [];
  state.lastArrows = Array.isArray(saved?.lastArrows) ? cloneJsonSafe(saved.lastArrows, []) : [];
  state.commandHistory = Array.isArray(saved?.commandHistory) ? cloneJsonSafe(saved.commandHistory, []) : [];
  state.lastFault = cloneJsonSafe(saved?.lastFault, null);
  state.safety = {
    ...state.safety,
    ...(cloneJsonSafe(saved?.safety, {}) ?? {}),
  };
  state.demo = {
    ...state.demo,
    ...(cloneJsonSafe(saved?.demo, {}) ?? {}),
    spots: {
      ...(state.demo?.spots ?? { start: null, end: null }),
      ...((cloneJsonSafe(saved?.demo?.spots, {}) ?? {})),
    },
  };
  state.automation = {
    ...state.automation,
    ...(cloneJsonSafe(saved?.automation, {}) ?? {}),
  };
  state.operator = createOperatorState();
  state.operator.workflowAcks = cloneJsonSafe(saved?.operator?.workflowAcks, {});
  state.operator.notes = Array.isArray(saved?.operator?.notes) ? cloneJsonSafe(saved.operator.notes, []) : [];

  if (boundary) {
    state.coverage = buildCoverageMap(boundary, cellSizeM);
    applyCoverageRuntime(state.coverage, saved?.coverage);
  }

  if (saved?.bridge) {
    bridge.restoreStatus(saved.bridge);
  }

  if (snapshot) {
    state.persistence.restoredAt = Date.now();
    state.persistence.lastSavedAt = Number(snapshot.savedAt ?? 0) || null;
    state.persistence.lastSaveReason = snapshot.reason ?? 'restored';
    state.persistence.lastSaveError = null;
  } else if (missionSnapshot) {
    state.persistence.restoredAt = Date.now();
    state.persistence.lastSaveReason = 'mission-db-fallback';
  }
}

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function createCommandId() {
  return `cmd_${Date.now()}_${Math.random().toString(36).slice(2, 8)}`;
}

function trimCommandHistory() {
  if (!Array.isArray(state.commandHistory)) {
    state.commandHistory = [];
    return;
  }
  if (state.commandHistory.length > COMMAND_HISTORY_LIMIT) {
    state.commandHistory = state.commandHistory.slice(0, COMMAND_HISTORY_LIMIT);
  }
}

function buildCommandTransportStatus(commandId, status, bridgeStatus = bridge.getStatus()) {
  const safeStatus = status ?? null;
  const bridgeCommandId = bridgeStatus.baseStation?.lastCmdId ?? null;
  const bridgeCommandStatus = bridgeStatus.baseStation?.lastCmdStatus ?? null;
  const seenByBaseStation = Boolean(commandId && bridgeCommandId === commandId);
  const ackedByLoRa = safeStatus === COMMAND_STATUS.ACKNOWLEDGED
    || safeStatus === COMMAND_STATUS.APPLIED
    || (seenByBaseStation && bridgeCommandStatus === COMMAND_STATUS.ACKNOWLEDGED);
  const appliedByRobot = safeStatus === COMMAND_STATUS.APPLIED;
  const parsedAck = bridgeStatus.baseStation?.lastAckParsed ?? null;

  let stage = COMMAND_TRANSPORT_STAGE.BACKEND_QUEUED;
  if (safeStatus === COMMAND_STATUS.FAILED) {
    stage = COMMAND_TRANSPORT_STAGE.FAILED;
  } else if (safeStatus === COMMAND_STATUS.TIMED_OUT) {
    stage = COMMAND_TRANSPORT_STAGE.TIMED_OUT;
  } else if (appliedByRobot) {
    stage = COMMAND_TRANSPORT_STAGE.ROBOT_APPLIED;
  } else if (ackedByLoRa) {
    stage = COMMAND_TRANSPORT_STAGE.LORA_ACKNOWLEDGED;
  } else if (safeStatus === COMMAND_STATUS.FORWARDED || safeStatus === COMMAND_STATUS.SENT || seenByBaseStation) {
    stage = COMMAND_TRANSPORT_STAGE.BASE_STATION_FORWARDED;
  }

  return {
    stage,
    backendStatus: safeStatus,
    baseStationSeen: seenByBaseStation,
    baseStationCommandId: bridgeCommandId,
    baseStationCommandStatus: bridgeCommandStatus,
    ackCategory: parsedAck?.category ?? null,
    ackSource: parsedAck?.source ?? null,
    robotAckState: parsedAck?.robotState ?? null,
    waypointIndex: parsedAck?.waypointIndex ?? null,
    waypointCount: parsedAck?.waypointCount ?? null,
    loRaAcked: ackedByLoRa,
    robotApplied: appliedByRobot,
  };
}

function upsertCommandHistory(entry) {
  if (!entry?.commandId) return;
  if (!Array.isArray(state.commandHistory)) {
    state.commandHistory = [];
  }

  const existingIndex = state.commandHistory.findIndex((item) => item?.commandId === entry.commandId);
  const next = {
    ...(existingIndex >= 0 ? state.commandHistory[existingIndex] : {}),
    ...entry,
    updatedAt: Date.now(),
  };

  if (existingIndex >= 0) {
    state.commandHistory.splice(existingIndex, 1);
  }
  state.commandHistory.unshift(next);
  trimCommandHistory();
}

function updateCommandTracking(commandId, patch = {}) {
  if (!commandId) return;
  const timestamp = Date.now();
  const status = patch.status ?? state.lastCommandStatus ?? null;
  const bridgeStatus = patch.bridgeStatus ?? bridge.getStatus();
  state.lastCommand = patch.cmd ?? state.lastCommand;
  state.lastCommandId = commandId;
  state.lastCommandStatus = status;
  state.lastCommandAt = timestamp;

  upsertCommandHistory({
    commandId,
    cmd: patch.cmd ?? state.lastCommand ?? null,
    source: patch.source ?? null,
    status,
    error: patch.error ?? null,
    bridgeAckSource: patch.bridgeAckSource ?? null,
    transport: buildCommandTransportStatus(commandId, status, bridgeStatus),
    detail: patch.detail ?? null,
    at: patch.at ?? timestamp,
  });

  publish(WS_EVENT.COMMAND_RECEIVED, {
    cmd: patch.cmd ?? state.lastCommand ?? null,
    commandId,
    status,
    at: state.lastCommandAt,
    error: patch.error ?? null,
    bridgeAckSource: patch.bridgeAckSource ?? null,
    transport: buildCommandTransportStatus(commandId, status, bridgeStatus),
  });
}

function resolveGpsFailsafeAction() {
  if (GPS_FAILSAFE_ACTION === CMD.ESTOP) return CMD.ESTOP;
  return CMD.PAUSE;
}

function getRobotGpsStatus(robot = state.robot) {
  if (!robot) {
    return { ready: false, reason: 'Robot telemetry is unavailable' };
  }
  if (!robot.gpsFix) {
    return { ready: false, reason: 'Robot GPS fix is not available' };
  }
  if (Number.isFinite(GPS_READY_MIN_SAT) && GPS_READY_MIN_SAT > 0 && Number(robot.gpsSat ?? 0) < GPS_READY_MIN_SAT) {
    return {
      ready: false,
      reason: `Robot GPS satellites ${Number(robot.gpsSat ?? 0)} below minimum ${GPS_READY_MIN_SAT}`,
    };
  }
  if (Number.isFinite(GPS_READY_MAX_HDOP) && GPS_READY_MAX_HDOP > 0) {
    const hdop = Number(robot.gpsHdop ?? 0);
    if (!Number.isFinite(hdop) || hdop <= 0 || hdop > GPS_READY_MAX_HDOP) {
      return {
        ready: false,
        reason: `Robot GPS HDOP ${Number.isFinite(hdop) ? hdop.toFixed(1) : 'unknown'} exceeds maximum ${GPS_READY_MAX_HDOP}`,
      };
    }
  }
  return { ready: true, reason: null };
}

function getDemoSpotGpsStatus(spot, label) {
  const robot = spot?.robot ?? null;
  if (!robot) {
    return { ready: false, reason: `${label} has no saved robot telemetry` };
  }
  const gpsStatus = getRobotGpsStatus(robot);
  if (gpsStatus.ready) {
    return gpsStatus;
  }
  return {
    ready: false,
    reason: `${label}: ${gpsStatus.reason}`,
  };
}

function getDemoSpotStatuses() {
  return {
    start: getDemoSpotGpsStatus(state.demo?.spots?.start, 'Spot A'),
    end: getDemoSpotGpsStatus(state.demo?.spots?.end, 'Spot B'),
  };
}

function getDemoConfig() {
  const current = state.demo?.config ?? {};
  const laneWidthM = Number(current.laneWidthM ?? 3.0);
  const cellSizeM = Number(current.cellSizeM ?? 0.75);
  const coverageWidthM = Number(current.coverageWidthM ?? 0.75);
  const geofenceToleranceM = Number(current.geofenceToleranceM ?? GEOFENCE_DEMO_TOLERANCE_M);
  const minSpotDistanceM = Number(current.minSpotDistanceM ?? DEMO_MIN_SPOT_DISTANCE_M);
  const passes = Number(current.passes ?? 1);
  const obstacleStopCm = Number(current.obstacleStopCm ?? DEMO_OBSTACLE_STOP_CM);
  const obstacleSidestepCm = Number(current.obstacleSidestepCm ?? DEMO_OBSTACLE_SIDESTEP_CM);
  const obstacleCooldownMs = Number(current.obstacleCooldownMs ?? DEMO_OBSTACLE_COOLDOWN_MS);
  const obstacleSidestepMs = Number(current.obstacleSidestepMs ?? DEMO_OBSTACLE_SIDESTEP_MS);

  return {
    laneWidthM: Number.isFinite(laneWidthM) && laneWidthM > 0 ? laneWidthM : 3.0,
    cellSizeM: Number.isFinite(cellSizeM) && cellSizeM > 0 ? cellSizeM : 0.75,
    coverageWidthM: Number.isFinite(coverageWidthM) && coverageWidthM > 0 ? coverageWidthM : 0.75,
    allowWeakGps: current.allowWeakGps !== false,
    geofenceToleranceM: Number.isFinite(geofenceToleranceM) && geofenceToleranceM >= 0 ? geofenceToleranceM : GEOFENCE_DEMO_TOLERANCE_M,
    minSpotDistanceM: Number.isFinite(minSpotDistanceM) && minSpotDistanceM > 0 ? minSpotDistanceM : DEMO_MIN_SPOT_DISTANCE_M,
    passes: Number.isFinite(passes) && passes >= 1 ? Math.min(5, Math.floor(passes)) : 1,
    obstaclePolicyEnabled: current.obstaclePolicyEnabled !== false,
    obstacleStopCm: Number.isFinite(obstacleStopCm) && obstacleStopCm > 0 ? obstacleStopCm : DEMO_OBSTACLE_STOP_CM,
    obstacleSidestepCm: Number.isFinite(obstacleSidestepCm) && obstacleSidestepCm > 0 ? obstacleSidestepCm : DEMO_OBSTACLE_SIDESTEP_CM,
    obstacleCooldownMs: Number.isFinite(obstacleCooldownMs) && obstacleCooldownMs >= 0 ? obstacleCooldownMs : DEMO_OBSTACLE_COOLDOWN_MS,
    obstacleSidestepMs: Number.isFinite(obstacleSidestepMs) && obstacleSidestepMs > 0 ? obstacleSidestepMs : DEMO_OBSTACLE_SIDESTEP_MS,
  };
}

function resolveDemoReadiness() {
  const missionState = currentMissionState();
  const gpsStatus = getRobotGpsStatus();
  const demoStatuses = getDemoSpotStatuses();
  const hasStart = Boolean(state.demo?.spots?.start);
  const hasEnd = Boolean(state.demo?.spots?.end);
  const hasPath = Array.isArray(state.lastPath) && state.lastPath.length > 0;
  const wpPushState = bridge.getStatus().wpPushState ?? 'none';

  const blockers = [];
  if (!state.demo?.enabled) blockers.push('Demo mode is not enabled');
  if (!hasStart) blockers.push('Spot A is not marked');
  if (!hasEnd) blockers.push('Spot B is not marked');
  if (hasStart && !demoStatuses.start.ready) blockers.push(demoStatuses.start.reason ?? 'Spot A GPS is not ready');
  if (hasEnd && !demoStatuses.end.ready) blockers.push(demoStatuses.end.reason ?? 'Spot B GPS is not ready');
  if (!hasPath) blockers.push('No demo path has been built');
  if (wpPushState !== 'committed' && hasPath) blockers.push(`Waypoints are ${wpPushState}, not committed`);
  if (!gpsStatus.ready) blockers.push(gpsStatus.reason ?? 'Robot GPS is not ready');
  if (missionState !== MISSION_STATE.CONFIGURING && missionState !== MISSION_STATE.PAUSED) {
    blockers.push(`Mission must be CONFIGURING or PAUSED, currently ${missionState}`);
  }

  return {
    missionState,
    hasStart,
    hasEnd,
    hasPath,
    wpPushState,
    gpsReady: gpsStatus.ready,
    gpsReason: gpsStatus.reason,
    readyToPlan: Boolean(state.demo?.enabled) && hasStart && hasEnd,
    readyToRun: blockers.length === 0,
    blockers,
  };
}

function resolveTelemetryFailsafeAction() {
  if (TELEMETRY_FAILSAFE_ACTION === CMD.PAUSE) return CMD.PAUSE;
  return CMD.ESTOP;
}

function resolveGeofenceFailsafeAction() {
  if (GEOFENCE_FAILSAFE_ACTION === CMD.PAUSE) return CMD.PAUSE;
  return CMD.ESTOP;
}

function isRobotInsideCoverageArea(robotPoint, options = {}) {
  if (!state.coverage || !robotPoint) return true;
  const { row, col } = worldToGrid(state.coverage, robotPoint);
  const demoConfig = getDemoConfig();
  const configuredToleranceM = Number.isFinite(Number(options.toleranceM))
    ? Number(options.toleranceM)
    : (state.demo?.enabled ? demoConfig.geofenceToleranceM : GEOFENCE_TOLERANCE_M);
  const safeCellSizeM = Math.max(0.01, Number(state.coverage?.cellSizeM ?? 1));
  const toleranceCells = Math.max(1, Math.ceil(Math.max(0, configuredToleranceM) / safeCellSizeM));

  for (let rowOffset = -toleranceCells; rowOffset <= toleranceCells; rowOffset++) {
    for (let colOffset = -toleranceCells; colOffset <= toleranceCells; colOffset++) {
      const candidateRow = row + rowOffset;
      const candidateCol = col + colOffset;
      if (!withinGrid(state.coverage, candidateRow, candidateCol)) continue;
      if (state.coverage.cells[candidateRow]?.[candidateCol]?.inside) {
        return true;
      }
    }
  }

  return false;
}

function resolveCoverageMarkRadiusM() {
  if (!Number.isFinite(COVERAGE_MARK_RADIUS_M) || COVERAGE_MARK_RADIUS_M <= 0) {
    return 0.5;
  }
  return COVERAGE_MARK_RADIUS_M;
}

function estimateGridCellCount(boundaryLatLon, cellSizeM) {
  if (!Array.isArray(boundaryLatLon) || boundaryLatLon.length < 3 || !Number.isFinite(cellSizeM) || cellSizeM <= 0) {
    return { width: 1, height: 1, cells: 1 };
  }

  const origin = {
    lat: boundaryLatLon[0].lat,
    lon: boundaryLatLon[0].lon,
  };

  const localPolygon = boundaryLatLon.map((point) => latLonToLocal(point, origin));
  const frame = resolveCoverageFrame(localPolygon);
  const width = Math.max(1, Math.ceil((frame.maxX - frame.minX) / cellSizeM));
  const height = Math.max(1, Math.ceil((frame.maxY - frame.minY) / cellSizeM));
  return { width, height, cells: width * height };
}

function resolveInputAreaCellSize(boundaryLatLon, requestedCellSizeM) {
  const requested = Number.isFinite(requestedCellSizeM)
    ? clampNumber(requestedCellSizeM, MIN_INPUT_AREA_CELL_SIZE_M, MAX_INPUT_AREA_CELL_SIZE_M)
    : 2.0;

  const firstEstimate = estimateGridCellCount(boundaryLatLon, requested);
  if (firstEstimate.cells <= MAX_INPUT_AREA_CELLS) {
    return {
      requestedCellSizeM: requested,
      effectiveCellSizeM: requested,
      adjusted: false,
      estimatedCells: firstEstimate.cells,
    };
  }

  const scale = Math.sqrt(firstEstimate.cells / MAX_INPUT_AREA_CELLS);
  const effectiveCellSizeM = clampNumber(requested * scale, requested, MAX_INPUT_AREA_CELL_SIZE_M);
  const secondEstimate = estimateGridCellCount(boundaryLatLon, effectiveCellSizeM);

  return {
    requestedCellSizeM: requested,
    effectiveCellSizeM,
    adjusted: effectiveCellSizeM > requested,
    estimatedCells: secondEstimate.cells,
  };
}

function readApiKey(req) {
  const directHeader = req.header('x-api-key');
  if (typeof directHeader === 'string' && directHeader.trim()) {
    return directHeader.trim();
  }

  const authHeader = req.header('authorization');
  if (typeof authHeader === 'string' && authHeader.startsWith('Bearer ')) {
    const token = authHeader.slice('Bearer '.length).trim();
    if (token) return token;
  }

  return null;
}

function identifyClientRole(req) {
  const apiKey = readApiKey(req);
  if (!apiKey) return null;
  if (APP_API_KEYS.has(apiKey)) return 'app';
  if (BOARD_API_KEYS.has(apiKey)) return 'board';
  return null;
}

function requireClientRole(allowedRoles) {
  const allowed = new Set(allowedRoles);

  return (req, res, next) => {
    if (!API_AUTH_ENABLED) {
      return next();
    }

    const role = identifyClientRole(req);
    if (!role || !allowed.has(role)) {
      metrics.authDenied += 1;
      return res.status(401).json({
        ok: false,
        error: `Unauthorized for role(s): ${Array.from(allowed).join(',')}`,
      });
    }

    req.clientRole = role;
    return next();
  };
}

const requireApp = requireClientRole(['app']);
const requireBoard = requireClientRole(['board']);
const requireAppOrBoard = requireClientRole(['app', 'board']);

function makeRateLimitMiddleware(bucketName, limitPerWindow, windowMs) {
  return (req, res, next) => {
    const role = req.clientRole ?? identifyClientRole(req) ?? 'anon';
    const key = `${bucketName}:${role}:${req.ip ?? 'unknown'}`;
    const now = Date.now();
    const windowStart = now - windowMs;

    const existing = rateLimitStore.get(key);
    if (!existing || existing.windowStart < windowStart) {
      rateLimitStore.set(key, { windowStart: now, count: 1 });
      return next();
    }

    existing.count += 1;
    if (existing.count > limitPerWindow) {
      metrics.rateLimitDenied += 1;
      return res.status(429).json({
        ok: false,
        error: `Rate limit exceeded for ${bucketName}`,
        bucket: bucketName,
        limitPerWindow,
        windowMs,
      });
    }

    return next();
  };
}

const rateLimitCommand = makeRateLimitMiddleware('command', COMMAND_RATE_LIMIT_PER_WINDOW, REQUEST_RATE_LIMIT_WINDOW_MS);
const rateLimitTelemetry = makeRateLimitMiddleware('telemetry', TELEMETRY_RATE_LIMIT_PER_WINDOW, REQUEST_RATE_LIMIT_WINDOW_MS);

function resetOperatorState() {
  state.operator = createOperatorState();
  scheduleRuntimeStateSave('operator.reset', { immediate: true });
}

function telemetryAgeMs(now = Date.now()) {
  return supervisionTelemetryAgeMs(state.robot, now);
}

function isTelemetryStale(now = Date.now()) {
  return supervisionIsTelemetryStale(state.robot, SUPERVISION_TELEMETRY_STALE_MS, now);
}

function recoveryResetNeeded() {
  const robotState = String(state.robot?.state ?? '').trim().toUpperCase();
  if ([ROBOT_STATE.ESTOP, ROBOT_STATE.ERROR].includes(robotState)) {
    return true;
  }

  const lastFaultAction = String(state.lastFault?.action ?? '').trim().toUpperCase();
  if ([FAULT_ACTION.ESTOP, FAULT_ACTION.PAUSE].includes(lastFaultAction)) {
    return true;
  }

  return Boolean(
    state.safety?.telemetryFailsafeAt
    || state.safety?.gpsFailsafeAt
    || state.safety?.geofenceFailsafeAt
  );
}

function clearRecoveredSafetyState({ telemetryRecovered = false, gpsRecovered = false, geofenceRecovered = false, faultRecovered = false } = {}) {
  let changed = false;

  if (telemetryRecovered && (state.safety.telemetryFailsafeAt || state.safety.telemetryFailsafeAction || state.safety.telemetryFailsafeReason)) {
    state.safety.telemetryFailsafeAt = 0;
    state.safety.telemetryFailsafeAction = null;
    state.safety.telemetryFailsafeReason = null;
    changed = true;
  }

  if (gpsRecovered && (state.safety.gpsFailsafeAt || state.safety.gpsFailsafeAction || state.safety.gpsFailsafeReason)) {
    state.safety.gpsFailsafeAt = 0;
    state.safety.gpsFailsafeAction = null;
    state.safety.gpsFailsafeReason = null;
    changed = true;
  }

  if (geofenceRecovered && (state.safety.geofenceFailsafeAt || state.safety.geofenceFailsafeAction || state.safety.geofenceFailsafeReason)) {
    state.safety.geofenceFailsafeAt = 0;
    state.safety.geofenceFailsafeAction = null;
    state.safety.geofenceFailsafeReason = null;
    changed = true;
  }

  if (
    faultRecovered
    && state.lastFault
    && [FAULT_ACTION.ESTOP, FAULT_ACTION.PAUSE].includes(String(state.lastFault.action ?? '').trim().toUpperCase())
  ) {
    state.lastFault = null;
    changed = true;
  }

  return changed;
}

function clearResetRecoveryState() {
  clearRecoveredSafetyState({
    telemetryRecovered: true,
    gpsRecovered: true,
    geofenceRecovered: true,
    faultRecovered: true,
  });

  if (state.robot && [ROBOT_STATE.ESTOP, ROBOT_STATE.ERROR].includes(String(state.robot.state ?? '').trim().toUpperCase())) {
    state.robot = {
      ...state.robot,
      state: ROBOT_STATE.PAUSE,
      timestampMs: Date.now(),
    };
  }
}

function remoteBaseStationAgeMs(now = Date.now()) {
  if (!state.remoteBaseStation?.receivedAt) return null;
  return Math.max(0, now - state.remoteBaseStation.receivedAt);
}

function isRemoteBaseStationFresh(now = Date.now()) {
  const age = remoteBaseStationAgeMs(now);
  return typeof age === 'number' && age <= REMOTE_BASE_STATION_STALE_MS;
}

function isRemoteCommandFallbackReady(now = Date.now()) {
  if (!isRemoteBaseStationFresh(now)) {
    return false;
  }

  const wifiState = String(state.remoteBaseStation?.wifiLinkState ?? '').trim().toLowerCase();
  return !wifiState || ['online', 'connected'].includes(wifiState);
}

function normalizeRemoteBaseStationStatus(payload) {
  if (!payload || typeof payload !== 'object' || Array.isArray(payload)) return null;

  const receivedAt = Date.now();
  const queueDepth = Number.isFinite(Number(payload.queue_depth ?? payload.queueDepth))
    ? Number(payload.queue_depth ?? payload.queueDepth)
    : null;
  const ackCount = Number.isFinite(Number(payload.ack_count ?? payload.ackCount))
    ? Number(payload.ack_count ?? payload.ackCount)
    : null;
  const lastLoRaAgeMs = Number.isFinite(Number(payload.last_lora_age_ms ?? payload.lastLoRaAgeMs))
    ? Number(payload.last_lora_age_ms ?? payload.lastLoRaAgeMs)
    : null;
  const lastAckAgeMs = Number.isFinite(Number(payload.last_ack_age_ms ?? payload.lastAckAgeMs))
    ? Number(payload.last_ack_age_ms ?? payload.lastAckAgeMs)
    : null;
  const configured = typeof payload.configured === 'boolean'
    ? payload.configured
    : (String(payload.configured ?? '').toLowerCase() === 'true');

  return {
    receivedAt,
    source: 'remote_status',
    connectionPath: 'remote_bridge',
    connectionPathLabel: 'Remote bridge',
    discoverySource: 'remote_status',
    reachable: true,
    statusVersion: payload.status_version ?? payload.statusVersion ?? null,
    mode: payload.mode ?? null,
    state: payload.state ?? null,
    wifiLinkState: payload.wifi_link_state ?? payload.wifiLinkState ?? null,
    loraLinkState: payload.lora_link_state ?? payload.loraLinkState ?? null,
    queueDepth,
    ackCount,
    lastLoRaAgeMs,
    lastAckAgeMs,
    lastCmd: payload.last_cmd ?? payload.lastCmd ?? null,
    lastCmdId: payload.last_cmd_id ?? payload.lastCmdId ?? null,
    lastCmdStatus: payload.last_cmd_status ?? payload.lastCmdStatus ?? null,
    lastAck: payload.last_ack ?? payload.lastAck ?? null,
    lastLoRa: payload.last_lora ?? payload.lastLoRa ?? null,
    backendUrl: payload.backend_url ?? payload.backendUrl ?? null,
    configured,
    apSsid: payload.ap_ssid ?? payload.apSsid ?? null,
    raw: cloneJsonSafe(payload, null),
  };
}

function buildRemoteBaseStationDiagnostics(now = Date.now()) {
  const ageMs = remoteBaseStationAgeMs(now);
  const fresh = isRemoteBaseStationFresh(now);
  const remote = state.remoteBaseStation ?? null;
  return {
    present: Boolean(remote),
    fresh,
    ageMs,
    lastReceivedAt: remote?.receivedAt ?? null,
    wifiLinkState: remote?.wifiLinkState ?? null,
    loraLinkState: remote?.loraLinkState ?? null,
    backendUrl: remote?.backendUrl ?? null,
    lastCmdId: remote?.lastCmdId ?? null,
    lastCmdStatus: remote?.lastCmdStatus ?? null,
    queueDepth: remote?.queueDepth ?? null,
    ackCount: remote?.ackCount ?? null,
  };
}

function normalizeRemoteTrackedStatus(rawStatus) {
  const token = String(rawStatus ?? '').trim().toLowerCase();
  if (!token) return null;
  if (['applied', 'complete', 'completed'].includes(token)) return COMMAND_STATUS.APPLIED;
  if (['acknowledged', 'ack', 'acked'].includes(token)) return COMMAND_STATUS.ACKNOWLEDGED;
  if (['forwarded', 'sent'].includes(token)) return COMMAND_STATUS.FORWARDED;
  if (['queued', 'pending', 'retry', 'retrying'].includes(token)) return COMMAND_STATUS.QUEUED;
  if (['failed', 'error'].includes(token)) return COMMAND_STATUS.FAILED;
  return null;
}

function transportStageForStatus(status) {
  switch (status) {
    case COMMAND_STATUS.APPLIED:
      return COMMAND_TRANSPORT_STAGE.ROBOT_APPLIED;
    case COMMAND_STATUS.ACKNOWLEDGED:
      return COMMAND_TRANSPORT_STAGE.LORA_ACKNOWLEDGED;
    case COMMAND_STATUS.FORWARDED:
      return COMMAND_TRANSPORT_STAGE.BASE_STATION_FORWARDED;
    case COMMAND_STATUS.FAILED:
      return COMMAND_TRANSPORT_STAGE.FAILED;
    case COMMAND_STATUS.TIMED_OUT:
      return COMMAND_TRANSPORT_STAGE.TIMED_OUT;
    default:
      return COMMAND_TRANSPORT_STAGE.BACKEND_QUEUED;
  }
}

function pruneRemoteCommandQueue(now = Date.now()) {
  for (let index = remoteCommandQueue.length - 1; index >= 0; index -= 1) {
    const entry = remoteCommandQueue[index];
    const createdAt = Number(entry?.createdAt ?? now);
    const ackedAt = Number(entry?.ackedAt ?? 0);
    const ageMs = Math.max(0, now - createdAt);
    const ackAgeMs = ackedAt > 0 ? Math.max(0, now - ackedAt) : 0;

    if (!entry?.deliveryAcked && ageMs > REMOTE_COMMAND_STALE_MS) {
      updateCommandTracking(entry.commandId, {
        cmd: entry.cmd,
        source: entry.source ?? 'remote-bridge',
        status: COMMAND_STATUS.TIMED_OUT,
        error: 'Remote base station command delivery timed out',
        detail: { stage: COMMAND_TRANSPORT_STAGE.TIMED_OUT, ageMs },
      });
      remoteCommandQueue.splice(index, 1);
      continue;
    }

    if (!entry?.deliveryAcked && isMotionWireCommand(entry?.cmd)) {
      const leaseCount = Number(entry?.leaseCount ?? 0);
      if (leaseCount >= REMOTE_COMMAND_MAX_MOTION_LEASES && ageMs >= REMOTE_COMMAND_MAX_MOTION_LEASE_AGE_MS) {
        updateCommandTracking(entry.commandId, {
          cmd: entry.cmd,
          source: entry.source ?? 'remote-bridge',
          status: COMMAND_STATUS.TIMED_OUT,
          error: 'Remote motion command lease limit reached without ACK',
          detail: { stage: COMMAND_TRANSPORT_STAGE.TIMED_OUT, ageMs, leaseCount },
        });
        remoteCommandQueue.splice(index, 1);
        continue;
      }
    }

    if (entry?.deliveryAcked && ackAgeMs > REMOTE_COMMAND_ACK_RETENTION_MS) {
      remoteCommandQueue.splice(index, 1);
    }
  }
}

function pendingRemoteCommandCount(now = Date.now()) {
  pruneRemoteCommandQueue(now);
  return remoteCommandQueue.filter((entry) => !entry.deliveryAcked).length;
}

function normalizeRemoteQueueCommand(cmd) {
  return typeof cmd === 'string' ? cmd.trim().toUpperCase() : '';
}

function isMotionWireCommand(cmd) {
  const token = normalizeRemoteQueueCommand(cmd);
  return token === CMD.FORWARD
    || token === CMD.BACKWARD
    || token === CMD.LEFT
    || token === CMD.RIGHT
    || token === CMD.STOP
    || token === CMD.DRIVE
    || token.startsWith('D:')
    || token.startsWith('DRIVE,');
}

function isModeWireCommand(cmd) {
  const token = normalizeRemoteQueueCommand(cmd);
  return token === CMD.MANUAL
    || token === CMD.AUTO
    || token === CMD.PAUSE
    || token === CMD.ESTOP
    || token === CMD.RESET;
}

function enqueueRemoteCommand({ commandId, cmd, source = 'remote-bridge' }) {
  pruneRemoteCommandQueue();
  const existing = remoteCommandQueue.find((entry) => entry.commandId === commandId);
  if (existing) {
    return existing;
  }

  // Keep manual/test interactions responsive: never allow stale motion backlog
  // to delay the newest motion update or mode transition.
  if (isMotionWireCommand(cmd) || isModeWireCommand(cmd)) {
    for (let index = remoteCommandQueue.length - 1; index >= 0; index -= 1) {
      const entry = remoteCommandQueue[index];
      const normalizedQueued = normalizeRemoteQueueCommand(entry?.cmd);
      const isPreemptibleMode = isModeWireCommand(entry?.cmd)
        && ![CMD.ESTOP, CMD.RESET].includes(normalizedQueued);
      if (!entry?.deliveryAcked && (isMotionWireCommand(entry.cmd) || isPreemptibleMode)) {
        remoteCommandQueue.splice(index, 1);
      }
    }
  }

  while (pendingRemoteCommandCount() >= REMOTE_COMMAND_MAX_QUEUE && pendingRemoteCommandCount() > 0) {
    // Keep admission non-blocking for newest operator/test commands.
    remoteCommandQueue.shift();
  }

  const entry = {
    commandId,
    cmd,
    source,
    createdAt: Date.now(),
    deliveryAcked: false,
    ackedAt: null,
    deliveryStatus: COMMAND_STATUS.QUEUED,
    leaseCount: 0,
    lastLeaseAt: null,
    lastError: null,
  };
  remoteCommandQueue.push(entry);
  return entry;
}

function queueRemoteWaypointPush(points, source = 'remote-bridge') {
  if (!Array.isArray(points) || points.length === 0) {
    return { ok: false, status: 400, error: 'No waypoint set available for remote queue', sent: 0 };
  }

  const commands = [
    LORA_WIRE.WP_CLEAR,
    ...points.map((point, index) => {
      const salt = clampNumber(Math.round(Number(point?.salt ?? 0)), 0, 100);
      const brine = clampNumber(Math.round(Number(point?.brine ?? 0)), 0, 100);
      return `${LORA_WIRE.WP_ADD}:${index}:${Number(point.lat).toFixed(6)},${Number(point.lon).toFixed(6)},${salt},${brine}`;
    }),
    `${LORA_WIRE.WP_LOAD}:${points.length}`,
  ];

  while ((pendingRemoteCommandCount() + commands.length) > REMOTE_COMMAND_MAX_QUEUE && pendingRemoteCommandCount() > 0) {
    remoteCommandQueue.shift();
  }

  let finalCommandId = null;
  for (const command of commands) {
    const commandId = createCommandId();
    const queued = enqueueRemoteCommand({ commandId, cmd: command, source: `${source}.remote` });
    if (!queued) {
      bridge.resetWpState();
      return { ok: false, status: 503, error: 'Unable to queue remote waypoint command', sent: 0 };
    }
    finalCommandId = commandId;
    bridge.observeCommand(command);
  }

  if (finalCommandId) {
    updateCommandTracking(finalCommandId, {
      cmd: `${LORA_WIRE.WP_LOAD}:${points.length}`,
      source,
      status: COMMAND_STATUS.QUEUED,
      detail: {
        stage: COMMAND_TRANSPORT_STAGE.BACKEND_QUEUED,
        remoteQueued: true,
        queueDepth: pendingRemoteCommandCount(),
        waypointCount: points.length,
        commandBatchSize: commands.length,
      },
    });
  }

  return {
    ok: true,
    status: 202,
    sent: points.length,
    queuedRemote: true,
    commandId: finalCommandId,
  };
}

function peekRemoteCommandForBoard(now = Date.now()) {
  pruneRemoteCommandQueue(now);
  const pending = remoteCommandQueue.find((entry) => !entry.deliveryAcked) ?? null;
  if (!pending) return null;

  const lastLeaseAt = Number(pending.lastLeaseAt ?? 0);
  if (lastLeaseAt > 0 && (now - lastLeaseAt) < REMOTE_COMMAND_LEASE_RETRY_MS) {
    return null;
  }

  return pending;
}

function acknowledgeRemoteCommand(commandId, payload = {}) {
  const entry = remoteCommandQueue.find((item) => item.commandId === commandId);
  if (!entry) {
    return null;
  }

  const status = normalizeRemoteTrackedStatus(payload?.status) ?? COMMAND_STATUS.FORWARDED;
  entry.deliveryStatus = status;
  entry.lastError = typeof payload?.error === 'string' ? payload.error : null;
  entry.deliveryAcked = payload?.status !== 'retry';
  entry.ackedAt = Date.now();

  updateCommandTracking(commandId, {
    cmd: entry.cmd,
    source: entry.source ?? 'remote-bridge',
    status,
    error: entry.lastError,
    detail: {
      stage: transportStageForStatus(status),
      remoteQueued: true,
      queueDepth: pendingRemoteCommandCount(),
    },
  });

  scheduleRuntimeStateSave('command.remote_ack');
  publishSupervision();
  publishOperator();
  pruneRemoteCommandQueue();
  return entry;
}

function getWorkflowAck(workflowId, stepId) {
  return state.operator.workflowAcks[workflowId]?.[stepId] ?? null;
}

function setWorkflowAck(workflowId, stepId, checked, meta = {}) {
  if (!state.operator.workflowAcks[workflowId]) {
    state.operator.workflowAcks[workflowId] = {};
  }

  if (!checked) {
    delete state.operator.workflowAcks[workflowId][stepId];
    if (Object.keys(state.operator.workflowAcks[workflowId]).length === 0) {
      delete state.operator.workflowAcks[workflowId];
    }
    scheduleRuntimeStateSave('operator.workflow_ack_clear', { immediate: true });
    return null;
  }

  const ack = {
    checked: true,
    actor: meta.actor ?? 'operator',
    note: meta.note ?? null,
    at: Date.now(),
  };
  state.operator.workflowAcks[workflowId][stepId] = ack;
  scheduleRuntimeStateSave('operator.workflow_ack', { immediate: true });
  return ack;
}

function addOperatorNote({ text, category = 'general', actor = 'operator' }) {
  const note = {
    id: `${Date.now()}-${Math.random().toString(36).slice(2, 8)}`,
    text,
    category,
    actor,
    at: Date.now(),
  };

  state.operator.notes.push(note);
  if (state.operator.notes.length > 100) {
    state.operator.notes = state.operator.notes.slice(-100);
  }
  mission.addNote(`[${category}] ${actor}: ${text}`);
  db.logEvent(mission.currentId(), EVENT_TYPE.OPERATOR_NOTE_ADDED, note);
  scheduleRuntimeStateSave('operator.note_added', { immediate: true });
  return note;
}

function buildAllowedActions() {
  const gpsStatus = getRobotGpsStatus();
  return buildSupervisionAllowedActions({
    missionState: currentMissionState(),
    wpPushState: bridge.getStatus().wpPushState,
    hasCoverage: Boolean(state.coverage),
    hasPath: state.lastPath.length > 0,
    zeroDispersionPath: pathHasZeroDispersion(),
    gpsReady: gpsStatus.ready,
    gpsReason: gpsStatus.reason,
    demoModeEnabled: Boolean(state.demo?.enabled),
    resetNeeded: recoveryResetNeeded(),
  });
}

function buildAlerts(now = Date.now()) {
  const gpsStatus = getRobotGpsStatus();
  const connectivity = buildConnectionState(now);
  return buildSupervisionAlerts({
    missionState: currentMissionState(),
    wpPushState: bridge.getStatus().wpPushState,
    hasCoverage: Boolean(state.coverage),
    hasPath: state.lastPath.length > 0,
    zeroDispersionPath: pathHasZeroDispersion(),
    gpsReady: gpsStatus.ready,
    gpsReason: gpsStatus.reason,
    demoModeEnabled: Boolean(state.demo?.enabled),
    telemetryStale: isTelemetryStale(now),
    telemetryAge: telemetryAgeMs(now),
    lastFault: state.lastFault,
    safety: state.safety,
    recovery: connectivity.recovery,
    now,
  });
}

function buildOperatorWorkflows() {
  return buildSupervisionWorkflows({
    missionState: currentMissionState(),
    wpPushState: bridge.getStatus().wpPushState,
    hasArea: Boolean(state.coverage),
    hasPath: state.lastPath.length > 0,
    latestNote: state.operator.notes.at(-1) ?? null,
    lastFault: state.lastFault,
    getWorkflowAck,
  });
}

function buildConnectionState(now = Date.now()) {
  const bridgeStatus = bridge.getStatus();
  const dbOk = db.ping();
  const telemetryStale = isTelemetryStale(now);
  const gpsStatus = getRobotGpsStatus();
  const missionState = currentMissionState();
  const localBaseStationReachable = bridgeStatus.baseStation?.statusOk === true;
  const remoteBaseStationFresh = isRemoteBaseStationFresh(now);
  const remoteBaseStation = remoteBaseStationFresh ? state.remoteBaseStation : null;
  const baseStationReachable = localBaseStationReachable || Boolean(remoteBaseStation);
  const robotReachable = Boolean(state.robot);
  const effectiveBaseStation = localBaseStationReachable
    ? bridgeStatus.baseStation
    : remoteBaseStation;
  const selectedBaseUrl = String(effectiveBaseStation?.selectedUrl ?? bridgeStatus.baseStationUrl ?? '').toLowerCase();
  const discoverySource = String(effectiveBaseStation?.discoverySource ?? '').toLowerCase();
  const baseStationPath = discoverySource === 'remote_status'
    ? 'remote_bridge'
    : discoverySource === 'mdns'
    ? 'field_network'
    : (selectedBaseUrl.includes('192.168.4.1') || selectedBaseUrl.includes('base-station.local')
      ? 'direct_backup'
      : 'field_network');
  const baseStationPathLabel = discoverySource === 'remote_status'
    ? 'Remote bridge'
    : discoverySource === 'mdns'
    ? 'Field network (auto)'
    : (baseStationPath === 'direct_backup' ? 'Direct backup' : 'Field network');

  const backend = {
    state: dbOk ? CONNECTION_STATE.ONLINE : CONNECTION_STATE.DEGRADED,
    dbOk,
    uptimeMs: Date.now() - metrics.startedAt,
    persistence: {
      restoredAt: state.persistence.restoredAt,
      lastSavedAt: state.persistence.lastSavedAt,
      lastSaveReason: state.persistence.lastSaveReason,
      lastSaveError: state.persistence.lastSaveError,
      exists: fs.existsSync(RUNTIME_STATE_PATH),
    },
  };

  const baseStationState = !baseStationReachable
    ? CONNECTION_STATE.OFFLINE
    : (!localBaseStationReachable && Boolean(remoteBaseStation))
      ? CONNECTION_STATE.ONLINE
      : bridgeStatus.degraded
      ? CONNECTION_STATE.DEGRADED
      : CONNECTION_STATE.ONLINE;

  const baseStation = {
    state: baseStationState,
    reachable: baseStationReachable,
    connectionPath: baseStationPath,
    connectionPathLabel: baseStationPathLabel,
    discoverySource: effectiveBaseStation?.discoverySource ?? null,
    mdnsEnabled: effectiveBaseStation?.mdnsEnabled ?? false,
    mdnsNames: effectiveBaseStation?.mdnsNames ?? [],
    mdnsServices: effectiveBaseStation?.mdnsServices ?? [],
    mode: effectiveBaseStation?.mode ?? null,
    stationState: effectiveBaseStation?.state ?? null,
    statusVersion: effectiveBaseStation?.statusVersion ?? null,
    wifiLinkState: effectiveBaseStation?.wifiLinkState ?? null,
    loraLinkState: effectiveBaseStation?.loraLinkState ?? null,
    queueDepth: effectiveBaseStation?.queueDepth ?? null,
    lastCmdId: effectiveBaseStation?.lastCmdId ?? null,
    lastCmdStatus: effectiveBaseStation?.lastCmdStatus ?? null,
    ackCount: effectiveBaseStation?.ackCount ?? null,
    lastAck: effectiveBaseStation?.lastAck ?? null,
    lastAckParsed: effectiveBaseStation?.lastAckParsed ?? null,
    lastLoRa: effectiveBaseStation?.lastLoRa ?? null,
    lastSuccessAt: localBaseStationReachable
      ? (bridgeStatus.lastSuccessAt ?? null)
      : (remoteBaseStation?.receivedAt ?? null),
    lastErrorAt: localBaseStationReachable ? (bridgeStatus.lastErrorAt ?? null) : null,
    lastError: localBaseStationReachable
      ? (bridgeStatus.lastCmdError ?? bridgeStatus.baseStation?.statusError ?? null)
      : null,
    degraded: localBaseStationReachable ? Boolean(bridgeStatus.degraded) : false,
    consecutiveFailures: bridgeStatus.consecutiveFailures ?? 0,
    remoteFresh: remoteBaseStationFresh,
    remoteAgeMs: remoteBaseStationAgeMs(now),
    remoteOnly: !localBaseStationReachable && Boolean(remoteBaseStation),
  };

  const robotState = !robotReachable
    ? CONNECTION_STATE.OFFLINE
    : telemetryStale
      ? CONNECTION_STATE.STALE
      : gpsStatus.ready
        ? CONNECTION_STATE.ONLINE
        : CONNECTION_STATE.DEGRADED;

  const robot = {
    state: robotState,
    reachable: robotReachable,
    telemetryStale,
    ageMs: telemetryAgeMs(now),
    robotState: state.robot?.state ?? null,
    gpsReady: gpsStatus.ready,
    gpsReason: gpsStatus.reason,
    gpsFix: state.robot?.gpsFix ?? null,
    gpsHdop: state.robot?.gpsHdop ?? null,
    gpsSat: state.robot?.gpsSat ?? null,
    source: state.robot?.source ?? null,
    timestampMs: state.robot?.timestampMs ?? null,
  };

  const gatewayLinkState = effectiveBaseStation?.loraLinkState ?? null;
  const gatewayLinkToken = String(gatewayLinkState ?? '').trim().toLowerCase();
  const gatewayLastAckAgeMs = Number.isFinite(Number(effectiveBaseStation?.lastAckAgeMs))
    ? Number(effectiveBaseStation.lastAckAgeMs)
    : null;
  const gatewayLastLoRaAgeMs = Number.isFinite(Number(effectiveBaseStation?.lastLoRaAgeMs))
    ? Number(effectiveBaseStation.lastLoRaAgeMs)
    : null;
  const gatewayLastAck = gatewayLastAckAgeMs == null || gatewayLastAckAgeMs <= 3000
    ? (effectiveBaseStation?.lastAck ?? null)
    : null;
  const gatewayLastLoRa = gatewayLastLoRaAgeMs == null || gatewayLastLoRaAgeMs <= 4500
    ? (effectiveBaseStation?.lastLoRa ?? null)
    : null;
  const gatewayEvidence = robotReachable && !telemetryStale
    ? 'robot-telemetry'
    : gatewayLastAck
      ? 'lora-ack'
      : gatewayLastLoRa
        ? 'lora-traffic'
        : null;
  const gatewayReachable = baseStationReachable && (
    ['online', 'connected', 'ready'].includes(gatewayLinkToken) || Boolean(gatewayEvidence)
  );
  const gatewayWorking = gatewayReachable && (
    ['online', 'connected', 'ready'].includes(gatewayLinkToken)
      || ['robot-telemetry', 'lora-ack'].includes(gatewayEvidence ?? '')
  );
  const gatewayState = !baseStationReachable
    ? CONNECTION_STATE.OFFLINE
    : gatewayWorking
      ? CONNECTION_STATE.ONLINE
      : (Boolean(gatewayEvidence) || ['degraded', 'idle', 'stale'].includes(gatewayLinkToken))
        ? CONNECTION_STATE.DEGRADED
        : CONNECTION_STATE.OFFLINE;

  const gateway = {
    state: gatewayState,
    reachable: gatewayReachable,
    working: gatewayWorking,
    linkState: gatewayLinkState,
    evidence: gatewayEvidence,
    reason: !baseStationReachable
      ? 'Base station is not reachable'
      : gatewayEvidence === 'robot-telemetry'
        ? 'Robot telemetry is flowing through the gateway'
        : gatewayEvidence === 'lora-ack'
          ? 'Gateway is acknowledging LoRa commands'
          : gatewayEvidence === 'lora-traffic'
            ? 'Base station is receiving LoRa traffic from the gateway'
            : (gatewayLinkToken === 'degraded' ? 'LoRa link is degraded' : 'No recent gateway traffic seen'),
    lastAck: gatewayLastAck,
    lastLoRa: gatewayLastLoRa,
    lastSeenAt: gatewayEvidence === 'robot-telemetry'
      ? (state.robot?.timestampMs ?? null)
      : (baseStation.lastSuccessAt ?? null),
  };

  const commandTransportAvailable = localBaseStationReachable || remoteBaseStationFresh;
  const commandPathReady = dbOk && commandTransportAvailable && robotReachable && !telemetryStale;
  const commandPath = {
    state: commandPathReady ? CONNECTION_STATE.READY : (baseStationReachable ? CONNECTION_STATE.DEGRADED : CONNECTION_STATE.OFFLINE),
    ready: commandPathReady,
    transportAvailable: commandTransportAvailable,
    wpPushState: bridgeStatus.wpPushState,
    lastCmd: bridgeStatus.lastCmd ?? state.lastCommand ?? null,
    lastCommandId: state.lastCommandId,
    lastCommandStatus: state.lastCommandStatus,
    bridgeLastCommandId: bridgeStatus.baseStation?.lastCmdId ?? null,
    bridgeLastCommandStatus: bridgeStatus.baseStation?.lastCmdStatus ?? null,
    lastTransport: state.lastCommandId
      ? buildCommandTransportStatus(state.lastCommandId, state.lastCommandStatus, bridgeStatus)
      : null,
    lastCmdAt: bridgeStatus.lastCmdAt ?? (state.lastCommandAt || null),
    lastCmdError: bridgeStatus.lastCmdError ?? null,
    missionState,
  };

  const hasRecoverablePath = Array.isArray(state.lastPath) && state.lastPath.length > 1;
  const robotMode = String(state.robot?.state ?? '').toUpperCase();
  const missionRecoveryNeeded =
    hasRecoverablePath &&
    [MISSION_STATE.CONFIGURING, MISSION_STATE.RUNNING, MISSION_STATE.PAUSED].includes(missionState) &&
    (
      !robotReachable ||
      bridgeStatus.wpPushState !== 'committed' ||
      (missionState === MISSION_STATE.RUNNING && robotMode !== 'AUTO') ||
      (missionState === MISSION_STATE.PAUSED && !['PAUSE', 'ERROR', 'ESTOP'].includes(robotMode))
    );
  const missionRecoveryReason = missionRecoveryNeeded
    ? (
      !robotReachable
        ? 'Robot telemetry must reconnect before the mission can continue'
        : bridgeStatus.wpPushState !== 'committed'
          ? 'The mission path should be re-committed to the robot'
          : missionState === MISSION_STATE.RUNNING
            ? `The backend expected AUTO but the robot reports ${robotMode || 'UNKNOWN'}`
            : `The backend expected a paused mission but the robot reports ${robotMode || 'UNKNOWN'}`
    )
    : null;

  let overallState = CONNECTION_STATE.ONLINE;
  let reason = null;
  if (!dbOk) {
    overallState = CONNECTION_STATE.DEGRADED;
    reason = 'Server database is unavailable';
  } else if (!baseStationReachable) {
    overallState = CONNECTION_STATE.DEGRADED;
    reason = 'Backend cannot reach the base station';
  } else if (!robotReachable) {
    overallState = CONNECTION_STATE.DEGRADED;
    reason = 'Robot telemetry has not been received yet';
  } else if (telemetryStale) {
    overallState = CONNECTION_STATE.DEGRADED;
    reason = 'Robot telemetry is stale';
  } else if (!gpsStatus.ready && [MISSION_STATE.CONFIGURING, MISSION_STATE.RUNNING, MISSION_STATE.PAUSED].includes(missionState)) {
    overallState = CONNECTION_STATE.DEGRADED;
    reason = gpsStatus.reason ?? 'Robot GPS readiness is not met';
  } else if (bridgeStatus.degraded) {
    overallState = CONNECTION_STATE.DEGRADED;
    reason = 'Base station link is degraded';
  }

  return {
    overall: {
      state: overallState,
      ready: overallState === CONNECTION_STATE.ONLINE,
      reason,
      missionState,
      connectionPath: baseStationPath,
      connectionPathLabel: baseStationPathLabel,
    },
    backend,
    baseStation,
    gateway,
    robot,
    commandPath,
    recovery: {
      needed: missionRecoveryNeeded,
      reason: missionRecoveryReason,
      lastPathPoints: hasRecoverablePath ? state.lastPath.length : 0,
      wpPushState: bridgeStatus.wpPushState ?? null,
      robotState: state.robot?.state ?? null,
    },
  };
}

function isIpv4Host(hostname) {
  return /^\d{1,3}(?:\.\d{1,3}){3}$/.test(String(hostname ?? '').trim());
}

function resolveManualCommandUrl(bridgeStatus) {
  const gatewayManualUrl = typeof bridgeStatus?.baseStation?.gatewayManualUrl === 'string'
    ? bridgeStatus.baseStation.gatewayManualUrl.trim()
    : '';
  if (gatewayManualUrl) {
    return gatewayManualUrl;
  }

  const selectedUrl = bridgeStatus?.baseStation?.selectedUrl ?? bridgeStatus?.baseStationUrl ?? null;
  const discoverySource = String(bridgeStatus?.baseStation?.discoverySource ?? '').toLowerCase();
  const mdnsServices = Array.isArray(bridgeStatus?.baseStation?.mdnsServices)
    ? bridgeStatus.baseStation.mdnsServices
    : [];

  if (!selectedUrl || discoverySource === 'remote_status') {
    return null;
  }

  try {
    const parsed = new URL(selectedUrl);
    if (!parsed.hostname.endsWith('.local')) {
      return selectedUrl;
    }
  } catch {
    return selectedUrl;
  }

  const candidateUrls = mdnsServices
    .flatMap((service) => Array.isArray(service?.urls) ? service.urls : [])
    .filter((url) => typeof url === 'string' && url);

  for (const candidateUrl of candidateUrls) {
    try {
      const parsed = new URL(candidateUrl);
      if (isIpv4Host(parsed.hostname)) {
        return candidateUrl;
      }
    } catch {
      // Ignore malformed candidate URLs and continue.
    }
  }

  for (const candidateUrl of candidateUrls) {
    try {
      const parsed = new URL(candidateUrl);
      if (!parsed.hostname.endsWith('.local')) {
        return candidateUrl;
      }
    } catch {
      // Ignore malformed candidate URLs and continue.
    }
  }

  return selectedUrl;
}

async function maybeRunScheduledMission() {
  const automation = getAutomationSnapshot();
  if (!automation.enabled || !automation.scheduledRunAt) return;
  if (scheduledMissionInFlight) return;
  if (Date.now() < automation.scheduledRunAt) return;

  scheduledMissionInFlight = true;
  try {
    const result = currentMissionState() === MISSION_STATE.PAUSED
      ? await resumeMissionAction('mission.schedule')
      : await startMissionAction('mission.schedule');

    state.automation.lastTriggeredAt = Date.now();
    if (!result.ok) {
      clearAutomationSchedule('schedule blocked', result.error ?? 'Scheduled autonomy could not start');
      scheduleRuntimeStateSave('mission.schedule.failed', { immediate: true });
    } else {
      scheduleRuntimeStateSave('mission.schedule.started', { immediate: true });
    }

    publishSupervision();
    publishOperator();
  } finally {
    scheduledMissionInFlight = false;
  }
}

function buildSupervisionSummary() {
  const now = Date.now();
  const robotAgeMs = telemetryAgeMs(now);
  const gpsStatus = getRobotGpsStatus();
  const connectivity = buildConnectionState(now);
  const demoReadiness = resolveDemoReadiness();
  const demoConfig = getDemoConfig();
  const loraStatus = bridge.getStatus();
  return {
    mission: mission.publicMission(),
    lora: bridge.getStatus(),
    robot: state.robot
      ? {
          lat: state.robot.lat,
          lon: state.robot.lon,
          heading: state.robot.heading,
          speed: state.robot.speed,
          motor: state.robot.motor ?? null,
          prox: state.robot.prox ?? null,
          disp: state.robot.disp ?? null,
          gpsFix: state.robot.gpsFix,
          gpsHdop: state.robot.gpsHdop,
          gpsSat: state.robot.gpsSat,
          gpsReady: gpsStatus.ready,
          gpsReason: gpsStatus.reason,
          state: state.robot.state,
          source: state.robot.source,
          timestampMs: state.robot.timestampMs,
          ageMs: robotAgeMs,
          stale: isTelemetryStale(now),
        }
      : null,
    homePoint: cloneJsonSafe(state.homePoint, null),
    coverage: state.coverage ? coverageStats(state.coverage) : null,
    safety: {
      telemetryFailsafeEnabled: TELEMETRY_FAILSAFE_ENABLED,
      telemetryFailsafeAction: resolveTelemetryFailsafeAction(),
      telemetryFailsafeAt: state.safety.telemetryFailsafeAt || null,
      telemetryFailsafeReason: state.safety.telemetryFailsafeReason,
      telemetryFailsafeCooldownMs: TELEMETRY_FAILSAFE_COOLDOWN_MS,
      gpsFailsafeEnabled: GPS_FAILSAFE_ENABLED,
      gpsFailsafeAction: resolveGpsFailsafeAction(),
      gpsFailsafeAt: state.safety.gpsFailsafeAt || null,
      gpsFailsafeReason: state.safety.gpsFailsafeReason,
      gpsFailsafeCooldownMs: GPS_FAILSAFE_COOLDOWN_MS,
      geofenceFailsafeEnabled: GEOFENCE_FAILSAFE_ENABLED,
      geofenceFailsafeAction: resolveGeofenceFailsafeAction(),
      geofenceFailsafeAt: state.safety.geofenceFailsafeAt || null,
      geofenceFailsafeReason: state.safety.geofenceFailsafeReason,
      geofenceFailsafeCooldownMs: GEOFENCE_FAILSAFE_COOLDOWN_MS,
    },
    demo: {
      enabled: Boolean(state.demo?.enabled),
      updatedAt: state.demo?.updatedAt || null,
      source: state.demo?.source ?? 'default',
      config: demoConfig,
      spots: state.demo?.spots ?? { start: null, end: null },
      spotGpsStatus: getDemoSpotStatuses(),
      obstacle: state.demo?.obstacle ?? null,
      readiness: demoReadiness,
      diagnostics: {
        missionState: currentMissionState(),
        wpPushState: loraStatus.wpPushState ?? 'none',
        loraDegraded: Boolean(loraStatus.degraded),
        loraLastError: loraStatus.lastCmdError ?? null,
        commandPathState: connectivity?.commandPath?.state ?? null,
      },
    },
    automation: getAutomationSnapshot(now),
    connectivity,
    alerts: buildAlerts(now),
    allowedActions: buildAllowedActions(),
    workflows: buildOperatorWorkflows(),
    notes: state.operator.notes,
  };
}

async function enforceDemoObstaclePolicy(triggerSource = 'telemetry') {
  if (!state.demo?.enabled) return;
  const config = getDemoConfig();
  if (!config.obstaclePolicyEnabled) return;
  if (!mission.isRunning()) return;
  if (!state.robot?.prox) return;

  const left = Number(state.robot.prox.left);
  const right = Number(state.robot.prox.right);
  const leftValid = Number.isFinite(left) && left > 0;
  const rightValid = Number.isFinite(right) && right > 0;
  if (!leftValid && !rightValid) return;

  const nearest = leftValid && rightValid ? Math.min(left, right) : (leftValid ? left : right);
  const now = Date.now();
  const cooldownUntil = Number(state.demo?.obstacle?.cooldownUntil ?? 0);
  if (now < cooldownUntil) return;

  if (nearest <= config.obstacleStopCm) {
    const stop = await dispatchCommand(CMD.STOP, {
      syncMission: false,
      source: `demo.obstacle.stop.${triggerSource}`,
      waitForAck: false,
      waitForState: false,
    });
    if (stop.ok) {
      state.demo.obstacle = {
        active: true,
        mode: 'stop',
        side: null,
        nearestCm: nearest,
        at: now,
        cooldownUntil: now + config.obstacleCooldownMs,
        note: 'Temporary stop triggered by proximity',
      };
      scheduleRuntimeStateSave('demo.obstacle.stop');
      publishSupervision();
    }
    return;
  }

  if (nearest <= config.obstacleSidestepCm) {
    const sidestepSide = (leftValid && rightValid)
      ? (left <= right ? 'RIGHT' : 'LEFT')
      : (leftValid ? 'RIGHT' : 'LEFT');
    const sidestepCmd = sidestepSide === 'LEFT' ? CMD.LEFT : CMD.RIGHT;

    const move = await dispatchCommand(sidestepCmd, {
      syncMission: false,
      source: `demo.obstacle.sidestep.${triggerSource}`,
      waitForAck: false,
      waitForState: false,
    });

    if (move.ok) {
      await sleep(Math.max(80, config.obstacleSidestepMs));
      await dispatchCommand(CMD.STOP, {
        syncMission: false,
        source: `demo.obstacle.sidestep-stop.${triggerSource}`,
        waitForAck: false,
        waitForState: false,
      });

      state.demo.obstacle = {
        active: true,
        mode: 'sidestep',
        side: sidestepSide,
        nearestCm: nearest,
        at: now,
        cooldownUntil: now + config.obstacleCooldownMs,
        note: `Temporary ${sidestepSide.toLowerCase()} sidestep triggered by proximity`,
      };
      scheduleRuntimeStateSave('demo.obstacle.sidestep');
      publishSupervision();
    }
    return;
  }

  if (state.demo?.obstacle?.active) {
    state.demo.obstacle = {
      active: false,
      mode: null,
      side: null,
      nearestCm: nearest,
      at: now,
      cooldownUntil: Number(state.demo?.obstacle?.cooldownUntil ?? 0),
      note: 'Clear',
    };
  }
}

async function enforceGeofenceFailsafePolicy(triggerSource = 'telemetry') {
  if (!GEOFENCE_FAILSAFE_ENABLED) return;
  if (safetyMonitorInFlight) return;
  if (!mission.isRunning()) return;
  if (!state.robot || !state.coverage) return;
  const demoConfig = getDemoConfig();
  const geofenceToleranceM = state.demo?.enabled ? demoConfig.geofenceToleranceM : GEOFENCE_TOLERANCE_M;
  if (isRobotInsideCoverageArea(state.robot, { toleranceM: geofenceToleranceM })) return;

  const now = Date.now();
  if ((now - (state.safety.geofenceFailsafeAt || 0)) < GEOFENCE_FAILSAFE_COOLDOWN_MS) {
    return;
  }

  const action = (state.demo?.enabled && GEOFENCE_DEMO_SOFT_ACTION)
    ? CMD.PAUSE
    : resolveGeofenceFailsafeAction();
  safetyMonitorInFlight = true;
  try {
    const dispatched = await dispatchCommand(action, {
      syncMission: false,
      source: `failsafe.geofence_breach.${triggerSource}`,
    });

    if (!dispatched.ok) {
      db.logEvent(mission.currentId(), EVENT_TYPE.SAFETY_ACTION, {
        policy: 'geofence_breach',
        action,
        ok: false,
        error: dispatched.error ?? 'dispatch_failed',
      });
      return;
    }

    const actionAt = Date.now();
    state.safety.geofenceFailsafeAt = actionAt;
    state.safety.geofenceFailsafeAction = action;
    state.safety.geofenceFailsafeReason = 'geofence_breach';
    state.lastFault = {
      fault: FAULT_CODE.GENERIC,
      action: action === CMD.ESTOP ? FAULT_ACTION.ESTOP : FAULT_ACTION.PAUSE,
      at: actionAt,
    };

    if (action === CMD.ESTOP) {
      if ([MISSION_STATE.RUNNING, MISSION_STATE.PAUSED].includes(currentMissionState())) {
        try {
          mission.abort('failsafe:geofence_breach', buildFinalMissionStats());
        } catch (_) {
          // ignore invalid transition if mission already terminal
        }
      }
      bridge.resetWpState();
    } else if (action === CMD.PAUSE && mission.isRunning()) {
      try {
        mission.pause('failsafe:geofence_breach');
      } catch (_) {
        // ignore invalid transition if mission already changed
      }
    }

    publish(WS_EVENT.FAULT_RECEIVED, {
      fault: 'GEOFENCE_BREACH',
      action,
      at: actionAt,
      source: 'failsafe',
    });

    db.logEvent(mission.currentId(), EVENT_TYPE.SAFETY_ACTION, {
      policy: 'geofence_breach',
      action,
      ok: true,
      robot: {
        lat: state.robot.lat,
        lon: state.robot.lon,
      },
    });

    metrics.safetyActions += 1;
    scheduleRuntimeStateSave('failsafe.telemetry', { immediate: true });
    publishSupervision();
    publishOperator();
  } finally {
    safetyMonitorInFlight = false;
  }
}

async function enforceTelemetryFailsafePolicy() {
  if (!TELEMETRY_FAILSAFE_ENABLED) return;
  if (safetyMonitorInFlight) return;
  if (!mission.isRunning()) return;
  if (!isTelemetryStale()) return;

  const now = Date.now();
  if ((now - (state.safety.telemetryFailsafeAt || 0)) < TELEMETRY_FAILSAFE_COOLDOWN_MS) {
    return;
  }

  const action = resolveTelemetryFailsafeAction();
  safetyMonitorInFlight = true;

  try {
    const dispatched = await dispatchCommand(action, {
      syncMission: false,
      source: 'failsafe.telemetry_stale',
    });

    if (!dispatched.ok) {
      db.logEvent(mission.currentId(), EVENT_TYPE.SAFETY_ACTION, {
        policy: 'telemetry_stale',
        action,
        ok: false,
        error: dispatched.error ?? 'dispatch_failed',
      });
      return;
    }

    const actionAt = Date.now();
    state.safety.telemetryFailsafeAt = actionAt;
    state.safety.telemetryFailsafeAction = action;
    state.safety.telemetryFailsafeReason = 'telemetry_stale';

    state.lastFault = {
      fault: FAULT_CODE.GENERIC,
      action: action === CMD.ESTOP ? FAULT_ACTION.ESTOP : FAULT_ACTION.PAUSE,
      at: actionAt,
    };

    if (action === CMD.ESTOP) {
      if ([MISSION_STATE.RUNNING, MISSION_STATE.PAUSED].includes(currentMissionState())) {
        try {
          mission.abort('failsafe:telemetry_stale', buildFinalMissionStats());
        } catch (_) {
          // ignore invalid transition if mission already terminal
        }
      }
      bridge.resetWpState();
    } else if (action === CMD.PAUSE && mission.isRunning()) {
      try {
        mission.pause('failsafe:telemetry_stale');
      } catch (_) {
        // ignore invalid transition if mission already changed
      }
    }

    publish(WS_EVENT.FAULT_RECEIVED, {
      fault: 'TELEMETRY_STALE',
      action,
      at: actionAt,
      source: 'failsafe',
    });

    db.logEvent(mission.currentId(), EVENT_TYPE.SAFETY_ACTION, {
      policy: 'telemetry_stale',
      action,
      ok: true,
      staleAgeMs: telemetryAgeMs(actionAt),
    });

    metrics.safetyActions += 1;

    publishSupervision();
    publishOperator();
  } finally {
    safetyMonitorInFlight = false;
  }
}

async function enforceGpsFailsafePolicy() {
  if (!GPS_FAILSAFE_ENABLED) return;
  if (safetyMonitorInFlight) return;
  if (!mission.isRunning()) return;
  if (isTelemetryStale()) return;

  const gpsStatus = getRobotGpsStatus();
  if (gpsStatus.ready) return;

  const now = Date.now();
  if ((now - (state.safety.gpsFailsafeAt || 0)) < GPS_FAILSAFE_COOLDOWN_MS) {
    return;
  }

  const action = resolveGpsFailsafeAction();
  safetyMonitorInFlight = true;

  try {
    const dispatched = await dispatchCommand(action, {
      syncMission: false,
      source: 'failsafe.gps_degraded',
      waitForAck: true,
      waitForState: true,
    });

    if (!dispatched.ok) {
      db.logEvent(mission.currentId(), EVENT_TYPE.SAFETY_ACTION, {
        policy: 'gps_degraded',
        action,
        ok: false,
        error: dispatched.error ?? 'dispatch_failed',
      });
      return;
    }

    const actionAt = Date.now();
    state.safety.gpsFailsafeAt = actionAt;
    state.safety.gpsFailsafeAction = action;
    state.safety.gpsFailsafeReason = gpsStatus.reason;
    state.lastFault = {
      fault: FAULT_CODE.GPS_LOSS,
      action: action === CMD.ESTOP ? FAULT_ACTION.ESTOP : FAULT_ACTION.PAUSE,
      at: actionAt,
    };

    if (action === CMD.ESTOP) {
      if ([MISSION_STATE.RUNNING, MISSION_STATE.PAUSED].includes(currentMissionState())) {
        try {
          mission.abort('failsafe:gps_degraded', buildFinalMissionStats());
        } catch (_) {
          // ignore invalid transition if mission already terminal
        }
      }
      bridge.resetWpState();
    } else if (action === CMD.PAUSE && mission.isRunning()) {
      try {
        mission.pause('failsafe:gps_degraded');
      } catch (_) {
        // ignore invalid transition if mission already changed
      }
    }

    publish(WS_EVENT.FAULT_RECEIVED, {
      fault: FAULT_CODE.GPS_LOSS,
      action,
      at: actionAt,
      source: 'failsafe',
      reason: gpsStatus.reason,
    });

    db.logEvent(mission.currentId(), EVENT_TYPE.SAFETY_ACTION, {
      policy: 'gps_degraded',
      action,
      ok: true,
      reason: gpsStatus.reason,
      gps: {
        fix: state.robot?.gpsFix ?? null,
        hdop: state.robot?.gpsHdop ?? null,
        sat: state.robot?.gpsSat ?? null,
      },
    });

    metrics.safetyActions += 1;
    scheduleRuntimeStateSave('failsafe.gps', { immediate: true });
    publishSupervision();
    publishOperator();
  } finally {
    safetyMonitorInFlight = false;
  }
}

function publishSupervision() {
  publish(WS_EVENT.SUPERVISION_UPDATED, buildSupervisionSummary());
}

function publishOperator() {
  publish(WS_EVENT.OPERATOR_UPDATED, {
    workflows: buildOperatorWorkflows(),
    notes: state.operator.notes,
    demo: state.demo,
    automation: getAutomationSnapshot(),
  });
}

restoreRuntimeState();
scheduleRuntimeStateSave('startup.reconciled', { immediate: true });

function buildFinalMissionStats() {
  const stats = state.coverage ? coverageStats(state.coverage) : null;
  return {
    coveragePct: getCoveragePct(stats),
    faultCount: mission.publicMission()?.faultCount ?? 0,
    cmdCount: mission.publicMission()?.cmdCount ?? 0,
  };
}

function arbitrateCommand(cmd) {
  if (Object.values(CMD).includes(cmd)) {
    return { ok: true };
  }

  if (isWaypointCommand(cmd)) {
    return { ok: true };
  }

  return { ok: false, status: 400, error: `Unsupported command: ${cmd}` };
}

function syncMissionToCommand(cmd) {
  if (cmd === CMD.ESTOP) {
    if (mission.isActive()) {
      try {
        mission.abort('operator_estop', buildFinalMissionStats());
      } catch (_) {
        // ignore invalid transition if mission already terminal
      }
    }
    bridge.resetWpState();
    scheduleRuntimeStateSave('command.estop', { immediate: true });
    return;
  }

  if (cmd === CMD.PAUSE) {
    if (mission.isRunning()) {
      try {
        mission.pause('command:PAUSE');
      } catch (_) {
        // ignore invalid transition if mission already paused/terminal
      }
    }
    scheduleRuntimeStateSave('command.pause', { immediate: true });
    return;
  }

  if (cmd === CMD.AUTO) {
    const missionState = currentMissionState();
    try {
      if (missionState === MISSION_STATE.CONFIGURING) {
        mission.start();
      } else if (missionState === MISSION_STATE.PAUSED) {
        mission.resume();
      }
    } catch (_) {
      // ignore invalid transition if mission state changed concurrently
    }
    scheduleRuntimeStateSave('command.auto', { immediate: true });
    return;
  }

  if (cmd === CMD.MANUAL || DRIVE_COMMANDS.has(cmd)) {
    if (mission.isRunning()) {
      try {
        mission.pause('command:manual_override');
      } catch (_) {
        // ignore invalid transition if mission is no longer running
      }
    }
    scheduleRuntimeStateSave('command.manual_override');
    return;
  }

  if (cmd === CMD.RESET) {
    const missionState = currentMissionState();
    if ([MISSION_STATE.ABORTED, MISSION_STATE.ERROR, MISSION_STATE.COMPLETED].includes(missionState)) {
      mission.reset();
      bridge.resetWpState();
    }
    clearResetRecoveryState();
    publishSupervision();
    publishOperator();
    scheduleRuntimeStateSave('command.reset', { immediate: true });
  }
}

function commandRequiresStrictConfirmation(cmd) {
  return [CMD.PAUSE, CMD.ESTOP, CMD.RESET].includes(cmd);
}

function expectedRobotStateForCommand(cmd) {
  switch (cmd) {
    case CMD.AUTO:
      return ROBOT_STATE.AUTO;
    case CMD.PAUSE:
      return ROBOT_STATE.PAUSE;
    case CMD.ESTOP:
      return ROBOT_STATE.ESTOP;
    case CMD.MANUAL:
      return ROBOT_STATE.MANUAL;
    case CMD.RESET:
      return ROBOT_STATE.PAUSE;
    default:
      return null;
  }
}

async function waitForRobotState(expectedState, options = {}) {
  if (!expectedState) {
    return { ok: true, state: null };
  }

  const {
    afterTimestampMs = 0,
    timeoutMs = COMMAND_STATE_CONFIRM_TIMEOUT_MS,
    pollMs = COMMAND_STATE_CONFIRM_POLL_MS,
  } = options;

  const startedAt = Date.now();
  while ((Date.now() - startedAt) < timeoutMs) {
    if (state.robot?.timestampMs >= afterTimestampMs && state.robot?.state === expectedState) {
      return {
        ok: true,
        state: state.robot.state,
        timestampMs: state.robot.timestampMs,
      };
    }
    await sleep(pollMs);
  }

  return {
    ok: false,
    error: `Timed out waiting for robot telemetry state ${expectedState}`,
    currentState: state.robot?.state ?? null,
    lastTelemetryAt: state.robot?.timestampMs ?? null,
  };
}

function setDemoMode(enabled, source = 'operator', configPatch = null) {
  const currentConfig = getDemoConfig();
  const mergedConfig = configPatch && typeof configPatch === 'object'
    ? {
        ...currentConfig,
        ...configPatch,
      }
    : currentConfig;

  state.demo = {
    ...state.demo,
    enabled: Boolean(enabled),
    config: mergedConfig,
    updatedAt: Date.now(),
    source,
  };
  publishOperator();
  publishSupervision();
  scheduleRuntimeStateSave('demo-mode.update', { immediate: true });
  return state.demo;
}

function markDemoSpot(kind, note = null, source = 'operator') {
  const normalizedKind = kind === 'end' ? 'end' : 'start';
  const robot = state.robot
    ? {
        lat: state.robot.lat ?? null,
        lon: state.robot.lon ?? null,
        heading: state.robot.heading ?? null,
        speed: state.robot.speed ?? null,
        state: state.robot.state ?? null,
        gpsFix: state.robot.gpsFix ?? null,
        gpsHdop: state.robot.gpsHdop ?? null,
        gpsSat: state.robot.gpsSat ?? null,
        timestampMs: state.robot.timestampMs ?? null,
      }
    : null;

  const spot = {
    kind: normalizedKind,
    label: normalizedKind === 'start' ? 'Spot A' : 'Spot B',
    note: typeof note === 'string' && note.trim() ? note.trim() : null,
    source,
    markedAt: Date.now(),
    robot,
  };

  state.demo = {
    ...state.demo,
    spots: {
      ...(state.demo?.spots ?? { start: null, end: null }),
      [normalizedKind]: spot,
    },
    updatedAt: Date.now(),
    source,
  };

  db.logEvent(mission.currentId(), EVENT_TYPE.OPERATOR_NOTE_ADDED, {
    category: 'demo.spot',
    spot: normalizedKind,
    note: spot.note,
    source,
  });

  publishOperator();
  publishSupervision();
  scheduleRuntimeStateSave('demo-mode.spot', { immediate: true });
  return spot;
}

function buildDemoBoundaryFromSpots(startSpot, endSpot, laneWidthM = 3.0, minSpotDistanceM = DEMO_MIN_SPOT_DISTANCE_M) {
  const start = startSpot?.robot;
  const end = endSpot?.robot;
  if (
    !start ||
    !end ||
    !Number.isFinite(start.lat) ||
    !Number.isFinite(start.lon) ||
    !Number.isFinite(end.lat) ||
    !Number.isFinite(end.lon)
  ) {
    return { ok: false, error: 'Both demo spots need saved robot positions before building a demo path' };
  }

  const origin = { lat: start.lat, lon: start.lon };
  const startLocal = latLonToLocal({ lat: start.lat, lon: start.lon }, origin);
  const endLocal = latLonToLocal({ lat: end.lat, lon: end.lon }, origin);
  const dx = endLocal.x - startLocal.x;
  const dy = endLocal.y - startLocal.y;
  const length = Math.hypot(dx, dy);
  if (!Number.isFinite(length) || length < minSpotDistanceM) {
    return {
      ok: false,
      error: `Demo spots are too close together to build a demo lane (minimum ${minSpotDistanceM.toFixed(2)} m)`,
    };
  }

  const dirX = dx / length;
  const dirY = dy / length;
  const perpX = -dirY;
  const perpY = dirX;
  const halfWidth = Math.max(DEMO_MIN_LANE_WIDTH_M / 2, laneWidthM / 2);
  const leadIn = Math.min(1.0, length * 0.1);
  const tailX = dirX * leadIn;
  const tailY = dirY * leadIn;

  const cornersLocal = [
    { x: startLocal.x - tailX + perpX * halfWidth, y: startLocal.y - tailY + perpY * halfWidth },
    { x: endLocal.x + tailX + perpX * halfWidth, y: endLocal.y + tailY + perpY * halfWidth },
    { x: endLocal.x + tailX - perpX * halfWidth, y: endLocal.y + tailY - perpY * halfWidth },
    { x: startLocal.x - tailX - perpX * halfWidth, y: startLocal.y - tailY - perpY * halfWidth },
  ];

  return {
    ok: true,
    baseStation: { lat: start.lat, lon: start.lon },
    start: { lat: start.lat, lon: start.lon },
    goal: { lat: end.lat, lon: end.lon },
    boundary: cornersLocal.map((point) => localToLatLon(point, origin)),
    laneWidthM: halfWidth * 2,
    laneLengthM: length,
  };
}

function configureActiveArea(baseStation, boundary, cellSizeM, homePoint = null) {
  const cellPlan = resolveInputAreaCellSize(boundary, Number(cellSizeM));
  const capturedHomePoint = normalizeLatLonPoint(homePoint, null)
    ?? normalizeLatLonPoint(state.robot, null)
    ?? normalizeLatLonPoint(baseStation, null)
    ?? normalizeLatLonPoint(state.baseStation, null);

  state.baseStation = baseStation;
  state.homePoint = capturedHomePoint;
  state.boundary = boundary;
  state.coverage = buildCoverageMap(boundary, cellPlan.effectiveCellSizeM);
  state.robot = null;
  state.trail = [];
  state.lastPath = [];
  state.lastArrows = [];
  bridge.resetWpState();
  resetOperatorState();

  const stats = coverageStats(state.coverage);
  publish(WS_EVENT.AREA_UPDATED, { baseStation, boundary, homePoint: capturedHomePoint, stats });

  if (mission.isActive()) {
    const missionState = mission.publicMission()?.state;
    if (missionState === MISSION_STATE.CONFIGURING) {
      try {
        mission.reset();
      } catch (_) { /* ignore */ }
    } else {
      try {
        mission.abort('area_reset', {
          coveragePct: getCoveragePct(stats),
          faultCount: mission.publicMission()?.faultCount ?? 0,
          cmdCount: mission.publicMission()?.cmdCount ?? 0,
        });
      } catch (_) { /* ignore */ }
      try {
        mission.reset();
      } catch (_) { /* ignore */ }
    }
  }

  mission.configure({ baseStation, boundary, cellSizeM: cellPlan.effectiveCellSizeM });
  return { stats, cellPlan, homePoint: capturedHomePoint };
}

function buildMissionPath({ mode = 'coverage', startPoint, goalPoint, coverageWidthM, saltPct, brinePct, homePoint = null, returnToBase = false, sweepDirection = 'auto', preferMinTurns = true }) {
  const normalizedStart = normalizeLatLonPoint(startPoint, null);
  state.homePoint = normalizeLatLonPoint(homePoint, null)
    ?? normalizeLatLonPoint(state.homePoint, null)
    ?? normalizeLatLonPoint(state.robot, null)
    ?? normalizedStart
    ?? normalizeLatLonPoint(state.baseStation, null);

  const basePath = mode === 'coverage'
    ? findCoveragePath(state.coverage, {
        swathWidthM: coverageWidthM,
        startLatLon: normalizedStart ?? startPoint,
        goalLatLon: goalPoint ?? null,
        sweepDirection,
        preferMinTurns,
      })
    : findPath(state.coverage, normalizedStart ?? startPoint, goalPoint);

  if (!basePath.ok) {
    state.lastPath = [];
    state.lastArrows = [];
    return basePath;
  }

  let plannedPoints = Array.isArray(basePath.points) ? basePath.points.slice() : [];
  const meta = {
    ...(basePath.meta ?? {}),
    homePoint: cloneJsonSafe(state.homePoint, null),
    returnToBase: Boolean(returnToBase && state.homePoint),
  };

  if (normalizedStart && plannedPoints.length > 0 && !pointsRoughlyEqual(normalizedStart, plannedPoints[0])) {
    const ingressPath = findPath(state.coverage, normalizedStart, plannedPoints[0]);
    if (ingressPath.ok && Array.isArray(ingressPath.points) && ingressPath.points.length > 0) {
      plannedPoints = mergePlannedSegments(ingressPath.points, plannedPoints);
      meta.ingressPlanned = true;
      meta.ingressPointCount = ingressPath.points.length;
    } else if (ingressPath.reason) {
      meta.ingressPlanned = false;
      meta.ingressReason = ingressPath.reason;
    }
  }

  const resolvedHomePoint = returnToBase ? resolveHomePoint(homePoint) : null;
  if (resolvedHomePoint && plannedPoints.length > 0 && !pointsRoughlyEqual(plannedPoints[plannedPoints.length - 1], resolvedHomePoint)) {
    const returnPath = findPath(state.coverage, plannedPoints[plannedPoints.length - 1], resolvedHomePoint);
    if (returnPath.ok && Array.isArray(returnPath.points) && returnPath.points.length > 0) {
      plannedPoints = mergePlannedSegments(plannedPoints, returnPath.points);
      meta.returnPlanned = true;
      meta.returnPointCount = returnPath.points.length;
    } else if (returnPath.reason) {
      meta.returnPlanned = false;
      meta.returnReason = returnPath.reason;
    }
  }

  state.lastPath = plannedPoints.map((point) => ({
    ...point,
    salt: saltPct,
    brine: brinePct,
  }));
  const arrowSpacingM = mode === 'coverage'
    ? Math.max(2.8, Number(coverageWidthM ?? 0.5) * 7.6)
    : 5;
  state.lastArrows = buildCoverageArrows(state.lastPath, {
    spacingM: arrowSpacingM,
    minArrowSeparationM: Math.max(0.7, arrowSpacingM * 0.32),
    initialOffsetM: Math.min(0.35, arrowSpacingM * 0.16),
  });

  publish(WS_EVENT.PATH_UPDATED, {
    mode,
    coverageWidthM: mode === 'coverage' ? coverageWidthM : null,
    points: state.lastPath,
    arrows: state.lastArrows,
    meta,
  });
  db.logEvent(mission.currentId(), EVENT_TYPE.PATH_PLANNED, {
    mode,
    pointCount: state.lastPath.length,
    saltPct,
    brinePct,
    coverageWidthM: mode === 'coverage' ? coverageWidthM : null,
    returnToBase: Boolean(meta.returnToBase),
    arrowCount: state.lastArrows.length,
  });
  scheduleRuntimeStateSave('path.planned', { immediate: true });
  publishSupervision();
  publishOperator();
  return {
    ...basePath,
    points: state.lastPath,
    meta: {
      ...meta,
      arrowCount: state.lastArrows.length,
      sweepDirection: mode === 'coverage' ? sweepDirection : null,
    },
  };
}

function planCoveragePathForCurrentArea({ startPoint, goalPoint, coverageWidthM, saltPct, brinePct, homePoint = null, returnToBase = true, sweepDirection = 'auto' }) {
  return buildMissionPath({
    mode: 'coverage',
    startPoint,
    goalPoint,
    coverageWidthM,
    saltPct,
    brinePct,
    homePoint,
    returnToBase,
    sweepDirection,
    preferMinTurns: true,
  });
}

function isHardwareLockedForDemo() {
  return Boolean(state.demo?.enabled);
}

function assertHardwareAllowed(actionLabel, options = {}) {
  const { allowWhenDemo = false } = options;
  if (!isHardwareLockedForDemo() || allowWhenDemo) {
    return { ok: true };
  }
  return {
    ok: false,
    status: 409,
    error: `${actionLabel} blocked: demo mode is active`,
  };
}

function isDemoSafeCommand(cmd) {
  return [
    CMD.ESTOP,
    CMD.PAUSE,
    CMD.STOP,
    CMD.DRIVE,
    CMD.MANUAL,
    CMD.RESET,
    CMD.FORWARD,
    CMD.BACKWARD,
    CMD.LEFT,
    CMD.RIGHT,
  ].includes(cmd);
}

function queueDispatchedRemoteCommand({ commandId, cmd, wireCommand = cmd, source, syncMission = true }) {
  const queued = enqueueRemoteCommand({ commandId, cmd: wireCommand, source });
  if (!queued) {
    metrics.commandsFailed += 1;
    const queueError = `Remote base station queue is full (${pendingRemoteCommandCount()}/${REMOTE_COMMAND_MAX_QUEUE})`;
    updateCommandTracking(commandId, {
      cmd,
      source,
      status: COMMAND_STATUS.FAILED,
      error: queueError,
      detail: {
        stage: COMMAND_TRANSPORT_STAGE.FAILED,
        remoteQueued: false,
        queueDepth: pendingRemoteCommandCount(),
      },
    });
    return { ok: false, status: 503, error: queueError };
  }

  if (isWaypointCommand(cmd)) {
    bridge.observeCommand(cmd);
  }

  updateCommandTracking(commandId, {
    cmd,
    source,
    status: COMMAND_STATUS.QUEUED,
    detail: {
      stage: COMMAND_TRANSPORT_STAGE.BACKEND_QUEUED,
      remoteQueued: true,
      queueDepth: pendingRemoteCommandCount(),
      queuedAt: queued.createdAt,
    },
  });
  db.logEvent(mission.currentId(), EVENT_TYPE.COMMAND_SENT, {
    cmd,
    commandId,
    status: COMMAND_STATUS.QUEUED,
    source,
    remoteQueued: true,
  });
  mission.recordCommand();

  if (syncMission) {
    syncMissionToCommand(cmd);
  }

  scheduleRuntimeStateSave(`dispatch.remote.${cmd}`, { immediate: !DRIVE_COMMANDS.has(cmd) });
  publishSupervision();
  publishOperator();
  metrics.commandsDispatched += 1;

  return {
    ok: true,
    status: 202,
    body: 'queued_remote',
    commandId,
    queuedRemote: true,
  };
}

async function dispatchCommand(cmd, options = {}) {
  const {
    syncMission = true,
    source = 'api.command',
    waitForAck = false,
    waitForState = false,
    wireCommand = null,
  } = options;
  const commandId = createCommandId();
  const shouldWaitForAck = waitForAck;
  const explicitlyWaitForState = waitForState;
  const shouldWaitForState = explicitlyWaitForState || commandRequiresStrictConfirmation(cmd);
  const commandText = typeof wireCommand === 'string' && wireCommand.trim()
    ? wireCommand.trim().toUpperCase()
    : compactWireCommandForCmd(cmd);

  const decision = arbitrateCommand(cmd);
  if (!decision.ok) {
    metrics.commandsFailed += 1;
    updateCommandTracking(commandId, {
      cmd,
      source,
      status: COMMAND_STATUS.FAILED,
      error: decision.error,
      detail: { stage: 'arbitration', status: decision.status },
    });
    return { ok: false, status: decision.status, error: decision.error };
  }

  updateCommandTracking(commandId, {
    cmd,
    source,
    status: COMMAND_STATUS.QUEUED,
  });

  const remoteFallbackReady = isRemoteCommandFallbackReady() && !bridge.getStatus().baseStation?.statusOk;
  if (remoteFallbackReady) {
    return queueDispatchedRemoteCommand({ commandId, cmd, wireCommand: commandText, source, syncMission });
  }

  const result = await bridge.sendCommand(commandText, { waitForAck: shouldWaitForAck, commandId, commandSource: source });
  if (!result.ok) {
    const ackTimedOut = shouldWaitForAck && result.status === 504;
    if (ackTimedOut) {
      if (isWaypointCommand(cmd)) {
        bridge.observeCommand(cmd);
      }
      updateCommandTracking(commandId, {
        cmd,
        source,
        status: COMMAND_STATUS.FORWARDED,
        error: null,
        detail: {
          stage: COMMAND_TRANSPORT_STAGE.BASE_STATION_FORWARDED,
          ackTimedOut: true,
          warning: result.error ?? null,
        },
      });
      db.logEvent(mission.currentId(), EVENT_TYPE.COMMAND_SENT, {
        cmd,
        commandId,
        status: COMMAND_STATUS.FORWARDED,
        source,
        ackTimedOut: true,
      });
      mission.recordCommand();
      if (syncMission) {
        syncMissionToCommand(cmd);
      }
      scheduleRuntimeStateSave(`dispatch.${cmd}`, { immediate: !DRIVE_COMMANDS.has(cmd) });
      publishSupervision();
      publishOperator();
      metrics.commandsDispatched += 1;
      return {
        ok: true,
        status: 202,
        body: result.body ?? '',
        commandId,
        ackTimedOut: true,
        warning: result.error ?? null,
      };
    }

    const remoteFallbackReadyAfterFailure = isRemoteCommandFallbackReady() && !bridge.getStatus().baseStation?.statusOk;
    if (remoteFallbackReadyAfterFailure) {
      return queueDispatchedRemoteCommand({ commandId, cmd, wireCommand: commandText, source, syncMission });
    }

    metrics.commandsFailed += 1;
    updateCommandTracking(commandId, {
      cmd,
      source,
      status: COMMAND_STATUS.FAILED,
      error: result.error ?? `Base station command failed (${result.status ?? 'no status'})`,
      detail: result,
    });
    return {
      ok: false,
      status: 502,
      error: result.error ?? `Base station command failed (${result.status ?? 'no status'})`,
      detail: result,
    };
  }

  if (isWaypointCommand(cmd)) {
    bridge.observeCommand(cmd);
  }

  const forwardedStatus = shouldWaitForState
    ? COMMAND_STATUS.FORWARDED
    : (shouldWaitForAck ? COMMAND_STATUS.ACKNOWLEDGED : COMMAND_STATUS.SENT);
  updateCommandTracking(commandId, {
    cmd,
    source,
    status: forwardedStatus,
    bridgeAckSource: result.ack?.source ?? null,
  });
  db.logEvent(mission.currentId(), EVENT_TYPE.COMMAND_SENT, { cmd, commandId, status: forwardedStatus, source });
  mission.recordCommand();

  if (syncMission) {
    syncMissionToCommand(cmd);
  }

  let responseStatus = result.status ?? 200;
  let responseWarning = null;

  if (shouldWaitForState) {
    const expectedState = expectedRobotStateForCommand(cmd);
    const confirmation = await waitForRobotState(expectedState, {
      afterTimestampMs: state.lastCommandAt,
    });
    if (!confirmation.ok) {
      updateCommandTracking(commandId, {
        cmd,
        source,
        status: COMMAND_STATUS.TIMED_OUT,
        error: confirmation.error,
        detail: {
          ...confirmation,
          expectedState,
          stateConfirmationPending: !explicitlyWaitForState,
        },
      });

      if (explicitlyWaitForState) {
        metrics.commandsFailed += 1;
        return {
          ok: false,
          status: 504,
          error: confirmation.error,
          detail: confirmation,
        };
      }

      responseStatus = 202;
      responseWarning = confirmation.error;
    } else {
      updateCommandTracking(commandId, {
        cmd,
        source,
        status: COMMAND_STATUS.APPLIED,
        detail: confirmation,
      });
      if (cmd === CMD.RESET) {
        clearResetRecoveryState();
        publishSupervision();
        publishOperator();
        publish(WS_EVENT.STATE_SNAPSHOT, publicState());
      }
    }
  }

  scheduleRuntimeStateSave(`dispatch.${cmd}`, { immediate: !DRIVE_COMMANDS.has(cmd) });
  publishSupervision();
  publishOperator();

  metrics.commandsDispatched += 1;

  return { ok: true, status: responseStatus, body: result.body, commandId, warning: responseWarning };
}

function buildSampleWaypointSet() {
  if (Array.isArray(state.lastPath) && state.lastPath.length >= 3) {
    return state.lastPath.slice(0, Math.min(3, state.lastPath.length)).map((point) => ({
      lat: Number(point.lat),
      lon: Number(point.lon),
      salt: clampNumber(Number(point.salt ?? 0), 0, 100),
      brine: clampNumber(Number(point.brine ?? 0), 0, 100),
    }));
  }

  if (state.robot && Number.isFinite(Number(state.robot.lat)) && Number.isFinite(Number(state.robot.lon))) {
    const lat = Number(state.robot.lat);
    const lon = Number(state.robot.lon);
    return [
      { lat, lon, salt: 25, brine: 75 },
      { lat: lat + 0.00002, lon, salt: 25, brine: 75 },
      { lat: lat + 0.00002, lon: lon + 0.00002, salt: 25, brine: 75 },
    ];
  }

  return null;
}

async function pushPathWaypoints(rawPoints = null, source = 'lora_bridge', options = {}) {
  const { allowWhenDemo = false } = options;
  const demoCheck = assertHardwareAllowed('Waypoint push', { allowWhenDemo });
  if (!demoCheck.ok) return demoCheck;
  if (![MISSION_STATE.CONFIGURING, MISSION_STATE.PAUSED].includes(currentMissionState())) {
    return { ok: false, status: 409, error: 'Waypoint push allowed only while mission is CONFIGURING or PAUSED' };
  }

  let points = rawPoints;
  if (!points && state.lastPath && state.lastPath.length > 0) {
    points = state.lastPath.map((p) => ({
      lat: p.lat,
      lon: p.lon,
      salt: p.salt ?? 100,
      brine: p.brine ?? 100,
    }));
  }
  if (!points || !Array.isArray(points) || points.length === 0) {
    return { ok: false, status: 400, error: 'No points provided and no cached path plan' };
  }

  const totalPoints = points.length;
  const maxLocalWaypoints = Number(LORA_WIRE.MAX_WAYPOINTS) || 50;
  const truncated = totalPoints > maxLocalWaypoints;
  if (truncated) {
    points = points.slice(0, maxLocalWaypoints);
  }

  if (pathHasZeroDispersion(points)) {
    return { ok: false, status: 409, error: 'Waypoint push blocked: path has 0% salt and 0% brine' };
  }

  const shouldPreferRemoteQueue = isRemoteCommandFallbackReady() && !bridge.getStatus().baseStation?.statusOk;
  const result = shouldPreferRemoteQueue
    ? queueRemoteWaypointPush(points, source)
    : await bridge.pushWaypoints(points);

  if (!result.ok && !shouldPreferRemoteQueue && isRemoteCommandFallbackReady()) {
    const remoteResult = queueRemoteWaypointPush(points, source);
    if (remoteResult.ok) {
      db.logEvent(mission.currentId(), EVENT_TYPE.PATH_PLANNED, {
        wpPushCount: remoteResult.sent,
        source,
        remoteQueued: true,
      });
      scheduleRuntimeStateSave('waypoints.remote_queued', { immediate: true });
      publishSupervision();
      publishOperator();
      return {
        ok: true,
        status: remoteResult.status ?? 202,
        sent: remoteResult.sent,
        points,
        queuedRemote: true,
        truncated,
        totalPoints,
      };
    }
    return remoteResult;
  }

  if (!result.ok) {
    return { ok: false, status: result.status ?? 502, error: result.error, sent: result.sent };
  }
  db.logEvent(mission.currentId(), EVENT_TYPE.PATH_PLANNED, {
    wpPushCount: result.sent,
    source,
    remoteQueued: Boolean(result.queuedRemote),
  });
  scheduleRuntimeStateSave(result.queuedRemote ? 'waypoints.remote_queued' : 'waypoints.committed', { immediate: true });
  publishSupervision();
  publishOperator();
  return {
    ok: true,
    status: result.status ?? 200,
    sent: result.sent,
    points,
    queuedRemote: Boolean(result.queuedRemote),
    truncated,
    totalPoints,
  };
}

async function startMissionAction(source = 'mission.start', options = {}) {
  const { allowWhenDemo = false, allowWeakGps = false } = options;
  const demoCheck = assertHardwareAllowed('Mission start', { allowWhenDemo });
  if (!demoCheck.ok) return demoCheck;
  if (currentMissionState() !== MISSION_STATE.CONFIGURING) {
    return { ok: false, status: 409, error: 'Mission must be CONFIGURING before start' };
  }
  if (pathHasZeroDispersion()) {
    return { ok: false, status: 409, error: 'Mission start blocked: path has 0% salt and 0% brine' };
  }
  const gpsStatus = getRobotGpsStatus();
  if (state.robot && !gpsStatus.ready && !allowWeakGps) {
    return { ok: false, status: 409, error: `Mission start blocked: ${gpsStatus.reason}` };
  }

  const loraStatus = bridge.getStatus();
  if (loraStatus.wpPushState !== 'committed' && state.lastPath && state.lastPath.length > 0) {
    const pushed = await pushPathWaypoints(null, source, { allowWhenDemo });
    if (!pushed.ok) return pushed;
  }

  if (bridge.getStatus().wpPushState !== 'committed') {
    return { ok: false, status: 409, error: 'Mission start requires a committed waypoint set' };
  }

  const autoIssuedAt = Date.now();
  const dispatched = await dispatchCommand(CMD.AUTO, { syncMission: false, source, waitForAck: true, waitForState: false });
  if (!dispatched.ok) {
    return { ok: false, status: dispatched.status ?? 500, error: dispatched.error };
  }

  const autoConfirmed = await waitForRobotState(ROBOT_STATE.AUTO, {
    afterTimestampMs: autoIssuedAt,
    timeoutMs: MISSION_AUTO_CONFIRM_TIMEOUT_MS,
  });
  if (!autoConfirmed.ok) {
    return {
      ok: false,
      status: 504,
      error: `AUTO command not confirmed by robot telemetry (${autoConfirmed.error})`,
      detail: autoConfirmed,
    };
  }

  const current = mission.start();
  state.automation = {
    ...state.automation,
    enabled: false,
    scheduledRunAt: null,
    label: null,
    lastTriggeredAt: Date.now(),
    lastResult: source === 'mission.schedule' ? 'started automatically' : 'started manually',
    lastError: null,
  };
  scheduleRuntimeStateSave('mission.start', { immediate: true });
  publishSupervision();
  publishOperator();
  return { ok: true, status: 200, mission: current };
}

async function pauseMissionAction(reason = null, source = 'mission.pause') {
  if (currentMissionState() !== MISSION_STATE.RUNNING) {
    return { ok: false, status: 409, error: 'Mission must be RUNNING before pause' };
  }
  const dispatched = await dispatchCommand(CMD.PAUSE, { syncMission: false, source, waitForAck: false, waitForState: false });
  if (!dispatched.ok) {
    return { ok: false, status: dispatched.status ?? 500, error: dispatched.error };
  }
  const current = mission.pause(reason);
  scheduleRuntimeStateSave('mission.pause', { immediate: true });
  publishSupervision();
  return { ok: true, status: 200, mission: current };
}

async function resumeMissionAction(source = 'mission.resume') {
  const demoCheck = assertHardwareAllowed('Mission resume');
  if (!demoCheck.ok) return demoCheck;
  if (currentMissionState() !== MISSION_STATE.PAUSED) {
    return { ok: false, status: 409, error: 'Mission must be PAUSED before resume' };
  }
  const gpsStatus = getRobotGpsStatus();
  if (state.robot && !gpsStatus.ready) {
    return { ok: false, status: 409, error: `Mission resume blocked: ${gpsStatus.reason}` };
  }
  if (bridge.getStatus().wpPushState !== 'committed') {
    return { ok: false, status: 409, error: 'Mission resume requires committed waypoints' };
  }
  const autoIssuedAt = Date.now();
  const dispatched = await dispatchCommand(CMD.AUTO, { syncMission: false, source, waitForAck: true, waitForState: false });
  if (!dispatched.ok) {
    return { ok: false, status: dispatched.status ?? 500, error: dispatched.error };
  }

  const autoConfirmed = await waitForRobotState(ROBOT_STATE.AUTO, {
    afterTimestampMs: autoIssuedAt,
    timeoutMs: MISSION_AUTO_CONFIRM_TIMEOUT_MS,
  });
  if (!autoConfirmed.ok) {
    return {
      ok: false,
      status: 504,
      error: `AUTO resume command not confirmed by robot telemetry (${autoConfirmed.error})`,
      detail: autoConfirmed,
    };
  }
  const current = mission.resume();
  state.automation = {
    ...state.automation,
    enabled: false,
    scheduledRunAt: null,
    label: null,
    lastTriggeredAt: Date.now(),
    lastResult: source === 'mission.schedule' ? 'resumed automatically' : 'resumed manually',
    lastError: null,
  };
  scheduleRuntimeStateSave('mission.resume', { immediate: true });
  publishSupervision();
  publishOperator();
  return { ok: true, status: 200, mission: current };
}

async function completeMissionAction(source = 'mission.complete') {
  if (currentMissionState() !== MISSION_STATE.RUNNING) {
    return { ok: false, status: 409, error: 'Mission must be RUNNING before complete' };
  }
  const dispatched = await dispatchCommand(CMD.PAUSE, { syncMission: false, source, waitForAck: false, waitForState: false });
  if (!dispatched.ok) {
    return { ok: false, status: dispatched.status ?? 500, error: dispatched.error };
  }
  const stats = state.coverage ? coverageStats(state.coverage) : {};
  const missionSnapshot = mission.publicMission();
  const current = mission.complete({
    coveragePct: getCoveragePct(stats),
    faultCount: missionSnapshot?.faultCount ?? 0,
    cmdCount: missionSnapshot?.cmdCount ?? 0,
  });
  bridge.resetWpState();
  scheduleRuntimeStateSave('mission.complete', { immediate: true });
  publishSupervision();
  return { ok: true, status: 200, mission: current };
}

async function abortMissionAction(reason = 'operator', source = 'mission.abort') {
  if (![MISSION_STATE.RUNNING, MISSION_STATE.PAUSED].includes(currentMissionState())) {
    return { ok: false, status: 409, error: 'Mission must be RUNNING or PAUSED before abort' };
  }
  const dispatched = await dispatchCommand(CMD.ESTOP, { syncMission: false, source, waitForAck: false, waitForState: false });
  if (!dispatched.ok) {
    return { ok: false, status: dispatched.status ?? 500, error: dispatched.error };
  }
  const stats = state.coverage ? coverageStats(state.coverage) : {};
  const missionSnapshot = mission.publicMission();
  const current = mission.abort(reason, {
    coveragePct: getCoveragePct(stats),
    faultCount: missionSnapshot?.faultCount ?? 0,
    cmdCount: missionSnapshot?.cmdCount ?? 0,
  });
  bridge.resetWpState();
  scheduleRuntimeStateSave('mission.abort', { immediate: true });
  publishSupervision();
  return { ok: true, status: 200, mission: current };
}

function buildTestMenu() {
  return [
    {
      id: 'bridge-status',
      title: 'Bridge Status',
      kind: 'inspect',
      description: 'Refresh the base station and LoRa bridge snapshot.',
      caution: 'safe',
      shortcut: 'S',
      group: 'System',
    },
    {
      id: 'gps-readiness',
      title: 'GPS Readiness',
      kind: 'inspect',
      description: 'Check GPS fix, satellites, HDOP, and autonomy readiness.',
      caution: 'safe',
    },
    {
      id: 'telemetry-snapshot',
      title: 'Telemetry Snapshot',
      kind: 'inspect',
      description: 'Inspect the latest robot telemetry and safety state.',
      caution: 'safe',
      shortcut: '1',
      group: 'System',
    },
    {
      id: 'websocket-test',
      title: 'WebSocket Test',
      kind: 'inspect',
      description: 'Publish a WebSocket test event and report connected live-update clients.',
      caution: 'safe',
      shortcut: 'W',
      group: 'System',
    },
    {
      id: 'state-snapshot',
      title: 'State Snapshot',
      kind: 'inspect',
      description: 'Summarize mission, robot, bridge, and last fault state together.',
      caution: 'safe',
      shortcut: '9',
      group: 'Sensors',
    },
    {
      id: 'gps-snapshot',
      title: 'GPS Snapshot',
      kind: 'inspect',
      description: 'Show GPS fix, position, speed, heading, satellites, and HDOP.',
      caution: 'safe',
      shortcut: '4',
      group: 'Sensors',
    },
    {
      id: 'mode-manual',
      title: 'Mode Manual',
      kind: 'command',
      description: 'Put the robot into MANUAL so direct drive commands are accepted.',
      caution: 'safe',
      shortcut: 'M',
      group: 'Modes',
    },
    {
      id: 'mode-test',
      title: 'Mode Test/Telemetry',
      kind: 'command',
      description: 'Exit MANUAL and return to PAUSE telemetry/test mode.',
      caution: 'safe',
      shortcut: 'T',
      group: 'Modes',
    },
    {
      id: 'mode-auto',
      title: 'Mode Auto',
      kind: 'command',
      description: 'Send AUTO and verify the robot reports AUTO telemetry state.',
      caution: 'caution',
      shortcut: 'U',
      group: 'Modes',
    },
    {
      id: 'ack-pause',
      title: 'Pause ACK',
      kind: 'command',
      description: 'Send PAUSE and verify command ACK plus telemetry state.',
      caution: 'safe',
      shortcut: 'P',
      group: 'Modes',
    },
    {
      id: 'ack-reset',
      title: 'Reset ACK',
      kind: 'command',
      description: 'Send RESET and verify recovery back to PAUSE.',
      caution: 'caution',
      shortcut: 'J',
      group: 'Modes',
    },
    {
      id: 'ack-estop',
      title: 'E-Stop ACK',
      kind: 'command',
      description: 'Send ESTOP and verify robot enters ESTOP.',
      caution: 'danger',
      shortcut: 'E',
      group: 'Modes',
    },
    {
      id: 'drive-forward',
      title: 'Drive Forward',
      kind: 'drive',
      description: 'Send a single FORWARD command for manual motion testing.',
      caution: 'danger',
      shortcut: 'F',
      group: 'Drive',
    },
    {
      id: 'drive-left',
      title: 'Drive Left',
      kind: 'drive',
      description: 'Send a single LEFT command for manual steering checks.',
      caution: 'danger',
      shortcut: 'V',
      group: 'Drive',
    },
    {
      id: 'drive-stop',
      title: 'Drive Stop',
      kind: 'drive',
      description: 'Send STOP to halt manual motion.',
      caution: 'safe',
      shortcut: 'T',
      group: 'Drive',
    },
    {
      id: 'drive-right',
      title: 'Drive Right',
      kind: 'drive',
      description: 'Send a single RIGHT command for manual steering checks.',
      caution: 'danger',
      shortcut: 'N',
      group: 'Drive',
    },
    {
      id: 'drive-backward',
      title: 'Drive Back',
      kind: 'drive',
      description: 'Send a single BACKWARD command for manual motion testing.',
      caution: 'danger',
      shortcut: 'B',
      group: 'Drive',
    },
    {
      id: 'push-sample-waypoints',
      title: 'Push Sample WPs',
      kind: 'transport',
      description: 'Send a tiny sample waypoint set to verify staged waypoint transport.',
      caution: 'caution',
      shortcut: 'C',
      group: 'Transport',
    },
    {
      id: 'commit-current-path',
      title: 'Commit Current Path',
      kind: 'mission',
      description: 'Push the current planned path to the robot as committed waypoints.',
      caution: 'caution',
      shortcut: 'K',
      group: 'Mission',
    },
    {
      id: 'mission-start-test',
      title: 'Mission Start',
      kind: 'mission',
      description: 'Run the normal mission start sequence with all server-side checks.',
      caution: 'danger',
      shortcut: 'I',
      group: 'Mission',
    },
    {
      id: 'mission-pause-test',
      title: 'Mission Pause',
      kind: 'mission',
      description: 'Pause the active mission through the full lifecycle path.',
      caution: 'safe',
      shortcut: 'H',
      group: 'Mission',
    },
    {
      id: 'mission-resume-test',
      title: 'Mission Resume',
      kind: 'mission',
      description: 'Resume a paused mission through the full lifecycle path.',
      caution: 'caution',
      shortcut: 'R',
      group: 'Mission',
    },
    {
      id: 'mission-complete-test',
      title: 'Mission Complete',
      kind: 'mission',
      description: 'Complete the active mission and close out the lifecycle cleanly.',
      caution: 'caution',
      shortcut: 'G',
      group: 'Mission',
    },
    {
      id: 'mission-abort-test',
      title: 'Mission Abort',
      kind: 'mission',
      description: 'Abort the active mission and force the ESTOP lifecycle path.',
      caution: 'danger',
      shortcut: 'Z',
      group: 'Mission',
    },
    {
      id: 'lora-example-set',
      title: 'LoRa Example Set',
      kind: 'transport',
      description: 'Send the STM32 example LoRa control sequence used by the console test menu.',
      caution: 'danger',
      shortcut: 'X',
      group: 'Transport',
    },
    {
      id: 'salt-50',
      title: 'Salt 50%',
      kind: 'dispersion',
      description: 'Send a 50% salt-only bench command.',
      caution: 'caution',
      shortcut: '7',
      group: 'Dispersion',
    },
    {
      id: 'brine-50',
      title: 'Brine 50%',
      kind: 'dispersion',
      description: 'Send a 50% brine-only bench command.',
      caution: 'caution',
      shortcut: '8',
      group: 'Dispersion',
    },
    {
      id: 'agitator-on',
      title: 'Agitator ON',
      kind: 'dispersion',
      description: 'Toggle the brine agitator on for individual bench testing.',
      caution: 'caution',
      shortcut: 'A',
      group: 'Dispersion',
    },
    {
      id: 'thrower-on',
      title: 'Thrower ON',
      kind: 'dispersion',
      description: 'Toggle the salt thrower on for individual bench testing.',
      caution: 'caution',
      shortcut: 'W',
      group: 'Dispersion',
    },
    {
      id: 'relay-on',
      title: 'Relay ON',
      kind: 'dispersion',
      description: 'Toggle the Sabertooth relay on for individual bench testing.',
      caution: 'caution',
      shortcut: 'O',
      group: 'Dispersion',
    },
    {
      id: 'vibration-on',
      title: 'Vibration ON',
      kind: 'dispersion',
      description: 'Toggle the vibration motor on for individual bench testing.',
      caution: 'caution',
      shortcut: 'Y',
      group: 'Dispersion',
    },
    {
      id: 'all-on',
      title: 'All Outputs On',
      kind: 'dispersion',
      description: 'Enable salt, brine, agitator, thrower, relay, and vibration outputs together.',
      caution: 'danger',
      shortcut: 'U',
      group: 'Dispersion',
    },
    {
      id: 'safe-off',
      title: 'Safe Outputs Off',
      kind: 'dispersion',
      description: 'Send the safe-off sequence to stop outputs, relay, and vibration.',
      caution: 'safe',
      shortcut: 'Q',
      group: 'Dispersion',
    },
    {
      id: 'mix-preset',
      title: 'Mix Preset',
      kind: 'raw',
      description: 'Send a CMD:AUTO,SALT:x,BRINE:y style mix preset from the server.',
      caution: 'danger',
      shortcut: 'L',
      group: 'Transport',
      needsInput: {
        field: 'mixText',
        label: 'Mix',
        placeholder: 'AUTO,SALT:25,BRINE:75',
      },
    },
    {
      id: 'raw-command',
      title: 'Raw Command',
      kind: 'raw',
      description: 'Send a custom plain-text command for bench testing firmware features.',
      caution: 'danger',
      shortcut: '?',
      group: 'Advanced',
      needsInput: {
        field: 'cmdText',
        label: 'Command',
        placeholder: 'CMD:AUTO,SALT:25,BRINE:75',
      },
    },
  ];
}

function resolveTestMenuAction(actionIdOrShortcut) {
  const token = String(actionIdOrShortcut ?? '').trim();
  if (!token) return null;
  const upper = token.toUpperCase();
  return buildTestMenu().find((item) => item.id === token || String(item.shortcut ?? '').toUpperCase() === upper) ?? null;
}

async function runTestMenuAction(actionId, input = {}) {
  const resolved = resolveTestMenuAction(actionId);
  if (!resolved) {
    return {
      ok: false,
      actionId,
      result: { error: `Unknown test action: ${actionId}` },
    };
  }

  // Test/service actions remain available in demo lock for field recovery.

  switch (resolved.id) {
    case 'bridge-status': {
      return {
        ok: true,
        actionId: resolved.id,
        result: await bridge.refreshStatus(),
      };
    }
    case 'gps-readiness': {
      const gpsStatus = getRobotGpsStatus();
      return {
        ok: gpsStatus.ready,
        actionId: resolved.id,
        result: {
          ...gpsStatus,
          robot: state.robot
            ? {
                state: state.robot.state ?? null,
                gpsFix: state.robot.gpsFix ?? null,
                gpsSat: state.robot.gpsSat ?? null,
                gpsHdop: state.robot.gpsHdop ?? null,
                timestampMs: state.robot.timestampMs ?? null,
              }
            : null,
        },
      };
    }
    case 'telemetry-snapshot': {
      return {
        ok: Boolean(state.robot),
        actionId: resolved.id,
        result: {
          robot: state.robot,
          safety: state.safety,
          lastFault: state.lastFault,
        },
      };
    }
    case 'websocket-test': {
      metrics.wsLastTestAt = Date.now();
      const delivered = publish(WS_EVENT.WS_TEST, {
        label: 'test_menu',
        websocket: buildWebSocketDiagnostics(),
      });
      return {
        ok: true,
        actionId: resolved.id,
        result: {
          delivered,
          at: metrics.wsLastTestAt,
          websocket: buildWebSocketDiagnostics(),
        },
      };
    }
    case 'state-snapshot': {
      return {
        ok: true,
        actionId: resolved.id,
        result: {
          mission: mission.publicMission(),
          robot: state.robot,
          lora: await bridge.refreshStatus(),
          safety: state.safety,
          lastFault: state.lastFault,
          allowedActions: buildAllowedActions(),
          alerts: buildAlerts(),
        },
      };
    }
    case 'gps-snapshot': {
      const gpsStatus = getRobotGpsStatus();
      return {
        ok: true,
        actionId: resolved.id,
        result: {
          ...gpsStatus,
          robot: state.robot
            ? {
                lat: state.robot.lat ?? null,
                lon: state.robot.lon ?? null,
                heading: state.robot.heading ?? null,
                speed: state.robot.speed ?? null,
                state: state.robot.state ?? null,
                gpsFix: state.robot.gpsFix ?? null,
                gpsSat: state.robot.gpsSat ?? null,
                gpsHdop: state.robot.gpsHdop ?? null,
                timestampMs: state.robot.timestampMs ?? null,
              }
            : null,
        },
      };
    }
    case 'mode-manual': {
      const result = await dispatchCommand(CMD.MANUAL, {
        syncMission: false,
        source: 'test-menu.mode-manual',
        waitForAck: true,
        waitForState: true,
      });
      return { ok: result.ok, actionId: resolved.id, result };
    }
    case 'mode-test': {
      const result = await dispatchCommand(CMD.PAUSE, {
        syncMission: false,
        source: 'test-menu.mode-test',
        waitForAck: true,
        waitForState: true,
      });
      return { ok: result.ok, actionId: resolved.id, result };
    }
    case 'mode-auto': {
      const result = await dispatchCommand(CMD.AUTO, {
        syncMission: false,
        source: 'test-menu.mode-auto',
        waitForAck: true,
        waitForState: true,
      });
      return { ok: result.ok, actionId: resolved.id, result };
    }
    case 'ack-pause': {
      const result = await dispatchCommand(CMD.PAUSE, {
        syncMission: false,
        source: 'test-menu.ack-pause',
        waitForAck: true,
        waitForState: true,
      });
      return { ok: result.ok, actionId: resolved.id, result };
    }
    case 'ack-reset': {
      const result = await dispatchCommand(CMD.RESET, {
        syncMission: false,
        source: 'test-menu.ack-reset',
        waitForAck: true,
        waitForState: true,
      });
      return { ok: result.ok, actionId: resolved.id, result };
    }
    case 'ack-estop': {
      const result = await dispatchCommand(CMD.ESTOP, {
        syncMission: false,
        source: 'test-menu.ack-estop',
        waitForAck: true,
        waitForState: true,
      });
      return { ok: result.ok, actionId: resolved.id, result };
    }
    case 'drive-forward':
    case 'drive-left':
    case 'drive-stop':
    case 'drive-right':
    case 'drive-backward': {
      const commandMap = {
        'drive-forward': CMD.FORWARD,
        'drive-left': CMD.LEFT,
        'drive-stop': CMD.STOP,
        'drive-right': CMD.RIGHT,
        'drive-backward': CMD.BACKWARD,
      };
      const command = commandMap[resolved.id];
      const result = await dispatchCommand(command, {
        syncMission: false,
        source: `test-menu.${resolved.id}`,
        waitForAck: false,
        waitForState: false,
      });
      return { ok: result.ok, actionId: resolved.id, result: { ...result, command } };
    }
    case 'push-sample-waypoints': {
      const points = buildSampleWaypointSet();
      if (!points) {
        return {
          ok: false,
          actionId: resolved.id,
          result: { error: 'No sample waypoint source available. Plan a path or provide live robot telemetry first.' },
        };
      }
      const pushed = await bridge.pushWaypoints(points);
      return {
        ok: pushed.ok,
        actionId: resolved.id,
        result: {
          ...pushed,
          points,
          lora: bridge.getStatus(),
        },
      };
    }
    case 'commit-current-path': {
      const result = await pushPathWaypoints(null, 'test-menu.commit-current-path');
      return { ok: result.ok, actionId: resolved.id, result };
    }
    case 'mission-start-test': {
      const result = await startMissionAction('test-menu.mission-start');
      return { ok: result.ok, actionId: resolved.id, result };
    }
    case 'mission-pause-test': {
      const result = await pauseMissionAction('test-menu.mission-pause');
      return { ok: result.ok, actionId: resolved.id, result };
    }
    case 'mission-resume-test': {
      const result = await resumeMissionAction('test-menu.mission-resume');
      return { ok: result.ok, actionId: resolved.id, result };
    }
    case 'mission-complete-test': {
      const result = await completeMissionAction('test-menu.mission-complete');
      return { ok: result.ok, actionId: resolved.id, result };
    }
    case 'mission-abort-test': {
      const result = await abortMissionAction('test-menu.mission-abort', 'test-menu.mission-abort');
      return { ok: result.ok, actionId: resolved.id, result };
    }
    case 'salt-50': {
      const result = await bridge.sendCommand('TEST SALT 50', { waitForAck: true, ackRequired: true });
      return { ok: result.ok, actionId: resolved.id, result: { ...result, command: 'TEST SALT 50' } };
    }
    case 'brine-50': {
      const result = await bridge.sendCommand('TEST BRINE 50', { waitForAck: true, ackRequired: true });
      return { ok: result.ok, actionId: resolved.id, result: { ...result, command: 'TEST BRINE 50' } };
    }
    case 'agitator-on': {
      const result = await bridge.sendCommand('AGITATOR ON', { waitForAck: true, ackRequired: true });
      return { ok: result.ok, actionId: resolved.id, result: { ...result, command: 'AGITATOR ON' } };
    }
    case 'thrower-on': {
      const result = await bridge.sendCommand('THROWER ON', { waitForAck: true, ackRequired: true });
      return { ok: result.ok, actionId: resolved.id, result: { ...result, command: 'THROWER ON' } };
    }
    case 'relay-on': {
      const result = await bridge.sendCommand('RELAY ON', { waitForAck: true, ackRequired: true });
      return { ok: result.ok, actionId: resolved.id, result: { ...result, command: 'RELAY ON' } };
    }
    case 'vibration-on': {
      const result = await bridge.sendCommand('VIBRATION ON', { waitForAck: true, ackRequired: true });
      return { ok: result.ok, actionId: resolved.id, result: { ...result, command: 'VIBRATION ON' } };
    }
    case 'all-on': {
      const result = await bridge.sendCommand('ALLON', { waitForAck: true, ackRequired: true });
      return { ok: result.ok, actionId: resolved.id, result: { ...result, command: 'ALLON' } };
    }
    case 'safe-off': {
      const commands = ['TEST SALT 0', 'TEST BRINE 0', 'AGITATOR OFF', 'THROWER OFF', 'RELAY OFF', 'VIBRATION OFF', 'STOP'];
      const steps = [];
      for (const command of commands) {
        const result = await bridge.sendCommand(command, { waitForAck: true, ackRequired: true });
        steps.push({ command, ok: result.ok, status: result.status ?? null, error: result.error ?? null });
        if (!result.ok) {
          return { ok: false, actionId: resolved.id, result: { steps, failedCommand: command } };
        }
        await sleep(100);
      }
      return { ok: true, actionId: resolved.id, result: { steps } };
    }
    case 'lora-example-set': {
      const commands = [
        'CMD:MANUAL',
        'CMD:AUTO,SALT:25,BRINE:75',
        'CMD:PAUSE',
        'CMD:ESTOP',
      ];
      const steps = [];
      for (const command of commands) {
        const result = await bridge.sendCommand(command, { waitForAck: true, ackRequired: true });
        steps.push({ command, ok: result.ok, status: result.status ?? null, error: result.error ?? null });
        if (!result.ok) {
          return { ok: false, actionId: resolved.id, result: { steps, failedCommand: command } };
        }
        await sleep(150);
      }
      return { ok: true, actionId: resolved.id, result: { steps, lora: await bridge.refreshStatus() } };
    }
    case 'mix-preset': {
      const mixText = String(input.mixText ?? '').trim();
      if (!mixText) {
        return {
          ok: false,
          actionId: resolved.id,
          result: { error: 'mixText is required for mix-preset' },
        };
      }
      const command = mixText.toUpperCase().startsWith('CMD:') ? mixText : `CMD:${mixText}`;
      const result = await bridge.sendCommand(command, { waitForAck: true, ackRequired: true });
      return {
        ok: result.ok,
        actionId: resolved.id,
        result: {
          ...result,
          command,
        },
      };
    }
    case 'raw-command': {
      const cmdText = String(input.cmdText ?? '').trim();
      if (!cmdText) {
        return {
          ok: false,
          actionId: resolved.id,
          result: { error: 'cmdText is required for raw-command' },
        };
      }
      const result = await bridge.sendCommand(cmdText, { waitForAck: false });
      return {
        ok: result.ok,
        actionId: resolved.id,
        result: {
          ...result,
          command: cmdText,
        },
      };
    }
    default:
      return {
        ok: false,
        actionId: resolved.id,
        result: { error: `Unknown test action: ${resolved.id}` },
      };
  }
}

function parseIncomingTelemetry(payload) {
  let body = parseMaybeJson(payload) ?? {};

  if (typeof payload === 'string') {
    const compact = parseCompactTelemetryFrame(payload);
    if (compact) {
      return compact;
    }
  }

  // Base station wraps last LoRa RX into a string field: {"last_lora":"<json>"}
  // Unwrap and re-parse so we treat it like direct telemetry.
  if (typeof body?.last_lora === 'string' && body.last_lora.trim().startsWith('{')) {
    const inner = parseMaybeJson(body.last_lora);
    if (inner && typeof inner === 'object') body = inner;
  }

  // Fault notification: {"fault":"IMU_TIMEOUT","action":"ESTOP"}
  if (typeof body?.fault === 'string') {
    const faultCode = Object.values(FAULT_CODE).includes(body.fault)
      ? body.fault
      : FAULT_CODE.GENERIC;
    const faultAction = Object.values(FAULT_ACTION).includes(body.action)
      ? body.action
      : FAULT_ACTION.LOG_ONLY;
    return {
      robot: null,
      isFault: true,
      fault: faultCode,
      action: faultAction,
      source: 'lora-fault',
      raw: body,
    };
  }

  const robotLat = coerceFiniteNumber(body?.robot?.lat, body?.robot?.latitude, body?.robot?.gps?.lat, body?.robot?.gps?.latitude);
  const robotLon = coerceFiniteNumber(body?.robot?.lon, body?.robot?.longitude, body?.robot?.gps?.lon, body?.robot?.gps?.longitude);
  if (body?.robot && robotLat !== null && robotLon !== null) {
    const motorM1 = coerceFiniteNumber(body.robot?.motor?.m1, body.motor?.m1, body.m1, 0) ?? 0;
    const motorM2 = coerceFiniteNumber(body.robot?.motor?.m2, body.motor?.m2, body.m2, 0) ?? 0;
    return {
      robot: {
        lat: robotLat,
        lon: robotLon,
        heading: coerceFiniteNumber(body.robot.heading, body.heading?.yaw, body.robot.yaw, 0) ?? 0,
        speed: coerceFiniteNumber(body.robot.speed, body.robot.velocity, 0) ?? 0,
        motor: { m1: motorM1, m2: motorM2 },
        prox: {
          left: coerceFiniteNumber(body.robot?.prox?.left, body.prox?.left, body.pl, null),
          right: coerceFiniteNumber(body.robot?.prox?.right, body.prox?.right, body.pr, null),
        },
        gpsFix: coerceBooleanLike(body.robot.fix, body.robot.gpsFix, body.gps?.fix, false) ?? false,
        gpsHdop: coerceFiniteNumber(body.robot.hdop, body.robot.gpsHdop, body.gps?.hdop, 0) ?? 0,
        gpsSat: coerceFiniteNumber(body.robot.sat, body.robot.gpsSat, body.gps?.sat, 0) ?? 0,
      },
      source: String(body.source ?? 'unknown'),
      stateName: typeof body.state === 'string' ? body.state : null,
      raw: body,
    };
  }

  const gpsLat = coerceFiniteNumber(body?.gps?.lat, body?.gps?.latitude, body?.lat, body?.latitude);
  const gpsLon = coerceFiniteNumber(body?.gps?.lon, body?.gps?.longitude, body?.lon, body?.longitude);
  if (gpsLat !== null && gpsLon !== null && (body?.gps || body?.state || body?.motor || body?.heading)) {
    const motorM1 = coerceFiniteNumber(body.motor?.m1, body.motor?.left, body.m1, 0) ?? 0;
    const motorM2 = coerceFiniteNumber(body.motor?.m2, body.motor?.right, body.m2, 0) ?? 0;
    const approxSpeed = Math.abs((motorM1 + motorM2) / 2) / 100;

    return {
      robot: {
        lat: gpsLat,
        lon: gpsLon,
        heading: coerceFiniteNumber(body.heading?.yaw, body.heading, body.yaw, body.h, 0) ?? 0,
        speed: coerceFiniteNumber(body.speed, body.velocity, approxSpeed) ?? approxSpeed,
        motor: { m1: motorM1, m2: motorM2 },
        prox: {
          left: coerceFiniteNumber(body.prox?.left, body.pl, null),
          right: coerceFiniteNumber(body.prox?.right, body.pr, null),
        },
        gpsFix: coerceBooleanLike(body.gps?.fix, body.fix, body.gpsFix, false) ?? false,
        gpsHdop: coerceFiniteNumber(body.gps?.hdop, body.hdop, body.gpsHdop, 0) ?? 0,
        gpsSat: coerceFiniteNumber(body.gps?.sat, body.sat, body.gpsSat, 0) ?? 0,
      },
      source: String(body.source ?? 'lora'),
      stateName: typeof body.state === 'string' ? body.state : null,
      raw: body,
    };
  }

  if (typeof body?.state === 'string' && (body?.motor || body?.heading || body?.prox || body?.disp)) {
    const prevLat = coerceFiniteNumber(state.robot?.lat, 0) ?? 0;
    const prevLon = coerceFiniteNumber(state.robot?.lon, 0) ?? 0;
    const motorM1 = coerceFiniteNumber(body.motor?.m1, body.motor?.left, 0) ?? 0;
    const motorM2 = coerceFiniteNumber(body.motor?.m2, body.motor?.right, 0) ?? 0;
    const approxSpeed = Math.abs((motorM1 + motorM2) / 2) / 100;

    return {
      robot: {
        lat: prevLat,
        lon: prevLon,
        heading: coerceFiniteNumber(body.heading?.yaw, body.heading, body.yaw, 0) ?? 0,
        speed: coerceFiniteNumber(body.speed, body.velocity, approxSpeed) ?? approxSpeed,
        motor: { m1: motorM1, m2: motorM2 },
        prox: {
          left: coerceFiniteNumber(body.prox?.left, body.pl, null),
          right: coerceFiniteNumber(body.prox?.right, body.pr, null),
        },
        gpsFix: coerceBooleanLike(body.gps?.fix, body.fix, state.robot?.gpsFix, false) ?? false,
        gpsHdop: coerceFiniteNumber(body.gps?.hdop, body.hdop, state.robot?.gpsHdop, 0) ?? 0,
        gpsSat: coerceFiniteNumber(body.gps?.sat, body.sat, state.robot?.gpsSat, 0) ?? 0,
      },
      source: String(body.source ?? 'lora-state'),
      stateName: body.state,
      raw: body,
    };
  }

  // Compact manual-drive telemetry from STM32:
  // {"s":"MANUAL","m1":55,"m2":55,"h":180.0,"pl":120,"pr":340}
  // This packet intentionally omits GPS to reduce airtime. Keep telemetry fresh
  // using last known robot GPS position when available.
  if (typeof body?.s === 'string' && body.s.trim().toUpperCase() === 'MANUAL') {
    const motorM1 = Number(body.m1 ?? 0);
    const motorM2 = Number(body.m2 ?? 0);
    const approxSpeed = Math.abs((motorM1 + motorM2) / 2) / 100;

    const prevLat = Number(state.robot?.lat);
    const prevLon = Number(state.robot?.lon);
    const prevGpsFix = Boolean(state.robot?.gpsFix ?? false);
    const prevGpsHdop = Number(state.robot?.gpsHdop ?? 0);
    const prevGpsSat = Number(state.robot?.gpsSat ?? 0);

    return {
      robot: {
        lat: Number.isFinite(prevLat) ? prevLat : 0,
        lon: Number.isFinite(prevLon) ? prevLon : 0,
        heading: Number(body.h ?? state.robot?.heading ?? 0),
        speed: Number(body.speed ?? approxSpeed),
        motor: { m1: motorM1, m2: motorM2 },
        prox: {
          left: coerceFiniteNumber(body.pl, body.prox?.left, null),
          right: coerceFiniteNumber(body.pr, body.prox?.right, null),
        },
        gpsFix: prevGpsFix,
        gpsHdop: Number.isFinite(prevGpsHdop) ? prevGpsHdop : 0,
        gpsSat: Number.isFinite(prevGpsSat) ? prevGpsSat : 0,
        sensors: {
          imuOk: coerceBooleanLike(body.imu, body.sensors?.imuOk, true) ?? true,
          gpsOk: coerceBooleanLike(body.gps, body.sensors?.gpsOk, prevGpsFix) ?? prevGpsFix,
          loraOk: coerceBooleanLike(body.lora, body.sensors?.loraOk, true) ?? true,
          degraded: coerceBooleanLike(body.deg, body.sensors?.degraded, false) ?? false,
        },
      },
      source: String(body.source ?? 'lora-manual'),
      stateName: 'MANUAL',
      raw: body,
    };
  }

  return null;
}

function parseCompactTelemetryFrame(raw) {
  if (typeof raw !== 'string') return null;
  const frame = raw.trim();
  if (!frame) return null;

  if (frame.startsWith('F:')) {
    const parts = frame.slice(2).split(',').map((part) => part.trim()).filter(Boolean);
    const faultCandidate = parts[0] ?? '';
    const actionCandidate = parts[1] ?? '';
    const faultCode = Object.values(FAULT_CODE).includes(faultCandidate)
      ? faultCandidate
      : FAULT_CODE.GENERIC;
    const faultAction = Object.values(FAULT_ACTION).includes(actionCandidate)
      ? actionCandidate
      : FAULT_ACTION.LOG_ONLY;
    return {
      robot: null,
      isFault: true,
      fault: faultCode,
      action: faultAction,
      source: 'compact-fault',
      raw: { frame },
    };
  }

  if (frame.startsWith('M:')) {
    const parts = frame.slice(2).split(',').map((part) => part.trim());
    if (parts.length < 7) return null;
    const heading = coerceFiniteNumber(parts[0], state.robot?.heading, 0) ?? 0;
    const motorM1 = coerceFiniteNumber(parts[1], 0) ?? 0;
    const motorM2 = coerceFiniteNumber(parts[2], 0) ?? 0;
    const approxSpeed = Math.abs((motorM1 + motorM2) / 2) / 100;
    const prevLat = Number(state.robot?.lat);
    const prevLon = Number(state.robot?.lon);
    const prevGpsFix = Boolean(state.robot?.gpsFix ?? false);
    const prevGpsHdop = Number(state.robot?.gpsHdop ?? 0);
    const prevGpsSat = Number(state.robot?.gpsSat ?? 0);

    return {
      robot: {
        lat: Number.isFinite(prevLat) ? prevLat : 0,
        lon: Number.isFinite(prevLon) ? prevLon : 0,
        heading,
        speed: approxSpeed,
        gpsFix: prevGpsFix,
        gpsHdop: Number.isFinite(prevGpsHdop) ? prevGpsHdop : 0,
        gpsSat: Number.isFinite(prevGpsSat) ? prevGpsSat : 0,
        sensors: {
          imuOk: coerceBooleanLike(parts[3], true) ?? true,
          gpsOk: coerceBooleanLike(parts[4], prevGpsFix) ?? prevGpsFix,
          loraOk: coerceBooleanLike(parts[5], true) ?? true,
          degraded: coerceBooleanLike(parts[6], false) ?? false,
        },
      },
      source: 'compact-manual',
      stateName: 'MANUAL',
      raw: { frame },
    };
  }

  if (frame.startsWith('S:') || frame.startsWith('T:')) {
    const mode = frame.charAt(0);
    const parts = frame.slice(2).split(',').map((part) => part.trim());
    const minFields = mode === 'T' ? 10 : 8;
    if (parts.length < minFields) return null;

    const stateName = parts[0] || null;
    const lat = coerceFiniteNumber(parts[1]);
    const lon = coerceFiniteNumber(parts[2]);
    const heading = coerceFiniteNumber(parts[3], 0) ?? 0;
    const speed = coerceFiniteNumber(parts[4], 0) ?? 0;
    const gpsFix = coerceBooleanLike(parts[5], false) ?? false;
    const gpsHdop = coerceFiniteNumber(parts[6], 0) ?? 0;
    const gpsSat = coerceFiniteNumber(parts[7], 0) ?? 0;

    if (lat === null || lon === null) return null;

    const robot = {
      lat,
      lon,
      heading,
      speed,
      gpsFix,
      gpsHdop,
      gpsSat,
    };

    if (mode === 'T') {
      const motorM1 = coerceFiniteNumber(parts[8], 0) ?? 0;
      const motorM2 = coerceFiniteNumber(parts[9], 0) ?? 0;
      robot.motor = { m1: motorM1, m2: motorM2 };
      const salt = coerceFiniteNumber(parts[10], null);
      const brine = coerceFiniteNumber(parts[11], null);
      if (salt !== null || brine !== null) {
        robot.disp = {
          salt: salt ?? 0,
          brine: brine ?? 0,
        };
      }
    }

    return {
      robot,
      source: mode === 'T' ? 'compact-telemetry' : 'compact-state',
      stateName,
      raw: { frame },
    };
  }

  return null;
}

async function ingestParsedTelemetry(parsed) {
  if (!parsed) {
    return { ok: false, error: 'Unsupported telemetry payload' };
  }

  metrics.telemetryAccepted += 1;

  if (parsed.isFault) {
    state.lastFault = { fault: parsed.fault, action: parsed.action, at: Date.now() };
    publish(WS_EVENT.FAULT_RECEIVED, { fault: parsed.fault, action: parsed.action, at: state.lastFault.at });

    db.logEvent(mission.currentId(), EVENT_TYPE.FAULT_RECEIVED, {
      fault: parsed.fault,
      action: parsed.action,
    });
    mission.recordFault();

    if (parsed.action === 'ESTOP') {
      if ([MISSION_STATE.RUNNING, MISSION_STATE.PAUSED].includes(currentMissionState())) {
        try { mission.abort(`fault:${parsed.fault}`, buildFinalMissionStats()); } catch (_) { /* ignore */ }
      }
      bridge.resetWpState();
    } else if (parsed.action === 'PAUSE' && mission.isRunning()) {
      try { mission.pause(`fault:${parsed.fault}`); } catch (_) { /* ignore */ }
    }

    publishSupervision();
    publishOperator();
    scheduleRuntimeStateSave('telemetry.fault', { immediate: true });

    return { ok: true, fault: parsed.fault, action: parsed.action };
  }

  state.robot = {
    lat: parsed.robot.lat,
    lon: parsed.robot.lon,
    heading: parsed.robot.heading,
    speed: parsed.robot.speed,
    motor: parsed.robot.motor ?? state.robot?.motor ?? null,
    prox: parsed.robot.prox ?? state.robot?.prox ?? null,
    disp: parsed.robot.disp ?? state.robot?.disp ?? null,
    gpsFix: parsed.robot.gpsFix,
    gpsHdop: parsed.robot.gpsHdop,
    gpsSat: parsed.robot.gpsSat,
    sensors: parsed.robot.sensors ?? state.robot?.sensors ?? null,
    source: parsed.source,
    state: parsed.stateName ?? state.robot?.state ?? ROBOT_STATE.IDLE,
    timestampMs: Date.now(),
    raw: parsed.raw,
  };

  if (state.robot.motor && (Date.now() - lastMotorTelemetryLogMs) >= 5000) {
    const m1 = Number(state.robot.motor.m1 ?? 0);
    const m2 = Number(state.robot.motor.m2 ?? 0);
    console.log(`[telemetry] motor m1=${m1} m2=${m2} state=${state.robot.state ?? 'UNKNOWN'} src=${state.robot.source ?? 'unknown'}`);
    lastMotorTelemetryLogMs = Date.now();
  }

  clearRecoveredSafetyState({
    telemetryRecovered: true,
    gpsRecovered: getRobotGpsStatus(state.robot).ready,
    geofenceRecovered: isRobotInsideCoverageArea(state.robot),
    faultRecovered: ![ROBOT_STATE.ESTOP, ROBOT_STATE.ERROR].includes(state.robot.state),
  });

  state.trail.push({ lat: state.robot.lat, lon: state.robot.lon, t: state.robot.timestampMs });
  if (state.trail.length > 5000) {
    state.trail = state.trail.slice(-5000);
  }

  let stats = null;
  if (state.coverage) {
    markCoverage(state.coverage, state.robot, resolveCoverageMarkRadiusM(), state.robot.timestampMs);
    stats = coverageStats(state.coverage);
  }

  publish(WS_EVENT.TELEMETRY_UPDATED, { robot: state.robot, coverage: stats });

  db.logEvent(mission.currentId(), EVENT_TYPE.TELEMETRY_RECEIVED, {
    lat: state.robot.lat,
    lon: state.robot.lon,
    heading: state.robot.heading,
    state: state.robot.state,
  });

  if (stats) mission.updateCoverage(getCoveragePct(stats));

  await enforceDemoObstaclePolicy('telemetry');
  await enforceGeofenceFailsafePolicy('telemetry');

  scheduleRuntimeStateSave('telemetry.update');
  publishSupervision();

  return { ok: true, coverage: stats };
}

async function pollBaseStationLoRaTelemetry() {
  try {
    const lora = await bridge.refreshStatus();
    const raw = typeof lora?.baseStation?.lastLoRa === 'string' ? lora.baseStation.lastLoRa.trim() : '';
    if (!raw || raw === lastPolledLoRaFrame) {
      return;
    }

    lastPolledLoRaFrame = raw;
    const parsed = parseIncomingTelemetry({ last_lora: raw });
    if (!parsed) {
      return;
    }

    await ingestParsedTelemetry(parsed);
  } catch (_) {
    // Best-effort poll only; direct POST telemetry remains supported.
  }
}

function publish(event, payload) {
  const packet = JSON.stringify({ event, payload, at: Date.now() });
  let delivered = 0;
  for (const client of wss.clients) {
    if (client.readyState === 1) {
      client.send(packet);
      delivered += 1;
    }
  }
  metrics.wsPublishes += 1;
  metrics.wsDeliveredMessages += delivered;
  return delivered;
}

function buildWebSocketDiagnostics() {
  let connectedClients = 0;
  for (const client of wss.clients) {
    if (client.readyState === 1) connectedClients += 1;
  }

  return {
    connectedClients,
    peakClients: metrics.wsPeakClients,
    connectionsOpened: metrics.wsConnectionsOpened,
    connectionsClosed: metrics.wsConnectionsClosed,
    publishes: metrics.wsPublishes,
    deliveredMessages: metrics.wsDeliveredMessages,
    lastTestAt: metrics.wsLastTestAt,
  };
}

function publicState() {
  return {
    baseStation: state.baseStation ?? state.remoteBaseStation,
    homePoint: state.homePoint,
    remoteBaseStation: state.remoteBaseStation,
    boundary: state.boundary,
    demo: state.demo,
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
    lastArrows: state.lastArrows,
    lastCommandId: state.lastCommandId,
    lastCommandStatus: state.lastCommandStatus,
    commandHistory: state.commandHistory,
    mission: mission.publicMission(),
    operator: {
      notes: state.operator.notes,
      workflows: buildOperatorWorkflows(),
    },
    persistence: state.persistence,
  };
}

app.get(API.HEALTH, (_req, res) => {
  const bridgeStatus = bridge.getStatus();
  const connection = buildConnectionState();
  const dbOk = connection.backend.dbOk;
  const missionState = currentMissionState();
  const telemetryStaleWhileRunning = missionState === MISSION_STATE.RUNNING && connection.robot.telemetryStale;

  const checks = {
    db: dbOk,
    bridge: connection.baseStation.reachable,
    gateway: connection.gateway?.reachable ?? false,
    telemetry: !telemetryStaleWhileRunning,
  };

  const ready = Boolean(checks.db && checks.bridge && checks.telemetry);
  const status = ready ? 200 : 503;

  res.status(status).json({
    ok: ready,
    ready,
    service: 'robot-lora-server',
    checks,
    auth: {
      enabled: API_AUTH_ENABLED,
      appKeyCount: APP_API_KEYS.size,
      boardKeyCount: BOARD_API_KEYS.size,
    },
    policies: {
      telemetryFailsafeEnabled: TELEMETRY_FAILSAFE_ENABLED,
      telemetryFailsafeAction: resolveTelemetryFailsafeAction(),
      geofenceFailsafeEnabled: GEOFENCE_FAILSAFE_ENABLED,
      geofenceFailsafeAction: resolveGeofenceFailsafeAction(),
    },
    connectivity: connection,
    remoteBaseStation: buildRemoteBaseStationDiagnostics(),
    missionState,
    telemetryStale: connection.robot.telemetryStale,
    lora: {
      degraded: bridgeStatus.degraded,
      consecutiveFailures: bridgeStatus.consecutiveFailures,
      lastSuccessAt: bridgeStatus.lastSuccessAt,
      lastErrorAt: bridgeStatus.lastErrorAt,
    },
    websocket: buildWebSocketDiagnostics(),
    commandHistory: state.commandHistory,
    connectivity: connection,
    persistence: {
      source: state.persistence.source,
      restoredAt: state.persistence.restoredAt,
      lastSavedAt: state.persistence.lastSavedAt,
      lastSaveReason: state.persistence.lastSaveReason,
      lastSaveError: state.persistence.lastSaveError,
      exists: fs.existsSync(RUNTIME_STATE_PATH),
    },
  });
});

app.get(API.METRICS, requireApp, (_req, res) => {
  const bridgeStatus = bridge.getStatus();
  res.json({
    ok: true,
    metrics: {
      ...metrics,
      uptimeMs: Date.now() - metrics.startedAt,
      bridge: {
        degraded: bridgeStatus.degraded,
        consecutiveFailures: bridgeStatus.consecutiveFailures,
        lastSuccessAt: bridgeStatus.lastSuccessAt,
        lastErrorAt: bridgeStatus.lastErrorAt,
      },
      rateLimits: {
        windowMs: REQUEST_RATE_LIMIT_WINDOW_MS,
        commandPerWindow: COMMAND_RATE_LIMIT_PER_WINDOW,
        telemetryPerWindow: TELEMETRY_RATE_LIMIT_PER_WINDOW,
      },
      websocket: buildWebSocketDiagnostics(),
    },
  });
});

app.get(API.WS_TEST, requireAppOrBoard, (_req, res) => {
  res.json({
    ok: true,
    websocket: buildWebSocketDiagnostics(),
  });
});

app.post(API.WS_TEST, requireAppOrBoard, (req, res) => {
  const label = typeof req.body?.label === 'string' && req.body.label.trim()
    ? req.body.label.trim()
    : 'manual_test';
  const payload = {
    label,
    websocket: buildWebSocketDiagnostics(),
  };
  metrics.wsLastTestAt = Date.now();
  const delivered = publish(WS_EVENT.WS_TEST, payload);
  res.json({
    ok: true,
    delivered,
    at: metrics.wsLastTestAt,
    websocket: buildWebSocketDiagnostics(),
  });
});

app.get(API.SUPERVISION_SUMMARY, requireAppOrBoard, (_req, res) => {
  res.json({ ok: true, summary: buildSupervisionSummary() });
});

app.get(API.DEMO_MODE, requireAppOrBoard, (_req, res) => {
  res.json({
    ok: true,
    demo: {
      ...state.demo,
      config: getDemoConfig(),
      readiness: resolveDemoReadiness(),
    },
  });
});

app.post(API.DEMO_MODE, requireApp, (req, res) => {
  const enabled = Boolean(req.body?.enabled);
  const source = typeof req.body?.source === 'string' && req.body.source.trim()
    ? req.body.source.trim()
    : 'app';
  const config = req.body?.config && typeof req.body.config === 'object'
    ? req.body.config
    : null;
  const demo = setDemoMode(enabled, source, config);
  res.json({ ok: true, demo: { ...demo, config: getDemoConfig(), readiness: resolveDemoReadiness() } });
});

app.post(API.DEMO_SPOT, requireApp, (req, res) => {
  if (!state.demo?.enabled) {
    return res.status(409).json({ ok: false, error: 'Demo mode is not active' });
  }
  const kind = String(req.body?.kind ?? '').trim().toLowerCase();
  if (!['start', 'end'].includes(kind)) {
    return res.status(400).json({ ok: false, error: 'kind must be start or end' });
  }
  const note = typeof req.body?.note === 'string' ? req.body.note : null;
  const source = typeof req.body?.source === 'string' && req.body.source.trim()
    ? req.body.source.trim()
    : 'app';
  const spot = markDemoSpot(kind, note, source);
  res.json({ ok: true, demo: state.demo, spot });
});

app.post(API.DEMO_PATH, requireApp, (req, res) => {
  if (!state.demo?.enabled) {
    return res.status(409).json({ ok: false, error: 'Demo mode is not active' });
  }

  const demoConfig = getDemoConfig();
  const allowWeakGps = req.body?.allowWeakGps !== undefined
    ? req.body?.allowWeakGps !== false
    : demoConfig.allowWeakGps;
  const startSpotStatus = getDemoSpotGpsStatus(state.demo?.spots?.start, 'Spot A');
  if (!startSpotStatus.ready && !allowWeakGps) {
    return res.status(409).json({ ok: false, error: `Demo path blocked: ${startSpotStatus.reason}` });
  }
  const endSpotStatus = getDemoSpotGpsStatus(state.demo?.spots?.end, 'Spot B');
  if (!endSpotStatus.ready && !allowWeakGps) {
    return res.status(409).json({ ok: false, error: `Demo path blocked: ${endSpotStatus.reason}` });
  }

  const laneWidthM = Number.isFinite(Number(req.body?.laneWidthM)) && Number(req.body.laneWidthM) > 0
    ? Number(req.body.laneWidthM)
    : demoConfig.laneWidthM;
  const cellSizeM = Number.isFinite(Number(req.body?.cellSizeM)) && Number(req.body.cellSizeM) > 0
    ? Number(req.body.cellSizeM)
    : demoConfig.cellSizeM;
  const coverageWidthM = Number.isFinite(Number(req.body?.coverageWidthM)) && Number(req.body.coverageWidthM) > 0
    ? Number(req.body.coverageWidthM)
    : demoConfig.coverageWidthM;
  const saltPct = Number.isFinite(Number(req.body?.saltPct))
    ? Math.max(0, Math.min(100, Math.round(Number(req.body.saltPct))))
    : 100;
  const brinePct = Number.isFinite(Number(req.body?.brinePct))
    ? Math.max(0, Math.min(100, Math.round(Number(req.body.brinePct))))
    : 100;

  const geometry = buildDemoBoundaryFromSpots(state.demo?.spots?.start, state.demo?.spots?.end, laneWidthM, demoConfig.minSpotDistanceM);
  if (!geometry.ok) {
    return res.status(400).json({ ok: false, error: geometry.error });
  }

  const configured = configureActiveArea(geometry.baseStation, geometry.boundary, cellSizeM);
  const path = buildMissionPath({
    mode: 'goal',
    startPoint: geometry.start,
    goalPoint: geometry.goal,
    saltPct,
    brinePct,
    homePoint: geometry.baseStation,
    returnToBase: true,
  });

  if (!path.ok) {
    return res.status(400).json({ ok: false, error: path.reason });
  }

  const configuredPasses = Number.isFinite(Number(req.body?.passes))
    ? Math.max(1, Math.min(5, Math.floor(Number(req.body.passes))))
    : demoConfig.passes;
  if (configuredPasses > 1 && Array.isArray(state.lastPath) && state.lastPath.length > 0) {
    const expanded = expandPathWithPasses(state.lastPath, configuredPasses);
    state.lastPath = expanded.map((point) => ({
      ...point,
      salt: saltPct,
      brine: brinePct,
    }));
    publish(WS_EVENT.PATH_UPDATED, {
      mode: 'demo-shuttle',
      coverageWidthM: null,
      points: state.lastPath,
      meta: {
        ...(path.meta ?? {}),
        passes: configuredPasses,
      },
    });
  }

  scheduleRuntimeStateSave('demo-mode.path', { immediate: true });
  res.json({
    ok: true,
    demo: {
      ...state.demo,
      config: demoConfig,
      readiness: resolveDemoReadiness(),
    },
    warnings: allowWeakGps && (!startSpotStatus.ready || !endSpotStatus.ready)
      ? [
          'Service override used: demo path was built with weak or unavailable GPS quality.',
          ...[startSpotStatus, endSpotStatus].filter((status) => !status.ready).map((status) => status.reason),
        ]
      : [],
    geometry: {
      laneWidthM: geometry.laneWidthM,
      laneLengthM: geometry.laneLengthM,
      boundary: geometry.boundary,
      baseStation: geometry.baseStation,
      start: geometry.start,
      goal: geometry.goal,
      cellSizeM: configured.cellPlan.effectiveCellSizeM,
      stats: configured.stats,
    },
    path: {
      mode: 'demo-shuttle',
      coverageWidthM: null,
      passes: configuredPasses,
      pointCount: state.lastPath.length,
      points: state.lastPath,
      meta: path.meta ?? null,
    },
  });
});

app.post(API.DEMO_RUN, requireApp, async (req, res) => {
  if (!state.demo?.enabled) {
    return res.status(409).json({ ok: false, error: 'Demo mode is not active' });
  }

  const demoConfig = getDemoConfig();
  const allowWeakGps = req.body?.allowWeakGps !== undefined
    ? req.body?.allowWeakGps !== false
    : demoConfig.allowWeakGps;

  try {
    const result = await startMissionAction('demo-mode.run', {
      allowWhenDemo: true,
      allowWeakGps,
    });
    if (!result.ok) {
      return res.status(result.status ?? 500).json({ ok: false, error: result.error });
    }
    return res.json({ ok: true, mission: result.mission, allowWeakGps, readiness: resolveDemoReadiness(), lora: bridge.getStatus() });
  } catch (err) {
    return res.status(400).json({ ok: false, error: err.message, readiness: resolveDemoReadiness(), lora: bridge.getStatus() });
  }
});

app.get(API.STATUS, requireAppOrBoard, (_req, res) => {
  const connection = buildConnectionState();
  const bridgeStatus = bridge.getStatus();
  const manualCommandUrl = resolveManualCommandUrl(bridgeStatus);
  res.json({
    battery: 85,
    state: state.robot?.state ?? ROBOT_STATE.IDLE,
    mode: 'SERVER',
    radio_mode: bridgeStatus.baseStation?.radioMode ?? null,
    last_cmd: state.lastCommand,
    last_cmd_id: state.lastCommandId,
    last_cmd_status: state.lastCommandStatus,
    command_history: state.commandHistory,
    last_fault: state.lastFault ?? null,
    demo: state.demo,
    base_station_url: bridgeStatus.baseStation?.selectedUrl ?? bridgeStatus.baseStationUrl ?? null,
    manual_command_url: manualCommandUrl,
    connectivity: connection.overall,
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

app.post(API.COMMAND, requireApp, rateLimitCommand, async (req, res) => {
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

  const rawWireCommand = typeof rawCmd === 'string' ? rawCmd.trim().toUpperCase() : '';
  let wireCommand = rawWireCommand
    ? (rawWireCommand === cmd ? compactWireCommandForCmd(cmd) : rawWireCommand)
    : compactWireCommandForCmd(cmd);
  let driveSeq = null;
  if (cmd === CMD.DRIVE) {
    let driveValue = null;
    let turnValue = null;

    if (typeof parsedBody === 'object' && parsedBody !== null) {
      driveValue = coerceFiniteNumber(parsedBody.throttle, parsedBody.drive);
      turnValue = coerceFiniteNumber(parsedBody.turn, parsedBody.steer);
      driveSeq = coerceFiniteNumber(parsedBody.seq, parsedBody.sequence);
    }

    if (typeof rawCmd === 'string') {
      const compactMatch = rawCmd.trim().toUpperCase().match(/^D\s*:\s*(-?\d+)\s*,\s*(-?\d+)(?:\s*,\s*S\s*:\s*(\d+))?$/);
      if (compactMatch) {
        driveValue = coerceFiniteNumber(driveValue, compactMatch[1]);
        turnValue = coerceFiniteNumber(turnValue, compactMatch[2]);
        driveSeq = coerceFiniteNumber(driveSeq, compactMatch[3]);
      }

      const driveMatch = rawCmd.trim().toUpperCase().match(/(?:THROTTLE|DRIVE)\s*:\s*(-?\d+)/);
      const turnMatch = rawCmd.trim().toUpperCase().match(/(?:TURN|STEER)\s*:\s*(-?\d+)/);
      if (driveMatch) driveValue = coerceFiniteNumber(driveValue, driveMatch[1]);
      if (turnMatch) turnValue = coerceFiniteNumber(turnValue, turnMatch[1]);
    }

    const drive = clampNumber(Math.round(coerceFiniteNumber(driveValue, 0)), -100, 100);
    const turn = clampNumber(Math.round(coerceFiniteNumber(turnValue, 0)), -100, 100);
    const normalizedSeq = Number.isFinite(Number(driveSeq))
      ? Math.max(0, Math.floor(Number(driveSeq)))
      : null;
    wireCommand = normalizedSeq == null ? `D:${drive},${turn}` : `D:${drive},${turn},S:${normalizedSeq}`;
  }

  if (cmd === CMD.DRIVE) {
    const now = Date.now();
    const seqMatch = wireCommand.match(/,S:(\d+)$/);
    const hasSeqTag = Boolean(seqMatch);
    if (!hasSeqTag && lastDriveWireCommand === wireCommand && (now - lastDriveWireAt) < DRIVE_DUPLICATE_SUPPRESS_MS) {
      return res.type('text/plain').send('OK');
    }
    lastDriveWireCommand = wireCommand;
    lastDriveWireAt = now;
  }

  const dispatched = await dispatchCommand(cmd, {
    wireCommand,
    waitForAck: false,
    waitForState: false,
  });
  if (!dispatched.ok) {
    return res.status(dispatched.status ?? 500).type('text/plain').send(dispatched.error);
  }

  return res.type('text/plain').send('OK');
});

app.get(API.ROOT, (_req, res) => {
  res.json({
    ok: true,
    service: 'robot-lora-server',
    endpoints: Object.values(API),
  });
});
app.post(API.INPUT_AREA, requireApp, (req, res) => {
  const { baseStation, boundary, cellSizeM = 2.0, homePoint = null } = req.body ?? {};
  const startedAt = Date.now();

  if (!baseStation || typeof baseStation.lat !== 'number' || typeof baseStation.lon !== 'number') {
    return res.status(400).json({ ok: false, error: 'baseStation.lat/lon required' });
  }

  if (!Array.isArray(boundary) || boundary.length < 3) {
    return res.status(400).json({ ok: false, error: 'boundary requires at least 3 points' });
  }

  const { stats, cellPlan, homePoint: capturedHomePoint } = configureActiveArea(baseStation, boundary, Number(cellSizeM), homePoint);
  scheduleRuntimeStateSave('area.set', { immediate: true });
  publishOperator();
  publishSupervision();

  return res.json({
    ok: true,
    stats,
    requestedCellSizeM: cellPlan.requestedCellSizeM,
    cellSizeM: cellPlan.effectiveCellSizeM,
    cellSizeAdjusted: cellPlan.adjusted,
    estimatedCells: cellPlan.estimatedCells,
    homePoint: capturedHomePoint,
    buildMs: Date.now() - startedAt,
  });
});

app.post(API.TELEMETRY, requireBoard, rateLimitTelemetry, async (req, res) => {
  const parsed = parseIncomingTelemetry(req.body);
  if (!parsed) {
    metrics.telemetryRejected += 1;
    const rawSnippet = typeof req.body === 'string'
      ? req.body
      : (() => {
          try {
            return JSON.stringify(req.body);
          } catch {
            return String(req.body ?? '');
          }
        })();
    console.warn(`[telemetry] rejected payload: ${String(rawSnippet).slice(0, 240)}`);
    return res.status(202).json({ ok: false, ignored: true, error: 'Unsupported telemetry payload' });
  }
  const result = await ingestParsedTelemetry(parsed);
  return res.json(result);
});

app.post(API.BASE_STATION_STATUS, requireBoard, rateLimitTelemetry, async (req, res) => {
  const normalized = normalizeRemoteBaseStationStatus(req.body);
  if (!normalized) {
    metrics.remoteBaseStationRejected += 1;
    console.warn('[remote-base-station] rejected status payload');
    return res.status(400).json({ ok: false, error: 'Unsupported base station status payload' });
  }

  metrics.remoteBaseStationAccepted += 1;
  metrics.remoteBaseStationLastAt = normalized.receivedAt;
  state.remoteBaseStation = normalized;
  if (!normalizeLatLonPoint(state.baseStation, null) && !normalizeLatLonPoint(state.homePoint, null)) {
    state.baseStation = normalized;
  }

  if (typeof normalized.lastCmdId === 'string' && normalized.lastCmdId.trim()) {
    const trackedStatus = normalizeRemoteTrackedStatus(normalized.lastCmdStatus);
    if (trackedStatus) {
      updateCommandTracking(normalized.lastCmdId.trim(), {
        cmd: normalized.lastCmd ?? null,
        source: 'remote-base-station.status',
        status: trackedStatus,
        error: trackedStatus === COMMAND_STATUS.FAILED ? 'Remote base station reported a command failure' : null,
        detail: {
          stage: transportStageForStatus(trackedStatus),
          remoteQueued: true,
          queueDepth: normalized.queueDepth ?? null,
        },
      });
    }
  }

  console.log(
    '[remote-base-station] status received',
    JSON.stringify({
      at: normalized.receivedAt,
      wifi: normalized.wifiLinkState,
      lora: normalized.loraLinkState,
      queueDepth: normalized.queueDepth,
      cmdId: normalized.lastCmdId,
      cmdStatus: normalized.lastCmdStatus,
      backendUrl: normalized.backendUrl,
    })
  );
  publishSupervision();
  publishOperator();
  publish(WS_EVENT.STATE_SNAPSHOT, publicState());
  scheduleRuntimeStateSave('base_station.remote_status');
  return res.json({
    ok: true,
    receivedAt: normalized.receivedAt,
    connectivity: buildConnectionState(),
    remoteBaseStation: buildRemoteBaseStationDiagnostics(),
  });
});

app.get(API.BASE_STATION_COMMAND, requireBoard, (_req, res) => {
  const pending = peekRemoteCommandForBoard();
  if (!pending) {
    return res.json({ ok: true, pending: false, queueDepth: pendingRemoteCommandCount() });
  }

  pending.leaseCount = Number(pending.leaseCount ?? 0) + 1;
  pending.lastLeaseAt = Date.now();

  return res.json({
    ok: true,
    pending: true,
    commandId: pending.commandId,
    cmd: pending.cmd,
    source: pending.source,
    queuedAt: pending.createdAt,
    queueDepth: pendingRemoteCommandCount(),
  });
});

const handleBaseStationCommandAck = (req, res) => {
  const commandId = typeof req.body?.commandId === 'string' ? req.body.commandId.trim() : '';
  if (!commandId) {
    return res.status(400).json({ ok: false, error: 'commandId is required' });
  }

  const ack = acknowledgeRemoteCommand(commandId, {
    status: req.body?.status,
    error: req.body?.error,
  });

  if (!ack) {
    return res.status(404).json({ ok: false, error: 'Command not found or already expired' });
  }

  return res.json({ ok: true, commandId, status: ack.deliveryStatus, queueDepth: pendingRemoteCommandCount() });
};

app.post(API.BASE_STATION_COMMAND_ACK, requireBoard, handleBaseStationCommandAck);
app.post('/api/base-station/command/ack', requireBoard, handleBaseStationCommandAck);
app.post('/api/base-station/ack', requireBoard, handleBaseStationCommandAck);
app.post('/api/base-station/commandAck', requireBoard, handleBaseStationCommandAck);
app.post('/api/base-station/cmd-ack', requireBoard, handleBaseStationCommandAck);

app.get(API.STATE, requireAppOrBoard, (_req, res) => {
  res.json({ ok: true, state: publicState() });
});

app.get(API.COMMAND_HISTORY, requireAppOrBoard, (_req, res) => {
  const connection = buildConnectionState();
  res.json({
    ok: true,
    commands: state.commandHistory,
    lastCommandId: state.lastCommandId,
    lastCommandStatus: state.lastCommandStatus,
    connectivity: connection.commandPath,
  });
});

app.get(API.COVERAGE, requireAppOrBoard, (_req, res) => {
  if (!state.coverage) {
    return res.status(400).json({ ok: false, error: 'input area is not initialized' });
  }

  const cells = [];
  for (let row = 0; row < state.coverage.height; row++) {
    for (let col = 0; col < state.coverage.width; col++) {
      const cell = state.coverage.cells[row][col];
      if (!cell.inside) continue;
      cells.push({
        row,
        col,
        covered: cell.covered,
        hits: cell.hits,
        lastSeenMs: cell.lastSeenMs,
        polygon: gridCellPolygon(state.coverage, row, col),
      });
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

app.post(API.PATH_PLAN, requireApp, (req, res) => {
  if (!state.coverage) {
    return res.status(400).json({ ok: false, error: 'input area is not initialized' });
  }

  const { goal, start, saltPct, brinePct, homePoint = null } = req.body ?? {};
  const requestedSweepDirection = typeof req.body?.sweepDirection === 'string'
    ? req.body.sweepDirection.trim().toLowerCase()
    : 'auto';
  const sweepDirection = ['auto', 'lefttoright', 'righttoleft'].includes(requestedSweepDirection)
    ? requestedSweepDirection
    : 'auto';
  const preferMinTurns = req.body?.preferMinTurns !== false;
  const requestedMode = typeof req.body?.mode === 'string' ? req.body.mode.trim().toLowerCase() : null;
  const coverageWidthM = Number.isFinite(Number(req.body?.coverageWidthM)) && Number(req.body?.coverageWidthM) > 0
    ? Number(req.body.coverageWidthM)
    : 0.5;

  const normalizedSalt = Number.isFinite(Number(saltPct))
    ? Math.max(0, Math.min(100, Math.round(Number(saltPct))))
    : 100;
  const normalizedBrine = Number.isFinite(Number(brinePct))
    ? Math.max(0, Math.min(100, Math.round(Number(brinePct))))
    : 100;

  const requestedStartPoint = normalizeLatLonPoint(start, null);
  const startPoint = requestedStartPoint
    ?? normalizeLatLonPoint(state.robot, null)
    ?? normalizeLatLonPoint(state.homePoint, null)
    ?? normalizeLatLonPoint(state.baseStation, null);
  if (!startPoint) {
    return res.status(400).json({ ok: false, error: 'no start point available' });
  }

  let mode = requestedMode;
  if (!mode) {
    mode = goal && typeof goal.lat === 'number' && typeof goal.lon === 'number' ? 'goal' : 'coverage';
  }

  if (!['coverage', 'goal'].includes(mode)) {
    return res.status(400).json({ ok: false, error: 'mode must be either "coverage" or "goal"' });
  }

  if (mode !== 'coverage' && (!goal || typeof goal.lat !== 'number' || typeof goal.lon !== 'number')) {
    return res.status(400).json({ ok: false, error: 'goal.lat/lon required for goal mode' });
  }

  const returnToBase = mode === 'coverage'
    ? req.body?.returnToBase !== false
    : req.body?.returnToBase === true;

  const path = buildMissionPath({
    mode,
    startPoint,
    goalPoint: goal ?? null,
    coverageWidthM,
    saltPct: normalizedSalt,
    brinePct: normalizedBrine,
    homePoint,
    returnToBase,
    sweepDirection,
    preferMinTurns,
  });

  if (!path.ok) {
    state.lastPath = [];
    state.lastArrows = [];
    return res.status(400).json({ ok: false, error: path.reason });
  }

  res.json({
    ok: true,
    mode,
    coverageWidthM: mode === 'coverage' ? coverageWidthM : null,
    sweepDirection: mode === 'coverage' ? sweepDirection : null,
    returnToBase,
    homePoint: cloneJsonSafe(state.homePoint, null),
    points: state.lastPath,
    arrows: state.lastArrows,
    meta: path.meta ?? null,
  });
});

// ---------------------------------------------------------------------------
// Phase B: Mission lifecycle endpoints
// ---------------------------------------------------------------------------

// POST /api/mission/start  — transitions CONFIGURING → RUNNING
// Also pushes staged waypoints (from lastPath) to STM32, then sends AUTO.
app.post(API.MISSION_START, requireApp, async (_req, res) => {
  try {
    const result = await startMissionAction('mission.start');
    if (!result.ok) {
      return res.status(result.status ?? 500).json({ ok: false, error: result.error });
    }
    return res.json({ ok: true, mission: result.mission });
  } catch (err) {
    return res.status(400).json({ ok: false, error: err.message });
  }
});

// POST /api/mission/pause  — transitions RUNNING → PAUSED
app.post(API.MISSION_PAUSE, requireApp, async (req, res) => {
  const reason = req.body?.reason ?? null;
  try {
    const result = await pauseMissionAction(reason, 'mission.pause');
    if (!result.ok) {
      return res.status(result.status ?? 500).json({ ok: false, error: result.error });
    }
    return res.json({ ok: true, mission: result.mission });
  } catch (err) {
    return res.status(400).json({ ok: false, error: err.message });
  }
});

// POST /api/mission/resume  — transitions PAUSED → RUNNING
app.post(API.MISSION_RESUME, requireApp, async (_req, res) => {
  try {
    const result = await resumeMissionAction('mission.resume');
    if (!result.ok) {
      return res.status(result.status ?? 500).json({ ok: false, error: result.error });
    }
    return res.json({ ok: true, mission: result.mission });
  } catch (err) {
    return res.status(400).json({ ok: false, error: err.message });
  }
});

// POST /api/mission/complete  — transitions RUNNING → COMPLETED  (operator-declared)
app.post(API.MISSION_COMPLETE, requireApp, async (_req, res) => {
  try {
    const result = await completeMissionAction('mission.complete');
    if (!result.ok) {
      return res.status(result.status ?? 500).json({ ok: false, error: result.error });
    }
    return res.json({ ok: true, mission: result.mission });
  } catch (err) {
    return res.status(400).json({ ok: false, error: err.message });
  }
});

// POST /api/mission/abort  — transitions RUNNING|PAUSED → ABORTED
app.post(API.MISSION_ABORT, requireApp, async (req, res) => {
  const reason = req.body?.reason ?? 'operator';
  try {
    const result = await abortMissionAction(reason, 'mission.abort');
    if (!result.ok) {
      return res.status(result.status ?? 500).json({ ok: false, error: result.error });
    }
    return res.json({ ok: true, mission: result.mission });
  } catch (err) {
    return res.status(400).json({ ok: false, error: err.message });
  }
});

// GET /api/mission/current  — live snapshot
app.get(API.MISSION_CURRENT, requireAppOrBoard, (_req, res) => {
  const m = mission.publicMission();
  if (!m) return res.status(404).json({ ok: false, error: 'No active mission' });
  return res.json({ ok: true, mission: m });
});

app.get(API.MISSION_SCHEDULE, requireAppOrBoard, (_req, res) => {
  return res.json({ ok: true, automation: getAutomationSnapshot() });
});

app.post(API.MISSION_SCHEDULE, requireApp, async (req, res) => {
  const rawAt = Number(req.body?.at ?? req.body?.scheduledRunAt ?? 0);
  const atMs = rawAt > 1e12 ? rawAt : rawAt * 1000;
  const label = typeof req.body?.label === 'string' ? req.body.label : '';
  const notify = req.body?.notify !== false;

  if (!Number.isFinite(atMs) || atMs <= Date.now() + 15000) {
    return res.status(400).json({ ok: false, error: 'Choose a future time at least 15 seconds from now.' });
  }
  if (![MISSION_STATE.CONFIGURING, MISSION_STATE.PAUSED].includes(currentMissionState())) {
    return res.status(409).json({ ok: false, error: 'Prepare the route first, then arm the automatic run while the mission is configuring or paused.' });
  }
  if (!Array.isArray(state.lastPath) || state.lastPath.length < 2) {
    return res.status(409).json({ ok: false, error: 'Build a route before arming automatic autonomy.' });
  }
  if (pathHasZeroDispersion()) {
    return res.status(409).json({ ok: false, error: 'Automatic autonomy is blocked while the path uses 0% salt and 0% brine.' });
  }

  let waypointPrepNote = null;
  const bridgeStatus = bridge.getStatus();
  if (bridgeStatus.wpPushState !== 'committed') {
    const pushed = await pushPathWaypoints(null, 'mission.schedule.arm');
    if (!pushed.ok) {
      return res.status(pushed.status ?? 409).json({ ok: false, error: pushed.error ?? 'Failed to pre-stage waypoints for the scheduled run.' });
    }
    if (pushed.truncated) {
      waypointPrepNote = pushed.queuedRemote
        ? `armed; queued ${pushed.sent}/${pushed.totalPoints} waypoints for remote staging`
        : `armed; pre-staged ${pushed.sent}/${pushed.totalPoints} waypoints locally`;
    } else {
      waypointPrepNote = pushed.queuedRemote
        ? 'armed; waypoints queued for remote staging'
        : 'armed; waypoints pre-staged';
    }
  }

  const automation = armAutomationSchedule({ atMs, label, notify });
  if (waypointPrepNote) {
    state.automation = {
      ...state.automation,
      lastResult: waypointPrepNote,
      lastError: null,
    };
  }
  scheduleRuntimeStateSave('mission.schedule.armed', { immediate: true });
  publishSupervision();
  publishOperator();
  return res.json({ ok: true, automation: getAutomationSnapshot() });
});

app.post(API.MISSION_SCHEDULE_CANCEL, requireApp, (_req, res) => {
  const automation = clearAutomationSchedule('cancelled', null);
  scheduleRuntimeStateSave('mission.schedule.cancelled', { immediate: true });
  publishSupervision();
  publishOperator();
  return res.json({ ok: true, automation });
});

// GET /api/mission/history?limit=50
app.get(API.MISSION_HISTORY, requireAppOrBoard, (req, res) => {
  const limit = Math.min(Number(req.query.limit ?? 50), 500);
  const rows = db.listMissions(limit);
  return res.json({ ok: true, missions: rows });
});

// GET /api/mission/:id/events?type=<EVENT_TYPE>
app.get(API.MISSION_EVENTS, requireAppOrBoard, (req, res) => {
  const id   = Number(req.params.id);
  const type = req.query.type ?? null;
  if (!Number.isInteger(id) || id < 1) {
    return res.status(400).json({ ok: false, error: 'Invalid mission id' });
  }
  const m = db.getMission(id);
  if (!m) return res.status(404).json({ ok: false, error: 'Mission not found' });
  const events = type
    ? db.getMissionEventsByType(id, type)
    : db.getMissionEvents(id);
  return res.json({ ok: true, mission: m, events });
});

// ---------------------------------------------------------------------------
// Phase C: LoRa bridge endpoints
// ---------------------------------------------------------------------------

// GET /api/lora/status  — bridge + WP push state
app.get(API.LORA_STATUS, requireAppOrBoard, async (_req, res) => {
  const lora = await bridge.refreshStatus();
  res.json({ ok: true, lora });
});

app.get(API.BRIDGE_SYNC, requireAppOrBoard, async (_req, res) => {
  const coverage = state.coverage
    ? {
        width: state.coverage.width,
        height: state.coverage.height,
        cellSizeM: state.coverage.cellSizeM,
        stats: coverageStats(state.coverage),
      }
    : null;

  res.json({
    ok: true,
    role: _req.clientRole ?? 'unknown',
    now: Date.now(),
    mission: mission.publicMission(),
    lora: await bridge.refreshStatus(),
    baseStation: state.baseStation,
    boundary: state.boundary,
    lastPath: state.lastPath,
    coverage,
    robot: state.robot,
    lastCommand: state.lastCommand,
    lastCommandId: state.lastCommandId,
    lastCommandStatus: state.lastCommandStatus,
    lastCommandAt: state.lastCommandAt,
    commandHistory: state.commandHistory,
    lastFault: state.lastFault,
    telemetryStale: isTelemetryStale(),
    connectivity: buildConnectionState(),
    persistence: state.persistence,
  });
});

// POST /api/lora/push-waypoints  — manual WP push trigger
// Body: { points: [{ lat, lon, salt, brine }, ...] }
// If points is omitted uses state.lastPath.
app.get(API.TEST_MENU, requireAppOrBoard, async (_req, res) => {
  res.json({
    ok: true,
    tests: buildTestMenu(),
    lora: await bridge.refreshStatus(),
    websocket: buildWebSocketDiagnostics(),
    gps: getRobotGpsStatus(),
    mission: mission.publicMission(),
    robot: state.robot,
    safety: state.safety,
    persistence: state.persistence,
    connectivity: buildConnectionState(),
    allowedActions: buildAllowedActions(),
    alerts: buildAlerts(),
    commandHistory: state.commandHistory,
  });
});

app.post(API.TEST_RUN, requireAppOrBoard, async (req, res) => {
  const actionId = String(req.body?.actionId ?? '').trim();
  if (!actionId) {
    return res.status(400).json({ ok: false, error: 'actionId is required' });
  }

  const output = await runTestMenuAction(actionId, req.body ?? {});
  return res.status(output.ok ? 200 : 409).json(output);
});

app.post(API.LORA_PUSH_WP, requireApp, async (req, res) => {
  const result = await pushPathWaypoints(req.body?.points, 'lora_bridge');
  if (!result.ok) {
    return res.status(result.status ?? 500).json({ ok: false, error: result.error, sent: result.sent });
  }
  return res.json({ ok: true, sent: result.sent });
});

app.get(API.OPERATOR_WORKFLOWS, requireApp, (_req, res) => {
  return res.json({
    ok: true,
    workflows: buildOperatorWorkflows(),
    notes: state.operator.notes,
    allowedActions: buildAllowedActions(),
  });
});

app.post(API.OPERATOR_WORKFLOW_STEP, requireApp, (req, res) => {
  const { workflowId, stepId } = req.params;
  const { checked = true, actor = 'operator', note = null } = req.body ?? {};

  const workflow = buildOperatorWorkflows().find((candidate) => candidate.id === workflowId);
  if (!workflow) {
    return res.status(404).json({ ok: false, error: 'Workflow not found' });
  }

  const step = workflow.steps.find((candidate) => candidate.id === stepId);
  if (!step) {
    return res.status(404).json({ ok: false, error: 'Workflow step not found' });
  }
  if (step.kind !== 'manual') {
    return res.status(409).json({ ok: false, error: 'Derived workflow steps cannot be acknowledged manually' });
  }
  if (!step.ready && checked) {
    return res.status(409).json({ ok: false, error: 'Workflow step is not ready for acknowledgement yet' });
  }

  const ack = setWorkflowAck(workflowId, stepId, Boolean(checked), { actor, note });
  db.logEvent(mission.currentId(), EVENT_TYPE.OPERATOR_WORKFLOW_UPDATED, {
    workflowId,
    stepId,
    checked: Boolean(checked),
    actor,
    note,
  });

  publishOperator();
  publishSupervision();

  return res.json({
    ok: true,
    ack,
    workflows: buildOperatorWorkflows(),
  });
});

app.get(API.OPERATOR_NOTES, requireApp, (_req, res) => {
  return res.json({ ok: true, notes: state.operator.notes });
});

app.post(API.OPERATOR_NOTES, requireApp, (req, res) => {
  const text = typeof req.body?.text === 'string' ? req.body.text.trim() : '';
  const category = typeof req.body?.category === 'string' && req.body.category.trim()
    ? req.body.category.trim()
    : 'general';
  const actor = typeof req.body?.actor === 'string' && req.body.actor.trim()
    ? req.body.actor.trim()
    : 'operator';

  if (!text) {
    return res.status(400).json({ ok: false, error: 'text is required' });
  }

  const note = addOperatorNote({ text, category, actor });
  publishOperator();
  publishSupervision();
  return res.json({ ok: true, note, notes: state.operator.notes });
});

wss.on('connection', (socket) => {
  metrics.wsConnectionsOpened += 1;
  metrics.wsPeakClients = Math.max(metrics.wsPeakClients, wss.clients.size);
  socket.send(JSON.stringify({ event: WS_EVENT.STATE_SNAPSHOT, payload: publicState(), at: Date.now() }));
  socket.on('message', (rawMessage) => {
    let parsed = null;
    try {
      parsed = JSON.parse(String(rawMessage ?? ''));
    } catch {
      return;
    }

    const event = typeof parsed?.event === 'string' ? parsed.event.trim().toLowerCase() : '';
    const payload = parsed?.payload;

    if (event === 'manual.drive') {
      queueWsManualDrive(payload);
      return;
    }

    if (event === 'manual.command') {
      const command = normalizeCommand(payload?.command);
      if (!command) {
        return;
      }
      void dispatchCommand(command, {
        wireCommand: typeof payload?.wireCommand === 'string' && payload.wireCommand.trim()
          ? payload.wireCommand.trim().toUpperCase()
          : compactWireCommandForCmd(command),
        waitForAck: false,
        waitForState: false,
        source: 'app.ws.manual-command',
      });
    }
  });
  socket.on('close', () => {
    metrics.wsConnectionsClosed += 1;
  });
});

const port = Number(process.env.PORT ?? 8080);

const safetyTimer = setInterval(() => {
  void maybeRunScheduledMission();
  void enforceTelemetryFailsafePolicy();
  void enforceGpsFailsafePolicy();
  void enforceGeofenceFailsafePolicy('monitor');
}, Math.max(100, SAFETY_MONITOR_INTERVAL_MS));
if (typeof safetyTimer.unref === 'function') safetyTimer.unref();

const baseStationLoRaPollTimer = setInterval(() => {
  void pollBaseStationLoRaTelemetry();
}, Math.max(120, BASE_STATION_LORA_POLL_MS));
if (typeof baseStationLoRaPollTimer.unref === 'function') baseStationLoRaPollTimer.unref();

const manualDriveWsTimer = setInterval(() => {
  if (latestWsManualDrive && (Date.now() - latestWsManualDriveQueuedAt) >= 0) {
    void flushQueuedWsManualDrive();
  }
}, Math.max(10, MANUAL_DRIVE_WS_FLUSH_MS));
if (typeof manualDriveWsTimer.unref === 'function') manualDriveWsTimer.unref();

const retentionTimer = setInterval(() => {
  if (!Number.isFinite(EVENT_RETENTION_DAYS) || EVENT_RETENTION_DAYS <= 0) {
    return;
  }
  const cutoffMs = Date.now() - Math.floor(EVENT_RETENTION_DAYS * 86400000);
  const pruned = db.pruneEventsBefore(cutoffMs);
  metrics.eventPruneRuns += 1;
  metrics.eventPrunedRows += pruned;
}, Math.max(60000, EVENT_RETENTION_PRUNE_INTERVAL_MS));
if (typeof retentionTimer.unref === 'function') retentionTimer.unref();

const rateLimitGcTimer = setInterval(() => {
  const cutoff = Date.now() - REQUEST_RATE_LIMIT_WINDOW_MS;
  for (const [key, value] of rateLimitStore.entries()) {
    if (value.windowStart < cutoff) {
      rateLimitStore.delete(key);
    }
  }
}, Math.max(30000, REQUEST_RATE_LIMIT_WINDOW_MS));
if (typeof rateLimitGcTimer.unref === 'function') rateLimitGcTimer.unref();

server.listen(port, () => {
  console.log(`[robot-lora-server] listening on http://localhost:${port}`);
});
