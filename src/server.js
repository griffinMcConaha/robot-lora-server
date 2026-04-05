const express = require('express');
const cors = require('cors');
const http = require('http');
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
const { latLonToLocal } = require('./geo');
const { findPath, findCoveragePath } = require('./pathing');
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

// Publish mission transitions over WebSocket
mission.onTransition((m) => publish(WS_EVENT.MISSION_UPDATED, m));

const app = express();
app.use(cors());
app.use(express.json({ limit: '1mb' }));
app.use(express.text({ type: 'text/*', limit: '256kb' }));
// Serve dashboard UI
app.use(express.static(path.join(__dirname, '..', 'public')));

const server = http.createServer(app);
const wss = new WebSocketServer({ server });

const SUPERVISION_TELEMETRY_STALE_MS = Number(process.env.SUPERVISION_TELEMETRY_STALE_MS ?? 15000);
const TELEMETRY_FAILSAFE_ENABLED = String(process.env.TELEMETRY_FAILSAFE_ENABLED ?? '1') !== '0';
const TELEMETRY_FAILSAFE_ACTION = String(process.env.TELEMETRY_FAILSAFE_ACTION ?? CMD.ESTOP).trim().toUpperCase();
const TELEMETRY_FAILSAFE_COOLDOWN_MS = Number(process.env.TELEMETRY_FAILSAFE_COOLDOWN_MS ?? Math.max(SUPERVISION_TELEMETRY_STALE_MS, 5000));
const SAFETY_MONITOR_INTERVAL_MS = Number(process.env.SAFETY_MONITOR_INTERVAL_MS ?? 500);
const GEOFENCE_FAILSAFE_ENABLED = String(process.env.GEOFENCE_FAILSAFE_ENABLED ?? '1') !== '0';
const GEOFENCE_FAILSAFE_ACTION = String(process.env.GEOFENCE_FAILSAFE_ACTION ?? CMD.ESTOP).trim().toUpperCase();
const GEOFENCE_FAILSAFE_COOLDOWN_MS = Number(process.env.GEOFENCE_FAILSAFE_COOLDOWN_MS ?? 5000);
const EVENT_RETENTION_DAYS = Number(process.env.EVENT_RETENTION_DAYS ?? 30);
const EVENT_RETENTION_PRUNE_INTERVAL_MS = Number(process.env.EVENT_RETENTION_PRUNE_INTERVAL_MS ?? 3600000);
const MAX_INPUT_AREA_CELLS = Number(process.env.MAX_INPUT_AREA_CELLS ?? 60000);
const MIN_INPUT_AREA_CELL_SIZE_M = Number(process.env.MIN_INPUT_AREA_CELL_SIZE_M ?? 0.5);
const MAX_INPUT_AREA_CELL_SIZE_M = Number(process.env.MAX_INPUT_AREA_CELL_SIZE_M ?? 8.0);
const COVERAGE_MARK_RADIUS_M = Number(process.env.COVERAGE_MARK_RADIUS_M ?? 0.5);
const REQUEST_RATE_LIMIT_WINDOW_MS = Number(process.env.REQUEST_RATE_LIMIT_WINDOW_MS ?? 60000);
const COMMAND_RATE_LIMIT_PER_WINDOW = Number(process.env.COMMAND_RATE_LIMIT_PER_WINDOW ?? 240);
const TELEMETRY_RATE_LIMIT_PER_WINDOW = Number(process.env.TELEMETRY_RATE_LIMIT_PER_WINDOW ?? 6000);
const COMMAND_STATE_CONFIRM_TIMEOUT_MS = Number(process.env.COMMAND_STATE_CONFIRM_TIMEOUT_MS ?? 8000);
const COMMAND_STATE_CONFIRM_POLL_MS = Number(process.env.COMMAND_STATE_CONFIRM_POLL_MS ?? 150);
const GPS_READY_MIN_SAT = Number(process.env.GPS_READY_MIN_SAT ?? 5);
const GPS_READY_MAX_HDOP = Number(process.env.GPS_READY_MAX_HDOP ?? 3.0);
const GPS_FAILSAFE_ENABLED = String(process.env.GPS_FAILSAFE_ENABLED ?? '1') !== '0';
const GPS_FAILSAFE_ACTION = String(process.env.GPS_FAILSAFE_ACTION ?? CMD.PAUSE).trim().toUpperCase();
const GPS_FAILSAFE_COOLDOWN_MS = Number(process.env.GPS_FAILSAFE_COOLDOWN_MS ?? 5000);

function parseKeyList(value) {
  if (typeof value !== 'string') return [];
  return value
    .split(',')
    .map((token) => token.trim())
    .filter(Boolean);
}

const APP_API_KEYS = new Set([
  ...parseKeyList(process.env.APP_API_KEYS),
  ...(typeof process.env.APP_API_KEY === 'string' && process.env.APP_API_KEY.trim() ? [process.env.APP_API_KEY.trim()] : []),
]);
const BOARD_API_KEYS = new Set([
  ...parseKeyList(process.env.BOARD_API_KEYS),
  ...(typeof process.env.BOARD_API_KEY === 'string' && process.env.BOARD_API_KEY.trim() ? [process.env.BOARD_API_KEY.trim()] : []),
]);
const API_AUTH_ENABLED = APP_API_KEYS.size > 0 || BOARD_API_KEYS.size > 0;

const rateLimitStore = new Map();

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
};

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
  operator: createOperatorState(),
};

let safetyMonitorInFlight = false;

const DRIVE_COMMANDS = new Set([
  CMD.FORWARD,
  CMD.BACKWARD,
  CMD.LEFT,
  CMD.RIGHT,
  CMD.STOP,
]);

function pathHasZeroDispersion(points = state.lastPath) {
  return Array.isArray(points)
    && points.length > 0
    && points.every((point) => Number(point?.salt ?? 0) <= 0 && Number(point?.brine ?? 0) <= 0);
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
    ...CMD_ALIASES,
    ...Object.fromEntries(Object.values(CMD).map((v) => [v, v])),
  };

  return aliases[token] ?? token;
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

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
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

function resolveTelemetryFailsafeAction() {
  if (TELEMETRY_FAILSAFE_ACTION === CMD.PAUSE) return CMD.PAUSE;
  return CMD.ESTOP;
}

function resolveGeofenceFailsafeAction() {
  if (GEOFENCE_FAILSAFE_ACTION === CMD.PAUSE) return CMD.PAUSE;
  return CMD.ESTOP;
}

function isRobotInsideCoverageArea(robotPoint) {
  if (!state.coverage || !robotPoint) return true;
  const { row, col } = worldToGrid(state.coverage, robotPoint);
  if (!withinGrid(state.coverage, row, col)) return false;
  return Boolean(state.coverage.cells[row][col]?.inside);
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
}

function telemetryAgeMs(now = Date.now()) {
  return supervisionTelemetryAgeMs(state.robot, now);
}

function isTelemetryStale(now = Date.now()) {
  return supervisionIsTelemetryStale(state.robot, SUPERVISION_TELEMETRY_STALE_MS, now);
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
    return null;
  }

  const ack = {
    checked: true,
    actor: meta.actor ?? 'operator',
    note: meta.note ?? null,
    at: Date.now(),
  };
  state.operator.workflowAcks[workflowId][stepId] = ack;
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
  });
}

function buildAlerts(now = Date.now()) {
  const gpsStatus = getRobotGpsStatus();
  return buildSupervisionAlerts({
    missionState: currentMissionState(),
    wpPushState: bridge.getStatus().wpPushState,
    hasCoverage: Boolean(state.coverage),
    hasPath: state.lastPath.length > 0,
    zeroDispersionPath: pathHasZeroDispersion(),
    gpsReady: gpsStatus.ready,
    gpsReason: gpsStatus.reason,
    telemetryStale: isTelemetryStale(now),
    telemetryAge: telemetryAgeMs(now),
    lastFault: state.lastFault,
    safety: state.safety,
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

function buildSupervisionSummary() {
  const now = Date.now();
  const robotAgeMs = telemetryAgeMs(now);
  const gpsStatus = getRobotGpsStatus();
  return {
    mission: mission.publicMission(),
    lora: bridge.getStatus(),
    robot: state.robot
      ? {
          lat: state.robot.lat,
          lon: state.robot.lon,
          heading: state.robot.heading,
          speed: state.robot.speed,
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
    alerts: buildAlerts(now),
    allowedActions: buildAllowedActions(),
    workflows: buildOperatorWorkflows(),
    notes: state.operator.notes,
  };
}

async function enforceGeofenceFailsafePolicy(triggerSource = 'telemetry') {
  if (!GEOFENCE_FAILSAFE_ENABLED) return;
  if (safetyMonitorInFlight) return;
  if (!mission.isRunning()) return;
  if (!state.robot || !state.coverage) return;
  if (isRobotInsideCoverageArea(state.robot)) return;

  const now = Date.now();
  if ((now - (state.safety.geofenceFailsafeAt || 0)) < GEOFENCE_FAILSAFE_COOLDOWN_MS) {
    return;
  }

  const action = resolveGeofenceFailsafeAction();
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
  });
}

function buildFinalMissionStats() {
  const stats = state.coverage ? coverageStats(state.coverage) : null;
  return {
    coveragePct: getCoveragePct(stats),
    faultCount: mission.publicMission()?.faultCount ?? 0,
    cmdCount: mission.publicMission()?.cmdCount ?? 0,
  };
}

function arbitrateCommand(cmd) {
  const missionState = currentMissionState();
  const wpPushState = bridge.getStatus().wpPushState;

  if (Object.values(CMD).includes(cmd)) {
    if (cmd === CMD.ESTOP || cmd === CMD.PAUSE || cmd === CMD.MANUAL || DRIVE_COMMANDS.has(cmd)) {
      return { ok: true };
    }
    if (cmd === CMD.RESET) {
      if ([MISSION_STATE.PAUSED, MISSION_STATE.ABORTED, MISSION_STATE.ERROR].includes(missionState)) {
        return { ok: true };
      }
      return { ok: false, status: 409, error: 'RESET allowed only when mission is PAUSED, ABORTED, or ERROR' };
    }
    if (cmd === CMD.AUTO) {
      if (wpPushState === 'committed') {
        return { ok: true };
      }
      return { ok: false, status: 409, error: 'AUTO requires committed waypoints' };
    }
  }

  if (isWaypointCommand(cmd)) {
    if ([MISSION_STATE.CONFIGURING, MISSION_STATE.PAUSED].includes(missionState)) {
      return { ok: true };
    }
    return { ok: false, status: 409, error: 'Waypoint commands are allowed only while mission is CONFIGURING or PAUSED' };
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
    return;
  }

  if (cmd === CMD.RESET) {
    const missionState = currentMissionState();
    if ([MISSION_STATE.ABORTED, MISSION_STATE.ERROR, MISSION_STATE.COMPLETED].includes(missionState)) {
      mission.reset();
      bridge.resetWpState();
    }
  }
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

async function dispatchCommand(cmd, options = {}) {
  const { syncMission = true, source = 'api.command', waitForAck = false, waitForState = false } = options;

  const decision = arbitrateCommand(cmd);
  if (!decision.ok) {
    metrics.commandsFailed += 1;
    return { ok: false, status: decision.status, error: decision.error };
  }

  const result = await bridge.sendCommand(cmd, { waitForAck });
  if (!result.ok) {
    metrics.commandsFailed += 1;
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

  state.lastCommand = cmd;
  state.lastCommandAt = Date.now();
  publish(WS_EVENT.COMMAND_RECEIVED, { cmd, at: state.lastCommandAt });
  db.logEvent(mission.currentId(), EVENT_TYPE.COMMAND_SENT, { cmd, source });
  mission.recordCommand();

  if (syncMission) {
    syncMissionToCommand(cmd);
  }

  if (waitForState) {
    const expectedState = expectedRobotStateForCommand(cmd);
    const confirmation = await waitForRobotState(expectedState, {
      afterTimestampMs: state.lastCommandAt,
    });
    if (!confirmation.ok) {
      metrics.commandsFailed += 1;
      return {
        ok: false,
        status: 504,
        error: confirmation.error,
        detail: confirmation,
      };
    }
  }

  publishSupervision();
  publishOperator();

  metrics.commandsDispatched += 1;

  return { ok: true, status: result.status, body: result.body };
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

async function pushPathWaypoints(rawPoints = null, source = 'lora_bridge') {
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
  if (pathHasZeroDispersion(points)) {
    return { ok: false, status: 409, error: 'Waypoint push blocked: path has 0% salt and 0% brine' };
  }

  const result = await bridge.pushWaypoints(points);
  if (!result.ok) {
    return { ok: false, status: 502, error: result.error, sent: result.sent };
  }
  db.logEvent(mission.currentId(), EVENT_TYPE.PATH_PLANNED, { wpPushCount: result.sent, source });
  publishSupervision();
  publishOperator();
  return { ok: true, status: 200, sent: result.sent, points };
}

async function startMissionAction(source = 'mission.start') {
  if (currentMissionState() !== MISSION_STATE.CONFIGURING) {
    return { ok: false, status: 409, error: 'Mission must be CONFIGURING before start' };
  }
  if (pathHasZeroDispersion()) {
    return { ok: false, status: 409, error: 'Mission start blocked: path has 0% salt and 0% brine' };
  }
  const gpsStatus = getRobotGpsStatus();
  if (!gpsStatus.ready) {
    return { ok: false, status: 409, error: `Mission start blocked: ${gpsStatus.reason}` };
  }

  const loraStatus = bridge.getStatus();
  if (loraStatus.wpPushState !== 'committed' && state.lastPath && state.lastPath.length > 0) {
    const pushed = await pushPathWaypoints(null, source);
    if (!pushed.ok) return pushed;
  }

  if (bridge.getStatus().wpPushState !== 'committed') {
    return { ok: false, status: 409, error: 'Mission start requires a committed waypoint set' };
  }

  const dispatched = await dispatchCommand(CMD.AUTO, { syncMission: false, source, waitForAck: true, waitForState: true });
  if (!dispatched.ok) {
    return { ok: false, status: dispatched.status ?? 500, error: dispatched.error };
  }

  const current = mission.start();
  publishSupervision();
  return { ok: true, status: 200, mission: current };
}

async function pauseMissionAction(reason = null, source = 'mission.pause') {
  if (currentMissionState() !== MISSION_STATE.RUNNING) {
    return { ok: false, status: 409, error: 'Mission must be RUNNING before pause' };
  }
  const dispatched = await dispatchCommand(CMD.PAUSE, { syncMission: false, source, waitForAck: true, waitForState: true });
  if (!dispatched.ok) {
    return { ok: false, status: dispatched.status ?? 500, error: dispatched.error };
  }
  const current = mission.pause(reason);
  publishSupervision();
  return { ok: true, status: 200, mission: current };
}

async function resumeMissionAction(source = 'mission.resume') {
  if (currentMissionState() !== MISSION_STATE.PAUSED) {
    return { ok: false, status: 409, error: 'Mission must be PAUSED before resume' };
  }
  const gpsStatus = getRobotGpsStatus();
  if (!gpsStatus.ready) {
    return { ok: false, status: 409, error: `Mission resume blocked: ${gpsStatus.reason}` };
  }
  if (bridge.getStatus().wpPushState !== 'committed') {
    return { ok: false, status: 409, error: 'Mission resume requires committed waypoints' };
  }
  const dispatched = await dispatchCommand(CMD.AUTO, { syncMission: false, source, waitForAck: true, waitForState: true });
  if (!dispatched.ok) {
    return { ok: false, status: dispatched.status ?? 500, error: dispatched.error };
  }
  const current = mission.resume();
  publishSupervision();
  return { ok: true, status: 200, mission: current };
}

async function completeMissionAction(source = 'mission.complete') {
  if (currentMissionState() !== MISSION_STATE.RUNNING) {
    return { ok: false, status: 409, error: 'Mission must be RUNNING before complete' };
  }
  const dispatched = await dispatchCommand(CMD.PAUSE, { syncMission: false, source, waitForAck: true, waitForState: true });
  if (!dispatched.ok) {
    return { ok: false, status: dispatched.status ?? 500, error: dispatched.error };
  }
  const stats = state.coverage ? coverageStats(state.coverage) : {};
  const current = mission.complete({
    coveragePct: getCoveragePct(stats),
    faultCount: db.getMissionEventSummary(mission.currentId() ?? 0)?.faultCount ?? 0,
    cmdCount: db.getMissionEventSummary(mission.currentId() ?? 0)?.cmdCount ?? 0,
  });
  bridge.resetWpState();
  publishSupervision();
  return { ok: true, status: 200, mission: current };
}

async function abortMissionAction(reason = 'operator', source = 'mission.abort') {
  if (![MISSION_STATE.RUNNING, MISSION_STATE.PAUSED].includes(currentMissionState())) {
    return { ok: false, status: 409, error: 'Mission must be RUNNING or PAUSED before abort' };
  }
  const dispatched = await dispatchCommand(CMD.ESTOP, { syncMission: false, source, waitForAck: true, waitForState: true });
  if (!dispatched.ok) {
    return { ok: false, status: dispatched.status ?? 500, error: dispatched.error };
  }
  const stats = state.coverage ? coverageStats(state.coverage) : {};
  const current = mission.abort(reason, {
    coveragePct: getCoveragePct(stats),
    faultCount: db.getMissionEventSummary(mission.currentId() ?? 0)?.faultCount ?? 0,
    cmdCount: db.getMissionEventSummary(mission.currentId() ?? 0)?.cmdCount ?? 0,
  });
  bridge.resetWpState();
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
      id: 'safe-off',
      title: 'Safe Outputs Off',
      kind: 'dispersion',
      description: 'Send the safe-off sequence to stop outputs and relays.',
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
      const result = await bridge.sendCommand('TEST SALT 50', { waitForAck: false });
      return { ok: result.ok, actionId: resolved.id, result: { ...result, command: 'TEST SALT 50' } };
    }
    case 'brine-50': {
      const result = await bridge.sendCommand('TEST BRINE 50', { waitForAck: false });
      return { ok: result.ok, actionId: resolved.id, result: { ...result, command: 'TEST BRINE 50' } };
    }
    case 'agitator-on': {
      const result = await bridge.sendCommand('AGITATOR ON', { waitForAck: false });
      return { ok: result.ok, actionId: resolved.id, result: { ...result, command: 'AGITATOR ON' } };
    }
    case 'thrower-on': {
      const result = await bridge.sendCommand('THROWER ON', { waitForAck: false });
      return { ok: result.ok, actionId: resolved.id, result: { ...result, command: 'THROWER ON' } };
    }
    case 'relay-on': {
      const result = await bridge.sendCommand('RELAY ON', { waitForAck: false });
      return { ok: result.ok, actionId: resolved.id, result: { ...result, command: 'RELAY ON' } };
    }
    case 'safe-off': {
      const commands = ['PCT:0', 'AGITATOR OFF', 'THROWER OFF', 'RELAY OFF'];
      const steps = [];
      for (const command of commands) {
        const result = await bridge.sendCommand(command, { waitForAck: false });
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
        const result = await bridge.sendCommand(command, { waitForAck: false });
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
      const result = await bridge.sendCommand(command, { waitForAck: false });
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

  if (body?.robot && typeof body.robot.lat === 'number' && typeof body.robot.lon === 'number') {
    return {
      robot: {
        lat: body.robot.lat,
        lon: body.robot.lon,
        heading: Number(body.robot.heading ?? body.heading?.yaw ?? 0),
        speed: Number(body.robot.speed ?? 0),
        gpsFix: Boolean(body.robot.fix ?? body.gps?.fix ?? false),
        gpsHdop: Number(body.robot.hdop ?? body.gps?.hdop ?? 0),
        gpsSat: Number(body.robot.sat ?? body.gps?.sat ?? 0),
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
        gpsFix: Boolean(body.gps.fix),
        gpsHdop: Number(body.gps.hdop ?? 0),
        gpsSat: Number(body.gps.sat ?? 0),
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
    mission: mission.publicMission(),
    operator: {
      notes: state.operator.notes,
      workflows: buildOperatorWorkflows(),
    },
  };
}

app.get(API.HEALTH, (_req, res) => {
  const bridgeStatus = bridge.getStatus();
  const dbOk = db.ping();
  const missionState = currentMissionState();
  const telemetryStaleWhileRunning = missionState === MISSION_STATE.RUNNING && isTelemetryStale();

  const checks = {
    db: dbOk,
    bridge: !bridgeStatus.degraded,
    telemetry: !telemetryStaleWhileRunning,
  };

  const ready = Object.values(checks).every(Boolean);
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
    missionState,
    telemetryStale: isTelemetryStale(),
    lora: {
      degraded: bridgeStatus.degraded,
      consecutiveFailures: bridgeStatus.consecutiveFailures,
      lastSuccessAt: bridgeStatus.lastSuccessAt,
      lastErrorAt: bridgeStatus.lastErrorAt,
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
    },
  });
});

app.get(API.SUPERVISION_SUMMARY, requireAppOrBoard, (_req, res) => {
  res.json({ ok: true, summary: buildSupervisionSummary() });
});

app.get(API.STATUS, requireAppOrBoard, (_req, res) => {
  res.json({
    battery: 85,
    state: state.robot?.state ?? ROBOT_STATE.IDLE,
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

  const dispatched = await dispatchCommand(cmd);
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
  const { baseStation, boundary, cellSizeM = 2.0 } = req.body ?? {};
  const startedAt = Date.now();

  if (!baseStation || typeof baseStation.lat !== 'number' || typeof baseStation.lon !== 'number') {
    return res.status(400).json({ ok: false, error: 'baseStation.lat/lon required' });
  }

  if (!Array.isArray(boundary) || boundary.length < 3) {
    return res.status(400).json({ ok: false, error: 'boundary requires at least 3 points' });
  }

  const cellPlan = resolveInputAreaCellSize(boundary, Number(cellSizeM));

  state.baseStation = baseStation;
  state.boundary = boundary;
  state.coverage = buildCoverageMap(boundary, cellPlan.effectiveCellSizeM);
  state.robot = null;
  state.trail = [];
  state.lastPath = [];
  bridge.resetWpState();
  resetOperatorState();

  const stats = coverageStats(state.coverage);
  publish(WS_EVENT.AREA_UPDATED, { baseStation, boundary, stats });

  // Create / replace mission record in CONFIGURING state
  // (Resetting area resets the mission lifecycle to CONFIGURING)
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
          faultCount:  mission.publicMission()?.faultCount ?? 0,
          cmdCount:    mission.publicMission()?.cmdCount   ?? 0,
        });
      } catch (_) { /* ignore */ }
      try {
        mission.reset();
      } catch (_) { /* ignore */ }
    }
  }
  mission.configure({ baseStation, boundary, cellSizeM: cellPlan.effectiveCellSizeM });
  publishOperator();
  publishSupervision();

  return res.json({
    ok: true,
    stats,
    requestedCellSizeM: cellPlan.requestedCellSizeM,
    cellSizeM: cellPlan.effectiveCellSizeM,
    cellSizeAdjusted: cellPlan.adjusted,
    estimatedCells: cellPlan.estimatedCells,
    buildMs: Date.now() - startedAt,
  });
});

app.post(API.TELEMETRY, requireBoard, rateLimitTelemetry, async (req, res) => {
  const parsed = parseIncomingTelemetry(req.body);
  if (!parsed) {
    metrics.telemetryRejected += 1;
    return res.status(400).json({ ok: false, error: 'Unsupported telemetry payload' });
  }

  metrics.telemetryAccepted += 1;

  // Handle fault notification separately — don't overwrite robot position.
  if (parsed.isFault) {
    state.lastFault = { fault: parsed.fault, action: parsed.action, at: Date.now() };
    publish(WS_EVENT.FAULT_RECEIVED, { fault: parsed.fault, action: parsed.action, at: state.lastFault.at });

    // Event log + mission counter
    db.logEvent(mission.currentId(), EVENT_TYPE.FAULT_RECEIVED, {
      fault:  parsed.fault,
      action: parsed.action,
    });
    mission.recordFault();

    // Auto-pause a running mission on ESTOP/PAUSE action
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

    return res.json({ ok: true, fault: parsed.fault, action: parsed.action });
  }

  state.robot = {
    lat: parsed.robot.lat,
    lon: parsed.robot.lon,
    heading: parsed.robot.heading,
    speed: parsed.robot.speed,
    gpsFix: parsed.robot.gpsFix,
    gpsHdop: parsed.robot.gpsHdop,
    gpsSat: parsed.robot.gpsSat,
    source: parsed.source,
    state: parsed.stateName ?? state.robot?.state ?? ROBOT_STATE.IDLE,
    timestampMs: Date.now(),
    raw: parsed.raw,
  };

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

  // Event log (sampled — log every telemetry update)
  db.logEvent(mission.currentId(), EVENT_TYPE.TELEMETRY_RECEIVED, {
    lat:     state.robot.lat,
    lon:     state.robot.lon,
    heading: state.robot.heading,
    state:   state.robot.state,
  });

  // Update rolling coverage in mission row
  if (stats) mission.updateCoverage(getCoveragePct(stats));

  await enforceGeofenceFailsafePolicy('telemetry');

  publishSupervision();

  return res.json({ ok: true, coverage: stats });
});

app.get(API.STATE, requireAppOrBoard, (_req, res) => {
  res.json({ ok: true, state: publicState() });
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

  const { goal, start, saltPct, brinePct } = req.body ?? {};
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

  const startPoint = start ?? state.robot ?? state.baseStation;
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

  const path = mode === 'coverage'
    ? findCoveragePath(state.coverage, { swathWidthM: coverageWidthM, startLatLon: startPoint, goalLatLon: goal ?? null })
    : findPath(state.coverage, startPoint, goal);

  if (!path.ok) {
    state.lastPath = [];
    return res.status(400).json({ ok: false, error: path.reason });
  }

  state.lastPath = path.points.map((point) => ({
    ...point,
    salt: normalizedSalt,
    brine: normalizedBrine,
  }));
  publish(WS_EVENT.PATH_UPDATED, {
    mode,
    coverageWidthM: mode === 'coverage' ? coverageWidthM : null,
    points: state.lastPath,
    meta: path.meta ?? null,
  });
  db.logEvent(mission.currentId(), EVENT_TYPE.PATH_PLANNED, {
    mode,
    pointCount: state.lastPath.length,
    saltPct: normalizedSalt,
    brinePct: normalizedBrine,
    coverageWidthM: mode === 'coverage' ? coverageWidthM : null,
  });
  publishSupervision();
  publishOperator();
  res.json({ ok: true, mode, coverageWidthM: mode === 'coverage' ? coverageWidthM : null, points: state.lastPath, meta: path.meta ?? null });
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
    lastCommandAt: state.lastCommandAt,
    lastFault: state.lastFault,
    telemetryStale: isTelemetryStale(),
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
    gps: getRobotGpsStatus(),
    mission: mission.publicMission(),
    robot: state.robot,
    safety: state.safety,
    allowedActions: buildAllowedActions(),
    alerts: buildAlerts(),
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
  socket.send(JSON.stringify({ event: WS_EVENT.STATE_SNAPSHOT, payload: publicState(), at: Date.now() }));
});

const port = Number(process.env.PORT ?? 8080);

const safetyTimer = setInterval(() => {
  void enforceTelemetryFailsafePolicy();
  void enforceGpsFailsafePolicy();
  void enforceGeofenceFailsafePolicy('monitor');
}, Math.max(100, SAFETY_MONITOR_INTERVAL_MS));
if (typeof safetyTimer.unref === 'function') safetyTimer.unref();

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
