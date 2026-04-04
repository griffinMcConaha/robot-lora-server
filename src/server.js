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
  return buildSupervisionAllowedActions({
    missionState: currentMissionState(),
    wpPushState: bridge.getStatus().wpPushState,
    hasCoverage: Boolean(state.coverage),
    hasPath: state.lastPath.length > 0,
    zeroDispersionPath: pathHasZeroDispersion(),
  });
}

function buildAlerts(now = Date.now()) {
  return buildSupervisionAlerts({
    missionState: currentMissionState(),
    wpPushState: bridge.getStatus().wpPushState,
    hasCoverage: Boolean(state.coverage),
    hasPath: state.lastPath.length > 0,
    zeroDispersionPath: pathHasZeroDispersion(),
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
  return {
    mission: mission.publicMission(),
    lora: bridge.getStatus(),
    robot: state.robot
      ? {
          lat: state.robot.lat,
          lon: state.robot.lon,
          heading: state.robot.heading,
          speed: state.robot.speed,
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

async function dispatchCommand(cmd, options = {}) {
  const { syncMission = true, source = 'api.command', waitForAck = false } = options;

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

  publishSupervision();
  publishOperator();

  metrics.commandsDispatched += 1;

  return { ok: true, status: result.status, body: result.body };
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
    if (currentMissionState() !== MISSION_STATE.CONFIGURING) {
      return res.status(409).json({ ok: false, error: 'Mission must be CONFIGURING before start' });
    }
    if (pathHasZeroDispersion()) {
      return res.status(409).json({ ok: false, error: 'Mission start blocked: path has 0% salt and 0% brine' });
    }

    const loraStatus = bridge.getStatus();

    // Push waypoints if we have a path plan and no committed set yet.
    if (loraStatus.wpPushState !== 'committed' && state.lastPath && state.lastPath.length > 0) {
      const points = state.lastPath.map((p) => ({
        lat:   p.lat,
        lon:   p.lon,
        salt:  p.salt  ?? 100,
        brine: p.brine ?? 100,
      }));
      const pushResult = await bridge.pushWaypoints(points);
      if (!pushResult.ok) {
        return res.status(502).json({ ok: false, error: pushResult.error, sent: pushResult.sent });
      }
    }

    if (bridge.getStatus().wpPushState !== 'committed') {
      return res.status(409).json({ ok: false, error: 'Mission start requires a committed waypoint set' });
    }

    const dispatched = await dispatchCommand(CMD.AUTO, { syncMission: false, source: 'mission.start', waitForAck: true });
    if (!dispatched.ok) {
      return res.status(dispatched.status ?? 500).json({ ok: false, error: dispatched.error });
    }

    const m = mission.start();
    publishSupervision();
    return res.json({ ok: true, mission: m });
  } catch (err) {
    return res.status(400).json({ ok: false, error: err.message });
  }
});

// POST /api/mission/pause  — transitions RUNNING → PAUSED
app.post(API.MISSION_PAUSE, requireApp, async (req, res) => {
  const reason = req.body?.reason ?? null;
  try {
    if (currentMissionState() !== MISSION_STATE.RUNNING) {
      return res.status(409).json({ ok: false, error: 'Mission must be RUNNING before pause' });
    }
    const dispatched = await dispatchCommand(CMD.PAUSE, { syncMission: false, source: 'mission.pause', waitForAck: true });
    if (!dispatched.ok) {
      return res.status(dispatched.status ?? 500).json({ ok: false, error: dispatched.error });
    }
    const m = mission.pause(reason);
    publishSupervision();
    return res.json({ ok: true, mission: m });
  } catch (err) {
    return res.status(400).json({ ok: false, error: err.message });
  }
});

// POST /api/mission/resume  — transitions PAUSED → RUNNING
app.post(API.MISSION_RESUME, requireApp, async (_req, res) => {
  try {
    if (currentMissionState() !== MISSION_STATE.PAUSED) {
      return res.status(409).json({ ok: false, error: 'Mission must be PAUSED before resume' });
    }
    if (bridge.getStatus().wpPushState !== 'committed') {
      return res.status(409).json({ ok: false, error: 'Mission resume requires committed waypoints' });
    }
    const dispatched = await dispatchCommand(CMD.AUTO, { syncMission: false, source: 'mission.resume', waitForAck: true });
    if (!dispatched.ok) {
      return res.status(dispatched.status ?? 500).json({ ok: false, error: dispatched.error });
    }
    const m = mission.resume();
    publishSupervision();
    return res.json({ ok: true, mission: m });
  } catch (err) {
    return res.status(400).json({ ok: false, error: err.message });
  }
});

// POST /api/mission/complete  — transitions RUNNING → COMPLETED  (operator-declared)
app.post(API.MISSION_COMPLETE, requireApp, async (_req, res) => {
  try {
    if (currentMissionState() !== MISSION_STATE.RUNNING) {
      return res.status(409).json({ ok: false, error: 'Mission must be RUNNING before complete' });
    }
    const dispatched = await dispatchCommand(CMD.PAUSE, { syncMission: false, source: 'mission.complete', waitForAck: true });
    if (!dispatched.ok) {
      return res.status(dispatched.status ?? 500).json({ ok: false, error: dispatched.error });
    }
    const stats = state.coverage ? coverageStats(state.coverage) : {};
    const m = mission.complete({
      coveragePct: getCoveragePct(stats),
      faultCount:  mission.publicMission()?.faultCount ?? 0,
      cmdCount:    mission.publicMission()?.cmdCount   ?? 0,
    });
    bridge.resetWpState();
    publishSupervision();
    return res.json({ ok: true, mission: m });
  } catch (err) {
    return res.status(400).json({ ok: false, error: err.message });
  }
});

// POST /api/mission/abort  — transitions RUNNING|PAUSED → ABORTED
app.post(API.MISSION_ABORT, requireApp, async (req, res) => {
  const reason = req.body?.reason ?? 'operator';
  try {
    if (![MISSION_STATE.RUNNING, MISSION_STATE.PAUSED].includes(currentMissionState())) {
      return res.status(409).json({ ok: false, error: 'Mission must be RUNNING or PAUSED before abort' });
    }
    const dispatched = await dispatchCommand(CMD.ESTOP, { syncMission: false, source: 'mission.abort', waitForAck: true });
    if (!dispatched.ok) {
      return res.status(dispatched.status ?? 500).json({ ok: false, error: dispatched.error });
    }
    const stats = state.coverage ? coverageStats(state.coverage) : {};
    const m = mission.abort(reason, {
      coveragePct: getCoveragePct(stats),
      faultCount:  mission.publicMission()?.faultCount ?? 0,
      cmdCount:    mission.publicMission()?.cmdCount   ?? 0,
    });
    bridge.resetWpState();
    publishSupervision();
    return res.json({ ok: true, mission: m });
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
app.get(API.LORA_STATUS, requireAppOrBoard, (_req, res) => {
  res.json({ ok: true, lora: bridge.getStatus() });
});

app.get(API.BRIDGE_SYNC, requireAppOrBoard, (_req, res) => {
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
    lora: bridge.getStatus(),
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
app.post(API.LORA_PUSH_WP, requireApp, async (req, res) => {
  if (![MISSION_STATE.CONFIGURING, MISSION_STATE.PAUSED].includes(currentMissionState())) {
    return res.status(409).json({ ok: false, error: 'Waypoint push allowed only while mission is CONFIGURING or PAUSED' });
  }

  let rawPoints = req.body?.points;
  if (!rawPoints && state.lastPath && state.lastPath.length > 0) {
    rawPoints = state.lastPath.map((p) => ({
      lat:   p.lat,
      lon:   p.lon,
      salt:  p.salt  ?? 100,
      brine: p.brine ?? 100,
    }));
  }
  if (!rawPoints || !Array.isArray(rawPoints) || rawPoints.length === 0) {
    return res.status(400).json({ ok: false, error: 'No points provided and no cached path plan' });
  }
  if (pathHasZeroDispersion(rawPoints)) {
    return res.status(409).json({ ok: false, error: 'Waypoint push blocked: path has 0% salt and 0% brine' });
  }
  const result = await bridge.pushWaypoints(rawPoints);
  if (!result.ok) {
    return res.status(502).json({ ok: false, error: result.error, sent: result.sent });
  }
  db.logEvent(mission.currentId(), EVENT_TYPE.PATH_PLANNED, { wpPushCount: result.sent, source: 'lora_bridge' });
  publishSupervision();
  publishOperator();
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
