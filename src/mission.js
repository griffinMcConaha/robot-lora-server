/**
 * mission.js
 *
 * Owns the authoritative in-memory mission record and exposes clean
 * transition methods.  All transitions are validated against
 * MISSION_STATE_TRANSITIONS before executing; invalid transitions throw.
 * Every transition is persisted to the DB and an optional callback is
 * invoked so server.js can publish WS events without a circular dep.
 */

'use strict';

const {
  MISSION_STATE,
  MISSION_STATE_TRANSITIONS,
  EVENT_TYPE,
} = require('./contracts');

const db = require('./db');

// ---------------------------------------------------------------------------
// In-memory mission record (mirrors the active DB row)
// ---------------------------------------------------------------------------
let _mission = null;   // null  → no active mission
let _onTransition = null; // (mission) => void  — injected by server.js

/**
 * Register a callback invoked after every successful state transition.
 * @param {(mission: object) => void} fn
 */
function onTransition(fn) {
  _onTransition = fn;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
function _assertTransition(from, to) {
  // Keep transition policy centralized here so callers cannot "accidentally"
  // create new mission flows by writing directly to the mission object.
  const allowed = MISSION_STATE_TRANSITIONS[from];
  if (!allowed || !allowed.has(to)) {
    throw new Error(`Invalid mission transition: ${from} → ${to}`);
  }
}

function _notify() {
  if (_onTransition && _mission) _onTransition(publicMission());
}

// ---------------------------------------------------------------------------
// Query
// ---------------------------------------------------------------------------

/** Current mission public snapshot (safe to send over WS / HTTP). */
function publicMission() {
  if (!_mission) return null;
  return {
    id:          _mission.id,
    state:       _mission.state,
    startedAt:   _mission.startedAt,
    endedAt:     _mission.endedAt ?? null,
    coveragePct: _mission.coveragePct,
    faultCount:  _mission.faultCount,
    cmdCount:    _mission.cmdCount,
    notes:       _mission.notes,
    baseStation: _mission.baseStation ?? null,
    boundary:    _mission.boundary    ?? [],
    cellSizeM:   _mission.cellSizeM,
  };
}

/** True when a mission is actively running navigation. */
function isRunning() {
  return _mission?.state === MISSION_STATE.RUNNING;
}

/** True when any mission is active (not IDLE/COMPLETED/ABORTED/ERROR). */
function isActive() {
  return _mission !== null &&
    ![MISSION_STATE.COMPLETED, MISSION_STATE.ABORTED, MISSION_STATE.ERROR]
      .includes(_mission.state);
}

/** Current mission id, or null. */
function currentId() {
  return _mission?.id ?? null;
}

// ---------------------------------------------------------------------------
// Lifecycle transitions
// ---------------------------------------------------------------------------

/**
 * Configure area and create a mission row.  Transitions IDLE → CONFIGURING.
 * Safe to call even when _mission is null (server boot, post-abort/complete).
 *
 * @param {{ baseStation: object, boundary: object[], cellSizeM: number }} config
 * @returns {object} publicMission snapshot
 */
function configure({ baseStation, boundary, cellSizeM = 2.0 } = {}) {
  // If there's an ended mission hanging around, clear it first
  if (_mission && [MISSION_STATE.COMPLETED, MISSION_STATE.ABORTED, MISSION_STATE.ERROR].includes(_mission.state)) {
    _mission = null;
  }

  const prevState = _mission?.state ?? MISSION_STATE.IDLE;
  _assertTransition(prevState, MISSION_STATE.CONFIGURING);

  // We create the DB row up front so every later event has a stable mission id.
  const id = db.createMission({ baseStation, boundary, cellSizeM });
  _mission = {
    id,
    state:       MISSION_STATE.CONFIGURING,
    startedAt:   Date.now(),
    endedAt:     null,
    coveragePct: 0,
    faultCount:  0,
    cmdCount:    0,
    notes:       '',
    baseStation,
    boundary,
    cellSizeM,
  };

  db.logEvent(id, EVENT_TYPE.AREA_SET, { baseStation, boundaryCount: boundary?.length ?? 0, cellSizeM });
  _notify();
  return publicMission();
}

/**
 * Start the mission.  Transitions CONFIGURING → RUNNING.
 * @returns {object} publicMission snapshot
 */
function start() {
  if (!_mission) throw new Error('No mission configured');
  _assertTransition(_mission.state, MISSION_STATE.RUNNING);

  _mission.state = MISSION_STATE.RUNNING;
  db.setMissionState(_mission.id, MISSION_STATE.RUNNING);
  db.logEvent(_mission.id, EVENT_TYPE.MISSION_STARTED, { at: Date.now() });
  _notify();
  return publicMission();
}

/**
 * Pause the mission.  Transitions RUNNING → PAUSED.
 * @param {string} [reason]   Optional reason string for the event log.
 * @returns {object} publicMission snapshot
 */
function pause(reason) {
  if (!_mission) throw new Error('No active mission');
  _assertTransition(_mission.state, MISSION_STATE.PAUSED);

  _mission.state = MISSION_STATE.PAUSED;
  db.setMissionState(_mission.id, MISSION_STATE.PAUSED);
  db.logEvent(_mission.id, EVENT_TYPE.MISSION_PAUSED, { reason: reason ?? null });
  _notify();
  return publicMission();
}

/**
 * Resume the mission.  Transitions PAUSED → RUNNING.
 * @returns {object} publicMission snapshot
 */
function resume() {
  if (!_mission) throw new Error('No active mission');
  _assertTransition(_mission.state, MISSION_STATE.RUNNING);

  _mission.state = MISSION_STATE.RUNNING;
  db.setMissionState(_mission.id, MISSION_STATE.RUNNING);
  db.logEvent(_mission.id, EVENT_TYPE.MISSION_RESUMED, { at: Date.now() });
  _notify();
  return publicMission();
}

/**
 * Complete the mission successfully.  Transitions RUNNING → COMPLETED.
 * @param {object} finalStats  { coveragePct, faultCount, cmdCount }
 * @returns {object} publicMission snapshot
 */
function complete(finalStats = {}) {
  if (!_mission) throw new Error('No active mission');
  _assertTransition(_mission.state, MISSION_STATE.COMPLETED);

  _applyFinalStats(finalStats);
  _mission.state   = MISSION_STATE.COMPLETED;
  _mission.endedAt = Date.now();

  db.closeMission(_mission.id, MISSION_STATE.COMPLETED, {
    coveragePct: _mission.coveragePct,
    faultCount:  _mission.faultCount,
    cmdCount:    _mission.cmdCount,
  });
  db.logEvent(_mission.id, EVENT_TYPE.MISSION_COMPLETED, {
    coveragePct: _mission.coveragePct,
    faultCount:  _mission.faultCount,
  });
  _notify();
  return publicMission();
}

/**
 * Abort the mission.  Transitions RUNNING|PAUSED → ABORTED.
 * @param {string} [reason]
 * @param {object} [finalStats]
 * @returns {object} publicMission snapshot
 */
function abort(reason, finalStats = {}) {
  if (!_mission) throw new Error('No active mission');
  _assertTransition(_mission.state, MISSION_STATE.ABORTED);

  _applyFinalStats(finalStats);
  _mission.state   = MISSION_STATE.ABORTED;
  _mission.endedAt = Date.now();

  db.closeMission(_mission.id, MISSION_STATE.ABORTED, {
    coveragePct: _mission.coveragePct,
    faultCount:  _mission.faultCount,
    cmdCount:    _mission.cmdCount,
  });
  db.logEvent(_mission.id, EVENT_TYPE.MISSION_ABORTED, {
    reason:      reason ?? null,
    coveragePct: _mission.coveragePct,
    faultCount:  _mission.faultCount,
  });
  _notify();
  return publicMission();
}

/**
 * Reset after a terminal state (COMPLETED/ABORTED/ERROR) → IDLE.
 * Clears the in-memory record so configure() can be called again.
 */
function reset() {
  if (_mission) {
    const prevState = _mission.state;
    _assertTransition(prevState, MISSION_STATE.IDLE);
  }
  // Reset intentionally clears the in-memory pointer instead of mutating the
  // existing object so downstream code stops treating the mission as active.
  _mission = null;
  _notify();
}

// ---------------------------------------------------------------------------
// Incremental updates (called by server on live data)
// ---------------------------------------------------------------------------

/**
 * Update rolling coverage percentage.  Persists to DB row.
 * @param {number} coveragePct  0–100
 */
function updateCoverage(coveragePct) {
  if (!_mission || !isActive()) return;
  _mission.coveragePct = coveragePct;
  db.updateCoverage(_mission.id, coveragePct);
}

/** Record a fault against the active mission. */
function recordFault() {
  if (!_mission || !isActive()) return;
  _mission.faultCount += 1;
  db.incrementFaults(_mission.id);
}

/** Record a command sent during the active mission. */
function recordCommand() {
  if (!_mission || !isActive()) return;
  _mission.cmdCount += 1;
  db.incrementCmds(_mission.id);
}

/** Append a timestamped operator note to the mission. */
function addNote(text) {
  if (!_mission || typeof text !== 'string' || !text.trim()) return publicMission();

  const trimmed = text.trim();
  const stamp = new Date().toISOString();
  const nextNotes = _mission.notes
    ? `${_mission.notes}\n[${stamp}] ${trimmed}`
    : `[${stamp}] ${trimmed}`;

  _mission.notes = nextNotes;
  db.updateMissionNotes(_mission.id, nextNotes);
  _notify();
  return publicMission();
}

// ---------------------------------------------------------------------------
// Internal
// ---------------------------------------------------------------------------
function _applyFinalStats({ coveragePct, faultCount, cmdCount } = {}) {
  // Final stats are merged selectively so callers can supply only the values
  // they actually know at transition time.
  if (coveragePct != null) _mission.coveragePct = coveragePct;
  if (faultCount  != null) _mission.faultCount  = faultCount;
  if (cmdCount    != null) _mission.cmdCount     = cmdCount;
}

// ---------------------------------------------------------------------------
// Rehydrate from DB on startup (pick up any mission left mid-flight)
// ---------------------------------------------------------------------------
(function _rehydrate() {
  const active = db.getActiveMission();
  if (!active) return;

  // A RUNNING mission from a previous server process was interrupted.
  // Transition it to PAUSED automatically — operator must explicitly resume.
  if (active.state === MISSION_STATE.RUNNING) {
    db.setMissionState(active.id, MISSION_STATE.PAUSED);
    db.logEvent(active.id, EVENT_TYPE.MISSION_PAUSED, { reason: 'server_restart' });
    active.state = MISSION_STATE.PAUSED;
  }

  _mission = {
    id:          active.id,
    state:       active.state,
    startedAt:   active.started_at,
    endedAt:     active.ended_at ?? null,
    coveragePct: active.coverage_pct ?? 0,
    faultCount:  active.fault_count  ?? 0,
    cmdCount:    active.cmd_count    ?? 0,
    notes:       active.notes        ?? '',
    baseStation: active.base_station ?? null,
    boundary:    active.boundary     ?? [],
    cellSizeM:   active.cell_size_m  ?? 2.0,
  };

  console.log(`[mission] rehydrated mission #${_mission.id} → ${_mission.state}`);
})();

module.exports = {
  onTransition,
  publicMission,
  isRunning,
  isActive,
  currentId,
  configure,
  start,
  pause,
  resume,
  complete,
  abort,
  reset,
  updateCoverage,
  recordFault,
  recordCommand,
  addNote,
};
