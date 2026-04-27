/**
 * db.js
 *
 * Manages two tables:
 *   missions   — one row per mission lifecycle (start → end)
 *   events     — append-only log of every significant event, keyed to a mission
 *
 * All writes are synchronous (better-sqlite3) so callers never need to await.
 * The database file is created at DATA_DIR/robot_missions.db (auto-created).
 */

'use strict';

const path = require('path');
const fs   = require('fs');
const Database = require('better-sqlite3');
const { EVENT_TYPE } = require('./contracts');

// ---------------------------------------------------------------------------
// Database location
// ---------------------------------------------------------------------------
const DATA_DIR = process.env.ROBOT_LORA_DATA_DIR
  ? path.resolve(process.env.ROBOT_LORA_DATA_DIR)
  : path.resolve(__dirname, '..', 'data');
if (!fs.existsSync(DATA_DIR)) fs.mkdirSync(DATA_DIR, { recursive: true });

const DB_PATH = path.join(DATA_DIR, 'robot_missions.db');
const db = new Database(DB_PATH);

// Enable WAL mode for better concurrent read performance
db.pragma('journal_mode = WAL');
db.pragma('foreign_keys = ON');
// Missions store some structured fields as JSON text. That keeps the schema
// simple while still letting higher layers persist the full boundary payload.

// ---------------------------------------------------------------------------
// Schema
// ---------------------------------------------------------------------------
db.exec(`
  CREATE TABLE IF NOT EXISTS missions (
    id            INTEGER PRIMARY KEY AUTOINCREMENT,
    state         TEXT    NOT NULL DEFAULT 'IDLE',
    started_at    INTEGER,          -- unix ms
    ended_at      INTEGER,          -- unix ms, NULL while active
    base_station  TEXT,             -- JSON {lat,lon}
    boundary      TEXT,             -- JSON array of {lat,lon}
    cell_size_m   REAL    DEFAULT 2.0,
    coverage_pct  REAL    DEFAULT 0,
    fault_count   INTEGER DEFAULT 0,
    cmd_count     INTEGER DEFAULT 0,
    notes         TEXT              -- free-form operator notes
  );

  CREATE TABLE IF NOT EXISTS events (
    id            INTEGER PRIMARY KEY AUTOINCREMENT,
    mission_id    INTEGER REFERENCES missions(id) ON DELETE CASCADE,
    type          TEXT    NOT NULL,  -- EVENT_TYPE value
    at            INTEGER NOT NULL,  -- unix ms
    payload       TEXT             -- JSON blob, optional
  );

  CREATE INDEX IF NOT EXISTS idx_events_mission ON events(mission_id);
  CREATE INDEX IF NOT EXISTS idx_events_type    ON events(type);
  CREATE INDEX IF NOT EXISTS idx_events_at      ON events(at);
`);

// ---------------------------------------------------------------------------
// Prepared statements
// ---------------------------------------------------------------------------
const stmts = {
  insertMission: db.prepare(`
    INSERT INTO missions (state, started_at, base_station, boundary, cell_size_m)
    VALUES (@state, @started_at, @base_station, @boundary, @cell_size_m)
  `),

  updateMissionState: db.prepare(`
    UPDATE missions SET state = @state WHERE id = @id
  `),

  updateMissionEnd: db.prepare(`
    UPDATE missions
    SET state = @state, ended_at = @ended_at,
        coverage_pct = @coverage_pct, fault_count = @fault_count, cmd_count = @cmd_count
    WHERE id = @id
  `),

  updateMissionCoverage: db.prepare(`
    UPDATE missions SET coverage_pct = @coverage_pct WHERE id = @id
  `),

  updateMissionNotes: db.prepare(`
    UPDATE missions SET notes = @notes WHERE id = @id
  `),

  incrementFaults: db.prepare(`
    UPDATE missions SET fault_count = fault_count + 1 WHERE id = @id
  `),

  incrementCmds: db.prepare(`
    UPDATE missions SET cmd_count = cmd_count + 1 WHERE id = @id
  `),

  getMission: db.prepare(`
    SELECT * FROM missions WHERE id = ?
  `),

  getActiveMission: db.prepare(`
    SELECT * FROM missions
    WHERE state NOT IN ('IDLE','COMPLETED','ABORTED','ERROR')
    ORDER BY id DESC LIMIT 1
  `),

  listMissions: db.prepare(`
    SELECT id, state, started_at, ended_at, coverage_pct, fault_count, cmd_count
    FROM missions ORDER BY id DESC LIMIT ?
  `),

  insertEvent: db.prepare(`
    INSERT INTO events (mission_id, type, at, payload)
    VALUES (@mission_id, @type, @at, @payload)
  `),

  getEvents: db.prepare(`
    SELECT * FROM events WHERE mission_id = ? ORDER BY at ASC
  `),

  getEventsByType: db.prepare(`
    SELECT * FROM events WHERE mission_id = ? AND type = ? ORDER BY at ASC
  `),

  countEvents: db.prepare(`
    SELECT COUNT(*) as n FROM events WHERE mission_id = ?
  `),

  deleteEventsBefore: db.prepare(`
    DELETE FROM events WHERE at < @cutoff_ms
  `),

  ping: db.prepare(`
    SELECT 1 as ok
  `),
};

// Prepared statements are defined once so hot paths such as telemetry updates
// and event logging avoid reparsing SQL on every call.

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/**
 * Create a new mission row and return its id.
 * @param {{ baseStation: object, boundary: object[], cellSizeM: number }} config
 * @returns {number} mission id
 */
function createMission({ baseStation, boundary, cellSizeM = 2.0 } = {}) {
  const result = stmts.insertMission.run({
    state:       'CONFIGURING',
    started_at:  Date.now(),
    base_station: JSON.stringify(baseStation ?? null),
    boundary:    JSON.stringify(boundary ?? []),
    cell_size_m: cellSizeM,
  });
  return result.lastInsertRowid;
}

/**
 * Transition mission state (no guard — caller enforces MISSION_STATE_TRANSITIONS).
 * @param {number} id
 * @param {string} newState
 */
function setMissionState(id, newState) {
  stmts.updateMissionState.run({ id, state: newState });
}

/**
 * Close a mission (COMPLETED or ABORTED) with final stats.
 * @param {number} id
 * @param {string} finalState
 * @param {{ coveragePct: number, faultCount: number, cmdCount: number }} stats
 */
function closeMission(id, finalState, { coveragePct = 0, faultCount = 0, cmdCount = 0 } = {}) {
  stmts.updateMissionEnd.run({
    id,
    state:        finalState,
    ended_at:     Date.now(),
    coverage_pct: coveragePct,
    fault_count:  faultCount,
    cmd_count:    cmdCount,
  });
}

/** Update rolling coverage percentage while a mission is active. */
function updateCoverage(id, coveragePct) {
  stmts.updateMissionCoverage.run({ id, coverage_pct: coveragePct });
}

/** Replace the free-form mission notes text. */
function updateMissionNotes(id, notes) {
  stmts.updateMissionNotes.run({ id, notes });
}

/** Increment fault counter on the active mission row. */
function incrementFaults(id) {
  stmts.incrementFaults.run({ id });
}

/** Increment command counter on the active mission row. */
function incrementCmds(id) {
  stmts.incrementCmds.run({ id });
}

/**
 * Append an event to the log.
 * @param {number|null}  missionId   null for pre-mission server events
 * @param {string}       type        EVENT_TYPE value
 * @param {object|null}  payload     arbitrary JSON-serialisable data
 */
function logEvent(missionId, type, payload = null) {
  stmts.insertEvent.run({
    mission_id: missionId ?? null,
    type,
    at:      Date.now(),
    payload: payload != null ? JSON.stringify(payload) : null,
  });
}

/** Return a single mission row (with parsed JSON fields). */
function getMission(id) {
  const row = stmts.getMission.get(id);
  return row ? _parseMissionRow(row) : null;
}

/**
 * Return the currently active mission, or null.
 * Active = state is CONFIGURING, RUNNING, or PAUSED.
 */
function getActiveMission() {
  const row = stmts.getActiveMission.get();
  return row ? _parseMissionRow(row) : null;
}

/** Return the N most recent missions (summary columns only). */
function listMissions(limit = 50) {
  return stmts.listMissions.all(limit);
}

/** Return all events for a mission, with payloads parsed. */
function getMissionEvents(missionId) {
  return stmts.getEvents.all(missionId).map(_parseEventRow);
}

/** Return events of a specific type for a mission. */
function getMissionEventsByType(missionId, type) {
  return stmts.getEventsByType.all(missionId, type).map(_parseEventRow);
}

/** Total event count for a mission. */
function countMissionEvents(missionId) {
  return stmts.countEvents.get(missionId)?.n ?? 0;
}

/** Prune old events before cutoff unix-ms timestamp. Returns deleted row count. */
function pruneEventsBefore(cutoffMs) {
  if (!Number.isFinite(cutoffMs)) return 0;
  const result = stmts.deleteEventsBefore.run({ cutoff_ms: cutoffMs });
  return Number(result?.changes ?? 0);
}

/** Close the database (for clean shutdown / tests). */
function close() {
  db.close();
}

/** Lightweight DB liveness check for health/readiness probes. */
function ping() {
  try {
    const row = stmts.ping.get();
    return row?.ok === 1;
  } catch (_) {
    return false;
  }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------
function _parseMissionRow(row) {
  // Rows are normalized at the DB boundary so the rest of the app can work
  // with plain JS objects instead of raw SQLite JSON strings.
  return {
    ...row,
    base_station: row.base_station ? JSON.parse(row.base_station) : null,
    boundary:     row.boundary     ? JSON.parse(row.boundary)     : [],
  };
}

function _parseEventRow(row) {
  return {
    ...row,
    payload: row.payload ? JSON.parse(row.payload) : null,
  };
}

// ---------------------------------------------------------------------------
// Log server start at module load
// ---------------------------------------------------------------------------
logEvent(null, EVENT_TYPE.SERVER_STARTED, { pid: process.pid, dbPath: DB_PATH });

module.exports = {
  createMission,
  setMissionState,
  closeMission,
  updateCoverage,
  updateMissionNotes,
  incrementFaults,
  incrementCmds,
  logEvent,
  getMission,
  getActiveMission,
  listMissions,
  getMissionEvents,
  getMissionEventsByType,
  countMissionEvents,
  pruneEventsBefore,
  ping,
  close,
};
