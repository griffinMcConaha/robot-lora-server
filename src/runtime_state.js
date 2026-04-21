'use strict';

/**
 * runtime_state.js
 *
 * Minimal persistence wrapper for the server's runtime snapshot. Higher-level
 * code decides what goes into the snapshot; this module only manages where it
 * lives on disk and how it is loaded/saved safely.
 */

const fs = require('fs');
const path = require('path');

const DATA_DIR = process.env.ROBOT_LORA_DATA_DIR
  ? path.resolve(process.env.ROBOT_LORA_DATA_DIR)
  : path.resolve(__dirname, '..', 'data');

if (!fs.existsSync(DATA_DIR)) {
  fs.mkdirSync(DATA_DIR, { recursive: true });
}

const RUNTIME_STATE_PATH = path.join(DATA_DIR, 'runtime_state.json');

function loadRuntimeState() {
  // Missing or empty files are normal during first boot or after cleanup.
  if (!fs.existsSync(RUNTIME_STATE_PATH)) {
    return null;
  }
  const raw = fs.readFileSync(RUNTIME_STATE_PATH, 'utf8');
  if (!raw.trim()) {
    return null;
  }
  return JSON.parse(raw);
}

function saveRuntimeState(snapshot) {
  // Write to a temp file first so readers never see a half-written snapshot.
  const tempPath = `${RUNTIME_STATE_PATH}.tmp`;
  fs.writeFileSync(tempPath, `${JSON.stringify(snapshot, null, 2)}\n`, 'utf8');
  fs.renameSync(tempPath, RUNTIME_STATE_PATH);
  return RUNTIME_STATE_PATH;
}

module.exports = {
  DATA_DIR,
  RUNTIME_STATE_PATH,
  loadRuntimeState,
  saveRuntimeState,
};
