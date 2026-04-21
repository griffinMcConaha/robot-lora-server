#!/usr/bin/env node
'use strict';

/**
 * sqlite-drill.js
 *
 * Simple backup/restore validation for the local mission database. It forces a
 * checkpoint, creates a backup, copies it to a restore candidate, and compares
 * row counts to catch obviously broken backup procedures.
 */

const fs = require('node:fs');
const path = require('node:path');
const Database = require('better-sqlite3');

const DATA_DIR = process.env.ROBOT_LORA_DATA_DIR
  ? path.resolve(process.env.ROBOT_LORA_DATA_DIR)
  : path.resolve(__dirname, '..', 'data');

const DB_FILE = process.env.SQLITE_DB_FILE || 'robot_missions.db';
const DB_PATH = path.join(DATA_DIR, DB_FILE);

const BACKUP_DIR = process.env.SQLITE_BACKUP_DIR
  ? path.resolve(process.env.SQLITE_BACKUP_DIR)
  : path.join(DATA_DIR, 'backups');

function timestampTag() {
  return new Date().toISOString().replace(/[:.]/g, '-');
}

function ensurePathExists(filePath) {
  if (!fs.existsSync(filePath)) {
    throw new Error(`File not found: ${filePath}`);
  }
}

function countRows(databasePath, tableName) {
  const db = new Database(databasePath, { readonly: true });
  try {
    const row = db.prepare(`SELECT COUNT(*) AS count FROM ${tableName}`).get();
    return Number(row?.count ?? 0);
  } finally {
    db.close();
  }
}

async function main() {
  ensurePathExists(DB_PATH);
  fs.mkdirSync(BACKUP_DIR, { recursive: true });

  const runId = timestampTag();
  const backupPath = path.join(BACKUP_DIR, `${DB_FILE}.${runId}.bak`);
  const restorePath = path.join(BACKUP_DIR, `${DB_FILE}.${runId}.restore.db`);

  const liveDb = new Database(DB_PATH);
  try {
    // Flush WAL state into the main database file before taking the backup.
    liveDb.pragma('wal_checkpoint(FULL)');
    await liveDb.backup(backupPath);
  } finally {
    liveDb.close();
  }

  fs.copyFileSync(backupPath, restorePath);

  const liveMissions = countRows(DB_PATH, 'missions');
  const liveEvents = countRows(DB_PATH, 'events');
  const restoredMissions = countRows(restorePath, 'missions');
  const restoredEvents = countRows(restorePath, 'events');

  if (liveMissions !== restoredMissions || liveEvents !== restoredEvents) {
    throw new Error(`Restore validation mismatch: live(missions=${liveMissions}, events=${liveEvents}) restore(missions=${restoredMissions}, events=${restoredEvents})`);
  }

  console.log('SQLITE DRILL: PASS');
  console.log(JSON.stringify({
    dbPath: DB_PATH,
    backupPath,
    restorePath,
    live: { missions: liveMissions, events: liveEvents },
    restored: { missions: restoredMissions, events: restoredEvents },
  }, null, 2));
}

main().catch((error) => {
  console.error('SQLITE DRILL: FAIL');
  console.error(error.message || error);
  process.exit(1);
});
