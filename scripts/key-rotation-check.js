#!/usr/bin/env node
'use strict';

/**
 * key-rotation-check.js
 *
 * Verifies the expected overlap/revocation behavior during API-key rotation.
 * This keeps auth cutovers repeatable instead of relying on ad hoc curl checks.
 */

const args = new Set(process.argv.slice(2));
const EXPECT_OLD_REVOKED = args.has('--expect-old-revoked');

const BASE_URL = process.env.ROTATION_BASE_URL || process.env.BASE_URL;
const NEW_APP_KEY = process.env.NEW_APP_KEY;
const OLD_APP_KEY = process.env.OLD_APP_KEY;

if (!BASE_URL) {
  console.error('Missing ROTATION_BASE_URL or BASE_URL');
  process.exit(1);
}

if (!NEW_APP_KEY) {
  console.error('Missing NEW_APP_KEY');
  process.exit(1);
}

async function call(path, key) {
  const headers = key ? { 'x-api-key': key } : {};
  const response = await fetch(`${BASE_URL}${path}`, { headers });
  let body;
  try {
    body = await response.json();
  } catch {
    body = null;
  }
  return { status: response.status, body };
}

function assert(condition, message) {
  if (!condition) {
    throw new Error(message);
  }
}

async function verifyKeyWorks(label, key) {
  // Metrics is a strict auth check, while health also proves the backend is
  // reachable enough to answer real requests after the rotation.
  const metrics = await call('/api/metrics', key);
  assert(metrics.status === 200 && metrics.body?.ok, `${label} key failed /api/metrics auth (status ${metrics.status})`);

  const health = await call('/api/health', key);
  assert(health.status === 200 || health.status === 503, `${label} key failed /api/health reachability (status ${health.status})`);

  return {
    metricsStatus: metrics.status,
    healthStatus: health.status,
    healthReady: Boolean(health.body?.ready),
    authEnabled: Boolean(health.body?.auth?.enabled),
  };
}

async function verifyOldRevoked(label, key) {
  const metrics = await call('/api/metrics', key);
  assert(metrics.status === 401, `${label} key expected revoked but got status ${metrics.status}`);
}

async function main() {
  console.log('Running key rotation validation...');

  const newResult = await verifyKeyWorks('NEW', NEW_APP_KEY);
  console.log('NEW key check:', newResult);

  if (OLD_APP_KEY) {
    if (EXPECT_OLD_REVOKED) {
      await verifyOldRevoked('OLD', OLD_APP_KEY);
      console.log('OLD key revocation check: passed');
    } else {
      const oldResult = await verifyKeyWorks('OLD', OLD_APP_KEY);
      console.log('OLD key overlap check:', oldResult);
    }
  } else {
    console.log('OLD key not provided; skipped old-key validation.');
  }

  console.log('KEY ROTATION CHECK: PASS');
}

main().catch((error) => {
  console.error('KEY ROTATION CHECK: FAIL');
  console.error(error.message || error);
  process.exit(1);
});
