#!/usr/bin/env node
'use strict';

const { execSync } = require('node:child_process');

const BASE_URL = process.env.RELEASE_GATE_BASE_URL || process.env.BASE_URL || '';
const APP_API_KEY = process.env.RELEASE_GATE_APP_API_KEY || process.env.APP_API_KEY || '';

function runCommand(commandLine) {
  try {
    execSync(commandLine, {
      stdio: 'inherit',
      env: process.env,
      windowsHide: true,
    });
  } catch {
    throw new Error(`${commandLine} failed`);
  }
}

async function fetchJson(url, headers = {}) {
  const response = await fetch(url, { headers });
  let body = null;
  try {
    body = await response.json();
  } catch {
    body = null;
  }
  return { response, body };
}

async function runEndpointChecks() {
  const headers = APP_API_KEY ? { 'x-api-key': APP_API_KEY } : {};

  const health = await fetchJson(`${BASE_URL}/api/health`, headers);
  if (health.response.status !== 200 || !health.body?.ok) {
    throw new Error(`/api/health check failed (status ${health.response.status})`);
  }

  const metrics = await fetchJson(`${BASE_URL}/api/metrics`, headers);
  if (metrics.response.status !== 200 || !metrics.body?.ok) {
    throw new Error(`/api/metrics check failed (status ${metrics.response.status}). Ensure an app API key is set.`);
  }

  const checks = health.body?.checks ?? {};
  if (!checks.db || !checks.bridge) {
    throw new Error(`/api/health reported failing checks: ${JSON.stringify(checks)}`);
  }

  console.log('Endpoint checks passed:', {
    healthStatus: health.response.status,
    metricsStatus: metrics.response.status,
    checks,
  });
}

async function main() {
  console.log('Running release gate checks...');

  runCommand('node --check src/server.js');
  runCommand('node --check src/lora_bridge.js');
  runCommand('node --check src/coverage.js');
  runCommand('node --check src/mission.js');
  runCommand('npm run test:hil');

  if (BASE_URL) {
    await runEndpointChecks();
  } else {
    console.log('Skipping live endpoint checks (set RELEASE_GATE_BASE_URL and RELEASE_GATE_APP_API_KEY to enable).');
  }

  console.log('RELEASE GATE: PASS');
}

main().catch((error) => {
  console.error('RELEASE GATE: FAIL');
  console.error(error.message || error);
  process.exit(1);
});
