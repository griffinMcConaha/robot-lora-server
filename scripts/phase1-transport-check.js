#!/usr/bin/env node
'use strict';

/**
 * phase1-transport-check.js
 *
 * Manual field diagnostic for validating the base-station HTTP bridge and the
 * LoRa acknowledgement loop outside the full server runtime.
 */

const http = require('http');
const https = require('https');

const bridge = require('../src/lora_bridge');

const BASE_STATION_URL = (process.env.BASE_STATION_URL ?? 'http://192.168.4.1').replace(/\/$/, '');
const REQUEST_TIMEOUT_MS = Number(process.env.LORA_REQUEST_TIMEOUT_MS ?? 3000);

const args = new Set(process.argv.slice(2));
const includeAuto = args.has('--include-auto');
const includeEstop = args.has('--include-estop');

function request(method, path, body = null) {
  return new Promise((resolve) => {
    const url = new URL(`${BASE_STATION_URL}${path}`);
    const lib = url.protocol === 'https:' ? https : http;
    const payload = body == null ? null : Buffer.from(body, 'utf8');
    const req = lib.request({
      hostname: url.hostname,
      port: url.port || (url.protocol === 'https:' ? 443 : 80),
      path: url.pathname + url.search,
      method,
      headers: payload ? {
        'Content-Type': 'text/plain',
        'Content-Length': payload.length,
      } : undefined,
    }, (res) => {
      const chunks = [];
      res.on('data', (chunk) => chunks.push(chunk));
      res.on('end', () => {
        resolve({
          ok: res.statusCode >= 200 && res.statusCode < 300,
          status: res.statusCode,
          body: Buffer.concat(chunks).toString('utf8'),
        });
      });
    });

    req.setTimeout(REQUEST_TIMEOUT_MS, () => {
      req.destroy(new Error(`timeout after ${REQUEST_TIMEOUT_MS}ms`));
    });

    req.on('error', (error) => {
      resolve({ ok: false, status: null, body: '', error: error.message });
    });

    if (payload) req.write(payload);
    req.end();
  });
}

async function getStatus() {
  const result = await request('GET', '/status');
  if (!result.ok) return result;
  try {
    return { ...result, json: JSON.parse(result.body) };
  } catch (error) {
    return { ok: false, status: result.status, body: result.body, error: `invalid status json: ${error.message}` };
  }
}

async function getLastLoRa() {
  return request('GET', '/last_lora');
}

function printSection(title) {
  process.stdout.write(`\n== ${title} ==\n`);
}

function printResult(name, result) {
  const label = result.ok ? 'PASS' : 'FAIL';
  process.stdout.write(`${label} ${name}`);
  if (result.detail) process.stdout.write(`: ${result.detail}`);
  process.stdout.write('\n');
}

async function run() {
  const checks = [];

  printSection('Base Station Reachability');
  const status = await getStatus();
  checks.push({
    name: 'GET /status',
    ok: Boolean(status.ok && status.json),
    detail: status.ok ? `mode=${status.json.mode ?? 'unknown'} queue_depth=${status.json.queue_depth ?? 'n/a'}` : (status.error ?? status.body ?? 'request failed'),
  });

  const lastLoRa = await getLastLoRa();
  checks.push({
    name: 'GET /last_lora',
    ok: Boolean(lastLoRa.ok),
    detail: lastLoRa.ok ? (lastLoRa.body.trim() || '<empty>') : (lastLoRa.error ?? lastLoRa.body ?? 'request failed'),
  });

  checks.slice(-2).forEach((check) => printResult(check.name, check));

  printSection('Waypoint Transport');
  const waypointProbe = [{
    // A single waypoint is enough to exercise WPCLEAR/WPB/WPLOAD end-to-end.
    lat: Number(process.env.PHASE1_TEST_LAT ?? 41.706392),
    lon: Number(process.env.PHASE1_TEST_LON ?? -81.514196),
    salt: Number(process.env.PHASE1_TEST_SALT ?? 10),
    brine: Number(process.env.PHASE1_TEST_BRINE ?? 20),
  }];
  const push = await bridge.pushWaypoints(waypointProbe);
  checks.push({
    name: 'WPCLEAR/WP/WPLOAD with ACK verification',
    ok: Boolean(push.ok),
    detail: push.ok ? `sent=${push.sent}` : push.error,
  });
  printResult(checks.at(-1).name, checks.at(-1));

  printSection('Command ACKs');
  const pause = await bridge.sendCommand('PAUSE', { waitForAck: true });
  checks.push({
    name: 'PAUSE command ACK',
    ok: Boolean(pause.ok),
    detail: pause.ok ? pause.ack?.ack ?? 'ack received' : pause.error,
  });
  printResult(checks.at(-1).name, checks.at(-1));

  if (includeAuto) {
    const auto = await bridge.sendCommand('AUTO', { waitForAck: true });
    checks.push({
      name: 'AUTO command ACK',
      ok: Boolean(auto.ok),
      detail: auto.ok ? auto.ack?.ack ?? 'ack received' : auto.error,
    });
    printResult(checks.at(-1).name, checks.at(-1));
  }

  if (includeEstop) {
    const estop = await bridge.sendCommand('ESTOP', { waitForAck: true });
    checks.push({
      name: 'ESTOP command ACK',
      ok: Boolean(estop.ok),
      detail: estop.ok ? estop.ack?.ack ?? 'ack received' : estop.error,
    });
    printResult(checks.at(-1).name, checks.at(-1));
  }

  printSection('Summary');
  const failed = checks.filter((check) => !check.ok);
  checks.forEach((check) => printResult(check.name, check));

  if (failed.length > 0) {
    process.exitCode = 1;
    process.stdout.write(`\nPhase 1 is not complete. ${failed.length} check(s) failed.\n`);
    return;
  }

  process.stdout.write('\nPhase 1 transport checks passed.\n');
}

run().catch((error) => {
  process.stderr.write(`${error.stack || error.message}\n`);
  process.exitCode = 1;
});
