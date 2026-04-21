/**
 * supervision.js
 *
 * Pure decision helpers for the dashboard and app. This module converts raw
 * mission/robot state into operator-facing affordances such as allowed actions,
 * alerts, and workflow steps.
 */
const { MISSION_STATE, FAULT_ACTION } = require('./contracts');

function createOperatorState() {
  return {
    workflowAcks: {},
    notes: [],
  };
}

function telemetryAgeMs(robot, now = Date.now()) {
  if (!robot?.timestampMs) return null;
  return Math.max(0, now - robot.timestampMs);
}

function isTelemetryStale(robot, staleThresholdMs, now = Date.now()) {
  const ageMs = telemetryAgeMs(robot, now);
  return ageMs == null ? true : ageMs > staleThresholdMs;
}

function buildAllowedActions({ missionState, wpPushState, hasCoverage, hasPath, zeroDispersionPath, gpsReady, gpsReason, demoModeEnabled, resetNeeded = false }) {
  // When telemetry is absent we keep the gate open so operators can still
  // configure the mission offline instead of locking the whole UI.
  const gpsGateOpen = gpsReady || gpsReason === 'Robot telemetry is unavailable';

  return [
    { id: 'input-area', enabled: true, reason: null },
    { id: 'path-plan', enabled: hasCoverage, reason: hasCoverage ? null : 'Area is not configured' },
    { id: 'demo-mode-toggle', enabled: true, reason: null },
    {
      id: 'push-waypoints',
      enabled: [MISSION_STATE.CONFIGURING, MISSION_STATE.PAUSED].includes(missionState) && hasPath && !zeroDispersionPath && !demoModeEnabled,
      reason: demoModeEnabled
        ? 'Demo mode is on. Hardware waypoint push is locked.'
        : !hasPath
        ? 'Path plan is required before waypoint push'
        : zeroDispersionPath
          ? 'Zero-dispersion path cannot be committed'
          : null,
    },
    {
      id: 'mission-start',
      enabled: missionState === MISSION_STATE.CONFIGURING && (wpPushState === 'committed' || hasPath) && !zeroDispersionPath && gpsGateOpen && !demoModeEnabled,
      reason: demoModeEnabled
        ? 'Demo mode is on. Live mission start is locked.'
        : missionState !== MISSION_STATE.CONFIGURING
        ? 'Mission must be CONFIGURING'
        : !gpsGateOpen
          ? (gpsReason ?? 'Robot GPS readiness is required before mission start')
        : zeroDispersionPath
          ? 'Zero-dispersion path cannot be started'
          : null,
    },
    { id: 'mission-pause', enabled: missionState === MISSION_STATE.RUNNING, reason: missionState === MISSION_STATE.RUNNING ? null : 'Mission is not RUNNING' },
    {
      id: 'mission-resume',
      enabled: missionState === MISSION_STATE.PAUSED && wpPushState === 'committed' && gpsGateOpen && !demoModeEnabled,
      reason: demoModeEnabled
        ? 'Demo mode is on. Live mission resume is locked.'
        : missionState !== MISSION_STATE.PAUSED
        ? 'Mission is not PAUSED'
        : !gpsGateOpen
          ? (gpsReason ?? 'Robot GPS readiness is required before mission resume')
          : null,
    },
    { id: 'mission-abort', enabled: [MISSION_STATE.RUNNING, MISSION_STATE.PAUSED].includes(missionState), reason: null },
    { id: 'mission-complete', enabled: missionState === MISSION_STATE.RUNNING, reason: missionState === MISSION_STATE.RUNNING ? null : 'Mission is not RUNNING' },
    {
      id: 'command-manual',
      enabled: true,
      reason: null,
    },
    {
      id: 'command-reset',
      enabled: resetNeeded || [MISSION_STATE.PAUSED, MISSION_STATE.ABORTED, MISSION_STATE.ERROR].includes(missionState),
      reason: null,
    },
  ];
}

function buildAlerts({
  missionState,
  wpPushState,
  hasCoverage,
  hasPath,
  zeroDispersionPath,
  gpsReady,
  gpsReason,
  demoModeEnabled,
  telemetryStale,
  telemetryAge,
  lastFault,
  safety,
  recovery,
  now = Date.now(),
}) {
  const alerts = [];

  // Order matters here: setup warnings first, then live safety issues. The UI
  // displays the array directly, so this keeps the operator narrative coherent.
  if (!hasCoverage) {
    alerts.push({ level: 'warning', code: 'AREA_UNCONFIGURED', message: 'Input area has not been configured.' });
  }
  if (missionState === MISSION_STATE.CONFIGURING && !hasPath) {
    alerts.push({ level: 'warning', code: 'PATH_MISSING', message: 'Mission has no planned path yet.' });
  }
  if ([MISSION_STATE.CONFIGURING, MISSION_STATE.PAUSED].includes(missionState) && wpPushState !== 'committed') {
    alerts.push({ level: 'warning', code: 'WAYPOINTS_UNCOMMITTED', message: 'Waypoints are not committed to the robot.' });
  }
  if (zeroDispersionPath) {
    alerts.push({
      level: 'warning',
      code: 'ZERO_DISPERSION_PATH',
      message: 'Current path uses 0% salt and 0% brine. Update dispersion before waypoint push or mission start.',
    });
  }
  if (demoModeEnabled) {
    alerts.push({
      level: 'warning',
      code: 'DEMO_MODE_ACTIVE',
      message: 'Demo mode is active. Autonomy is locked, but manual driving and spot marking stay available.',
    });
  }
  if (recovery?.needed) {
    alerts.push({
      level: 'warning',
      code: 'MISSION_RECOVERY_NEEDED',
      message: recovery.reason ?? 'The active mission needs to be re-synced with the robot before it can continue.',
    });
  }
  if (!gpsReady) {
    alerts.push({
      level: 'warning',
      code: 'GPS_FIX_REQUIRED',
      message: gpsReason ?? 'Robot GPS fix is required before starting or resuming autonomy.',
    });
  }
  if (safety?.gpsFailsafeAt) {
    alerts.push({
      level: 'critical',
      code: 'GPS_FAILSAFE_TRIGGERED',
      message: `GPS fail-safe triggered ${safety.gpsFailsafeAction ?? 'UNKNOWN'} at ${new Date(safety.gpsFailsafeAt).toISOString()}.`,
      at: safety.gpsFailsafeAt,
    });
  }
  if (missionState === MISSION_STATE.RUNNING && telemetryStale) {
    alerts.push({
      level: 'critical',
      code: 'TELEMETRY_STALE',
      message: telemetryAge == null
        ? 'No telemetry received while mission is RUNNING.'
        : `Telemetry is stale (${telemetryAge} ms since last update).`,
    });
  }
  if (lastFault) {
    alerts.push({
      level: lastFault.action === FAULT_ACTION.ESTOP ? 'critical' : 'warning',
      code: 'LAST_FAULT',
      message: `Last fault ${lastFault.fault} requested ${lastFault.action}.`,
      at: lastFault.at,
    });
  }
  if (safety?.telemetryFailsafeAt) {
    alerts.push({
      level: 'critical',
      code: 'TELEMETRY_FAILSAFE_TRIGGERED',
      message: `Telemetry fail-safe triggered ${safety.telemetryFailsafeAction ?? 'UNKNOWN'} at ${new Date(safety.telemetryFailsafeAt).toISOString()}.`,
      at: safety.telemetryFailsafeAt,
    });
  }
  if (safety?.geofenceFailsafeAt) {
    alerts.push({
      level: 'critical',
      code: 'GEOFENCE_FAILSAFE_TRIGGERED',
      message: `Geofence fail-safe triggered ${safety.geofenceFailsafeAction ?? 'UNKNOWN'} at ${new Date(safety.geofenceFailsafeAt).toISOString()}.`,
      at: safety.geofenceFailsafeAt,
    });
  }

  return alerts;
}

function buildOperatorWorkflows({ missionState, wpPushState, hasArea, hasPath, latestNote, lastFault, getWorkflowAck }) {
  // These workflows are intentionally derived from current state rather than
  // persisted as a separate machine, which keeps them resilient to restarts.
  const workflows = [
    {
      id: 'preflight',
      title: 'Preflight',
      active: [MISSION_STATE.IDLE, MISSION_STATE.CONFIGURING].includes(missionState),
      steps: [
        { id: 'area-configured', title: 'Area configured', kind: 'derived', checked: hasArea, ready: true },
        { id: 'path-planned', title: 'Path planned', kind: 'derived', checked: hasPath, ready: hasArea },
        { id: 'waypoints-committed', title: 'Waypoints committed', kind: 'derived', checked: wpPushState === 'committed', ready: hasPath },
        {
          id: 'radio-check',
          title: 'Radio check completed',
          kind: 'manual',
          checked: Boolean(getWorkflowAck('preflight', 'radio-check')?.checked),
          ready: hasArea,
          ack: getWorkflowAck('preflight', 'radio-check'),
        },
        {
          id: 'area-clear',
          title: 'Area clearance confirmed',
          kind: 'manual',
          checked: Boolean(getWorkflowAck('preflight', 'area-clear')?.checked),
          ready: hasArea,
          ack: getWorkflowAck('preflight', 'area-clear'),
        },
      ],
    },
    {
      id: 'recovery',
      title: 'Recovery',
      active: [MISSION_STATE.PAUSED, MISSION_STATE.ABORTED, MISSION_STATE.ERROR].includes(missionState) || Boolean(lastFault),
      steps: [
        {
          id: 'fault-reviewed',
          title: 'Fault review completed',
          kind: 'manual',
          checked: Boolean(getWorkflowAck('recovery', 'fault-reviewed')?.checked),
          ready: Boolean(lastFault),
          ack: getWorkflowAck('recovery', 'fault-reviewed'),
        },
        {
          id: 'field-clear',
          title: 'Field clearance confirmed',
          kind: 'manual',
          checked: Boolean(getWorkflowAck('recovery', 'field-clear')?.checked),
          ready: [MISSION_STATE.PAUSED, MISSION_STATE.ABORTED, MISSION_STATE.ERROR].includes(missionState),
          ack: getWorkflowAck('recovery', 'field-clear'),
        },
        {
          id: 'reset-eligible',
          title: 'Reset eligibility confirmed',
          kind: 'derived',
          checked: [MISSION_STATE.PAUSED, MISSION_STATE.ABORTED, MISSION_STATE.ERROR].includes(missionState),
          ready: true,
        },
      ],
    },
    {
      id: 'shutdown',
      title: 'Shutdown',
      active: [MISSION_STATE.COMPLETED, MISSION_STATE.ABORTED, MISSION_STATE.ERROR, MISSION_STATE.IDLE].includes(missionState),
      steps: [
        {
          id: 'mission-safe',
          title: 'Mission in safe terminal state',
          kind: 'derived',
          checked: [MISSION_STATE.COMPLETED, MISSION_STATE.ABORTED, MISSION_STATE.ERROR, MISSION_STATE.IDLE].includes(missionState),
          ready: true,
        },
        {
          id: 'note-entered',
          title: 'Shutdown note recorded',
          kind: 'derived',
          checked: Boolean(latestNote),
          ready: true,
        },
      ],
    },
  ];

  return workflows.map((workflow) => ({
    ...workflow,
    complete: workflow.steps.every((step) => step.checked),
  }));
}

module.exports = {
  createOperatorState,
  telemetryAgeMs,
  isTelemetryStale,
  buildAllowedActions,
  buildAlerts,
  buildOperatorWorkflows,
};
