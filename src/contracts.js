/**
 * contracts.js — Phase A + C: frozen message contracts + IDs/sequences
 *
 * Single source of truth for every token, event name, field name, and
 * protocol constant shared across the full robot stack:
 *
 *   STM32 (Core)  ←UART5→  Gateway ESP32  ←LoRa→  Base Station ESP32
 *       ↑ UART4 ↓                                        ↕ HTTP / WiFi
 *   SB ESP32 (dispersion)                         robot-lora-server  ↔  App
 *
 * All values are Object.freeze()'d so accidental mutation throws in strict mode.
 * Cross-repo consumers (firmware, app) should copy the relevant sub-sections or
 * reference this file as the canonical specification.
 */

'use strict';

// ---------------------------------------------------------------------------
// 1. ROBOT STATES  (STM32 state machine, reported in telemetry "state" field)
// ---------------------------------------------------------------------------
/** @readonly */
const ROBOT_STATE = Object.freeze({
  MANUAL:  'MANUAL',   // 0 – operator driving via LoRa commands
  AUTO:    'AUTO',     // 1 – autonomous coverage mode
  PAUSE:   'PAUSE',    // 2 – motion paused, dispersion stopped
  ERROR:   'ERROR',    // 3 – recoverable error, requires RESET
  ESTOP:   'ESTOP',    // 4 – hard stop, latched, requires RESET
  IDLE:    'IDLE',     // server-side default before first telemetry
});

// ---------------------------------------------------------------------------
// 2. COMMANDS  (app/server → base station → LoRa → gateway → STM32)
//
//    Wire format over LoRa:  CMD:<TOKEN>[,SALT:<pct>,BRINE:<pct>]
//    HTTP body (app→base):   { "cmd": "<token>" } or plain-text token
// ---------------------------------------------------------------------------
/** @readonly */
const CMD = Object.freeze({
  FORWARD:  'FORWARD',
  BACKWARD: 'BACKWARD',
  LEFT:     'LEFT',
  RIGHT:    'RIGHT',
  STOP:     'STOP',
  PAUSE:    'PAUSE',
  AUTO:     'AUTO',
  MANUAL:   'MANUAL',
  ESTOP:    'ESTOP',
  RESET:    'RESET',   // clears ESTOP/ERROR latch → transitions to PAUSE
});

/** Aliases accepted on inbound normalisation (not sent by firmware). */
const CMD_ALIASES = Object.freeze({
  FWD:  CMD.FORWARD,
  BACK: CMD.BACKWARD,
});

// ---------------------------------------------------------------------------
// 3. FAULT CODES  (STM32 → LoRa → server, "fault" field)
//
//   Numeric mapping matches firmware FaultCode_t (robot_sm.h):
//     FAULT_NONE=0  IMU_TIMEOUT=1  GPS_LOSS=2  MOTOR_FEEDBACK=3
//     BATTERY_COLD=4  PROXIMITY_WARN=5  PROXIMITY_CRIT=6
//     FAULT_DISPERSION=7  FAULT_GENERIC=255
//
//   Note: firmware "FAULT_DISPERSION" maps to server "DISPERSION_CLOG".
// ---------------------------------------------------------------------------
/** @readonly */
const FAULT_CODE = Object.freeze({
  NONE:             'NONE',
  IMU_TIMEOUT:      'IMU_TIMEOUT',
  GPS_LOSS:         'GPS_LOSS',
  MOTOR_FEEDBACK:   'MOTOR_FEEDBACK',   // firmware FAULT_MOTOR_FEEDBACK (3)
  BATTERY_COLD:     'BATTERY_COLD',
  PROXIMITY_WARN:   'PROXIMITY_WARN',
  PROXIMITY_CRIT:   'PROXIMITY_CRIT',
  DISPERSION_CLOG:  'DISPERSION_CLOG',  // firmware FAULT_DISPERSION (7)
  GENERIC:          'GENERIC',
});

/**
 * Maps FAULT_CODE strings to their firmware FaultCode_t integer values.
 * Used by lora_contracts.h generation and LoRa fault packet parsing.
 * @readonly
 */
const FAULT_CODE_NUM = Object.freeze({
  NONE:            0,
  IMU_TIMEOUT:     1,
  GPS_LOSS:        2,
  MOTOR_FEEDBACK:  3,
  BATTERY_COLD:    4,
  PROXIMITY_WARN:  5,
  PROXIMITY_CRIT:  6,
  DISPERSION_CLOG: 7,   // firmware: FAULT_DISPERSION
  GENERIC:         255,
});

// ---------------------------------------------------------------------------
// 4. FAULT ACTIONS  (STM32 → LoRa → server, "action" field)
// ---------------------------------------------------------------------------
/** @readonly */
const FAULT_ACTION = Object.freeze({
  PAUSE:    'PAUSE',
  ESTOP:    'ESTOP',
  LOG_ONLY: 'LOG_ONLY',
});

// ---------------------------------------------------------------------------
// 5. DISPERSION COMMANDS  (STM32 UART4 → SB ESP32, 9600 baud)
//
//   PCT:<pct>                   — same % for both salt and brine
//   SALT:<pct>,BRINE:<pct>      — independent targets
//   TEST SALT <pct>             — bench-test salt pump only
//   TEST BRINE <pct>            — bench-test brine pump only
// ---------------------------------------------------------------------------
/** @readonly */
const DISP_CMD_PREFIX = Object.freeze({
  PCT:   'PCT',
  SALT:  'SALT',
  BRINE: 'BRINE',
  TEST_SALT:  'TEST SALT',
  TEST_BRINE: 'TEST BRINE',
});

/** Responses from SB ESP32 back to STM32. */
const DISP_RESPONSE = Object.freeze({
  OK:             'STATUS:OK',
  ERR_BAD_CMD:    'STATUS:ERROR,BAD_CMD',
  ERR_OVERFLOW:   'STATUS:ERROR,OVERFLOW',
  // FLOW:SALT:<ml_min>,BRINE:<ml_min>,RPM:<rpm>  — periodic status push
  FLOW_PREFIX:    'FLOW:',
});

// ---------------------------------------------------------------------------
// 6. GATEWAY FRAMING PROTOCOL  (Gateway ESP32 ↔ Base Station ESP32 over LoRa)
//
//   Each LoRa packet is a UTF-8 line (max CHUNK_BYTES bytes payload):
//     Middle frame:  S:<seq>:M:<chunk>\n
//     End frame:     S:<seq>:E:<chunk>\n
//   Acknowledgement (sent back over LoRa):  ACK:S:<seq>:M/<E>:<chunk>
//   Idle flush timeout: 30 ms of UART silence triggers an End frame.
// ---------------------------------------------------------------------------
/** @readonly */
const GATEWAY = Object.freeze({
  CHUNK_BYTES:    150,          // max LoRa payload per frame (bytes)
  IDLE_FLUSH_MS:  30,           // UART idle timer before forced End frame
  FRAME_SEP:      ':',          // token separator
  FRAME_PREFIX:   'S',          // all frames start with 'S'
  FRAME_MIDDLE:   'M',          // middle-chunk marker
  FRAME_END:      'E',          // end-chunk / final frame marker
  ACK_PREFIX:     'ACK:',       // ack line prefix sent back to sender
  LORA_FREQ_MHZ:  915,          // LoRa centre frequency (MHz)
  LORA_SF:        7,            // spreading factor
  LORA_BW:        4,            // bandwidth index (125 kHz)
  LORA_CR:        1,            // coding rate (4/5)
  LORA_PREAMBLE:  8,            // preamble length (symbols)
});

// ---------------------------------------------------------------------------
// 7. BASE STATION NETWORK CONFIG  (jakep2377/base_station)
// ---------------------------------------------------------------------------
/** @readonly */
const BASE_STATION = Object.freeze({
  STA_SSID:     '42Fortress',           // primary WiFi network
  AP_SSID:      'SaltRobot_Base',       // fallback AP
  AP_PASS:      'saltrobot123',
  AP_IP:        '192.168.4.1',          // static IP in AP mode
  HTTP_PATH_STATUS:    '/status',       // GET → JSON (includes queue_depth, last_lora)
  HTTP_PATH_CMD:       '/command',      // POST → plain text; 503 queue_full on backlog
  HTTP_PATH_LAST_LORA: '/last_lora',    // GET → text/plain raw last LoRa RX (Phase D)
});

// ---------------------------------------------------------------------------
// 8. MISSION STATES  (server-side lifecycle, published in mission.state field)
// ---------------------------------------------------------------------------
/** @readonly */
const MISSION_STATE = Object.freeze({
  IDLE:        'IDLE',        // no active mission, server ready
  CONFIGURING: 'CONFIGURING', // area/boundary set, not yet started
  RUNNING:     'RUNNING',     // robot executing coverage autonomously
  PAUSED:      'PAUSED',      // operator or fault-triggered pause
  COMPLETED:   'COMPLETED',   // full coverage reached or operator ended cleanly
  ABORTED:     'ABORTED',     // operator hard-stopped or unrecoverable fault
  ERROR:       'ERROR',       // server-side error (e.g. DB failure)
});

/** Valid state transitions: MISSION_STATE_TRANSITIONS[from] = Set<to> */
const MISSION_STATE_TRANSITIONS = Object.freeze({
  [MISSION_STATE.IDLE]:        new Set([MISSION_STATE.CONFIGURING]),
  [MISSION_STATE.CONFIGURING]: new Set([MISSION_STATE.RUNNING, MISSION_STATE.IDLE]),
  [MISSION_STATE.RUNNING]:     new Set([MISSION_STATE.PAUSED, MISSION_STATE.COMPLETED, MISSION_STATE.ABORTED, MISSION_STATE.ERROR]),
  [MISSION_STATE.PAUSED]:      new Set([MISSION_STATE.RUNNING, MISSION_STATE.ABORTED]),
  [MISSION_STATE.COMPLETED]:   new Set([MISSION_STATE.IDLE]),
  [MISSION_STATE.ABORTED]:     new Set([MISSION_STATE.IDLE]),
  [MISSION_STATE.ERROR]:       new Set([MISSION_STATE.IDLE]),
});

// ---------------------------------------------------------------------------
// 9. EVENT LOG TYPES  (written to DB by server on every significant action)
// ---------------------------------------------------------------------------
/** @readonly */
const EVENT_TYPE = Object.freeze({
  MISSION_STARTED:    'mission.started',
  MISSION_PAUSED:     'mission.paused',
  MISSION_RESUMED:    'mission.resumed',
  MISSION_COMPLETED:  'mission.completed',
  MISSION_ABORTED:    'mission.aborted',
  TELEMETRY_RECEIVED: 'telemetry.received',
  COMMAND_SENT:       'command.sent',
  FAULT_RECEIVED:     'fault.received',
  OPERATOR_NOTE_ADDED: 'operator.note.added',
  OPERATOR_WORKFLOW_UPDATED: 'operator.workflow.updated',
  AREA_SET:           'area.set',
  PATH_PLANNED:       'path.planned',
  SERVER_STARTED:     'server.started',
});

// ---------------------------------------------------------------------------
// 10. SERVER WebSocket EVENT NAMES  (server → app)
// ---------------------------------------------------------------------------
/** @readonly */
const WS_EVENT = Object.freeze({
  STATE_SNAPSHOT:    'state.snapshot',    // full state pushed on connect
  AREA_UPDATED:      'area.updated',      // input area / boundary changed
  TELEMETRY_UPDATED: 'telemetry.updated', // new robot position or stats
  PATH_UPDATED:      'path.updated',      // coverage path recalculated
  COMMAND_RECEIVED:  'command.received',  // command accepted by server
  FAULT_RECEIVED:    'fault.received',    // fault notification from STM32
  MISSION_UPDATED:   'mission.updated',   // mission state transition
  SUPERVISION_UPDATED: 'supervision.updated',
  OPERATOR_UPDATED:    'operator.updated',
});

// ---------------------------------------------------------------------------
// 11. SERVER HTTP API PATHS
// ---------------------------------------------------------------------------
/** @readonly */
const API = Object.freeze({
  ROOT:             '/',
  STATUS:           '/status',                // app-compat: GET
  COMMAND:          '/command',               // app-compat: POST
  HEALTH:           '/api/health',            // GET
  SUPERVISION_SUMMARY: '/api/supervision/summary',
  INPUT_AREA:       '/api/input-area',        // POST
  TELEMETRY:        '/api/telemetry',         // POST
  STATE:            '/api/state',             // GET
  COVERAGE:         '/api/coverage',          // GET
  PATH_PLAN:        '/api/path/plan',         // POST
  // Mission lifecycle (Phase B)
  MISSION_START:    '/api/mission/start',     // POST
  MISSION_PAUSE:    '/api/mission/pause',     // POST
  MISSION_RESUME:   '/api/mission/resume',    // POST
  MISSION_ABORT:    '/api/mission/abort',     // POST
  MISSION_COMPLETE: '/api/mission/complete',  // POST
  MISSION_CURRENT:  '/api/mission/current',   // GET
  MISSION_HISTORY:  '/api/mission/history',   // GET
  MISSION_EVENTS:   '/api/mission/:id/events',// GET
  OPERATOR_WORKFLOWS: '/api/operator/workflows',
  OPERATOR_WORKFLOW_STEP: '/api/operator/workflows/:workflowId/steps/:stepId',
  OPERATOR_NOTES: '/api/operator/notes',
  // LoRa bridge (Phase C)
  LORA_PUSH_WP:     '/api/lora/push-waypoints', // POST
  LORA_STATUS:      '/api/lora/status',          // GET
});

// ---------------------------------------------------------------------------
// 12. LORA WIRE PROTOCOL — waypoint injection + arbitration  (Phase C)
//
//   The server pushes waypoints to the STM32 over LoRa using this text
//   protocol.  Each line is sent as a plain-text POST /command to the base
//   station, which queues them for LoRa TX.  The firmware must handle these
//   commands in uart_lora.c / main.c (see firmware/lora_contracts.h).
//
//   Sequence (server → base station → LoRa → gateway → STM32):
//     WPCLEAR               — clear staged waypoints, reset mission
//     WP:<idx>:<lat>,<lon>,<salt>,<brine>  — add one waypoint (0-indexed)
//     WPLOAD:<count>         — commit staged waypoints; ready for AUTO
//
//   Wire constraints:
//     - Max payload per LoRa frame: GATEWAY.CHUNK_BYTES (150 bytes)
//     - WP line max: "WP:49:41.0762,-81.5140,100,100" ≈ 31 chars  ✓
//     - Max waypoints per mission: MAX_WAYPOINTS (50)
//
//   ACK from STM32 (sent back over LoRa to base station):
//     ACK:WPCLEAR
//     ACK:WP:<idx>
//     ACK:WPLOAD:<count>
//
//   On firmware side, RESET over LoRa transitions to STATE_PAUSE
//   (same as console RESET command).
// ---------------------------------------------------------------------------
/** @readonly */
const LORA_WIRE = Object.freeze({
  // Waypoint injection protocol
  WP_CLEAR:   'WPCLEAR',          // clear staged waypoints
  WP_ADD:     'WP',               // WP:<idx>:<lat>,<lon>,<salt%>,<brine%>
  WP_LOAD:    'WPLOAD',           // WPLOAD:<count>  — commit + arm auto mode
  WP_ACK_CLEAR: 'ACK:WPCLEAR',
  WP_ACK_ADD:   'ACK:WP',         // prefix; full frame: ACK:WP:<idx>
  WP_ACK_LOAD:  'ACK:WPLOAD',     // prefix; full frame: ACK:WPLOAD:<count>
  MAX_WAYPOINTS: 50,              // matches firmware MAX_WAYPOINTS

  // Inter-line delay for sequential WP commands (ms)
  // Gives the base station/gateway time to flush each frame.
  WP_INTERLINE_MS: 120,
});

// ---------------------------------------------------------------------------
// 13. COMMAND ARBITRATION  (server-side enforcement, Phase C)
//
//   Priority levels (highest wins when multiple requests arrive together):
//     P0 ESTOP       — always forwarded, aborts mission immediately
//     P1 RESET       — forwarded only from PAUSED/ERROR/ABORTED server state
//     P2 PAUSE       — forwarded unconditionally
//     P3 MANUAL      — forwarded; warns if WP push is committed (auto loaded)
//     P4 drive cmds  — FORWARD/BACKWARD/LEFT/RIGHT/STOP — forwarded always
//     P5 AUTO        — forwarded only when wpPushState === 'committed'
//     P6 WP protocol — forwarded only when mission is CONFIGURING/PAUSED
// ---------------------------------------------------------------------------
/** @readonly */
const ARBITRATION = Object.freeze({
  // Priority numbers (lower = higher priority)
  P_ESTOP:   0,
  P_RESET:   1,
  P_PAUSE:   2,
  P_MANUAL:  3,
  P_DRIVE:   4,
  P_AUTO:    5,
  P_WP:      6,

  // wp push lifecycle states
  WP_NONE:        'none',        // no waypoints staged
  WP_PENDING:     'pending',     // WPCLEAR sent, WP:n sending in progress
  WP_COMMITTED:   'committed',   // WPLOAD:n sent, firmware has waypoints
  WP_FAILED:      'failed',      // comms error during push
});

// ---------------------------------------------------------------------------
// 14. TELEMETRY SCHEMA  (reference / documentation — not runtime-enforced)
//
//   Source: STM32 firmware, transmitted over UART5 (115 200 baud) every 10 s.
//   After gateway framing + base-station passthrough the server receives the
//   inner JSON verbatim via POST /api/telemetry or base-station's last_lora.
//
//   {
//     "state":   <ROBOT_STATE>,
//     "gps":     { "lat": number, "lon": number, "fix": 0|1, "sat": int, "hdop": number },
//     "motor":   { "m1": int(-100..100), "m2": int(-100..100) },
//     "heading": { "yaw": number(deg), "pitch": number(deg) },
//     "disp":    { "salt": int(0..100), "brine": int(0..100) },
//     "temp":    number(°C),
//     "prox":    { "left": int(cm), "right": int(cm) }
//   }
//
//   Fault packet (does NOT include gps/motor fields):
//   {
//     "fault":  <FAULT_CODE>,
//     "action": <FAULT_ACTION>
//   }
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// 13. MESSAGE SEQUENCE IDs
//
//   Used for logging, tracing, and future ACK matching.
//   The gateway assigns an incrementing uint16 per UART session (wraps at 65535).
//   The server mirrors this in the 'at' timestamp field on WS events.
//   Reserved range 0x0000: unsequenced / legacy compatibility.
// ---------------------------------------------------------------------------
const SEQ = Object.freeze({
  UNSEQUENCED: 0,
  MAX:         65535,
  WRAP_AT:     65536,
});

// ---------------------------------------------------------------------------
// Exports
// ---------------------------------------------------------------------------
module.exports = {
  ROBOT_STATE,
  CMD,
  CMD_ALIASES,
  FAULT_CODE,
  FAULT_CODE_NUM,
  FAULT_ACTION,
  DISP_CMD_PREFIX,
  DISP_RESPONSE,
  GATEWAY,
  BASE_STATION,
  MISSION_STATE,
  MISSION_STATE_TRANSITIONS,
  EVENT_TYPE,
  WS_EVENT,
  API,
  SEQ,
  LORA_WIRE,
  ARBITRATION,
};
