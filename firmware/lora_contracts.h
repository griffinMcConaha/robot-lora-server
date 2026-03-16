/**
 * @file   lora_contracts.h
 * @brief  LoRa wire protocol constants shared between the server and STM32.
 *
 * ┌──────────────────────────────────────────────────────────────────────────┐
 * │  GENERATED FROM  robot-lora-server/src/contracts.js  —  DO NOT EDIT     │
 * │  Match every numeric/string constant here to contracts.js exactly.       │
 * │  Update both files together when the protocol changes.                   │
 * └──────────────────────────────────────────────────────────────────────────┘
 *
 * Copy this file to  Core/Inc/lora_contracts.h  in the STM32 project.
 *
 * Usage example (uart_lora.c):
 *   #include "lora_contracts.h"
 *   if (strncmp(buf, LORA_WP_ADD_PREFIX, strlen(LORA_WP_ADD_PREFIX)) == 0) { ... }
 */

#ifndef LORA_CONTRACTS_H
#define LORA_CONTRACTS_H

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * 1.  ROBOT STATE  (must match RobotState_t in robot_sm.h)
 * ═══════════════════════════════════════════════════════════════════════════ */
#define ROBOT_STATE_MANUAL   0
#define ROBOT_STATE_AUTO     1
#define ROBOT_STATE_PAUSE    2
#define ROBOT_STATE_ERROR    3
#define ROBOT_STATE_ESTOP    4

/* LoRa text tokens that drive lora_parse_state_request() */
#define LORA_CMD_AUTO           "AUTO"
#define LORA_CMD_MANUAL         "MANUAL"
#define LORA_CMD_PAUSE          "PAUSE"
#define LORA_CMD_ESTOP          "ESTOP"
#define LORA_CMD_STOP           "STOP"
/**
 * RESET is NOT currently handled in uart_lora.c lora_parse_state_request().
 * ─── FIRMWARE PATCH REQUIRED ────────────────────────────────────────────
 * In uart_lora.c, inside lora_parse_state_request(), add:
 *
 *   if (strcmp(payload, LORA_CMD_RESET) == 0) {
 *       *out_state = ROBOT_STATE_PAUSE;   // same effect as console RESET
 *       return 1;
 *   }
 * ────────────────────────────────────────────────────────────────────────
 */
#define LORA_CMD_RESET          "RESET"

/* Manual drive tokens that drive lora_parse_manual_cmd() */
#define LORA_CMD_FORWARD        "FORWARD"
#define LORA_CMD_BACKWARD       "BACKWARD"
#define LORA_CMD_LEFT           "LEFT"
#define LORA_CMD_RIGHT          "RIGHT"
#define LORA_CMD_STOP_DRIVE     "STOP"    /* shared with state STOP */

/* ═══════════════════════════════════════════════════════════════════════════
 * 2.  FAULT CODES  (must match FaultCode_t in robot_sm.h)
 * ═══════════════════════════════════════════════════════════════════════════ */
#define LORA_FAULT_NONE             0
#define LORA_FAULT_IMU_TIMEOUT      1
#define LORA_FAULT_GPS_LOSS         2
#define LORA_FAULT_MOTOR_FEEDBACK   3   /* encoder / flow-sensor failure       */
#define LORA_FAULT_BATTERY_COLD     4
#define LORA_FAULT_PROXIMITY_WARN   5
#define LORA_FAULT_PROXIMITY_CRIT   6
#define LORA_FAULT_DISPERSION       7   /* server calls this DISPERSION_CLOG   */
#define LORA_FAULT_GENERIC          255

/* ═══════════════════════════════════════════════════════════════════════════
 * 3.  WAYPOINT INJECTION PROTOCOL  (server → base-station → STM32 via LoRa)
 *
 *   Step 1:   WPCLEAR
 *               Clears all staged waypoints and resets the mission index.
 *               Firmware ACKs with: ACK:WPCLEAR
 *
 *   Step 2:   WP:<idx>:<lat>,<lon>,<salt>,<brine>
 *               Adds one waypoint at zero-based index <idx>.
 *               <lat>/<lon>  — decimal degrees, 6 decimal places
 *               <salt>       — integer 0–100 (firmware divides by 100.0f)
 *               <brine>      — integer 0–100 (firmware divides by 100.0f)
 *               Firmware ACKs with: ACK:WP:<idx>
 *
 *   Step 3:   WPLOAD:<count>
 *               Commits the staged waypoints and loads the mission.
 *               Equivalent to calling Mission_Load() + RobotSM_LoadMission().
 *               Firmware ACKs with: ACK:WPLOAD:<count>
 *
 *   After WPLOAD the robot is ready for AUTO.  Send  AUTO  to begin.
 *
 *   Wire constraints
 *   ─────────────────
 *   Max LoRa payload  : 150 bytes (GATEWAY.CHUNK_BYTES)
 *   Longest WP line   : "WP:49:41.0762,-81.5140,100,100" ≈ 31 chars  ✓
 *   Max waypoints/mission: LORA_MAX_WAYPOINTS (50)
 *   Inter-line delay  : LORA_WP_INTERLINE_MS (120 ms) — set by lora_bridge.js
 * ═══════════════════════════════════════════════════════════════════════════ */

#define LORA_WP_CLEAR           "WPCLEAR"
#define LORA_WP_ADD_PREFIX      "WP"      /* full form: WP:<idx>:<lat>,<lon>,<salt>,<brine> */
#define LORA_WP_LOAD_PREFIX     "WPLOAD"  /* full form: WPLOAD:<count>                       */

#define LORA_WP_ACK_CLEAR       "ACK:WPCLEAR"
#define LORA_WP_ACK_ADD_PREFIX  "ACK:WP"   /* full form: ACK:WP:<idx>                        */
#define LORA_WP_ACK_LOAD_PREFIX "ACK:WPLOAD" /* full form: ACK:WPLOAD:<count>                */

#define LORA_MAX_WAYPOINTS      50          /* must match MAX_WAYPOINTS in mission.h          */

/* ═══════════════════════════════════════════════════════════════════════════
 * 4.  FIRMWARE PATCHES REQUIRED  (uart_lora.c)
 *
 *  ─── PATCH 1: Add RESET to lora_parse_state_request() ───────────────────
 *
 *  In uart_lora.c → lora_parse_state_request() add before the return 0:
 *
 *    } else if (strcmp(payload, LORA_CMD_RESET) == 0) {
 *        *out_state = ROBOT_STATE_PAUSE;
 *        return 1;
 *
 *  ─── PATCH 2: Add waypoint injection parser ──────────────────────────────
 *
 *  In uart_lora.c → lora_process_message() (or equivalent dispatcher) add:
 *
 *    // WPCLEAR
 *    if (strcmp(payload, LORA_WP_CLEAR) == 0) {
 *        Mission_Clear();
 *        LoRa_SendAck(LORA_WP_ACK_CLEAR);
 *        return;
 *    }
 *
 *    // WP:<idx>:<lat>,<lon>,<salt>,<brine>
 *    if (strncmp(payload, LORA_WP_ADD_PREFIX ":", 3) == 0) {
 *        uint8_t idx;
 *        float   lat, lon;
 *        int     salt_pct, brine_pct;
 *        // sscanf format: "WP:%hhu:%f,%f,%d,%d"
 *        if (sscanf(payload + 3, "%hhu:%f,%f,%d,%d",
 *                   &idx, &lat, &lon, &salt_pct, &brine_pct) == 5) {
 *            // Firmware Waypoint_t.salt_rate is float 0.0–1.0
 *            Mission_AddWaypoint(lat, lon,
 *                                (uint8_t)salt_pct,
 *                                (uint8_t)brine_pct);
 *            char ack[32];
 *            snprintf(ack, sizeof(ack), LORA_WP_ACK_ADD_PREFIX ":%hhu", idx);
 *            LoRa_SendAck(ack);
 *        }
 *        return;
 *    }
 *
 *    // WPLOAD:<count>
 *    if (strncmp(payload, LORA_WP_LOAD_PREFIX ":", 7) == 0) {
 *        uint8_t count;
 *        if (sscanf(payload + 7, "%hhu", &count) == 1) {
 *            RobotSM_LoadMission();         // commit waypoints, arm AUTO
 *            char ack[32];
 *            snprintf(ack, sizeof(ack), LORA_WP_ACK_LOAD_PREFIX ":%hhu", count);
 *            LoRa_SendAck(ack);
 *        }
 *        return;
 *    }
 *
 *  ─── NOTE on salt_rate / brine_rate ─────────────────────────────────────
 *
 *  Waypoint_t (mission.h) stores salt_rate and brine_rate as uint8_t.
 *  The server sends integer percent values (0–100).
 *  If Mission_AddWaypoint() internally stores them as-is (uint8_t 0–100),
 *  the dispenser logic must divide by 100.0f before applying to PWM output.
 *  Verify this in robot_actions.c → RobotActions_SetDispenser().
 *
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ═══════════════════════════════════════════════════════════════════════════
 * 5.  GATEWAY FRAMING  (base-station → STM32 LoRa receive path)
 *
 *   Gateway wraps each payload:
 *     Start frame : S:<seq>:M:<chunk>\n
 *     End frame   : S:<seq>:E:<chunk>\n
 *   Max chunk size: 150 bytes
 *   Sequence:       uint16, wraps at 65535
 * ═══════════════════════════════════════════════════════════════════════════ */
#define LORA_GATEWAY_FRAME_START_PREFIX  "S:"
#define LORA_GATEWAY_FRAME_MID_TOKEN     ":M:"
#define LORA_GATEWAY_FRAME_END_TOKEN     ":E:"
#define LORA_GATEWAY_MAX_CHUNK_BYTES     150
#define LORA_GATEWAY_SEQ_MAX             65535U

#ifdef __cplusplus
}
#endif

#endif /* LORA_CONTRACTS_H */
