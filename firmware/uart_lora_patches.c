/**
 * uart_lora_patches.c  —  STM32 patch guide for Core/Src/uart_lora.c
 *
 * Repo: griffinMcConaha/Senior-Design-Project-DT10
 * File: Core/Src/uart_lora.c
 *
 * HOW TO APPLY
 * ────────────
 * This file is NOT a drop-in replacement — uart_lora.c is large (~700 lines)
 * and most of it is unchanged.  Instead, apply each numbered patch below
 * by finding the BEFORE block in the original file and replacing it with
 * the AFTER block.  Each patch is self-contained and independent.
 *
 * PATCH SUMMARY
 * ─────────────
 *  Patch 1 — Add new #include lines at top of file
 *  Patch 2 — Add static WP receive buffer  (new global vars, near top)
 *  Patch 3 — Add 'W'/'w' to first-byte filter in LoRA_RxByte()
 *  Patch 4 — Add RESET handler in lora_parse_state_request()
 *  Patch 5 — Add WPCLEAR / WP: / WPLOAD dispatcher in LoRA_RxByte()
 *  Patch 6 — Fix LoRA_SendFault() string table to match FaultCode_t enum
 */

#include "uart_lora.h"   /* already in original */
/* ... other existing includes ... */

/* ═══════════════════════════════════════════════════════════════════════════
 * PATCH 1 — Add includes
 * Location: near the top of uart_lora.c, after the existing #include block
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * BEFORE (no change needed if already present):
 *   #include <string.h>
 *   ... existing includes ...
 *
 * AFTER — add these two lines if not already present:
 */
#include "lora_contracts.h"   /* shared constants — copy firmware/lora_contracts.h
                                  into Core/Inc/ of the STM32 project first       */
#include "mission.h"          /* Mission_AddWaypoint, Mission_Init, MAX_WAYPOINTS */
#include "robot_sm.h"         /* RobotSM_LoadMission, Waypoint_t, RobotSM_t       */


/* ═══════════════════════════════════════════════════════════════════════════
 * PATCH 2 — Static WP receive buffer
 * Location: near top of uart_lora.c, with the other static global variables
 *           (e.g. after the LoRA_State_t definition or near the top of the
 *           static variable block)
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * ADD these lines (do NOT replace anything — just insert):
 */

/* Over-the-air mission buffer — filled by WPCLEAR/WP:/WPLOAD sequence */
static Waypoint_t  s_lora_wp_buf[MAX_WAYPOINTS];
static uint8_t     s_lora_wp_count = 0;

/* External state machine (defined in main.c) */
extern RobotSM_t g_sm;


/* ═══════════════════════════════════════════════════════════════════════════
 * PATCH 3 — Add 'W'/'w' to the first-byte filter in LoRA_RxByte()
 *
 * Location: inside LoRA_RxByte() — the early-return guard that rejects bytes
 *           whose first character is not part of any known command.
 *           Search for the long condition with 'C', 'S', 'A', 'M', 'P', 'E'.
 *
 * Without this patch all WPCLEAR / WP: / WPLOAD frames are silently dropped.
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * BEFORE:
 *
 *   if (byte != 'C' && byte != 'S' && byte != 'A' && byte != 'M' && byte != 'P'
 *       && byte != 'E' && byte != 'a' && byte != 'm' && byte != 'p' && byte != 'e') {
 *       return;
 *   }
 *
 * AFTER:
 */
#if 0   /* illustrative — copy the block below into uart_lora.c */
    if (byte != 'C' && byte != 'S' && byte != 'A' && byte != 'M' && byte != 'P'
        && byte != 'E' && byte != 'W'                                /* ← ADDED */
        && byte != 'a' && byte != 'm' && byte != 'p' && byte != 'e' && byte != 'w') { /* ← 'w' added */
        return;
    }
#endif


/* ═══════════════════════════════════════════════════════════════════════════
 * PATCH 4 — Add RESET handler in lora_parse_state_request()
 *
 * Location: inside lora_parse_state_request(), after the ESTOP/STOP block
 *           and before the final   return 0;   (the "unknown command" return).
 *
 * RESET transitions the robot to STATE_PAUSE (clears ESTOP/ERROR latch).
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * BEFORE (schematic — find the actual ESTOP block and insert after it):
 *
 *   if (strcmp(payload, "ESTOP") == 0 || strcmp(payload, "STOP") == 0) {
 *       *out_state = 4;   // STATE_ESTOP
 *       return 1;
 *   }
 *                           ← INSERT HERE
 *   return 0;  // unknown
 *
 * AFTER — insert this block between ESTOP handling and the final return:
 */
#if 0   /* illustrative */
    if (strcmp(payload, "RESET") == 0) {
        *out_state = 2;   /* STATE_PAUSE — same effect as console RESET */
        return 1;
    }
#endif


/* ═══════════════════════════════════════════════════════════════════════════
 * PATCH 5 — WPCLEAR / WP: / WPLOAD dispatcher in LoRA_RxByte()
 *
 * Location: inside LoRA_RxByte(), in the section that dispatches the
 *           assembled command string (variable name is typically 'cmd' or
 *           stored in the stream_buffer).  The existing structure is:
 *
 *     if (lora_parse_state_request(cmd, &state)) {
 *         ...
 *     } else if (lora_parse_manual_request(cmd, &manual_cmd)) {
 *         ...
 *     } else {
 *         // invalid / unknown
 *     }
 *
 * INSERT the three new else-if branches between the manual_request block
 * and the final else { } so the chain becomes:
 *
 *     } else if (lora_parse_manual_request(...)) {
 *         ...
 *     } else if (strncmp(cmd, "WPCLEAR", 7) == 0) {   ← NEW
 *         ...
 *     } else if (strncmp(cmd, "WP:", 3) == 0) {        ← NEW
 *         ...
 *     } else if (strncmp(cmd, "WPLOAD:", 7) == 0) {    ← NEW
 *         ...
 *     } else {
 *         // invalid
 *     }
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * The three new branches to insert:
 */
#if 0   /* illustrative */

    /* ── WPCLEAR ── clear the staged LoRa waypoint buffer */
    } else if (strncmp(cmd, "WPCLEAR", 7) == 0) {
        memset(s_lora_wp_buf, 0, sizeof(s_lora_wp_buf));
        s_lora_wp_count = 0;
        Mission_Init();           /* also clears the mission module's internal buffer */
        LoRA_SendRaw("ACK:WPCLEAR");

    /* ── WP:<idx>:<lat>,<lon>,<salt>,<brine> ── add one waypoint */
    } else if (strncmp(cmd, "WP:", 3) == 0) {
        uint8_t  idx;
        float    lat, lon;
        int      salt_pct, brine_pct;

        /* sscanf parses: "WP:0:41.076200,-81.514000,80,20" */
        if (sscanf(cmd + 3, "%hhu:%f,%f,%d,%d",
                   &idx, &lat, &lon, &salt_pct, &brine_pct) == 5
            && idx < MAX_WAYPOINTS)
        {
            /* Clamp rates to 0-100 before storing */
            if (salt_pct  < 0)   salt_pct  = 0;
            if (salt_pct  > 100) salt_pct  = 100;
            if (brine_pct < 0)   brine_pct = 0;
            if (brine_pct > 100) brine_pct = 100;

            /* Write directly to the index slot so out-of-order delivery works */
            s_lora_wp_buf[idx].latitude  = lat;
            s_lora_wp_buf[idx].longitude = lon;
            s_lora_wp_buf[idx].salt_rate  = (float)salt_pct  / 100.0f;
            s_lora_wp_buf[idx].brine_rate = (float)brine_pct / 100.0f;

            /* Advance count to the highest index seen + 1 */
            if ((uint8_t)(idx + 1) > s_lora_wp_count)
                s_lora_wp_count = (uint8_t)(idx + 1);

            /* Also register with the mission module for stats/reporting */
            Mission_AddWaypoint(lat, lon, (uint8_t)salt_pct, (uint8_t)brine_pct);

            char ack[24];
            snprintf(ack, sizeof(ack), "ACK:WP:%hhu", idx);
            LoRA_SendRaw(ack);
        }

    /* ── WPLOAD:<count> ── commit staged waypoints into state machine */
    } else if (strncmp(cmd, "WPLOAD:", 7) == 0) {
        uint8_t count;
        if (sscanf(cmd + 7, "%hhu", &count) == 1 && count > 0
            && count == s_lora_wp_count)
        {
            /* Hand the waypoint array to the state machine.
               Robot enters STATE_AUTO when server sends the AUTO command. */
            RobotSM_LoadMission(&g_sm, s_lora_wp_buf, s_lora_wp_count);

            char ack[28];
            snprintf(ack, sizeof(ack), "ACK:WPLOAD:%hhu", count);
            LoRA_SendRaw(ack);
        }

#endif


/* ═══════════════════════════════════════════════════════════════════════════
 * PATCH 6 — Fix LoRA_SendFault() string table
 *
 * Location: inside LoRA_SendFault() — the switch(fault_code) block that
 *           maps numeric fault codes to strings for transmission.
 *
 * The original table was incorrect (labels shifted vs. robot_sm.h enum).
 * New table matches FaultCode_t in Core/Inc/robot_sm.h exactly:
 *   FAULT_NONE=0  FAULT_IMU_TIMEOUT=1  FAULT_GPS_LOSS=2  FAULT_MOTOR_FEEDBACK=3
 *   FAULT_BATTERY_COLD=4  FAULT_PROXIMITY_WARN=5  FAULT_PROXIMITY_CRIT=6
 *   FAULT_DISPERSION=7  FAULT_GENERIC=255
 *
 * The wire string "DISPERSION_CLOG" for case 7 is intentional — it matches
 * the contracts.js FAULT_CODE.DISPERSION_CLOG key that the server expects.
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * BEFORE (in LoRA_SendFault switch block):
 *
 *   case 3: fault_name = "PROXIMITY_WARN";  break;
 *   case 4: fault_name = "PROXIMITY_CRIT";  break;
 *   case 5: fault_name = "BATTERY_COLD";    break;
 *   case 6: fault_name = "DISPERSION_CLOG"; break;
 *   case 7: fault_name = "GENERIC";         break;
 *
 * AFTER — replace the entire switch body with:
 */
#if 0   /* illustrative */
    switch (fault_code) {
        case 0:   fault_name = "NONE";            break;
        case 1:   fault_name = "IMU_TIMEOUT";     break;
        case 2:   fault_name = "GPS_LOSS";        break;
        case 3:   fault_name = "MOTOR_FEEDBACK";  break;   /* FAULT_MOTOR_FEEDBACK  */
        case 4:   fault_name = "BATTERY_COLD";    break;   /* FAULT_BATTERY_COLD    */
        case 5:   fault_name = "PROXIMITY_WARN";  break;   /* FAULT_PROXIMITY_WARN  */
        case 6:   fault_name = "PROXIMITY_CRIT";  break;   /* FAULT_PROXIMITY_CRIT  */
        case 7:   fault_name = "DISPERSION_CLOG"; break;   /* FAULT_DISPERSION      */
        default:  fault_name = "GENERIC";         break;   /* FAULT_GENERIC (255)   */
    }
#endif
