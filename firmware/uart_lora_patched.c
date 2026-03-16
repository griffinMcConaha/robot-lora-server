/*
 * uart_lora_patched.c — patched Core/Src/uart_lora.c for Phase D
 *
 * Drop in as Core/Src/uart_lora.c in griffinMcConaha/Senior-Design-Project-DT10.
 * Also copy firmware/lora_contracts.h → Core/Inc/lora_contracts.h.
 *
 * Changes vs. original:
 *  [P1] #include "mission.h"
 *  [P2] Static WP receive buffer  s_lora_wp_buf / s_lora_wp_count / extern g_sm
 *  [P3] First-byte filter: added 'W' / 'w'
 *  [P4] lora_parse_state_request: RESET → STATE_PAUSE
 *  [P5] LoRA_RxByte dispatch: WPCLEAR / WP: / WPLOAD: branches
 *  [P6] LoRA_SendFault: fixed fault string table to match FaultCode_t enum
 */

#include "uart_lora.h"
#include "robot_sm.h"
#include "system_health.h"
#include "mission.h"   /* [P1] Mission_Init, Mission_AddWaypoint, MAX_WAYPOINTS */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

// ============================================================================
// LoRA UART COMMUNICATION MODULE (UART 5)
// Receives commands from mobile app via LoRA ESP32
// Sends diagnostic data back to base station
// ============================================================================

#define LORA_RX_BUFFER_SIZE 256
#define LORA_COMMAND_TIMEOUT_MS 5000
#define LORA_STREAM_BUFFER_SIZE 512
#define LORA_TX_DIAGNOSTICS_ENABLED 1

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t rx_buffer[LORA_RX_BUFFER_SIZE];
    uint16_t rx_index;
    uint8_t raw_buffer[LORA_RX_BUFFER_SIZE];
    uint16_t raw_index;
    char last_raw_frame[LORA_RX_BUFFER_SIZE];
    uint32_t raw_frame_count;
    uint32_t last_rx_ms;
    uint32_t last_tx_ms;  // Track last successful transmission for health monitoring
    uint32_t rx_count;
    uint32_t tx_count;
    char last_command[128];
    char last_tx_payload[320];
    uint8_t pending_state_request; // 0=none, else state value
    LoRA_ManualCommand_t pending_manual_cmd;
    uint8_t manual_command_valid;
    uint8_t command_valid;
    uint8_t stream_seq_init;
    uint32_t stream_expected_seq;
    uint32_t stream_gap_count;
    char stream_buffer[LORA_STREAM_BUFFER_SIZE];
    uint16_t stream_len;
} LoRA_State_t;

static LoRA_State_t lora_state = {0};
static uint8_t s_lora_verbose = 0;

/* [P2] Over-the-air mission buffer filled by WPCLEAR / WP: / WPLOAD: sequence */
static Waypoint_t s_lora_wp_buf[MAX_WAYPOINTS];
static uint8_t    s_lora_wp_count = 0;
extern RobotSM_t  g_sm;   /* defined in main.c */

// ============================================================================
// Internal helpers
// ============================================================================

static uint8_t lora_parse_state_request(const char *cmd, uint8_t *out_state)
{
    if (!cmd || !out_state) return 0;

    char normalized[64];
    size_t in_len = strlen(cmd);
    if (in_len >= sizeof(normalized)) {
        in_len = sizeof(normalized) - 1;
    }

    memcpy(normalized, cmd, in_len);
    normalized[in_len] = '\0';

    // Trim leading/trailing whitespace
    char *start = normalized;
    while (*start && isspace((unsigned char)*start)) {
        start++;
    }

    char *end = start + strlen(start);
    while (end > start && isspace((unsigned char)*(end - 1))) {
        end--;
    }
    *end = '\0';

    // Normalize case for robust matching
    for (char *p = start; *p != '\0'; ++p) {
        *p = (char)toupper((unsigned char)*p);
    }

    // Accept optional "CMD:" prefix
    const char *payload = start;
    if (strncmp(payload, "CMD:", 4) == 0) {
        payload += 4;
    }

    // Strip optional parameter section (e.g. AUTO,SALT:20)
    char *comma = strchr((char *)payload, ',');
    if (comma) {
        *comma = '\0';
    }

    if (strcmp(payload, "AUTO") == 0) {
        *out_state = 1; // STATE_AUTO
        return 1;
    }
    if (strcmp(payload, "MANUAL") == 0) {
        *out_state = 0; // STATE_MANUAL
        return 1;
    }
    if (strcmp(payload, "PAUSE") == 0) {
        *out_state = 2; // STATE_PAUSE
        return 1;
    }
    if (strcmp(payload, "ESTOP") == 0 || strcmp(payload, "STOP") == 0) {
        *out_state = 4; // STATE_ESTOP
        return 1;
    }
    /* [P4] RESET transitions to STATE_PAUSE (clears ESTOP / ERROR latch) */
    if (strcmp(payload, "RESET") == 0) {
        *out_state = 2; /* STATE_PAUSE */
        return 1;
    }

    return 0;
}

static uint8_t lora_parse_manual_request(const char *cmd, LoRA_ManualCommand_t *out_cmd)
{
    if (!cmd || !out_cmd) return 0;

    char normalized[80];
    size_t in_len = strlen(cmd);
    if (in_len >= sizeof(normalized)) {
        in_len = sizeof(normalized) - 1;
    }

    memcpy(normalized, cmd, in_len);
    normalized[in_len] = '\0';

    char *start = normalized;
    while (*start && isspace((unsigned char)*start)) {
        start++;
    }

    char *end = start + strlen(start);
    while (end > start && isspace((unsigned char)*(end - 1))) {
        end--;
    }
    *end = '\0';

    for (char *p = start; *p != '\0'; ++p) {
        *p = (char)toupper((unsigned char)*p);
    }

    const char *payload = start;
    if (strncmp(payload, "CMD:", 4) == 0) {
        payload += 4;
    }

    if (strcmp(payload, "FORWARD") == 0) {
        *out_cmd = LORA_MANUAL_CMD_FORWARD;
        return 1;
    }
    if (strcmp(payload, "BACK") == 0 || strcmp(payload, "BACKWARD") == 0) {
        *out_cmd = LORA_MANUAL_CMD_BACK;
        return 1;
    }
    if (strcmp(payload, "LEFT") == 0) {
        *out_cmd = LORA_MANUAL_CMD_LEFT;
        return 1;
    }
    if (strcmp(payload, "RIGHT") == 0) {
        *out_cmd = LORA_MANUAL_CMD_RIGHT;
        return 1;
    }
    if (strcmp(payload, "STOP") == 0) {
        *out_cmd = LORA_MANUAL_CMD_STOP;
        return 1;
    }

    // Basic JSON compatibility for upcoming app payloads
    if (payload[0] == '{') {
        if (strstr(payload, "FORWARD") != NULL) {
            *out_cmd = LORA_MANUAL_CMD_FORWARD;
            return 1;
        }
        if (strstr(payload, "BACKWARD") != NULL || strstr(payload, "BACK") != NULL) {
            *out_cmd = LORA_MANUAL_CMD_BACK;
            return 1;
        }
        if (strstr(payload, "LEFT") != NULL) {
            *out_cmd = LORA_MANUAL_CMD_LEFT;
            return 1;
        }
        if (strstr(payload, "RIGHT") != NULL) {
            *out_cmd = LORA_MANUAL_CMD_RIGHT;
            return 1;
        }
        if (strstr(payload, "STOP") != NULL) {
            *out_cmd = LORA_MANUAL_CMD_STOP;
            return 1;
        }
    }

    return 0;
}

static void lora_uart5_send(const char *msg, const char *tag)
{
    if (!lora_state.huart || !msg) return;

#if LORA_TX_DIAGNOSTICS_ENABLED
    HAL_UART_Transmit(lora_state.huart, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    strncpy(lora_state.last_tx_payload, msg, sizeof(lora_state.last_tx_payload) - 1);
    lora_state.last_tx_payload[sizeof(lora_state.last_tx_payload) - 1] = '\0';
    lora_state.last_tx_ms = HAL_GetTick();
    lora_state.tx_count++;
    if (s_lora_verbose) {
        printf("[LORA] Sent %s: %s", tag ? tag : "", msg);
    }
#else
    (void)tag;
    if (s_lora_verbose) {
        printf("[LORA] TX suppressed\r\n");
    }
#endif
}

static const char* lora_unwrap_stream_frame(char *msg, uint32_t *out_seq, uint8_t *out_has_seq)
{
    if (out_has_seq) *out_has_seq = 0;
    if (out_seq) *out_seq = 0;
    if (!msg) return NULL;

    if (strncmp(msg, "S:", 2) != 0) {
        return msg;
    }

    char *seq_start = msg + 2;
    char *endptr = NULL;
    unsigned long seq = strtoul(seq_start, &endptr, 10);
    if (endptr == seq_start || !endptr || *endptr != ':') {
        return msg;
    }

    if (out_seq) {
        *out_seq = (uint32_t)seq;
    }
    if (out_has_seq) {
        *out_has_seq = 1;
    }

    return endptr + 1;
}

// ============================================================================
// Public API
// ============================================================================

// Initialize LoRA UART interface (UART5, 115200 baud)
void LoRA_Init(UART_HandleTypeDef *huart5)
{
    if (!huart5) return;

    lora_state.huart = huart5;
    lora_state.rx_index = 0;
    lora_state.raw_index = 0;
    lora_state.raw_frame_count = 0;
    lora_state.last_rx_ms = 0;
    lora_state.last_tx_ms = 0;
    lora_state.rx_count = 0;
    lora_state.tx_count = 0;
    lora_state.pending_state_request = 0;
    lora_state.pending_manual_cmd = LORA_MANUAL_CMD_NONE;
    lora_state.manual_command_valid = 0;
    lora_state.stream_seq_init = 0;
    lora_state.stream_expected_seq = 0;
    lora_state.stream_gap_count = 0;
    lora_state.stream_len = 0;
    memset(lora_state.rx_buffer, 0, LORA_RX_BUFFER_SIZE);
    memset(lora_state.raw_buffer, 0, LORA_RX_BUFFER_SIZE);
    memset(lora_state.last_raw_frame, 0, sizeof(lora_state.last_raw_frame));
    memset(lora_state.last_command, 0, sizeof(lora_state.last_command));
    memset(lora_state.last_tx_payload, 0, sizeof(lora_state.last_tx_payload));
    memset(lora_state.stream_buffer, 0, sizeof(lora_state.stream_buffer));

    if (s_lora_verbose) {
        printf("[LORA] LoRA module initialized on UART5 (115200 baud)\r\n");
    }
}

// Process incoming byte from UART 5 RX (call from ISR)
void LoRA_RxByte(uint8_t byte)
{
    if (!lora_state.huart) return;

    // Check for line ending (CR or LF)
    if (byte == '\r' || byte == '\n')
    {
        if (lora_state.raw_index > 0)
        {
            lora_state.raw_buffer[lora_state.raw_index] = '\0';
            strncpy(lora_state.last_raw_frame, (const char *)lora_state.raw_buffer,
                    sizeof(lora_state.last_raw_frame) - 1);
            lora_state.last_raw_frame[sizeof(lora_state.last_raw_frame) - 1] = '\0';
            lora_state.raw_frame_count++;
            lora_state.raw_index = 0;
        }

        // Process complete message
        if (lora_state.rx_index > 0)
        {
            lora_state.rx_buffer[lora_state.rx_index] = '\0';
            lora_state.last_rx_ms = HAL_GetTick();
            lora_state.rx_count++;

            if (s_lora_verbose) {
                printf("[LORA] Received: %s\r\n", lora_state.rx_buffer);
            }

            // Parse command
            lora_state.command_valid = 0;
            lora_state.manual_command_valid = 0;
            uint32_t stream_seq = 0;
            uint8_t has_stream_seq = 0;
            const char *cmd = lora_unwrap_stream_frame((char *)lora_state.rx_buffer, &stream_seq, &has_stream_seq);
            const char *payload = cmd;
            uint8_t chunk_marked = 0;
            uint8_t chunk_is_end = 1;

            if (has_stream_seq) {
                if (!lora_state.stream_seq_init) {
                    lora_state.stream_expected_seq = stream_seq + 1;
                    lora_state.stream_seq_init = 1;
                } else {
                    if (stream_seq != lora_state.stream_expected_seq) {
                        lora_state.stream_gap_count++;
                        if (s_lora_verbose) {
                            printf("[LORA] Stream sequence gap: got %lu expected %lu (gaps=%lu)\r\n",
                                   (unsigned long)stream_seq,
                                   (unsigned long)lora_state.stream_expected_seq,
                                   (unsigned long)lora_state.stream_gap_count);
                        }
                        lora_state.stream_len = 0;
                    }
                    lora_state.stream_expected_seq = stream_seq + 1;
                }

                if (payload && (payload[0] == 'M' || payload[0] == 'E') && payload[1] == ':') {
                    chunk_marked = 1;
                    chunk_is_end = (payload[0] == 'E') ? 1u : 0u;
                    payload += 2;
                }

                if (chunk_marked) {
                    size_t pl = strlen(payload);
                    size_t remain = (size_t)(LORA_STREAM_BUFFER_SIZE - 1 - lora_state.stream_len);
                    if (pl > remain) {
                        pl = remain;
                    }
                    if (pl > 0) {
                        memcpy(&lora_state.stream_buffer[lora_state.stream_len], payload, pl);
                        lora_state.stream_len += (uint16_t)pl;
                        lora_state.stream_buffer[lora_state.stream_len] = '\0';
                    }

                    if (!chunk_is_end) {
                        lora_state.rx_index = 0;
                        return;
                    }

                    cmd = lora_state.stream_buffer;
                    lora_state.stream_len = 0;
                } else {
                    cmd = payload;
                }
            }

            if (lora_parse_state_request(cmd, &lora_state.pending_state_request))
            {
                lora_state.command_valid = 1;
                strncpy(lora_state.last_command, cmd, sizeof(lora_state.last_command) - 1);
                lora_state.last_command[sizeof(lora_state.last_command) - 1] = '\0';
                if (s_lora_verbose) {
                    printf("[LORA] Valid command: %s -> state=%u\r\n", cmd, lora_state.pending_state_request);
                }
            }
            else if (lora_parse_manual_request(cmd, &lora_state.pending_manual_cmd))
            {
                lora_state.manual_command_valid = 1;
                strncpy(lora_state.last_command, cmd, sizeof(lora_state.last_command) - 1);
                lora_state.last_command[sizeof(lora_state.last_command) - 1] = '\0';
                if (s_lora_verbose) {
                    printf("[LORA] Valid manual command: %s -> cmd=%u\r\n", cmd, (unsigned)lora_state.pending_manual_cmd);
                }
            }
            /* [P5] Waypoint management commands (WPCLEAR / WP:<idx>:<lat>,<lon>,<salt%>,<brine%> / WPLOAD:<count>) */
            else if (strncmp(cmd, "WPCLEAR", 7) == 0)
            {
                memset(s_lora_wp_buf, 0, sizeof(s_lora_wp_buf));
                s_lora_wp_count = 0;
                Mission_Init();
                LoRA_SendRaw("ACK:WPCLEAR");
                if (s_lora_verbose) {
                    printf("[LORA] WPCLEAR: mission buffer cleared\r\n");
                }
            }
            else if (strncmp(cmd, "WP:", 3) == 0)
            {
                uint8_t idx;
                float lat, lon;
                int salt_pct, brine_pct;
                if (sscanf(cmd + 3, "%hhu:%f,%f,%d,%d", &idx, &lat, &lon, &salt_pct, &brine_pct) == 5
                    && idx < MAX_WAYPOINTS)
                {
                    if (salt_pct  < 0)   salt_pct  = 0;
                    if (salt_pct  > 100) salt_pct  = 100;
                    if (brine_pct < 0)   brine_pct = 0;
                    if (brine_pct > 100) brine_pct = 100;

                    s_lora_wp_buf[idx].latitude  = lat;
                    s_lora_wp_buf[idx].longitude = lon;
                    s_lora_wp_buf[idx].salt_rate  = (float)salt_pct  / 100.0f;
                    s_lora_wp_buf[idx].brine_rate = (float)brine_pct / 100.0f;
                    if ((uint8_t)(idx + 1) > s_lora_wp_count) {
                        s_lora_wp_count = (uint8_t)(idx + 1);
                    }
                    Mission_AddWaypoint(lat, lon, (uint8_t)salt_pct, (uint8_t)brine_pct);

                    char ack[24];
                    snprintf(ack, sizeof(ack), "ACK:WP:%hhu", idx);
                    LoRA_SendRaw(ack);
                    if (s_lora_verbose) {
                        printf("[LORA] WP[%u] lat=%.6f lon=%.6f salt=%d brine=%d\r\n",
                               idx, lat, lon, salt_pct, brine_pct);
                    }
                }
            }
            else if (strncmp(cmd, "WPLOAD:", 7) == 0)
            {
                uint8_t count;
                if (sscanf(cmd + 7, "%hhu", &count) == 1
                    && count > 0
                    && count == s_lora_wp_count)
                {
                    RobotSM_LoadMission(&g_sm, s_lora_wp_buf, s_lora_wp_count);
                    char ack[28];
                    snprintf(ack, sizeof(ack), "ACK:WPLOAD:%hhu", count);
                    LoRA_SendRaw(ack);
                    if (s_lora_verbose) {
                        printf("[LORA] WPLOAD: loaded %u waypoints into g_sm\r\n", count);
                    }
                }
            }
            else
            {
                if (s_lora_verbose) {
                    printf("[LORA] Invalid command: %s\r\n", cmd);
                }
            }

            // Reset buffer
            lora_state.rx_index = 0;
        }
        return;
    }

    // Always capture raw UART5 bytes for monitor mode
    if (lora_state.raw_index < (LORA_RX_BUFFER_SIZE - 1))
    {
        lora_state.raw_buffer[lora_state.raw_index++] = byte;
    }
    else
    {
        lora_state.raw_index = 0;
    }

    // Accept control frames that start with CMD:/stream wrappers or bare state words
    if (lora_state.rx_index == 0) {
        /* [P3] Added 'W'/'w' to allow WPCLEAR / WP: / WPLOAD: commands */
        if (byte != 'C' && byte != 'S' && byte != 'A' && byte != 'M' && byte != 'P'
            && byte != 'E' && byte != 'W'
            && byte != 'a' && byte != 'm' && byte != 'p' && byte != 'e' && byte != 'w') {
            return;
        }
    } else if (lora_state.rx_buffer[0] == 'C') {
        if (lora_state.rx_index == 1) {
            if (byte != 'M') {
                lora_state.rx_index = 0;
                return;
            }
        } else if (lora_state.rx_index == 2) {
            if (byte != 'D') {
                lora_state.rx_index = 0;
                return;
            }
        } else if (lora_state.rx_index == 3) {
            if (byte != ':') {
                lora_state.rx_index = 0;
                return;
            }
        }
    }

    // Add byte to buffer
    if (lora_state.rx_index < (LORA_RX_BUFFER_SIZE - 1))
    {
        lora_state.rx_buffer[lora_state.rx_index++] = byte;
    }
    else
    {
        // Buffer overflow - reset
        if (s_lora_verbose) {
            printf("[LORA] RX buffer overflow, resetting\r\n");
        }
        lora_state.rx_index = 0;
    }
}

// Periodic update (call from main loop)
void LoRA_Tick(uint32_t now_ms)
{
    // Optional: Check for timeout on incomplete messages
    if (lora_state.rx_index > 0)
    {
        if ((now_ms - lora_state.last_rx_ms) > LORA_COMMAND_TIMEOUT_MS)
        {
            if (s_lora_verbose) {
                printf("[LORA] RX timeout, discarding incomplete message\r\n");
            }
            lora_state.rx_index = 0;
        }
    }

    // Report health status to system health monitor
    // Consider LoRa healthy if any RX/TX activity has happened recently.
    uint32_t last_activity_ms = lora_state.last_tx_ms;
    if (lora_state.last_rx_ms > last_activity_ms) {
        last_activity_ms = lora_state.last_rx_ms;
    }

    if (last_activity_ms == 0) {
        SystemHealth_SetSensorStatus(SENSOR_LORA, SENSOR_TIMEOUT);
    } else if ((now_ms - last_activity_ms) > 10000) {
        SystemHealth_SetSensorStatus(SENSOR_LORA, SENSOR_TIMEOUT);
    } else {
        SystemHealth_SetSensorStatus(SENSOR_LORA, SENSOR_OK);
    }
}

// Send state diagnostics to base station (JSON format)
void LoRA_SendState(uint8_t state, float gps_lat, float gps_lon,
                    int motor_m1, int motor_m2)
{
    if (!lora_state.huart) return;

    char state_name[16] = "UNKNOWN";
    switch (state)
    {
        case 0: strcpy(state_name, "MANUAL"); break;
        case 1: strcpy(state_name, "AUTO");   break;
        case 2: strcpy(state_name, "PAUSE");  break;
        case 3: strcpy(state_name, "ERROR");  break;
        case 4: strcpy(state_name, "ESTOP");  break;
    }

    // JSON format: {"state":"MODE","gps":{"lat":0.0,"lon":0.0},"motor":{"m1":0,"m2":0}}
    char msg[160];
    snprintf(msg, sizeof(msg),
             "{\"state\":\"%s\",\"gps\":{\"lat\":%.4f,\"lon\":%.4f},\"motor\":{\"m1\":%d,\"m2\":%d}}\r\n",
             state_name, gps_lat, gps_lon, motor_m1, motor_m2);

    lora_uart5_send(msg, "state");
}

// Send comprehensive telemetry to base station (JSON format)
void LoRA_SendTelemetry(uint8_t state, float gps_lat, float gps_lon, uint8_t gps_has_fix,
                        uint8_t gps_num_sat, float gps_hdop,
                        int motor_m1, int motor_m2, float yaw_deg, float pitch_deg,
                        uint8_t salt_rate, uint8_t brine_rate, float temp_c,
                        uint16_t prox_left_cm, uint16_t prox_right_cm)
{
    if (!lora_state.huart) return;

    char state_name[16] = "UNKNOWN";
    switch (state)
    {
        case 0: strcpy(state_name, "MANUAL"); break;
        case 1: strcpy(state_name, "AUTO");   break;
        case 2: strcpy(state_name, "PAUSE");  break;
        case 3: strcpy(state_name, "ERROR");  break;
        case 4: strcpy(state_name, "ESTOP");  break;
    }

    char prox_left_json[20];
    char prox_right_json[20];
    if (prox_left_cm == 65535u) {
        strcpy(prox_left_json, "\"NO_DETECT\"");
    } else {
        snprintf(prox_left_json, sizeof(prox_left_json), "%u", prox_left_cm);
    }
    if (prox_right_cm == 65535u) {
        strcpy(prox_right_json, "\"NO_DETECT\"");
    } else {
        snprintf(prox_right_json, sizeof(prox_right_json), "%u", prox_right_cm);
    }

    // JSON format with proximity sensors
    char msg[320];
    snprintf(msg, sizeof(msg),
             "{\"state\":\"%s\",\"gps\":{\"lat\":%.4f,\"lon\":%.4f,\"fix\":%u,\"sat\":%u,\"hdop\":%.1f},"
             "\"motor\":{\"m1\":%d,\"m2\":%d},\"heading\":{\"yaw\":%.1f,\"pitch\":%.1f},"
             "\"disp\":{\"salt\":%u,\"brine\":%u},\"temp\":%.1f,"
             "\"prox\":{\"left\":%s,\"right\":%s}}\r\n",
             state_name, gps_lat, gps_lon, gps_has_fix, gps_num_sat, gps_hdop,
             motor_m1, motor_m2, yaw_deg, pitch_deg,
             salt_rate, brine_rate, temp_c,
             prox_left_json, prox_right_json);

    lora_uart5_send(msg, "telemetry");
}

// Send fault information to base station (JSON format)
void LoRA_SendFault(uint8_t fault_code, uint8_t action)
{
    if (!lora_state.huart) return;

    const char *fault_name = "UNKNOWN";
    switch (fault_code)
    {
        /* [P6] Fixed to match FaultCode_t in robot_sm.h */
        case 0: fault_name = "NONE";            break;
        case 1: fault_name = "IMU_TIMEOUT";     break;
        case 2: fault_name = "GPS_LOSS";        break;
        case 3: fault_name = "MOTOR_FEEDBACK";  break;  /* was PROXIMITY_WARN */
        case 4: fault_name = "BATTERY_COLD";    break;  /* was PROXIMITY_CRIT */
        case 5: fault_name = "PROXIMITY_WARN";  break;  /* was BATTERY_COLD   */
        case 6: fault_name = "PROXIMITY_CRIT";  break;  /* was DISPERSION_CLOG */
        case 7: fault_name = "DISPERSION_CLOG"; break;  /* was GENERIC        */
        default: fault_name = "GENERIC";        break;  /* replaces case 8 RESERVED */
    }

    const char *action_name = "UNKNOWN";
    switch (action)
    {
        case 0: action_name = "PAUSE";    break;
        case 1: action_name = "ESTOP";    break;
        case 2: action_name = "LOG_ONLY"; break;
    }

    // JSON format: {"fault":"IMU_TIMEOUT","action":"ESTOP"}
    char msg[96];
    snprintf(msg, sizeof(msg), "{\"fault\":\"%s\",\"action\":\"%s\"}\r\n", fault_name, action_name);

    lora_uart5_send(msg, "fault");
}

// ============================================================================
// Command accessors
// ============================================================================

uint8_t LoRA_GetPendingCommand(uint8_t *out_state)
{
    if (!out_state) return 0;

    if (lora_state.command_valid)
    {
        *out_state = lora_state.pending_state_request;
        lora_state.command_valid = 0;
        return 1;
    }

    return 0;
}

uint8_t LoRA_GetPendingManualCommand(LoRA_ManualCommand_t *out_cmd)
{
    if (!out_cmd) return 0;

    if (lora_state.manual_command_valid)
    {
        *out_cmd = lora_state.pending_manual_cmd;
        lora_state.manual_command_valid = 0;
        return 1;
    }

    return 0;
}

const char* LoRA_GetLastCommand(void)
{
    return lora_state.last_command;
}

// ============================================================================
// Raw frame / TX payload accessors
// ============================================================================

const char* LoRA_GetLastRawFrame(void)
{
    return lora_state.last_raw_frame;
}

uint32_t LoRA_GetRawFrameCount(void)
{
    return lora_state.raw_frame_count;
}

const char* LoRA_GetLastTxPayload(void)
{
    return lora_state.last_tx_payload;
}

// Send raw test string over LoRa UART
void LoRA_SendRaw(const char *text)
{
    if (!lora_state.huart || !text || text[0] == '\0') return;

    char msg[192];
    size_t in_len = strlen(text);

    if (in_len >= sizeof(msg) - 3) {
        in_len = sizeof(msg) - 3;
    }

    memcpy(msg, text, in_len);
    msg[in_len] = '\0';

    // Ensure CRLF line ending for ESP32 line parser compatibility
    if (in_len == 0 || msg[in_len - 1] != '\n') {
        msg[in_len++] = '\r';
        msg[in_len++] = '\n';
        msg[in_len] = '\0';
    }

    HAL_UART_Transmit(lora_state.huart, (uint8_t *)msg, in_len, HAL_MAX_DELAY);
    strncpy(lora_state.last_tx_payload, msg, sizeof(lora_state.last_tx_payload) - 1);
    lora_state.last_tx_payload[sizeof(lora_state.last_tx_payload) - 1] = '\0';
    lora_state.last_tx_ms = HAL_GetTick();
    lora_state.tx_count++;
    if (s_lora_verbose) {
        printf("[LORA] Raw TX: %s", msg);
    }
}

// ============================================================================
// Diagnostics / metrics
// ============================================================================

void LoRA_SetVerbose(uint8_t enable)
{
    s_lora_verbose = enable ? 1u : 0u;
}

uint32_t LoRA_GetLastTxMs(void)
{
    return lora_state.last_tx_ms;
}

uint32_t LoRA_GetLastRxMs(void)
{
    return lora_state.last_rx_ms;
}

uint32_t LoRA_GetTxCount(void)
{
    return lora_state.tx_count;
}

uint32_t LoRA_GetRxCount(void)
{
    return lora_state.rx_count;
}
