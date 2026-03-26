/**
 * salt_base_station.c — Phase D  (jakep2377/base_station)
 *
 * Drop-in replacement for main/salt_base_station.c.
 *
 * Phase D changes vs. original:
 *  1. LORA_QUEUE_DEPTH 64  (was hardcoded 8 in app_main)
 *     Queue must hold a full WP push: 1 WPCLEAR + 50 WP + 1 WPLOAD = 52 cmds.
 *     Depth 64 gives comfortable headroom.
 *
 *  2. status_json[512]  (was 256)
 *     Full telemetry JSON incl. last_lora is ~300 bytes; 256 caused truncation.
 *
 *  3. last_lora_rx[256] static buffer
 *     Stores the last raw LoRa RX string verbatim, separate from status_json,
 *     so the /last_lora endpoint can serve it without parsing the status blob.
 *
 *  4. GET /last_lora  new endpoint
 *     Returns last_lora_rx as text/plain.  Lets robot-lora-server poll raw
 *     LoRa frames (ACKs, telemetry) without needing to parse status JSON.
 *
 *  5. POST /command: blocking queue send (100 ms timeout)
 *     Returns HTTP 503 "queue_full" when queue is saturated instead of
 *     silently dropping.  robot-lora-server lora_bridge.js detects this and
 *     retries with back-off so no WP commands are lost under load.
 *
 *  6. lora_tx_task: 3-retry on LoRaSend failure
 *     Each retry waits 50 ms, giving the radio time to reset its state.
 *     Logs an error only after all 3 attempts fail.
 *
 *  7. /status JSON includes queue_depth
 *     Operators and the server can see how full the queue is at a glance.
 *
 *  8. lora_rx_task: saves last_lora_rx and embeds it in /status
 *     rx[n] is null-terminated before snprintf to prevent buffer overread.
 */

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "esp_http_server.h"

// LoRa component
#include "ra01s.h"

// ---------------- Wi-Fi Credentials ----------------
#define STA_SSID "42Fortress"
#define STA_PASS "sweetdoll684"

#define AP_SSID  "SaltRobot_Base"
#define AP_PASS  "saltrobot123"

// ---------------- LoRa Settings ----------------
#define LORA_FREQ_HZ      915000000
#define LORA_TX_POWER_DBM 22
#define LORA_TCXO_VOLT    3.3f
#define LORA_USE_LDO      true

#define LORA_SF          7
#define LORA_BW          4
#define LORA_CR          1
#define LORA_PREAMBLE    8
#define LORA_PAYLOAD_LEN 0
#define LORA_CRC_ON      true
#define LORA_INVERT_IRQ  false

// Limit command size over LoRa
#define LORA_CMD_MAX     200

// Phase D: queue depth large enough for a full WP push
// (1 WPCLEAR + MAX_WAYPOINTS WP lines + 1 WPLOAD = 52 minimum)
#define LORA_QUEUE_DEPTH 64

static const char *TAG = "BASE";

// ---------------- Wi-Fi Event Group ----------------
static EventGroupHandle_t wifi_event_group;
static const int WIFI_GOT_IP_BIT = BIT0;

// ---------------- Shared State ----------------
static SemaphoreHandle_t status_lock;

// Phase D: 512-byte status_json (full telemetry incl. last_lora is ~300 bytes)
static char status_json[512] = "{\"battery\":85,\"state\":\"IDLE\",\"mode\":\"BOOT\"}";
static char last_cmd[192]    = "none";

// Phase D: separate verbatim store for last LoRa RX, served via GET /last_lora
static char last_lora_rx[256] = "";
static uint32_t lora_tx_ok_count = 0;
static uint32_t lora_tx_fail_count = 0;
static uint32_t lora_tx_fail_streak = 0;
static uint32_t lora_last_tx_ok_ms = 0;
static uint32_t lora_last_tx_fail_ms = 0;

// ---------------- LoRa Queue ----------------
typedef struct {
    uint16_t len;
    char payload[LORA_CMD_MAX];
} lora_cmd_t;

static QueueHandle_t lora_cmd_q;

// ---------------- HTTP Handlers ----------------

static esp_err_t status_get_handler(httpd_req_t *req) {
    xSemaphoreTake(status_lock, portMAX_DELAY);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, status_json, HTTPD_RESP_USE_STRLEN);
    xSemaphoreGive(status_lock);
    return ESP_OK;
}

static esp_err_t health_get_handler(httpd_req_t *req) {
    int queue_depth = (int)uxQueueMessagesWaiting(lora_cmd_q);
    bool degraded = lora_tx_fail_streak >= 3;

    char health_json[256];
    snprintf(health_json, sizeof(health_json),
             "{\"ok\":%s,\"bridge_degraded\":%s,\"queue_depth\":%d,"
             "\"lora_tx_ok\":%lu,\"lora_tx_fail\":%lu,\"lora_tx_fail_streak\":%lu}",
             degraded ? "false" : "true",
             degraded ? "true" : "false",
             queue_depth,
             (unsigned long)lora_tx_ok_count,
             (unsigned long)lora_tx_fail_count,
             (unsigned long)lora_tx_fail_streak);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, health_json, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Phase D: GET /last_lora — returns raw last LoRa RX as text/plain
static esp_err_t last_lora_get_handler(httpd_req_t *req) {
    xSemaphoreTake(status_lock, portMAX_DELAY);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, last_lora_rx, HTTPD_RESP_USE_STRLEN);
    xSemaphoreGive(status_lock);
    return ESP_OK;
}

static esp_err_t command_post_handler(httpd_req_t *req) {
    const int max_body = (int)sizeof(last_cmd) - 1;
    int len = req->content_len;

    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }
    if (len > max_body) {
        httpd_resp_send_err(req, HTTPD_413_CONTENT_TOO_LARGE, "Body too large");
        return ESP_FAIL;
    }

    // Read the body into last_cmd
    int received = 0;
    while (received < len) {
        int r = httpd_req_recv(req, last_cmd + received, len - received);
        if (r <= 0) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Body recv failed");
            return ESP_FAIL;
        }
        received += r;
    }
    last_cmd[received] = '\0';

    ESP_LOGI(TAG, "Received /command: %s", last_cmd);

    // Phase D: queue command with 100 ms wait; return 503 if queue is full
    // (was: xQueueSend(..., 0) — silently dropped on full queue)
    if (lora_cmd_q) {
        lora_cmd_t c = {0};
        snprintf(c.payload, sizeof(c.payload), "%s", last_cmd);
        c.len = (uint16_t)strnlen(c.payload, sizeof(c.payload));

        if (xQueueSend(lora_cmd_q, &c, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGW(TAG, "LoRa cmd queue full; returning 503");
            httpd_resp_set_status(req, "503 Service Unavailable");
            httpd_resp_sendstr(req, "queue_full");
            return ESP_OK;
        }
    }

    // Phase D: include live queue depth in status JSON
    xSemaphoreTake(status_lock, portMAX_DELAY);
    snprintf(status_json, sizeof(status_json),
             "{\"battery\":85,\"state\":\"CMD_QUEUED\",\"mode\":\"HTTP\","
             "\"last_cmd\":\"%.120s\",\"queue_depth\":%d,"
             "\"lora_tx_ok\":%lu,\"lora_tx_fail\":%lu,\"lora_tx_fail_streak\":%lu}",
             last_cmd,
             (int)uxQueueMessagesWaiting(lora_cmd_q),
             (unsigned long)lora_tx_ok_count,
             (unsigned long)lora_tx_fail_count,
             (unsigned long)lora_tx_fail_streak);
    xSemaphoreGive(status_lock);

    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static httpd_handle_t start_http_server(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t status_uri = {
            .uri     = "/status",
            .method  = HTTP_GET,
            .handler = status_get_handler
        };
        httpd_uri_t cmd_uri = {
            .uri     = "/command",
            .method  = HTTP_POST,
            .handler = command_post_handler
        };
        httpd_uri_t health_uri = {
            .uri     = "/health",
            .method  = HTTP_GET,
            .handler = health_get_handler
        };
        // Phase D: new endpoint for raw LoRa RX polling
        httpd_uri_t last_lora_uri = {
            .uri     = "/last_lora",
            .method  = HTTP_GET,
            .handler = last_lora_get_handler
        };

        httpd_register_uri_handler(server, &status_uri);
        httpd_register_uri_handler(server, &cmd_uri);
        httpd_register_uri_handler(server, &health_uri);
        httpd_register_uri_handler(server, &last_lora_uri);

        ESP_LOGI(TAG, "HTTP server: GET /status, GET /health, POST /command, GET /last_lora");
    }
    return server;
}

// ---------------- Wi-Fi Events ----------------

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    (void)arg;
    (void)data;

    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "STA disconnected, retrying...");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_event_group, WIFI_GOT_IP_BIT);
    }
}

// ---------------- STA then AP fallback ----------------

static void start_sta_then_fallback_ap(void) {
    wifi_event_group = xEventGroupCreate();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    // Try STA first
    esp_netif_create_default_wifi_sta();

    wifi_config_t sta_cfg = {0};
    strncpy((char*)sta_cfg.sta.ssid,     STA_SSID, sizeof(sta_cfg.sta.ssid));
    strncpy((char*)sta_cfg.sta.password, STA_PASS,  sizeof(sta_cfg.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Trying STA connect...");
    EventBits_t bits = xEventGroupWaitBits(
        wifi_event_group, WIFI_GOT_IP_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(12000));

    if (bits & WIFI_GOT_IP_BIT) {
        ESP_LOGI(TAG, "STA connected (got IP).");
        xSemaphoreTake(status_lock, portMAX_DELAY);
        snprintf(status_json, sizeof(status_json),
                 "{\"battery\":85,\"state\":\"IDLE\",\"mode\":\"STA\"}");
        xSemaphoreGive(status_lock);
        return;
    }

    // Fallback AP mode
    ESP_LOGW(TAG, "STA failed. Starting AP mode...");
    ESP_ERROR_CHECK(esp_wifi_stop());

    esp_netif_create_default_wifi_ap();

    wifi_config_t ap_cfg = {0};
    strncpy((char*)ap_cfg.ap.ssid,     AP_SSID, sizeof(ap_cfg.ap.ssid));
    strncpy((char*)ap_cfg.ap.password, AP_PASS,  sizeof(ap_cfg.ap.password));
    ap_cfg.ap.ssid_len       = (uint8_t)strlen(AP_SSID);
    ap_cfg.ap.max_connection = 4;
    ap_cfg.ap.authmode       = WIFI_AUTH_WPA2_PSK;
    if (strlen(AP_PASS) == 0) ap_cfg.ap.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    xSemaphoreTake(status_lock, portMAX_DELAY);
    snprintf(status_json, sizeof(status_json),
             "{\"battery\":85,\"state\":\"IDLE\",\"mode\":\"AP\"}");
    xSemaphoreGive(status_lock);

    ESP_LOGI(TAG, "AP started. SSID=%s (AP IP is usually 192.168.4.1)", AP_SSID);
}

// ---------------- LoRa ----------------

static void lora_init(void) {
    ESP_LOGI(TAG, "Initializing LoRa...");
    LoRaInit();

    int rc = LoRaBegin(LORA_FREQ_HZ, LORA_TX_POWER_DBM, LORA_TCXO_VOLT, LORA_USE_LDO);
    if (rc != 0) {
        ESP_LOGE(TAG, "LoRaBegin failed (%d). Check pins/TCXO/freq.", rc);
        return;
    }

    LoRaConfig(LORA_SF, LORA_BW, LORA_CR,
               LORA_PREAMBLE, LORA_PAYLOAD_LEN, LORA_CRC_ON, LORA_INVERT_IRQ);

    ESP_LOGI(TAG, "LoRa ready @ %d Hz", (int)LORA_FREQ_HZ);
}

// Phase D: TX task with up to 3 retries on LoRaSend failure
static void lora_tx_task(void *arg) {
    (void)arg;
    lora_cmd_t c;

    while (1) {
        if (xQueueReceive(lora_cmd_q, &c, portMAX_DELAY) == pdTRUE) {
            if (c.len > 0) {
                ESP_LOGI(TAG, "LoRa TX: %.*s", c.len, c.payload);

                bool sent = false;
                for (int retry = 0; retry < 3; retry++) {
                    if (LoRaSend((uint8_t*)c.payload, c.len, SX126x_TXMODE_SYNC)) {
                        sent = true;
                        break;
                    }
                    ESP_LOGW(TAG, "LoRaSend retry %d/3 for: %.*s", retry + 1, c.len, c.payload);
                    vTaskDelay(pdMS_TO_TICKS(50));
                }

                if (!sent) {
                    lora_tx_fail_count++;
                    lora_tx_fail_streak++;
                    lora_last_tx_fail_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
                    ESP_LOGE(TAG, "LoRaSend failed after 3 retries: %.*s", c.len, c.payload);
                    xSemaphoreTake(status_lock, portMAX_DELAY);
                    snprintf(status_json, sizeof(status_json),
                             "{\"battery\":85,\"state\":\"LORA_TX_FAIL\",\"mode\":\"LORA\","
                             "\"queue_depth\":%d,\"last_cmd\":\"%.120s\","
                             "\"lora_tx_ok\":%lu,\"lora_tx_fail\":%lu,\"lora_tx_fail_streak\":%lu,"
                             "\"lora_last_tx_ok_ms\":%lu,\"lora_last_tx_fail_ms\":%lu,"
                             "\"bridge_degraded\":%s}",
                             (int)uxQueueMessagesWaiting(lora_cmd_q),
                             last_cmd,
                             (unsigned long)lora_tx_ok_count,
                             (unsigned long)lora_tx_fail_count,
                             (unsigned long)lora_tx_fail_streak,
                             (unsigned long)lora_last_tx_ok_ms,
                             (unsigned long)lora_last_tx_fail_ms,
                             (lora_tx_fail_streak >= 3) ? "true" : "false");
                    xSemaphoreGive(status_lock);
                } else {
                    lora_tx_ok_count++;
                    lora_tx_fail_streak = 0;
                    lora_last_tx_ok_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
                }
            }
        }
    }
}

// Phase D: RX task — saves last_lora_rx and includes queue_depth in status
static void lora_rx_task(void *arg) {
    (void)arg;
    // rx[255] leaves rx[254] as the last usable byte; null-terminate at rx[n]
    uint8_t rx[255];

    while (1) {
        uint8_t n = LoRaReceive(rx, sizeof(rx) - 1);
        if (n > 0) {
            rx[n] = '\0';  // safe: sizeof(rx)-1 bytes were the max read

            xSemaphoreTake(status_lock, portMAX_DELAY);

            // Phase D: store verbatim for /last_lora
            snprintf(last_lora_rx, sizeof(last_lora_rx), "%s", (char*)rx);

            // Phase D: include queue depth in /status JSON
            snprintf(status_json, sizeof(status_json),
                     "{\"battery\":85,\"state\":\"IDLE\",\"mode\":\"LORA\","
                     "\"queue_depth\":%d,\"last_lora\":\"%.200s\","
                     "\"lora_tx_ok\":%lu,\"lora_tx_fail\":%lu,\"lora_tx_fail_streak\":%lu,"
                     "\"lora_last_tx_ok_ms\":%lu,\"lora_last_tx_fail_ms\":%lu,"
                     "\"bridge_degraded\":%s}",
                     (int)uxQueueMessagesWaiting(lora_cmd_q),
                     last_lora_rx,
                     (unsigned long)lora_tx_ok_count,
                     (unsigned long)lora_tx_fail_count,
                     (unsigned long)lora_tx_fail_streak,
                     (unsigned long)lora_last_tx_ok_ms,
                     (unsigned long)lora_last_tx_fail_ms,
                     (lora_tx_fail_streak >= 3) ? "true" : "false");

            xSemaphoreGive(status_lock);

            ESP_LOGI(TAG, "LoRa RX (%d): %s", n, (char*)rx);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ---------------- app_main ----------------

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    status_lock = xSemaphoreCreateMutex();

    start_sta_then_fallback_ap();
    start_http_server();

    // Phase D: use LORA_QUEUE_DEPTH constant (was hardcoded 8)
    lora_cmd_q = xQueueCreate(LORA_QUEUE_DEPTH, sizeof(lora_cmd_t));
    lora_init();
    xTaskCreate(lora_tx_task, "lora_tx", 4096, NULL, 5, NULL);
    xTaskCreate(lora_rx_task, "lora_rx", 4096, NULL, 5, NULL);
}

// test: curl -X POST http://192.168.4.1/command -H "Content-Type: text/plain" -d "forward"
// test: curl http://192.168.4.1/status
// test: curl http://192.168.4.1/last_lora
