/*
 * Cantonese speech-to-text via the self-hosted voice-proxy (FastAPI +
 * MiniMax ASR). See muse_stt_voiceproxy.h.
 *
 * Concurrency model:
 *   - s_lock serialises requests (one in flight at a time).
 *   - The caller (voice task) hands over pcm/frames, notifies the worker,
 *     and waits on s_done up to its timeout. On timeout it abandons the
 *     request: the worker keeps going and later stores its result into the
 *     module-owned s_result, never into caller memory.
 *   - Results are tagged with a sequence id so a late give from an
 *     abandoned request can never be mistaken for the current one.
 *   - Prewarms and transcribes share the worker; a s_warm flag tells them
 *     apart (prewarm is set first and always wins the check).
 */
#include "muse_stt_voiceproxy.h"

#include <stdio.h>
#include <string.h>

#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "muse_stt_vp";

#ifndef CONFIG_MUSE_TTS_VOICEPROXY_URL
#define CONFIG_MUSE_TTS_VOICEPROXY_URL ""
#endif
#ifndef CONFIG_MUSE_TTS_VOICEPROXY_TOKEN
#define CONFIG_MUSE_TTS_VOICEPROXY_TOKEN ""
#endif

#define STT_URL         CONFIG_MUSE_TTS_VOICEPROXY_URL "/v1/stt"
#define HEALTH_URL      CONFIG_MUSE_TTS_VOICEPROXY_URL "/v1/health"
#define HTTP_TIMEOUT_MS 25000          /* the proxy's own upstream timeout is 30 s */
#define RESP_MAX        1024           /* enough for {"text": "...", ...} */
#define WRITE_CHUNK     16384

/* -------------------------------------------------------------------------- */
/*  Module state                                                              */
/* -------------------------------------------------------------------------- */

static SemaphoreHandle_t s_lock;      /* one request at a time */
static SemaphoreHandle_t s_done;      /* worker -> caller */
static TaskHandle_t s_task;
static bool s_up;

/* request handoff (caller writes under s_lock, worker snapshots on wake) */
static const int16_t *s_pcm;
static size_t s_frames;
static uint32_t s_req_id;             /* incremented per request */
static volatile uint32_t s_resp_id;   /* the id whose result s_result holds */
static volatile bool s_ok;
static char s_result[512];            /* module-owned: safe to fill after a caller times out */

/* worker-side HTTP state */
static esp_http_client_handle_t s_client;
static char *s_auth;
static int64_t s_last_use_us;
static volatile bool s_warm;

bool muse_stt_voiceproxy_enabled(void)
{
    return CONFIG_MUSE_TTS_VOICEPROXY_URL[0] != '\0' &&
           CONFIG_MUSE_TTS_VOICEPROXY_TOKEN[0] != '\0';
}

/* -------------------------------------------------------------------------- */
/*  Worker-side HTTP                                                          */
/* -------------------------------------------------------------------------- */

static bool client_ready(void)
{
    if (s_client) {
        return true;
    }
    const esp_http_client_config_t cfg = {
        .url               = STT_URL,
        .timeout_ms        = HTTP_TIMEOUT_MS,
        .buffer_size       = 1024,
        .buffer_size_tx    = 1024,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .keep_alive_enable = true,
    };
    s_client = esp_http_client_init(&cfg);
    if (!s_client) {
        return false;
    }
    esp_http_client_set_header(s_client, "Authorization", s_auth);
    return true;
}

/* Reads the response body to the end (storing up to cap-1 bytes), so the
 * keep-alive connection can be reused. Returns the stored length, or -1. */
static int read_body(char *resp, size_t cap)
{
    size_t stored = 0;
    char junk[256];
    for (;;) {
        char *dst = stored + sizeof(junk) <= cap ? resp + stored : junk;
        size_t room = stored + sizeof(junk) <= cap ? cap - stored : sizeof(junk);
        int n = esp_http_client_read(s_client, dst, room);
        if (n <= 0) {
            break;
        }
        if (dst != junk) {
            stored += n;
        }
    }
    if (cap) {
        resp[stored] = '\0';
    }
    return (int)stored;
}

/* One POST /v1/stt attempt. Returns false before a response arrived. */
static bool post_attempt(const int16_t *pcm, size_t frames, char *resp, size_t cap, int *resp_len)
{
    const char *boundary = "----MuseSttVpBoundary7MA4YWxkTrZu0gW";
    char head[256], tail[256];
    int head_len = snprintf(head, sizeof head,
        "--%s\r\n"
        "Content-Disposition: form-data; name=\"audio\"; filename=\"speech.pcm\"\r\n"
        "Content-Type: audio/L16; rate=16000; channels=1\r\n"
        "\r\n", boundary);
    int tail_len = snprintf(tail, sizeof tail,
        "\r\n--%s\r\n"
        "Content-Disposition: form-data; name=\"lang\"\r\n"
        "\r\n"
        "yue\r\n--%s--\r\n", boundary, boundary);
    if (head_len < 0 || tail_len < 0) {
        return false;
    }
    size_t pcm_bytes = frames * sizeof(int16_t);
    if (esp_http_client_open(s_client, head_len + (int)pcm_bytes + tail_len) != ESP_OK) {
        return false;
    }
    bool ok = esp_http_client_write(s_client, head, head_len) == head_len;
    const char *p = (const char *)pcm;
    for (size_t left = pcm_bytes; ok && left;) {
        size_t n = left < WRITE_CHUNK ? left : WRITE_CHUNK;
        ok = esp_http_client_write(s_client, p, n) == (int)n;
        p += n;
        left -= n;
    }
    if (ok) {
        ok = esp_http_client_write(s_client, tail, tail_len) == tail_len;
    }
    if (!ok) {
        esp_http_client_close(s_client);
        return false;
    }
    esp_http_client_fetch_headers(s_client);
    int status = esp_http_client_get_status_code(s_client);
    int n = read_body(resp, cap);
    *resp_len = n < 0 ? 0 : n;
    if (status != 200) {
        ESP_LOGW(TAG, "STT HTTP %d: %.120s", status, resp);
        return true;   /* a response arrived; the caller reads the status via resp_len==0 path */
    }
    return true;
}

static bool post_pcm(const int16_t *pcm, size_t frames, char *resp, size_t cap, int *resp_len)
{
    if (!client_ready()) {
        return false;
    }
    esp_http_client_set_url(s_client, STT_URL);
    esp_http_client_set_method(s_client, HTTP_METHOD_POST);
    bool reused = s_last_use_us && esp_timer_get_time() - s_last_use_us < 60 * 1000000LL;
    if (!post_attempt(pcm, frames, resp, cap, resp_len)) {
        if (reused) {
            ESP_LOGI(TAG, "kept connection had closed; reconnecting");
            esp_http_client_close(s_client);
            s_last_use_us = 0;
            if (!post_attempt(pcm, frames, resp, cap, resp_len)) {
                esp_http_client_close(s_client);
                return false;
            }
        } else {
            esp_http_client_close(s_client);
            return false;
        }
    }
    s_last_use_us = esp_timer_get_time();
    return true;
}

/* GET /v1/health, unless the connection carried something recently. */
static void do_prewarm(void)
{
    if (s_last_use_us && esp_timer_get_time() - s_last_use_us < 30 * 1000000LL) {
        return;
    }
    if (!client_ready()) {
        return;
    }
    int64_t t0 = esp_timer_get_time();
    esp_http_client_set_url(s_client, HEALTH_URL);
    esp_http_client_set_method(s_client, HTTP_METHOD_GET);
    if (esp_http_client_open(s_client, 0) != ESP_OK) {
        esp_http_client_close(s_client);
        s_last_use_us = 0;
        return;
    }
    esp_http_client_fetch_headers(s_client);
    char junk[128];
    while (esp_http_client_read(s_client, junk, sizeof junk) > 0) {
    }
    int status = esp_http_client_get_status_code(s_client);
    if (status == 200) {
        s_last_use_us = esp_timer_get_time();
    } else {
        esp_http_client_close(s_client);
        s_last_use_us = 0;
    }
    ESP_LOGI(TAG, "warm-up: %s in %d ms", status == 200 ? "connected" : "no answer",
             (int)((esp_timer_get_time() - t0) / 1000));
}

static void do_transcribe(void)
{
    const int16_t *pcm = s_pcm;
    size_t frames = s_frames;
    uint32_t id = s_req_id;
    char resp[RESP_MAX];
    int rn = 0;
    bool ok = false;
    int64_t t0 = esp_timer_get_time();
    if (pcm && frames && post_pcm(pcm, frames, resp, sizeof resp, &rn) && rn > 0) {
        cJSON *root = cJSON_ParseWithLength(resp, rn);
        cJSON *text = root ? cJSON_GetObjectItemCaseSensitive(root, "text") : NULL;
        if (cJSON_IsString(text) && text->valuestring) {
            strlcpy(s_result, text->valuestring, sizeof s_result);
            ok = s_result[0] != '\0';
        }
        cJSON_Delete(root);
    }
    s_ok = ok;
    s_resp_id = id;
    if (ok) {
        ESP_LOGI(TAG, "transcript in %d ms: \"%s\"",
                 (int)((esp_timer_get_time() - t0) / 1000), s_result);
    } else {
        ESP_LOGW(TAG, "transcribe failed after %d ms",
                 (int)((esp_timer_get_time() - t0) / 1000));
    }
}

static void worker(void *arg)
{
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (s_warm) {
            s_warm = false;
            do_prewarm();
        } else if (s_pcm && s_frames) {
            do_transcribe();
            xSemaphoreGive(s_done);
        }
    }
}

/* -------------------------------------------------------------------------- */
/*  Public API                                                                */
/* -------------------------------------------------------------------------- */

static bool ensure_started(void)
{
    if (s_up) {
        return true;
    }
    if (!muse_stt_voiceproxy_enabled()) {
        return false;
    }
    s_lock = xSemaphoreCreateMutex();
    s_done = xSemaphoreCreateBinary();
    size_t auth_len = strlen(CONFIG_MUSE_TTS_VOICEPROXY_TOKEN) + 8;
    s_auth = heap_caps_malloc(auth_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_auth) {
        snprintf(s_auth, auth_len, "Bearer %s", CONFIG_MUSE_TTS_VOICEPROXY_TOKEN);
    }
    if (!s_lock || !s_done || !s_auth ||
        xTaskCreatePinnedToCoreWithCaps(worker, "muse_stt_vp", 12 * 1024, NULL, 4, &s_task, 0,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
        ESP_LOGE(TAG, "start failed");
        return false;
    }
    s_up = true;
    ESP_LOGI(TAG, "STT on: voice-proxy @ %s", CONFIG_MUSE_TTS_VOICEPROXY_URL);
    return true;
}

void muse_stt_voiceproxy_prewarm(void)
{
    if (!ensure_started()) {
        return;
    }
    s_warm = true;
    xTaskNotifyGive(s_task);
}

bool muse_stt_voiceproxy_transcribe(const int16_t *pcm, size_t frames,
                                    char *out, size_t cap, uint32_t timeout_ms)
{
    if (!out || cap == 0 || !pcm || !frames) {
        return false;
    }
    out[0] = '\0';
    if (!ensure_started()) {
        return false;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    /* Drop a stale give left by a request that timed out earlier. */
    while (xSemaphoreTake(s_done, 0) == pdTRUE) {
    }
    s_pcm = pcm;
    s_frames = frames;
    uint32_t id = ++s_req_id;
    xTaskNotifyGive(s_task);
    bool ok = false;
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    for (;;) {
        int64_t left_us = deadline - esp_timer_get_time();
        if (left_us <= 0) {
            break;
        }
        if (xSemaphoreTake(s_done, pdMS_TO_TICKS(left_us / 1000 + 1)) != pdTRUE) {
            break;
        }
        if (s_resp_id == id) {
            ok = s_ok;
            break;
        }
        /* A late give from an abandoned request: keep waiting. */
    }
    s_pcm = NULL;
    s_frames = 0;
    xSemaphoreGive(s_lock);
    if (ok) {
        strlcpy(out, s_result, cap);
        char *t = out;
        while (*t == ' ') {
            t++;
        }
        if (t != out) {
            memmove(out, t, strlen(t) + 1);
        }
        ok = out[0] != '\0';
    }
    return ok;
}
