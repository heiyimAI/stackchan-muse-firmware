/*
 * Cantonese TTS via self-hosted voice-proxy (FastAPI + MiniMax T2A v2).
 *
 * Modeled after JJenglert1/esp32-muse-agent's muse_tts_openai.c
 * (Apache 2.0). Pointed at our backend instead of OpenAI. Off
 * unless both URL and TOKEN are set via menuconfig.
 *
 * Architecture:
 *   muse_chat_session.cpp start_tts() hook
 *      -> muse_tts_voiceproxy_request(id, text)        [non-blocking]
 *   FreeRTOS task fetches MP3 over HTTPS (PSRAM stream buffer)
 *      -> tts_pump() in muse_chat_session.cpp polls
 *         muse_tts_voiceproxy_read() and feeds tts_data()
 *   minimp3 decode in muse_chat_session.cpp drains to ES8311.
 *
 * The key is compiled in (CONFIG_MUSE_TTS_VOICEPROXY_TOKEN), so
 * anyone holding the device can read it from the firmware image —
 * use a low-spending-capable token.
 */
#include "muse_tts_voiceproxy.h"

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
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "muse_tts_vp";

#ifndef CONFIG_MUSE_TTS_VOICEPROXY_URL
#define CONFIG_MUSE_TTS_VOICEPROXY_URL ""
#endif
#ifndef CONFIG_MUSE_TTS_VOICEPROXY_TOKEN
#define CONFIG_MUSE_TTS_VOICEPROXY_TOKEN ""
#endif
#ifndef CONFIG_MUSE_TTS_VOICEPROXY_VOICE
#define CONFIG_MUSE_TTS_VOICEPROXY_VOICE "Cantonese_GentleLady"
#endif

#define TTS_URL      CONFIG_MUSE_TTS_VOICEPROXY_URL "/v1/tts"
#define PREWARM_URL  CONFIG_MUSE_TTS_VOICEPROXY_URL "/v1/voices"
#define MP3_STREAM_BYTES (64 * 1024)
#define READ_CHUNK   4096
#define TIMEOUT_MS   35000   /* voice-proxy upstream is MiniMax with 30s timeout */

/* -------------------------------------------------------------------------- */
/*  Internal state                                                            */
/* -------------------------------------------------------------------------- */

static SemaphoreHandle_t s_lock;
static TaskHandle_t      s_task;
static StreamBufferHandle_t s_mp3;
static char             *s_text;       /* waiting request's text, under s_lock */
static uint32_t          s_want;       /* request id wanted, 0 = none */
static volatile uint32_t   s_stream_id;  /* whose bytes s_mp3 holds */
static volatile uint32_t   s_status_id;
static volatile int        s_status;

/* One HTTP client for the SDK's lifetime; the proxy's HTTPS server keeps
 * the connection open across calls so we reuse it. Fresh handshake costs
 * about a second. Only this task touches it. */
static esp_http_client_handle_t s_client;
static char             *s_auth;
static int64_t           s_last_use_us; /* when the connection last carried a request */
static volatile bool    s_warm;        /* muse_tts_voiceproxy_prewarm() asked for a warm-up */

static struct {
    uint32_t  id;          /* 0 = a warm-up, whose body is dropped */
    bool      ok;          /* still pushing */
    size_t    total;
    int64_t   first_us;
    char      err[512];    /* first ~500 bytes of an error body — FastAPI can be verbose */
    size_t    err_len;
} s_rx;

/* -------------------------------------------------------------------------- */
/*  Helpers                                                                   */
/* -------------------------------------------------------------------------- */

static bool wanted(uint32_t id)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool w = s_want == id;
    xSemaphoreGive(s_lock);
    return w;
}

static char *build_body(const char *text)
{
    /* Matches voice-proxy/app/tts.py TtsRequest (lines 29-35). */
    cJSON *j = cJSON_CreateObject();
    if (!j) {
        return NULL;
    }
    cJSON_AddStringToObject(j, "text", text);
    cJSON_AddStringToObject(j, "voice_id", CONFIG_MUSE_TTS_VOICEPROXY_VOICE);
    cJSON_AddStringToObject(j, "format", "mp3");
    cJSON_AddBoolToObject(j, "stream", false);
    cJSON_AddNumberToObject(j, "sample_rate", 32000);
    cJSON_AddNumberToObject(j, "speed", 1.0);
    char *body = cJSON_PrintUnformatted(j);
    cJSON_Delete(j);
    return body;
}

/* Hands buf to the session task, waiting for room while the request is
 * still wanted. */
static bool push(uint32_t id, const uint8_t *buf, size_t len)
{
    while (len) {
        size_t n = xStreamBufferSend(s_mp3, buf, len, pdMS_TO_TICKS(100));
        buf += n;
        len -= n;
        if (len && !wanted(id)) {
            return false;
        }
    }
    return true;
}

/* -------------------------------------------------------------------------- */
/*  HTTP plumbing                                                            */
/* -------------------------------------------------------------------------- */

static esp_err_t on_http(esp_http_client_event_t *evt)
{
    if (evt->event_id != HTTP_EVENT_ON_DATA || !evt->data_len) {
        return ESP_OK;
    }
    int status = esp_http_client_get_status_code(evt->client);
    if (status != 200) {
        size_t room = sizeof(s_rx.err) - 1 - s_rx.err_len;
        size_t n = evt->data_len < room ? evt->data_len : room;
        memcpy(s_rx.err + s_rx.err_len, evt->data, n);
        s_rx.err_len += n;
        s_rx.err[s_rx.err_len] = '\0';
        return ESP_OK;
    }
    if (!s_rx.id) {
        return ESP_OK;   /* warm-up body is dropped */
    }
    if (!s_rx.total) {
        s_rx.first_us = esp_timer_get_time();
    }
    s_rx.total += evt->data_len;
    if (s_rx.ok && !push(s_rx.id, evt->data, evt->data_len)) {
        s_rx.ok = false;   /* cancelled: keep draining but drop bytes */
    }
    return ESP_OK;
}

static bool client_ready(void)
{
    if (s_client) {
        return true;
    }
    const esp_http_client_config_t cfg = {
        .url               = TTS_URL,
        .timeout_ms        = TIMEOUT_MS,
        .buffer_size       = READ_CHUNK,
        .buffer_size_tx    = 2048,
        .event_handler     = on_http,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .keep_alive_enable = true,
    };
    s_client = esp_http_client_init(&cfg);
    if (!s_client) {
        return false;
    }
    esp_http_client_set_header(s_client, "Authorization", s_auth);
    esp_http_client_set_header(s_client, "Content-Type", "application/json");
    return true;
}

/* One request on the shared client, retried once on a fresh connection if
 * the old one had closed. Traefik typically drops idle >60s. */
static esp_err_t perform(void)
{
    bool reused = s_last_use_us && esp_timer_get_time() - s_last_use_us < 60 * 1000000LL;
    esp_err_t err = esp_http_client_perform(s_client);
    if (err != ESP_OK && reused && !s_rx.total) {
        ESP_LOGI(TAG, "kept connection had closed (%s); reconnecting",
                 esp_err_to_name(err));
        esp_http_client_close(s_client);
        s_rx.err_len = 0;
        err = esp_http_client_perform(s_client);
    }
    if (err == ESP_OK) {
        s_last_use_us = esp_timer_get_time();
    } else {
        esp_http_client_close(s_client);
        s_last_use_us = 0;
    }
    return err;
}

/* Opens the TLS connection ahead of the reply with a free GET /v1/voices. */
static void warm_up(void)
{
    if (s_last_use_us && esp_timer_get_time() - s_last_use_us < 30 * 1000000LL) {
        return;
    }
    if (!client_ready()) {
        return;
    }
    int64_t t0 = esp_timer_get_time();
    s_rx.id = 0;
    s_rx.err_len = 0;
    s_rx.total = 0;
    esp_http_client_set_url(s_client, PREWARM_URL);
    esp_http_client_set_method(s_client, HTTP_METHOD_GET);
    esp_http_client_set_post_field(s_client, NULL, 0);
    esp_err_t err = perform();
    ESP_LOGI(TAG, "warm-up: %s in %d ms",
             err == ESP_OK ? "connected" : esp_err_to_name(err),
             (int)((esp_timer_get_time() - t0) / 1000));
}

static int fetch(uint32_t id, const char *text)
{
    char *body = build_body(text);
    if (!body || !client_ready()) {
        ESP_LOGW(TAG, "out of memory or client not ready");
        free(body);
        return MUSE_TTS_VOICEPROXY_FAILED;
    }
    int64_t t0 = esp_timer_get_time();
    bool warm = s_last_use_us != 0;
    s_rx.id = id;
    s_rx.ok = true;
    s_rx.total = 0;
    s_rx.first_us = 0;
    s_rx.err_len = 0;
    esp_http_client_set_url(s_client, TTS_URL);
    esp_http_client_set_method(s_client, HTTP_METHOD_POST);
    esp_http_client_set_post_field(s_client, body, strlen(body));
    esp_err_t err = perform();
    esp_http_client_set_post_field(s_client, NULL, 0);
    free(body);
    int status = err == ESP_OK ? esp_http_client_get_status_code(s_client) : 0;
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "can't reach voice-proxy (%s)", esp_err_to_name(err));
        return MUSE_TTS_VOICEPROXY_FAILED;
    }
    if (status != 200) {
        ESP_LOGW(TAG, "voice-proxy HTTP %d: %s", status, s_rx.err);
        return MUSE_TTS_VOICEPROXY_FAILED;
    }
    int64_t t1 = esp_timer_get_time();
    ESP_LOGI(TAG, "speech: %u chars -> %u bytes of MP3, first byte %d ms, all %d ms (%s connection)",
             (unsigned)strlen(text), (unsigned)s_rx.total,
             s_rx.first_us ? (int)((s_rx.first_us - t0) / 1000) : -1,
             (int)((t1 - t0) / 1000),
             warm ? "kept" : "new");
    return s_rx.ok && s_rx.total ? MUSE_TTS_VOICEPROXY_DONE : MUSE_TTS_VOICEPROXY_FAILED;
}

/* -------------------------------------------------------------------------- */
/*  Task and public API                                                      */
/* -------------------------------------------------------------------------- */

static void tts_task(void *arg)
{
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        for (;;) {
            xSemaphoreTake(s_lock, portMAX_DELAY);
            uint32_t id = s_want;
            char *text = s_text;
            s_text = NULL;
            xSemaphoreGive(s_lock);
            if (!id || !text) {
                heap_caps_free(text);
                if (s_warm) {
                    s_warm = false;
                    warm_up();
                    continue;   /* a request may have come in meanwhile */
                }
                break;
            }
            s_warm = false;
            /* The session task never blocks on s_mp3, so the reset can't
             * fail; stale bytes from the last request go with it. */
            xStreamBufferReset(s_mp3);
            s_stream_id = id;
            s_status_id = id;
            s_status = MUSE_TTS_VOICEPROXY_RUNNING;
            int rc = fetch(id, text);
            heap_caps_free(text);
            if (s_status_id == id) {
                s_status = rc;
            }
            if (wanted(id)) {
                break;   /* nothing newer waiting */
            }
        }
    }
}

void muse_tts_voiceproxy_start(void)
{
    if (s_task || !CONFIG_MUSE_TTS_VOICEPROXY_URL[0] || !CONFIG_MUSE_TTS_VOICEPROXY_TOKEN[0]) {
        return;
    }
    s_lock = xSemaphoreCreateMutex();
    size_t auth_len = strlen(CONFIG_MUSE_TTS_VOICEPROXY_TOKEN) + 8;
    s_auth = heap_caps_malloc(auth_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_auth) {
        snprintf(s_auth, auth_len, "Bearer %s", CONFIG_MUSE_TTS_VOICEPROXY_TOKEN);
    }
    s_mp3 = xStreamBufferCreateWithCaps(MP3_STREAM_BYTES, 1, MALLOC_CAP_SPIRAM);
    if (!s_lock || !s_auth || !s_mp3 ||
        xTaskCreatePinnedToCoreWithCaps(tts_task, "muse_tts_vp", 12 * 1024, NULL, 4, &s_task, 0,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
        ESP_LOGE(TAG, "start failed");
        s_task = NULL;
        return;
    }
    ESP_LOGI(TAG, "voice-proxy TTS on: voice_id %s @ %s",
             CONFIG_MUSE_TTS_VOICEPROXY_VOICE, CONFIG_MUSE_TTS_VOICEPROXY_URL);
}

bool muse_tts_voiceproxy_enabled(void)
{
    return s_task != NULL;
}

bool muse_tts_voiceproxy_request(uint32_t id, const char *text)
{
    if (!s_task || !id || !text || !text[0]) {
        return false;
    }
    size_t n = strlen(text) + 1;
    char *copy = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!copy) {
        return false;
    }
    memcpy(copy, text, n);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    heap_caps_free(s_text);
    s_text = copy;
    s_want = id;
    xSemaphoreGive(s_lock);
    s_status_id = id;
    s_status = MUSE_TTS_VOICEPROXY_RUNNING;
    xTaskNotifyGive(s_task);
    return true;
}

void muse_tts_voiceproxy_prewarm(void)
{
    if (s_task) {
        s_warm = true;
        xTaskNotifyGive(s_task);
    }
}

void muse_tts_voiceproxy_cancel(void)
{
    if (!s_task) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_want = 0;
    heap_caps_free(s_text);
    s_text = NULL;
    xSemaphoreGive(s_lock);
}

size_t muse_tts_voiceproxy_read(uint32_t id, uint8_t *buf, size_t cap)
{
    if (!s_task || s_stream_id != id) {
        return 0;
    }
    return xStreamBufferReceive(s_mp3, buf, cap, 0);
}

int muse_tts_voiceproxy_status(uint32_t id)
{
    if (s_status_id != id) {
        return MUSE_TTS_VOICEPROXY_RUNNING;
    }
    return s_status;
}