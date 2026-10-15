/*
 * Cantonese TTS via self-hosted voice-proxy.
 *
 * Modeled after JJenglert1/esp32-muse-agent's muse_tts_openai.{c,h}
 * (Apache 2.0) but pointed at our FastAPI backend that wraps
 * MiniMax T2A v2 (Cantonese_GentleLady). Off unless both URL and
 * TOKEN are set via menuconfig (CONFIG_MUSE_TTS_VOICEPROXY_*).
 *
 * One request at a time runs on its own PSRAM task; the session
 * task polls for MP3 with muse_tts_voiceproxy_read() while
 * tts_pump() (defined in muse_chat_session.cpp) feeds the SDK's
 * tts_data() callback.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Starts the fetch task. Call once from muse_hatch_start().
 * No-op if URL or TOKEN is empty. */
void muse_tts_voiceproxy_start(void);

/* URL and TOKEN are compiled in and the task is running. */
bool muse_tts_voiceproxy_enabled(void);

/* Fetches speech for text, tagged id, replacing any request in
 * flight. False if it can't start. */
bool muse_tts_voiceproxy_request(uint32_t id, const char *text);

/* Opens the TLS connection to voice-proxy ahead of a reply
 * (during the user's turn), unless one is fresh. */
void muse_tts_voiceproxy_prewarm(void);

/* Stops the request in flight (called from turn_finish). */
void muse_tts_voiceproxy_cancel(void);

/* Copies up to cap bytes of request id's MP3 without blocking;
 * 0 when none is waiting. Called from tts_pump(). */
size_t muse_tts_voiceproxy_read(uint32_t id, uint8_t *buf, size_t cap);

/* Where request id stands. */
int muse_tts_voiceproxy_status(uint32_t id);

#define MUSE_TTS_VOICEPROXY_RUNNING 0
#define MUSE_TTS_VOICEPROXY_DONE 1      /* every byte handed to muse_tts_voiceproxy_read */
#define MUSE_TTS_VOICEPROXY_FAILED (-1)

#ifdef __cplusplus
}
#endif