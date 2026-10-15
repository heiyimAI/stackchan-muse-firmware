/*
 * Cantonese speech-to-text via the self-hosted voice-proxy (FastAPI +
 * MiniMax ASR). The input-side sibling of muse_tts_voiceproxy: the voice
 * task posts the kept recording here after PTT release, and the transcript
 * is injected into the chat via muse_chat_say_text() so the reply is
 * captioned and spoken.
 *
 * The HTTP work runs on this module's own PSRAM-stack task; callers block
 * on a semaphore, never on TLS. One request is in flight at a time.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* URL and TOKEN are configured (pure config check; the worker starts
 * lazily on the first prewarm/transcribe). */
bool muse_stt_voiceproxy_enabled(void);

/* Opens the TLS connection to the voice-proxy (GET /v1/health) on the
 * worker task. Call at PTT press so the handshake hides under the
 * recording. Non-blocking. */
void muse_stt_voiceproxy_prewarm(void);

/* Transcribes 16 kHz mono PCM (frames, raw int16) via POST /v1/stt with
 * lang=yue. Blocking: waits up to timeout_ms for the worker. On success
 * the NUL-terminated transcript is copied into out (trimmed of leading
 * spaces) and true is returned. False on any failure (no transcript). */
bool muse_stt_voiceproxy_transcribe(const int16_t *pcm, size_t frames,
                                    char *out, size_t cap, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
