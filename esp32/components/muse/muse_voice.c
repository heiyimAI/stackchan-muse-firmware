/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "muse_voice.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "muse_adpcm.h"
#include "muse_audio.h"
#include "muse_board.h"
#include "muse_chat.h"
#include "muse_input.h"
#include "muse_mem.h"
#include "muse_settings.h"
#include "muse_state.h"
#ifdef CONFIG_MUSE_STT_VOICEPROXY
#include "muse_stt_voiceproxy.h"
#endif
#include "muse_vad.h"
#ifdef CONFIG_MUSE_BODY
#include "muse_body.h"
#else
#define muse_body_set_pose(p) ((void)0)
#endif
#ifdef CONFIG_MUSE_WAKE
#include "muse_wake.h"
#endif
#include "muse_wifi.h"

static const char *TAG = "muse_voice";

/* Set by muse_voice_request_hush(): the voice task consumes it. */
static volatile bool s_hush;

#define MAX_SECS 15
#define TAIL_FRAMES (MUSE_AUDIO_RATE * 12 / 100)   /* capture lag + poll interval, stops before the release click */
#define MAX_FRAMES (MUSE_AUDIO_RATE * MAX_SECS)
#define MIN_HELD_FRAMES (MUSE_AUDIO_RATE * 3 / 10)   /* shorter presses are taps, not speech */

/*
 * The speech-end detector's numbers, from Kconfig where the Hatch build
 * provides them (its menu is where they're documented). The fallbacks are
 * those same defaults, so a build without them still records voice turns.
 */
#ifdef CONFIG_MUSE_VAD_DROP_DB
#define VAD_DROP_DB ((float)CONFIG_MUSE_VAD_DROP_DB)
#define VAD_FLOOR_DB ((float)CONFIG_MUSE_VAD_FLOOR_DB)
#define VAD_MIN_SPEECH_CHUNKS CONFIG_MUSE_VAD_MIN_SPEECH_CHUNKS
#define VAD_SILENCE_CHUNKS CONFIG_MUSE_VAD_SILENCE_CHUNKS
#define VAD_NO_SPEECH_MS CONFIG_MUSE_VAD_NO_SPEECH_MS
#define VAD_PREROLL_CHUNKS CONFIG_MUSE_VAD_PREROLL_CHUNKS
#else
#define VAD_DROP_DB 18.0f
#define VAD_FLOOR_DB (-45.0f)
#define VAD_MIN_SPEECH_CHUNKS 15
#define VAD_SILENCE_CHUNKS 15
#define VAD_NO_SPEECH_MS 5000
#define VAD_PREROLL_CHUNKS 4
#endif
#if CONFIG_MUSE_BOARD_M5STACK_CARDPUTER_ADV || CONFIG_MUSE_BOARD_AI_PASSPORT
#define PRE_CHUNKS 6                                   /* 120 ms; a 320 ms upload burst stalls on the ADV and C3 */
#else
#define PRE_CHUNKS 16                                  /* 320 ms of audio kept from before the press */
#endif
#define SETTLE_CHUNKS 10   /* after Muse makes a sound, 200 ms of capture is its own tail */
#define REST_BACKSTOP_MS 60000

#if CONFIG_MUSE_HATCH
/* A note recorded while Hatch is out of reach is saved in PSRAM, and goes once it's back. */
#define HOLD_NOTES 1
#else
#define HOLD_NOTES 0   /* no PSRAM: a press needs Hatch */
#endif
#define HELD_MAX 4
#define HELD_TRIES 8                            /* then a saved note is dropped */
#define HELD_POLL_MS 5000                       /* resting with notes saved: check for Wi-Fi this often */
#define HELD_KEEP_US (30LL * 60 * 1000000)      /* muse_voice_notes_waiting() */
#define RETRY_MIN_US (15LL * 1000000)
#define RETRY_MAX_US (120LL * 1000000)
#define STALL_US (30LL * 1000000)               /* Hatch took none of a note this long: give up */
#define ACK_WAIT_US (30LL * 1000000)            /* asleep, waiting for the VM to have a note */

static QueueHandle_t s_queue;
static volatile bool s_monitor;
static volatile float s_monitor_db = -100.0f;
static volatile bool s_chirp;
static volatile bool s_loopback;
static volatile bool s_mp3test;

/*
 * Pre-roll: while idle the mic keeps running into this ring, so a recording
 * can start a little before the press. People start talking as they press,
 * and the power key reports the press late.
 */
typedef struct {
#if MUSE_LOW_MEM
    muse_adpcm_t start;   /* encoder state before this chunk, to decode it alone */
    uint8_t adpcm[MUSE_AUDIO_CHUNK / 2];
#else
    int16_t pcm[MUSE_AUDIO_CHUNK];
#endif
} pre_chunk_t;

static pre_chunk_t *s_pre;   /* PRE_CHUNKS, a ring */
static size_t s_pre_next, s_pre_fill;
#if MUSE_LOW_MEM
static muse_adpcm_t s_pre_enc;
#endif
static int16_t s_chunk[MUSE_AUDIO_CHUNK];
static int s_settle;

/*
 * The note being recorded (HOLD_NOTES), kept whole in case it can't go now.
 * Streaming, Hatch has had s_sent frames of it.
 */
static int16_t *s_rec;   /* MAX_FRAMES */
static size_t s_rec_n, s_sent;
static bool s_live = true;
static bool s_tried;     /* it went to Hatch */

typedef struct {
    int16_t *pcm;
    size_t frames;
    int64_t at_us;   /* when it was recorded */
    int tries;
} held_note_t;

static int s_held_count;
static volatile bool s_waiting;        /* muse_voice_notes_waiting() */
#if HOLD_NOTES
static held_note_t s_held[HELD_MAX];   /* oldest first */
static int64_t s_next_send_us;
static int64_t s_send_backoff_us = RETRY_MIN_US;
#endif

/* The i-th oldest pre-roll chunk, into out. */
static void pre_get(size_t i, int16_t *out)
{
    const pre_chunk_t *c = &s_pre[(s_pre_next + PRE_CHUNKS - s_pre_fill + i) % PRE_CHUNKS];
#if MUSE_LOW_MEM
    muse_adpcm_t dec = c->start;
    muse_adpcm_decode_block(&dec, c->adpcm, MUSE_AUDIO_CHUNK, out);
#else
    memcpy(out, c->pcm, sizeof(c->pcm));
#endif
}

/* Call after Muse plays anything: its own sound must not become pre-roll. */
static void pre_reset(void)
{
    s_pre_fill = 0;
    s_settle = SETTLE_CHUNKS;
}

/*
 * Keeps only the newest `chunks` of the pre-roll — for a voice-triggered
 * turn, whose ring holds the wake phrase (and whatever the room was doing
 * while it was said). Not pre_reset(): that would also drop the first
 * chunks after the trigger, which are the ones the note needs.
 */
static void pre_keep(size_t chunks)
{
    if (s_pre_fill > chunks) {
        s_pre_fill = chunks;
    }
}

/*
 * The room's level: the median of the last 320 ms of idle chunks, followed
 * quickly down and slowly up so that a voice is never learned as the
 * background. The wake word's guard and the end-of-speech detector both
 * measure against it — a fixed floor would have to be re-tuned for every
 * room (and the first one tried sat exactly on this room's noise).
 * The scheme is esp32-thinks'.
 */
#define NOISE_MEDIAN 16        /* 320 ms of 20 ms chunks */
#define NOISE_FOLLOW_DB 6.0f   /* within this of the estimate, it is the room */
/*
 * A voice has to beat the room by this much before it is speech. The
 * estimate is the room's *median*, so without it half the room's own noise
 * sits above the threshold: the note then never finds 1.4 s of quiet and
 * runs to the cap. (esp32-thinks' speech guard uses the same 6 dB.)
 */
#define NOISE_VOICE_DB 6.0f
static float s_noise_ring[NOISE_MEDIAN];
static unsigned s_noise_at, s_noise_fill;
static float s_noise_db = (float)VAD_FLOOR_DB;   /* until the ring has filled */
static bool s_noise_measured;

static void noise_track(float db)
{
    s_noise_ring[s_noise_at] = db;
    s_noise_at = (s_noise_at + 1) % NOISE_MEDIAN;
    if (s_noise_fill < NOISE_MEDIAN) {
        s_noise_fill++;
        return;
    }
    float sorted[NOISE_MEDIAN];
    memcpy(sorted, s_noise_ring, sizeof sorted);
    for (unsigned i = 1; i < NOISE_MEDIAN; i++) {   /* 16 entries: insertion sort is plenty */
        float x = sorted[i];
        unsigned j = i;
        while (j > 0 && sorted[j - 1] > x) {
            sorted[j] = sorted[j - 1];
            j--;
        }
        sorted[j] = x;
    }
    float background = sorted[NOISE_MEDIAN / 2];
    if (!s_noise_measured) {
        s_noise_db = background;
        s_noise_measured = true;
    } else if (background < s_noise_db + NOISE_FOLLOW_DB) {
        s_noise_db += 0.1f * (background - s_noise_db);
    } else {
        s_noise_db += 0.002f;   /* louder than the room: speech, not the room */
    }
}

/* One 20 ms idle read: feeds the pre-roll ring and the mic meter. */
static void idle_capture(void)
{
    if (muse_audio_read(s_chunk, MUSE_AUDIO_CHUNK) != ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(20));
        return;
    }
    if (s_monitor) {
        float db = muse_audio_dbfs(s_chunk, MUSE_AUDIO_CHUNK);
        s_monitor_db = db > s_monitor_db ? db : s_monitor_db * 0.9f + db * 0.1f;
    }
    if (s_settle > 0) {
        s_settle--;
        return;
    }
    noise_track(muse_audio_dbfs(s_chunk, MUSE_AUDIO_CHUNK));
    pre_chunk_t *c = &s_pre[s_pre_next];
#if MUSE_LOW_MEM
    c->start = s_pre_enc;
    muse_adpcm_encode_block(&s_pre_enc, s_chunk, MUSE_AUDIO_CHUNK, c->adpcm);
#else
    memcpy(c->pcm, s_chunk, sizeof(c->pcm));
#endif
    s_pre_next = (s_pre_next + 1) % PRE_CHUNKS;
    s_pre_fill = s_pre_fill < PRE_CHUNKS ? s_pre_fill + 1 : PRE_CHUNKS;
}

/* Non-blocking: returns true if an event of `type` arrived (others dropped). */
static bool got_event(muse_ptt_t type)
{
    muse_input_event_t ev;
    bool hit = false;
    while (xQueueReceive(s_queue, &ev, 0) == pdTRUE) {
        muse_state_poke();
        hit |= ev.type == type;
    }
    return hit;
}

/* Level stats of a recording, gathered chunk by chunk for the log. */
typedef struct {
    double acc;
    size_t frames;
    int peak, clipped;
    float floor_db;
    float last_db;       /* the newest full chunk's level: what the VAD steps on */
    float tail_db[25];   /* the last 0.5 s, to see where speech ends and the release click lands */
    size_t chunks;
} rec_stats_t;

/* Adds one captured chunk (up to MUSE_AUDIO_CHUNK frames) to the stats. */
static void keep(rec_stats_t *st, const int16_t *pcm, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        int v = abs(pcm[i]);
        st->peak = v > st->peak ? v : st->peak;
        st->clipped += v >= 32000;
        st->acc += (double)pcm[i] * pcm[i];
    }
    st->frames += n;
    if (n == MUSE_AUDIO_CHUNK) {
        float db = muse_audio_dbfs(pcm, n);
        st->last_db = db;
        st->floor_db = db < st->floor_db ? db : st->floor_db;
        st->tail_db[st->chunks++ % 25] = db;
    }
}

/* Gives Hatch what it has room for of the note being recorded. */
static void feed_live(void)
{
#if HOLD_NOTES
    if (s_live && s_sent < s_rec_n) {
        s_sent += muse_hatch_turn_audio_wait(s_rec + s_sent, s_rec_n - s_sent, 0);
    }
#endif
}

/* Streams the note being recorded from its start, then the rest as it comes. */
static void go_live(void)
{
#ifdef CONFIG_MUSE_STT_VOICEPROXY
    /* STT mode: nothing goes to Muse during recording. The whole note stays
     * in s_rec; after release it is transcribed on the voice-proxy and the
     * transcript is injected as a turn (finish_note). */
    if (muse_stt_voiceproxy_enabled()) {
        return;
    }
#endif
    muse_hatch_turn_begin();
    s_live = s_tried = true;
    s_sent = 0;
    feed_live();
}

/* One chunk of speech: into the stats, and on to Hatch, by way of the kept note if there is one. */
static void take(rec_stats_t *st, const int16_t *pcm)
{
    keep(st, pcm, MUSE_AUDIO_CHUNK);
    if (!s_rec) {
        if (s_live) {
            muse_hatch_turn_audio(pcm, MUSE_AUDIO_CHUNK);
        }
        return;
    }
    memcpy(s_rec + s_rec_n, pcm, MUSE_AUDIO_CHUNK * sizeof(int16_t));
    s_rec_n += MUSE_AUDIO_CHUNK;
    feed_live();
}

static void drop_rec(void);

/*
 * Records speech until release or MAX_SECS, starting with the pre-roll; *held
 * gets the part after the press. It streams to Hatch as it goes if it can.
 * If not, the note is kept in s_rec to send later, and streams from partway
 * if Hatch comes within reach. `voice` is a turn with no finger holding it
 * open (a wake word's): the note ends on the VAD once the speech has stopped,
 * and a turn where nothing is said at all is abandoned rather than recorded
 * to the cap. Returns false if Hatch failed the turn with no kept note to
 * fall back on, or if a voice turn was abandoned (why says what happened).
 * There is no start chirp: anything played now would land on top of the
 * first words.
 */
static bool record(bool barge_in, bool voice, size_t *held, char *why, size_t cap)
{
    muse_state_set_mode(MUSE_MODE_LISTENING);
    muse_body_set_pose(MUSE_BODY_POSE_LISTEN);
    s_hush = false;
    muse_state_set_progress(0);
    s_rec_n = s_sent = 0;
    s_live = s_tried = false;
    if (!s_rec || (muse_hatch_ready() && !s_held_count)) {
        go_live();
    }
#ifdef CONFIG_MUSE_STT_VOICEPROXY
    /* Open the transcription connection while the user talks, so the POST
     * goes out the moment the note ends. */
    muse_stt_voiceproxy_prewarm();
#endif
    muse_state_set_caption(s_live ? "LISTENING..." : "RECORDING...");
    bool heard = false, ok = true;
    bool gave_up = false;   /* Hatch failed this note: it's kept, and goes later */
    char text[96];
    rec_stats_t st = { 0 };
    muse_vad_t vad = { .drop_db = VAD_DROP_DB, .floor_db = VAD_FLOOR_DB,
                       .min_speech_chunks = VAD_MIN_SPEECH_CHUNKS,
                       .silence_chunks = VAD_SILENCE_CHUNKS };
    muse_vad_reset(&vad);
    /* Talking to a wake word is not the same as holding a button: nobody is
     * standing there to end the note, and nobody said there would be speech. */
    const size_t no_speech_frames = (size_t)MUSE_AUDIO_RATE * VAD_NO_SPEECH_MS / 1000;
    bool abandoned = false;

    size_t n = 0;
    if (barge_in) {
        /* Pressed during playback: the queued tail of the reply is still sounding. */
        for (int i = 0; i < SETTLE_CHUNKS; i++) {
            muse_audio_read(s_chunk, MUSE_AUDIO_CHUNK);
        }
    } else {
        for (size_t i = 0; i < s_pre_fill; i++) {
            pre_get(i, s_chunk);
            take(&st, s_chunk);
            n += MUSE_AUDIO_CHUNK;
        }
    }
    size_t pre = n;
    bool released = false;
    size_t stop_at = MAX_FRAMES;
    while (n + MUSE_AUDIO_CHUNK <= stop_at) {
        if (muse_audio_read(s_chunk, MUSE_AUDIO_CHUNK) != ESP_OK) {
            break;
        }
        muse_state_set_level(muse_audio_level(s_chunk, MUSE_AUDIO_CHUNK));
        take(&st, s_chunk);
        /* Live transcript as the caption. A failure stops the streaming; the
         * kept note goes later, or without one the failure is the caption. */
        muse_hatch_ev_t ev;
        while (s_live && (ev = muse_hatch_turn_event(text, sizeof(text))) != MUSE_HATCH_EV_NONE) {
            if (ev == MUSE_HATCH_EV_HEARD && text[0]) {
                heard = true;
                muse_state_set_caption("%s", text);
            } else if (ev == MUSE_HATCH_EV_ERROR) {
                ESP_LOGW(TAG, "muse: %s", text);
                muse_hatch_turn_cancel();
                s_live = false;
                gave_up = true;
                if (!s_rec) {
                    ok = false;
                    strlcpy(why, text, cap);
                    muse_state_set_caption("%s", text);
                }
            }
        }
        n += MUSE_AUDIO_CHUNK;
        if (s_hush) {
            /* The head was touched: the note is not wanted. */
            s_hush = false;
            ESP_LOGI(TAG, "note dropped by touch");
            muse_hatch_turn_cancel();
            drop_rec();
            *held = n - pre;
            why[0] = '\0';
            return false;
        }
        bool tick = n % (MUSE_AUDIO_CHUNK * 5) == 0;
#ifdef CONFIG_MUSE_STT_VOICEPROXY
        /* STT mode keeps the note local; there is nothing to stream to Muse. */
        bool live_ok = !muse_stt_voiceproxy_enabled();
#else
        bool live_ok = true;
#endif
        if (tick && live_ok && s_rec && !s_live && !gave_up && !s_held_count && muse_hatch_ready()) {
            ESP_LOGI(TAG, "Muse in reach: streaming the note so far");
            go_live();
        }
        muse_state_set_progress((float)n / MAX_FRAMES);
        if (!heard && ok && tick) {
            muse_state_set_caption("%s %.1fs", s_live ? "LISTENING" : "RECORDING", (double)n / MUSE_AUDIO_RATE);
        }
        /*
         * Capture runs 60-80 ms behind real time and people let go on their
         * last syllable, so keep going briefly after release.
         */
        if (!released) {
            bool up = got_event(MUSE_PTT_UP);
            if (voice) {
                /* The room as the idle loop last measured it, plus enough to
                 * rise above the room's own noise: the note is judged
                 * against where this room actually sits, not against a
                 * number chosen on a bench. */
                vad.floor_db = s_noise_db + NOISE_VOICE_DB;
            }
            bool speech_over = voice && muse_vad_step(&vad, st.last_db);
            if (up || speech_over) {
                released = true;
                stop_at = n + TAIL_FRAMES < MAX_FRAMES ? n + TAIL_FRAMES : MAX_FRAMES;
                if (speech_over && !up) {
                    ESP_LOGI(TAG, "voice: speech ended at %.1f dBFS after %.1fs",
                             st.last_db, (double)n / MUSE_AUDIO_RATE);
                }
            } else if (voice && !vad.armed && n >= no_speech_frames) {
                /* Not `spoke`: a single spike over the floor — a click, a
                 * door — would turn that off for the rest of the note, and
                 * the note would then run to the cap. Armed means 300 ms of
                 * speech in all, which scattered clicks never reach. */
                abandoned = true;
                break;
            }
        }
    }
    muse_state_set_level(0);
    *held = n - pre;

    char tail[160];
    int tl = 0;
    size_t shown = st.chunks < 25 ? st.chunks : 25;
    for (size_t i = st.chunks - shown; i < st.chunks; i++) {
        tl += snprintf(tail + tl, sizeof(tail) - tl, " %.0f", st.tail_db[i % 25]);
    }
    ESP_LOGI(TAG, "end levels:%s", tail);
    float rms_db = st.frames ? 10.0f * log10f((float)(st.acc / st.frames / (32768.0 * 32768.0)) + 1e-10f) : -100.0f;
    ESP_LOGI(TAG, "recorded %.2fs%s: rms %.1f dBFS, peak %.1f dBFS, floor %.1f dBFS, %d clipped, gain %d dB",
             (double)n / MUSE_AUDIO_RATE, released ? "" : " (max)", rms_db < -100.0f ? -100.0f : rms_db,
             20.0 * log10((st.peak + 1) / 32768.0), st.floor_db, st.clipped, muse_settings_mic_gain());
    if (voice) {
        /* What the detector saw, so the floor can be tuned to the room:
         * `armed` means enough speech crossed the threshold to let the note
         * end by itself, and a note that never arms is abandoned. */
        ESP_LOGI(TAG, "voice: peak %.1f dBFS, threshold %.1f dBFS, %s",
                 vad.peak_db, muse_vad_threshold(&vad),
                 vad.armed ? "armed" : vad.spoke ? "peaks only" : "all below the floor");
    }
    if (abandoned) {
        /* The wake word heard something that wasn't a person, or the mic is
         * dead: either way there is no note to send, and no reason to sit
         * here for the rest of the cap. The caller drops it and says so. */
        ESP_LOGI(TAG, "voice: nothing said in %.1fs, giving up", (double)n / MUSE_AUDIO_RATE);
        muse_hatch_turn_cancel();
        drop_rec();   /* its buffer is half a megabyte of PSRAM: free it here */
        strlcpy(why, "DIDN'T CATCH THAT", cap);
        return false;
    }
    return ok;
}

static void go_idle(const char *caption);

/*
 * Plays Hatch's reply as it arrives, with its text as the caption. Returns
 * true if interrupted by a new press. *delivered: the VM has the note.
 */
static bool hatch_reply(bool *delivered)
{
    muse_state_set_mode(MUSE_MODE_THINKING);
    muse_body_set_pose(MUSE_BODY_POSE_THINK);
    s_hush = false;
    muse_state_set_caption("SENDING VOICE NOTE");   /* until there's a transcript or reply */
    static int16_t buf[MUSE_AUDIO_CHUNK];
    static const int16_t silence[MUSE_AUDIO_CHUNK];
    char text[96];
    static char page[MUSE_CAPTION_MAX];
    bool done = false, speaking = false, replied = false;
    size_t played = 0;
    int64_t t0 = esp_timer_get_time();
    *delivered = false;
    for (;;) {
        if (s_hush) {
            /* Touch to hush: a finished reply, not an interruption —
             * the caller goes idle instead of recording a new note. */
            s_hush = false;
            ESP_LOGI(TAG, "reply hushed by touch");
            muse_hatch_turn_cancel();
            muse_state_set_level(0);
            return false;
        }
        muse_hatch_ev_t ev;
        while ((ev = muse_hatch_turn_event(text, sizeof(text))) != MUSE_HATCH_EV_NONE) {
            switch (ev) {
            case MUSE_HATCH_EV_HEARD:
                if (!speaking && !replied) {
                    muse_state_set_caption("\"%s\"", text);
                }
                break;
            case MUSE_HATCH_EV_SENT:
                *delivered = true;
                if (!speaking && !replied) {
                    muse_state_set_caption("NOTE SENT - WAITING FOR MUSE");
                }
                break;
            case MUSE_HATCH_EV_REPLY:
                replied = *delivered = true;
                /* Once speech starts, the caption follows it. The event only
                 * has room for the page's start; the page itself comes below. */
                if (!speaking && !muse_hatch_turn_caption(played, page, sizeof(page))) {
                    muse_state_set_caption("%s", text);
                }
                break;
            case MUSE_HATCH_EV_DONE:
                done = *delivered = true;
                break;
            case MUSE_HATCH_EV_ERROR:
                ESP_LOGW(TAG, "muse: %s", text);
                muse_state_set_level(0);
                go_idle(text);
                return false;
            default:
                break;
            }
        }
        if (got_event(MUSE_PTT_DOWN)) {
            ESP_LOGI(TAG, "reply interrupted");
            muse_hatch_turn_cancel();
            muse_state_set_level(0);
            return true;
        }
        size_t n = muse_hatch_turn_read(buf, MUSE_AUDIO_CHUNK, speaking || done ? 0 : 20);
        if (n) {
            if (!speaking) {
                speaking = true;
                muse_state_set_mode(MUSE_MODE_SPEAKING);
                muse_body_set_pose(MUSE_BODY_POSE_SPEAK);
                ESP_LOGI(TAG, "reply audio after %.2fs", (esp_timer_get_time() - t0) / 1e6);
            }
            muse_state_set_level(muse_audio_level(buf, n));
            muse_audio_write(buf, n);
            played += n;
        } else if (done) {
            break;
        } else if (speaking) {
            /* Between messages: keep the speaker fed so it doesn't replay stale DMA. */
            muse_state_set_level(0);
            muse_audio_write(silence, MUSE_AUDIO_CHUNK);
        }
        /* The page being said, or before the speech the reply's opening page. */
        if ((speaking || replied) && muse_hatch_turn_caption(played, page, sizeof(page))) {
            muse_state_set_caption("%s", page);
        }
    }
    muse_state_set_level(0);
    ESP_LOGI(TAG, "muse reply: %.2fs of audio, %.2fs total", (double)played / MUSE_AUDIO_RATE,
             (esp_timer_get_time() - t0) / 1e6);
    if (!played) {
        /* No speech (TTS unavailable): leave the reply text up for a moment. */
        vTaskDelay(pdMS_TO_TICKS(2500));
    }
    return false;
}

static void go_idle(const char *caption)
{
    muse_state_set_progress(0);
    muse_state_set_mode(MUSE_MODE_IDLE);
    muse_body_set_pose(MUSE_BODY_POSE_IDLE);
    muse_state_set_caption("%s", caption);
}

/* Why a press can't go to Hatch; voice notes only go there. */
static const char *not_ready_reason(void)
{
    muse_hatch_status_t st;
    muse_hatch_status(&st);
    switch (st.state) {
    case MUSE_HATCH_NOT_SET: return "SET UP MUSE FIRST";
    case MUSE_HATCH_OFFLINE: return "NO WI-FI";
    default: return "CAN'T REACH MUSE";
    }
}

static volatile bool s_resting;

/*
 * Asleep on battery with nothing to do: power the codecs down (the loop rests
 * Wi-Fi alongside). They come back within one wait on waking, and a talk press
 * that woke Muse is kept for the loop (a note may follow). On USB power
 * nothing changes asleep.
 */
static void set_resting(bool rest)
{
    if (rest == s_resting) {
        return;
    }
    s_resting = rest;
    muse_audio_power(!rest);
    muse_hatch_set_resting(rest);
    if (rest) {
        /* Presses from before sleeping are stale, but not one that just woke Muse. */
        muse_input_event_t ev;
        while (xQueueReceive(s_queue, &ev, 0) == pdTRUE) {
            if (ev.wake) {
                xQueueSendToFront(s_queue, &ev, 0);
                break;
            }
        }
    } else {
        pre_reset();
    }
    ESP_LOGI(TAG, "%s", rest ? "resting: codecs off, Wi-Fi modem sleep" : "awake: codecs on");
}

/*
 * A press that woke the screen: listen from now, into the pre-roll, and
 * record only if it's still held after a tap's length. False if it was let
 * go sooner: it only woke Muse.
 */
static bool held_on_waking(void)
{
    s_settle = 0;   /* nothing played: no tail of Muse's own to skip */
    for (size_t n = 0; n < MIN_HELD_FRAMES; n += MUSE_AUDIO_CHUNK) {
        if (got_event(MUSE_PTT_UP)) {
            return false;
        }
        idle_capture();
    }
    return !got_event(MUSE_PTT_UP);
}

static void drop_rec(void)
{
    free(s_rec);
    s_rec = NULL;
}

#if HOLD_NOTES
/* True if a press is queued, left there for the loop; releases ahead of it are dropped. */
static bool press_waiting(void)
{
    muse_input_event_t ev;
    while (xQueuePeek(s_queue, &ev, 0) == pdTRUE) {
        if (ev.type == MUSE_PTT_DOWN) {
            return true;
        }
        xQueueReceive(s_queue, &ev, 0);
    }
    return false;
}

static void back_off(void)
{
    s_next_send_us = esp_timer_get_time() + s_send_backoff_us;
    s_send_backoff_us = s_send_backoff_us * 2 > RETRY_MAX_US ? RETRY_MAX_US : s_send_backoff_us * 2;
}

/* Saves the note just recorded to send later. `tried`: Hatch failed it, so it waits out the backoff. */
static void hold_rec(bool tried)
{
    if (s_held_count >= HELD_MAX) {   /* can_record() leaves room: not expected */
        drop_rec();
        go_idle("COULDN'T SAVE THE NOTE");
        return;
    }
    if (!s_rec_n) {   /* nothing captured: realloc to 0 would free s_rec and return NULL */
        drop_rec();
        go_idle("HOLD LONGER TO TALK");
        return;
    }
    int16_t *pcm = heap_caps_realloc(s_rec, s_rec_n * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_held[s_held_count++] = (held_note_t){
        .pcm = pcm ? pcm : s_rec,
        .frames = s_rec_n,
        .at_us = esp_timer_get_time(),
        .tries = tried,
    };
    s_rec = NULL;
    if (tried) {
        back_off();
    }
    ESP_LOGI(TAG, "saved a %.1fs note to send later (%d waiting)", (double)s_rec_n / MUSE_AUDIO_RATE, s_held_count);
    go_idle(tried ? "SAVED, WILL TRY AGAIN" : "SAVED, SENDS WHEN ONLINE");
}

static void drop_oldest(void)
{
    free(s_held[0].pcm);
    s_held_count--;
    memmove(s_held, s_held + 1, s_held_count * sizeof(s_held[0]));
    if (!s_held_count) {
        s_next_send_us = 0;
        s_send_backoff_us = RETRY_MIN_US;
    }
}

typedef enum {
    FED,
    FEED_FAILED,    /* the turn failed, or Hatch stopped taking audio */
    FEED_PRESSED,   /* a press is queued: it goes first */
} feed_t;

/*
 * Gives Hatch a note from *sent on, waiting for room as it goes, then ends
 * the turn. Stops for a press only if `yield`.
 */
static feed_t feed_rest(const int16_t *pcm, size_t frames, size_t *sent, bool yield)
{
    char text[96];
    int64_t moved = esp_timer_get_time();
    for (;;) {
        muse_hatch_ev_t ev;
        while ((ev = muse_hatch_turn_event(text, sizeof(text))) != MUSE_HATCH_EV_NONE) {
            if (ev == MUSE_HATCH_EV_ERROR) {
                ESP_LOGW(TAG, "muse: %s", text);
                return FEED_FAILED;
            }
        }
        if (*sent == frames) {
            break;
        }
        if (yield && press_waiting()) {
            return FEED_PRESSED;
        }
        size_t n = muse_hatch_turn_audio_wait(pcm + *sent, frames - *sent, 100);
        *sent += n;
        int64_t now = esp_timer_get_time();
        if (n) {
            moved = now;
        } else if (now - moved > STALL_US) {
            ESP_LOGW(TAG, "Muse stopped taking the note");
            return FEED_FAILED;
        }
    }
    muse_hatch_turn_end();
    return FED;
}

/* Asleep: waits for the VM to have the note just sent. */
static bool wait_delivered(void)
{
    char text[96];
    int64_t give_up = esp_timer_get_time() + ACK_WAIT_US;
    while (esp_timer_get_time() < give_up) {
        muse_hatch_ev_t ev;
        while ((ev = muse_hatch_turn_event(text, sizeof(text))) != MUSE_HATCH_EV_NONE) {
            if (ev == MUSE_HATCH_EV_SENT || ev == MUSE_HATCH_EV_REPLY || ev == MUSE_HATCH_EV_DONE) {
                return true;
            }
            if (ev == MUSE_HATCH_EV_ERROR) {
                ESP_LOGW(TAG, "muse: %s", text);
                return false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    ESP_LOGW(TAG, "no word that the note arrived");
    return false;
}

/*
 * Sends the oldest saved note. Awake, it's a turn like any other, reply and
 * all; asleep (quiet), it's done once the VM has the note, and the reply
 * waits in the app. A press while it goes up comes first, and the note stays
 * saved. Returns true if a press interrupted the reply.
 */
static bool send_held(bool quiet)
{
    held_note_t *h = &s_held[0];
    ESP_LOGI(TAG, "sending a saved note%s: %.1fs, from %llds ago, try %d", quiet ? " (asleep)" : "",
             (double)h->frames / MUSE_AUDIO_RATE, (long long)((esp_timer_get_time() - h->at_us) / 1000000),
             h->tries + 1);
    if (!quiet) {
        muse_state_set_mode(MUSE_MODE_THINKING);
        muse_state_set_caption("SENDING SAVED NOTE");
    }
    muse_hatch_turn_begin();
    size_t sent = 0;
    feed_t fed = feed_rest(h->pcm, h->frames, &sent, true);
    bool delivered = false, interrupted = false;
    if (fed == FED && quiet) {
        delivered = wait_delivered();
    } else if (fed == FED) {
        interrupted = hatch_reply(&delivered);
    }
    if (fed != FED || quiet) {
        muse_hatch_turn_cancel();   /* asleep, the reply is left for the app */
    }
    if (fed == FEED_PRESSED) {
        if (!quiet) {
            go_idle("");
        }
        return false;
    }
    const char *caption;
    if (delivered || interrupted) {
        drop_oldest();
        s_next_send_us = 0;   /* the next one right away */
        s_send_backoff_us = RETRY_MIN_US;
        if (quiet) {
            muse_state_set_caption("SAVED NOTE SENT");
        }
        return interrupted;
    } else if (++h->tries >= HELD_TRIES) {
        ESP_LOGW(TAG, "giving up on a saved note after %d tries", h->tries);
        drop_oldest();
        caption = "COULDN'T SEND A SAVED NOTE";
    } else {
        back_off();
        caption = "SAVED NOTE: WILL TRY AGAIN";
    }
    if (quiet) {
        muse_state_set_caption("%s", caption);
    } else {
        go_idle(caption);
    }
    return false;
}

/*
 * Whether to send a saved note now: Hatch within reach and the backoff over.
 * Coming within reach starts the backoff over. Keeps s_waiting too.
 */
static bool held_due(void)
{
    static bool was_ready = true;
    if (!s_held_count) {
        was_ready = true;
        s_waiting = false;
        return false;
    }
    int64_t now = esp_timer_get_time();
    s_waiting = now - s_held[0].at_us < HELD_KEEP_US;
    bool ready = muse_hatch_ready();
    if (ready && !was_ready) {
        s_next_send_us = 0;
        s_send_backoff_us = RETRY_MIN_US;
    }
    was_ready = ready;
    return ready && now >= s_next_send_us;
}
#endif

/*
 * After the release: the rest of the note goes to Hatch and the reply plays,
 * or if it can't go now it's saved to send later. Returns true if a press
 * interrupted the reply.
 */
static bool finish_note(void)
{
    bool delivered;
#ifdef CONFIG_MUSE_STT_VOICEPROXY
    /* STT mode: transcribe the kept note on the voice-proxy and inject the
     * transcript as a turn whose reply is captioned and spoken. The note
     * goes nowhere else: the HOLD_NOTES retry machinery is bypassed, so a
     * failed transcription ends the turn. */
    if (muse_stt_voiceproxy_enabled()) {
        if (!s_rec || !s_rec_n) {
            ESP_LOGW(TAG, "stt: nothing recorded (s_rec=%p frames=%u)",
                     (void *)s_rec, (unsigned)s_rec_n);
            go_idle("DIDN'T CATCH THAT");
            return false;
        }
        muse_audio_chirp(0);
        muse_state_set_mode(MUSE_MODE_THINKING);
        muse_state_set_caption("TRANSCRIBING...");
        char text[512];
        bool ok = muse_stt_voiceproxy_transcribe(s_rec, s_rec_n, text, sizeof(text), 26000);
        drop_rec();
        if (s_hush) {
            /* The head was touched while the note was being transcribed:
             * that is a stop, not a question. The transcript is dropped
             * rather than sent to Muse. */
            s_hush = false;
            ESP_LOGI(TAG, "turn dropped by touch");
            go_idle("");
            return false;
        }
        if (!ok) {
            go_idle("DIDN'T CATCH THAT");
            return false;
        }
        if (!muse_chat_say_text(text)) {
            go_idle("CAN'T REACH MUSE");
            return false;
        }
        return hatch_reply(&delivered);
    }
#endif
#if HOLD_NOTES
    if (s_rec) {
        bool fed = false, interrupted = false;
        delivered = false;
        if (s_live && s_sent == s_rec_n) {
            muse_hatch_turn_end();   /* before the chirp, which takes ~90 ms */
            fed = true;
        }
        muse_audio_chirp(0);
        if (s_live && !fed) {
            /* Hatch is behind (still connecting, say): the rest from the kept note. */
            muse_state_set_mode(MUSE_MODE_THINKING);
            muse_state_set_caption("SENDING VOICE NOTE");
            fed = feed_rest(s_rec, s_rec_n, &s_sent, false) == FED;
            if (!fed) {
                muse_hatch_turn_cancel();
            }
        }
        if (fed) {
            interrupted = hatch_reply(&delivered);
        }
        if (delivered || interrupted) {
            drop_rec();
        } else {
            hold_rec(s_tried);
        }
        return interrupted;
    }
#endif
    muse_hatch_turn_end();   /* before the chirp, which takes ~90 ms */
    muse_audio_chirp(0);
    return hatch_reply(&delivered);
}

/*
 * Whether a press can record: Hatch within reach, or (HOLD_NOTES) set up
 * with room to save the note. If not, the caption says why.
 */
static bool can_record(void)
{
    if (!muse_wifi_connected()) {
        muse_wifi_apply();   /* retry now, not after the backoff */
    }
    bool ready = muse_hatch_ready();
#if HOLD_NOTES
    muse_hatch_status_t st;
    muse_hatch_status(&st);
    if (st.state == MUSE_HATCH_NOT_SET) {
        go_idle("SET UP MUSE FIRST");
        return false;
    }
    if ((!ready || s_held_count) && s_held_count >= HELD_MAX) {
        go_idle("NOTES STILL WAITING TO SEND");
        return false;
    }
    if (ready && s_held_count) {
        s_next_send_us = 0;   /* the saved ones go first, right after */
    }
    s_rec = heap_caps_malloc(MAX_FRAMES * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_rec) {
        ESP_LOGE(TAG, "no memory to keep a note");
    }
#endif
    if (!ready && !s_rec) {
        go_idle(not_ready_reason());
        return false;
    }
    return true;
}

static void voice_task(void *arg)
{
    bool pending_down = false;
    muse_audio_selftest();
    for (;;) {
        bool wake = false;
        bool voice = false;   /* the press came from a wake word, not a finger */
        if (!pending_down) {
            muse_input_event_t ev;
            bool asleep = muse_state_asleep();
            bool battery = muse_state_on_battery();
            bool rest = asleep && battery && !s_chirp && !s_mp3test && !s_loopback;
#ifdef CONFIG_MUSE_WAKE
            /* Asleep on battery, resting used to power the codecs down — and
             * with them the microphone, so the wake word could not be heard
             * at all. With the engine up, the codecs stay on and this loop
             * keeps reading: that is the price of always listening. Wi-Fi
             * still rests; the engine runs from the model partition, not the
             * network. Saying so costs the connection its slow-poll hint,
             * which is a fair description: it is listening, not resting. */
            bool listening = rest && muse_wake_listening();
#else
            bool listening = false;
#endif
#if HOLD_NOTES
            /* A press goes first: send_held() leaves it queued and returns
             * without backing off, so retrying before it's read would spin. */
            if (held_due() && !press_waiting()) {
                set_resting(false);   /* full power while it goes, even asleep */
                muse_wifi_power(MUSE_WIFI_FULL);
                pending_down = send_held(asleep);
                pre_reset();
                if (!pending_down && !asleep && muse_state_mode(NULL) != MUSE_MODE_IDLE) {
                    muse_state_make_happy();
                    go_idle("");
                }
                continue;
            }
#endif
            set_resting(rest && !listening);
            /* On battery Wi-Fi rests with the codecs, and between turns dozes
             * between beacons as xiaozhi's idle does; a press wakes it fully. */
            muse_wifi_power(rest ? MUSE_WIFI_REST : battery ? MUSE_WIFI_DOZE : MUSE_WIFI_FULL);
            if (rest && !listening) {
                /* Wait for waking, USB power or a bench request without
                 * polling (the timeout is only a backstop, or a look for
                 * Wi-Fi to send saved notes), so the CPU can stay asleep. */
                muse_state_wait_awake(s_waiting ? HELD_POLL_MS : REST_BACKSTOP_MS);
                continue;
            }
            if (s_chirp) {
                s_chirp = false;
                muse_audio_chirp(1);
                pre_reset();
            }
            if (s_mp3test) {
                s_mp3test = false;
                int16_t *pcm = NULL;
                size_t n = muse_hatch_mp3_selftest(&pcm);
                for (size_t i = 0; i < n; i += MUSE_AUDIO_CHUNK) {
                    muse_audio_write(pcm + i, n - i < MUSE_AUDIO_CHUNK ? n - i : MUSE_AUDIO_CHUNK);
                }
                free(pcm);
                pre_reset();
            }
            if (s_loopback) {
                s_loopback = false;
                muse_audio_loopback_test(muse_settings_volume());
                pre_reset();
            }
            /* The 20 ms read paces this loop. */
            idle_capture();
#ifdef CONFIG_MUSE_WAKE
            /* The wake word listens to the same chunk the pre-roll just got —
             * but not through the settle window after Muse has played
             * something: that tail is Muse's own voice, and it would be
             * listening to itself. The detector runs here, in this loop, so
             * it has to keep up with real time. */
            if (s_settle <= 0) {
                muse_wake_feed(s_chunk, MUSE_AUDIO_CHUNK, s_noise_db);
            }
#endif
            if (xQueueReceive(s_queue, &ev, 0) != pdTRUE) {
                continue;
            }
            muse_state_poke();
            if (ev.type != MUSE_PTT_DOWN) {
                continue;
            }
            wake = ev.wake;
            voice = ev.voice;
        }
        if (wake && !held_on_waking()) {
            continue;   /* a tap: it only woke Muse */
        }
        muse_wifi_power(MUSE_WIFI_FULL);
        if (!can_record()) {
            pending_down = false;
            continue;
        }
        if (voice) {
            /* The wake phrase is not part of the note; the last few chunks of
             * the ring are, in case the trigger point clipped the first one. */
            pre_keep(VAD_PREROLL_CHUNKS);
        }
#ifdef CONFIG_MUSE_WAKE
        /* Everything the engine has buffered belongs to the far side of this
         * turn — the phrase that opened it, or the room while it ran — and
         * must not be listened to again. Nothing is fed to it until the loop
         * comes back round to its idle read. */
        muse_wake_flush();
#endif
        size_t held;
        char why[96];
        bool ok = record(pending_down, voice, &held, why, sizeof(why));
        pending_down = false;
        if (held < MIN_HELD_FRAMES && !wake && !voice) {
            muse_hatch_turn_cancel();
            drop_rec();
            pre_reset();
            go_idle("HOLD LONGER TO TALK");
            continue;
        }
        if (!ok) {
            pre_reset();
            go_idle(why);   /* Hatch failed it while recording */
            continue;
        }
        pending_down = finish_note();
        pre_reset();
        if (!pending_down && muse_state_mode(NULL) != MUSE_MODE_IDLE) {
            muse_state_make_happy();
            go_idle("");
        }
    }
}

esp_err_t muse_voice_start(QueueHandle_t queue)
{
    s_queue = queue;
    s_pre = heap_caps_malloc(PRE_CHUNKS * sizeof(pre_chunk_t), MUSE_BIG_CAPS);
    if (!s_pre || muse_audio_init(muse_settings_volume(), muse_settings_mic_gain()) != ESP_OK) {
        muse_state_set_mode(MUSE_MODE_ERROR);
        muse_state_set_caption("AUDIO INIT FAILED");
        return ESP_FAIL;
    }
    /* Stack in PSRAM if there is any (this task never writes flash) to spare internal RAM for Wi-Fi/BLE. */
    if (xTaskCreatePinnedToCoreWithCaps(voice_task, "muse_voice", 6144, NULL, 6, NULL, MUSE_AUDIO_CORE,
                                        MUSE_BIG_CAPS) != pdPASS) {
        /* Without the task nothing would ever rest the codecs, which
         * muse_audio_init() left open: on the Watcher that's ~33 mA. */
        muse_audio_power(false);
        muse_state_set_mode(MUSE_MODE_ERROR);
        muse_state_set_caption("AUDIO INIT FAILED");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void muse_voice_set_monitor(bool on)
{
    s_monitor_db = -100.0f;
    s_monitor = on;
}

float muse_voice_monitor_db(void)
{
    return s_monitor_db;
}

void muse_voice_request_chirp(void)
{
    s_chirp = true;
    muse_state_nudge();
}

void muse_voice_request_loopback(void)
{
    s_loopback = true;
    muse_state_nudge();
}

void muse_voice_request_mp3test(void)
{
    s_mp3test = true;
    muse_state_nudge();
}

void muse_voice_request_hush(void)
{
    s_hush = true;
    muse_state_nudge();
}

bool muse_voice_resting(void)
{
    return s_resting;
}

bool muse_voice_notes_waiting(void)
{
    return s_waiting;
}
