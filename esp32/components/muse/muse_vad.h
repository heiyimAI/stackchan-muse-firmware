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

#pragma once

#include <stdbool.h>

/*
 * A tiny energy VAD over one 20 ms chunk's dBFS level, for turns with no
 * finger to release (the wake word): speech ends after `silence_chunks`
 * consecutive chunks below a threshold relative to the running speech peak.
 * Pure state and arithmetic — no esp-sr, no FreeRTOS — so record()'s use of
 * it is host-testable (tests/muse_vad_harness.c).
 *
 * One chunk is speech when its level is at or above
 *     max(peak_db - drop_db, floor_db)
 * The floor keeps a quiet room's noise from counting as speech; the relative
 * part adapts to how loud the speaker actually is. A stop is only armed once
 * min_speech_chunks of speech have been heard in all — in all, not in a row,
 * because the dip between two words of one sentence would otherwise keep a
 * quiet speaker from arming and their note would run to the cap. Scattered
 * chunks of room noise reach that count slowly and cannot arm a note;
 * min_speech_chunks of a voice reach it in a fraction of a second.
 */

typedef struct {
    /* configuration — set by the caller, kept across resets */
    float drop_db;           /* stop threshold: peak minus this          */
    float floor_db;          /* the threshold never goes below this      */
    int min_speech_chunks;   /* speech needed before a stop can fire     */
    int silence_chunks;      /* consecutive silent chunks that end it    */

    /* state */
    float peak_db;           /* running speech peak (dBFS)               */
    int speech_chunks;       /* chunks above the threshold, in total     */
    int silence_run;         /* chunks below the threshold in a row      */
    bool armed;              /* enough speech seen: silence may stop it  */
    bool spoke;              /* any speech seen at all this turn         */
} muse_vad_t;

/* Clears the state; the configuration fields are left alone. */
void muse_vad_reset(muse_vad_t *v);

/* The level a chunk has to reach to count as speech right now — the rule
 * muse_vad_step() applies, for logging and for callers that want to show
 * the speaker how far off they are. */
float muse_vad_threshold(const muse_vad_t *v);

/* Feeds one chunk's dBFS. True once speech has ended — sticky, so it keeps
 * returning true until reset; the caller stops recording on the first one. */
bool muse_vad_step(muse_vad_t *v, float db);

/*
 * The wake word's second opinion. WakeNet reports after it has heard its
 * phrase, and a burst of noise can score just over its threshold; this keeps
 * the last second of chunk levels and accepts only if some 256 ms of it was
 * 12 dB above the room. Same idea as esp32-thinks' wake guard, same numbers.
 */
typedef struct {
    float power[32];     /* the last ~1 s, as power so windows can be averaged */
    unsigned at, count;
    unsigned step_ms;    /* what one entry covers, for the window in samples */
} muse_energy_t;

void muse_energy_reset(muse_energy_t *e, unsigned step_ms);
void muse_energy_feed(muse_energy_t *e, float db);
bool muse_energy_accept(const muse_energy_t *e, float floor_db);
