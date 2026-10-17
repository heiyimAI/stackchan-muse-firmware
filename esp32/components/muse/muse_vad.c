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

#include "muse_vad.h"

#include <math.h>
#include <string.h>

/* The wake guard's window and margin, as esp32-thinks settled on them. */
#define ENERGY_WINDOW_MS 256
#define ENERGY_OVER_DB 12.0f

void muse_energy_reset(muse_energy_t *e, unsigned step_ms)
{
    memset(e->power, 0, sizeof e->power);
    e->at = 0;
    e->count = 0;
    e->step_ms = step_ms ? step_ms : 20;
}

void muse_energy_feed(muse_energy_t *e, float db)
{
    e->power[e->at] = powf(10.0f, db / 10.0f);
    e->at = (e->at + 1) % (sizeof e->power / sizeof e->power[0]);
    if (e->count < sizeof e->power / sizeof e->power[0]) {
        e->count++;
    }
}

bool muse_energy_accept(const muse_energy_t *e, float floor_db)
{
    const unsigned cap = (unsigned)(sizeof e->power / sizeof e->power[0]);
    unsigned width = ENERGY_WINDOW_MS / e->step_ms;
    if (width < 1) {
        width = 1;
    }
    if (width > cap) {
        width = cap;
    }
    if (e->count < width) {
        return false;   /* not enough heard yet to judge any window */
    }
    unsigned oldest = (e->at + cap - e->count) % cap;
    double sum = 0, best = 0;
    for (unsigned i = 0; i < e->count; i++) {
        sum += e->power[(oldest + i) % cap];
        if (i >= width) {
            sum -= e->power[(oldest + i - width) % cap];
        }
        if (i + 1 >= width && sum > best) {
            best = sum;
        }
    }
    /* Compare the best window's mean power with the room, 12 dB up. */
    return best / width >= powf(10.0f, (floor_db + ENERGY_OVER_DB) / 10.0f);
}

void muse_vad_reset(muse_vad_t *v)
{
    v->peak_db = -100.0f;
    v->speech_chunks = 0;
    v->silence_run = 0;
    v->armed = false;
    v->spoke = false;
}

float muse_vad_threshold(const muse_vad_t *v)
{
    float threshold = v->peak_db - v->drop_db;
    return threshold < v->floor_db ? v->floor_db : threshold;
}

bool muse_vad_step(muse_vad_t *v, float db)
{
    float threshold = muse_vad_threshold(v);
    if (db >= threshold) {
        v->peak_db = db > v->peak_db ? db : v->peak_db;
        v->silence_run = 0;
        v->spoke = true;
        if (++v->speech_chunks >= v->min_speech_chunks) {
            v->armed = true;
        }
    } else if (v->armed) {
        v->silence_run++;
    }
    return v->armed && v->silence_run >= v->silence_chunks;
}
