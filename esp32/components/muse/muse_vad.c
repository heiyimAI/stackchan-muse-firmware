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
