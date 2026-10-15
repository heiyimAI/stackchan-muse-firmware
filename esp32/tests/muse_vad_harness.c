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

#include <stdio.h>

#include "muse_vad.h"

/*
 * Hand-computed scenarios for the energy VAD. Configuration mirrors the
 * Kconfig defaults: drop 18 dB below the speech peak, floor -45 dBFS,
 * 15 chunks (300 ms) of speech to arm, 15 chunks of silence to stop.
 *
 * The threshold at each step is max(peak_db - 18, -45), where peak_db is
 * the running peak of the chunks fed so far. Every expectation below is
 * derived from that rule by hand, one chunk at a time. Which of the two
 * binds is the point of half these scenarios: the floor for anything at
 * or below -27 dBFS peak (the quiet speaker), the drop for anything louder.
 */

static int failures;

static void expect(bool ok, const char *what)
{
    if (!ok) {
        fprintf(stderr, "FAIL %s\n", what);
        failures++;
    }
}

static muse_vad_t make(void)
{
    muse_vad_t v = {
        .drop_db = 18.0f,
        .floor_db = -45.0f,
        .min_speech_chunks = 15,
        .silence_chunks = 15,
    };
    muse_vad_reset(&v);
    return v;
}

/* Feeds n chunks at db; returns how many steps fired (0 or 1 expected). */
static int feed(muse_vad_t *v, float db, int n)
{
    int fired = 0;
    for (int i = 0; i < n; i++) {
        if (muse_vad_step(v, db)) {
            fired++;
        }
    }
    return fired;
}

/* Speech, then silence long enough to stop exactly on the 15th silent chunk. */
static void test_normal(void)
{
    muse_vad_t v = make();
    expect(feed(&v, -20.0f, 20) == 0, "normal: speech alone does not stop");
    expect(v.armed, "normal: 20 speech chunks arm the stop");
    expect(feed(&v, -60.0f, 14) == 0, "normal: 14 silent chunks are not enough");
    expect(feed(&v, -60.0f, 1) == 1, "normal: the 15th silent chunk stops");
    expect(v.spoke, "normal: spoke is set");
}

/* Speech shorter than min_speech_chunks never arms; silence cannot stop it. */
static void test_no_arm(void)
{
    muse_vad_t v = make();
    expect(feed(&v, -20.0f, 10) == 0, "no_arm: 10 chunks do not fire");
    expect(!v.armed, "no_arm: stop is not armed");
    expect(feed(&v, -60.0f, 100) == 0, "no_arm: silence alone never stops");
    expect(v.spoke, "no_arm: the short speech still counts as spoken");
}

/* A pause shorter than silence_chunks resets and speech resumes. */
static void test_mid_pause(void)
{
    muse_vad_t v = make();
    expect(feed(&v, -20.0f, 20) == 0, "mid_pause: speech");
    expect(feed(&v, -60.0f, 10) == 0, "mid_pause: a 200 ms pause does not stop");
    expect(feed(&v, -20.0f, 10) == 0, "mid_pause: speech resumes");
    expect(v.silence_run == 0, "mid_pause: the pause's silence run was reset");
    expect(feed(&v, -60.0f, 15) == 1, "mid_pause: the final silence stops");
}

/* A quiet room: everything below the floor, so nothing is speech. */
static void test_quiet_room(void)
{
    muse_vad_t v = make();
    expect(feed(&v, -60.0f, 200) == 0, "quiet_room: noise never stops");
    expect(!v.spoke, "quiet_room: nothing counted as speech");
    expect(!v.armed, "quiet_room: never armed");
}

/* A soft voice: peak -40 keeps the floor as the threshold (-40-18 = -58 < -45). */
static void test_soft_voice(void)
{
    muse_vad_t v = make();
    expect(feed(&v, -40.0f, 20) == 0, "soft: speech");
    expect(v.armed, "soft: armed");
    expect(feed(&v, -56.0f, 1) == 0, "soft: -56 is below the floor");
    expect(feed(&v, -56.0f, 14) == 1, "soft: silence ends it");
}

/* The threshold is inclusive: a chunk exactly at the threshold is speech.
 * This is the loud-speaker case, where the drop (-43 from -25), not the
 * floor, is the threshold. */
static void test_threshold_edges(void)
{
    muse_vad_t v = make();
    expect(feed(&v, -25.0f, 20) == 0, "edges: speech at -25");
    /* threshold now max(-43, -45) = -43 */
    expect(feed(&v, -43.0f, 20) == 0, "edges: exactly at the threshold is speech");
    expect(v.silence_run == 0, "edges: still no silence run");
    expect(feed(&v, -43.1f, 15) == 1, "edges: just below the threshold is silence");
}

/* Loud speech: the threshold follows the peak (-10 -> -28), not the floor. */
static void test_loud(void)
{
    muse_vad_t v = make();
    expect(feed(&v, -10.0f, 20) == 0, "loud: speech at -10");
    expect(feed(&v, -28.0f, 1) == 0, "loud: at the threshold still speech");
    expect(feed(&v, -40.0f, 14) == 0, "loud: well below the threshold counts");
    expect(feed(&v, -40.0f, 1) == 1, "loud: ends on the 15th");
}

/* Speech that never holds for 300 ms in a row — a quiet voice, or simply the
 * dips between words — still arms, so its note can end. Counting the chunks
 * in a row would run this one to the cap. */
static void test_dipped_speech_arms(void)
{
    muse_vad_t v = make();
    for (int i = 0; i < 20; i++) {
        feed(&v, -30.0f, 1);   /* 20 ms of voice: above the -45 floor (peak-18 = -48 does not bind) */
        feed(&v, -50.0f, 1);   /* 20 ms of a dip below it */
    }
    expect(v.armed, "dipped: 20 speech chunks in all arm it");
    expect(v.spoke, "dipped: spoke is set");
    /* Armed on the 15th speech chunk; only the dip after it is silence —
     * every speech chunk resets the run. */
    expect(v.silence_run == 1, "dipped: the dip after arming counted");
    expect(feed(&v, -60.0f, 13) == 0, "dipped: 14 in all are not enough");
    expect(feed(&v, -60.0f, 1) == 1, "dipped: the 15th ends it");
}

/* The rule behind it all, which the device logs for tuning the floor: the
 * floor until the peak outgrows it (-27 dBFS and quieter), the drop above. */
static void test_threshold_rule(void)
{
    muse_vad_t v = make();
    expect(muse_vad_threshold(&v) == -45.0f, "threshold: the floor before anything is heard");
    feed(&v, -40.0f, 1);   /* peak -40: -58 is under the floor */
    expect(muse_vad_threshold(&v) == -45.0f, "threshold: the floor for a soft voice");
    feed(&v, -20.0f, 1);   /* peak -20: -38 is over it */
    expect(muse_vad_threshold(&v) == -38.0f, "threshold: the drop for a loud one");
    expect(v.armed == false, "threshold: one chunk of each never arms");
}

int main(void)
{
    test_normal();
    test_no_arm();
    test_mid_pause();
    test_quiet_room();
    test_soft_voice();
    test_threshold_edges();
    test_loud();
    test_dipped_speech_arms();
    test_threshold_rule();
    if (failures) {
        fprintf(stderr, "%d failure(s)\n", failures);
        return 1;
    }
    printf("muse_vad: all scenarios pass\n");
    return 0;
}
