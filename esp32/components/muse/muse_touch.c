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

#include "muse_touch.h"

#define TOUCH_IDLE 0
#define TOUCH_DOWN 1
#define TOUCH_SWIPING 2

/* A light finger does not hold an intensity of 1 steadily: the panel drops to
 * 0 for a poll or two and picks the finger back up. Taking that as a lift
 * ended the gesture and made the next sample a *new* press — which, once a
 * turn was listening, hushed it, so every touch started a turn and then
 * immediately threw it away. Four polls of nothing (200 ms) is the release. */
#define TOUCH_RELEASE_FRAMES 4

void muse_touch_reset(muse_touch_t *t, int swipe_threshold)
{
    t->swipe_threshold = swipe_threshold > 0 ? swipe_threshold : 40;
    t->release_frames = TOUCH_RELEASE_FRAMES;
    t->state = TOUCH_IDLE;
    t->start_pos = 0;
    t->up_run = 0;
}

bool muse_touch_down(const uint8_t intensity[3])
{
    return intensity[0] || intensity[1] || intensity[2];
}

int muse_touch_position(const uint8_t intensity[3])
{
    int total = intensity[0] + intensity[1] + intensity[2];
    if (!total) {
        return 0;
    }
    /* Left zone pulls to -100, right to +100, centre by how much it sees. */
    return (intensity[0] * -100 + intensity[2] * 100) / total;
}

muse_touch_gesture_t muse_touch_step(muse_touch_t *t, const uint8_t intensity[3])
{
    bool down = muse_touch_down(intensity);
    switch (t->state) {
    case TOUCH_IDLE:
        if (down) {
            t->state = TOUCH_DOWN;
            t->start_pos = muse_touch_position(intensity);
            t->up_run = 0;
            return MUSE_TOUCH_PRESS;
        }
        return MUSE_TOUCH_NONE;

    case TOUCH_DOWN:
        if (!down) {
            if (++t->up_run < t->release_frames) {
                return MUSE_TOUCH_NONE;   /* a gap in a finger, not a lift */
            }
            t->state = TOUCH_IDLE;
            t->up_run = 0;
            return MUSE_TOUCH_RELEASE;
        }
        t->up_run = 0;
        if (muse_touch_position(intensity) - t->start_pos > t->swipe_threshold) {
            t->state = TOUCH_SWIPING;
            return MUSE_TOUCH_SWIPE_FWD;
        }
        if (muse_touch_position(intensity) - t->start_pos < -t->swipe_threshold) {
            t->state = TOUCH_SWIPING;
            return MUSE_TOUCH_SWIPE_BACK;
        }
        return MUSE_TOUCH_NONE;

    default:   /* TOUCH_SWIPING: wait for the finger to lift */
        if (!down) {
            if (++t->up_run < t->release_frames) {
                return MUSE_TOUCH_NONE;
            }
            t->state = TOUCH_IDLE;
            t->up_run = 0;
            return MUSE_TOUCH_RELEASE;
        }
        t->up_run = 0;
        return MUSE_TOUCH_NONE;
    }
}
