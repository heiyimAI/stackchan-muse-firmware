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
#include <stdint.h>

/*
 * The StackChan body's 3-zone head panel (Si12T), as gestures. The chip
 * reports two bits of intensity per zone — 0 (not touched) to 3 — so the
 * "position" of a touch is the weighted average of the three zones, -100
 * (left) to +100 (right), and a swipe is that position moving far enough
 * while a finger stays down.
 *
 * Pure state and arithmetic, no I2C — the driver is in muse_body.c and the
 * decisions here are host-testable (tests/muse_touch_harness.c).
 */

typedef enum {
    MUSE_TOUCH_NONE = 0,
    MUSE_TOUCH_PRESS,
    MUSE_TOUCH_RELEASE,
    MUSE_TOUCH_SWIPE_FWD,    /* position moved +threshold while held */
    MUSE_TOUCH_SWIPE_BACK,
} muse_touch_gesture_t;

typedef struct {
    /* configuration */
    int swipe_threshold;   /* how far the position must move, of ±100 */

    /* state */
    int state;             /* 0 idle, 1 touched, 2 swiping */
    int start_pos;
} muse_touch_t;

void muse_touch_reset(muse_touch_t *t, int swipe_threshold);

/* Weighted position of the three zones, -100 (left) to +100 (right). */
int muse_touch_position(const uint8_t intensity[3]);

/* One poll's worth of intensities in, at most one gesture out. */
muse_touch_gesture_t muse_touch_step(muse_touch_t *t, const uint8_t intensity[3]);

/* Whether the panel counts this poll as a finger down. */
bool muse_touch_down(const uint8_t intensity[3]);
