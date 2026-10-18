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

#include "muse_touch.h"

/*
 * Hand-worked scenarios for the head panel's gesture machine. The Si12T
 * reports two bits of intensity per zone (0..3), so the position is
 *     (left * -100 + right * 100) / (left + centre + right)
 * and a swipe is that position moving more than the threshold while the
 * finger stays down.
 */

static int failures;

static void expect(bool ok, const char *what)
{
    if (!ok) {
        fprintf(stderr, "FAIL %s\n", what);
        failures++;
    }
}

static muse_touch_t make(void)
{
    muse_touch_t t;
    muse_touch_reset(&t, 40);
    return t;
}

/* A tap: press on contact, release when the finger lifts, nothing in between. */
static void test_tap(void)
{
    muse_touch_t t = make();
    const uint8_t centre[3] = { 0, 1, 0 };
    const uint8_t none[3] = { 0, 0, 0 };
    expect(muse_touch_step(&t, none) == MUSE_TOUCH_NONE, "tap: untouched is quiet");
    expect(muse_touch_step(&t, centre) == MUSE_TOUCH_PRESS, "tap: contact is a press");
    expect(muse_touch_step(&t, centre) == MUSE_TOUCH_NONE, "tap: holding is quiet");
    expect(muse_touch_step(&t, none) == MUSE_TOUCH_RELEASE, "tap: lifting is a release");
    expect(muse_touch_step(&t, none) == MUSE_TOUCH_NONE, "tap: and then quiet again");
}

/* A finger dragged from the middle towards the right zone. */
static void test_swipe_forward(void)
{
    muse_touch_t t = make();
    const uint8_t centre[3] = { 0, 3, 0 };   /* position 0 */
    const uint8_t right[3] = { 0, 0, 3 };    /* position +100 */
    const uint8_t none[3] = { 0, 0, 0 };
    expect(muse_touch_step(&t, centre) == MUSE_TOUCH_PRESS, "fwd: press at centre");
    expect(muse_touch_step(&t, right) == MUSE_TOUCH_SWIPE_FWD, "fwd: +100 is a swipe");
    expect(muse_touch_step(&t, right) == MUSE_TOUCH_NONE, "fwd: one swipe per touch");
    expect(muse_touch_step(&t, none) == MUSE_TOUCH_RELEASE, "fwd: lift ends it");
}

/* The other way: from the right zone to the left one. */
static void test_swipe_back(void)
{
    muse_touch_t t = make();
    const uint8_t right[3] = { 0, 0, 3 };
    const uint8_t left[3] = { 3, 0, 0 };
    expect(muse_touch_step(&t, right) == MUSE_TOUCH_PRESS, "back: press at right");
    expect(muse_touch_step(&t, left) == MUSE_TOUCH_SWIPE_BACK, "back: -200 is a swipe");
}

/* A steady hold whose position wobbles a little is not a swipe. */
static void test_hold(void)
{
    muse_touch_t t = make();
    const uint8_t a[3] = { 1, 1, 0 };   /* -50 */
    const uint8_t b[3] = { 1, 2, 0 };   /* -33 */
    expect(muse_touch_step(&t, a) == MUSE_TOUCH_PRESS, "hold: press");
    expect(muse_touch_step(&t, b) == MUSE_TOUCH_NONE, "hold: 17 of drift is not a swipe");
    expect(muse_touch_step(&t, a) == MUSE_TOUCH_NONE, "hold: and back is not one either");
}

/* The threshold is a boundary, not a suggestion. */
static void test_threshold(void)
{
    muse_touch_t t = make();
    const uint8_t centre[3] = { 0, 3, 0 };   /* 0 */
    const uint8_t drift[3] = { 1, 2, 0 };    /* -33 */
    const uint8_t past[3] = { 2, 1, 0 };     /* -66 */
    expect(muse_touch_step(&t, centre) == MUSE_TOUCH_PRESS, "edge: press");
    expect(muse_touch_step(&t, drift) == MUSE_TOUCH_NONE, "edge: -33 is under 40");
    expect(muse_touch_step(&t, past) == MUSE_TOUCH_SWIPE_BACK, "edge: -66 is over it");
}

/* The position itself: the three zones as one number. */
static void test_position(void)
{
    const uint8_t l[3] = { 3, 0, 0 };
    const uint8_t r[3] = { 0, 0, 3 };
    const uint8_t c[3] = { 1, 1, 1 };
    const uint8_t n[3] = { 0, 0, 0 };
    const uint8_t both[3] = { 1, 0, 1 };
    expect(muse_touch_position(l) == -100, "pos: left is -100");
    expect(muse_touch_position(r) == 100, "pos: right is +100");
    expect(muse_touch_position(c) == 0, "pos: all three is 0");
    expect(muse_touch_position(n) == 0, "pos: none is 0");
    expect(muse_touch_position(both) == 0, "pos: both ends cancel");
    expect(!muse_touch_down(n), "pos: nothing down");
    expect(muse_touch_down(c), "pos: something down");
}

int main(void)
{
    test_tap();
    test_swipe_forward();
    test_swipe_back();
    test_hold();
    test_threshold();
    test_position();
    if (failures) {
        fprintf(stderr, "%d failure(s)\n", failures);
        return 1;
    }
    printf("muse_touch: all scenarios pass\n");
    return 0;
}
