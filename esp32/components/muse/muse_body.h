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
#include <stddef.h>
#include <stdint.h>

/*
 * The M5 StackChan body under the CoreS3: two SCS bus servos (yaw and pitch),
 * a 3-zone capacitive head panel (Si12T) and 12 RGB LEDs. The servos need
 * their rail switched on through the body's IO expander; the LEDs are driven
 * by that same expander. Nothing here is fatal: a CoreS3 without the body
 * logs once and every call becomes a no-op.
 *
 * The servos are on UART1 at 1 Mbps (TX GPIO6, RX GPIO7); the expander and
 * the touch chip share the board's I2C bus (0x6F and 0x68). Angles are in
 * tenths of a degree, matching the factory firmware: yaw -1280..1280, pitch
 * 30..870.
 */

typedef enum {
    MUSE_BODY_POSE_IDLE = 0,   /* centred, still */
    MUSE_BODY_POSE_LISTEN,     /* frozen: the microphones are live */
    MUSE_BODY_POSE_THINK,      /* tilts to one side, then the other */
    MUSE_BODY_POSE_SPEAK,      /* gentle sway while a reply plays */
} muse_body_pose_t;

/* Starts the body. Call once, from the boot task (it touches NVS and I2C).
 * False when no body answers; everything else keeps working. */
bool muse_body_start(void);

/* Whether the body's IO expander answered at boot. */
bool muse_body_present(void);

/* Target pose. Sets a target only — the body task writes the servo frames —
 * and LISTEN freezes motion outright (the motors share the body with the
 * microphones). No-op without a body. */
void muse_body_set_pose(muse_body_pose_t pose);

/* The wake perk, run before the microphone opens: moves the head and blocks
 * ~300 ms so the servos are still when recording starts. No-op, and no
 * delay, without a body. */
void muse_body_pre_listen(void);

/* Bench and console. Angles are tenths of a degree and are clamped. */
bool muse_body_move(int yaw_tenths, int pitch_tenths);
void muse_body_leds(uint8_t r, uint8_t g, uint8_t b);
bool muse_body_zero(int yaw_raw, int pitch_raw);

/* Sets the touch sensitivity in the running device (1 = most sensitive,
 * 7 = least). False if the value is out of range or no head panel answered. */
bool muse_body_set_sens(int level);

/* Bench-only: writes a test string to the servo UART and reads it back from
 * the TX line itself. True if every byte comes back identical — i.e. the
 * UART controller and the 1 Mbps timing are good on this board. */
bool muse_body_loopback(void);

/* What the two servos say about themselves — torque, position, moving,
 * voltage, temperature — as one line, for the console. This is the only
 * way to tell "not moving" from "not listening". */
int muse_body_read(char *out, size_t cap);

/* One line of JSON for the console: what answered, where the head is, what
 * the head panel sees. Returns the length written. */
int muse_body_status(char *out, size_t cap);

/* Every address that answers on the board's I2C bus, as "6f,68" — the body's
 * expander is 0x6F and its head panel 0x68, so this says whether anything of
 * the body is there at all. Returns the length written. */
int muse_body_scan(char *out, size_t cap);
