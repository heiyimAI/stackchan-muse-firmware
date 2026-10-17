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

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/*
 * The wake word: esp-sr's WakeNet model on its own, listening to the same
 * 20 ms microphone chunks the voice task reads while it is idle. A detection
 * starts a turn with nobody's finger on the button — muse_input_voice_trigger
 * — so the note ends on the speech itself (muse_vad.h).
 *
 * Not esp-sr's AFE. The AFE is a whole front end (echo cancelling, noise
 * suppression, its own task and ring buffers), built for boards with more
 * internal RAM to spare than this one has; only its wake word is wanted
 * here, and a WakeNet model can be asked directly. That keeps the engine to
 * a model handle, a one-chunk buffer in PSRAM, and the detect() call in the
 * voice task's own loop — which is how esp32-thinks runs one on a board with
 * less RAM still.
 *
 * Nothing here is fatal: with no model in the model partition, or no memory
 * for the one it found, it logs and leaves the button as the only way in.
 */

/* Starts the engine: mounts the model partition and creates the model.
 * Must run on a task whose stack is in internal RAM — mounting the partition
 * maps it into the flash cache, which IDF cannot do from a PSRAM stack.
 * muse_app_run() runs on the boot task, which is. */
bool muse_wake_start(QueueHandle_t queue);

/* Whether the engine is up and could start a turn right now. The voice task
 * asks this before resting the codecs: asleep, they are what keeps the
 * microphone — and so the wake word — alive. */
bool muse_wake_listening(void);

/* One 20 ms chunk of microphone audio, exactly as the idle read got it, and
 * the room's level (`floor_db`, where the voice task's adaptive noise floor
 * has settled). Never blocks; the detector runs here, so it must keep up
 * with real time. */
void muse_wake_feed(const int16_t *pcm, size_t frames, float floor_db);

/* Throws away everything buffered and the detector's own state. Call at a
 * turn boundary: the wake phrase, and the room while it was said, must not
 * be listened to twice. */
void muse_wake_flush(void);

/* Overrides the model's own detection threshold (0.4 to 0.9999, higher is
 * stricter). The model ships with one tuned by Espressif — the boot log
 * prints what it is — and this is for field tuning a phrase the model was
 * not trained on. False if there is no model, or the value is out of range. */
bool muse_wake_set_threshold(float threshold);

/* What the model listens for, for the console: its first word name (or ""
 * when there is none) and the threshold in force. */
const char *muse_wake_word(void);
float muse_wake_threshold(void);
