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

#include "muse_wake.h"

#include <math.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "model_path.h"

#include "muse_audio.h"
#include "muse_chat.h"
#include "muse_input.h"
#include "muse_mem.h"
#include "muse_state.h"
#include "muse_vad.h"

static const char *TAG = "muse_wake";

/* Where the model partition is mounted: whatever esp-sr packed into
 * srmodels.bin, chosen by esp-sr's own menuconfig (CONFIG_SR_WN_*). */
#define MODEL_PARTITION "model"
/* A detection is a turn. The two seconds after it belong to the phrase's own
 * tail and to whatever Muse starts saying back: neither may start another. */
#define MUTE_US 2000000LL

static const esp_wn_iface_t *s_wn;
static model_iface_data_t *s_model;
static int16_t *s_buf;         /* one detection's worth, PSRAM */
static int s_need;             /* samples per detect() */
static unsigned s_step_ms;     /* how much audio one detect() look covers */
static int s_fill;             /* samples held for the next one */
static int64_t s_muted_until_us;
static muse_energy_t s_energy;
static char s_word[64];        /* what the model says it listens for */

bool muse_wake_start(QueueHandle_t queue)
{
    (void)queue;   /* the trigger goes through muse_input, which owns the queue */
    if (s_model) {
        return true;
    }
    srmodel_list_t *models = esp_srmodel_init(MODEL_PARTITION);
    if (!models) {
        ESP_LOGW(TAG, "no model partition: the button is the only way in");
        return false;
    }
    char *name = esp_srmodel_filter(models, ESP_WN_PREFIX, NULL);
    if (!name) {
        ESP_LOGW(TAG, "no wake word model in the partition: the button is the only way in");
        return false;
    }
    s_wn = esp_wn_handle_from_name(name);
    if (!s_wn) {
        ESP_LOGW(TAG, "no handle for %s: the button is the only way in", name);
        return false;
    }
    s_model = s_wn->create(name, DET_MODE_90);
    if (!s_model) {
        ESP_LOGW(TAG, "%s would not start (internal %u free): the button is the only way in",
                 name, (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        return false;
    }
    s_need = s_wn->get_samp_chunksize(s_model);
    s_buf = s_need > 0 ? heap_caps_malloc((size_t)s_need * sizeof(int16_t), MUSE_BIG_CAPS) : NULL;
    if (!s_buf) {
        ESP_LOGW(TAG, "no memory for the detector's buffer: the button is the only way in");
        s_wn->destroy(s_model);
        s_model = NULL;
        return false;
    }
    int rate = s_wn->get_samp_rate(s_model);
    s_step_ms = (unsigned)((1000L * s_need) / (rate > 0 ? rate : MUSE_AUDIO_RATE));
    s_fill = 0;
    muse_energy_reset(&s_energy, s_step_ms);

    /* The model knows what it listens for and how sure it has to be; say so,
     * because on the device this log is the only place that is visible. */
    int words = s_wn->get_word_num(s_model);
    for (int i = 1; i <= words; i++) {
        const char *w = s_wn->get_word_name(s_model, i);
        ESP_LOGI(TAG, "wake word %d: \"%s\", threshold %.3f", i, w ? w : "?",
                 (double)s_wn->get_det_threshold(s_model, i));
    }
    const char *word = words > 0 ? s_wn->get_word_name(s_model, 1) : NULL;
    strlcpy(s_word, word ? word : "", sizeof s_word);
#if CONFIG_MUSE_WAKE_THRESHOLD_PCT > 0
    s_wn->set_det_threshold(s_model, (float)CONFIG_MUSE_WAKE_THRESHOLD_PCT / 100.0f, 1);
    ESP_LOGI(TAG, "threshold lowered to %d%% to hear a phrase the model was not trained on",
             CONFIG_MUSE_WAKE_THRESHOLD_PCT);
#endif
    ESP_LOGI(TAG, "listening for \"%s\": %d Hz, %d samples a look, threshold %.3f",
             s_word, s_wn->get_samp_rate(s_model), s_need,
             (double)s_wn->get_det_threshold(s_model, 1));
    return true;
}

bool muse_wake_listening(void)
{
    return s_model != NULL;
}

const char *muse_wake_word(void)
{
    return s_word;
}

float muse_wake_threshold(void)
{
    return s_model ? s_wn->get_det_threshold(s_model, 1) : 0.0f;
}

bool muse_wake_set_threshold(float threshold)
{
    if (!s_model || !s_wn->set_det_threshold) {
        return false;
    }
    if (!(threshold >= 0.4f && threshold <= 0.9999f)) {
        return false;
    }
    s_wn->set_det_threshold(s_model, threshold, 1);
    /* The call's return value is not documented; read it back instead. */
    ESP_LOGI(TAG, "threshold now %.4f (was %.4f)", (double)muse_wake_threshold(), (double)threshold);
    return fabsf(muse_wake_threshold() - threshold) < 0.001f;
}

/* The detector said its phrase. Start a turn, or say why not. */
static void on_wake(void)
{
    s_muted_until_us = esp_timer_get_time() + MUTE_US;
    muse_energy_reset(&s_energy, s_step_ms);
    /*
     * Deliberately not s_wn->clean(): the model's clean() walks a queue that
     * only exists once a detection has been through it, and dereferences it
     * blind — calling it early (or twice) panics the device
     * (dl_convq_queue_bzero, EXCVADDR 0x10). Nothing here needs it: the
     * detector's own context is about a second of audio, and the mute above
     * covers two, so by the time it listens again the old phrase is gone.
     */
    if (muse_state_mode(NULL) != MUSE_MODE_IDLE) {
        return;   /* a turn is already running */
    }
    muse_hatch_status_t st;
    muse_hatch_status(&st);
    if (st.state == MUSE_HATCH_NOT_SET) {
        ESP_LOGI(TAG, "wake word heard, but Muse is not set up");
        return;
    }
    ESP_LOGI(TAG, "wake word heard");
    muse_input_voice_trigger("voice");
}

void muse_wake_feed(const int16_t *pcm, size_t frames, float floor_db)
{
    if (!s_model || !pcm || !frames) {
        return;
    }
    if (esp_timer_get_time() < s_muted_until_us) {
        return;   /* just after a turn: the room's tail must not re-trigger */
    }
    while (frames > 0) {
        int room = s_need - s_fill;
        int take = frames < (size_t)room ? (int)frames : room;
        memcpy(s_buf + s_fill, pcm, (size_t)take * sizeof(int16_t));
        s_fill += take;
        pcm += take;
        frames -= (size_t)take;
        if (s_fill < s_need) {
            return;
        }
        s_fill = 0;
        muse_energy_feed(&s_energy, muse_audio_dbfs(s_buf, (size_t)s_need));
        if (s_wn->detect(s_model, s_buf) != WAKENET_DETECTED) {
            continue;
        }
        if (!muse_energy_accept(&s_energy, floor_db)) {
            /* The model is sure, the room is not: a burst of noise scored.
             * This is the same check esp32-thinks calls its wake guard. */
            ESP_LOGI(TAG, "a detection with no speech behind it (the room is %.1f dBFS): ignored",
                     (double)floor_db);
            muse_energy_reset(&s_energy, s_step_ms);
            continue;
        }
        on_wake();
        return;
    }
}

void muse_wake_flush(void)
{
    s_fill = 0;
    muse_energy_reset(&s_energy, s_step_ms);
    /* No s_wn->clean() — see on_wake(). Our half-chunk and the energy ring
     * are the only state we own. */
}
