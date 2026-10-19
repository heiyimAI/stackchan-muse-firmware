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

#include "muse_body.h"

#include <stdio.h>
#include <string.h>

#include "bsp/esp-bsp.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"

#include "muse_input.h"
#include "muse_mem.h"
#include "muse_state.h"
#include "muse_touch.h"

static const char *TAG = "muse_body";

/* ---- the body's IO expander (PY32), at 0x6F on the board I2C bus ------- */
#define EXP_ADDR 0x6F
#define EXP_REG_UID_L 0x00
#define EXP_REG_VERSION 0x02
#define EXP_REG_GPIO_M_L 0x03   /* direction: 1 = output */
#define EXP_REG_GPIO_M_H 0x04
#define EXP_REG_GPIO_O_L 0x05   /* output level */
#define EXP_REG_GPIO_O_H 0x06
#define EXP_REG_GPIO_PU_L 0x09
#define EXP_REG_GPIO_PU_H 0x0A
#define EXP_REG_GPIO_DRV_L 0x13
#define EXP_REG_GPIO_DRV_H 0x14
#define EXP_REG_LED_CFG 0x24    /* low 6 bits = count, bit 6 = latch */
#define EXP_REG_LED_RAM 0x30    /* RGB565, two bytes each, little endian */
#define EXP_PIN_SERVO_POWER 0   /* "VM EN": the servo rail's switch */
#define EXP_PIN_LED_DATA 13
#define LED_COUNT 12

/* ---- the head panel (Si12T), at 0x68 on the same bus ------------------- */
#define TOUCH_ADDR 0x68
#define SI12T_SENS1 0x02
#define SI12T_CTRL1 0x08
#define SI12T_CTRL2 0x09
#define SI12T_REF_RST1 0x0A
#define SI12T_REF_RST2 0x0B
#define SI12T_CH_HOLD1 0x0C
#define SI12T_CH_HOLD2 0x0D
#define SI12T_CAL_HOLD1 0x0E
#define SI12T_CAL_HOLD2 0x0F
#define SI12T_OUTPUT1 0x10
/*
 * Si12T is read in 8 levels, encoded in 0x11..0x77. The top nibble is the
 * range and the bottom the threshold; lower nibble = more sensitive
 * (a finger crosses it more easily). 0x11 is the lightest touch; 0x77 the
 * firmest. We map our 1..7 directly onto those nibbles so the
 * parameter is meaningful on either side of the SDK.
 */
#define SI12T_NIBBLE(lvl) ((uint8_t)(0x10 * (lvl) + (lvl)))

/* ---- the servos: SCS bus on UART1 -------------------------------------- */
#define SERVO_UART UART_NUM_1
#define SERVO_TX 6
#define SERVO_RX 7
#define SERVO_BAUD 1000000
#define YAW_ID 1
#define PITCH_ID 2
#define YAW_ZERO_DEFAULT 460
#define PITCH_ZERO_DEFAULT 620
#define YAW_MIN (-1280)
#define YAW_MAX 1280
#define PITCH_MIN 30
#define PITCH_MAX 870
#define RAW_MIN 0
#define RAW_MAX 1000
#define SCS_GOAL_POSITION 42
#define SCS_TORQUE_ENABLE 40

/* LED colours, RGB565 */
#define RGB565(r, g, b) ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))
#define COL_IDLE RGB565(24, 12, 0)
#define COL_LISTEN RGB565(0, 160, 40)
#define COL_THINK RGB565(170, 90, 0)
#define COL_SPEAK RGB565(0, 120, 140)
#define COL_SPEAK_DIM RGB565(0, 60, 70)
#define COL_ERROR RGB565(180, 0, 0)
#define COL_TOUCH RGB565(255, 255, 255)

/* The perceptible half of the body's life: tenths of a degree. */
#define IDLE_YAW 0
#define IDLE_PITCH 450
#define PERK_YAW 0
#define PERK_PITCH 620
#define PERK_MS 250
#define THINK_YAW (-600)
#define THINK_PITCH 520
#define THINK_MS 600
#define THINK2_YAW (-200)
#define THINK2_PITCH 480
#define THINK2_AFTER_MS 1200
#define SPEAK_YAW 80
#define SPEAK_YAW2 (-80)
#define SPEAK_PITCH 470
#define SPEAK_PITCH2 430
#define SPEAK_SWAY_MS 600
#define PRE_LISTEN_SETTLE_MS 300

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_exp;     /* expander: power, LEDs */
static i2c_master_dev_handle_t s_touch_dev;   /* head panel */
static SemaphoreHandle_t s_scs_lock;      /* one writer on the servo bus at a time */
static bool s_servos;                     /* expander + UART are up */
static bool s_leds;
static int s_zero_yaw = YAW_ZERO_DEFAULT;
static int s_zero_pitch = PITCH_ZERO_DEFAULT;

static muse_touch_t s_touch;
static uint8_t s_touch_last[3];
static volatile int64_t s_flash_until_us;

static volatile int s_pose;
static volatile bool s_pose_dirty;
static volatile bool s_frozen;
static int64_t s_pose_us;
static int s_phase;
static int s_sens = CONFIG_MUSE_BODY_TOUCH_SENS;  /* runtime override of Kconfig */
static int s_goto_yaw = IDLE_YAW;         /* what was last commanded, for status */
static int s_goto_pitch = IDLE_PITCH;

/* ------------------------------------------------------------------ I2C -- */

static esp_err_t exp_write(uint8_t reg, const uint8_t *data, size_t len)
{
    uint8_t buf[1 + 32];
    if (!s_exp || len > sizeof(buf) - 1) {
        return ESP_ERR_INVALID_STATE;
    }
    buf[0] = reg;
    memcpy(buf + 1, data, len);
    return i2c_master_transmit(s_exp, buf, len + 1, 50);
}

static esp_err_t exp_write8(uint8_t reg, uint8_t value)
{
    return exp_write(reg, &value, 1);
}

static esp_err_t exp_read8(uint8_t reg, uint8_t *value)
{
    if (!s_exp) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_transmit_receive(s_exp, &reg, 1, value, 1, 50);
}

/* Sets or clears one bit of a low/high register pair. */
static esp_err_t exp_bit(uint8_t reg_l, uint8_t reg_h, int pin, bool on)
{
    uint8_t reg = pin < 8 ? reg_l : reg_h;
    uint8_t value = 0;
    esp_err_t err = exp_read8(reg, &value);
    if (err != ESP_OK) {
        return err;
    }
    uint8_t mask = (uint8_t)(1u << (pin & 7));
    value = on ? (value | mask) : (value & ~mask);
    return exp_write8(reg, value);
}

static esp_err_t touch_read8(uint8_t reg, uint8_t *value)
{
    if (!s_touch_dev) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_transmit_receive(s_touch_dev, &reg, 1, value, 1, 50);
}

static esp_err_t touch_write8(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    if (!s_touch_dev) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_transmit(s_touch_dev, buf, sizeof buf, 50);
}

/* --------------------------------------------------------------- servos -- */

/* One SCS frame: FF FF id len 03 addr data… chk, checksum over id..data. */
static void scs_frame(uint8_t id, uint8_t addr, const uint8_t *data, int len)
{
    uint8_t f[24];
    if (len <= 0 || len > 12) {
        return;
    }
    f[0] = 0xFF;
    f[1] = 0xFF;
    f[2] = id;
    f[3] = (uint8_t)(len + 3);
    f[4] = 0x03;   /* INST_WRITE */
    f[5] = addr;
    memcpy(f + 6, data, (size_t)len);
    uint8_t sum = 0;
    for (int i = 2; i < 6 + len; i++) {
        sum += f[i];
    }
    f[6 + len] = (uint8_t)~sum;
    uart_write_bytes(SERVO_UART, f, (size_t)(7 + len));
}

/* Sends a write and takes the servo's answer:
 *     FF FF id len err chk           (len = 2 for a write)
 * `err` is the servo's own status byte and is worth logging: 0x01 voltage,
 * 0x02 instruction, 0x04 checksum, 0x08 overheat, 0x10 angle limit,
 * 0x20 overload. Note the reply has SIX bytes: the status is reply[4], not
 * reply[3] (which is always the length, 2). */
static bool scs_write(uint8_t id, uint8_t addr, const uint8_t *data, int len)
{
    bool ok = false;
    static uint8_t last_err[3] = { 0, 0, 0 };
    if (!s_servos || !s_scs_lock) {
        return false;
    }
    if (xSemaphoreTake(s_scs_lock, pdMS_TO_TICKS(100)) == pdTRUE) {
        uart_flush_input(SERVO_UART);
        scs_frame(id, addr, data, len);
        uint8_t reply[6];
        int n = uart_read_bytes(SERVO_UART, reply, sizeof reply, pdMS_TO_TICKS(20));
        ok = n >= 6 && reply[0] == 0xFF && reply[1] == 0xFF;
        if (ok && id < 3) {
            uint8_t err = reply[4];
            if (err != last_err[id]) {
                ESP_LOGW(TAG, "servo id %d reg 0x%02x status 0x%02x", id, addr, err);
                last_err[id] = err;
            }
            ok = err == 0;
        }
        xSemaphoreGive(s_scs_lock);
    }
    return ok;
}

/* One READ: FF FF id 04 02 addr n chk, answered by
 *     FF FF id (n+2) err data… chk
 * Returns n, or -1. *status gets the servo's error byte, which is what the
 * firmware needs to tell "not moving" from "not listening". */
static int scs_read(uint8_t id, uint8_t addr, uint8_t *data, int n, uint8_t *status)
{
    if (!s_servos || !s_scs_lock || n <= 0 || n > 8) {
        return -1;
    }
    int got = -1;
    if (xSemaphoreTake(s_scs_lock, pdMS_TO_TICKS(100)) == pdTRUE) {
        uart_flush_input(SERVO_UART);
        uint8_t f[8] = { 0xFF, 0xFF, id, 4, 0x02, addr, (uint8_t)n, 0 };
        uint8_t sum = 0;
        for (int i = 2; i < 7; i++) {
            sum += f[i];
        }
        f[7] = (uint8_t)~sum;
        uart_write_bytes(SERVO_UART, f, sizeof f);
        uint8_t h[5];
        int hn = uart_read_bytes(SERVO_UART, h, sizeof h, pdMS_TO_TICKS(20));
        if (hn == (int)sizeof h && h[0] == 0xFF && h[1] == 0xFF &&
            h[3] == (uint8_t)(n + 2)) {
            *status = h[4];
            int dn = uart_read_bytes(SERVO_UART, data, (size_t)n, pdMS_TO_TICKS(20));
            if (dn == n) {
                got = n;
            }
        }
        xSemaphoreGive(s_scs_lock);
    }
    return got;
}

/* One line per servo: what it says about itself. */
int muse_body_read(char *out, size_t cap)
{
    int n = 0;
    static const char *names[] = { "", "yaw", "pitch" };
    for (uint8_t id = 1; id <= 2; id++) {
        uint8_t torque = 0xFF, pos[2] = { 0xFF, 0xFF }, moving = 0xFF, voltage = 0xFF, temp = 0xFF;
        uint8_t st = 0xFF;
        int got = 0;
        if (scs_read(id, SCS_TORQUE_ENABLE, &torque, 1, &st) == 1) got++;
        if (scs_read(id, 56, pos, 2, &st) == 2) got++;
        if (scs_read(id, 66, &moving, 1, &st) == 1) got++;
        if (scs_read(id, 62, &voltage, 1, &st) == 1) got++;
        if (scs_read(id, 63, &temp, 1, &st) == 1) got++;
        /* Big-endian on the wire: SCSCL sets End = 1, so byte 0 is the high
         * half. Reading it the other way turned 1002 into 59907. */
        int position = (pos[0] << 8) | pos[1];
        n += snprintf(out + n, cap - (size_t)n,
                      "%s%s: torque %d, pos %d, moving %d, %.1f V, %d C, status 0x%02x (%d/5 reads)",
                      n ? " " : "", names[id], torque, position, moving,
                      voltage == 0xFF ? -1 : voltage * 0.1, temp, st, got);
    }
    return n;
}

static int clamp_int(int v, int lo, int hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

/* Tenths of a degree in, servo steps out, measured from the zero position. */
static int angle_to_raw(int tenths, int zero)
{
    return zero + tenths * 32 / 100;
}

/*
 * Motion, the way the factory drives it: hold a target and stream 20 ms
 * steps towards it. A servo that ignores a single "go there" write still
 * follows a stream, and this is exactly the factory's own loop (its spring
 * animation writes one small increment every 20 ms with time = 20).
 */
#define SERVO_STEP_RAW 12      /* ~3.75 deg a tick, about 190 deg/s */

static int s_yaw_raw = -1;     /* what the servo was last told, in raw steps */
static int s_pitch_raw = -1;
static int s_yaw_target = 0;
static int s_pitch_target = 0;
static volatile bool s_driving;   /* motion_drive() owns the step; the task waits */

static void servo_target(int yaw_tenths, int pitch_tenths)
{
    yaw_tenths = clamp_int(yaw_tenths, YAW_MIN, YAW_MAX);
    pitch_tenths = clamp_int(pitch_tenths, PITCH_MIN, PITCH_MAX);
    s_yaw_target = clamp_int(angle_to_raw(yaw_tenths, s_zero_yaw), RAW_MIN, RAW_MAX);
    s_pitch_target = clamp_int(angle_to_raw(pitch_tenths, s_zero_pitch), RAW_MIN, RAW_MAX);
    s_goto_yaw = yaw_tenths;
    s_goto_pitch = pitch_tenths;
}

static int step_toward(int have, int want, int step)
{
    int d = want - have;
    if (d > step) {
        return have + step;
    }
    if (d < -step) {
        return have - step;
    }
    return want;
}

/* Goal position, move time and speed, high byte first — the servo's own
 * order (SCSCL's End = 1). Everything little-endian went to the bus as a
 * byte-swapped number, which is why the servos never moved. */
static void put_pos(uint8_t *b, int raw, uint16_t ms)
{
    b[0] = (uint8_t)(raw >> 8);
    b[1] = (uint8_t)(raw & 0xFF);
    b[2] = (uint8_t)(ms >> 8);
    b[3] = (uint8_t)(ms & 0xFF);
    b[4] = 0;   /* speed 0: the move time is what counts */
    b[5] = 0;
}

static bool motion_settled(void)
{
    return s_yaw_raw >= 0 && s_yaw_raw == s_yaw_target && s_pitch_raw == s_pitch_target;
}

/* One 20 ms tick: advance both axes and tell the servos that moved. */
static void motion_step(void)
{
    if (!s_servos) {
        return;
    }
    if (s_yaw_raw < 0) {
        s_yaw_raw = s_yaw_target;   /* nothing read at boot: start from the target */
        s_pitch_raw = s_pitch_target;
        return;
    }
    int y = step_toward(s_yaw_raw, s_yaw_target, SERVO_STEP_RAW);
    int p = step_toward(s_pitch_raw, s_pitch_target, SERVO_STEP_RAW);
    uint8_t b[6];
    if (y != s_yaw_raw) {
        put_pos(b, y, 20);
        scs_write(YAW_ID, SCS_GOAL_POSITION, b, sizeof b);
        s_yaw_raw = y;
    }
    if (p != s_pitch_raw) {
        put_pos(b, p, 20);
        scs_write(PITCH_ID, SCS_GOAL_POSITION, b, sizeof b);
        s_pitch_raw = p;
    }
}

/* Runs the current target to completion, or until the budget is spent.
 * Callable from any task: it drives the bus itself rather than waiting for
 * the body task. */
static void motion_drive(int budget_ms)
{
    int64_t end = esp_timer_get_time() + (int64_t)budget_ms * 1000;
    s_driving = true;
    while (!motion_settled() && esp_timer_get_time() < end) {
        motion_step();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    s_driving = false;
}

/* ----------------------------------------------------------------- LEDs -- */

static uint16_t s_led_px[LED_COUNT];
static uint16_t s_led_sent[LED_COUNT];
static bool s_led_sent_ok;

static void leds_flush(void)
{
    uint8_t buf[LED_COUNT * 2];
    for (int i = 0; i < LED_COUNT; i++) {
        buf[i * 2] = (uint8_t)(s_led_px[i] & 0xFF);
        buf[i * 2 + 1] = (uint8_t)(s_led_px[i] >> 8);
    }
    if (exp_write(EXP_REG_LED_RAM, buf, sizeof buf) != ESP_OK) {
        return;
    }
    uint8_t cfg = 0;
    if (exp_read8(EXP_REG_LED_CFG, &cfg) != ESP_OK) {
        return;
    }
    exp_write8(EXP_REG_LED_CFG, (uint8_t)(cfg | (1 << 6)));   /* latch */
    memcpy(s_led_sent, s_led_px, sizeof s_led_sent);
    s_led_sent_ok = true;
}

static void leds_fill(uint16_t colour)
{
    for (int i = 0; i < LED_COUNT; i++) {
        s_led_px[i] = colour;
    }
}

static bool leds_changed(void)
{
    return !s_led_sent_ok || memcmp(s_led_sent, s_led_px, sizeof s_led_px) != 0;
}

static void leds_tick(int64_t now)
{
    if (!s_leds) {
        return;
    }
    if (now < s_flash_until_us) {
        leds_fill(COL_TOUCH);   /* the head was touched: say so at once */
    } else if (muse_state_asleep()) {
        leds_fill(0);
    } else {
        switch (muse_state_mode(NULL)) {
        case MUSE_MODE_LISTENING:
            leds_fill(COL_LISTEN);
            break;
        case MUSE_MODE_THINKING: {
            /* One bright LED circling the ring: it is working on it. */
            static const uint8_t dim[] = { 40, 25, 10 };
            for (int i = 0; i < LED_COUNT; i++) {
                bool lit = i == (int)((now / 120000) % LED_COUNT);
                s_led_px[i] = lit ? COL_THINK : RGB565(dim[0], dim[1], dim[2]);
            }
            break;
        }
        case MUSE_MODE_SPEAKING: {
            /* Breathing with the sway. */
            bool bright = ((now / 600000) & 1) != 0;
            leds_fill(bright ? COL_SPEAK : COL_SPEAK_DIM);
            break;
        }
        case MUSE_MODE_ERROR:
            leds_fill(COL_ERROR);
            break;
        default:
            leds_fill(COL_IDLE);
            break;
        }
    }
    if (leds_changed()) {
        leds_flush();
    }
}

/* --------------------------------------------------------------- motion -- */

static void motion_tick(int64_t now)
{
    if (!s_servos) {
        return;
    }
    if (s_frozen) {
        s_pose_dirty = false;
        return;
    }
    int pose = s_pose;
    if (s_pose_dirty) {
        s_pose_dirty = false;
        switch (pose) {
        case MUSE_BODY_POSE_THINK:
            servo_target(THINK_YAW, THINK_PITCH);
            break;
        case MUSE_BODY_POSE_SPEAK:
            servo_target(SPEAK_YAW, SPEAK_PITCH);
            break;
        default:
            servo_target(IDLE_YAW, IDLE_PITCH);
            break;
        }
    }
    if (pose == MUSE_BODY_POSE_THINK) {
        if (s_phase == 0 && now - s_pose_us > THINK2_AFTER_MS * 1000LL) {
            s_phase = 1;
            servo_target(THINK2_YAW, THINK2_PITCH);
        }
    } else if (pose == MUSE_BODY_POSE_SPEAK) {
        if (now - s_pose_us > SPEAK_SWAY_MS * 1000LL) {
            s_pose_us = now;
            s_phase ^= 1;
            servo_target(s_phase ? SPEAK_YAW2 : SPEAK_YAW,
                         s_phase ? SPEAK_PITCH2 : SPEAK_PITCH);
        }
    }
    /* One 20 ms increment, as the factory does — unless a caller is already
     * driving the same two counters from its own task. */
    if (!s_driving) {
        motion_step();
    }
}

/* ---------------------------------------------------------------- touch -- */

static void touch_poll(void)
{
    uint8_t raw = 0;
    if (!s_touch_dev || touch_read8(SI12T_OUTPUT1, &raw) != ESP_OK) {
        return;
    }
    uint8_t intensity[3] = {
        (uint8_t)(raw & 0x03),
        (uint8_t)((raw >> 2) & 0x03),
        (uint8_t)((raw >> 4) & 0x03),
    };
    memcpy(s_touch_last, intensity, sizeof s_touch_last);
    switch (muse_touch_step(&s_touch, intensity)) {
    case MUSE_TOUCH_PRESS:
        s_flash_until_us = esp_timer_get_time() + 100000;
        muse_input_touch_press();
        break;
    case MUSE_TOUCH_RELEASE:
        muse_input_touch_release();
        break;
    case MUSE_TOUCH_SWIPE_FWD:
    case MUSE_TOUCH_SWIPE_BACK:
        /* Unbound for now: this is where volume (or a dismiss) would go. */
        break;
    default:
        break;
    }
}

/* ----------------------------------------------------------------- task -- */

static void body_task(void *arg)
{
    (void)arg;
    int64_t next_touch = 0;
    int64_t next_led = 0;
    for (;;) {
        int64_t now = esp_timer_get_time();
        if (now >= next_touch) {
            touch_poll();
            next_touch = now + 50 * 1000;
        }
        if (now >= next_led) {
            leds_tick(now);
            next_led = now + 40 * 1000;
        }
        motion_tick(now);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* --------------------------------------------------------------- public -- */

bool muse_body_present(void)
{
    return s_exp != NULL;
}

void muse_body_set_pose(muse_body_pose_t pose)
{
    if (!s_servos) {
        return;
    }
    s_pose = pose;
    s_pose_us = esp_timer_get_time();
    s_phase = 0;
    s_frozen = pose == MUSE_BODY_POSE_LISTEN;
    s_pose_dirty = !s_frozen;
}

void muse_body_pre_listen(void)
{
    if (!s_servos) {
        return;
    }
    /* Drive the perk from here — the caller is the voice task or the body
     * task itself — so the motors are still by the time recording starts. */
    s_frozen = false;
    servo_target(PERK_YAW, PERK_PITCH);
    motion_drive(PERK_MS + 200);
    vTaskDelay(pdMS_TO_TICKS(PRE_LISTEN_SETTLE_MS / 2));
    s_pose = MUSE_BODY_POSE_LISTEN;
    s_frozen = true;
    s_pose_dirty = false;
    s_pose_us = esp_timer_get_time();
}

bool muse_body_move(int yaw_tenths, int pitch_tenths)
{
    if (!s_servos) {
        return false;
    }
    s_frozen = true;   /* a bench move holds until a pose is set again */
    servo_target(yaw_tenths, pitch_tenths);
    motion_drive(2000);
    return motion_settled();
}

/*
 * Loopback test: route the UART's RX to its own TX line so every byte the
 * engine writes is read back. A pass proves the UART controller, the baud
 * rate, the pins, and the TX half are all working on the CoreS3 side; a
 * fail (or empty read) means the body is the limit. Does not need a servo.
 */
bool muse_body_loopback(void)
{
    if (!s_servos) {
        return false;
    }
    if (xSemaphoreTake(s_scs_lock, pdMS_TO_TICKS(100)) == pdTRUE) {
        uart_flush_input(SERVO_UART);
        const char *msg = "PING-FOR-LOOPBACK-1234";
        uart_write_bytes(SERVO_UART, msg, strlen(msg));
        uint8_t back[32];
        int n = uart_read_bytes(SERVO_UART, back, sizeof back, pdMS_TO_TICKS(50));
        bool ok = (n == (int)strlen(msg) && memcmp(back, msg, strlen(msg)) == 0);
        ESP_LOGI(TAG, "loopback: wrote %u, read %d %s", (unsigned)strlen(msg), n, ok ? "OK" : "mismatch");
        xSemaphoreGive(s_scs_lock);
        return ok;
    }
    return false;
}

void muse_body_leds(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_leds) {
        return;
    }
    leds_fill(RGB565(r, g, b));
    leds_flush();
}

static esp_err_t nvs_read_i32(nvs_handle_t h, const char *key, int *value)
{
    int32_t v = 0;
    esp_err_t err = nvs_get_i32(h, key, &v);
    if (err == ESP_OK) {
        *value = (int)v;
    }
    return err;
}

bool muse_body_zero(int yaw_raw, int pitch_raw)
{
    if (yaw_raw < RAW_MIN || yaw_raw > RAW_MAX || pitch_raw < RAW_MIN || pitch_raw > RAW_MAX) {
        return false;
    }
    nvs_handle_t h;
    if (nvs_open("body", NVS_READWRITE, &h) != ESP_OK) {
        return false;
    }
    bool ok = nvs_set_i32(h, "yaw0", yaw_raw) == ESP_OK &&
              nvs_set_i32(h, "pitch0", pitch_raw) == ESP_OK &&
              nvs_commit(h) == ESP_OK;
    nvs_close(h);
    if (ok) {
        s_zero_yaw = yaw_raw;
        s_zero_pitch = pitch_raw;
        ESP_LOGI(TAG, "zero saved: yaw %d, pitch %d", yaw_raw, pitch_raw);
    }
    return ok;
}

int muse_body_scan(char *out, size_t cap)
{
    int n = 0;
    i2c_master_bus_handle_t bus = bsp_i2c_get_handle();
    if (!bus) {
        return snprintf(out, cap, "no bus");
    }
    /* A scan briefly attaches and detaches a device per address. The bus
     * driver does not like this: each removal can take down a peer that
     * another handle is still using. To avoid that, we add ONE 100 kHz
     * device per address, and we never re-add a body part here — if a
     * body handle dropped out, only a re-probe of the cached handle
     * restores it, and that is what the body task does at the next
     * touch poll. */
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        i2c_master_dev_handle_t dev;
        i2c_device_config_t cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = 100000,
        };
        if (i2c_master_bus_add_device(bus, &cfg, &dev) != ESP_OK) {
            continue;
        }
        uint8_t reg = 0, value = 0;
        bool there = i2c_master_transmit_receive(dev, &reg, 1, &value, 1, 20) == ESP_OK;
        i2c_master_bus_rm_device(dev);
        if (there) {
            n += snprintf(out + n, cap - (size_t)n, "%s%02x", n ? "," : "", addr);
            if ((size_t)n >= cap - 4) {
                break;
            }
        }
    }
    /* The scan tends to leave the body's two handles in a state where
     * the next I/O fails. Send the head a few warm-up reads (cheap on
     * the I2C bus, restores the bus driver's view) so the user's
     * next `>body` reports a real answer instead of a stale false. */
    if (s_exp) {
        uint8_t uid = 0, ver = 0;
        if (exp_read8(EXP_REG_UID_L, &uid) == ESP_OK && exp_read8(EXP_REG_VERSION, &ver) == ESP_OK) {
            /* the bus is back, all good */
        }
    }
    if (s_touch_dev) {
        uint8_t probe = 0;
        touch_read8(SI12T_OUTPUT1, &probe);
    }
    return n;
}

bool muse_body_set_sens(int level)
{
    if (level < 1 || level > 7 || !s_touch_dev) {
        return false;
    }
    uint8_t nibble = SI12T_NIBBLE(level);
    for (int i = 0; i < 6; i++) {
        if (touch_write8((uint8_t)(SI12T_SENS1 + i), nibble) != ESP_OK) {
            return false;
        }
    }
    s_sens = level;
    ESP_LOGI(TAG, "touch sensitivity set to %d (nibble 0x%02x)", level, nibble);
    return true;
}

int muse_body_status(char *out, size_t cap)
{
    return snprintf(out, cap,
                    "{\"present\":%s,\"servos\":%s,\"touch\":%s,\"yaw\":%d,\"pitch\":%d,"
                    "\"intensity\":[%u,%u,%u],\"sens\":%d}",
                    s_exp ? "true" : "false", s_servos ? "true" : "false",
                    s_touch_dev ? "true" : "false", s_goto_yaw, s_goto_pitch,
                    s_touch_last[0], s_touch_last[1], s_touch_last[2], s_sens);
}

bool muse_body_start(void)
{
    if (s_exp) {
        return true;
    }
    s_bus = bsp_i2c_get_handle();
    if (!s_bus) {
        ESP_LOGW(TAG, "no I2C bus: body disabled");
        return false;
    }
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = EXP_ADDR,
        .scl_speed_hz = 100000,   /* the body parts are 100 kHz, like the factory */
    };
    if (i2c_master_bus_add_device(s_bus, &cfg, &s_exp) != ESP_OK) {
        ESP_LOGW(TAG, "no body: expander add failed");
        s_exp = NULL;
        return false;
    }
    uint8_t uid_lo = 0, version = 0;
    if (exp_read8(EXP_REG_UID_L, &uid_lo) != ESP_OK || exp_read8(EXP_REG_VERSION, &version) != ESP_OK) {
        ESP_LOGW(TAG, "no body: the expander does not answer (the head panel and servos stay off)");
        i2c_master_bus_rm_device(s_exp);
        s_exp = NULL;
        return false;
    }
    ESP_LOGI(TAG, "body expander: uid %02x, version %02x", uid_lo, version);

    /* Servo rail on, then give it the quarter second it needs. */
    exp_bit(EXP_REG_GPIO_M_L, EXP_REG_GPIO_M_H, EXP_PIN_SERVO_POWER, true);
    exp_bit(EXP_REG_GPIO_PU_L, EXP_REG_GPIO_PU_H, EXP_PIN_SERVO_POWER, true);
    exp_bit(EXP_REG_GPIO_O_L, EXP_REG_GPIO_O_H, EXP_PIN_SERVO_POWER, true);
    vTaskDelay(pdMS_TO_TICKS(200));

    /* The LED ring hangs off the same expander. */
    exp_bit(EXP_REG_GPIO_M_L, EXP_REG_GPIO_M_H, EXP_PIN_LED_DATA, true);
    exp_bit(EXP_REG_GPIO_PU_L, EXP_REG_GPIO_PU_H, EXP_PIN_LED_DATA, true);
    exp_bit(EXP_REG_GPIO_DRV_L, EXP_REG_GPIO_DRV_H, EXP_PIN_LED_DATA, false);   /* push-pull */
    if (exp_write8(EXP_REG_LED_CFG, LED_COUNT) == ESP_OK) {
        s_leds = true;
        leds_fill(0);
        leds_flush();
        leds_fill(0);
        leds_flush();
    }

    /* The head panel. */
    i2c_device_config_t tcfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TOUCH_ADDR,
        .scl_speed_hz = 100000,
    };
    if (i2c_master_bus_add_device(s_bus, &tcfg, &s_touch_dev) == ESP_OK) {
        uint8_t probe = 0;
        if (touch_read8(SI12T_OUTPUT1, &probe) == ESP_OK) {
            /* The factory init: reference calibration and channel holds open,
             * then soft reset, channel enables, ctrl1, sensitivity. */
            static const uint8_t zeros[] = { 0, 0, 0, 0, 0, 0 };
            touch_write8(SI12T_REF_RST1, zeros[0]);
            touch_write8(SI12T_REF_RST2, zeros[1]);
            touch_write8(SI12T_CH_HOLD1, zeros[2]);
            touch_write8(SI12T_CH_HOLD2, zeros[3]);
            touch_write8(SI12T_CAL_HOLD1, zeros[4]);
            touch_write8(SI12T_CAL_HOLD2, zeros[5]);
            touch_write8(SI12T_CTRL2, 0x0F);
            touch_write8(SI12T_CTRL2, 0x07);
            touch_write8(SI12T_CTRL1, 0x22);
            for (int i = 0; i < 6; i++) {
                touch_write8((uint8_t)(SI12T_SENS1 + i), SI12T_NIBBLE(CONFIG_MUSE_BODY_TOUCH_SENS));
            }
            muse_touch_reset(&s_touch, 40);
            ESP_LOGI(TAG, "head panel: on");
        } else {
            ESP_LOGW(TAG, "no head panel: touch disabled");
            i2c_master_bus_rm_device(s_touch_dev);
            s_touch_dev = NULL;
        }
    }

    /* The servos. */
    s_scs_lock = xSemaphoreCreateMutex();
    uart_config_t uart = {
        .baud_rate = SERVO_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };
    if (s_scs_lock &&
        uart_driver_install(SERVO_UART, 1024, 0, 0, NULL, 0) == ESP_OK &&
        uart_param_config(SERVO_UART, &uart) == ESP_OK &&
        uart_set_pin(SERVO_UART, SERVO_TX, SERVO_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) == ESP_OK) {
        s_servos = true;
    } else {
        ESP_LOGW(TAG, "servo bus unavailable: motion disabled");
    }

    /* Calibration from NVS, then a first move so the head knows where it is. */
    nvs_handle_t h;
    if (nvs_open("body", NVS_READONLY, &h) == ESP_OK) {
        nvs_read_i32(h, "yaw0", &s_zero_yaw);
        nvs_read_i32(h, "pitch0", &s_zero_pitch);
        nvs_close(h);
    }
    s_zero_yaw = clamp_int(s_zero_yaw, RAW_MIN, RAW_MAX);
    s_zero_pitch = clamp_int(s_zero_pitch, RAW_MIN, RAW_MAX);
    if (s_servos) {
        ESP_LOGI(TAG, "servos: zero yaw %d, pitch %d", s_zero_yaw, s_zero_pitch);
        /* Where is the head? The factory reads it too and starts from there. */
        for (uint8_t id = 1; id <= 2; id++) {
            uint8_t pos[2], st = 0;
            if (scs_read(id, 56, pos, 2, &st) == 2) {
                int raw = (pos[0] << 8) | pos[1];   /* high byte first */
                if (id == YAW_ID) {
                    s_yaw_raw = raw;
                } else {
                    s_pitch_raw = raw;
                }
                ESP_LOGI(TAG, "servo %d present position %d (status 0x%02x)", id, raw, st);
            } else {
                ESP_LOGW(TAG, "servo %d did not report its position", id);
            }
        }
        servo_target(IDLE_YAW, IDLE_PITCH);
        motion_drive(1200);
    }

    /* One task for the whole body: touch, LEDs and motion. */
    if (xTaskCreatePinnedToCoreWithCaps(body_task, "muse_body", 4096, NULL, 3, NULL, 1,
                                        MUSE_BIG_CAPS) != pdPASS) {
        ESP_LOGW(TAG, "no task for the body: touch and motion disabled");
        s_touch_dev = NULL;
        s_servos = false;
        s_leds = false;
    }
    ESP_LOGI(TAG, "body up: servos %s, touch %s, leds %s", s_servos ? "yes" : "no",
             s_touch_dev ? "yes" : "no", s_leds ? "yes" : "no");
    return true;
}
