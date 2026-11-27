/*
 * Batoid "eye" LEDs for ZMK - based on the original QMK firmware
 * (firmware/kotte/batoid/keymaps/default/keymap.c).
 *
 * LED semantics: ON = eye OPEN, OFF = eye CLOSED.
 *
 * Modes:
 *  - Standard (while typing): mirrored - right-hand keys light the LEFT eye,
 *    left-hand keys light the RIGHT eye. Keys released -> both eyes open.
 *  - Turbo (WPM >= 70): left/right eyes alternate every 100 ms, 6-frame loop.
 *  - Idle (1 s after the last key): deterministic 12 s / 150-frame timeline.
 *    Each frame is 80 ms; a blink = 3 frames = 240 ms. The cycle restarts
 *    whenever idle begins, so the eyes are open for the first 1.6 s.
 *      frames  21-23 : both eyes blink once
 *      frames  51-53, 57-59 : both eyes blink twice
 *      frames  81-83 : left-eye wink (right eye stays open)
 *      frames 111-113 : right-eye wink (left eye stays open)
 *    All other frames: both eyes open.
 *
 * Connection gate: when neither USB nor BLE is connected, both eyes stay dark.
 * All light output goes through set_eyes(), so the gate is applied there.
 *
 * GPIOs are GPIO_ACTIVE_LOW (see overlay); gpio_pin_set_dt(...,1) = LED on.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/wpm_state_changed.h>
#include <zmk/endpoints.h>

#define LED_L_NODE DT_NODELABEL(caps_lock_led)
#define LED_R_NODE DT_NODELABEL(bt_status_led)

static const struct gpio_dt_spec led_l = GPIO_DT_SPEC_GET(LED_L_NODE, gpios);
static const struct gpio_dt_spec led_r = GPIO_DT_SPEC_GET(LED_R_NODE, gpios);

#define TURBO_WPM       70
#define IDLE_MS         1000
#define TICK_MS         10
#define TURBO_FRAME_MS  100
#define TURBO_FRAME_MAX 6

/* Idle blink timeline: 80 ms per frame, 150 frames = 12 s per cycle. */
#define BLINK_FRAME_MS  80
#define BLINK_FRAMES    150
#define BLINK_CYCLE_MS  (BLINK_FRAME_MS * BLINK_FRAMES)

/* Build marker, forced into .rodata so the final binary can be verified. */
static const char eye_leds_build_tag[] __attribute__((used)) = "BATOID_EYE_LEDS_OPEN_V8";

static uint8_t current_wpm;
static int pressed_keys;
static bool standard_mode = true;
static bool turbo_playing;
static bool idle_playing = true;

static int turbo_frame;
static int64_t turbo_frame_ms;
static int64_t idle_start_at;
static bool idle_pending;

static int active_eye; /* 0 = left eye, 1 = right eye */

static void set_eyes(bool left_on, bool right_on);

/* --- deterministic 12 s idle blink timeline ------------------------------ */
/* The cycle starts over each time idle begins, so the first blink is ~1.6 s
 * after the last keypress. All timing comes from one uptime reference. */
static int64_t blink_cycle_start;

static void enter_idle(int64_t now)
{
    idle_playing = true;
    idle_pending = false;
    turbo_playing = false;
    standard_mode = false;
    blink_cycle_start = now;
}

static bool frame_in(int frame, int first, int last)
{
    return frame >= first && frame <= last;
}

static void blink_tick(int64_t now)
{
    int64_t elapsed = now - blink_cycle_start;

    /* Wrap safely even after days of uptime. */
    elapsed %= BLINK_CYCLE_MS;
    if (elapsed < 0) {
        elapsed += BLINK_CYCLE_MS;
    }

    /* 1-based frame number in [1,150]; each frame lasts 80 ms. */
    int frame = (int)(elapsed / BLINK_FRAME_MS) + 1;

    if (frame_in(frame, 21, 23) ||
        frame_in(frame, 51, 53) ||
        frame_in(frame, 57, 59)) {
        /* Double-eye blink, 240 ms each. LED off = eye closed. */
        set_eyes(false, false);
    } else if (frame_in(frame, 81, 83)) {
        /* Left-eye wink: left eye closed, right eye stays open. */
        set_eyes(false, true);
    } else if (frame_in(frame, 111, 113)) {
        /* Right-eye wink: right eye closed, left eye stays open. */
        set_eyes(true, false);
    } else {
        /* Every other frame: both eyes open. */
        set_eyes(true, true);
    }
}

static void set_eyes(bool left_on, bool right_on)
{
    /* No USB/BLE connection -> both eyes stay dark. */
    if (!zmk_endpoint_is_connected()) {
        left_on = false;
        right_on = false;
    }
    gpio_pin_set_dt(&led_l, left_on ? 1 : 0);
    gpio_pin_set_dt(&led_r, right_on ? 1 : 0);
}

static void eye_led_tick(struct k_timer *timer)
{
    int64_t now = k_uptime_get();

    /* Only go idle once every key is released; holding a key must keep the
     * mirrored typing state even if the previous idle timer has elapsed. */
    if (idle_pending && pressed_keys == 0 && now >= idle_start_at) {
        enter_idle(now);
    }
    /* If no mode is driving the eyes (e.g. turbo just ended), fall back to
     * typing-mirror or idle so the eyes never get stuck dark. */
    if (!turbo_playing && !idle_playing && !standard_mode) {
        if (pressed_keys > 0) {
            standard_mode = true;
        } else {
            enter_idle(now);
        }
    }

    if (turbo_playing) {
        if (now - turbo_frame_ms >= TURBO_FRAME_MS) {
            turbo_frame_ms = now;
            turbo_frame = (turbo_frame >= TURBO_FRAME_MAX) ? 1 : turbo_frame + 1;
        }
        switch (turbo_frame) {
        case 1:
        case 3:
        case 5:
            set_eyes(true, false);
            break;
        case 2:
        case 4:
        case 6:
            set_eyes(false, true);
            break;
        default:
            set_eyes(false, false);
            break;
        }
    }

    if (idle_playing) {
        standard_mode = false;
        blink_tick(now);
    }

    if (standard_mode) {
        if (pressed_keys > 0) {
            set_eyes(active_eye == 0, active_eye == 1);
        } else {
            /* Keys just released but the idle animation has not started yet:
             * both eyes must be OPEN, never a dark gap. */
            set_eyes(true, true);
        }
    }
}

K_TIMER_DEFINE(eye_led_timer, eye_led_tick, NULL);

static int eye_led_listener(const zmk_event_t *eh)
{
    struct zmk_position_state_changed *pos = as_zmk_position_state_changed(eh);
    if (pos) {
        if (pos->state) {
            idle_playing = false;
            standard_mode = true;
            uint32_t col = pos->position % 12;
            active_eye = (col > 5) ? 0 : 1; /* right hand -> left eye */
        }
        if (current_wpm >= TURBO_WPM) {
            turbo_playing = true;
            standard_mode = false;
            idle_playing = false;
        }
        pressed_keys += pos->state ? 1 : -1;
        if (pressed_keys < 0) {
            pressed_keys = 0;
        }
        idle_pending = true;
        idle_start_at = k_uptime_get() + IDLE_MS;
        return ZMK_EV_EVENT_BUBBLE;
    }

    struct zmk_wpm_state_changed *wpm = as_zmk_wpm_state_changed(eh);
    if (wpm) {
        current_wpm = wpm->state;
        if (current_wpm >= TURBO_WPM) {
            turbo_playing = true;
            standard_mode = false;
            idle_playing = false;
        } else {
            /* Let the eyes fall back to typing-mirror / idle blinking. */
            turbo_playing = false;
        }
        return ZMK_EV_EVENT_BUBBLE;
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(eye_led, eye_led_listener);
ZMK_SUBSCRIPTION(eye_led, zmk_position_state_changed);
ZMK_SUBSCRIPTION(eye_led, zmk_wpm_state_changed);

static int eye_led_init(void)
{
    volatile const char *tag = eye_leds_build_tag;
    (void)tag;

    if (!device_is_ready(led_l.port) || !device_is_ready(led_r.port)) {
        return -ENODEV;
    }
    gpio_pin_configure_dt(&led_l, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_r, GPIO_OUTPUT_INACTIVE);

    turbo_frame_ms = k_uptime_get();
    standard_mode = true;
    idle_playing = true;
    blink_cycle_start = k_uptime_get();
    set_eyes(false, false);

    k_timer_start(&eye_led_timer, K_MSEC(TICK_MS), K_MSEC(TICK_MS));
    return 0;
}

SYS_INIT(eye_led_init, APPLICATION, 90);
