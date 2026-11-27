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
 *    Each frame is 80 ms; a blink = 3 frames = 240 ms. The cycle is anchored
 *    to the moment the last key was released, so the first blink lands 1.6 s
 *    after typing stops.
 *      frames  21-23 : both eyes blink once
 *      frames  51-53, 57-59 : both eyes blink twice
 *      frames  81-83 : left-eye wink (right eye stays open)
 *      frames 111-113 : right-eye wink (left eye stays open)
 *    All other frames: both eyes open.
 *
 * Disconnected: when no USB/BLE endpoint is connected, the active BLE
 * channel's bond state is shown instead: no bond -> both LEDs flash
 * together; bonded but disconnected -> LEDs flash alternately. Once an
 * endpoint connects, the normal eye modes resume.
 * All light output goes through set_eyes().
 *
 * GPIOs are GPIO_ACTIVE_LOW (see overlay); gpio_pin_set_dt(...,1) = LED on.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/wpm_state_changed.h>
#include <zmk/endpoints.h>
#include <zmk/ble.h>

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

/* Disconnected status flash: 500 ms per phase (1 s full cycle). */
#define HINT_HALF_MS    500

/* Build marker, forced into .rodata so the final binary can be verified. */
static const char eye_leds_build_tag[] __attribute__((used)) = "BATOID_EYE_LEDS_OPEN_V9";

static uint8_t current_wpm;
static int pressed_keys;
static bool standard_mode = true;
static bool turbo_playing;
static bool idle_playing = true;

static int turbo_frame = 1;
static int64_t turbo_frame_ms;
static int64_t idle_start_at;
static bool idle_pending;

static int active_eye; /* 0 = left eye, 1 = right eye */

static void set_eyes(bool left_on, bool right_on);

/* --- deterministic 12 s idle blink timeline ------------------------------ */
/* The cycle is anchored to the instant the last key was released, so the
 * first blink lands 1.6 s after typing stops. */
static int64_t blink_cycle_start;
static int64_t last_release_at;

static void enter_idle(int64_t now)
{
    idle_playing = true;
    idle_pending = false;
    turbo_playing = false;
    standard_mode = false;
    /* Anchor to the last key release, not to the idle timer expiry. */
    blink_cycle_start = last_release_at ? last_release_at : now;
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
    gpio_pin_set_dt(&led_l, left_on ? 1 : 0);
    gpio_pin_set_dt(&led_r, right_on ? 1 : 0);
}

/* No endpoint connected: show the active BLE channel's bond state. */
static void disconnected_hint(int64_t now)
{
    bool phase = ((now / HINT_HALF_MS) & 1) != 0;

#if IS_ENABLED(CONFIG_ZMK_BLE)
    if (!zmk_ble_active_profile_is_open()) {
        /* Bonded but disconnected: LEDs flash alternately. */
        set_eyes(phase, !phase);
        return;
    }
#endif
    /* No device paired on the active channel (or no BLE): flash together. */
    set_eyes(phase, phase);
}

static void eye_led_tick(struct k_timer *timer)
{
    int64_t now = k_uptime_get();

    /* Connection gate first: when nothing is connected, show the BLE bond
     * hint instead of the eye animation. */
    if (!zmk_endpoint_is_connected()) {
        disconnected_hint(now);
        return;
    }

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
            /* Defensive: frame is always 1..6, never leave both eyes dark. */
            set_eyes(true, false);
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
        if (current_wpm >= TURBO_WPM && !turbo_playing) {
            turbo_playing = true;
            turbo_frame = 1;
            turbo_frame_ms = k_uptime_get();
            standard_mode = false;
            idle_playing = false;
        }
        pressed_keys += pos->state ? 1 : -1;
        if (pressed_keys < 0) {
            pressed_keys = 0;
        } else if (pressed_keys > 48) {
            /* 4x12 matrix: guard against lost release events. */
            pressed_keys = 48;
        }
        if (pressed_keys == 0) {
            /* Anchor the blink timeline to the moment typing actually stops. */
            last_release_at = k_uptime_get();
        }
        idle_pending = true;
        idle_start_at = k_uptime_get() + IDLE_MS;
        return ZMK_EV_EVENT_BUBBLE;
    }

    struct zmk_wpm_state_changed *wpm = as_zmk_wpm_state_changed(eh);
    if (wpm) {
        current_wpm = wpm->state;
        int64_t now = k_uptime_get();
        if (current_wpm >= TURBO_WPM) {
            if (!turbo_playing) {
                turbo_playing = true;
                turbo_frame = 1;
                turbo_frame_ms = now;
                standard_mode = false;
                idle_playing = false;
            }
        } else if (turbo_playing) {
            turbo_playing = false;
            /* Resume a concrete mode immediately so the eyes never freeze on
             * the last turbo frame for a whole tick. */
            if (pressed_keys > 0) {
                idle_playing = false;
                standard_mode = true;
            } else {
                enter_idle(now);
            }
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

    int64_t now = k_uptime_get();
    turbo_frame_ms = now;
    last_release_at = now;
    blink_cycle_start = now;
    turbo_frame = 1;
    standard_mode = true;
    idle_playing = true;
    set_eyes(false, false);

    k_timer_start(&eye_led_timer, K_MSEC(TICK_MS), K_MSEC(TICK_MS));
    return 0;
}

SYS_INIT(eye_led_init, APPLICATION, 90);
