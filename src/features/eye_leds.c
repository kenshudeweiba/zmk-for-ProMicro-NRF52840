/*
 * Batoid "eye" LEDs for ZMK - faithful port of the original QMK firmware
 * (firmware/kotte/batoid/keymaps/default/keymap.c).
 *
 * Three modes, exactly like the original:
 *  - Standard (while typing): mirrored - right-hand keys light the LEFT eye,
 *    left-hand keys light the RIGHT eye. Keys released -> both off.
 *  - Turbo (WPM >= 70): left/right eyes alternate every 100 ms, 6-frame loop.
 *  - Idle (1 s after the last key): both eyes off, except a blink at idle
 *    frames 21, 51 and 53 of a 300-frame loop at 80 ms/frame (~24 s).
 *
 * There is deliberately NO caps-lock / connection-status display here, to
 * match the original QMK behavior.
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
#define IDLE_FRAME_MS   80
#define TURBO_FRAME_MS  100
#define IDLE_FRAME_MAX  300
#define TURBO_FRAME_MAX 6

/* Build marker, forced into .rodata so the final binary can be verified. */
static const char eye_leds_build_tag[] __attribute__((used)) = "BATOID_EYE_LEDS_QMK_V3";

static uint8_t current_wpm;
static int pressed_keys;
static bool standard_mode = true;
static bool turbo_playing;
static bool idle_playing = true;

static int idle_frame;
static int turbo_frame;
static int64_t idle_frame_ms;
static int64_t turbo_frame_ms;
static int64_t idle_start_at;
static bool idle_pending;

static int active_eye; /* 0 = left eye, 1 = right eye */

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

    if (idle_pending && now >= idle_start_at) {
        idle_playing = true;
        idle_pending = false;
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
        turbo_playing = false;
        if (now - idle_frame_ms >= IDLE_FRAME_MS) {
            idle_frame_ms = now;
            idle_frame = (idle_frame >= IDLE_FRAME_MAX) ? 1 : idle_frame + 1;
        }
        set_eyes(false, false);
        if (idle_frame == 21 || idle_frame == 51 || idle_frame == 53) {
            set_eyes(true, true);
        }
    }

    if (standard_mode) {
        if (pressed_keys > 0) {
            set_eyes(active_eye == 0, active_eye == 1);
        } else {
            set_eyes(false, false);
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

    idle_frame_ms = k_uptime_get();
    turbo_frame_ms = idle_frame_ms;
    standard_mode = true;
    idle_playing = true;
    set_eyes(false, false);

    k_timer_start(&eye_led_timer, K_MSEC(TICK_MS), K_MSEC(TICK_MS));
    return 0;
}

SYS_INIT(eye_led_init, APPLICATION, 90);
