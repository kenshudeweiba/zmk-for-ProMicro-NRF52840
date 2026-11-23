/*
 * Batoid "eye" LEDs for ZMK.
 *
 * Mirrors the original QMK batoid firmware behavior:
 *  - While typing, the eye on the side being typed on lights up
 *    (right-hand keys -> LEFT eye, left-hand keys -> RIGHT eye).
 *  - Fast typing (WPM >= TURBO_WPM) runs a left/right alternating animation.
 *  - After IDLE_MS of no key activity, the LEDs are handed back to the
 *    classic status display: caps lock state + endpoint connection state,
 *    so the two LEDs keep showing caps/connection while idle.
 *
 * The LEDs are the two GPIOs described in the overlay (gpio-leds nodes).
 * We drive them directly with gpio_pin_set_dt(). NOTE: the ZMK
 * `zmk,indicator-leds` node for these two LEDs is intentionally removed from
 * the overlay, because this module becomes the single owner of the pins.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/wpm_state_changed.h>
#include <zmk/hid_indicators.h>
#include <zmk/endpoints.h>

#include <dt-bindings/zmk/hid_indicators.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(eye_leds, CONFIG_LOG_DEFAULT_LEVEL);

/* The two physical LEDs, declared as gpio-leds children in the overlay. */
#define LED_L_NODE DT_NODELABEL(caps_lock_led)
#define LED_R_NODE DT_NODELABEL(bt_status_led)

static const struct gpio_dt_spec led_l = GPIO_DT_SPEC_GET(LED_L_NODE, gpios);
static const struct gpio_dt_spec led_r = GPIO_DT_SPEC_GET(LED_R_NODE, gpios);

#define IDLE_MS      1500  /* stop typing -> back to status display */
#define TURBO_WPM    70
#define TICK_MS      50

/* Runtime state */
static int64_t last_key_ms;
static bool turbo_mode;
static bool status_mode = true;   /* true while showing caps/connection */
static uint8_t turbo_frame;

/* column index -> side. Batoid matrix is 4x12; col<=5 left hand, col>=6 right. */
static void set_eyes(bool left_on, bool right_on) {
    gpio_pin_set_dt(&led_l, left_on ? 1 : 0);
    gpio_pin_set_dt(&led_r, right_on ? 1 : 0);
}

/* Classic status display: caps lock on left LED, connection on right LED. */
static void show_status(void) {
    bool connected = zmk_endpoint_is_connected();
    zmk_hid_indicators_t ind = zmk_hid_indicators_get_current_profile();
    bool caps = connected && (ind & HID_INDICATOR_CAPS_LOCK);

    /* GPIO_ACTIVE_LOW: value 1 == active (LED on) via gpio_pin_set_dt. */
    gpio_pin_set_dt(&led_l, caps ? 1 : 0);        /* caps lock */
    gpio_pin_set_dt(&led_r, connected ? 1 : 0);   /* connection */
}

static void eye_led_tick(struct k_timer *timer) {
    int64_t now = k_uptime_get();

    if ((now - last_key_ms) >= IDLE_MS) {
        if (!status_mode) {
            status_mode = true;
            turbo_mode = false;
            show_status();
        }
        return;
    }

    status_mode = false;

    if (turbo_mode) {
        /* alternate every tick */
        turbo_frame ^= 1;
        set_eyes(turbo_frame == 0, turbo_frame == 1);
    } else {
        /* hold the eye for the side last typed on (set in the listener) */
        set_eyes((last_key_ms > 0) && turbo_frame == 0,
                 (last_key_ms > 0) && turbo_frame == 1);
    }
}

K_TIMER_DEFINE(eye_led_timer, eye_led_tick, NULL);

static int eye_led_listener(const zmk_event_t *eh) {
    if (as_zmk_position_state_changed(eh)) {
        const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
        if (ev->state) {  /* key press */
            uint32_t col = ev->position % 12;
            bool right_hand = (col >= 6);
            /* mirrored, like original batoid: right hand -> left eye */
            turbo_frame = right_hand ? 0 : 1;
            last_key_ms = k_uptime_get();
            status_mode = false;
        }
    } else if (as_zmk_wpm_state_changed(eh)) {
        const struct zmk_wpm_state_changed *ev = as_zmk_wpm_state_changed(eh);
        turbo_mode = (ev->state >= TURBO_WPM);
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(eye_led, eye_led_listener);
ZMK_SUBSCRIPTION(eye_led, zmk_position_state_changed);
ZMK_SUBSCRIPTION(eye_led, zmk_wpm_state_changed);

static int eye_led_init(void) {
    if (!device_is_ready(led_l.port) || !device_is_ready(led_r.port)) {
        LOG_ERR("eye LED GPIO port not ready");
        return -ENODEV;
    }
    gpio_pin_configure_dt(&led_l, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_r, GPIO_OUTPUT_INACTIVE);

    /* start in status mode */
    status_mode = true;
    show_status();

    k_timer_start(&eye_led_timer, K_MSEC(TICK_MS), K_MSEC(TICK_MS));
    return 0;
}

SYS_INIT(eye_led_init, APPLICATION, 90);
