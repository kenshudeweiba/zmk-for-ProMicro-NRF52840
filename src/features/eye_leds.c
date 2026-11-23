/*
 * Batoid "eye" LEDs for ZMK - based on the original QMK firmware
 * (firmware/kotte/batoid/keymaps/default/keymap.c).
 *
 * Three modes:
 *  - Standard (while typing): mirrored - right-hand keys light the LEFT eye,
 *    left-hand keys light the RIGHT eye. Keys released -> both off.
 *  - Turbo (WPM >= 70): left/right eyes alternate every 100 ms, 6-frame loop.
 *  - Idle (1 s after the last key): both eyes off, with an IRREGULAR blink:
 *    random wait (1.2-8 s, occasionally 9-18 s), then a slow, human-like
 *    blink (1-3 times) OR a wink (both eyes on -> one eye closes -> both
 *    on again). No fixed schedule and no repeating loop.
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

#define BLINK_WAIT_MIN  1200
#define BLINK_WAIT_MAX  8000
#define BLINK_LONG_MIN  9000
#define BLINK_LONG_MAX  18000
/* Slowed down to ~160-240 ms so a blink reads as a real, visible blink. */
#define BLINK_ON_MIN    160
#define BLINK_ON_MAX    240
#define BLINK_GAP_MIN   180
#define BLINK_GAP_MAX   320
/* Wink timing: both on -> one eye closes -> both on again. */
#define WINK_BOTH_MIN   160
#define WINK_BOTH_MAX   240
#define WINK_HALF_MIN   100
#define WINK_HALF_MAX   160
#define WINK_BACK_MIN   120
#define WINK_BACK_MAX   180

/* Build marker, forced into .rodata so the final binary can be verified. */
static const char eye_leds_build_tag[] __attribute__((used)) = "BATOID_EYE_LEDS_WINK_V5";

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

/* --- tiny xorshift32 PRNG, seeded from the boot cycle counter ------------ */
static uint32_t rng_state;

static uint32_t blink_rand(void)
{
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}

static int32_t rand_range(int32_t lo, int32_t hi)
{
    return lo + (int32_t)(blink_rand() % (uint32_t)(hi - lo + 1));
}

static void set_eyes(bool left_on, bool right_on);

/* --- idle blink scheduler ------------------------------------------------ */
enum blink_phase {
    BLINK_IDLE,  /* waiting for the next (random) animation */
    BLINK_ON,    /* ordinary blink: both eyes lit */
    BLINK_OFF,   /* ordinary blink: gap between two blinks of one burst */
    WINK_BOTH,   /* wink step 1: both eyes lit */
    WINK_HALF,   /* wink step 2: one eye closed */
    WINK_BACK,   /* wink step 3: closed eye reopens, both lit */
};

static enum blink_phase blink_phase = BLINK_IDLE;
static int64_t blink_clock;
static int32_t blink_wait_ms;
static int32_t blink_on_ms;
static int32_t blink_gap_ms;
static int blink_remaining;

static bool wink_close_left; /* which eye closes during a wink */
static int32_t wink_both_ms;
static int32_t wink_half_ms;
static int32_t wink_back_ms;

static void blink_reset(int64_t now)
{
    blink_phase = BLINK_IDLE;
    blink_clock = now;
    blink_remaining = 0;

    /* Roughly 15% of the pauses are much longer, so the rhythm feels natural. */
    blink_wait_ms = (blink_rand() % 100 < 15)
                        ? rand_range(BLINK_LONG_MIN, BLINK_LONG_MAX)
                        : rand_range(BLINK_WAIT_MIN, BLINK_WAIT_MAX);
}

static void start_wink(int64_t now)
{
    wink_close_left = (blink_rand() & 1) != 0;
    wink_both_ms = rand_range(WINK_BOTH_MIN, WINK_BOTH_MAX);
    wink_half_ms = rand_range(WINK_HALF_MIN, WINK_HALF_MAX);
    wink_back_ms = rand_range(WINK_BACK_MIN, WINK_BACK_MAX);
    blink_clock = now;
    blink_phase = WINK_BOTH;
}

static void blink_tick(int64_t now)
{
    switch (blink_phase) {
    case BLINK_IDLE:
        set_eyes(false, false);
        if (now - blink_clock >= blink_wait_ms) {
            /* ~30% of idle animations are a wink, the rest ordinary blinks. */
            if (blink_rand() % 100 < 30) {
                start_wink(now);
                break;
            }
            /* 1 blink ~60%, 2 blinks ~30%, 3 blinks ~10%. */
            uint32_t r = blink_rand() % 10;
            blink_remaining = (r < 6) ? 1 : (r < 9) ? 2 : 3;
            blink_on_ms = rand_range(BLINK_ON_MIN, BLINK_ON_MAX);
            blink_gap_ms = rand_range(BLINK_GAP_MIN, BLINK_GAP_MAX);
            blink_clock = now;
            blink_phase = BLINK_ON;
        }
        break;

    case BLINK_ON:
        set_eyes(true, true);
        if (now - blink_clock >= blink_on_ms) {
            blink_clock = now;
            blink_phase = BLINK_OFF;
        }
        break;

    case BLINK_OFF:
        set_eyes(false, false);
        if (now - blink_clock >= blink_gap_ms) {
            blink_clock = now;
            blink_remaining--;
            if (blink_remaining > 0) {
                /* Each blink of a burst gets its own timing. */
                blink_on_ms = rand_range(BLINK_ON_MIN, BLINK_ON_MAX);
                blink_gap_ms = rand_range(BLINK_GAP_MIN, BLINK_GAP_MAX);
                blink_phase = BLINK_ON;
            } else {
                blink_reset(now);
            }
        }
        break;

    case WINK_BOTH:
        set_eyes(true, true);
        if (now - blink_clock >= wink_both_ms) {
            blink_clock = now;
            blink_phase = WINK_HALF;
        }
        break;

    case WINK_HALF:
        /* One eye stays lit while the other closes for a moment. */
        set_eyes(!wink_close_left, wink_close_left);
        if (now - blink_clock >= wink_half_ms) {
            blink_clock = now;
            blink_phase = WINK_BACK;
        }
        break;

    case WINK_BACK:
        set_eyes(true, true);
        if (now - blink_clock >= wink_back_ms) {
            blink_reset(now);
        }
        break;
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

    if (idle_pending && now >= idle_start_at) {
        idle_playing = true;
        idle_pending = false;
        turbo_playing = false;
        blink_reset(now);
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
            /* Mix key timing into the PRNG so successive idle bursts differ. */
            rng_state ^= (uint32_t)k_uptime_get() * 2654435761u;
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

    rng_state = k_cycle_get_32() ^ 0x9E3779B9u;
    if (rng_state == 0) {
        rng_state = 0x12345678u;
    }
    blink_rand(); /* discard the first, weakest xorshift output */

    turbo_frame_ms = k_uptime_get();
    standard_mode = true;
    idle_playing = true;
    blink_reset(k_uptime_get());
    set_eyes(false, false);

    k_timer_start(&eye_led_timer, K_MSEC(TICK_MS), K_MSEC(TICK_MS));
    return 0;
}

SYS_INIT(eye_led_init, APPLICATION, 90);
