/**
 * @file overdrive.c
 * @brief RF-input overdrive indicator (see overdrive.h for the rationale).
 */

#include "overdrive.h"

/* Four samples at the ADC task's 50 ms cadence cover exactly 200 ms.  A
 * true maximum over the window, not an exponential decay: the operator's
 * question is "did the input ever get this high in the last 200 ms", and a
 * decay would answer a different one. */
#define OVERDRIVE_WINDOW_SAMPLES   4U

static uint16_t s_hist[OVERDRIVE_WINDOW_SAMPLES];
static uint8_t  s_hist_idx;
static uint16_t s_peak;
static bool     s_tripped;
static bool     s_flash_on;
static bool     s_beep_pending;
static bool     s_beep_latched;

void overdrive_init(void)
{
    for (uint8_t i = 0U; i < OVERDRIVE_WINDOW_SAMPLES; i++) {
        s_hist[i] = 0U;
    }
    s_hist_idx     = 0U;
    s_peak         = 0U;
    s_tripped      = false;
    s_flash_on     = false;
    s_beep_pending = false;
    s_beep_latched = false;
}

void overdrive_tick(uint32_t now_ms, uint16_t rf_in_raw)
{
    uint16_t peak;

    s_hist[s_hist_idx] = rf_in_raw;
    s_hist_idx = (uint8_t)((s_hist_idx + 1U) % OVERDRIVE_WINDOW_SAMPLES);

    peak = s_hist[0];
    for (uint8_t i = 1U; i < OVERDRIVE_WINDOW_SAMPLES; i++) {
        if (s_hist[i] > peak) {
            peak = s_hist[i];
        }
    }
    s_peak = peak;

    s_tripped  = (peak >= OVERDRIVE_TRIP_RAW);
    s_flash_on = s_tripped &&
                 (((now_ms / OVERDRIVE_FLASH_HALF_MS) & 1U) == 0U);

    /* One beep per excursion.  The latch is only released when the level
     * falls back below the trip point, so a long overdrive - or a trip that
     * bounces around the threshold - cannot restart the buzzer mid-cycle. */
    if (s_tripped) {
        if (!s_beep_latched) {
            s_beep_latched = true;
            s_beep_pending = true;
        }
    } else {
        s_beep_latched = false;
    }
}

uint16_t overdrive_peak_raw(void)
{
    return s_peak;
}

uint8_t overdrive_led_bright(void)
{
    uint32_t span;
    uint32_t above;
    uint32_t bright;

    if (s_tripped) {
        return s_flash_on ? 255U : 0U;
    }
    if (s_peak <= OVERDRIVE_LED_MIN_RAW) {
        return 0U;
    }

    span  = (uint32_t)(OVERDRIVE_TRIP_RAW - OVERDRIVE_LED_MIN_RAW);
    above = (uint32_t)(s_peak - OVERDRIVE_LED_MIN_RAW);
    bright = (above * 255U) / span;

    return (uint8_t)((bright > 255U) ? 255U : bright);
}

bool overdrive_is_tripped(void)
{
    return s_tripped;
}

bool overdrive_take_beep(void)
{
    bool pending = s_beep_pending;

    s_beep_pending = false;
    return pending;
}
