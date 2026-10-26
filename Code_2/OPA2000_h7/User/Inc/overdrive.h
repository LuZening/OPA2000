/**
 * @file overdrive.h
 * @brief RF-input overdrive indicator: 200 ms peak detector + LED + P0 beep.
 *
 * @details Where the number comes from
 *   The overdrive envelope on the RF board is `RF_IN_OVER_FROM_DECK`,
 *   squared up by 74LVC1G14GV into OVERDRIVE_SAMPLE / OVERDRIVE_PEAK.  It
 *   never reaches the MCU board as its own signal: this board has no
 *   dedicated overdrive ADC channel, and the external-interrupt route was
 *   dropped from the design (see the firmware design spec, "Overdrive 的
 *   外部中断输入取消了").  Overdrive is judged in software from the same
 *   node COMP1 watches - ADC1 rank 0, INP4 / PC4, the D5/R40 peak detector -
 *   which is why this module reads ADC1_RANK_RF_IN.
 *
 *   The D5/R40 detector gain is NOT calibrated, so no wattage can be derived
 *   from it.  Both thresholds below are therefore raw 16-bit ADC codes and
 *   are placeholders until measured on the bench.
 *
 * @details Behaviour
 *   - The reported level is the maximum raw code seen over the last 200 ms
 *     (4 x 50 ms ADC-task passes), so a single overshoot is not averaged
 *     away by the 50 ms sampling rate.
 *   - LED brightness maps linearly from OVERDRIVE_LED_MIN_RAW (0 = off) to
 *     OVERDRIVE_TRIP_RAW (255 = full).
 *   - At or above OVERDRIVE_TRIP_RAW the board is overdriven: the LED
 *     flashes red at 2 Hz and one P0 buzzer cycle is requested.  The request
 *     is a one-shot per excursion - it cannot re-arm until the level drops
 *     back below the trip point, so a complete buzzer cycle is never cut
 *     short and never re-triggered while it is still playing.
 */

#ifndef OVERDRIVE_H
#define OVERDRIVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/*============================================================================
 *  Tunables - PLACEHOLDERS, to be replaced with measured values
 *============================================================================*/

/*!< Below this raw code the LED stays dark.  Placeholder. */
#define OVERDRIVE_LED_MIN_RAW      1000U

/*!< At or above this raw code the input is overdriven: LED flashes red and
 *   one P0 buzzer cycle is requested.  Placeholder - must be measured
 *   against the D5/R40 detector on the bench, in the same raw-code units
 *   COMP1's reference is derived from. */
#define OVERDRIVE_TRIP_RAW         40000U

/*!< One full red/off cycle = 500 ms, i.e. 250 ms in each half. */
#define OVERDRIVE_FLASH_HALF_MS    250U

/*============================================================================
 *  Public API
 *============================================================================*/

/* Reset the peak window and the beep latch.  Called once from the ADC task
 * before the first sample arrives. */
void overdrive_init(void);

/* Feed one RF-input sample.  Call from the ADC task every pass (50 ms) with
 * HAL_GetTick() and the raw code from g_adc_data.adc1_raw[ADC1_RANK_RF_IN].
 * Deliberately free of floating point and of LVGL so it can run in any task. */
void overdrive_tick(uint32_t now_ms, uint16_t rf_in_raw);

/*!< Maximum raw code over the last 200 ms window. */
uint16_t overdrive_peak_raw(void);

/*!< LED brightness, 0 (off) .. 255 (full).  Already includes the red flash
 *   while overdriven. */
uint8_t overdrive_led_bright(void);

/*!< True while the 200 ms peak is at or above OVERDRIVE_TRIP_RAW. */
bool overdrive_is_tripped(void);

/*!< One-shot: true exactly once per overdrive excursion.  The caller starts
 *   the P0 buzzer with it; the latch re-arms only after the level falls back
 *   below the trip point. */
bool overdrive_take_beep(void);

#ifdef __cplusplus
}
#endif

#endif /* OVERDRIVE_H */
