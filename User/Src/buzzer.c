/**
 * @file buzzer.c
 * @brief 3-tone warning buzzer implementation (TIM15_CH2 + TIM7 tick).
 *
 * @details State machine:
 *   0..2999 ticks (300 ms): high note (~2.5 kHz)
 *   3000..5999 ticks      : mid note  (~1.5 kHz)
 *   6000..8999 ticks      : low note  (~800 Hz)
 *   9000..39999 ticks     : silence (3 s)
 *   back to 0
 *
 * Notes are produced by writing TIM15_ARR to give the desired output
 * frequency (Timer clock = APB1 = 200 MHz / 16 prescaler = 12.5 MHz;
 * ARR = 12.5e6 / freq_hz; 50 % duty via CCR = ARR/2).
 *
 * The state machine is fully integer-only (no floating point) and is
 * safe to call from ISR context (TIM7 update IRQ at main prio 1).
 */

#include "buzzer.h"
#include "stm32h7xx_hal.h"

/* TIM15: APB1 timer clock = 200 MHz.  Prescaler chosen to make ARR
 * 16-bit friendly: 200 MHz / 16 = 12.5 MHz.  ARR=12.5e6/freq gives
 * ~5 kHz max, 200 Hz min. */
#define BUZZER_TIM_PRESCALER    15U       /*!< /16 -> 12.5 MHz */
#define BUZZER_TIM_CLOCK_HZ     12500000U
#define BUZZER_DUTY_50PCT(x)    ((x) >> 1)

/* 3-tone warning timing (100 us tick counts).  300 ms = 3000 ticks,
 * 3 s silence = 30000 ticks. */
#define TICKS_PER_NOTE          3000U
#define TICKS_SILENCE           30000U
#define TICK_TOTAL              (3U * TICKS_PER_NOTE + TICKS_SILENCE) /* 39000 */

/*============================================================================
 *  Private state (volatile for ISR/task sharing)
 *============================================================================*/

static volatile bool     s_active        = false;
static volatile uint32_t s_tick_counter  = 0U;

/*============================================================================
 *  Private helpers
 *============================================================================*/

static void buzzer_set_freq(uint32_t freq_hz)
{
    if (freq_hz == 0U) {
        HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);
        return;
    }
    uint32_t arr = BUZZER_TIM_CLOCK_HZ / freq_hz;
    if (arr == 0U) arr = 1U;
    if (arr > 0xFFFFU) arr = 0xFFFFU;
    htim15.Instance->PSC = BUZZER_TIM_PRESCALER;
    htim15.Instance->ARR = (uint16_t)arr;
    htim15.Instance->CCR2 = (uint16_t)BUZZER_DUTY_50PCT(arr);
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
}

/*============================================================================
 *  Public API
 *============================================================================*/

void buzzer_init(void)
{
    s_active       = false;
    s_tick_counter = 0U;
    htim15.Instance->PSC = BUZZER_TIM_PRESCALER;
    htim15.Instance->ARR = 0U;
    HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);
}

void buzzer_start(void)
{
    s_active       = true;
    s_tick_counter = 0U;
    buzzer_set_freq(BUZZER_NOTE_HIGH_HZ);
}

void buzzer_stop(void)
{
    s_active       = false;
    s_tick_counter = 0U;
    buzzer_set_freq(0U);
}

bool buzzer_is_active(void) { return s_active; }

void buzzer_tick(void)
{
    if (!s_active) {
        return;
    }

    /* TIM7 update fires every 100 us, so s_tick_counter increments
     * at 10 kHz.  Periods above 39 000 cover one full cycle. */
    uint32_t c = ++s_tick_counter;

    if      (c < TICKS_PER_NOTE)              buzzer_set_freq(BUZZER_NOTE_HIGH_HZ);
    else if (c < 2U * TICKS_PER_NOTE)        buzzer_set_freq(BUZZER_NOTE_MID_HZ);
    else if (c < 3U * TICKS_PER_NOTE)        buzzer_set_freq(BUZZER_NOTE_LOW_HZ);
    else if (c < TICK_TOTAL)                 buzzer_set_freq(0U);   /* silence */
    else                                     s_tick_counter = 0U;   /* next loop */
}