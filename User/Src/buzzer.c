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
/* Wall-clock deadline in HAL_GetTick() ms at which the buzzer auto-stops.
 * 0 means "no timeout - loop until buzzer_stop()".  Capped at 10 s by
 * design rule: even a P0 alert must not keep the operator's ear ringing
 * forever. */
static volatile uint32_t s_deadline_ms   = 0U;
/* Which severity is currently driving the buzzer.  Used by the
 * re-trigger guards below: a new same-severity call while one is
 * already in flight is a no-op, while a stricter-severity call always
 * takes over.  ALERT_SEV_NONE = idle. */
static volatile AlertSeverity s_current_sev = ALERT_SEV_NONE;

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
    s_deadline_ms  = 0U;
    s_current_sev  = ALERT_SEV_NONE;
    htim15.Instance->PSC = BUZZER_TIM_PRESCALER;
    htim15.Instance->ARR = 0U;
    HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);
}

/* Re-trigger guard.  Returns true if a new call of `incoming` severity
 * should actually engage the buzzer:
 *
 *   - idle                  -> always engage
 *   - same severity already -> DROP, the operator already heard it and
 *                              we do not want to reset the 10 s cap
 *                              (otherwise a chatter source could keep
 *                              the buzzer ringing past 10 s by re-arming
 *                              faster than the deadline)
 *   - stricter severity     -> TAKE OVER, reset the state machine and
 *                              the deadline so the operator gets the
 *                              full pattern of the more urgent alert
 *   - looser severity       -> DROP, the running pattern is more
 *                              important; do not interrupt it
 *
 * Severity semantics: ALERT_SEV_P0 is *more* severe than ALERT_SEV_P1,
 * but the enum has P0=1 < P1=2 (P0 declared first).  Therefore
 * "incoming is stricter than current" means incoming has a *smaller*
 * numeric value, and same-severity is "==".
 */
static bool buzzer_should_engage(AlertSeverity incoming)
{
    if (!s_active || s_current_sev == ALERT_SEV_NONE) {
        return true;
    }
    /* Re-arm only on upgrade to a *stricter* (smaller-numbered) severity.
     * Equal-severity is dropped to preserve the deadline; a less-strict
     * call (numerically larger) is dropped so we don't downgrade an
     * active emergency. */
    if (incoming == s_current_sev) {
        return false;
    }
    if (incoming == ALERT_SEV_NONE) {
        return false;
    }
    if (s_current_sev == ALERT_SEV_NONE) {
        return true;
    }
    /* Smaller enum value = stricter.  P0(1) is stricter than P1(2). */
    return (uint32_t)incoming < (uint32_t)s_current_sev;
}

void buzzer_start(void)
{
    /* Default: P0-style 3-tone loop, capped at 10 s.  P0 always wins
     * over P1; P0-over-P0 is suppressed (re-trigger guard). */
    buzzer_start_with_timeout(10000U);
}

void buzzer_start_with_timeout(uint32_t max_ms)
{
    /* Cap at 10 s: design rule - no single fault keeps the buzzer ringing
     * for more than 10 s, even if the operator never dismisses.  Caller-
     * requested values above 10 s are silently clamped. */
    if (max_ms == 0U || max_ms > 10000U) {
        max_ms = 10000U;
    }
    if (!buzzer_should_engage(ALERT_SEV_P0)) {
        return;     /* P0 already running, or P0 in flight and this is a repeat */
    }
    s_active       = true;
    s_tick_counter = 0U;
    s_deadline_ms  = HAL_GetTick() + max_ms;
    s_current_sev  = ALERT_SEV_P0;
    buzzer_set_freq(BUZZER_NOTE_HIGH_HZ);
}

void buzzer_start_p1(void)
{
    /* P1 alert: thermal / fan / pump.  Short single-tone - operationally
     * we want the operator to notice without making them hunt for the
     * mute.  The 3-tone loop was designed for P0 emergencies; P1 does
     * not need that much theatre.  P1-over-P1 is suppressed; P1 is
     * dropped if P0 is already sounding (P0 wins). */
    if (!buzzer_should_engage(ALERT_SEV_P1)) {
        return;     /* P1 already running, or P0 in flight (P0 > P1) */
    }
    s_active       = true;
    s_tick_counter = 0U;
    s_deadline_ms  = HAL_GetTick() + 1000U;        /* 1 s, still under the 10 s cap */
    s_current_sev  = ALERT_SEV_P1;
    buzzer_set_freq(BUZZER_NOTE_MID_HZ);
}

void buzzer_stop(void)
{
    s_active       = false;
    s_tick_counter = 0U;
    s_deadline_ms  = 0U;
    s_current_sev  = ALERT_SEV_NONE;
    /* Pull the PWM output low (freq 0) AND clear the ARR register so a
     * subsequent buzzer_start() cannot read a stale note from a previous
     * alert.  ARR is what the host test reads to identify which note the
     * PWM is currently generating. */
    buzzer_set_freq(0U);
    htim15.Instance->ARR = 0U;
}

bool buzzer_is_active(void) { return s_active; }

#if defined(HOST_TEST_NATIVE)
/* See buzzer.h: the 10 s cap is wall-clock based and a host test cannot
 * sleep through it, so the deadline is made reachable here rather than
 * reaching into the static from the test file (which would not link). */
uint32_t buzzer_test_get_deadline(void) { return s_deadline_ms; }
void buzzer_test_set_deadline(uint32_t deadline_ms) { s_deadline_ms = deadline_ms; }
/* Move the deadline strictly into the past.  HAL_GetTick() is 0 right
 * after test_wall_clock_reset(), so a naive "- 1" would wrap to
 * 0xFFFFFFFF and the wrap-safe comparison in buzzer_tick() would then
 * read it as still-in-the-future.  Subtracting a full second keeps the
 * result on the past side of the comparison for any plausible clock. */
void buzzer_test_expire_deadline(void) { s_deadline_ms = HAL_GetTick() - 1000U; }
#endif

void buzzer_tick(void)
{
    if (!s_active) {
        return;
    }

    /* Wall-clock deadline check - takes priority over the 3-tone cycle
     * so the 10 s hard cap is enforced regardless of where we are in the
     * loop.  HAL_GetTick() is 1 ms granularity (SysTick), plenty for a
     * 10 s deadline.  Unsigned subtraction is wrap-safe. */
    if (s_deadline_ms != 0U) {
        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - s_deadline_ms) < 0x80000000U) {
            /* now >= deadline: stop. */
            buzzer_stop();
            return;
        }
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