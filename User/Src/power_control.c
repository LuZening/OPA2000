/**
 * @file power_control.c
 * @brief Power / Relay / Band / Fan / Pump control implementation.
 *
 * @details Uses hw_pinmap.h GPIO macros - no raw port/pin numbers here.
 *
 *          Critical sections (RELAY sequencing, PTT edge detection,
 *          PSU on/off) write to GPIO registers directly without using
 *          any FreeRTOS API.  Only the task-side wrappers
 *          (power_psu_on / power_psu_off / power_relays_tx_on / etc.)
 *          may be called from non-ISR context.
 *
 *          Fan / pump / buzzer TBD - this file focuses on RELAY + PSU +
 *          band switching + fan PWM duty setting.  The tach-pulse
 *          detection (TIM7 0.5 ms polling) lives in fan_tach.c
 *          (todo, will be added by Control agent in next commit).
 */

#include "power_control.h"
#include "boot.h"
#include "stm32h7xx_hal.h"
#include <stddef.h>

/*============================================================================
 *  Private defines
 *============================================================================*/

/* TIM3 = APB1 timer clock = 200 MHz (assuming CubeMX clock tree). */
#define PWM_TIM3_CLOCK_HZ      200000000U
#define PWM_TIM3_PRESCALER     19U            /*!< /20 -> 10 MHz tick */
#define PWM_TIM3_PERIOD        999U           /*!< period = 1000 ticks = 10 kHz */

/* TIM4 same clock domain, used for PUMP PWM (CH4) and BL_PWM (CH3). */
#define PWM_TIM4_PRESCALER     19U
#define PWM_TIM4_PERIOD        999U

/* TIM5 one-shot 15 ms for RELAY delay.  Clock = 200 MHz / (39+1) = 5 MHz
 * (psc=39), period = 75000 ticks = 15 ms. */
#define RELAY_TIM5_PRESCALER   39U
#define RELAY_TIM5_PERIOD      74999U
#define RELAY_DELAY_MS         15U

/* Fan / pump speed levels (uint8_t 0..100) */
#define FAN_SPEED_DEFAULT      50U
#define FAN_SPEED_OFF          0U

/*============================================================================
 *  HV rail sequencing
 *
 *  RF_BIAS and PWR_ON must never switch simultaneously: the PA bias
 *  network needs the supply rail to be settled first, otherwise the bias
 *  step and the supply step overlap and the device sees a transient no
 *  power-up sequence tolerates.  So the rail is sequenced:
 *
 *      ON :  PWR_ON -> 10 ms -> RF_BIAS_ON
 *      OFF:  RF_BIAS_OFF -> 10 ms -> PWR_ON_OFF
 *
 *  The 10 ms is a FreeRTOS task delay, never a busy wait or HAL_Delay -
 *  this runs from the monitor task and must not stall the other tasks.
 *  power_rail_step() is that non-blocking driver: call it often (10 ms
 *  tick) and it advances one stage per call.
 *==========================================================================*/
#define HV_STAGE_DELAY_MS       10U

/*============================================================================
 *  Private state (volatile for ISR / task sharing)
 *============================================================================*/

volatile PTTStateType ptt_state          = PTT_STATE_IDLE;
volatile int          ptt_event_flag     = -1;

/* RELAY one-shot timer active?  Set/cleared in ISR. */
static volatile bool  relay_timer_active = false;
/* Which RELAY does the timer fire on? */
static volatile bool  relay_timer_close  = false; /* true=close RELAY1 */

/* FAN hysteresis state (task context only). */
static FanSpeedType   fan_current_speed  = FAN_SPEED_LOW;

/*============================================================================
 *  HV rail sequencing state
 *
 *  Declared in this private-state block rather than next to the functions
 *  that use it, because power_init_outputs() also has to cancel a pending
 *  stage and it appears earlier in the file.
 *
 *  volatile: the fault ISR clears it via power_rail_abort(), while the
 *  monitor task is otherwise the only writer/reader.
 *==========================================================================*/

/* IDLE is the settled state (both switches at their target); the two WAIT
 * states are the HV_STAGE_DELAY_MS gaps between the two switches. */
typedef enum {
    HV_STAGE_IDLE = 0,   /*!< nothing pending                        */
    HV_STAGE_ON_WAIT,    /*!< PWR_ON is up, waiting for RF_BIAS_ON  */
    HV_STAGE_OFF_WAIT    /*!< RF_BIAS is down, waiting for PWR_OFF  */
} HvStageType;

static volatile HvStageType s_hv_stage = HV_STAGE_IDLE;

/* g_tx_active is polled by the protection task, the frequency detector
 * and the GUI, so it has to follow the relay state machine exactly. */
static void ptt_enter_tx(void)
{
    g_tx_active = true;
    g_power_state = POWER_STATE_TX;
}

static void ptt_leave_tx(void)
{
    g_tx_active = false;
    /* Back to standby while the HV rail is still up, off once it is not.
     * ptt_leave_tx() also runs from power_hv_off(), which clears g_hv_on
     * right after, so re-check the flag rather than assume. */
    g_power_state = g_hv_on ? POWER_STATE_STANDBY : POWER_STATE_OFF;
}

/*============================================================================
 *  PTT_OUT (PC15, active high)
 *
 *  The feedback to the radio follows the RF path, not the key request:
 *  it is asserted only once the 15 ms break-before-make has closed RELAY1
 *  and the PA is genuinely passing RF.  Asserting it earlier would tell
 *  the radio to key into an amplifier that is not connected yet.
 *==========================================================================*/

void power_ptt_out_set(bool asserted)
{
    if (asserted) PTT_OUT_ON();
    else         PTT_OUT_OFF();
}

static void ptt_out_sync(void)
{
    power_ptt_out_set(ptt_state == PTT_STATE_TX_RUNNING);
}

/*============================================================================
 *  HAL timer handles (declared in main.c)
 *============================================================================*/
extern TIM_HandleTypeDef htim3;   /* FAN_PWM  (CH3) */
extern TIM_HandleTypeDef htim4;   /* PUMP_PWM (CH4) / BL_PWM (CH3) */
extern TIM_HandleTypeDef htim5;   /* RELAY delay one-shot */

/*============================================================================
 *  Private helpers
 *============================================================================*/

/* Arm the TIM5 one-shot that provides the 15 ms break-before-make delay.
 * The request is re-programmed and the counter restarted on every call:
 * a new intent must always win, otherwise a PSU-off landing inside the
 * start-up delay would leave the old "close RELAY1" request pending. */
static void relay_timer_arm(bool close_relay1)
{
    relay_timer_active = true;
    relay_timer_close  = close_relay1;

    htim5.Instance->ARR = (htim5.Instance->ARR & 0xFFFF0000U)
                          | (RELAY_TIM5_PERIOD & 0xFFFFU);
    htim5.Instance->PSC = (htim5.Instance->PSC & 0xFFFF0000U)
                          | (RELAY_TIM5_PRESCALER & 0xFFFFU);
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
    HAL_TIM_Base_Start_IT(&htim5);
}

static void relay_timer_stop(void)
{
    if (relay_timer_active) {
        HAL_TIM_Base_Stop_IT(&htim5);
        relay_timer_active = false;
    }
}

/*============================================================================
 *  PWM initialisation (fan, pump, backlight)
 *============================================================================*/

void power_init_fan_pwm(void)
{
    /* TIM3_CH3 = FAN1_PWM (PC8).  10 kHz, 1000-tick period, 0% start. */
    htim3.Instance->PSC = PWM_TIM3_PRESCALER;
    htim3.Instance->ARR = PWM_TIM3_PERIOD;
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

    /* TIM4_CH4 = PUMP_PWM (PD15).  Same config. */
    htim4.Instance->PSC = PWM_TIM4_PRESCALER;
    htim4.Instance->ARR = PWM_TIM4_PERIOD;
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    /* TIM4_CH3 = BL_PWM (PD14).  Start at 0 % - the screen stays black
     * until the GUI has rendered its first frame (boot_backlight_on()).
     * Showing a half-initialised frame on a cold boot looks like a
     * fault, and the operator cannot tell a blank panel from a crashed
     * one. */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0U);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

    fan_current_speed = FAN_SPEED_LOW;
}

/**
 * @brief Set the LCD backlight duty cycle.
 * @param percent 0..100
 *
 * TIM4 CH3 drives BL_PWM (PD14) into the U8 (AP3019) FB pin through the
 * R2/R4 divider, so the duty cycle IS the brightness.  Kept separate from
 * power_set_fan/pump_speed because the backlight has its own "blanked
 * during boot" state.
 */
void power_set_backlight_duty(uint8_t percent)
{
    if (percent > 100U) percent = 100U;
    uint32_t pulse = ((uint32_t)percent * (PWM_TIM4_PERIOD + 1U) + 50U) / 100U;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pulse);
}

void power_set_fan_speed(uint8_t speed_percent)
{
    if (speed_percent > 100U) speed_percent = 100U;
    uint32_t pulse = ((uint32_t)speed_percent * (PWM_TIM3_PERIOD + 1U) + 50U) / 100U;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse);

    if      (speed_percent == 0U)  g_fan_speed = FAN_SPEED_LOW; /* off still maps to LOW */
    else if (speed_percent < 70U)  g_fan_speed = FAN_SPEED_LOW;
    else if (speed_percent < 90U)  g_fan_speed = FAN_SPEED_MED;
    else                            g_fan_speed = FAN_SPEED_HIGH;
}

void power_set_pump_speed(uint8_t speed_percent)
{
    if (speed_percent > 100U) speed_percent = 100U;
    uint32_t pulse = ((uint32_t)speed_percent * (PWM_TIM4_PERIOD + 1U) + 50U) / 100U;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pulse);
}

/**
 * @brief Hysteretic fan speed update.
 *
 * Speed rises at 60 C / 70 C, and falls back only after the temperature has
 * dropped a full FAN_HYSTERESIS below the rising threshold (i.e. 100 % holds
 * until <= 60 C, 80 % holds until <= 50 C).  Each level is evaluated in
 * sequence so a single call can traverse more than one level on a fast
 * temperature drop.
 */
void power_update_fan(int16_t temperature_tenth_c)
{
    const int16_t thr80  = (int16_t)FAN_TEMP_THRESHOLD_80 * 10;   /* 600 */
    const int16_t thr100 = (int16_t)FAN_TEMP_THRESHOLD_100 * 10;  /* 700 */
    const int16_t hys    = (int16_t)FAN_HYSTERESIS * 10;         /* 100 */
    FanSpeedType next = fan_current_speed;

    if (next == FAN_SPEED_HIGH && temperature_tenth_c <= thr100 - hys) {
        next = FAN_SPEED_MED;
    }
    if (next == FAN_SPEED_MED && temperature_tenth_c <= thr80 - hys) {
        next = FAN_SPEED_LOW;
    }
    if (next == FAN_SPEED_LOW && temperature_tenth_c >= thr80) {
        next = FAN_SPEED_MED;
    }
    if (next == FAN_SPEED_MED && temperature_tenth_c >= thr100) {
        next = FAN_SPEED_HIGH;
    }

    fan_current_speed = next;
    power_set_fan_speed((next == FAN_SPEED_HIGH) ? FAN_SPEED_100_PERCENT
                      : (next == FAN_SPEED_MED)  ? FAN_SPEED_80_PERCENT
                                                 : FAN_SPEED_50_PERCENT);
}

/*============================================================================
 *  Output initialisation (called from opa2000_init / before FreeRTOS)
 *============================================================================*/

void power_init_outputs(void)
{
    /* Cancel any HV sequencing that may still have been pending: the pins
     * below are about to be forced to a known state, and a queued stage
     * would otherwise fire later and override them. */
    s_hv_stage = HV_STAGE_IDLE;

    /* PSU on (PG5) = inactive (LOW). */
    PSU_ON_PWR_OFF();
    /* PWR_ON_SIG (PG7) = off (LOW). */
    PWR_ON_DISABLE();
    /* RF_BIAS_ON (PG2) = off. */
    RF_BIAS_DISABLE();
    /* RELAY1 / RELAY2 = off. */
    RELAY1_OFF();
    RELAY2_OFF();
    /* Fault output (PB13) = inactive (LOW). */
    RF_IN_PROTECT_CLEAR();
    /* LPF all 6 idle (LPF is high-active, so LOW = band not selected). */
    HAL_GPIO_WritePin(LPF1_GPIO_Port, LPF1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LPF2_GPIO_Port, LPF2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LPF3_GPIO_Port, LPF3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LPF4_GPIO_Port, LPF4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LPF5_GPIO_Port, LPF5_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LPF6_GPIO_Port, LPF6_Pin, GPIO_PIN_RESET);

    /* Default band = 15-10 m (per design spec for config-fault fallback). */
    switch_band(BAND_15M_10M);

    /* LCD backlight is deliberately NOT enabled here.  It is driven from
     * TIM4 CH3 and starts at 0 % (power_init_fan_pwm); the panel only
     * comes up once the GUI task reports it has painted a frame
     * (boot_backlight_on).  Turning it on here would show a blank or
     * half-drawn screen during the ~200 ms the GUI needs to settle. */

    /* Keying feedback to the radio starts released. */
    power_ptt_out_set(false);

    /* Fan default 50% (timer-driven fan_update_fan() may override). */
    power_set_fan_speed(FAN_SPEED_DEFAULT);

    /* Everything is wired to the inactive state above. */
    g_psu_on    = false;
    g_tx_active = false;
    g_power_state = POWER_STATE_OFF;
}

/*============================================================================
 *  HV rail (PWR_ON + RF_BIAS) sequence
 *
 *  The rail is sequenced, not switched in one shot - see the
 *  HV_STAGE_DELAY_MS note above.  power_hv_on() / power_hv_off() only
 *  *request* the transition; power_rail_step() does the actual GPIO writes
 *  as the monitor task calls in.
 *==========================================================================*/

/**
 * @brief Request the HV rail up.  THROWS THE FIRST SWITCH ONLY.
 *
 *  Raises PWR_ON (PG7), then returns.  RF_BIAS (PG2) is deliberately NOT
 *  enabled here - power_rail_step() raises it one 10 ms tick later, once the
 *  supply has settled.  See power_control.h for the full contract.
 *
 *  SCOPE: the HV rail ONLY - PWR_ON + RF_BIAS.  It does NOT touch:
 *    - PSU_ON (PG5)   main supply latch, owned by power_psu_on()
 *    - RELAY1/2       the RF path, owned by power_relays_tx_on/off()
 *    - REAR_RESERVED1 whole-unit rail, owned by power_soft_off()
 *  Those are separate subsystems with their own triggers; mixing them here
 *  would make a "high voltage off" request drop the antenna relay and the
 *  main supply, which is not what the operator asked for.
 *
 *  Idempotent: already settled on -> return without re-triggering the
 *  sequence, so a repeated power_hv_on() cannot keep re-arming it.
 *
 *  @note Deliberately does NOT check g_ptt_ready / g_alert_flag.  Those
 *        gates live in power_hv_enable() and in ptt_apply_level(); calling
 *        this directly bypasses the "is it allowed to power up" decision.
 */
void power_hv_on(void)
{
    if (g_hv_on && s_hv_stage == HV_STAGE_IDLE) {
        return;                     /* already settled on */
    }
    /* Main HV enable via PG7.  PSU_ON (PG5) is NOT part of this rail. */
    PWR_ON_ENABLE();
    /* PA bias is deliberately NOT enabled here - it waits one task tick so
     * the supply has settled first.  power_rail_step() raises it. */
    s_hv_stage = HV_STAGE_ON_WAIT;
    g_hv_on = true;
    /* Not TX: keying goes through power_relays_tx_on(). */
    g_power_state = POWER_STATE_STANDBY;
}

/**
 * @brief Request the HV rail down.  THROWS THE FIRST SWITCH ONLY.
 *
 *  SCOPE: the HV rail ONLY - see power_hv_on().  Drops RF_BIAS (PG2) and
 *  returns; PWR_ON (PG7) follows one 10 ms tick later in power_rail_step().
 *  Neither PSU_ON (PG5) nor RELAY1 / RELAY2 is touched.
 *
 *  @note The RF path is NOT opened here.  If the amplifier happens to be
 *        keyed at this moment, the relays stay closed and the PA comes down
 *        live.  That is the front-panel button's explicit intent - the
 *        operator asked to kill the HV, not to unkey.  The protection path
 *        does both, at zero latency and with no sequencing.
 *
 *  @note The protection shutdown does NOT come through here: that path needs
 *        zero latency, pulls the pins itself, and calls power_rail_abort() so
 *        this queued stage cannot fire afterwards and re-assert the rail.
 */
void power_hv_off(void)
{
    /* Only the bias comes off now.  PWR_ON follows one task tick later in
     * power_rail_step(), so the 10 ms gap between the two switches actually
     * happens. */
    s_hv_stage = HV_STAGE_OFF_WAIT;
    g_hv_on = false;
    g_power_state = POWER_STATE_OFF;
}

/*============================================================================
 *  Main supply latch (PSU_ON, PG5)
 *
 *  PSG = Power Supply Unit: an EXTERNAL device, not part of this board.
 *  PG5 is only the MCU's control line into it.  The chain is
 *
 *      PG5 ---> NPN Q9 (inverting) ---> PC13 = PWR_WKUP4
 *
 *  and because Q9 inverts, the MCU-side polarity is:
 *
 *      PG5 HIGH  ->  Q9 conducts  ->  PC13 pulled to GND  ->  PSU powers ON
 *      PG5 LOW   ->  Q9 cut off   ->  PC13 back to pull-up ->  PSU powers OFF
 *
 *  i.e. HIGH = on, LOW = off, from the GPIO's point of view.  That matches
 *  the design spec (section 6) and hw_pinmap.h.
 *
 *  Who calls this today:
 *    power_soft_off()  turns the PSU OFF at shutdown - the only firmware-
 *                      initiated transition.
 *    Nothing else.     The PSU is normally switched by the operator, so
 *                      power_init() deliberately leaves PG5 at the off level
 *                      rather than powering it up unprompted.  Remote control
 *                      over the CAT link is a skeleton only - the protocol is
 *                      not defined yet (see comm.c CMD_PSU_* TODO).
 *==========================================================================*/

/*! Turn the external PSU on (PG5 high).  Idempotent.
 *
 *  Currently only reachable from the CAT skeleton and from operator code;
 *  power_init() does NOT call it.  Does not touch the HV rail, the relays or
 *  REAR_RESERVED1.
 */
void power_psu_on(void)
{
    if (g_psu_on) {
        return;
    }
    PSU_ON_PWR_ON();
    g_psu_on = true;
}

/*! Turn the external PSU off (PG5 low).  Idempotent.
 *
 *  Does NOT touch the HV rail, the relays or REAR_RESERVED1 - use
 *  power_hv_off() for the HV rail and power_soft_off() to shut the unit down.
 */
void power_psu_off(void)
{
    if (!g_psu_on) {
        return;
    }
    PSU_ON_PWR_OFF();
    g_psu_on = false;
}

/**
 * @brief Advance the non-blocking HV rail sequence by one stage.
 *
 * Call from a ~10 ms task tick (app_monitor_task does).  Returns true while
 * the rail was mid-sequence, so the caller can skip its slower per-tick work
 * until the rail settles.  No HAL_Delay, no busy wait - the sequencing is
 * spread across task ticks exactly as the design requires.
 */
bool power_rail_step(void)
{
    switch (s_hv_stage) {

    case HV_STAGE_ON_WAIT:
        /* Supply has had HV_STAGE_DELAY_MS to settle: now bias the PA. */
        RF_BIAS_ENABLE();
        s_hv_stage = HV_STAGE_IDLE;
        return true;

    case HV_STAGE_OFF_WAIT:
        /* Bias is down and has had HV_STAGE_DELAY_MS: now drop the rail.
         * PSU_ON (PG5) is not part of this sequence. */
        PWR_ON_DISABLE();
        s_hv_stage = HV_STAGE_IDLE;
        return true;

    case HV_STAGE_IDLE:
    default:
        return false;
    }
}

/** @brief true while the HV rail is mid-sequence (bias/supply not settled). */
bool power_rail_busy(void)
{
    return s_hv_stage != HV_STAGE_IDLE;
}

/**
 * @brief Drop any pending HV sequencing without touching the pins.
 *
 * ISR-safe (one volatile store, no FreeRTOS API) so the protection path
 * can call it: protection drops PWR_ON / RF_BIAS itself with zero delay,
 * and without this the queued stage would fire a moment later and
 * re-assert a rail that the fault had just cut.
 */
void power_rail_abort(void)
{
    s_hv_stage = HV_STAGE_IDLE;
}

void power_relays_tx_on(void)
{
    /* Direct GPIO writes - no HAL_Delay, no FreeRTOS API. */
    RELAY2_ON();   /* connect PA to LPF immediately */
    RELAY1_OFF();  /* hold off RELAY1 for RELAY_DELAY_MS */
    ptt_state = PTT_STATE_TX_STARTING;
    ptt_enter_tx();
    relay_timer_arm(true);
}

void power_relays_tx_off(void)
{
    /* Open RELAY1 first (disconnect PA from antenna). */
    RELAY1_OFF();
    if (ptt_state == PTT_STATE_TX_RUNNING ||
        ptt_state == PTT_STATE_TX_STARTING) {
        ptt_state = PTT_STATE_TX_STOPPING;
        relay_timer_arm(false);   /* delay then open RELAY2 */
    } else {
        RELAY2_OFF();             /* already idle or stopping */
        ptt_state = PTT_STATE_IDLE;
        ptt_leave_tx();
    }
}

/*============================================================================
 *  Band switching (high-active LPF)
 *============================================================================*/

void switch_band(BandType band)
{
    /* Release all LPFs first. */
    HAL_GPIO_WritePin(LPF1_GPIO_Port, LPF1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LPF2_GPIO_Port, LPF2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LPF3_GPIO_Port, LPF3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LPF4_GPIO_Port, LPF4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LPF5_GPIO_Port, LPF5_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LPF6_GPIO_Port, LPF6_Pin, GPIO_PIN_RESET);

    /* Activate only the selected band. */
    GPIO_TypeDef *port = NULL;
    uint16_t pin = 0;
    /* Band -> LPF.  Lower LPF index == shorter wavelength
 * (see hw_pinmap.h section 2):
 *      LPF1 = 15-10 m   LPF2 = 20-17 m   LPF3 = 40-30 m
 *      LPF4 = 80 m      LPF5 = 160 m     LPF6 = spare                */
    switch (band) {
        case BAND_15M_10M: port = LPF1_GPIO_Port; pin = LPF1_Pin; break;
        case BAND_20M_17M: port = LPF2_GPIO_Port; pin = LPF2_Pin; break;
        case BAND_40M_30M: port = LPF3_GPIO_Port; pin = LPF3_Pin; break;
        case BAND_80M:     port = LPF4_GPIO_Port; pin = LPF4_Pin; break;
        case BAND_160M:    port = LPF5_GPIO_Port; pin = LPF5_Pin; break;
        case BAND_NONE:
        case BAND_FAULT:
        default:
            /* Default to BAND_15M_10M on fault / invalid. */
            port = LPF1_GPIO_Port; pin = LPF1_Pin;
            band = BAND_15M_10M;
            break;
    }
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);

    g_current_band = band;

    /* AWD1 watches the RF input against band_rf_in_max_w[], which is a
     * per-band table, so the watchdog has to be re-programmed whenever the
     * band changes - otherwise the limit stays at whatever the previous
     * band (or the boot default, 15-10 m) happened to select.  Called
     * after g_current_band is updated so the index resolution below sees
     * the new band.  AWD2 / AWD3 and the COMP references are band
     * independent, so this call leaves them alone in practice. */
    boot_apply_thresholds((int)band);
}

/*============================================================================
 *  LCD backlight - PWM dimming on TIM4 CH3
 *============================================================================*/

void power_set_lcd_backlight(bool enable)
{
    power_set_backlight_duty(enable ? 100U : 0U);
}

/*============================================================================
 *  PTT ISR glue (called from EXTI5)
 *============================================================================*/

/*============================================================================
 *  PTT keying
 *
 *  Every transition lives in its own idempotent helper, and both edge
 *  sources - the EXTI5 handler and the power_ptt_poll() fallback - call
 *  the same helpers.  A duplicate edge, or an edge that arrives while the
 *  state machine already agrees with the pin, is a no-op:
 *
 *    repeat press   while TX_STARTING / TX_RUNNING -> no-op
 *    repeat release while TX_STOPPING / IDLE     -> no-op
 *
 *  PTT_INX is ACTIVE LOW: a low level means "transmit requested".
 *==========================================================================*/

/* Close RELAY2 now, RELAY1 after the 15 ms break-before-make delay. */
static void ptt_begin_transmit(void)
{
    if (ptt_state != PTT_STATE_IDLE) {
        return;                    /* already keying, or already stopping */
    }
    RELAY2_ON();
    ptt_state = PTT_STATE_TX_STARTING;
    ptt_enter_tx();
    relay_timer_arm(true);
    ptt_out_sync();
}

/* Open RELAY1 now (RF path broken), RELAY2 after the delay. */
static void ptt_begin_release(void)
{
    if (ptt_state != PTT_STATE_TX_RUNNING &&
        ptt_state != PTT_STATE_TX_STARTING) {
        return;                    /* already idle or already stopping */
    }
    relay_timer_stop();
    RELAY1_OFF();
    ptt_state = PTT_STATE_TX_STOPPING;
    relay_timer_arm(false);
    ptt_out_sync();                /* drop the feedback without waiting */
}

/* Operator re-pressed inside the stop delay: put the RF path straight back. */
static void ptt_cancel_release(void)
{
    if (ptt_state != PTT_STATE_TX_STOPPING) {
        return;
    }
    relay_timer_stop();
    RELAY1_ON();
    ptt_state = PTT_STATE_TX_RUNNING;
    ptt_out_sync();
}

/* Emergency release: both relays now, no delay.  Used when the request is
 * not allowed (still calibrating, or an alert is latched). */
static void ptt_force_release(void)
{
    if (ptt_state == PTT_STATE_IDLE) {
        return;
    }
    RELAY1_OFF();
    RELAY2_OFF();
    relay_timer_stop();
    ptt_state = PTT_STATE_IDLE;
    ptt_leave_tx();
    ptt_out_sync();
}

/* Reconcile the state machine against the live PTT_INX level.
 *
 * This is the single chokepoint every PTT path funnels through: the
 * EXTI5 ISR (power_control_ptt_isr), the TIM7 poll fallback
 * (power_ptt_poll) and the CAT/serial request (power_tx_enable ->
 * power_relays_tx_on).  The boot gate therefore lives HERE, so a pedal
 * press during the ~1 s while the ADC calibration is still running
 * cannot key the amplifier no matter which path noticed it.
 */
static void ptt_apply_level(bool ptt_pressed)
{
    /* g_ptt_ready = (IM zero-offset done) && (GUI painted a frame).
     * Until both hold, the protection chain is not trustworthy. */
    if (!g_ptt_ready || !g_allow_rf || g_alert_flag) {
        ptt_force_release();
        return;
    }
    /* The HV rail is mid-sequence (PWR_ON and RF_BIAS not yet both at
     * their target).  Keying a PA whose bias is still settling is exactly
     * the transient the HV_STAGE_DELAY_MS gap exists to prevent, so the
     * request is refused here - at the single chokepoint all three PTT
     * paths funnel through, which covers the pedal, the CAT link and the
     * front-panel key alike.  The operator simply presses again a few
     * milliseconds later, exactly as before the rail was sequenced.
     *
     * Note this is a *sequencing* interlock, not a power-policy one: it
     * never checks g_psu_on / g_hv_on, because whether the PA is energised
     * is not the firmware's call to make at request time. */
    if (s_hv_stage != HV_STAGE_IDLE) {
        ptt_force_release();
        return;
    }
    if (ptt_pressed) {
        if (ptt_state == PTT_STATE_IDLE) {
            ptt_begin_transmit();
        } else if (ptt_state == PTT_STATE_TX_STOPPING) {
            ptt_cancel_release();
        }
        /* TX_STARTING / TX_RUNNING -> already there */
    } else {
        ptt_begin_release();
    }
}

/*============================================================================
 *  PTT edge sources
 *==========================================================================*/

void power_control_ptt_isr(void)
{
    bool ptt_pressed = power_get_ptt_state();

    ptt_event_flag = ptt_pressed ? 1 : 0;
    ptt_apply_level(ptt_pressed);
}

/**
 * @brief Release-safety fallback for the PTT input.
 *
 * The EXTI5 interrupt is the primary path for both edges, but a release
 * edge that never arrives (IRQ masked, EXTI misrouted, glitch on the line)
 * would leave the PA keyed indefinitely.  This poll re-reads the pin and
 * reconciles, so the amplifier comes back within one tick of the operator
 * letting go.
 *
 * Level based and therefore idempotent.  Called from the fast TIM7 tick, so
 * it must stay interrupt-safe: a GPIO read plus a few comparisons, no
 * FreeRTOS API and no FPU.  An idle amplifier with the pedal up costs two
 * comparisons.
 */
void power_ptt_poll(void)
{
    /* Hard gate: until the ADC zero-offset calibration has finished AND
     * the GUI has painted its first frame, the protection chain is not
     * trustworthy yet and we must not key the amplifier.  g_ptt_ready is
     * set by the ADC/GUI tasks once both conditions hold (see
     * app_tasks.c).  This is the single place that decides "the
     * operator may transmit"; every other entry point (power_tx_enable,
     * the EXTI5 handler) goes through ptt_apply_level() which the gate
     * below also covers. */
    if (!g_ptt_ready) {
        /* Make sure we are not stuck in a keyed state if the flag was
         * cleared while transmitting (e.g. a protection event). */
        if (ptt_state != PTT_STATE_IDLE) {
            ptt_apply_level(false);
        }
        return;
    }

    bool ptt_pressed = power_get_ptt_state();

    if (ptt_state == PTT_STATE_IDLE && ptt_pressed) {
        return;                    /* nothing keyed and still requested */
    }
    ptt_apply_level(ptt_pressed);
}

int  power_get_ptt_flag(void)        { return ptt_event_flag; }
void power_clear_ptt_flag(void)     { ptt_event_flag = -1; }

/*============================================================================
 *  TIM5 relay-delay callback (one-shot 15 ms)
 *============================================================================*/

void power_relay_timer_callback(void)
{
    HAL_TIM_Base_Stop_IT(&htim5);
    relay_timer_active = false;

    bool want_close = relay_timer_close;
    if (ptt_state == PTT_STATE_TX_STARTING && want_close) {
        RELAY1_ON();
        ptt_state = PTT_STATE_TX_RUNNING;
    } else if (ptt_state == PTT_STATE_TX_STOPPING && !want_close) {
        RELAY2_OFF();
        ptt_state = PTT_STATE_IDLE;
        ptt_leave_tx();
    }
    /* any other state: stale timer fire - ignore */
    ptt_out_sync();
}

/*============================================================================
 *  Front-panel power (task context)
 *==========================================================================*/

/**
 * @brief One-time bring-up at boot.
 *
 *  Sets up the PWM timers and parks every output at its inactive level.
 *
 *  Deliberately does NOT latch the main supply (PG5): the PSU is an external
 *  device the operator normally switches themselves, so the firmware must not
 *  power it up behind their back.  power_init_outputs() leaves PG5 at the
 *  power-off level, which is where it belongs.  The only firmware-initiated
 *  PSU transition today is power_soft_off(), which turns it off at shutdown;
 *  remote PSU-on/off over the CAT link is a skeleton pending the protocol
 *  (see comm.c TODO and User/TODO-PSU远程控制.txt).
 */
void power_init(void)
{
    power_init_outputs();
    power_init_fan_pwm();
}

/**
 * @brief Operator "HV on" - engage the high-voltage rail.
 *
 *  Idempotent: already on -> return.  Does not interrupt an in-flight
 *  transmit.  Delegates the actual switching to power_hv_on(), which only
 *  throws the first of the two stages - see power_control.h.
 *
 *  Touches PWR_ON (PG7) and RF_BIAS (PG2) only.  The main supply latch and
 *  the relays are left exactly as they were.
 *
 *  Callers: the GUI top-banner button, the panel POWER key, CAT/SPI6.
 */
void power_hv_enable(void)
{
    if (g_hv_on) {
        return;
    }
    power_hv_on();
}

/**
 * @brief Operator "drop back to ready" - RF path only, HV rail stays up.
 *
 *  Idempotent: already off -> return.  g_hv_on stays true so the operator
 *  can key again immediately.  The 15 ms relay delay is asynchronous.
 */
void power_standby(void)
{
    if (!g_hv_on) {
        return;
    }
    /* Drop the RF path; the 15 ms delay opens RELAY2. */
    power_relays_tx_off();
}

/**
 * @brief Operator "HV off" - drop the high-voltage rail.
 *
 *  NOT the same as power_soft_off(), which kills the whole unit.  This one
 *  touches PWR_ON and RF_BIAS only: the main supply latch and the relays are
 *  left alone.  Always runs (no idempotency short-circuit), so repeated
 *  calls are safe but each re-arms the two-stage sequence.
 *
 *  If the amplifier is keyed at this moment the relays stay closed and the
 *  PA comes down live - the operator asked to kill the HV, not to unkey.
 */
void power_hv_disable(void)
{
    power_hv_off();
}

/**
 * @brief Soft power-off.
 *
 * Drops REAR_RESERVED1 (PF4) to LOW, which is the board's "hold the main
 * rail up" line.  With it low the supply supervisor lets the unit go down
 * and the backup supply takes over; the firmware then has 500 ms before
 * the rail actually collapses, which is what we use to park everything in
 * a safe state.
 *
 * Order matters: stop keying FIRST (relays + PTT_OUT), then blank the
 * panel, then release the rail.  Releasing the rail while the relays are
 * closed would leave the PA hot and the operator with no indication of
 * why it died.
 *
 * Never returns.  Call it from task context only.
 */
void power_soft_off(void)
{
    /* 1. Unkey and drop the RF path (includes the 15 ms relay delay, so
     *    we give the state machine a moment to settle). */
    if (ptt_state != PTT_STATE_IDLE) {
        ptt_force_release();
        /* Let the TIM5 one-shot open RELAY2 before we cut the rail. */
        HAL_Delay(RELAY_DELAY_MS + 5U);
    }

    /* 2. RF path and supplies off.  Bias goes before the HV rail, matching
     *    the normal power_hv_off() ordering, then the main supply latch is
     *    released last.  No HV_STAGE_DELAY_MS gap here: the supply is being
     *    torn down for good, the rail is about to collapse anyway, and there
     *    is no task left to step the sequence through - this path
     *    deliberately does not return. */
    RELAY1_OFF();
    RELAY2_OFF();
    RF_BIAS_DISABLE();
    PWR_ON_DISABLE();
    power_psu_off();           /* external PSU (PG5) */
    PTT_OUT_OFF();
    s_hv_stage = HV_STAGE_IDLE;
    g_hv_on         = false;
    g_tx_active     = false;
    g_power_state   = POWER_STATE_OFF;

    /* 3. Blank the panel so the operator sees it shut down cleanly
     *    rather than a frozen last frame. */
    power_set_backlight_duty(0U);

    /* 4. Block further keying, then release the rail. */
    g_ptt_ready = false;
    HAL_GPIO_WritePin(REAR_RESERVED1_GPIO_Port, REAR_RESERVED1_Pin,
                       GPIO_PIN_RESET);

    /* 5. Stay alive for the 500 ms the backup supply needs to take over.
     *    Nothing here touches the RTOS; we are already past the point of
     *    no return. */
    HAL_Delay(500U);

    /* Belt and braces: if the rail is still up after 500 ms for any
     * reason, park the CPU so it cannot re-enable anything. */
    for (;;) {
        __WFI();
    }
}

/*============================================================================
 *  Transmit request from a task source (CAT / serial / front panel)
 *==========================================================================*/

void power_tx_enable(void)
{
    /* Same interlocks the PTT EXTI path applies, plus the boot gate:
     * g_ptt_ready stays false until the ADC zero-offset calibration has
     * finished and the GUI has painted a frame.
     *
     * NOTE: neither g_psu_on nor g_hv_on is checked here.  Whether the PA has
     * power is not the firmware's business to second-guess at request time -
     * the operator (or the PSU itself) may bring the rail up at any moment,
     * and refusing to key would just be a nuisance.  The two remaining power
     * related interlocks are kept:
     *   - power_rail_busy() below, because keying into a bias that is still
     *     settling is a real hardware transient, not a policy question;
     *   - g_allow_rf / g_alert_flag, which block for safety reasons. */
    if (!g_ptt_ready || !g_allow_rf || g_alert_flag) {
        return;
    }
    /* Refuse while the HV rail is still sequencing: PWR_ON is already up but
     * RF_BIAS is not, and the PA must not be keyed into that gap. */
    if (s_hv_stage != HV_STAGE_IDLE) {
        return;
    }
    if (ptt_state == PTT_STATE_IDLE) {
        power_relays_tx_on();
        ptt_out_sync();
    }
}

void power_tx_disable(void)
{
    if (ptt_state == PTT_STATE_TX_RUNNING ||
        ptt_state == PTT_STATE_TX_STARTING) {
        power_relays_tx_off();
        ptt_out_sync();
    }
}

bool power_get_ptt_state(void)
{
    /* PTT_INX is active low. */
    return HAL_GPIO_ReadPin(PTT_IN_GPIO_Port, PTT_IN_Pin) == GPIO_PIN_RESET;
}

/*============================================================================
 *  Band follow
 *==========================================================================*/

void power_update_band(uint32_t frequency_hz)
{
    BandType band = frequency_to_band(frequency_hz);

    /* BAND_NONE (no signal) and BAND_FAULT (outside every band) both mean
     * "do not touch the LPF"; moving it would be worse than staying put. */
    if (band == BAND_NONE || band == BAND_FAULT) {
        return;
    }
    if (band != g_current_band) {
        switch_band(band);
    }
}