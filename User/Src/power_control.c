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
 *  Private state (volatile for ISR / task sharing)
 *============================================================================*/

volatile PTTStateType ptt_state          = PTT_STATE_IDLE;
volatile int          ptt_event_flag     = -1;

/* RELAY one-shot timer active?  Set/cleared in ISR. */
static volatile bool  relay_timer_active = false;
/* Which RELAY does the timer fire on? */
static volatile bool  relay_timer_close  = false; /* true=close RELAY1 */

/* FAN hysteresis state (read by task, updated by task only). */
static FanSpeedType   fan_current_speed  = FAN_SPEED_LOW;
static bool           fan_speed_increasing = true;

/*============================================================================
 *  HAL timer handles (declared in main.c)
 *============================================================================*/
extern TIM_HandleTypeDef htim3;   /* FAN_PWM  (CH3) */
extern TIM_HandleTypeDef htim4;   /* PUMP_PWM (CH4) / BL_PWM (CH3) */
extern TIM_HandleTypeDef htim5;   /* RELAY delay one-shot */

/*============================================================================
 *  Private helpers
 *============================================================================*/

static void relay_timer_arm(bool close_relay1)
{
    if (!relay_timer_active) {
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

    /* TIM4_CH3 = BL_PWM (PD14).  Start at 100 % (default brightness). */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM_TIM4_PERIOD + 1U);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

    fan_current_speed = FAN_SPEED_LOW;
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
 *   - speed increasing: thresholds exactly at FAN_TEMP_THRESHOLD_80/100
 *   - speed decreasing: thresholds lowered by FAN_HYSTERESIS (sticky)
 */
void power_update_fan(int16_t temperature_tenth_c)
{
    uint8_t new_percent;
    int16_t thr80 = (int16_t)FAN_TEMP_THRESHOLD_80 * 10;     /* 60.0 C -> 600 */
    int16_t thr100 = (int16_t)FAN_TEMP_THRESHOLD_100 * 10;    /* 70.0 C -> 700 */
    int16_t hys = (int16_t)FAN_HYSTERESIS * 10;               /* 10 C -> 100 */

    if (fan_speed_increasing) {
        if      (temperature_tenth_c >= thr100) new_percent = 100;
        else if (temperature_tenth_c >= thr80)  new_percent = 80;
        else                                    new_percent = 50;
    } else {
        if      (temperature_tenth_c >= thr100 - hys) new_percent = 100;
        else if (temperature_tenth_c >= thr80  - hys) new_percent = 80;
        else                                          new_percent = 50;
    }

    FanSpeedType new_state =
        (new_percent >= 100) ? FAN_SPEED_HIGH :
        (new_percent >=  80) ? FAN_SPEED_MED  :
                                 FAN_SPEED_LOW;
    if      (new_state > fan_current_speed) fan_speed_increasing = true;
    else if (new_state < fan_current_speed) fan_speed_increasing = false;
    fan_current_speed = new_state;

    power_set_fan_speed(new_percent);
}

/*============================================================================
 *  Output initialisation (called from opa2000_init / before FreeRTOS)
 *============================================================================*/

void power_init_outputs(void)
{
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
    /* LPF all 6 (low-active, default = LOW). */
    HAL_GPIO_WritePin(LPF1_GPIO_Port, LPF1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LPF2_GPIO_Port, LPF2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LPF3_GPIO_Port, LPF3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LPF4_GPIO_Port, LPF4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LPF5_GPIO_Port, LPF5_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LPF6_GPIO_Port, LPF6_Pin, GPIO_PIN_RESET);

    /* Default band = 15-10 m (per design spec for config-fault fallback). */
    switch_band(BAND_15M_10M);

    /* LCD backlight on. */
    power_set_lcd_backlight(true);

    /* Fan default 50% (timer-driven fan_update_fan() may override). */
    power_set_fan_speed(FAN_SPEED_DEFAULT);
}

/*============================================================================
 *  PSU + RF_BIAS + RELAY sequence
 *============================================================================*/

void power_psu_on(void)
{
    /*PG5 -> high = Q9 latches PC13 LOW -> PSU turn-on latch active. */
    PSU_ON_PWR_ON();
    /* Main PSU enable via PG7. */
    PWR_ON_ENABLE();
    /* PA bias enable. */
    RF_BIAS_ENABLE();
    g_power_state = POWER_STATE_STANDBY;
}

void power_psu_off(void)
{
    /* Disable relays BEFORE disabling HV supply. */
    power_relays_tx_off();
    PWR_ON_DISABLE();
    RF_BIAS_DISABLE();
    PSU_ON_PWR_OFF();
    g_power_state = POWER_STATE_OFF;
}

void power_relays_tx_on(void)
{
    /* Direct GPIO writes - no HAL_Delay, no FreeRTOS API. */
    RELAY2_ON();   /* connect PA to LPF immediately */
    RELAY1_OFF();  /* hold off RELAY1 for RELAY_DELAY_MS */
    ptt_state = PTT_STATE_TX_STARTING;
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
    switch (band) {
        case BAND_160M:    port = LPF5_GPIO_Port; pin = LPF5_Pin; break;
        case BAND_80M:     port = LPF4_GPIO_Port; pin = LPF4_Pin; break;
        case BAND_40M_30M: port = LPF3_GPIO_Port; pin = LPF3_Pin; break;
        case BAND_20M_17M: port = LPF2_GPIO_Port; pin = LPF2_Pin; break;
        case BAND_15M_10M: port = LPF1_GPIO_Port; pin = LPF1_Pin; break;
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
}

/*============================================================================
 *  LCD backlight - simple GPIO control (PWM dimming in TIM4 task)
 *============================================================================*/

void power_set_lcd_backlight(bool enable)
{
    HAL_GPIO_WritePin(BL_PWM_GPIO_Port, BL_PWM_Pin,
                       enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/*============================================================================
 *  PTT ISR glue (called from EXTI5)
 *============================================================================*/

void power_control_ptt_isr(void)
{
    /* PTT_INX active-low. */
    bool ptt_pressed = (HAL_GPIO_ReadPin(PTT_IN_GPIO_Port, PTT_IN_Pin)
                        == GPIO_PIN_RESET);
    ptt_event_flag = ptt_pressed ? 1 : 0;

    /* Hard reject during calibration or fault - PTT blocked. */
    if (!g_allow_rf || g_alert_flag) {
        /* Force shutdown of any in-flight TX. */
        if (ptt_state == PTT_STATE_TX_STARTING ||
            ptt_state == PTT_STATE_TX_RUNNING) {
            RELAY1_OFF();
            RELAY2_OFF();
            relay_timer_stop();
            ptt_state = PTT_STATE_IDLE;
        }
        return;
    }

    if (ptt_pressed) {
        /* Start TX */
        if (ptt_state == PTT_STATE_IDLE) {
            RELAY2_ON();
            ptt_state = PTT_STATE_TX_STARTING;
            relay_timer_arm(true);
        } else if (ptt_state == PTT_STATE_TX_STOPPING) {
            /* Cancel stop sequence: re-close RELAY1 immediately. */
            relay_timer_stop();
            RELAY1_ON();
            ptt_state = PTT_STATE_TX_RUNNING;
        }
        /* else already TX_RUNNING / TX_STARTING -> ignore */
    } else {
        /* Stop TX */
        if (ptt_state == PTT_STATE_TX_RUNNING ||
            ptt_state == PTT_STATE_TX_STARTING) {
            relay_timer_stop();
            RELAY1_OFF();
            ptt_state = PTT_STATE_TX_STOPPING;
            relay_timer_arm(false);
        }
        /* else IDLE / TX_STOPPING -> ignore */
    }
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
    }
    /* any other state: stale timer fire - ignore */
}