/**
 * @file power_control.c
 * @brief Power Control Implementation
 */

#include "power_control.h"
#include "stm32h7xx_hal.h"
#include "main.h"

/*============================================================================
 * GPIO Definitions (from CubeMX)
 *============================================================================*/

/* Power control GPIO ports */
#define PWR_ON_GPIO_PORT     GPIOE
#define PWR_ON_GPIO_PIN      GPIO_PIN_2

#define RF_BIAS_GPIO_PORT    GPIOB
#define RF_BIAS_GPIO_PIN     GPIO_PIN_11

#define RELAY1_GPIO_PORT     GPIOD
#define RELAY1_GPIO_PIN      GPIO_PIN_15

#define RELAY2_GPIO_PORT     GPIOD
#define RELAY2_GPIO_PIN      GPIO_PIN_14

#define ALL_FAULT_GPIO_PORT  GPIOE
#define ALL_FAULT_GPIO_PIN   GPIO_PIN_12

#define LCD_BL_GPIO_PORT     GPIOD
#define LCD_BL_GPIO_PIN      GPIO_PIN_9

/* Band selection GPIO ports and pins (low level active) */
/* Mapping: BAND_SEL1X = 15-10M, BAND_SEL2X = 20-17M, BAND_SEL3X = 40-30M, BAND_SEL4X = 80M, BAND_SEL5X = 160M */
static const struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} band_sel_pins[] = {
    {GPIOC, BAND_SEL1X_Pin},   /* BAND_SEL1X - BAND_15M_10M */
    {GPIOC, BAND_SEL2X_Pin},   /* BAND_SEL2X - BAND_20M_17M */
    {GPIOE, BAND_SEL3X_Pin},   /* BAND_SEL3X - BAND_40M_30M */
    {GPIOE, BAND_SEL4X_Pin},   /* BAND_SEL4X - BAND_80M */
    {GPIOD, BAND_SEL5X_Pin},   /* BAND_SEL5X - BAND_160M */
};

/* PTT input */
#define PTT_IN_GPIO_PORT     GPIOC
#define PTT_IN_GPIO_PIN      GPIO_PIN_14

/* Fan PWM */
#define FAN_GPIO_PORT        GPIOC
#define FAN_GPIO_PIN         GPIO_PIN_8

/* Buzzer - TIM15 Channel 2 */
#define BUZZER_TIM             htim15
#define BUZZER_CHANNEL         TIM_CHANNEL_2

/* Alert note frequencies */
#define NOTE_HIGH   880     /* A5 */
#define NOTE_MID    659     /* E5 */
#define NOTE_LOW    440     /* A4 */

/* Alert timing */
#define NOTE_DURATION_MS   300
#define NOTE_INTERVAL_MS   3000

/* RELAY delay in milliseconds */
#define RELAY_DELAY_MS       15

/*============================================================================
 * Private Variables
 *============================================================================*/

static BandType current_band = BAND_NONE;
static uint8_t current_fan_speed = 0;
static bool buzzer_active = false;

/* Fan speed hysteresis */
static FanSpeedType fan_current_speed = FAN_SPEED_LOW;
static bool fan_speed_increasing = true;

/* PTT state machine - all variables are volatile for ISR access */
typedef enum {
    PTT_STATE_IDLE,           /* Idle, not transmitting */
    PTT_STATE_TX_STARTING,    /* RELAY2 closed, waiting to close RELAY1 */
    PTT_STATE_TX_RUNNING,     /* Both RELAY1 and RELAY2 closed, transmitting */
    PTT_STATE_TX_STOPPING     /* RELAY1 released, waiting to release RELAY2 */
} PTTStateType;

static volatile PTTStateType ptt_state = PTT_STATE_IDLE;

/* TIM15 one-shot timer for RELAY delay */
static volatile bool relay_timer_active = false;
static volatile bool relay_timer_target_close = false;  /* true = closing RELAY1, false = opening RELAY2 */

/* Flag to indicate PTT has new event (for monitor task fallback) */
static volatile int ptt_event_flag = -1;  /* -1 = no event, 0 = released, 1 = pressed */

/*============================================================================
 * Private Functions
 *============================================================================*/

/**
 * @brief Set GPIO output state
 */
static void power_set_gpio(GPIO_TypeDef *port, uint16_t pin, bool state)
{
    HAL_GPIO_WritePin(port, pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief Get GPIO input state
 */
static bool power_get_gpio(GPIO_TypeDef *port, uint16_t pin)
{
    return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET;  /* Active low */
}

/**
 * @brief Start TIM15 one-shot timer for RELAY delay
 * @param close_relay1: true = delay to close RELAY1, false = delay to release RELAY2
 */
static void start_relay_timer(bool close_relay1)
{
    if (!relay_timer_active) {
        relay_timer_active = true;
        relay_timer_target_close = close_relay1;

        /* Configure TIM15 for RELAY_DELAY_MS */
        htim5.Instance->ARR = RELAY_DELAY_MS * 100;  /* Assume 100kHz clock -> 10us per tick */
        htim5.Instance->CNT = 0;
        __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
        HAL_TIM_Base_Start_IT(&htim5);
    }
}

/**
 * @brief Stop RELAY timer
 */
static void stop_relay_timer(void)
{
    if (relay_timer_active) {
        HAL_TIM_Base_Stop_IT(&htim5);
        relay_timer_active = false;
    }
}

/**
 * @brief Handle TIM15 interrupt for RELAY delay
 * @note Called from HAL_TIM_PeriodElapsedCallback in main.c
 */
void power_relay_timer_callback(void)
{
    /* Stop timer first */
    HAL_TIM_Base_Stop_IT(&htim5);
    relay_timer_active = false;

    /* Check current PTT state to determine what to do */
    /* If we were starting TX (waiting to close RELAY1) */
    if (ptt_state == PTT_STATE_TX_STARTING) {
        /* Close RELAY1 to complete TX sequence */
        HAL_GPIO_WritePin(RELAY1_GPIO_PORT, RELAY1_GPIO_PIN, GPIO_PIN_SET);
        ptt_state = PTT_STATE_TX_RUNNING;
    }
    /* If we were stopping TX (waiting to release RELAY2) */
    else if (ptt_state == PTT_STATE_TX_STOPPING) {
        /* Release RELAY2 to complete stop sequence */
        HAL_GPIO_WritePin(RELAY2_GPIO_PORT, RELAY2_GPIO_PIN, GPIO_PIN_RESET);
        ptt_state = PTT_STATE_IDLE;
    }
    /* Otherwise timer fired without valid state - ignore */
}

/**
 * @brief Turn off all band selections
 */
static void power_clear_all_bands(void)
{
    for (int i = 0; i < sizeof(band_sel_pins)/sizeof(band_sel_pins[0]); i++) {
        HAL_GPIO_WritePin(band_sel_pins[i].port, band_sel_pins[i].pin, GPIO_PIN_SET);
    }
}

/*============================================================================
 * Public Functions
 *============================================================================*/

/**
 * @brief Initialize power control
 */
void power_init(void)
{
    /* All outputs start off/disabled */

    /* Power signals */
    power_set_gpio(PWR_ON_GPIO_PORT, PWR_ON_GPIO_PIN, false);
    power_set_gpio(RF_BIAS_GPIO_PORT, RF_BIAS_GPIO_PIN, false);

    /* Relays */
    power_set_gpio(RELAY1_GPIO_PORT, RELAY1_GPIO_PIN, false);
    power_set_gpio(RELAY2_GPIO_PORT, RELAY2_GPIO_PIN, false);

    /* Fault output */
    power_set_gpio(ALL_FAULT_GPIO_PORT, ALL_FAULT_GPIO_PIN, false);

    /* LCD backlight - enable by default */
    power_set_gpio(LCD_BL_GPIO_PORT, LCD_BL_GPIO_PIN, true);

    /* Clear all band selections */
    power_clear_all_bands();

    /* Set default band to BAND_15M_10M on power-up */
    switch_band(BAND_15M_10M);

    /* Fan - initialize to 0% */
    power_set_fan_speed(0);

    /* Buzzer - off */
    power_set_buzzer(false);

    /* Initial fan speed to 50% as per requirements */
    current_fan_speed = FAN_SPEED_50_PERCENT;
}

/**
 * @brief Power on sequence
 */
void power_on(void)
{
    if (g_power_state == POWER_STATE_OFF) {
        /* Enable power and RF bias */
        power_set_gpio(PWR_ON_GPIO_PORT, PWR_ON_GPIO_PIN, true);
        power_set_gpio(RF_BIAS_GPIO_PORT, RF_BIAS_GPIO_PIN, true);

        g_power_state = POWER_STATE_STANDBY;
    }
}

/**
 * @brief Power off sequence
 */
void power_off(void)
{
    /* Disable transmit first */
    power_tx_disable();

    /* Disable power and RF bias */
    power_set_gpio(PWR_ON_GPIO_PORT, PWR_ON_GPIO_PIN, false);
    power_set_gpio(RF_BIAS_GPIO_PORT, RF_BIAS_GPIO_PIN, false);

    /* Disable all relays */
    power_set_gpio(RELAY1_GPIO_PORT, RELAY1_GPIO_PIN, false);
    power_set_gpio(RELAY2_GPIO_PORT, RELAY2_GPIO_PIN, false);

    g_power_state = POWER_STATE_OFF;
}

/**
 * @brief Standby mode
 */
void power_standby(void)
{
    if (g_power_state == POWER_STATE_OFF) {
        power_on();
    }

    /* Make sure not in TX mode */
    power_tx_disable();

    g_power_state = POWER_STATE_STANDBY;
}

/**
 * @brief Transmit enable
 */
void power_tx_enable(void)
{
    if (g_power_state == POWER_STATE_OFF) {
        return;
    }

    /* Enable relay 1 and 2 for transmit */
    power_set_gpio(RELAY1_GPIO_PORT, RELAY1_GPIO_PIN, true);
    power_set_gpio(RELAY2_GPIO_PORT, RELAY2_GPIO_PIN, true);

    g_power_state = POWER_STATE_TX;
    g_ptt_active = true;
}

/**
 * @brief Transmit disable
 */
void power_tx_disable(void)
{
    /* Disable relays */
    power_set_gpio(RELAY1_GPIO_PORT, RELAY1_GPIO_PIN, false);
    power_set_gpio(RELAY2_GPIO_PORT, RELAY2_GPIO_PIN, false);

    g_power_state = POWER_STATE_STANDBY;
    g_ptt_active = false;
}

/**
 * @brief Switch to specified band (LPF band selection)
 * @param band Target band
 * @note BAND_SEL1X~BAND_SEL6X are low-level active
 *       Default to BAND_15M_10M on invalid band or config fault
 */
void switch_band(BandType band)
{
    /* Release all relays first (set high) */
    HAL_GPIO_WritePin(GPIOC, BAND_SEL1X_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, BAND_SEL2X_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, BAND_SEL3X_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, BAND_SEL4X_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, BAND_SEL5X_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, BAND_SEL6X_Pin, GPIO_PIN_SET);

    /* Activate the selected band */
    switch (band) {
        case BAND_160M:
            HAL_GPIO_WritePin(GPIOD, BAND_SEL5X_Pin, GPIO_PIN_RESET);
            break;
        case BAND_80M:
            HAL_GPIO_WritePin(GPIOE, BAND_SEL4X_Pin, GPIO_PIN_RESET);
            break;
        case BAND_40M_30M:
            HAL_GPIO_WritePin(GPIOE, BAND_SEL3X_Pin, GPIO_PIN_RESET);
            break;
        case BAND_20M_17M:
            HAL_GPIO_WritePin(GPIOC, BAND_SEL2X_Pin, GPIO_PIN_RESET);
            break;
        case BAND_15M_10M:
            HAL_GPIO_WritePin(GPIOC, BAND_SEL1X_Pin, GPIO_PIN_RESET);
            break;
        case BAND_FAULT:
        case BAND_NONE:
        default:
            /* Default to BAND_15M_10M on fault */
            HAL_GPIO_WritePin(GPIOC, BAND_SEL1X_Pin, GPIO_PIN_RESET);
            break;
    }

    current_band = band;
    g_current_band = band;
}

/**
 * @brief Set band selection (legacy function, calls switch_band)
 */
void power_set_band(BandType band)
{
    if (band == BAND_NONE || band == BAND_FAULT) {
        /* Default to BAND_15M_10M */
        switch_band(BAND_15M_10M);
    } else {
        switch_band(band);
    }
}

/**
 * @brief Update band based on frequency
 */
void power_update_band(uint32_t frequency)
{
    BandType band = frequency_to_band(frequency);

    if (band != current_band) {
        power_set_band(band);
    }
}

/**
 * @brief Get PTT state
 */
bool power_get_ptt_state(void)
{
    /* Active low */
    return !HAL_GPIO_ReadPin(PTT_IN_GPIO_PORT, PTT_IN_GPIO_PIN);
}

/**
 * @brief Set fan speed (0-100%)
 */
void power_set_fan_speed(uint8_t speed_percent)
{
    static bool pwm_started = false;

    if (speed_percent > 100) speed_percent = 100;

    current_fan_speed = speed_percent;

    /* Reconfigure TIM3 for 10KHz PWM:
     * Timer clock = HCLK (assumed 200MHz)
     * PSC = 19 => divider = 20
     * ARR = 999 => period = 1000
     * PWM freq = 200MHz / 20 / 1000 = 10KHz
     * Duty = 0-100% maps to 0-999 */
    if (!pwm_started) {
        /* Stop timer, reconfigure, then start */
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
        __HAL_TIM_SET_PRESCALER(&htim3, 19);
        __HAL_TIM_SET_AUTORELOAD(&htim3, 999);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
        pwm_started = true;
    }

    uint32_t pulse = (speed_percent * 999 + 50) / 100;  /* 0-100% -> 0-999 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse);
}

/**
 * @brief Update fan based on temperature
 */
void power_update_fan(float temperature)
{
    uint8_t new_speed;

    if (fan_speed_increasing) {
        /* Speed increasing - follow thresholds exactly */
        if (temperature >= FAN_TEMP_THRESHOLD_100) {
            new_speed = FAN_SPEED_100_PERCENT;
            fan_current_speed = FAN_SPEED_HIGH;
        } else if (temperature >= FAN_TEMP_THRESHOLD_80) {
            new_speed = FAN_SPEED_80_PERCENT;
            fan_current_speed = FAN_SPEED_MED;
        } else {
            new_speed = FAN_SPEED_50_PERCENT;
            fan_current_speed = FAN_SPEED_LOW;
        }
    } else {
        /* Speed decreasing - apply hysteresis (10 degree lower) */
        if (temperature >= FAN_TEMP_THRESHOLD_100 - FAN_HYSTERESIS) {
            new_speed = FAN_SPEED_100_PERCENT;
            fan_current_speed = FAN_SPEED_HIGH;
        } else if (temperature >= FAN_TEMP_THRESHOLD_80 - FAN_HYSTERESIS) {
            new_speed = FAN_SPEED_80_PERCENT;
            fan_current_speed = FAN_SPEED_MED;
        } else {
            new_speed = FAN_SPEED_50_PERCENT;
            fan_current_speed = FAN_SPEED_LOW;
        }
    }

    /* Update speed direction */
    if (new_speed > current_fan_speed) {
        fan_speed_increasing = true;
    } else if (new_speed < current_fan_speed) {
        fan_speed_increasing = false;
    }

    power_set_fan_speed(new_speed);
}

/**
 * @brief Set buzzer state
 */
void power_set_buzzer(bool enable)
{
    buzzer_active = enable;

    if (enable) {
        /* Start buzzer PWM */
        /* HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2); */
    } else {
        /* Stop buzzer */
        /* HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2); */
    }
}

/**
 * @brief Set buzzer PWM frequency
 * @param freq_hz Frequency in Hz (0 = off)
 */
static void buzzer_set_frequency(uint32_t freq_hz)
{
    if (freq_hz == 0) {
        HAL_TIM_PWM_Stop(&BUZZER_TIM, BUZZER_CHANNEL);
        return;
    }

    /* Calculate ARR for given frequency
     * Timer clock = 200MHz / (prescaler+1) = 200MHz / 16 = 12.5MHz
     * ARR = 12.5MHz / freq_hz */
    uint32_t arr = 12500000 / freq_hz;

    BUZZER_TIM.Instance->ARR = arr;
    BUZZER_TIM.Instance->CCR2 = arr / 2;  /* 50% duty cycle */

    HAL_TIM_PWM_Start(&BUZZER_TIM, BUZZER_CHANNEL);
}

/**
 * @brief Play alert sound sequence (3 notes: high-mid-low)
 * Note: This should be called from a task, not ISR
 */
void power_play_alert(void)
{
    /* Alert sequence: 3 notes (high, mid, low), each 300ms, interval 3s */
    /* First cycle: HIGH-MID-LOW with 300ms each */
    /* Then wait 3s and repeat */

    /* This is a simple implementation that plays one sequence */
    /* For continuous alert, call this function periodically from task */

    static uint32_t alert_start_time = 0;
    static uint8_t alert_note = 0;

    uint32_t current_time = HAL_GetTick();

    if (alert_start_time == 0) {
        alert_start_time = current_time;
        alert_note = 0;
    }

    uint32_t elapsed = current_time - alert_start_time;

    /* Each note plays for 300ms */
    if (elapsed < 300) {
        buzzer_set_frequency(NOTE_HIGH);
    } else if (elapsed < 600) {
        buzzer_set_frequency(NOTE_MID);
    } else if (elapsed < 900) {
        buzzer_set_frequency(NOTE_LOW);
    } else {
        buzzer_set_frequency(0);  /* Stop */
        /* Reset for next cycle (called again after interval) */
        if (elapsed >= NOTE_INTERVAL_MS) {
            alert_start_time = current_time;
        }
    }
}

/**
 * @brief Set LCD backlight
 */
void power_set_lcd_backlight(bool enable)
{
    power_set_gpio(LCD_BL_GPIO_PORT, LCD_BL_GPIO_PIN, enable);
}

/**
 * @brief Set fault output
 */
void power_set_fault_out(bool fault)
{
    power_set_gpio(ALL_FAULT_GPIO_PORT, ALL_FAULT_GPIO_PIN, fault);
}

/**
 * @brief PTT input change ISR handler
 * @note Called from HAL_GPIO_EXTI_Callback - do NOT use FreeRTOS APIs
 *       This runs in interrupt context - must be fast and handle RELAY directly
 */
void power_control_ptt_isr(void)
{
    /* Read PTT state - active low (pressed = low) */
    bool ptt_pressed = (HAL_GPIO_ReadPin(PTT_IN_GPIO_PORT, PTT_IN_GPIO_PIN) == GPIO_PIN_RESET);

    /* Set flag for monitor task fallback detection */
    ptt_event_flag = ptt_pressed ? 1 : 0;

    if (ptt_pressed) {
        /* PTT pressed - START TX sequence */
        /* Sequence: Close RELAY2 immediately -> wait delay -> Close RELAY1 */

        /* Check current state - only start if IDLE or already TX */
        if (ptt_state == PTT_STATE_IDLE) {
            /* Close RELAY2 immediately (this connects PA output to antenna) */
            HAL_GPIO_WritePin(RELAY2_GPIO_PORT, RELAY2_GPIO_PIN, GPIO_PIN_SET);

            /* Start timer to close RELAY1 after delay */
            ptt_state = PTT_STATE_TX_STARTING;
            start_relay_timer(true);  /* true = closing RELAY1 */
        }
        else if (ptt_state == PTT_STATE_TX_STOPPING) {
            /* We were stopping TX but PTT pressed again - cancel stop sequence */
            /* Stop the timer */
            stop_relay_timer();

            /* Re-close RELAY1 immediately (we had already opened it) */
            HAL_GPIO_WritePin(RELAY1_GPIO_PORT, RELAY1_GPIO_PIN, GPIO_PIN_SET);

            /* Back to TX running */
            ptt_state = PTT_STATE_TX_RUNNING;
        }
        /* If already TX_RUNNING or TX_STARTING, ignore (already handling) */
    }
    else {
        /* PTT released - STOP TX sequence */
        /* Sequence: Release RELAY1 immediately -> wait delay -> Release RELAY2 */

        /* Check current state - only stop if running or starting */
        if (ptt_state == PTT_STATE_TX_RUNNING || ptt_state == PTT_STATE_TX_STARTING) {
            /* Stop any pending timer (if in TX_STARTING state) */
            stop_relay_timer();

            /* Release RELAY1 immediately (disconnect PA from antenna) */
            HAL_GPIO_WritePin(RELAY1_GPIO_PORT, RELAY1_GPIO_PIN, GPIO_PIN_RESET);

            /* Start timer to release RELAY2 after delay */
            ptt_state = PTT_STATE_TX_STOPPING;
            start_relay_timer(false);  /* false = releasing RELAY2 */
        }
        /* If already IDLE or TX_STOPPING, ignore (already handling or already off) */
    }
}

/**
 * @brief Get PTT flag (set by ISR)
 * @return -1 if no new flag, 0 if released, 1 if pressed
 */
int power_get_ptt_flag(void)
{
    return ptt_event_flag;
}

/**
 * @brief Clear PTT flag
 */
void power_clear_ptt_flag(void)
{
    ptt_event_flag = -1;
}

/**
 * @brief RELAY timer callback - called from TIM5 ISR
 * @note Handles delayed RELAY operations
 */
void power_relay_timer_callback(void);