/**
 * @file protection.c
 * @brief Protection System Implementation
 */

#include "protection.h"
#include "power_control.h"
#include "stm32h7xx_hal.h"

/*============================================================================
 * Private Variables
 *============================================================================*/

static volatile bool protection_active = false;
static volatile AlertType last_fault = ALERT_NONE;

/* Flag for ISR to GUI task communication */
static volatile bool fault_occurred = false;

/* External GPIO definitions */
#define HARDPWR_ERR_GPIO_PORT    GPIOB
#define HARDPWR_ERR_GPIO_PIN     GPIO_PIN_12

#define OVERDRIVE_GPIO_PORT      GPIOE
#define OVERDRIVE_GPIO_PIN       GPIO_PIN_8

/*============================================================================
 * Public Functions
 *============================================================================*/

/**
 * @brief Initialize protection system
 */
void protection_init(void)
{
    protection_active = false;
    last_fault = ALERT_NONE;
    fault_occurred = false;
}

/**
 * @brief Check all protection conditions
 */
void protection_check(void)
{
    if (!g_ptt_active || g_power_state != POWER_STATE_TX) {
        /* Not transmitting, no protection needed */
        return;
    }

    /* Check NTC temperature > limit */
    if (g_adc_data.ntc_temp > g_config.protect_ntc_temp_c) {
        protection_trigger(ALERT_HARDWARE_FAULT);
        return;
    }

    /* Check transmit power 1 > limit */
    if (g_adc_data.tx_power1 > g_config.protect_tx_power1_w) {
        protection_trigger(ALERT_HARDWARE_FAULT);
        return;
    }

    /* Check transmit power 2 > limit */
    if (g_adc_data.tx_power2 > g_config.protect_tx_power2_w) {
        protection_trigger(ALERT_HARDWARE_FAULT);
        return;
    }

    /* Check reflected power 2 > 400W */
    if (g_adc_data.ref_power2 > g_config.protect_reflect_power2_w) {
        protection_trigger(ALERT_REFLECT_POWER_HIGH);
        return;
    }

    /* Check reflected power 2 > 200W AND SWR > 2.0 */
    if (g_adc_data.ref_power2 > g_config.protect_reflect_power2_swr &&
        g_adc_data.swr > g_config.protect_swr_threshold) {
        protection_trigger(ALERT_SWR_HIGH);
        return;
    }

    /* Check reflected power 1 > 600W */
    if (g_adc_data.ref_power1 > g_config.protect_reflect_power1_w) {
        protection_trigger(ALERT_REFLECT_POWER_HIGH);
        return;
    }

    /* Check main current > 40A */
    if (g_adc_data.current > g_config.protect_main_current_a) {
        protection_trigger(ALERT_CURRENT_HIGH);
        return;
    }
}

/**
 * @brief Trigger protection fault
 */
void protection_trigger(AlertType fault_type)
{
    /* Prevent re-entry */
    if (protection_active) {
        return;
    }

    protection_active = true;
    last_fault = fault_type;
    fault_occurred = true;
    g_alert_active = fault_type;
    g_alert_flag = true;

    /* Immediately disable PTT - direct GPIO write for speed */
    /* This is critical - we don't call functions that might use FreeRTOS APIs */

    /* Disable relays immediately */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);  /* RELAY1 */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);  /* RELAY2 */

    /* Shut down high voltage supply */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);   /* PWR_ON_SIG */

    /* Set fault output */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);    /* ALL_FAULT_OUT */

    /* Update global state - use direct assignment not bitwise */
    g_ptt_active = false;
    g_power_state = POWER_STATE_STANDBY;
}

/**
 * @brief Clear protection fault
 */
void protection_clear(void)
{
    protection_active = false;
    last_fault = ALERT_NONE;
    fault_occurred = false;
    g_alert_active = ALERT_NONE;
    g_alert_flag = false;

    /* Clear fault output */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);  /* ALL_FAULT_OUT */
}

/**
 * @brief Check if protection is active
 */
bool protection_is_active(void)
{
    return protection_active;
}

/**
 * @brief HARDPWR_ERR external interrupt handler
 * @note Called from EXTI ISR - must not use FreeRTOS APIs
 */
void protection_hardpwr_err_isr(void)
{
    /* Check if the pin is actually high */
    if (HAL_GPIO_ReadPin(HARDPWR_ERR_GPIO_PORT, HARDPWR_ERR_GPIO_PIN) == GPIO_PIN_SET) {
        protection_trigger(ALERT_HARDWARE_FAULT);
    }
}

/**
 * @brief COMP1 interrupt handler
 * @note Called from COMP ISR - must not use FreeRTOS APIs
 */
void protection_comp1_isr(void)
{
    /* Comparator 1 triggered - overcurrent or other fault */
    protection_trigger(ALERT_HARDWARE_FAULT);
}

/**
 * @brief COMP2 interrupt handler
 * @note Called from COMP ISR - must not use FreeRTOS APIs
 */
void protection_comp2_isr(void)
{
    /* Comparator 2 triggered */
    protection_trigger(ALERT_HARDWARE_FAULT);
}

/**
 * @brief Overdrive interrupt handler
 * @note Called from EXTI ISR - must not use FreeRTOS APIs
 */
void protection_overdrive_isr(void)
{
    /* Check if the pin is actually high */
    if (HAL_GPIO_ReadPin(OVERDRIVE_GPIO_PORT, OVERDRIVE_GPIO_PIN) == GPIO_PIN_SET) {
        protection_trigger(ALERT_OVERDRIVE);
    }
}

/**
 * @brief PTT input change callback
 */
void protection_ptt_change_isr(void)
{
    /* Check PTT state - active low */
    bool ptt_active = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_RESET);

    if (!ptt_active && g_power_state == POWER_STATE_TX) {
        /* PTT released during transmit */
        g_ptt_active = false;
        /* Relays will be disabled by normal flow */
    }
}

/**
 * @brief Frequency mismatch detected
 */
void protection_freq_mismatch(void)
{
    protection_trigger(ALERT_FREQ_MISMATCH);
}

/**
 * @brief Get fault description string
 */
const char* protection_get_fault_string(void)
{
    switch (last_fault) {
        case ALERT_SWR_HIGH:
            return "SWR Too High";
        case ALERT_REFLECT_POWER_HIGH:
            return "Reflected Power High";
        case ALERT_CURRENT_HIGH:
            return "Current Too High";
        case ALERT_HARDWARE_FAULT:
            return "Hardware Fault";
        case ALERT_OVERDRIVE:
            return "Overdrive Detected";
        case ALERT_FREQ_MISMATCH:
            return "Frequency Mismatch";
        default:
            return "Unknown Fault";
    }
}