/**
 * @file power_control.h
 * @brief Power Control Header (PTT, Relay, Band Switching)
 */

#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "opa2000.h"

#ifndef LVGL_SIM
#include "main.h"

/*============================================================================
 * External Variables
 *============================================================================*/

extern TIM_HandleTypeDef htim3;   /* For fan PWM */
extern TIM_HandleTypeDef htim5;   /* For RELAY delay timer */
extern TIM_HandleTypeDef htim15;  /* For buzzer PWM */
#endif

/*============================================================================
 * Function Prototypes
 *============================================================================*/

/**
 * @brief Initialize power control outputs
 */
void power_init(void);

/**
 * @brief Power on sequence
 */
void power_on(void);

/**
 * @brief Power off sequence
 */
void power_off(void);

/**
 * @brief Standby mode (ready but not transmitting)
 */
void power_standby(void);

/**
 * @brief Transmit mode enable
 */
void power_tx_enable(void);

/**
 * @brief Transmit mode disable
 */
void power_tx_disable(void);

/**
 * @brief Set band selection
 * @param band Band type
 */
void power_set_band(BandType band);

/**
 * @brief Switch to specified band (LPF band selection)
 * @param band Target band
 */
void switch_band(BandType band);

/**
 * @brief Update band based on frequency
 * @param frequency Frequency in Hz
 */
void power_update_band(uint32_t frequency);

/**
 * @brief Get PTT input state
 * @return true if PTT is active (low)
 */
bool power_get_ptt_state(void);

/**
 * @brief Set fan speed
 * @param speed Fan speed percentage (0-100)
 */
void power_set_fan_speed(uint8_t speed_percent);

/**
 * @brief Update fan speed based on temperature
 * @param temperature Current temperature in Celsius
 */
void power_update_fan(float temperature);

/**
 * @brief Set buzzer state
 * @param enable Enable or disable
 */
void power_set_buzzer(bool enable);

/**
 * @brief Play alert sound sequence
 */
void power_play_alert(void);

/**
 * @brief Set LCD backlight
 * @param enable Enable or disable
 */
void power_set_lcd_backlight(bool enable);

/**
 * @brief Trigger fault output
 * @param fault True to indicate fault
 */
void power_set_fault_out(bool fault);

/**
 * @brief Get PTT flag (set by ISR, cleared by task)
 * @return -1 if no new flag, 0 if released, 1 if pressed
 */
int power_get_ptt_flag(void);

/**
 * @brief Clear PTT flag
 */
void power_clear_ptt_flag(void);

/**
 * @brief PTT input change ISR handler
 * @note Called from HAL_GPIO_EXTI_Callback - do NOT use FreeRTOS APIs
 */
void power_control_ptt_isr(void);

/**
 * @brief TIM5 relay timer callback
 * @note Called from TIM5_IRQHandler - handles delayed RELAY operations
 */
void power_relay_timer_callback(void);

#ifdef __cplusplus
}
#endif

#endif /* POWER_CONTROL_H */