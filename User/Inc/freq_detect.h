/**
 * @file freq_detect.h
 * @brief Frequency Detector Header
 */

#ifndef FREQ_DETECT_H
#define FREQ_DETECT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "opa2000.h"
#include "main.h"

/*============================================================================
 * External Variables
 *============================================================================*/

extern TIM_HandleTypeDef htim4;  /* TIM4 for frequency counting (ETR) */
extern TIM_HandleTypeDef htim7;  /* TIM7 for 100us check interval */

/*============================================================================
 * Function Prototypes
 *============================================================================*/

/**
 * @brief Initialize frequency detector
 */
void freq_detect_init(void);

/**
 * @brief Start frequency detection (TIM4 ETR counter + TIM7 100us timer)
 */
void freq_detect_start(void);

/**
 * @brief Stop frequency detection
 */
void freq_detect_stop(void);

/**
 * @brief Frequency detection ISR - called from TIM7 IRQ every 100us
 * @note Runs in ISR context - no FreeRTOS APIs, direct GPIO only
 */
void freq_detect_isr(void);

/**
 * @brief Task-level frequency check (updates band, handles alert state)
 * @note Called from protect task
 */
void freq_detect_check(void);

/**
 * @brief Get last measured frequency
 * @return Frequency in Hz
 */
uint32_t freq_detect_get_frequency(void);

/**
 * @brief Get current detected band
 * @return Band type
 */
BandType freq_detect_get_band(void);

#ifdef __cplusplus
}
#endif

#endif /* FREQ_DETECT_H */
