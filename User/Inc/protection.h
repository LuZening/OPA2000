/**
 * @file protection.h
 * @brief Protection System Header
 */

#ifndef PROTECTION_H
#define PROTECTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "opa2000.h"

/*============================================================================
 * Function Prototypes
 *============================================================================*/

/**
 * @brief Initialize protection system
 */
void protection_init(void);

/**
 * @brief Check all protection conditions
 * @note Should be called periodically from task
 */
void protection_check(void);

/**
 * @brief Trigger protection fault
 * @param fault_type Type of fault
 */
void protection_trigger(AlertType fault_type);

/**
 * @brief Clear protection fault
 */
void protection_clear(void);

/**
 * @brief Check if protection is active
 * @return true if protection is active
 */
bool protection_is_active(void);

/**
 * @brief External interrupt handler for HARDPWR_ERR
 */
void protection_hardpwr_err_isr(void);

/**
 * @brief External interrupt handler for COMP1
 */
void protection_comp1_isr(void);

/**
 * @brief External interrupt handler for COMP2
 */
void protection_comp2_isr(void);

/**
 * @brief External interrupt handler for Overdrive
 */
void protection_overdrive_isr(void);

/**
 * @brief PTT input change callback
 */
void protection_ptt_change_isr(void);

/**
 * @brief Frequency mismatch detected
 */
void protection_freq_mismatch(void);

/**
 * @brief Get protection fault description
 * @return Fault description string
 */
const char* protection_get_fault_string(void);

#ifdef __cplusplus
}
#endif

#endif /* PROTECTION_H */