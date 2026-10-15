/**
 * @file    config.h
 * @brief   Persistent configuration access for OPA2000 H7.
 *
 * @details The layout itself lives in `ConfigType` (opa2000.h) because the
 *          GUI and the comm path read the individual fields straight out of
 *          the global `g_config`.  This header is only the API around it:
 *          load / save / defaults / CRC, all backed by FatFS.
 *
 *          config.c owns both the layout and the CRC; bump CONFIG_VERSION
 *          whenever a field moves, so an old /fs/config.bin is rejected
 *          instead of being reinterpreted.
 *
 * @version 1.0
 * @date    2026-10
 * @author  Control Agent (wt-control branch)
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include "opa2000.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONFIG_MAGIC            0x4F504131U   /*!< "OPA1" */
/* 3 = added comp1_ref_mV / comp2_ref_mV (the two COMP comparator DAC
 *      references).  Bumped because the layout changed: without it an
 *      older record would pass the magic+version gate and have its bytes
 *      reinterpreted, leaving both DAC references at whatever followed
 *      them in the old layout. */
#define CONFIG_VERSION          3U
#define CONFIG_FILENAME         "/fs/config.bin"
#define CONFIG_BACKUP_FILENAME  "/fs/config.bak"

bool config_init(void);            /*!< load from flash, fall back to defaults */
bool config_save(void);            /*!< append the current g_config to flash  */
void config_reset_default(void);   /*!< safe defaults into g_config, then save */
bool config_validate(void);        /*!< CRC of g_config still matches        */
bool config_sdcard_ready(void);    /*!< FatFS mounted and usable              */

/* Returns true if the loaded (or default) g_config.touch transform is
 * still the identity and `calibrated` is 0, which means the operator
 * never ran the touch calibration pass.  Used by app_init_task to
 * decide whether to start the GUI on the calibration screen. */
bool config_touch_needs_calibration(void);

/*==============================================================================
 *  Deferred saving
 *
 *  Writing to the internal flash stalls instruction fetch for the whole
 *  erase + program sequence, so it must not happen where a deadline
 *  matters - not in an LVGL callback, not while the operator is dragging a
 *  slider.  Instead the GUI raises a flag when it edits something and
 *  app_config_task() does the actual write once per second, at the lowest
 *  priority in the system.
 *
 *  Rule of thumb: call config_mark_dirty() from any code that changes a
 *  g_config field.  Never call config_save() from a UI callback.
 *===========================================================================*/

/** @brief Record that g_config changed and should be written to flash. */
void config_mark_dirty(void);

/** @brief Clear the dirty flag (called by the save task once written). */
void config_clear_dirty(void);

/** @brief true if config_mark_dirty() has been called since the last write. */
bool config_is_dirty(void);

/*============================================================================
 *  Concurrency rules for g_config
 *============================================================================
 *  The PUBLIC config_*() functions (init / save / load / validate /
 *  reset_default / mark_dirty / clear_dirty / is_dirty) DO NOT take any
 *  lock.  Callers are responsible:
 *    - Single-threaded boot path (main() before osKernelStart) is safe
 *      with no lock at all.
 *    - Single-field writes (`g_config.x = y`) are atomic on Cortex-M7
 *      for word-size values; no lock needed, no lock taken.
 *    - Single-field reads (comm task reading g_config.radio_baud_rate)
 *      do not need a lock.
 *    - config_save() / config_init() / config_validate() / etc. each
 *      touch many fields.  When they are called from a multi-task
 *      context, the caller MUST wrap them in config_lock/unlock.
 *  The single design rule: any code that needs the whole struct to
 *  be consistent across two operations takes config_lock() / unlocks.
 *  ISR: no config_*() call from an ISR.
 *============================================================================*/
void config_lock(void);
void config_unlock(void);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H */