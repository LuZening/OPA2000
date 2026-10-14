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
#define CONFIG_VERSION          2U
#define CONFIG_FILENAME         "/fs/config.bin"
#define CONFIG_BACKUP_FILENAME  "/fs/config.bak"

bool config_init(void);            /*!< load from flash, fall back to defaults */
bool config_save(void);            /*!< append the current g_config to flash  */
void config_reset_default(void);   /*!< safe defaults into g_config, then save */
bool config_validate(void);        /*!< CRC of g_config still matches        */
bool config_sdcard_ready(void);    /*!< FatFS mounted and usable              */

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

const char *config_get_radio_type_string(RadioType type);
RadioType    config_set_radio_type_from_string(const char *str);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H */