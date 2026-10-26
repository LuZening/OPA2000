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

/* Human-readable name of a RadioType ("ICOM" / "KENWOOD" / ...), for the
 * CAT CONFIG query and the GUI link caption.  Out-of-range values fall
 * back to the UNKNOWN entry rather than indexing past the table. */
const char *config_get_radio_type_string(RadioType type);

/* Inverse of the above: parse a name back into a RadioType.  Exact match
 * first, then case-insensitive, then a few partial aliases ("civ" ->
 * ICOM).  Returns RADIO_TYPE_UNKNOWN when nothing matches. */
RadioType config_set_radio_type_from_string(const char *str);

/*==============================================================================
 *  Deferred saving
 *
 *  Writing to the internal flash stalls instruction fetch for the whole
 *  erase + program sequence, so it must not happen where a deadline
 *  matters - not in an LVGL callback, not while the operator is dragging a
 *  slider.  Instead the code that edits a field raises a flag, and
 *  app_config_task() does the actual write at the lowest priority in the
 *  system.
 *
 *  HOW THE WAKE-UP WORKS
 *  The save task does NOT poll: it blocks on a thread flag
 *  (CONFIG_SAVE_FLAG).  Whoever changes something calls config_mark_dirty(),
 *  which sets the dirty flag AND posts that flag to the save task, so the
 *  task wakes at once instead of on the next timer tick.
 *
 *  The 1 s tick that used to drive it is gone.  It existed only to
 *  coalesce a burst of edits into one write, and the staging in the GUI
 *  (s_menu_dirty, committed when the settings menu closes) does that job
 *  much better: the save now lands on a boundary the operator perceives
 *  as "I finished editing" instead of on an arbitrary timer edge.
 *
 *  Two things keep a lost wake-up from losing an edit:
 *    - The flag is sticky.  A config_mark_dirty() that arrives BEFORE the
 *      save task reaches its wait is remembered by the flag, not lost.
 *    - The save task re-checks config_is_dirty() after every wake, so a
 *      write that is requested twice writes once, and a write requested
 *      while a previous one is in flight is not dropped.
 *
 *  Rule of thumb: call config_mark_dirty() from any code that changes a
 *  g_config field.  Never call config_save() from a UI callback.
 *===========================================================================*/

/** @brief Record that g_config changed and should be written to flash.
 *
 *  Sets the dirty flag and wakes the save task, which performs the write
 *  at the lowest priority in the system.  Safe from any task context. */
void config_mark_dirty(void);

/** @brief Clear the dirty flag (called by the save task once written). */
void config_clear_dirty(void);

/** @brief true if config_mark_dirty() has been called since the last write. */
bool config_is_dirty(void);

/*==============================================================================
 *  Wiring the save task to the dirty flag
 *
 *  config_mark_dirty() has to reach the save task to wake it, and the save
 *  task is created by app_tasks.c.  Rather than have config.c reach into
 *  app_tasks.c's private handle, the task REGISTERS itself here at
 *  startup and config.c keeps the pointer.
 *
 *  The handle is an osThreadId_t, which is what osThreadFlagsSet() takes.
 *  It is NULL until the task registers, and config_mark_dirty() copes with
 *  that: the dirty flag alone is the durable record, so an edit made
 *  before the task exists is still written when the task first runs.
 *
 *  The type is spelled as void* here so this header stays usable in the
 *  host test build and in the PC simulator, neither of which has the RTOS
 *  headers.  On the firmware it is exactly an osThreadId_t.
 *===========================================================================*/

/** @brief The thread flag the save task waits on. */
#define CONFIG_SAVE_FLAG   0x01U

/** @brief Publish the save task's thread id so config_mark_dirty() can
 *   wake it.  Called once by app_config_task() before it waits. */
void config_set_save_task(void *task_id);

/** @brief Wake the save task, if there is one.  Called by
 *   config_mark_dirty().  Does nothing when no task has registered yet -
 *   the dirty flag still carries the edit. */
void config_notify_save_task(void);

/** @brief Write the pending config to flash, if there is anything pending.
 *
 *  The write step of the save task, factored out so it can be exercised
 *  without an RTOS.  Clears the dirty flag first, then writes under the
 *  config lock, so an edit that lands mid-write is kept for the next
 *  pass rather than swallowed.
 *
 *  @return true if a write happened, false if there was nothing to write. */
bool config_flush_pending(void);

/*============================================================================
 *  Concurrency rules for g_config
 *============================================================================
 *  WHY THE LOCK IS CALLER-SIDE
 *  The mutex is deliberately NOT taken inside any config_*() function.
 *  Two reasons, and the second is the one that bites:
 *
 *    1. A function that locks itself cannot be called by a caller that
 *       already holds the lock.  config_save() is reached both from
 *       config_flush_pending() (which locks around it) and from the CAT
 *       link and the power-off flush (which lock around it too).  If the
 *       lock were inside config_save(), every one of those call sites
 *       would have to stop locking to avoid a double acquire - and
 *       FreeRTOS ordinary mutexes are NOT recursive, so a double acquire
 *       is a self-deadlock, not a no-op.
 *
 *    2. The write that actually needs protection is not only the flash
 *       write.  config_save() snapshots g_config and CRCs it, and the
 *       EDITORS have to be inside the same lock for that snapshot to be
 *       self-consistent.  An editor is a caller, not config_save(), so
 *       the lock has to be visible at both ends - which means both ends
 *       are callers and neither can be the function itself.
 *
 *  THE RULE
 *    - Single-field writes (`g_config.x = y`) are atomic on Cortex-M7
 *      for word-size values; no lock needed.
 *    - Single-field reads (comm task reading g_config.radio_baud_rate)
 *      need no lock.
 *    - MULTI-field writes (prot_to_config() writing ten thresholds,
 *      cal_finish() writing a whole touch transform, config_reset_default()
 *      writing every default) MUST be wrapped in config_lock/unlock, so
 *      they cannot interleave with a config_save() snapshot.
 *    - config_save() / config_init() / config_validate() / etc. each
 *      touch many fields.  When they are called from a multi-task
 *      context, the caller MUST wrap them in config_lock/unlock.
 *  The single design rule: any code that needs the whole struct to be
 *  consistent across two operations takes config_lock() and unlocks.
 *  ISR: no config_*() call from an ISR.
 *============================================================================*/
void config_lock(void);
void config_unlock(void);

/** @brief Create the config mutex.  Call once, before the tasks that use
 *   it are created - the mutex used to be created lazily inside
 *   config_lock(), which was a race between two first-time callers. */
void config_mutex_init(void);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H */