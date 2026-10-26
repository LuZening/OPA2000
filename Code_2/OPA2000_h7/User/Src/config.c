/**
 * @file config.c
 * @brief Configuration storage on the internal flash.
 */

#include "config.h"
#include "flash_eeprom.h"

#ifndef LVGL_SIM
#include "cmsis_os.h"
#endif

#include <string.h>
/* offsetof, for the crc32-is-last guard below.  Not pulled in by stdint.h. */
#include <stddef.h>
/* strcasecmp is POSIX; MSVC exposes _stricmp.  Map once here so the
 * portable spelling works on every host toolchain we use for unit
 * tests.  (The CMSIS-RTOS2 path in the firmware does not need this
 * because it runs on the ARM toolchain which has strcasecmp natively.) */
#if defined(_MSC_VER)
#define strcasecmp _stricmp
#endif

/*============================================================================
 *  Layout guards
 *
 *  These fail the BUILD, not the field.  Growing ConfigType is a normal
 *  thing to do and every path below degrades quietly if it is not paired
 *  with the rest of the storage layer:
 *
 *  - Past EEPROM_MAX_PAYLOAD_BYTES the record no longer fits the staging
 *    buffer, so config_save() returns false.  The config task ignores that
 *    return value (app_tasks.c) and retries on the next wake-up, and the
 *    operator sees their edits vanish on the next reboot with no message.
 *    A silent failure that only shows up as "the settings reset themselves"
 *    is far more expensive to diagnose than a build break.
 *
 *  - The CRC in config_init() rejects a record whose bytes do not match, but
 *    it cannot tell "same layout, different values" from "same values,
 *    different layout".  Bumping CONFIG_VERSION is what invalidates an old
 *    blob after a field is inserted or reordered; nothing forces you to.
 *    Renumber the versions rather than editing an old one, so a blob written
 *    by any earlier build is still recognised as old.
 *==========================================================================*/

/* MSVC only learned _Static_assert in C11 mode; the host test build asks for
 * C11 explicitly, but spell out the fallback so this file still compiles if
 * that ever changes.  C11 6.7.10 / N1570 7.18. */
#if defined(_MSC_VER) && (!defined(__STDC_VERSION__) \
                         || __STDC_VERSION__ < 201112L)
#define OPA2000_STATIC_ASSERT(cond, msg)                              \
    typedef char opa2000_static_assert_##__LINE__[(cond) ? 1 : -1]
#else
#define OPA2000_STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)
#endif

OPA2000_STATIC_ASSERT(sizeof(ConfigType) <= EEPROM_MAX_PAYLOAD_BYTES,
               "ConfigType exceeds the EEPROM payload cap - config_save() "
               "would fail silently and every operator edit would be lost on "
               "reboot. Raise EEPROM_MAX_PAYLOAD_BYTES if the record really "
               "has to be this big.");

OPA2000_STATIC_ASSERT(offsetof(ConfigType, crc32) + sizeof(uint32_t)
                   == sizeof(ConfigType),
               "crc32 must stay the LAST member of ConfigType - the CRC is "
               "computed over everything ahead of it, so a field appended "
               "after it would silently escape the checksum.");

OPA2000_STATIC_ASSERT(CONFIG_VERSION != 0U,
               "CONFIG_VERSION 0 is reserved for 'never written'.");

/*============================================================================
 * Private Variables
 *============================================================================*/

static const char * const radio_type_strings[] = {
    "Unknown",
    "ICOM CIV",
    "Yaesu",
    "Kenwood"
};

/* Raised by config_mark_dirty(), consumed by app_config_task().
 *
 * volatile because it is written from the GUI task and read from the config
 * task.  A lost update here only means the change is persisted a second
 * later, so no critical section is warranted - and taking one from an LVGL
 * callback would be the wrong trade. */
static volatile bool s_dirty;

/* Mutex that protects the whole "snapshot the live struct + stamp
 * magic/version/CRC + push to flash" path.  See config_lock/unlock
 * contract in config.h: writers take it for the entire save/load/
 * validate/reset; single-field writes and reads stay unlocked.
 *
 * On the PC simulator (LVGL_SIM) the shim falls back to a no-op so this
 * mutex exists but is never taken, matching the existing gui_lock
 * behaviour for LVGL code paths. */
static osMutexId_t s_config_mutex;

/*============================================================================
 * Public Functions (lock / unlock)
 *============================================================================*/

void config_mutex_init(void)
{
#ifndef LVGL_SIM
    /* Created once, before any task runs, so two tasks calling
     * config_lock() at the same moment cannot each create a mutex and
     * end up with two different locks.  (Creating it lazily inside
     * config_lock() was the old behaviour, and it was a race waiting for
     * the first time the GUI and the save task wrote at once.)
     *
     * This has to run after osKernelInitialize() and before the tasks
     * are created - app_tasks_create() is called from MX_FREERTOS_Init(),
     * which is exactly that window. */
    if (s_config_mutex == NULL) {
        s_config_mutex = osMutexNew(NULL);
    }
#endif
}

void config_lock(void)
{
#ifndef LVGL_SIM
    if (s_config_mutex != NULL) {
        osMutexAcquire(s_config_mutex, osWaitForever);
    }
#else
    /* PC simulator: no FreeRTOS, no contention.  The shim must still
     * provide a "do nothing" path so the symbol is defined and the
     * source compiles. */
    (void)s_config_mutex;
#endif
}

void config_unlock(void)
{
#ifndef LVGL_SIM
    if (s_config_mutex != NULL) {
        osMutexRelease(s_config_mutex);
    }
#else
    (void)s_config_mutex;
#endif
}

/*============================================================================
 * Private Functions
 *============================================================================*/

/**
 * @brief Recompute the CRC over everything ahead of the crc32 field
 *
 * crc32 is the last member of ConfigType, so "everything ahead of it" is
 * simply the first sizeof(ConfigType) - 4 bytes.  See the comment on the
 * field itself - moving it would silently invalidate every stored blob.
 */
static void config_update_crc(void)
{
    g_config.crc32 = crc32_calculate((const uint8_t *)&g_config,
                                     sizeof(ConfigType) - sizeof(uint32_t));
}

/*============================================================================
 * Public Functions
 *============================================================================*/

/**
 * @brief Whether a previously saved config was found on flash
 *
 * Still meaningful even though nothing is mounted any more: it tells the
 * caller whether the defaults it is about to fall back on are coming from
 * an empty store or from a store whose contents could not be trusted.
 */
bool config_sdcard_ready(void)
{
    return eeprom_current_id() != 0U;
}

/**
 * @brief Load the newest config record
 *
 * A CRC mismatch on the stored record is caught inside the EEPROM layer,
 * which only ever hands back a record whose payload CRC checks out.
 */
bool config_init(void)
{
    ConfigType stored;

    if (!eeprom_load(&stored, sizeof(stored))) {
        /* Empty store or nothing readable.  Start from the defaults; they
         * get written out by config_reset_default(). */
        config_reset_default();
        return false;
    }

    /* One more gate before the data is allowed near g_config: the record
     * also carries a magic and a layout version, so a store written by an
     * older firmware is rejected rather than reinterpreted. */
    if (stored.magic != CONFIG_MAGIC || stored.version != CONFIG_VERSION) {
        config_reset_default();
        return false;
    }

    g_config = stored;

    /* allow_protection was carved out of what used to be reserved[] padding,
     * which config_reset_default() always zeroed.  So a record written by an
     * older firmware lands 0 there - indistinguishable from an operator who
     * genuinely turned protection off.  Default the ambiguous case to
     * "armed": reading a stale padding byte as "ignore over-temperature" is
     * the one failure mode worth bending over backwards to avoid, and the
     * operator can always set it back to 0 from the GUI or CAT.
     * No CRC bump is needed for this - the layout is byte-for-byte identical. */
    if (g_config.allow_protection == 0U) {
        g_config.allow_protection = 1U;
    }

    /* The zero-offset calibration is a MEASUREMENT of the unit in front
     * of it, so it cannot be inherited from a previous session: the
     * sensor drifts with temperature and age, and the stored offset
     * describes a different moment on a different day.  A record saved
     * while the calibration was valid would otherwise arrive here with
     * im_calibrated = 1, and that flag is what arms the main-current
     * protection - so a stale "yes" would put a live threshold at a
     * stale current.
     *
     * The detected sensor model goes with it: it is detected in the
     * same window, and a stored model would describe a board that may
     * not be the one fitted now.
     *
     * The offset VALUE is kept (it is a useful starting point and the
     * GUI shows it), but the flag is cleared: the calibration runs again
     * on every boot, and only a fresh measurement may arm the current
     * protection.  adc_init() clears this too, but that runs later -
     * in the ADC task - and this window would otherwise be open. */
    g_config.im_calibrated  = 0U;
    g_config.im_sensor_type = IM_SENSOR_UNKNOWN;

    s_dirty = false;
    return true;
}

/**
 * @brief Append the current g_config to the flash ring
 *
 * CALLER TAKES THE LOCK.  This function does not lock, and it must not:
 * every path that reaches it already holds the config mutex (see
 * config_flush_pending(), comm_action_save(), the power-off flush), and a
 * FreeRTOS ordinary mutex is not recursive, so locking here as well would
 * deadlock on the second acquire.
 *
 * Holding the lock is what makes the snapshot below self-consistent.  The
 * struct is 128 bytes and the CRC pass reads 124 of them; if an editor
 * (prot_to_config() writing ten thresholds, say) interleaved with that
 * read, the copy would be a mix of old and new fields.  The record's own
 * CRC would still match - it is computed over the same copy - so nothing
 * would report an error, but the flash would hold a half-applied set of
 * settings.  Hence: the editors take the lock too.
 *
 * Callers must not use this from a UI callback - see config_mark_dirty().
 */
bool config_save(void)
{
    ConfigType out = g_config;

    /* im_calibrated is a per-boot measurement, not a setting: it says
     * "the zero offset of THIS unit was measured in THIS session", and
     * it is what arms the main-current protection.  Persisting it would
     * let a record written while the calibration happened to be valid
     * arrive on a later boot with the flag still set, putting a live
     * threshold at a stale current.  config_init() clears it on the way
     * back in, but it should never reach the flash in the first place.
     *
     * The detected sensor model is the same kind of thing: it is
     * detected in the same window and describes the board that happens
     * to be fitted right now. */
    out.im_calibrated  = 0U;
    out.im_sensor_type = IM_SENSOR_UNKNOWN;

    /* Stamp magic / version / CRC into a copy so g_config itself keeps the
     * caller's values (a save must not mutate live settings). */
    out.magic   = CONFIG_MAGIC;
    out.version = (uint8_t)CONFIG_VERSION;
    out.crc32   = 0U;
    out.crc32   = crc32_calculate((const uint8_t *)&out,
                                  sizeof(ConfigType) - sizeof(uint32_t));

    return eeprom_save(&out, sizeof(out));
}

/**
 * @brief Write the pending config to flash, if there is anything pending
 *
 * This is the body of the save task's write step, factored out so the
 * host tests can drive it without an RTOS.  It is deliberately the only
 * place that turns "something changed" into "the flash was written", so
 * the clear-then-write ordering is stated once and cannot drift between
 * the task and the tests.
 *
 * Clear first, then write.  If an edit lands between the clear and the
 * write, config_save() stores the newer values and the dirty flag is
 * still set for the next pass - the edit is not lost.  Clearing
 * afterwards instead would swallow it.
 *
 * The lock keeps this write from interleaving with another writer (the
 * CAT link's explicit "save" command and the power-off flush both write
 * directly).  Without it two eeprom_save() calls could pick the same
 * target slot and one of the records would be silently lost.
 *
 * A FAILED write re-raises the dirty flag and asks the save task to come
 * back.  That matters more here than anywhere else in the module: with a
 * thread-flag wake-up there is no timer left to retry on, so clearing the
 * flag before the write and then dropping a false return would turn a
 * transient flash error into a permanently lost setting - the operator
 * would see their edit vanish on the next reboot with nothing logged.
 * The poll this replaced retried every second for exactly this reason.
 *
 * @return true if a write happened, false if there was nothing to write.
 */
bool config_flush_pending(void)
{
    if (!config_is_dirty()) {
        return false;
    }

    config_lock();
    bool ok = config_save();
    config_unlock();
    config_clear_dirty();

    if (!ok) {
        /* Put the edit back so it is written again.  config_mark_dirty()
         * both re-sets the flag and re-posts the thread flag, so the save
         * task comes straight back for another attempt. */
        config_mark_dirty();
        return false;
    }

    return true;
}

/**
 * @brief Reset configuration to defaults and persist them immediately
 *
 * Unlike an ordinary edit this one is written straight through: it runs at
 * boot, before the save task exists, and the operator is waiting on it.
 *
 * CALLER TAKES THE LOCK whenever it runs from a multi-task context.  This
 * function does NOT call config_lock() itself, and that is load-bearing:
 * comm_action_reset_cfg() holds the same (non-recursive) FreeRTOS mutex
 * around its call, so locking in here would deadlock on the second
 * acquire.  The boot path (config_init(), before the scheduler starts) has
 * no other task to race with and passes no lock.
 *
 * The rule is the module's: the PUBLIC config_*() functions never lock,
 * and whoever needs a multi-field-consistent view locks around them.  This
 * function writes every field in g_config, so a caller in a task MUST lock
 * - otherwise this memset-plus-refill could interleave with the save task's
 * config_save() snapshot and leave the flash holding a config that is half
 * factory defaults and half the operator's settings.
 */
void config_reset_default(void)
{
    /* Start from a clean slate: the reserved[] padding must be zero or the
     * CRC of an otherwise-identical blob would differ between runs.
     * Cast through (void *) to drop the volatile qualifier: g_config is
     * volatile for the benefit of the multi-task readers, but this
     * function is the single writer and is called from task context, so
     * memset's lack of volatile reload is not a correctness issue here. */
    memset((void *)&g_config, 0, sizeof(g_config));

    g_config.pc_baud_rate    = DEFAULT_PC_BAUD_RATE;
    g_config.radio_type      = RADIO_TYPE_UNKNOWN;
    g_config.radio_baud_rate = DEFAULT_RADIO_BAUD_RATE;

    /* Protection defaults - these mirror the DEFAULT_PROTECT_* / MAX_* in
     * opa2000.h; protection.c and boot.c read them back out of g_config and
     * never compare against a constant of their own. */
    g_config.band_rev1_max_w      = (uint16_t)DEFAULT_PROTECT_REFLECT_POWER1_W;
    g_config.band_rev2_max_w      = (uint16_t)DEFAULT_PROTECT_REFLECT_POWER2_W;
    g_config.band_rev2_swr_w      = (uint16_t)DEFAULT_PROTECT_REFLECT_POWER2_SWR;
    g_config.band_swr_max_x100    = (uint16_t)DEFAULT_PROTECT_SWR_THRESHOLD_X100;
    g_config.band_ntc_max_tenth_c = (uint16_t)(MAX_NTC_TEMP_C * 10);
    g_config.band_fwd1_max_w      = (uint16_t)MAX_TX_POWER1_W;
    g_config.band_fwd2_max_w      = (uint16_t)MAX_TX_POWER2_W;
    g_config.band_im_max_mA       = (uint16_t)(MAX_MAIN_CURRENT_A * 1000U);
    g_config.band_vbias_max_mV    = 5000U;
    for (int i = 0; i < 6; i++) {
        g_config.band_rf_in_max_w[i] = 20U;
    }

    /* COMP comparator references, as the DAC reference VOLTAGE in mV.
     * DAC1 is 12-bit on VDDA 3.3 V, so 3300 mV is the ceiling.
     *   COMP2 (PE9, main current IM via CC6920BSO-50A) has a documented
     *   transfer function V = 1650 + I*26.4, so the 30 A design limit is
     *   exactly 2442 mV.  That is also the practical maximum: 2500 mV is
     *   the ADC full scale on the very same node.
     *   COMP1 (PB2, RF1 peak detector through D5/R40) has NOT been
     *   calibrated - the D5/R40 gain is unknown, so no wattage can be
     *   derived from it.  Until it is, sit well below full scale and far
     *   above the detector noise floor: COMP1 acts as a last-resort
     *   overshoot catcher, while AWD1 on the same node does the precise
     *   band-dependent limit in firmware.  See
     *   User/TODO-COMP1标定待办.txt. */
    g_config.comp1_ref_mV         = DEFAULT_COMP1_REF_MV;
    g_config.comp2_ref_mV         = DEFAULT_COMP2_REF_MV;

    /* IM zero-offset: 1.65 V of a 2.5 V reference on a 16-bit converter.
     * The sensor model is detected at power-on from the sensor's own
     * resting output, so it starts out unknown. */
    g_config.im_zero_offset_adc = 43254U;   /* 1.65 / 2.5 * 65535 */
    g_config.im_calibrated      = 0U;
    g_config.im_sensor_type     = IM_SENSOR_UNKNOWN;

    /* Touch: identity transform, flagged uncalibrated so touch.c passes raw
     * through until the operator runs the calibration screen. */
    g_config.touch.x_gain      = 1.0f;
    g_config.touch.x_offset    = 0.0f;
    g_config.touch.y_gain      = 1.0f;
    g_config.touch.y_offset    = 0.0f;
    g_config.touch.calibrated  = 0U;

    /* Coupler trims zero == "no calibration, use the OPA1000 model as is". */
    g_config.fwd1 = (coupler_t){0, 0};
    g_config.fwd2 = (coupler_t){0, 0};
    g_config.rev1 = (coupler_t){0, 0};
    g_config.rev2 = (coupler_t){0, 0};

    g_config.language        = MULTILANG_CHINESE;
    g_config.buzzer_enable   = 1U;
    g_config.temp_fahrenheit = 0U;
    /* Protection armed out of the box.  Defaulting this to 0 would ship a
     * unit that silently ignores over-temperature and over-current. */
    g_config.allow_protection = 1U;

    config_update_crc();

    /* Written straight through, and the flag cleared only if it worked, so
     * a failed write is retried by the save task on its next tick. */
    if (config_save()) {
        s_dirty = false;
    }
}

/**
 * @brief Validate configuration CRC
 */
bool config_validate(void)
{
    uint32_t stored_crc = g_config.crc32;

    g_config.crc32 = 0U;
    uint32_t calculated_crc = crc32_calculate((const uint8_t *)&g_config,
                                             sizeof(ConfigType) - sizeof(uint32_t));
    g_config.crc32 = stored_crc;

    return (stored_crc == calculated_crc);
}

/*============================================================================
 *  Deferred saving
 *==========================================================================*/

void config_mark_dirty(void)
{
    /* The dirty flag is the durable record; the thread flag is only the
     * wake-up.  Setting the flag FIRST means an edit is never lost, even
     * if the task has not registered yet or the notification is dropped:
     * the save task re-reads config_is_dirty() on every wake, and
     * config_set_save_task() re-arms the wake-up for an edit that landed
     * before the task existed. */
    s_dirty = true;
    config_notify_save_task();
}

/*============================================================================
 *  Save-task wiring
 *
 *  The save task blocks on a thread flag instead of polling, so somebody
 *  has to be able to wake it.  config.c cannot reach into app_tasks.c's
 *  private task handle, so the task publishes its own id here once and
 *  config_mark_dirty() uses it.
 *==========================================================================*/

#if !defined(LVGL_SIM)
/* The save task's thread id, or NULL until it registers.  Typed as void*
   so this file does not depend on the RTOS headers when it is built for
   the host test or for the PC simulator. */
static void *s_save_task_id = NULL;
#endif

void config_set_save_task(void *task_id)
{
#if !defined(LVGL_SIM)
    s_save_task_id = task_id;
    /* An edit that landed before this task existed is still pending: the
       dirty flag carries it, but nobody was there to be woken.  Post the
       flag now so the edit is written instead of waiting for the next
       one to arrive. */
    if (config_is_dirty()) {
        config_notify_save_task();
    }
#else
    (void)task_id;
#endif
}

void config_notify_save_task(void)
{
#if !defined(LVGL_SIM)
    if (s_save_task_id == NULL) {
        /* No task yet (the edit happened during boot).  The dirty flag
         * carries it, and config_set_save_task() posts the flag once the
         * task does exist. */
        return;
    }
    /* A stale flag is harmless: the task re-checks config_is_dirty() after
     * every wake, so a wake with nothing to write is a no-op. */
    (void)osThreadFlagsSet((osThreadId_t)s_save_task_id, CONFIG_SAVE_FLAG);
#endif
}

void config_clear_dirty(void)
{
    s_dirty = false;
}

bool config_is_dirty(void)
{
    return s_dirty;
}

/**
 * @brief Get radio type string
 */
const char *config_get_radio_type_string(RadioType type)
{
    if (type <= RADIO_TYPE_KENWOOD) {
        return radio_type_strings[type];
    }
    return radio_type_strings[RADIO_TYPE_UNKNOWN];
}

/**
 * @brief Set radio type from string
 */
RadioType config_set_radio_type_from_string(const char *str)
{
    if (str == NULL) return RADIO_TYPE_UNKNOWN;

    /* Try exact match first (case sensitive) */
    for (int i = 0; i <= RADIO_TYPE_KENWOOD; i++) {
        if (strcmp(str, radio_type_strings[i]) == 0) {
            return (RadioType)i;
        }
    }

    /* Try case insensitive */
    for (int i = 0; i <= RADIO_TYPE_KENWOOD; i++) {
        if (strcasecmp(str, radio_type_strings[i]) == 0) {
            return (RadioType)i;
        }
    }

    /* Try partial matches */
    if (strcasecmp(str, "icom") == 0 || strcasecmp(str, "civ") == 0) {
        return RADIO_TYPE_ICOM_CIV;
    }
    if (strcasecmp(str, "yaesu") == 0) {
        return RADIO_TYPE_YAESU;
    }
    if (strcasecmp(str, "kenwood") == 0) {
        return RADIO_TYPE_KENWOOD;
    }

    return RADIO_TYPE_UNKNOWN;
}

bool config_touch_needs_calibration(void)
{
    /* The reset defaults leave `touch.calibrated = 0` and the gain/offset
     * pair at the identity.  If a previously saved config was loaded but
     * the operator never finished calibration (or loaded a fresh board),
     * we still want to prompt them.  We do NOT take the lock here - the
     * read is a snapshot, atomic on a word-size field, and any "you
     * just calibrated it" race is harmless (the user just goes through
     * the screen twice). */
    if (g_config.touch.calibrated != 0U) {
        return false;
    }
    /* Identity transform check: gain == 1, offset == 0.  Use an epsilon
     * for the float gain so a future "factory-calibrated to 0.99" still
     * gets accepted. */
    if (g_config.touch.x_gain < 0.99f || g_config.touch.x_gain > 1.01f) {
        return false;
    }
    if (g_config.touch.y_gain < 0.99f || g_config.touch.y_gain > 1.01f) {
        return false;
    }
    return true;
}
