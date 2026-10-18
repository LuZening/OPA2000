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
 *    return value (app_tasks.c) and retries once a second forever, and the
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

void config_lock(void)
{
#ifndef LVGL_SIM
    if (s_config_mutex == NULL) {
        s_config_mutex = osMutexNew(NULL);
    }
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
    s_dirty = false;
    return true;
}

/**
 * @brief Append the current g_config to the flash ring
 *
 * Callers must not use this from a UI callback - see config_mark_dirty().
 */
bool config_save(void)
{
    ConfigType out = g_config;

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
 * @brief Reset configuration to defaults and persist them immediately
 *
 * Unlike an ordinary edit this one is written straight through: it runs at
 * boot, before the save task exists, and the operator is waiting on it.
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

    /* IM zero-offset: 1.65 V of a 2.5 V reference on a 16-bit converter. */
    g_config.im_zero_offset_adc = 43254U;   /* 1.65 / 2.5 * 65535 */
    g_config.im_calibrated      = 0U;

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
    s_dirty = true;
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