/**
 * @file config.c
 * @brief Configuration storage on the internal flash.
 */

#include "config.h"
#include "flash_eeprom.h"

#include <string.h>
#include <strings.h>

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
     * CRC of an otherwise-identical blob would differ between runs. */
    memset(&g_config, 0, sizeof(g_config));

    g_config.pc_baud_rate    = DEFAULT_PC_BAUD_RATE;
    g_config.radio_type      = RADIO_TYPE_UNKNOWN;
    g_config.radio_baud_rate = 19200U;

    /* Protection defaults - these mirror the DEFAULT_PROTECT_* / MAX_* in
     * opa2000.h, which protection.c compares against directly. */
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