/**
 * @file config.c
 * @brief Configuration Management Implementation using FatFS
 */

#include "config.h"
#include "ff.h"
#include "sdcard_storage.h"
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

static FATFS fs;
static bool fs_mounted = false;

/*============================================================================
 * Private Functions
 *============================================================================*/

/**
 * @brief Mount FatFS if not already mounted
 */
static bool config_mount_fs(void)
{
    if (fs_mounted) {
        return true;
    }

    if (!sdcard_storage_take_mutex(5000)) {
        return false;
    }

    FRESULT res = f_mount(&fs, "", 1);
    if (res != FR_OK) {
        sdcard_storage_give_mutex();
        return false;
    }

    fs_mounted = true;
    sdcard_storage_give_mutex();
    return true;
}

/**
 * @brief Write config to file
 */
static bool config_write_file(const char *filename, const void *data, uint32_t len)
{
    FIL fp;
    UINT bw;
    FRESULT res;

    if (!sdcard_storage_take_mutex(5000)) {
        return false;
    }

    res = f_open(&fp, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        sdcard_storage_give_mutex();
        return false;
    }

    res = f_write(&fp, data, len, &bw);
    f_close(&fp);

    sdcard_storage_give_mutex();
    return (res == FR_OK && bw == len);
}

/**
 * @brief Read config from file
 */
static bool config_read_file(const char *filename, void *data, uint32_t len)
{
    FIL fp;
    UINT br;
    FRESULT res;

    if (!sdcard_storage_take_mutex(5000)) {
        return false;
    }

    res = f_open(&fp, filename, FA_READ);
    if (res != FR_OK) {
        sdcard_storage_give_mutex();
        return false;
    }

    res = f_read(&fp, data, len, &br);
    f_close(&fp);

    sdcard_storage_give_mutex();
    return (res == FR_OK && br == len);
}

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
 * @brief Check if SD card is ready
 */
bool config_sdcard_ready(void)
{
    return fs_mounted;
}

/**
 * @brief Initialize configuration from FatFS
 */
bool config_init(void)
{
    ConfigType stored;

    /* Mount FatFS */
    if (!config_mount_fs()) {
        config_reset_default();
        return false;
    }

    /* Read the blob as-is and validate magic / version / CRC before it is
     * allowed anywhere near g_config - a partially written file would
     * otherwise be interpreted as real settings. */
    if (!config_read_file(CONFIG_FILENAME, &stored, sizeof(ConfigType))) {
        /* Config file not accessible or not programmed */
        config_reset_default();
        return false;
    }

    if (stored.magic != CONFIG_MAGIC || stored.version != CONFIG_VERSION) {
        /* Invalid or old layout, use defaults */
        config_reset_default();
        return false;
    }

    {
        uint32_t want = stored.crc32;
        stored.crc32 = 0U;
        if (crc32_calculate((const uint8_t *)&stored,
                            sizeof(ConfigType) - sizeof(uint32_t)) != want) {
            config_reset_default();
            return false;
        }
    }

    g_config = stored;
    return true;
}

/**
 * @brief Save configuration to FatFS file
 */
bool config_save(void)
{
    if (!config_mount_fs()) {
        return false;
    }

    /* Stamp magic / version / CRC into a copy so g_config itself keeps the
     * caller's values (a save must not mutate live settings). */
    ConfigType out = g_config;
    out.magic   = CONFIG_MAGIC;
    out.version = (uint8_t)CONFIG_VERSION;
    out.crc32   = 0U;
    out.crc32   = crc32_calculate((const uint8_t *)&out,
                                  sizeof(ConfigType) - sizeof(uint32_t));

    if (!config_write_file(CONFIG_FILENAME, &out, sizeof(ConfigType))) {
        return false;
    }

    /* Backup copy: if power is lost mid-write of the main file the loader
     * can still fall back to this one. */
    (void)config_write_file(CONFIG_BACKUP_FILENAME, &out, sizeof(ConfigType));

    return true;
}

/**
 * @brief Reset configuration to defaults
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

    g_config.language      = MULTILANG_CHINESE;
    g_config.buzzer_enable = 1U;
    g_config.temp_fahrenheit = 0U;

    config_update_crc();

    /* Persist so the defaults survive a reboot even with no SD write from
     * the GUI. Failure is not fatal - config_init() will just re-default. */
    (void)config_save();
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