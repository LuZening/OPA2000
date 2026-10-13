/**
 * @file config.c
 * @brief Configuration Management Implementation using FatFS
 */

#include "config.h"
#include "ff.h"
#include "sdcard_storage.h"
#include <string.h>

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
 * @brief Update config CRC
 */
static void config_update_crc(void)
{
    /* Calculate CRC of all config except CRC itself */
    uint32_t crc = crc32_calculate((const uint8_t*)&g_config,
                                   sizeof(ConfigType) - sizeof(uint32_t));
    g_config.crc32 = crc;
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
    uint32_t magic;
    uint32_t version;

    /* Mount FatFS */
    if (!config_mount_fs()) {
        config_reset_default();
        return false;
    }

    /* Try to read magic and version first */
    ConfigType temp_config;
    if (!config_read_file(CONFIG_FILENAME, &temp_config, sizeof(ConfigType))) {
        /* Config file not accessible or not programmed */
        config_reset_default();
        return false;
    }

    magic = temp_config.pc_baud_rate;  /* Reuse field for magic check */
    version = temp_config.radio_type;  /* Reuse field for version check */

    /* Check magic and version - using first fields as magic/version */
    if (magic != CONFIG_MAGIC || version != CONFIG_VERSION) {
        /* Invalid or old config, use defaults */
        config_reset_default();
        return false;
    }

    /* Read full configuration */
    if (!config_read_file(CONFIG_FILENAME, &g_config, sizeof(ConfigType))) {
        config_reset_default();
        return false;
    }

    /* Validate CRC */
    if (!config_validate()) {
        config_reset_default();
        return false;
    }

    return true;
}

/**
 * @brief Save configuration to FatFS file
 */
bool config_save(void)
{
    ConfigType save_config;

    /* Ensure filesystem is mounted */
    if (!config_mount_fs()) {
        return false;
    }

    /* Copy current config and embed magic/version in the copy */
    memcpy(&save_config, &g_config, sizeof(ConfigType));
    save_config.pc_baud_rate = CONFIG_MAGIC;
    save_config.radio_type = (RadioType)CONFIG_VERSION;

    /* Update CRC on the copy */
    uint32_t crc = crc32_calculate((const uint8_t*)&save_config,
                                   sizeof(ConfigType) - sizeof(uint32_t));
    save_config.crc32 = crc;

    /* Write main config file */
    if (!config_write_file(CONFIG_FILENAME, &save_config, sizeof(ConfigType))) {
        return false;
    }

    /* Write backup file */
    config_write_file(CONFIG_BACKUP_FILENAME, &save_config, sizeof(ConfigType));

    return true;
}

/**
 * @brief Reset configuration to defaults
 */
void config_reset_default(void)
{
    /* Set default values */
    g_config.pc_baud_rate = DEFAULT_PC_BAUD_RATE;
    g_config.radio_type = RADIO_TYPE_UNKNOWN;
    g_config.radio_baud_rate = 19200;

    /* Protection defaults */
    g_config.protect_reflect_power2_w = DEFAULT_PROTECT_REFLECT_POWER2_W;
    g_config.protect_reflect_power2_swr = DEFAULT_PROTECT_REFLECT_POWER2_SWR;
    g_config.protect_swr_threshold = DEFAULT_PROTECT_SWR_THRESHOLD;
    g_config.protect_reflect_power1_w = DEFAULT_PROTECT_REFLECT_POWER1_W;
    g_config.protect_main_current_a = DEFAULT_PROTECT_MAIN_CURRENT_A;
    g_config.protect_ntc_temp_c = DEFAULT_PROTECT_NTC_TEMP_C;
    g_config.protect_tx_power1_w = DEFAULT_PROTECT_TX_POWER1_W;
    g_config.protect_tx_power2_w = DEFAULT_PROTECT_TX_POWER2_W;

    /* Default calibration (unity) */
    g_config.cal_tx_power1.forward_slope = 1.0f;
    g_config.cal_tx_power1.forward_offset = 0.0f;
    g_config.cal_tx_power2.forward_slope = 1.0f;
    g_config.cal_tx_power2.forward_offset = 0.0f;
    g_config.cal_ref_power1.reflect_slope = 1.0f;
    g_config.cal_ref_power1.reflect_offset = 0.0f;
    g_config.cal_ref_power2.reflect_slope = 1.0f;
    g_config.cal_ref_power2.reflect_offset = 0.0f;
    g_config.cal_rf_input.forward_slope = 1.0f;
    g_config.cal_rf_input.forward_offset = 0.0f;

    /* Touch calibration invalid */
    g_config.touch_cal.x_slope = 1.0f;
    g_config.touch_cal.x_offset = 0.0f;
    g_config.touch_cal.y_slope = 1.0f;
    g_config.touch_cal.y_offset = 0.0f;
    g_config.touch_cal.valid = false;

    /* Default language */
    g_config.language = MULTILANG_ENGLISH;

    /* Calculate and set CRC */
    config_update_crc();

    /* Save to FatFS */
    config_save();
}

/**
 * @brief Validate configuration CRC
 */
bool config_validate(void)
{
    uint32_t stored_crc = g_config.crc32;
    uint32_t calculated_crc;

    /* Temporarily zero CRC for calculation */
    g_config.crc32 = 0;
    calculated_crc = crc32_calculate((const uint8_t*)&g_config, sizeof(ConfigType));

    /* Restore stored CRC */
    g_config.crc32 = stored_crc;

    return (stored_crc == calculated_crc);
}

/**
 * @brief Get radio type string
 */
const char* config_get_radio_type_string(RadioType type)
{
    if (type >= RADIO_TYPE_UNKNOWN && type <= RADIO_TYPE_KENWOOD) {
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