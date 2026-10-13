/**
 * @file opa2000.c
 * @brief OPA2000 Global Variables and Utility Functions
 */

#include "opa2000.h"
#include "config.h"

/*============================================================================
 * Global Variables
 *============================================================================*/

/* ADC Data - placed in non-cached SRAM for DMA access */
/* The ADC DMA buffer will be placed in a specific memory region */
volatile AdcDataType g_adc_data = {0};

/* Configuration - loaded from EEPROM or initialized */
ConfigType g_config = {
    .pc_baud_rate = DEFAULT_PC_BAUD_RATE,
    .radio_type = RADIO_TYPE_UNKNOWN,
    .radio_baud_rate = 19200,
    .protect_reflect_power2_w = DEFAULT_PROTECT_REFLECT_POWER2_W,
    .protect_reflect_power2_swr = DEFAULT_PROTECT_REFLECT_POWER2_SWR,
    .protect_swr_threshold = DEFAULT_PROTECT_SWR_THRESHOLD,
    .protect_reflect_power1_w = DEFAULT_PROTECT_REFLECT_POWER1_W,
    .protect_main_current_a = DEFAULT_PROTECT_MAIN_CURRENT_A,
    .cal_tx_power1 = {1.0f, 0.0f, 1.0f, 0.0f},
    .cal_tx_power2 = {1.0f, 0.0f, 1.0f, 0.0f},
    .cal_ref_power1 = {1.0f, 0.0f, 1.0f, 0.0f},
    .cal_ref_power2 = {1.0f, 0.0f, 1.0f, 0.0f},
    .touch_cal = {1.0f, 0.0f, 1.0f, 0.0f, false},
    .language = MULTILANG_ENGLISH,
    .crc32 = 0
};

/* Power state */
volatile PowerStateType g_power_state = POWER_STATE_OFF;

/* Current band detected from radio/freq detector */
volatile BandType g_current_band = BAND_NONE;

/* PTT state */
volatile bool g_ptt_active = false;

/* Alert state */
volatile AlertType g_alert_active = ALERT_NONE;
volatile bool g_alert_flag = false;

/* Touch screen type */
TouchType g_touch_type = TOUCH_NONE;

/* Fan speed control */
volatile FanSpeedType g_fan_speed = FAN_SPEED_LOW;

/* Measured frequency from frequency detector */
volatile uint32_t g_measured_frequency = 0;

/*============================================================================
 * Utility Functions
 *============================================================================*/

/**
 * @brief Calculate CRC32 of data
 * @param data Pointer to data
 * @param length Length of data in bytes
 * @return CRC32 value
 */
uint32_t crc32_calculate(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFF;

    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint32_t j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }

    return ~crc;
}

/**
 * @brief Linear interpolation
 * @param x Input value
 * @param x1 First reference point x
 * @param y1 First reference point y
 * @param x2 Second reference point x
 * @param y2 Second reference point y
 * @return Interpolated y value
 */
float linear_interpolate(float x, float x1, float y1, float x2, float y2)
{
    if (x2 == x1) return y1;
    return y1 + (y2 - y1) * (x - x1) / (x2 - x1);
}

/* Band frequency range strings for GUI display */
const char* strFreqRanges[] = {
    "1.800-2.100 MHz",   /* 160m */
    "3.500-3.900 MHz",   /* 80m */
    "7.000-10.150 MHz",  /* 40-30m */
    "14.000-18.168 MHz", /* 20-17m */
    "21.000-29.700 MHz"  /* 15-10m */
};

/* Band name strings */
const char* strBands[] = {
    "160m",
    "80m",
    "40-30m",
    "20-17m",
    "15-10m"
};

/**
 * @brief Convert frequency to band
 * @param frequency_hz Frequency in Hz
 * @return Band type
 */
BandType frequency_to_band(uint32_t frequency_hz)
{
    if (frequency_hz < 1800000) {
        return BAND_FAULT;
    } else if (frequency_hz < 2100000) {
        return BAND_160M;
    } else if (frequency_hz < 3900000) {
        return BAND_80M;
    } else if (frequency_hz < 10150000) {
        return BAND_40M_30M;
    } else if (frequency_hz < 18168000) {
        return BAND_20M_17M;
    } else if (frequency_hz < 29700000) {
        return BAND_15M_10M;
    } else {
        return BAND_FAULT;
    }
}

/**
 * @brief Get band selection mask
 * @param band Band type
 * @return Band selection mask for GPIO outputs
 */
uint16_t band_to_selection_mask(BandType band)
{
    switch (band) {
        case BAND_160M:     return BAND_SEL_160M;
        case BAND_80M:      return BAND_SEL_80M;
        case BAND_40M_30M:  return BAND_SEL_40M_30M;
        case BAND_20M_17M:  return BAND_SEL_20M_17M;
        case BAND_15M_10M:  return BAND_SEL_15M_10M;
        default:            return BAND_SEL_15M_10M;  /* Default to 15-10M */
    }
}

/**
 * @brief Apply calibration to raw ADC value
 * @param raw_adc Raw ADC value
 * @param cal Calibration data
 * @return Calibrated value
 */
float apply_calibration(uint16_t raw_adc, const CalDataType *cal, bool is_reflect)
{
    float adc_float = (float)raw_adc;
    if (is_reflect) {
        return adc_float * cal->reflect_slope + cal->reflect_offset;
    } else {
        return adc_float * cal->forward_slope + cal->forward_offset;
    }
}

/**
 * @brief Calculate SWR from forward and reflected power
 * @param forward_power Forward power in watts
 * @param reflect_power Reflected power in watts
 * @return SWR value
 */
float calculate_swr(float forward_power, float reflect_power)
{
    if (forward_power < 1.0f) {
        return 1.0f;
    }

    float gamma = reflect_power / forward_power;
    if (gamma > 0.99f) gamma = 0.99f;

    return (1.0f + gamma) / (1.0f - gamma);
}