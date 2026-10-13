/**
 * @file language.h
 * @brief Multi-language String Definitions
 */

#ifndef LANGUAGE_H
#define LANGUAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "opa2000.h"

/*============================================================================
 * String Count
 *============================================================================*/

#define LANG_STRING_COUNT    (STR_CONNECT_ANTENNA + 1)

/*============================================================================
 * String Indices
 *============================================================================*/

typedef enum {
    STR_ID = 0,

    /* ADC Channel Names */
    STR_ADC_TX_POWER1,         /* 发射功率1 / Transmit Power 1 */
    STR_ADC_REF_POWER1,        /* 反射功率1 / Reflected Power 1 */
    STR_ADC_TX_POWER2,         /* 发射功率2 / Transmit Power 2 */
    STR_ADC_REF_POWER2,        /* 反射功率2 / Reflected Power 2 */
    STR_ADC_RF_INPUT,          /* RF输入功率 / RF Input Power */
    STR_ADC_MAIN_CURRENT,      /* 主电流 / Main Current */
    STR_ADC_MAIN_VOLTAGE,      /* 主电压 / Main Voltage */
    STR_ADC_NTC_TEMP,          /* 核心NTC温度 / Core NTC Temperature */
    STR_ADC_DETECTOR,          /* 输出检波 / Output Detector */

    /* Power states */
    STR_IDLE,
    STR_STANDBY,
    STR_TRANSMIT,
    STR_READY,

    /* Measurements */
    STR_TX_POWER,
    STR_REF_POWER,
    STR_SWR,
    STR_VOLTAGE,
    STR_CURRENT,
    STR_TEMPERATURE,
    STR_BAND,

    /* Alerts */
    STR_ALERT,
    STR_SWR_HIGH,
    STR_REFLECT_HIGH,
    STR_CURRENT_HIGH,
    STR_HARDWARE_FAULT,
    STR_OVERDRIVE,
    STR_FREQ_MISMATCH,
    STR_ALARM,

    /* Buttons/Actions */
    STR_DISMISS,
    STR_OK,
    STR_CANCEL,
    STR_SAVE,
    STR_CALIBRATE,

    /* Calibration */
    STR_TOUCH_CALIBRATION,
    STR_CAL_POINT,
    STR_PRESS_POINT,
    STR_CAL_SUCCESS,
    STR_CAL_FAILED,
    STR_RETRY,

    /* Menu items */
    STR_SETTINGS,
    STR_LANGUAGE,
    STR_RADIO_TYPE,
    STR_RADIO_BAUD,

    /* Power */
    STR_POWER_ON,
    STR_POWER_OFF,
    STR_POWER_BUTTON,

    /* Directional coupler */
    STR_COUPLE_CAL,
    STR_CONNECT_LOAD,
    STR_CONNECT_ANTENNA
} StringId;

/*============================================================================
 * Function Prototypes
 *============================================================================*/

/**
 * @brief Get string by ID in current language
 * @param id String ID
 * @return String pointer
 */
const char* lang_get_string(StringId id);

/**
 * @brief Get English string by ID
 * @param id String ID
 * @return String pointer (English)
 */
const char* lang_get_string_en(StringId id);

/**
 * @brief Get Chinese string by ID
 * @param id String ID
 * @return String pointer (Chinese)
 */
const char* lang_get_string_zh(StringId id);

/**
 * @brief Set current language
 * @param lang Language type
 */
void lang_set_language(LanguageType lang);

/**
 * @brief Get current language
 * @return Current language
 */
LanguageType lang_get_language(void);

#ifdef __cplusplus
}
#endif

#endif /* LANGUAGE_H */