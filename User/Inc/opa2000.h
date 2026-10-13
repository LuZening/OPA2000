/**
 * @file opa2000.h
 * @brief OPA2000 Global Definitions and Types
 */

#ifndef OPA2000_H
#define OPA2000_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef LVGL_SIM
#include "main.h"
#endif

/*============================================================================
 * Configuration Constants
 *============================================================================*/

/* ADC Channels - ADC1 (5ch) + ADC2 (4ch) = 9 total */
/* ADC1: 发射功率1, 反射功率1, 发射功率2, 反射功率2, RF输入功率 */
/* ADC2: 主电流, 主电压, 核心NTC温度传感器, 输出检波采样 */
#define ADC_CHANNEL_TX_POWER1     0   /* Transmit Power 1 / 发射功率1 */
#define ADC_CHANNEL_REF_POWER1    1   /* Reflected Power 1 / 反射功率1 */
#define ADC_CHANNEL_TX_POWER2     2   /* Transmit Power 2 / 发射功率2 */
#define ADC_CHANNEL_REF_POWER2    3   /* Reflected Power 2 / 反射功率2 */
#define ADC_CHANNEL_RF_INPUT      4   /* RF Input Power / RF输入功率 */
#define ADC_CHANNEL_MAIN_CURRENT  5   /* Main Current / 主电流 */
#define ADC_CHANNEL_MAIN_VOLTAGE  6   /* Main Voltage / 主电压 */
#define ADC_CHANNEL_NTC_TEMP      7   /* NTC Temperature / 核心NTC温度传感器 */
#define ADC_CHANNEL_DETECTOR      8   /* Output Sampling / 输出检波采样 */

#define ADC_CHANNEL_COUNT         9

/* Protection Thresholds (default values) */
#define DEFAULT_PROTECT_REFLECT_POWER2_W     400     /* Watts */
#define DEFAULT_PROTECT_REFLECT_POWER2_SWR   200     /* Watts */
#define DEFAULT_PROTECT_SWR_THRESHOLD        2.0f    /* SWR */
#define DEFAULT_PROTECT_REFLECT_POWER1_W     600     /* Watts */
#define DEFAULT_PROTECT_MAIN_CURRENT_A       40.0f   /* Amperes */
#define DEFAULT_PROTECT_NTC_TEMP_C           90.0f   /* Celsius */
#define DEFAULT_PROTECT_TX_POWER1_W          1100.0f /* Watts */
#define DEFAULT_PROTECT_TX_POWER2_W          1050.0f /* Watts */

/* Fan Control */
#define FAN_SPEED_50_PERCENT     50
#define FAN_SPEED_80_PERCENT     80
#define FAN_SPEED_100_PERCENT    100
#define FAN_TEMP_THRESHOLD_80    60      /* Celsius */
#define FAN_TEMP_THRESHOLD_100   70      /* Celsius */
#define FAN_HYSTERESIS           10      /* Hysteresis for speed decrease */

/* Software Protection Limits */
#define MAX_NTC_TEMP_C           90      /* Maximum NTC temperature in Celsius */
#define MAX_TX_POWER1_W          1100    /* Maximum transmit power 1 in Watts */
#define MAX_TX_POWER2_W          1050    /* Maximum transmit power 2 in Watts */

/* Band Frequency Ranges (Hz) - 5 combined bands */
#define BAND_160M_MIN    1800000
#define BAND_160M_MAX    2100000
#define BAND_80M_MIN     3500000
#define BAND_80M_MAX     3900000
#define BAND_40M_30M_MIN 7000000
#define BAND_40M_30M_MAX 10150000
#define BAND_20M_17M_MIN 14000000
#define BAND_20M_17M_MAX 18168000
#define BAND_15M_10M_MIN 21000000
#define BAND_15M_10M_MAX 29700000

/* Band Selection Masks (for GPIO relay control) */
#define BAND_SEL_160M     0x01    /* BAND_SEL5X */
#define BAND_SEL_80M      0x02    /* BAND_SEL4X */
#define BAND_SEL_40M_30M  0x04    /* BAND_SEL3X */
#define BAND_SEL_20M_17M  0x08    /* BAND_SEL2X */
#define BAND_SEL_15M_10M  0x10    /* BAND_SEL1X */

/* Band strings for display */
extern const char* strFreqRanges[];
extern const char* strBands[];

/* Communication */
#define DEFAULT_PC_BAUD_RATE      19200
#define RX_QUEUE_SIZE             256
#define TX_QUEUE_SIZE             256

/* Frequency Detection */
/* TIM7 fires every 100us to check TIM4 ETR counter */
#define FREQDET_CHECK_INTERVAL_US  100     /* microseconds */
#define FREQDET_MULTIPLIER         10000   /* count * 10000 = Hz (1s/100us) */
#define FREQDET_MIN_VALID_FREQ_HZ  1000000 /* 1MHz threshold: ignore noise */

/* TIM7 config for 100us period (timer clock ~200MHz) */
#define FREQDET_TIM7_PRESCALER     19      /* divider = 20 */
#define FREQDET_TIM7_PERIOD        999     /* period = 1000 ticks */
/* 200MHz / 20 / 1000 = 10KHz = 100us */

/* Buzzer */
#define BUZZER_NOTE_DURATION_MS    300
#define BUZZER_INTERVAL_MS         3000

/*============================================================================
 * Data Types
 *============================================================================*/

/* Language Type */
typedef enum {
    MULTILANG_CHINESE = 0,
    MULTILANG_ENGLISH,
} LanguageType;

/* Radio Type */
typedef enum {
    RADIO_TYPE_UNKNOWN = 0,
    RADIO_TYPE_ICOM_CIV,
    RADIO_TYPE_YAESU,
    RADIO_TYPE_KENWOOD
} RadioType;

/* Band Type */
typedef enum {
    BAND_NONE = 0,
    BAND_160M,
    BAND_80M,
    BAND_40M_30M,
    BAND_20M_17M,
    BAND_15M_10M,
    BAND_FAULT      /* Invalid frequency */
} BandType;

/* Power State */
typedef enum {
    POWER_STATE_OFF = 0,
    POWER_STATE_STANDBY,
    POWER_STATE_TX
} PowerStateType;

/* Alert Type */
typedef enum {
    ALERT_NONE = 0,
    ALERT_SWR_HIGH,
    ALERT_REFLECT_POWER_HIGH,
    ALERT_CURRENT_HIGH,
    ALERT_HARDWARE_FAULT,
    ALERT_OVERDRIVE,
    ALERT_FREQ_MISMATCH
} AlertType;

/* Fan Speed */
typedef enum {
    FAN_SPEED_LOW = 0,
    FAN_SPEED_MED,
    FAN_SPEED_HIGH
} FanSpeedType;

/* Touch Screen Type */
typedef enum {
    TOUCH_NONE = 0,
    TOUCH_RESISTIVE,
    TOUCH_CAPACITIVE
} TouchType;

/*============================================================================
 * Multi-language String Structure
 *============================================================================*/

typedef struct {
    const char *zh;    /* Chinese string */
    const char *en;    /* English string */
} MultiLangStr;

/*============================================================================
 * ADC Data Structure
 *============================================================================*/

typedef struct {
    uint16_t raw[ADC_CHANNEL_COUNT];      /* Raw ADC values */
    float voltage;                         /* Main voltage (V) */
    float current;                         /* Main current (A) */
    float tx_power1;                       /* Transmit power 1 (W) */
    float tx_power2;                       /* Transmit power 2 (W) */
    float ref_power1;                      /* Reflected power 1 (W) */
    float ref_power2;                      /* Reflected power 2 (W) */
    float rf_input;                        /* RF input power (W) */
    float ntc_temp;                        /* NTC temperature (C) */
    float detector;                        /* Detector sample (V) */
    float swr;                             /* VSWR calculation */
} AdcDataType;

/*============================================================================
 * Calibration Data Structure
 *============================================================================*/

typedef struct {
    float forward_slope;       /* Forward power slope */
    float forward_offset;      /* Forward power offset */
    float reflect_slope;       /* Reflected power slope */
    float reflect_offset;      /* Reflected power offset */
} CalDataType;

/*============================================================================
 * Touch Calibration Structure
 *============================================================================*/

typedef struct {
    float x_slope;
    float x_offset;
    float y_slope;
    float y_offset;
    bool valid;
} TouchCalDataType;

/*============================================================================
 * Configuration Structure
 *============================================================================*/

typedef struct {
    /* Communication */
    uint32_t pc_baud_rate;
    RadioType radio_type;
    uint32_t radio_baud_rate;

    /* Protection thresholds */
    float protect_reflect_power2_w;
    float protect_reflect_power2_swr;
    float protect_swr_threshold;
    float protect_reflect_power1_w;
    float protect_main_current_a;
    float protect_ntc_temp_c;
    float protect_tx_power1_w;
    float protect_tx_power2_w;

    /* Calibration */
    CalDataType cal_tx_power1;
    CalDataType cal_tx_power2;
    CalDataType cal_ref_power1;
    CalDataType cal_ref_power2;
    CalDataType cal_rf_input;

    /* Touch calibration */
    TouchCalDataType touch_cal;

    /* Language */
    LanguageType language;

    /* Valid flag */
    uint32_t crc32;
} ConfigType;

/*============================================================================
 * Global Variables Declaration
 *============================================================================*/

/* ADC Data - placed in SRAM for DMA access */
extern volatile AdcDataType g_adc_data;

/* Configuration */
extern ConfigType g_config;

/* Current power state */
extern volatile PowerStateType g_power_state;

/* Current band */
extern volatile BandType g_current_band;

/* PTT state */
extern volatile bool g_ptt_active;

/* Alert state */
extern volatile AlertType g_alert_active;
extern volatile bool g_alert_flag;

/* Touch type detected */
extern TouchType g_touch_type;

/* Fan speed */
extern volatile FanSpeedType g_fan_speed;

/*============================================================================
 * Function Prototypes
 *============================================================================*/

/* Utility functions */
uint32_t crc32_calculate(const uint8_t *data, uint32_t length);
float linear_interpolate(float x, float x1, float y1, float x2, float y2);
BandType frequency_to_band(uint32_t frequency_hz);
float apply_calibration(uint16_t raw_adc, const CalDataType *cal, bool is_reflect);
float calculate_swr(float forward_power, float reflect_power);

#ifdef __cplusplus
}
#endif

#endif /* OPA2000_H */