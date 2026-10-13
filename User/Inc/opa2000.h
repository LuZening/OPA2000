/**
 * @file opa2000.h
 * @brief OPA2000 Global Definitions and Types
 *
 * @details This is the SHARED shared-type header between Control, GUI
 *          and Networking agents.  It defines:
 *            - band enumeration and frequency ranges
 *            - power / alert / fan / touch / radio enums
 *            - the cross-task `AdcDataType` and `ConfigType` blobs
 *
 *          GPIO pin assignments live in User/Inc/hw_pinmap.h.
 *          ADC channel rank conversion functions live in
 *          User/Inc/adc_calibration.h.
 */

#ifndef OPA2000_H
#define OPA2000_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/*============================================================================
 * Configuration constants
 *============================================================================*/

/* ADC1 ranks - order MUST match dma_buf_adc1[] layout in user_adc.c.
 * dma_buf_adc1[ADC1_RANK_*] is filled by ADC1 DMA circular scan. */
#define ADC1_RANK_RF_IN        0U  /*!< INP4  (PC4)  - RF1 input power      */
#define ADC1_RANK_IM           1U  /*!< INP5  (PB1)  - main current IM       */
#define ADC1_RANK_REV1         2U  /*!< INP6  (PF12) - reflected power 1    */
#define ADC1_RANK_FWD1         3U  /*!< INP8  (PC5)  - forward power 1       */
#define ADC1_RANK_REV2         4U  /*!< INP9  (PB0)  - reflected power 2    */
#define ADC1_RANK_VBIAS        5U  /*!< INP14 (PA2)  - VBIAS_SENSE          */
#define ADC1_RANK_FWD2         6U  /*!< INP15 (PA3)  - forward power 2       */
#define ADC1_RANK_VM           7U  /*!< INP16 (PA0)  - main voltage          */
#define ADC1_RANK_KEY_ADC      8U  /*!< INP17 (PA1)  - key resistor divider */
#define ADC1_RANK_COUNT        9U

/* ADC2 ranks */
#define ADC2_RANK_IF_PA        0U  /*!< INP0  (PC2_C) - PA output 455kHz IF */
#define ADC2_RANK_IF_RADIO     1U  /*!< INP1  (PC3_C) - radio input 455kHz IF */
#define ADC2_INJ_NTC_CORE      0U  /*!< InjRank 1 = INP2  (PF13) - core NTC */
#define ADC2_INJ_NTC_AUX       1U  /*!< InjRank 2 = INP6  (PF14) - aux NTC  */
#define ADC2_INJ_TSENSE        2U  /*!< InjRank 3 = TEMPSENSOR             */

/* Protection defaults (overridable by config_t) */
#define DEFAULT_PROTECT_REFLECT_POWER2_W     400     /* Watts */
#define DEFAULT_PROTECT_REFLECT_POWER2_SWR   200     /* Watts */
#define DEFAULT_PROTECT_SWR_THRESHOLD_X100   200     /* 2.00 */
#define DEFAULT_PROTECT_REFLECT_POWER1_W     600     /* Watts */
#define DEFAULT_PROTECT_MAIN_CURRENT_A       30      /* Amperes */
#define DEFAULT_PROTECT_NTC_TEMP_C           90      /* Celsius */
#define DEFAULT_PROTECT_TX_POWER1_W         1100    /* Watts */
#define DEFAULT_PROTECT_TX_POWER2_W         1050    /* Watts */

/* Fan control */
#define FAN_SPEED_50_PERCENT     50
#define FAN_SPEED_80_PERCENT     80
#define FAN_SPEED_100_PERCENT    100
#define FAN_TEMP_THRESHOLD_80    60      /* Celsius - to 80% */
#define FAN_TEMP_THRESHOLD_100   70      /* Celsius - to 100% */
#define FAN_HYSTERESIS           10      /* degrees - sticky decrease */

/* Software protection limits */
#define MAX_NTC_TEMP_C           90
#define MAX_TX_POWER1_W          1100
#define MAX_TX_POWER2_W          1050
#define MAX_MAIN_CURRENT_A       30

/* Band frequency ranges (Hz) - 5 combined bands.
 * freq_to_band() returns BAND_* based on these. */
#define BAND_160M_MIN       1800000U
#define BAND_160M_MAX       2100000U
#define BAND_80M_MIN        3500000U
#define BAND_80M_MAX        3900000U
#define BAND_40M_30M_MIN    7000000U
#define BAND_40M_30M_MAX   10150000U
#define BAND_20M_17M_MIN   14000000U
#define BAND_20M_17M_MAX   18168000U
#define BAND_15M_10M_MIN   21000000U
#define BAND_15M_10M_MAX   29700000U

#define DEFAULT_PC_BAUD_RATE      19200U

/* Frequency detection (TIM1 ETR + TIM7 100us tick)
 * TIM7 ticks every 100us; counts accumulated; freq = count * 10000 */
#define FREQDET_MULTIPLIER         10000U  /*!< count * 10000 = Hz (1s/100us) */
#define FREQDET_MIN_VALID_FREQ_HZ  1000000U /*!< 1 MHz threshold */

/* Buzzer 3-tone warning timing */
#define BUZZER_NOTE_DURATION_MS    300U
#define BUZZER_NOTE_INTERVAL_MS    3000U
#define BUZZER_NOTE_HIGH_HZ        2500U
#define BUZZER_NOTE_MID_HZ         1500U
#define BUZZER_NOTE_LOW_HZ         800U

/*============================================================================
 * Data types
 *============================================================================*/

/* Language */
typedef enum {
    MULTILANG_CHINESE = 0,
    MULTILANG_ENGLISH,
} LanguageType;

/* Radio protocol (Networking agent owns the parser) */
typedef enum {
    RADIO_TYPE_UNKNOWN = 0,
    RADIO_TYPE_ICOM_CIV,
    RADIO_TYPE_YAESU,
    RADIO_TYPE_KENWOOD,
} RadioType;

/* Band */
typedef enum {
    BAND_NONE = 0,
    BAND_160M,
    BAND_80M,
    BAND_40M_30M,
    BAND_20M_17M,
    BAND_15M_10M,
    BAND_FAULT,            /*!< Invalid frequency */
} BandType;

/* Power state machine */
typedef enum {
    POWER_STATE_OFF = 0,
    POWER_STATE_STANDBY,
    POWER_STATE_TX,
} PowerStateType;

/* Alert cause */
typedef enum {
    ALERT_NONE = 0,
    ALERT_SWR_HIGH,
    ALERT_REFLECT_POWER_HIGH,
    ALERT_CURRENT_HIGH,
    ALERT_HARDWARE_FAULT,
    ALERT_OVERDRIVE,
    ALERT_FREQ_MISMATCH,
    ALERT_NTC_OVERHEAT,
    ALERT_VBIAS_FAULT,
} AlertType;

/* Fan speed state */
typedef enum {
    FAN_SPEED_LOW = 0,   /*!< 50 % */
    FAN_SPEED_MED,       /*!< 80 % */
    FAN_SPEED_HIGH,      /*!< 100 % */
} FanSpeedType;

/*============================================================================
 * Cross-task shared structures
 *============================================================================*/

/* Multi-language string for GUI */
typedef struct {
    const char *zh;
    const char *en;
} MultiLangStr;

/* ADC processed data - written by Control agent (user_adc.c),
 * read by GUI (via accessor) and protection task. */
typedef struct {
    /* Raw 16-bit ADC values from DMA buffer, indexed by ADC1_RANK_*. */
    volatile uint16_t adc1_raw[ADC1_RANK_COUNT];
    volatile uint16_t adc2_if_pa_raw;
    volatile uint16_t adc2_if_radio_raw;
    volatile uint16_t adc2_inj_ntc_core_raw;
    volatile uint16_t adc2_inj_ntc_aux_raw;
    volatile uint16_t adc2_inj_tsense_raw;

    /* Engineering units.  All values are post-calibration; physical
     * units documented in OPA2000_H7_固件设计要求.txt. */
    volatile int32_t im_mA;              /*!< main current, milli-Ampere */
    volatile int32_t vm_mV;              /*!< main voltage, milli-Volt */
    volatile int32_t fwd1_mW;            /*!< forward power 1, milli-Watt */
    volatile int32_t fwd2_mW;            /*!< forward power 2 */
    volatile int32_t rev1_mW;            /*!< reflected power 1 */
    volatile int32_t rev2_mW;            /*!< reflected power 2 */
    volatile int32_t rf_in_mW;           /*!< RF input power */
    volatile int32_t vbias_mV;           /*!< PA bias voltage, mV */
    volatile int32_t ntc_core_tenth_c;    /*!< core NTC, 0.1 Celsius */
    volatile int32_t ntc_aux_tenth_c;     /*!< aux NTC, 0.1 Celsius */
    volatile int32_t mcu_int_tenth_c;    /*!< MCU internal temp, 0.1 Celsius */

    /* VSWR - derived from fwd2_mW and rev2_mW.
     * If fwd2_mW < 10W (10 000 mW), swr is forced to 100 (=1.0). */
    volatile int32_t swr_x100;           /*!< SWR * 100 */

    /* Sequence counter (increments on every fresh DMA batch) */
    volatile uint32_t seq;
} AdcDataType;

/* Calibration for a single forward / reflected channel */
typedef struct {
    int32_t a_q16;     /*!< slope, Q16.16 fixed-point */
    int32_t b_mW;      /*!< intercept, milli-Watt */
} coupler_t;

/* Persistent configuration blob.  Stored at /fs/config.bin by
 * Networking/sdconfig.cpp.  Mirrored by config.c into the global g_config. */
typedef struct {
    uint32_t pc_baud_rate;
    RadioType radio_type;
    uint32_t radio_baud_rate;

    /* Per-band protection thresholds (Hz / Watts / Amps / Celsius) */
    uint16_t band_rf_in_max_w[6];     /*!< AWD1 RF input threshold, by band */
    uint16_t band_rev1_max_w;        /*!< AWD3 reflected 1 threshold */
    uint16_t band_rev2_max_w;        /*!< software REV2 threshold */
    uint16_t band_rev2_swr_w;        /*!< SWR-check REV2 threshold */
    uint16_t band_swr_max_x100;      /*!< SWR threshold * 100 */
    uint16_t band_ntc_max_tenth_c;   /*!< NTC overheat threshold */
    uint16_t band_fwd1_max_w;        /*!< FWD1 software threshold */
    uint16_t band_fwd2_max_w;        /*!< FWD2 software threshold */
    uint16_t band_im_max_a;          /*!< main current threshold */
    uint16_t band_vbias_max_mV;      /*!< VBIAS threshold */

    /* Coupler calibration (linear fit y = (a/65536) * raw + b) */
    coupler_t fwd1, fwd2, rev1, rev2;

    /* IM zero-offset calibration */
    uint16_t im_zero_offset_adc;
    uint8_t  im_calibrated;

    /* Misc */
    LanguageType language;
    uint8_t  buzzer_enable;
    uint8_t  reserved[6];

    uint32_t crc32;
} ConfigType;

/*============================================================================
 * Global state (declared; defined in opa2000.c)
 *============================================================================*/

extern volatile AdcDataType    g_adc_data;
extern volatile PowerStateType g_power_state;
extern volatile BandType       g_current_band;
extern volatile bool           g_ptt_active;
extern volatile bool           g_allow_rf;        /*!< CC6920 zero-offset done */
extern volatile AlertType      g_alert_active;
extern volatile bool           g_alert_flag;
extern volatile FanSpeedType   g_fan_speed;
extern volatile uint32_t       g_radio_freq_hz;   /*!< last measured Hz */

extern ConfigType  g_config;

/* Band strings for GUI display (defined in opa2000.c) */
extern const char *strFreqRanges[];
extern const char *strBands[];

/*============================================================================
 * Function prototypes
 *============================================================================*/

/* Utility functions */
uint32_t crc32_calculate(const uint8_t *data, uint32_t length);
BandType frequency_to_band(uint32_t frequency_hz);
int32_t  calculate_swr_x100(int32_t fwd2_mW, int32_t rev2_mW);

/* Initialization (called once at startup) */
void opa2000_init(void);

#ifdef __cplusplus
}
#endif

#endif /* OPA2000_H */