/**
 * @file opa2000.c
 * @brief OPA2000 Global State and Utility Functions
 *
 * @details Defines all `extern volatile` globals declared in opa2000.h
 *          and provides band/SWR/CRC helpers shared between Control,
 *          GUI and Networking.
 *
 * @warning All `volatile` globals are touched from multiple tasks and
 *          ISRs (ADC DMA, COMP, EXTI).  Variables that are only read
 *          by the task side use `volatile` + `const` pattern.
 */

#include "opa2000.h"
#include "adc_calibration.h"
#include <math.h>     /*!< logf() for NTC B-equation */

/*============================================================================
 *  1. Global state (defined here, declared volatile in opa2000.h)
 *============================================================================*/

/* NOTE: g_adc_data, g_config live in RAM (default).  The DMA buffer
 * itself is placed in DTCMRAM by the linker script (see .ld). */

/* Raw + engineering values - updated by Control task from ADC DMA */
volatile AdcDataType    g_adc_data;

/* Front-panel state.  g_psu_on is the operator's power/ready switch,
 * g_tx_active follows the PTT relay sequence.  See opa2000.h. */
volatile bool           g_psu_on      = false;
volatile bool           g_tx_active   = false;

volatile BandType       g_current_band = BAND_NONE;

/* IM zero-offset calibration gate (Control sets true after success).
 * PWR_ON_SIG (PG7) and RF_BIAS_ON (PG2) are only allowed to be HIGH
 * when g_allow_rf is true. */
volatile bool           g_allow_rf     = false;

/* Alert */
volatile AlertType      g_alert_active = ALERT_NONE;
volatile bool           g_alert_flag   = false;

/* Fan speed */
volatile FanSpeedType   g_fan_speed    = FAN_SPEED_LOW;

/* Last measured RF frequency in Hz, written by Control freq_detect task */
volatile uint32_t       g_radio_freq_hz = 0U;

/* Operator-visible power state, kept in step with g_psu_on / g_tx_active
 * by power_control.c. */
volatile PowerStateType g_power_state   = POWER_STATE_OFF;

/* Persistent config blob.  Default-initialised in opa2000_init(); the
 * Networking agent's sdconfig.cpp overwrites it from /fs/config.bin
 * after startup. */
ConfigType g_config;

/*============================================================================
 *  2. Band / frequency helpers
 *============================================================================*/

BandType frequency_to_band(uint32_t frequency_hz)
{
    if (frequency_hz < BAND_160M_MIN) return BAND_NONE;
    if (frequency_hz <= BAND_160M_MAX) return BAND_160M;
    if (frequency_hz < BAND_80M_MIN)  return BAND_FAULT;
    if (frequency_hz <= BAND_80M_MAX)  return BAND_80M;
    if (frequency_hz < BAND_40M_30M_MIN) return BAND_FAULT;
    if (frequency_hz <= BAND_40M_30M_MAX) return BAND_40M_30M;
    if (frequency_hz < BAND_20M_17M_MIN) return BAND_FAULT;
    if (frequency_hz <= BAND_20M_17M_MAX) return BAND_20M_17M;
    if (frequency_hz < BAND_15M_10M_MIN) return BAND_FAULT;
    if (frequency_hz <= BAND_15M_10M_MAX) return BAND_15M_10M;
    return BAND_FAULT;
}

/* VSWR = (1 + |Gamma|) / (1 - |Gamma|), |Gamma| = sqrt(Pr / P_fwd).
 * Returned as x100 (e.g. 150 = 1.50).  Below 10 W forward power the
 * result is forced to 100 (=1.00) per design spec to avoid noise
 * inflating SWR at low signal levels. */

/* floor(sqrt(n)) for n <= 2^32-1, by binary search (16 iterations). */
static uint32_t isqrt_u32(uint32_t n)
{
    uint32_t lo = 0U, hi = 65535U, mid;

    if (n == 0U) return 0U;
    while (lo < hi) {
        mid = lo + ((hi - lo + 1U) / 2U);
        if (mid <= (n / mid)) lo = mid;
        else                    hi = mid - 1U;
    }
    return lo;
}

int32_t calculate_swr_x100(int32_t fwd2_mW, int32_t rev2_mW)
{
    uint32_t gamma_sq_q16, root, swr;

    if (fwd2_mW < 10000) return 100;       /* below 10 W */
    if (rev2_mW <= 0)     return 100;       /* perfect match / no signal */
    if (rev2_mW >= fwd2_mW) return 999;     /* impossible: clamp */

    gamma_sq_q16 = (uint32_t)(((int64_t)rev2_mW << 16) / fwd2_mW);
    root = isqrt_u32(gamma_sq_q16);          /* |Gamma| in Q8 (0..256)   */
    if (root >= 256U) return 999;

    swr = ((256U + root) * 100U) / (256U - root);
    if (swr < 100U) swr = 100U;
    if (swr > 999U) swr = 999U;
    return (int32_t)swr;
}

/*============================================================================
 *  3. CRC32 (polynomial 0xEDB88320, IEEE 802.3)
 *============================================================================*/

uint32_t crc32_calculate(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            uint32_t mask = (crc & 1U) ? 0xEDB88320U : 0U;
            crc = (crc >> 1) ^ mask;
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

/*============================================================================
 *  4. Band display strings (for GUI)
 *============================================================================*/

const char *strFreqRanges[] = {
    "1.800-2.100 MHz",   /* 160m   */
    "3.500-3.900 MHz",   /* 80m    */
    "7.000-10.150 MHz",  /* 40-30m */
    "14.000-18.168 MHz", /* 20-17m */
    "21.000-29.700 MHz", /* 15-10m */
};

const char *strBands[] = {
    "160m",
    "80m",
    "40-30m",
    "20-17m",
    "15-10m",
};

/*============================================================================
 *  5. Initialisation
 *============================================================================*/

void opa2000_init(void)
{
    /* Zero ADC data buffer */
    for (unsigned i = 0; i < ADC1_RANK_COUNT; i++) g_adc_data.adc1_raw[i] = 0U;
    g_adc_data.adc2_if_pa_raw       = 0U;
    g_adc_data.adc2_if_radio_raw    = 0U;
    g_adc_data.adc2_inj_ntc_core_raw = 0U;
    g_adc_data.adc2_inj_ntc_aux_raw  = 0U;
    g_adc_data.adc2_inj_tsense_raw    = 0U;
    g_adc_data.im_mA = 0;
    g_adc_data.vm_mV = 0;
    g_adc_data.fwd1_mW = g_adc_data.fwd2_mW = 0;
    g_adc_data.rev1_mW = g_adc_data.rev2_mW = 0;
    g_adc_data.rf_in_mW = 0;
    g_adc_data.vbias_mV = 0;
    g_adc_data.key_adc  = 0U;
    g_adc_data.ntc_core_tenth_c = 250;   /* 25.0 C default */
    g_adc_data.ntc_aux_tenth_c = 250;
    g_adc_data.mcu_int_tenth_c = 250;
    g_adc_data.swr_x100 = 100;
    g_adc_data.seq = 0U;

    /* Default config (Networking agent may overwrite from SD card) */
    g_config.pc_baud_rate          = DEFAULT_PC_BAUD_RATE;
    g_config.radio_type            = RADIO_TYPE_UNKNOWN;
    g_config.radio_baud_rate       = 19200U;
    for (int i = 0; i < 6; i++) g_config.band_rf_in_max_w[i] = 20U;
    g_config.band_rev1_max_w       = 600U;
    g_config.band_rev2_max_w       = 400U;
    g_config.band_rev2_swr_w       = 200U;
    g_config.band_swr_max_x100     = (uint16_t)DEFAULT_PROTECT_SWR_THRESHOLD_X100;
    g_config.band_ntc_max_tenth_c  = (uint16_t)(MAX_NTC_TEMP_C * 10U);
    g_config.band_fwd1_max_w       = (uint16_t)MAX_TX_POWER1_W;
    g_config.band_fwd2_max_w       = (uint16_t)MAX_TX_POWER2_W;
    g_config.band_im_max_mA        = (uint16_t)(MAX_MAIN_CURRENT_A * 1000U);
    g_config.band_vbias_max_mV     = 5000U;
    g_config.im_zero_offset_adc    = 43254U;  /* 1.65 V / 2.5 V * 65535 */
    g_config.im_calibrated         = 0U;
    g_config.language              = MULTILANG_CHINESE;
    g_config.buzzer_enable         = 1U;
    g_config.temp_fahrenheit       = 0U;

    /* Touch: identity transform, flagged uncalibrated so touch.c passes raw
     * through until the operator runs the calibration screen. */
    g_config.touch.x_gain = 1.0f;
    g_config.touch.x_offset = 0.0f;
    g_config.touch.y_gain = 1.0f;
    g_config.touch.y_offset = 0.0f;
    g_config.touch.calibrated = 0U;

    /* Coupler trims stay at zero until calibrated: adc_calibration.c treats
     * gain 0 as "use the OPA1000 front-end model unchanged". */
    g_config.fwd1 = (coupler_t){0, 0};
    g_config.fwd2 = (coupler_t){0, 0};
    g_config.rev1 = (coupler_t){0, 0};
    g_config.rev2 = (coupler_t){0, 0};

    g_psu_on      = false;
    g_tx_active   = false;
    g_power_state = POWER_STATE_OFF;
    g_current_band = BAND_NONE;
    g_allow_rf = false;
    g_alert_active = ALERT_NONE;
    g_alert_flag = false;
    g_fan_speed = FAN_SPEED_LOW;
    g_radio_freq_hz = 0U;
}