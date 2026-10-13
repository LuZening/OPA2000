/**
 * @file user_adc.c
 * @brief ADC data acquisition, filtering and IM zero-offset state
 *        machine.  All integer arithmetic.
 *
 * @details
 *   - adc1_dma_buf[ADC_DMA_BUFFER_SIZE][ADC1_RANK_COUNT] is filled
 *     by ADC1 circular DMA.  Each row is a single scan of 9 ranks.
 *   - adc2_dma_buf[ADC_DMA_BUFFER_SIZE][2] is the ADC2 regular scan
 *     buffer (INP0 + INP1 = 455 kHz IF samples).
 *   - adc2_inj_*_raw are written by the ADC2 injected end-of-sequence
 *     IRQ.  Three ranks: core NTC, aux NTC, internal temp sensor.
 *
 *   adc_process() walks the most-recent ADC1 buffer snapshot (latest
 *   ADC_DMA_BUFFER_SIZE samples), averages each rank, runs the
 *   adc*_raw_to_*_mW / _mV / _mA functions from adc_calibration.h,
 *   stores results in g_adc_data.
 *
 *   adc_im_zero_offset_tick() implements the CC6920BSO-50A
 *   zero-offset state machine described in OPA2000_H7_固件设计要求
 *   section [main current zero-offset calibration].  Called from the
 *   Control task at 10 ms tick.  Reads 50 samples (500 ms) before
 *   setting g_allow_rf = true.
 */

#include "user_adc.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/*============================================================================
 *  DMA buffer placement
 *============================================================================*/

/* Place DMA buffers in DTCMRAM (non-cacheable) for cache-coherent DMA.
 * See STM32H7B0XX_FLASH.ld to ensure .dma_buffer section is in DTCMR. */
#define DMA_BUFFER_SECTION  __attribute__((section(".dma_buffer"), aligned(4)))

uint16_t adc1_dma_buf[ADC_DMA_BUFFER_SIZE * ADC1_RANK_COUNT] DMA_BUFFER_SECTION;
uint16_t adc2_dma_buf[ADC_DMA_BUFFER_SIZE * 2U]              DMA_BUFFER_SECTION;

/* ADC2 injected ranks written by the JEOC ISR.  Modified by ISR. */
volatile uint16_t adc2_inj_ntc_core_raw;
volatile uint16_t adc2_inj_ntc_aux_raw;
volatile uint16_t adc2_inj_tsense_raw;

/*============================================================================
 *  IM zero-offset state machine
 *============================================================================*/

/* Phases:
 *   0 = idle (already calibrated or never run)
 *   1 = sampling (~500 ms window)
 *   2 = validating the average
 *   3 = failed -> back to 1 (or 0 after 3 retries)
 *   0 = success -> g_allow_rf = true
 */
#define IM_CAL_SAMPLE_COUNT  50U        /*!< ~500 ms at 10 ms tick */

typedef struct {
    uint8_t  phase;
    uint8_t  retry;
    uint16_t accum[IM_CAL_SAMPLE_COUNT];
    uint8_t  sample_idx;
} im_cal_state_t;

static im_cal_state_t s_im_cal = { .phase = 1U, .retry = 0U,
                                   .sample_idx = 0U };

/* Expected ADC code at I = 0 A: 1.65 V / 2.5 V * 65535 = 43253.86.
 * Tolerance: ±40 mV -> ±1048 LSB. */
#define IM_ZERO_EXPECTED     43254
#define IM_ZERO_TOLERANCE    1048

/*============================================================================
 *  Public API
 *============================================================================*/

void adc_init(void)
{
    /* CubeMX has configured DMA in MX_DMA_Init + ADC1/2 in MX_ADC1_Init /
     * MX_ADC2_Init.  We only need to (re)zero the buffers and reset the
     * IM zero-offset state machine so it runs again on each boot. */
    memset((void *)adc1_dma_buf, 0, sizeof adc1_dma_buf);
    memset((void *)adc2_dma_buf, 0, sizeof adc2_dma_buf);
    adc2_inj_ntc_core_raw = 0U;
    adc2_inj_ntc_aux_raw  = 0U;
    adc2_inj_tsense_raw    = 0U;

    s_im_cal.phase       = 1U;
    s_im_cal.retry        = 0U;
    s_im_cal.sample_idx   = 0U;
    memset(s_im_cal.accum, 0, sizeof s_im_cal.accum);

    /* Allow calibration to re-arm on every boot. */
    g_config.im_calibrated    = 0U;
    g_allow_rf                = false;
}

void adc_start(void)
{
    if (HAL_ADC_Start_DMA(&hadc1,
                          (uint32_t *)adc1_dma_buf,
                          ADC_DMA_BUFFER_SIZE * ADC1_RANK_COUNT)
        != HAL_OK) {
        Error_Handler();
    }
    /* Enable ADC1 AWD interrupt (only WatchDog interrupts per design) */
    /* (configured in HAL_ADC_Start but kept disabled until calibration done) */

    if (HAL_ADC_Start_DMA(&hadc2,
                          (uint32_t *)adc2_dma_buf,
                          ADC_DMA_BUFFER_SIZE * 2U)
        != HAL_OK) {
        Error_Handler();
    }
    /* Trigger injected conversion once per started interrupt cycle. */
    if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK) {
        Error_Handler();
    }
}

void adc_stop(void)
{
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_ADCEx_InjectedStop(&hadc2);
}

/*============================================================================
 *  Process DMA buffer -> g_adc_data (integer units)
 *============================================================================*/

static inline uint16_t adc1_latest(uint8_t rank)
{
    /* Last row of DMA buffer. */
    return adc1_dma_buf[(ADC_DMA_BUFFER_SIZE - 1U) * ADC1_RANK_COUNT + rank];
}

static uint16_t adc1_average_rank(uint8_t rank, uint16_t *out_avg)
{
    uint32_t sum = 0U;
    for (uint16_t i = 0; i < ADC_DMA_BUFFER_SIZE; i++) {
        sum += adc1_dma_buf[i * ADC1_RANK_COUNT + rank];
    }
    *out_avg = (uint16_t)(sum / ADC_DMA_BUFFER_SIZE);
    return 0;
}

void adc_process(void)
{
    uint16_t avg;

    /* ADC1 ranks: average across the rolling DMA buffer. */
    uint16_t raw_rank[ADC1_RANK_COUNT];
    for (uint8_t r = 0; r < ADC1_RANK_COUNT; r++) {
        adc1_average_rank(r, &raw_rank[r]);
        g_adc_data.adc1_raw[r] = raw_rank[r];
    }

    /* Convert to engineering units via adc_calibration.h. */
    g_adc_data.rf_in_mW  = adc1_raw_to_rf_in_mW(raw_rank[ADC1_RANK_RF_IN]);
    g_adc_data.im_mA     = adc1_raw_to_im_mA(raw_rank[ADC1_RANK_IM]);
    g_adc_data.rev1_mW   = adc1_raw_to_rev1_mW(raw_rank[ADC1_RANK_REV1]);
    g_adc_data.fwd1_mW   = adc1_raw_to_fwd1_mW(raw_rank[ADC1_RANK_FWD1]);
    g_adc_data.rev2_mW   = adc1_raw_to_rev2_mW(raw_rank[ADC1_RANK_REV2]);
    g_adc_data.vbias_mV  = adc1_raw_to_vbias_sense_mV(raw_rank[ADC1_RANK_VBIAS]);
    g_adc_data.fwd2_mW   = adc1_raw_to_fwd2_mW(raw_rank[ADC1_RANK_FWD2]);
    g_adc_data.vm_mV    = adc1_raw_to_vm_mV(raw_rank[ADC1_RANK_VM]);
    g_adc_data.key_adc   = raw_rank[ADC1_RANK_KEY_ADC];     /* raw code, no unit */

    /* ADC2: keep the latest 455kHz IF samples for the GUI / demod task. */
    g_adc_data.adc2_if_pa_raw    = adc2_dma_buf[(ADC_DMA_BUFFER_SIZE - 1U) * 2U];
    g_adc_data.adc2_if_radio_raw = adc2_dma_buf[(ADC_DMA_BUFFER_SIZE - 1U) * 2U + 1U];

    /* ADC2 injected channels (one-shot per trigger) */
    g_adc_data.adc2_inj_ntc_core_raw = adc2_inj_ntc_core_raw;
    g_adc_data.adc2_inj_ntc_aux_raw  = adc2_inj_ntc_aux_raw;
    g_adc_data.adc2_inj_tsense_raw    = adc2_inj_tsense_raw;
    g_adc_data.ntc_core_tenth_c  = adc2_inj_raw_to_core_ntc_tenth_c(adc2_inj_ntc_core_raw);
    g_adc_data.ntc_aux_tenth_c   = adc2_inj_raw_to_aux_ntc_tenth_c(adc2_inj_ntc_aux_raw);
    g_adc_data.mcu_int_tenth_c   = adc2_inj_raw_to_mcu_internal_tenth_c(adc2_inj_tsense_raw);

    /* SWR: use fwd2/rev2 milli-Watts, return x100. */
    g_adc_data.swr_x100 = calculate_swr_x100(g_adc_data.fwd2_mW,
                                            g_adc_data.rev2_mW);

    g_adc_data.seq++;
}

/*============================================================================
 *  IM zero-offset state machine
 *============================================================================*/

bool adc_im_zero_offset_tick(void)
{
    if (g_config.im_calibrated) {
        return true;
    }

    switch (s_im_cal.phase) {
    case 1: { /* sampling */
        /* Read IM rank from the latest DMA buffer; skip the first sample. */
        uint16_t raw = adc1_latest(ADC1_RANK_IM);

        s_im_cal.accum[s_im_cal.sample_idx++] = raw;
        if (s_im_cal.sample_idx >= IM_CAL_SAMPLE_COUNT) {
            s_im_cal.sample_idx = 0U;
            s_im_cal.phase = 2U;
        }
        return false;
    }
    case 2: { /* validate */
        /* Mean */
        uint32_t sum = 0U;
        for (uint8_t i = 0; i < IM_CAL_SAMPLE_COUNT; i++) {
            sum += s_im_cal.accum[i];
        }
        uint16_t avg = (uint16_t)(sum / IM_CAL_SAMPLE_COUNT);
        int32_t dev = (int32_t)avg - (int32_t)IM_ZERO_EXPECTED;
        if (dev < 0) dev = -dev;
        if (dev <= IM_ZERO_TOLERANCE) {
            /* Success */
            g_config.im_zero_offset_adc = avg;
            g_config.im_calibrated = 1U;
            g_allow_rf = true;
            s_im_cal.phase = 0U;
            return true;
        }
        /* Failure - retry up to 3 times then alarm */
        if (++s_im_cal.retry >= 3U) {
            g_alert_active = ALERT_HARDWARE_FAULT;
            g_alert_flag   = true;
            s_im_cal.phase = 0U;     /* stop sampling */
            return false;
        }
        /* back to sampling */
        s_im_cal.phase = 1U;
        s_im_cal.sample_idx = 0U;
        return false;
    }
    default:
        return g_config.im_calibrated != 0U;
    }
}