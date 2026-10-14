/**
 * @file user_adc.h
 * @brief ADC data acquisition, filtering and engineering-unit conversion.
 *
 * @details Three pipelines:
 *   1. ADC1 DMA circular (DMA1_Stream6) fills adc1_dma_buf[ADC1_RANK_COUNT].
 *   2. ADC2 DMA circular (DMA1_Stream7) fills adc2_dma_buf[ADC2_RANK_REG].
 *   3. ADC2 injected end-of-sequence IRQ fills adc2_inj_buf.
 *
 * user_adc_process() (called from HAL_ADC_ConvCpltCallback via task
 * notification, or periodically from the Control task) reads the latest
 * DMA snapshot, computes average / max / most-recent for each rank,
 * applies calibration, and stores the engineering-unit values into
 * g_adc_data (defined in opa2000.c).
 *
 * The IM zero-offset state machine (CC6920BSO-50A calibration) lives
 * in this file: on startup it averages ~500 ms of IM samples while
 * RF_BIAS / PWR_ON are off, validates the result, and writes
 * g_config.im_zero_offset_adc + sets g_allow_rf = true.
 */

#ifndef USER_ADC_H
#define USER_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "opa2000.h"
#include "adc_calibration.h"
#include "main.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;

/* DMA buffer size = 16-sample rolling average. */
#define ADC_DMA_BUFFER_SIZE    16U

/*============================================================================
 *  Public API
 *============================================================================*/

/* Start ADC1 + ADC2 circular DMA + injected conversion. */
void adc_start(void);
/* Stop ADC1 + ADC2 circular DMA. */
void adc_stop(void);
/* Process the latest DMA snapshot into g_adc_data.  Called from the
 * Control task periodically (e.g. 100 ms tick) and from the IM
 * zero-offset state machine during calibration.
 *
 * ALSO runs from the ADC1 DMA completion interrupt at main priority 0, so
 * it must stay integer-only and free of libm / FPU work. */
void adc_process(void);
/* ADC1 + ADC2 start + DMA enable.  Idempotent. */
void adc_init(void);

/* Advance the coupler-2 peak holds by one step, using the time elapsed
 * since the previous call.  Attack is instantaneous, release is
 * exponential with tau = 500 ms.
 *
 * Call from task context only (the ADC task): the release uses expf(), and
 * this must not run in the priority-0 DMA interrupt.  It is deliberately
 * independent of PTT / g_tx_active - the holds report what the coupler
 * measures, not what the keying state suggests.
 *
 * Reads g_adc_data.fwd2_mW / rev2_mW (so adc_process() must have run at
 * least once) and writes fwd2_peak_mW / rev2_peak_mW. */
void adc_peak_hold_tick(void);

/* IM zero-offset state machine.  Run from a task at >= 10 ms tick.
 * Returns true once calibration has succeeded (g_allow_rf set). */
bool adc_im_zero_offset_tick(void);

/* Place the DMA buffers in a dedicated linker section so the linker
 * script can locate them in DTCMRAM (zero-func) for cache-coherent
 * DMA access.  See STM32H7B0XX_FLASH.ld. */
extern uint16_t adc1_dma_buf[ADC_DMA_BUFFER_SIZE * ADC1_RANK_COUNT];
extern uint16_t adc2_dma_buf[ADC_DMA_BUFFER_SIZE * 2U];
extern volatile uint16_t adc2_inj_ntc_core_raw;
extern volatile uint16_t adc2_inj_ntc_aux_raw;
extern volatile uint16_t adc2_inj_tsense_raw;

#ifdef __cplusplus
}
#endif

#endif /* USER_ADC_H */