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
 * g_config.im_zero_offset_adc.
 *
 * It is a MEASUREMENT step, not a safety interlock: the amplifier may be
 * keyed as soon as the 100 ms power-on silence is over, whether or not
 * this calibration ever succeeds.  A current sensor that will not settle
 * costs the operator the current reading, not the amplifier.
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

/* DMA buffer size = 1-sample rolling average. */
#define ADC_DMA_BUFFER_SIZE    16U

/*!< Power-on silence, in milliseconds.
 *
 * From MX_ADC*_Init() until the DMA starts, adc_presettle() has powered
 * the converters but adc_start() has not been called, so no sample is
 * taken at all.  The first conversions of a cold ADC are not merely noisy,
 * they are wrong in a structured way: the internal regulator, the reference
 * buffer and the CC6920 bias all need to charge, and the coupler detectors'
 * DC rest point has not settled.  Sampling immediately would put that
 * transient into the peak holds (tau = PEAK_TAU_MS) and the GUI would show
 * a spike that decays over that time, which reads as a real transmission
 * that never happened.
 *
 * 100 ms is comfortably longer than the worst-case settling of the
 * reference buffer on this part, and still well inside the operator's
 * patience - the backlight comes up about as fast. */
#define ADC_PRESETTLE_MS       100U

/*============================================================================
 *  Coupler-2 peak hold (the GUI's TX power / reflected / SWR readout)
 *
 *  out[n] = max(sample[n], out[n-1] * exp(-dt / tau))
 *
 *  Attack is instantaneous; release is exponential.  tau is PEAK_TAU_MS:
 *  the hold falls to 1/e (36.8 %) of its amplitude one tau after the input
 *  drops below it, and to 5 % after three tau.  Slower than a plain
 *  averaging filter on purpose - the operator reads a peak value, not an
 *  instantaneous one, so a key-down has to stay on screen long enough to be
 *  read.  The cost is that the reflected / SWR readouts linger for a few
 *  tau after unkeying; nothing in the protection path reads these - those
 *  compare the instantaneous values.
 *
 *  The two constants live here rather than in user_adc.c so the host tests
 *  can derive their one-tau step from them instead of hard-coding the
 *  numbers.
 *==========================================================================*/

/*!< Release time constant of the peak hold, in milliseconds. */
#define PEAK_TAU_MS        2000U

/*!< Upper bound on one decay step, in milliseconds (2 tau).  A debugger
 *   pause or an interrupt storm then cannot turn into one giant jump, and
 *   the hold still cannot collapse in a single task tick. */
#define PEAK_MAX_STEP_MS   10000U

/*============================================================================
 *  Public API
 *============================================================================*/

/* Power the converters without sampling.  Call once after MX_ADC*_Init(),
 * then wait ADC_PRESETTLE_MS, then call adc_start().  Split from
 * adc_start() so the silence can be spent in a task rather than HAL_Delay(),
 * which would block every other task at boot. */
void adc_presettle(void);
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
 * exponential with tau = PEAK_TAU_MS.
 *
 * Call from task context only (the ADC task): the release uses expf(), and
 * this must not run in the priority-0 DMA interrupt.  It is deliberately
 * independent of PTT / g_tx_active - the holds report what the coupler
 * measures, not what the keying state suggests.
 *
 * Reads g_adc_data.fwd2_mW / rev2_mW (so adc_process() must have run at
 * least once) and writes fwd2_peak_mW / rev2_peak_mW. */
void adc_peak_hold_tick(void);

/* Re-arm the ADC2 injected sequence (NTC core, NTC aux, MCU die temp) so it
 * keeps producing fresh readings.
 *
 * WHY THIS IS NEEDED: MX_ADC2_Init() asks for ADC_INJECTED_SOFTWARE_START
 * with no trigger source and InjectedDiscontinuousConvMode = DISABLE, so
 * HAL_ADCEx_InjectedStart() runs the three ranks exactly once and the
 * sequence then parks.  Nothing in hardware re-arms it, so without this
 * call adc2_inj_*_raw stay frozen at the values measured at boot.
 *
 * Call from task context (the ADC task) at its own tick rate; the function
 * gates on its own 100 ms period, so calling it more often is free and
 * calling it less often stretches the interval.  One tick comparison plus,
 * every 100 ms, three microsecond-long conversions.
 *
 * Note this is unrelated to the NTC over-temperature protection, which
 * watches the ADC1 analog watchdog and never touches the injected path. */
void adc_injected_tick(void);

/* Power-on stabilisation gate.  This is the only entry point the ADC task
 * needs: it owns the whole power-up sequence, so a caller cannot forget a
 * step or run two of them concurrently.
 *
 *   1. The 100 ms power-on silence happens BEFORE this function is ever
 *      called (adc_presettle() + vTaskDelay in the task).  During it the
 *      converters are powered but no conversion is taken, and every
 *      watchdog is parked at full scale by boot_hold_thresholds_open().
 *   2. ARMING -> ARMED, on the first call: boot_arm_thresholds() pushes
 *      the real AWD/COMP limits out, CLEAR_FAULTX_OC is released, the
 *      comparators are started and g_allow_rf goes true.  From that
 *      moment the amplifier may be keyed.
 *   3. The IM zero-offset calibration runs in the BACKGROUND from then
 *      on, as a MEASUREMENT step.  It does not gate the transmit path:
 *      a current sensor that will not settle costs the operator the
 *      current reading, not the amplifier.
 *
 * Call from the ADC task at the task tick rate (50 ms).  Returns true
 * once (and after) ARMED. */
bool adc_stabilisation_tick(void);

/* True once the power-on silence is over, the real protection limits are
 * armed and the amplifier may be keyed.  PTT must stay disabled until
 * this is true.
 *
 * This is NOT "the zero-offset calibration finished" - that is an
 * independent measurement step. */
bool adc_stabilisation_done(void);

/* IM zero-offset state machine.  Run from a task at >= 10 ms tick.
 * Returns true once calibration has succeeded.
 *
 * This is a MEASUREMENT step: it produces g_config.im_zero_offset_adc so
 * the current reading can be expressed in amps, and (via the caller) the
 * AWD2 threshold can be re-fitted to it.  It does NOT set g_allow_rf and
 * does NOT gate the transmit path.
 *
 * NOTE: adc_stabilisation_tick() calls this for you.  Call it directly
 * only from a test. */
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