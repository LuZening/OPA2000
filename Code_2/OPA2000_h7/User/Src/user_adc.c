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
 *   It also publishes the coupler-2 instantaneous values that the peak
 *   holds (fwd2_peak_mW / rev2_peak_mW) are derived from.  The holds
 *   themselves are advanced by adc_peak_hold_tick() from the ADC task,
 *   never from here: this function runs in the ADC1 DMA interrupt at main
 *   priority 0 and the release step needs expf().
 *
 *   adc_im_zero_offset_tick() implements the CC6920BSO-50A
 *   zero-offset state machine described in OPA2000_H7_固件设计要求
 *   section [main current zero-offset calibration].  Called from the
 *   Control task at 10 ms tick.  Reads 50 samples (500 ms) before
 *   setting g_allow_rf = true.
 */

#include "user_adc.h"
#include "boot.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include <math.h>

/*============================================================================
 *  DMA buffer placement
 *============================================================================*/

/* DMA 缓冲放在 AXI SRAM 的不可缓存窗口（app.ld 的 .dma_no_cache，
 * 0x24040000 起，由 MPU 标为 non-cacheable）。
 * 注意：H7 的 DMA1/DMA2（ADC 经 DMAMUX 挂在其上）无法访问 DTCM/ITCM，
 * 所以 .dma_buffer 不能放 DTCM——放到非缓存 SRAM 后 DMA 与 CPU 天然一致。
 * 该属性仅 GCC 有效；host 测试构建会忽略它。 */
#if defined(__GNUC__)
#define DMA_BUFFER_SECTION  __attribute__((section(".dma_buffer"), aligned(4)))
#else
#define DMA_BUFFER_SECTION
#endif

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
 *  Power-on stabilisation gate
 *============================================================================
 *
 *  The analog front end is not trustworthy the instant the peripherals come
 *  up: the rail is still charging, the D5/R40 peak detector is climbing out
 *  of its reset floor, and the CC6920 output is wherever the bias network
 *  happens to sit.  Sampling it early yields a wrong zero offset and, worse,
 *  a displayed reading that jumps around.
 *
 *  Three stages, in this order:
 *
 *    SETTLING   1 s from the first tick.  Nothing is believed and no
 *               threshold is armed - boot_hold_thresholds_open() already
 *               parked every watchdog at full scale, so nothing can latch.
 *               CLEAR_FAULTX_OC is held low throughout, which keeps the
 *               external fault latch in its cleared state.
 *    CALIBRATING  the CC6920 zero-offset calibration runs (the existing
 *               IM_CAL_SAMPLE_COUNT-window state machine below).
 *    ARMED      calibration passed: the real AWD/COMP limits are pushed,
 *               CLEAR_FAULTX_OC is released, the GUI starts showing ADC
 *               values, and g_ptt_ready may finally go true.
 *
 *  Only the ADC task ticks this, so the stage advances at a predictable
 *  50 ms regardless of what the other tasks are doing.
 *==========================================================================*/

#define STAB_SETTLE_TICKS    20U   /* 20 ticks x 50 ms = 1.0 s */

typedef enum {
    STAB_SETTLING = 0,
    STAB_CALIBRATING,
    STAB_ARMED
} stab_stage_t;

static stab_stage_t s_stab_stage = STAB_SETTLING;
static uint8_t       s_stab_ticks = 0U;

/** @brief True once the settling period has passed and the limits are armed.
 *  Power_control gates PTT on this via g_ptt_ready. */
bool adc_stabilisation_done(void)
{
    return s_stab_stage == STAB_ARMED;
}

/*============================================================================
 *  Peak-hold (peak detector with exponential release)
 *
 *  The GUI shows three bar graphs - forward power, reflected power and SWR -
 *  and derives all three from the same two held values, so they always agree.
 *  A plain instantaneous reading is unreadable on a bar graph: between two
 *  syllables of speech the forward power collapses and the bar drops to zero.
 *  The classic fix is a peak detector: attack instantly, release slowly.
 *
 *      out[n] = max(sample[n], out[n-1] * exp(-dt / tau))
 *
 *  tau is 500 ms as specified: after the input falls below the held value,
 *  the hold decays to 1/e (36.8 %) of its amplitude in 500 ms, to 5 % in
 *  1.5 s and to ~0.7 % in 5 s.  Fast enough to look responsive, slow enough
 *  that a short spike is still readable.
 *
 *  Two properties this deliberately does NOT have:
 *
 *  - No dependence on PTT / g_tx_active.  A peak meter is an instrument; it
 *    reports what the coupler actually sees.  If the reading is zero when
 *    idle, the hold decays to zero by itself; if it is non-zero (noise, a
 *    stuck front end, RF leaking in from elsewhere) that is exactly what the
 *    operator needs to see, so it is shown rather than hidden.  Gating on
 *    key-down would also make the bar jump at a moment unrelated to the RF.
 *
 *  - No FPU in interrupt context.  adc_process() also runs from the ADC1 DMA
 *    completion interrupt at main priority 0, so the peak-hold step is
 *    deliberately kept out of that path: adc_process() publishes the
 *    instantaneous values only, and adc_peak_hold_tick() - which does the
 *    expf() - is called from the ADC task.  That keeps the highest priority
 *    interrupt free of both library calls and wasted arithmetic.
 *==========================================================================*/

/*!< Release time constant, seconds (spec: 500 ms). */
#define PEAK_TAU_S             0.5f

/*!< Upper bound on one decay step.  Past ~10 tau the curve is visually zero
 *   anyway, and clamping keeps a long task stall from turning into one giant
 *   jump that would look like a glitch on screen. */
#define PEAK_MAX_STEP_MS       10000U

/* Time of the previous peak-hold step, in ticks.  Read by the ADC task
 * only, so no volatile and no atomicity concerns. */
static uint32_t s_peak_last_tick = 0U;

/* One peak-hold step for a single channel. */
static float peak_hold_step(float prev, float sample, float dt_s)
{
    if (sample >= prev) {
        return sample;               /* attack: instantaneous */
    }
    if (dt_s <= 0.0f) {
        return prev;                 /* no elapsed time -> no release */
    }
    float decayed = prev * expf(-dt_s / PEAK_TAU_S);

    /* Guard the tail: expf() underflows towards 0 and the result is stored
     * back into an int32_t, so anything below the representable resolution
     * is snapped to exactly zero rather than left to dither. */
    if (decayed < 0.5f) {
        decayed = 0.0f;
    }
    /* sample can still be the larger of the two after the decay. */
    return (sample > decayed) ? sample : decayed;
}

void adc_peak_hold_tick(void)
{
    /* ADC task context (never the DMA interrupt): the FPU and libm are
     * both fine here. */
    uint32_t now = HAL_GetTick();

    /* Unsigned subtraction, so the HAL_GetTick() wrap at ~49 days is a
     * non-issue.  Saturate the step so a long stall (e.g. the debugger
     * breakpoint or a burst of higher-priority interrupts) cannot produce a
     * negative interval. */
    uint32_t dt_ms = now - s_peak_last_tick;
    s_peak_last_tick = now;
    if (dt_ms > PEAK_MAX_STEP_MS) {
        dt_ms = PEAK_MAX_STEP_MS;
    }
    float dt_s = (float)dt_ms * 0.001f;

    g_adc_data.fwd2_peak_mW =
        (int32_t)peak_hold_step((float)g_adc_data.fwd2_peak_mW,
                                (float)g_adc_data.fwd2_mW, dt_s);
    g_adc_data.rev2_peak_mW =
        (int32_t)peak_hold_step((float)g_adc_data.rev2_peak_mW,
                                (float)g_adc_data.rev2_mW, dt_s);
}

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

    /* Peak holds start empty; the first tick establishes the time base. */
    s_peak_last_tick      = 0U;
    g_adc_data.fwd2_peak_mW = 0;
    g_adc_data.rev2_peak_mW = 0;

    /* Power-on stabilisation restarts too.  adc_init() runs once per boot
     * from app_adc_task(), so without this a soft restart (or a unit test
     * calling adc_init() to reset the module) would leave the gate sitting
     * in STAB_ARMED from the previous run and skip the settling window
     * entirely - i.e. it would key the amplifier on readings taken while
     * the analog side was still moving. */
    s_stab_stage          = STAB_SETTLING;
    s_stab_ticks          = 0U;

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
    /* AWD1/AWD2/AWD3 interrupts are already armed by MX_ADC1_Init():
     * it calls HAL_ADC_AnalogWDGConfig() once per watchdog with
     * ITMode = ENABLE, and that HAL call enables the matching bit in
     * ADC1->IER itself (stm32h7xx_hal_adc.c, the AWD2/AWD3 branches).
     * Routing and thresholds:
     *   AWD1 -> CH4 RF input power   (threshold via adc_awd_set_rf_in_mW)
     *   AWD2 -> CH5 main current IM  (threshold via adc_awd_set_im_mA)
     *   AWD3 -> CH6 coupler 1 reverse (threshold via adc_awd_set_rev1_mW)
     * So nothing extra is needed here beyond starting the conversions. */

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
    g_adc_data.key_adc   = adc1_raw_to_key_adc_code(raw_rank[ADC1_RANK_KEY_ADC]);

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

    /* SWR: use fwd2/rev2 milli-Watts, return x100.  This is the
     * instantaneous value; the GUI derives its own SWR from the peak holds
     * so all three of its bar graphs agree with each other. */
    g_adc_data.swr_x100 = calculate_swr_x100(g_adc_data.fwd2_mW,
                                            g_adc_data.rev2_mW);

    /* NOTE: the peak holds are advanced by adc_peak_hold_tick(), not here.
     * This function also runs from the ADC1 DMA completion interrupt at main
     * priority 0, and the release step needs expf(); keeping it out of here
     * keeps libm and the FPU out of the highest priority interrupt. */

    g_adc_data.seq++;
}

/*============================================================================
 *  Power-on stabilisation driver
 *============================================================================*/

bool adc_stabilisation_tick(void)
{
    switch (s_stab_stage) {
    case STAB_SETTLING:
        /* Hold CLEAR_FAULTX_OC low for the whole settling window.  The pin
         * is open-collector on the board (C65-2 / D10-1 / R71-1), so driving
         * it low is what actually resets the external fault latch - a plain
         * GPIO write high would merely release it. */
        CLEAR_FAULTX_OC_LOW();

        if (++s_stab_ticks >= STAB_SETTLE_TICKS) {
            s_stab_stage = STAB_CALIBRATING;
            /* Fresh calibration window: main() may have loaded a stored
             * im_zero_offset_adc, but we want one measured now that the
             * analog side is settled. */
            g_config.im_calibrated   = 0U;
            g_allow_rf               = false;
            s_im_cal.phase           = 1U;
            s_im_cal.retry           = 0U;
            s_im_cal.sample_idx      = 0U;
        }
        return false;

    case STAB_CALIBRATING:
        /* CLEAR_FAULTX_OC stays low until the calibration has actually
         * produced a usable offset. */
        CLEAR_FAULTX_OC_LOW();

        if (adc_im_zero_offset_tick()) {
            /* Calibration passed.  This is the first moment the readings
             * can be believed, so this is where the real limits go out -
             * boot_arm_thresholds() both closes the settling gate and
             * applies, so the very next call to boot_apply_thresholds()
             * (a band change, a Settings edit) works normally again. */
            boot_arm_thresholds((int)g_current_band);
            CLEAR_FAULTX_OC_HIGH();
            s_stab_stage = STAB_ARMED;
            return true;
        }
        return false;

    case STAB_ARMED:
    default:
        return true;
    }
}

/*============================================================================
 *  IM zero-offset state machine
 *============================================================================*/

bool adc_im_zero_offset_tick(void){
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