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
#include "protection.h"
#include "stm32h7xx_hal.h"
#include "comp.h"
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

/* IM_ZERO_EXPECTED / IM_ZERO_TOLERANCE live in adc_calibration.h, because
 * adc_calibration.c needs them too: the AWD2 threshold is fitted to the
 * measured zero offset, so it has to know the same plausible band the
 * calibration validates against. */

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
 *  Two INDEPENDENT things come out of the power-on window, and they are
 *  deliberately not tied to each other:
 *
 *    1. The AMPLIFIER MAY TRANSMIT.  This is what g_allow_rf means, and
 *       it becomes true as soon as the 100 ms power-on silence
 *       (adc_presettle() + vTaskDelay(ADC_PRESETTLE_MS)) is over and the
 *       DMA is running.  From that point the readings are good enough to
 *       judge the amplifier against its limits, so the operator may key.
 *
 *    2. The MAIN-CURRENT ZERO OFFSET is calibrated.  This is a
 *       MEASUREMENT step, not a safety interlock: it produces the number
 *       that makes g_adc_data.im_mA read as amps instead of as a raw
 *       code.  It runs in the background and its outcome does NOT gate
 *       the transmit path.
 *
 *  Why they are separated: a current sensor whose zero offset will not
 *  settle is a measurement problem, and the operator still needs to be
 *  able to transmit.  Making the calibration an interlock meant that a
 *  drifting CC6920 (or a wrong reference voltage) silently turned the
 *  amplifier into a brick - it could not be keyed at all, and the only
 *  visible symptom was a current reading that never made sense.  Now a
 *  failed calibration costs you the current reading and nothing else.
 *
 *  The 1.0 s "do nothing" stage that used to live between ADC_PRESETTLE_MS
 *  and the calibration is gone as well.  The silence is enough on its own:
 *  boot_hold_thresholds_open() parks every watchdog at full scale from
 *  main(), so even the very first DMA-full interrupt cannot latch.
 *
 *  Only the ADC task ticks this, so the stage advances at a predictable
 *  50 ms regardless of what the other tasks are doing.
 *==========================================================================*/

typedef enum {
    STAB_ARMING = 0,
    STAB_COMPARATOR_WAIT,   /*!< settling window; COMP1/COMP2 not enabled yet */
    STAB_ARMED
} stab_stage_t;

static stab_stage_t s_stab_stage = STAB_ARMING;

/*!< True once the zero-offset calibration has produced a usable offset AND
 *   the current protection (AWD2 + COMP2) has been re-fitted to it.  The
 *   re-fit is the reason this flag exists: both the watchdog threshold and
 *   the comparator reference are expressed relative to the zero offset, so
 *   a freshly measured offset moves them and they have to follow it. */
static bool s_current_protection_fitted = false;

/* How long the analog side is given to settle before the comparators are
 *   enabled.  20 ticks at the 50 ms ADC-task cadence is 1 s, matching the
 *   post-silence settling window the design spec already calls for.
 *
 *   Why a delay at all: COMP1 watches the D5/R40 peak detector, whose
 *   output is still charging at the moment the silence ends.  A comparator
 *   evaluates continuously the moment it is enabled, so enabling it
 *   against a threshold straight out of g_config means its FIRST
 *   comparison already sees the charging output above the limit and
 *   latches ALERT_OVERDRIVE at every cold boot.  Enabling it only after
 *   this window means the first comparison is against a settled input, so
 *   an edge means a real fault. */
#define STAB_COMP_SETTLE_TICKS   20U
static uint8_t s_comp_settle_ticks = 0U;

/** @brief True once the power-on silence is over and the limits are armed.
 *  Power_control gates PTT on this via g_ptt_ready.
 *
 *  This is NOT "the zero-offset calibration finished" any more - that is
 *  an independent measurement step (see the block comment above).  What it
 *  means is: the converters are running, the real AWD/COMP limits are
 *  pushed, and the amplifier may be keyed. */
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
 *  tau is PEAK_TAU_MS: after the input falls below the held value, the
 *  hold decays to 1/e (36.8 %) of its amplitude in one tau and to 5 % in
 *  three.  Long on purpose - the reading has to stay on screen long enough
 *  to be read.  The cost is that the reflected / SWR readouts also linger
 *  for a few tau after unkeying; no protection decision reads these
 *  (protection compares the instantaneous values in g_adc_data).
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

/*!< Release time constant, seconds.  The published value is PEAK_TAU_MS
 *   (user_adc.h) so the host tests can derive their one-tau step from it;
 *   this is only the form the decay needs. */
#define PEAK_TAU_S             ((float)PEAK_TAU_MS * 0.001f)

/* Time of the previous peak-hold step, in ticks.  Read by the ADC task
 * only, so no volatile and no atomicity concerns. */
static uint32_t s_peak_last_tick = 0U;

/*============================================================================
 *  ADC2 injected conversions - periodic restart
 *
 * Why they stop after one pass
 * ---------------------------
 *  MX_ADC2_Init() configures the injected sequence as:
 *      ExternalTrigInjecConv          = ADC_INJECTED_SOFTWARE_START
 *      InjectedDiscontinuousConvMode  = DISABLE
 *      InjectedNbrOfConversion       = 3   (NTC core, NTC aux, die temp)
 *
 *  With no trigger source and discontinuous mode off, the injected state
 *  machine runs those three ranks exactly once and then parks itself.
 *  Nothing re-arms it, so adc2_inj_*_raw freeze at whatever the first pass
 *  measured - which on a cold part is the ambient temperature at boot,
 *  forever.  The NTC readings on the status bar and the MCU die temperature
 *  in the sensor-inspect page are then not measurements at all.
 *
 *  The fix
 *  -------
 *  Re-arm the sequence from task context on a timer.  Software start plus a
 *  re-arm is enough: the three conversions take microseconds (1.5 cycles +
 *  12-bit sample on three channels), so re-arming at 100 ms costs nothing.
 *
 *  100 ms, not faster: the slowest thing these feeds is the NTC number on
 *  the status bar, which is meaningless to read faster than the eye can
 *  resolve.  A faster rate would just be interrupt traffic for the same
 *  information.
 *
 *  Why the _IT variant
 *  -------------------
 *  The re-arm must be HAL_ADCEx_InjectedStart_IT(), not the polling
 *  HAL_ADCEx_InjectedStart().  The difference is not cosmetic:
 *
 *    - HAL_ADCEx_InjectedStart()     starts the sequence and returns.  It
 *      never enables the end-of-sequence interrupt, so nothing ever reads
 *      the JDRx registers.  The conversion result stays in the hardware
 *      and adc2_inj_*_raw keep whatever they were last set to.
 *    - HAL_ADCEx_InjectedStart_IT()   additionally does
 *      __HAL_ADC_ENABLE_IT(hadc, ADC_IT_JEOS) (stm32h7xx_hal_adc_ex.c,
 *      the ADC_EOC_SEQ_CONV branch, which is what hadc2 is configured for).
 *      The end-of-sequence interrupt is what runs
 *      HAL_ADCEx_InjectedConvCpltCallback() in main.c, and THAT is what
 *      reads JDR1/JDR2/JDR3 into adc2_inj_ntc_core_raw /
 *      adc2_inj_ntc_aux_raw / adc2_inj_tsense_raw and then calls
 *      protection_check_ntc().
 *
 *  So with the polling variant the NTC over-temperature protection is dead
 *  code: protection_check_ntc() would never be called from the injected
 *  path, and the NTC reading the operator sees is frozen at the value the
 *  very first conversion produced.  The re-arm alone does not fix that; the
 *  interrupt is what makes the result observable.
 *
 *  One re-arm per period, and the interrupt does the publishing.
 *==========================================================================*/

/*!< Re-arm interval, milliseconds.  Matches the requirement's floor of
 *  "at least every 100 ms". */
#define INJ_PERIOD_MS            100U

/*!< Tick of the previous re-arm.  Unsigned subtraction makes the
 *  HAL_GetTick() wrap (~49 days) a non-issue. */
static uint32_t s_inj_last_tick  = 0U;

/**
 * @brief Re-arm the ADC2 injected sequence if the period has elapsed
 *
 * Task context only (the ADC task).  Call at the task tick rate; the call
 * itself is a tick comparison plus an occasional three-conversion burst, so
 * calling it every 50 ms is indistinguishable from calling it every 100 ms
 * in cost, and calling it more often than that is pointless because the
 * period check gates the actual start.
 *
 * The start uses the _IT variant on purpose - see the block comment above:
 * it is the end-of-sequence interrupt that publishes the JDRs and runs the
 * NTC over-temperature check.
 *
 * Failure is ignored on purpose.  The start only fails on a bad state or a
 * disabled ADC, and if that is true then re-arming every 100 ms forever
 * would spin in a task and bury the real cause.  One attempt per period is
 * the smallest useful retry, and if the peripheral is truly broken the
 * protection path reports that long before the NTC display does.
 */
void adc_injected_tick(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - s_inj_last_tick) < INJ_PERIOD_MS) {
        return;
    }
    s_inj_last_tick = now;

    (void)HAL_ADCEx_InjectedStart_IT(&hadc2);
}

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

    /* Same for the injected re-arm: 0 means "a full period has elapsed", so
     * the first adc_injected_tick() fires immediately rather than waiting a
     * pointless 100 ms before the very first measurement. */
    s_inj_last_tick       = 0U;

    /* Power-on stabilisation restarts too.  adc_init() runs once per boot
     * from app_adc_task(), so without this a soft restart (or a unit test
     * calling adc_init() to reset the module) would leave the gate sitting
     * in STAB_ARMED from the previous run and skip the arming step
     * entirely - i.e. it would key the amplifier before the protection
     * chain was live. */
    s_stab_stage          = STAB_ARMING;
    s_current_protection_fitted = false;
    s_comp_settle_ticks   = 0U;

    /* The zero-offset calibration re-arms on every boot, so the offset is
     * always measured on the unit that is actually running rather than
     * inherited from a previous one.  The detected sensor model goes
     * with it: the same window measures both, and a stored model from a
     * previous session would describe a board that may not be the one
     * fitted now. */
    g_config.im_calibrated    = 0U;
    g_config.im_sensor_type   = IM_SENSOR_UNKNOWN;

    /* RF is not permitted yet: the converters are powered but no
     * conversion has been taken, so nothing is known about the amplifier.
     * adc_stabilisation_tick() releases this once the silence is over. */
    g_allow_rf                = false;
}

/* Bring the converters up, but leave the DMA stopped.
 *
 * Called once from app_adc_task() before its first delay.  Nothing is
 * sampled yet at this point: the analog side has just been enabled and its
 * references, the CC6920 bias and the coupler detectors are all still
 * charging, so any conversion started now would capture a transient. */
void adc_presettle(void)
{
    /* Power up ADC1's voltage regulator and reference buffer without
     * starting the regular sequence.  Setting ADEN is what triggers both,
     * and HAL_ADC_Start_DMA() would otherwise spend its own first
     * conversion waiting for them - the one conversion we do not want. */
    ADC1->CR |= ADC_CR_ADVREGEN;    /* internal regulator on  */
    ADC1->CR |= ADC_CR_ADEN;        /* converter powered, not scanning */

    /* Same for ADC2, which feeds the injected NTC channels. */
    ADC2->CR |= ADC_CR_ADVREGEN;
    ADC2->CR |= ADC_CR_ADEN;
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
    /* Note: hadc2 interrupts at very high frequency, which will storm the cpu in debug mode.
     * That is why the regular-conversion DMA start below is commented out. */
//    if (HAL_ADC_Start_DMA(&hadc2,
//                          (uint32_t *)adc2_dma_buf,
//                          ADC_DMA_BUFFER_SIZE * 2U)
//        != HAL_OK) {
//        Error_Handler();
//    }
    /* Trigger injected conversion once per started interrupt cycle.
     *
     * This only fires ONE pass: MX_ADC2_Init() asks for
     * ADC_INJECTED_SOFTWARE_START with no trigger source and
     * InjectedDiscontinuousConvMode = DISABLE, so the three ranks (NTC core,
     * NTC aux, die temp) convert once and the injected state machine parks.
     * adc_injected_tick() is what keeps it going from then on.
     *
     * The _IT variant on purpose: it enables the end-of-sequence interrupt
     * that publishes the JDRs and runs the NTC over-temperature check.
     * See the adc_injected_tick() block comment for the full reasoning. */
    if (HAL_ADCEx_InjectedStart_IT(&hadc2) != HAL_OK) {
        Error_Handler();
    }
}

void adc_stop(void)
{
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    /* _IT variant: also clears ADC_IT_JEOS, so a conversion that completes
     * after the stop cannot call the injected callback any more. */
    HAL_ADCEx_InjectedStop_IT(&hadc2);
}

/*============================================================================
 *  Process DMA buffer -> g_adc_data (integer units)
 *============================================================================*/

static inline uint16_t adc1_latest(uint8_t rank)
{
    /* Last row of DMA buffer. */
    return adc1_dma_buf[(ADC_DMA_BUFFER_SIZE - 1U) * ADC1_RANK_COUNT + rank];
}

/**
 * @brief Does this ADC1 rank report the window PEAK instead of the mean?
 *
 *  The four coupler power channels do.  They are the numbers the operator
 *  reads (TX PWR / REV PWR on both couplers) and the ones the software
 *  coupler protection compares against the configured limits, and both want
 *  the energy that is actually going out - a boxcar mean over the window
 *  reports a number that no conversion ever produced.  Worse, it makes the
 *  reading lag: with a mean, a rise is smeared over the whole window, and
 *  an RF burst short enough to occupy only part of it reads far below its
 *  true level.
 *
 *  Everything else stays on the mean: the current, bias, voltage and NTC
 *  channels are slow quantities where averaging is what suppresses noise,
 *  and their peaks are either meaningless or already handled elsewhere
 *  (the RF-input overdrive indicator has its own 200 ms peak window).
 */
static bool adc1_rank_uses_peak(uint8_t rank)
{
    return (rank == ADC1_RANK_FWD1) || (rank == ADC1_RANK_REV1) ||
           (rank == ADC1_RANK_FWD2) || (rank == ADC1_RANK_REV2);
}


void adc_process(void)
{
    /* ADC1 ranks: pick the code for each rank.  Debug builds use the
     * freshest sample (the last row of the DMA buffer), release builds
     * reduce the window instead - the MEAN for the slow channels and the
     * PEAK for the four coupler power channels (see adc1_rank_uses_peak()).
     * The mean trades a few cycles of lag for a quieter number; the peak is
     * what the coupler readings and their protection are actually about.
     * Either way the result is one 16-bit code per rank, fed through
     * adc_calibration.h. */
    uint16_t raw_rank[ADC1_RANK_COUNT];
    uint8_t  r;

#if defined(DEBUG)
    for (r = 0; r < ADC1_RANK_COUNT; r++) {
        uint16_t code = adc1_dma_buf[(ADC_DMA_BUFFER_SIZE - 1U) * ADC1_RANK_COUNT + r];
        g_adc_data.adc1_raw[r] = code;
        raw_rank[r] = code;
    }
#else
    for (r = 0; r < ADC1_RANK_COUNT; r++) {
        uint16_t code;

        if (adc1_rank_uses_peak(r)) {
            /* Largest code anywhere in the window.  No division, no sum
             * overflow: one compare per row. */
            code = 0U;
            for (uint16_t row = 0U; row < ADC_DMA_BUFFER_SIZE; row++) {
                uint16_t v = adc1_dma_buf[row * ADC1_RANK_COUNT + r];
                if (v > code) {
                    code = v;
                }
            }
        } else {
            uint32_t sum = 0U;
            for (uint16_t row = 0U; row < ADC_DMA_BUFFER_SIZE; row++) {
                sum += adc1_dma_buf[row * ADC1_RANK_COUNT + r];
            }
            code = (uint16_t)(sum / ADC_DMA_BUFFER_SIZE);
        }

        g_adc_data.adc1_raw[r] = code;
        raw_rank[r] = code;
    }
#endif

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
    case STAB_ARMING:
        /* The 100 ms power-on silence is over and the DMA is running, so
         * the readings are good enough to judge the amplifier against its
         * limits.  Two things happen here, and they are deliberately
         * independent of each other:
         *
         *   1. The protection chain goes live.  The real AWD/COMP limits
         *      replace the wide-open placeholders, CLEAR_FAULTX_OC is
         *      released, and the comparators are started.  Until this
         *      point nothing could have latched a fault, which is exactly
         *      what the silence was for.
         *
         *   2. g_allow_rf goes true, which is what lets the operator key
         *      the amplifier.  This does NOT wait for the zero-offset
         *      calibration: that is a measurement step (see the block
         *      comment at the top of this file), and a current sensor
         *      that will not settle must not be able to brick the
         *      amplifier.
         *
         * NOTE that boot_apply_thresholds() reaches
         * boot_apply_comp_thresholds(), which programs the comparator
         * references from g_config.  Programming a real reference here is
         * harmless ONLY because the comparators are still disabled - they
         * are enabled one stage later, once the settling window has given
         * the analog side time to reach its real resting level.  Enabling
         * them here instead (which is what this used to do) makes COMP1
         * compare the D5/R40 peak detector's charging output against a
         * live threshold on its very first evaluation and latch
         * ALERT_OVERDRIVE at every cold boot.
         */
        boot_arm_thresholds((int)g_current_band);
        CLEAR_FAULTX_OC_LOW();
        s_stab_stage = STAB_COMPARATOR_WAIT;

        /* The digital fault lines join the protection chain here too.  The
         * power board's fault line comes up with the supply, so it can be
         * asserting (or pulse once) while the rail is still settling -
         * that is not a fault, and acting on it would mean a P0 shutdown
         * with a 10 s buzzer at every cold boot.  A line that is STILL
         * asserted at this point really is a fault, and the arm function
         * latches it, because a line that never fell produces no new edge. */
        protection_arm_digital_faults();

        /* The amplifier may be keyed from here on.  This is the only
         * writer of g_allow_rf on the power-up path; protection.c clears
         * it on a P0 fault and restores it when the operator dismisses. */
        g_allow_rf = true;

        /* Deliberately NOT starting the comparators here.  MX_COMPx_Init()
         * only configures the peripheral, and HAL_COMP_Start_IT() is what
         * makes HAL_COMP_TriggerCallback() reachable - so until the window
         * closes, the COMP1 overdrive and COMP2 over-current protections
         * are genuinely off rather than armed against a placeholder.  That
         * is the safe way round: a comparator that has never been enabled
         * cannot produce an edge, whatever its reference says. */
        s_comp_settle_ticks = 0U;
        return false;   /* not armed yet - see STAB_COMPARATOR_WAIT */

    case STAB_COMPARATOR_WAIT:
        /* The references have been sitting at the operator's real limits
         * since the arming tick, with the comparators still disabled.  Hold
         * that for a settling window so the analog side has time to reach
         * its real resting level, then enable them: their first comparison
         * is against a settled input, so a trip means a real fault.
         *
         * This is what the 1 s post-silence window in the design spec is
         * for.  Without it the amplifier either alarms at every cold boot
         * (comparators enabled too early) or has no overdrive /
         * over-current protection at all for its first second. */
        if (++s_comp_settle_ticks < STAB_COMP_SETTLE_TICKS) {
            return false;
        }
        s_comp_settle_ticks = STAB_COMP_SETTLE_TICKS;

        /* --- Enable the two comparators ---------------------------------- */
        /* Started only now, with the references already programmed and the
         * input settled.  This is the first moment at which an edge from
         * either comparator means anything.
         *
         * USE_COMP (see opa2000.h) gates the start.  With it off the
         * comparators are never armed, so COMP1 overdrive and COMP2
         * over-current are both dead.
         *
         * The comparators are ON in this build.  They were switched off
         * during bring-up to stop a power-on overdrive trip, and that
         * switch was never thrown back - which left the firmware with one
         * over-current path where the design calls for two.  The settling
         * window above is the proper fix for the original problem, and it
         * is what makes switching them back on safe. */
#ifdef USE_COMP
        if (HAL_COMP_Start_IT(&hcomp1) != HAL_OK) {
          Error_Handler();
        }
        if (HAL_COMP_Start_IT(&hcomp2) != HAL_OK) {
          Error_Handler();
        }
#endif

        /* Release the external latch clear ON this tick, not the next one.
         * The protection chain is live from here: the real limits are in the
         * watchdogs and both comparators are started, so the latch is
         * supposed to work normally again.  Leaving the pin low until the
         * next tick would keep the external fault latch held clear for one
         * more tick after the firmware started believing it was armed. */
        CLEAR_FAULTX_OC_HIGH();
        s_stab_stage = STAB_ARMED;
        return true;

    case STAB_ARMED:
    default:
        /* The zero-offset calibration runs in the BACKGROUND, here, and its
         * outcome is a measurement result - not a transmit gate.  It is
         * ticked on every pass so that a sensor which needs several
         * attempts (a cold CC6920 drifting as it warms) eventually
         * settles without anything having to be power-cycled.
         *
         * When it does succeed, the CURRENT PROTECTION has to be
         * re-fitted, because it is expressed relative to the zero
         * offset: AWD2's trip point is the offset plus the configured
         * current limit, and COMP2's reference voltage is the offset's
         * voltage plus the limit's drop.  Until the calibration succeeds
         * both are parked where they cannot trip (see
         * adc_im_protection_valid()), and this call is what brings them
         * live. */
        if (!s_current_protection_fitted && adc_im_zero_offset_tick()) {
            s_current_protection_fitted = true;
            boot_apply_thresholds((int)g_current_band);
        }
        CLEAR_FAULTX_OC_HIGH();
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

        /* Which sensor is fitted?  The same average that is about to be
         * validated as a zero offset also identifies the part: the
         * bidirectional sensor rests at 1.65 V and the unidirectional
         * one below 0.3 V, and nothing else can be mistaken for either.
         *
         * This is done HERE, in the same window that measures the zero
         * point, because the two are the same measurement: the sensor's
         * resting output IS its zero point, and which voltage that
         * resting output sits at is what tells the two families apart.
         * Doing it separately would mean a second settling wait for
         * information this window already has.
         *
         * The result is remembered in g_config.im_sensor_type so the
         * current reading, the AWD2 threshold and the GUI all agree on
         * what is fitted. */
        uint8_t sensor = adc_im_detect_sensor(avg);

        if (sensor == IM_SENSOR_UNKNOWN) {
            /* Neither signature.  A disconnected sensor, a broken one,
             * or a reference voltage the firmware does not expect all
             * land here.  Keep sampling - the offset may settle into
             * one of the two windows once the analog side is quiet.
             * The current protection stays parked meanwhile, which is
             * the safe state: an unrecognised sensor must not arm a
             * threshold we cannot place. */
            s_im_cal.retry++;
            s_im_cal.phase = 1U;
            s_im_cal.sample_idx = 0U;
            return false;
        }

        /* A recognised sensor.  The offset is accepted if it lands
         * where THAT sensor can rest - the band for the bidirectional
         * part, or the 0.3 V ceiling for the unidirectional one. */
        {
            int32_t dev = (int32_t)avg - (int32_t)IM_ZERO_EXPECTED;
            if (dev < 0) dev = -dev;
            bool plausible = (sensor == IM_SENSOR_BIDIRECTIONAL)
                             ? (dev <= IM_ZERO_TOLERANCE)
                             : ((uint32_t)avg <= IM_UNI_REST_MAX_CODE);

            if (plausible) {
                /* Success.  Note what is NOT here any more: g_allow_rf.
                 * This is a measurement step - it produces the number that
                 * makes the current reading meaningful - and it deliberately
                 * does not touch the transmit permission.  g_allow_rf is
                 * set by the power-on path when the silence ends (see
                 * adc_stabilisation_tick()) and by protection.c when a
                 * fault is dismissed. */
                g_config.im_sensor_type    = sensor;
                g_config.im_zero_offset_adc = avg;
                g_config.im_calibrated = 1U;
                s_im_cal.phase = 0U;
                return true;
            }
        }
        /* Out of tolerance: keep sampling.
         *
         * This used to count three attempts and then raise
         * ALERT_HARDWARE_FAULT, but that path was removed - a zero-offset
         * that will not settle is far more often a cold-soak / warm-up
         * artefact (the CC6920's offset drifts with temperature, and the
         * PA is drawing nothing at this point so nothing warms it) than a
         * real hardware failure.  alarming on it taught the operator to
         * ignore the alert.
         *
         * Retrying forever is safe because the current protection
         * stays parked until the fit succeeds, and nothing downstream
         * arms the real AWD/COMP limits until then - which, for the same
         * reason, this failure keeps us from reaching.  The visible
         * symptom of a never-settling offset is a current reading that
         * never appears and a main-current protection that never
         * arms - the amplifier itself still transmits normally.
         *
         * s_im_cal.retry keeps counting passes but is no longer read for
         * any decision - it is there to be watched in a debugger to see how
         * long the offset has been refusing to settle. */
        s_im_cal.retry++;
        s_im_cal.phase = 1U;
        s_im_cal.sample_idx = 0U;
        return false;
    }
    default:
        return g_config.im_calibrated != 0U;
    }
}
