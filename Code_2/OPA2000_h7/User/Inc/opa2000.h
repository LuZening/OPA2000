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
#define DEFAULT_PROTECT_REFLECT_POWER2_W     300     /* Watts */
#define DEFAULT_PROTECT_REFLECT_POWER2_SWR   200     /* Watts */
#define DEFAULT_PROTECT_SWR_THRESHOLD_X100   210     /* 2.10 */
#define DEFAULT_PROTECT_REFLECT_POWER1_W     600     /* Watts */
#define DEFAULT_PROTECT_MAIN_CURRENT_A       30      /* Amperes */
#define DEFAULT_PROTECT_NTC_TEMP_C           60      /* Celsius */
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
#define MAX_NTC_TEMP_C           70
#define MAX_TX_POWER1_W          1100
#define MAX_TX_POWER2_W          1050
#define MAX_MAIN_CURRENT_A       30

/*==============================================================================
 *  Hardware protection feature switches
 *
 *  These are COMPILE-TIME switches.  They exist because the hardware
 *  protection paths are not interchangeable, and during bring-up it is
 *  sometimes necessary to remove one of them to see what the others do.
 *
 *  Both are "defined = enabled", the same polarity as the HAL's own
 *  USE_HAL_ADC_REGISTER_CALLBACKS.  Uncomment to ENABLE; comment out to
 *  disable.  Nothing defines them on the compiler command line - they
 *  live here so that there is exactly ONE place to look.
 *
 *  The DEFAULT state of each switch matches what the firmware does today,
 *  so adding these macros changes no behaviour on its own:
 *    USE_AWD  - ENABLED  (the watchdogs have always been live)
 *    USE_COMP - ENABLED  (the comparators were switched off during
 *                         bring-up and the switch was never thrown back;
 *                         the settling window has since made it safe)
 *
 *  USE_AWD - the ADC analog watchdogs (AWD1 RF input, AWD2 main current,
 *            AWD3 coupler-1 reverse).
 *
 *    When disabled, the three HAL_ADC*LevelOutOfWindow*Callback() handlers
 *    become empty, so a watchdog trip no longer reaches
 *    protection_trigger_locked().  The thresholds are ALSO parked at full
 *    scale, which is what actually stops the hardware: an unwatched
 *    threshold would still raise the ADC interrupt on every conversion
 *    above it, and at main priority 0 that is an interrupt storm that
 *    starves everything else.
 *
 *    >>> WARNING - READ THIS BEFORE USING IT <<<
 *    The main-current over-current protection has NO software backstop.
 *    protection_check() and protection_check_adc1_dma() compare the
 *    coupler channels, SWR and VBIAS, but there is deliberately no
 *    software comparison of g_adc_data.im_mA against
 *    g_config.band_im_max_mA: the current limit is a HARDWARE limit by
 *    design, because the software path (100 ms task tick) is far too slow
 *    for a short-circuit.
 *
 *    So with USE_AWD disabled, the amplifier keeps ONE over-current path
 *    (COMP2) - but only if USE_COMP is also enabled.  Switching BOTH off
 *    leaves the amplifier with no over-current protection of any kind.
 *    If the amplifier can draw a lethal current, do not ship that
 *    combination.
 *
 * USE_COMP - the hardware comparators (COMP1 RF overdrive, COMP2
 *            over-current).
 *
 *    When disabled, HAL_COMP_Start_IT() is not called, so neither
 *    comparator's interrupt is ever armed.  The DAC references are still
 *    programmed (boot_apply_comp_thresholds()) - that costs nothing and
 *    keeps the pins in a known state - but nothing listens to the
 *    comparator outputs.
 *
 *    Note that the comparators ARE enabled in this build.  They were
 *    switched off during bring-up to stop a power-on overdrive trip, and
 *    that switch was never thrown back - which left the firmware with
 *    ONE over-current path (AWD2) where the design calls for two.  The
 *    one-second settling window in user_adc.c is the proper fix for the
 *    original problem, and it is what makes switching them back on safe.
 *
 *  Both switches are checked in the HAL callback bodies and in the
 *  threshold setters, so the firmware cannot end up in a state where the
 *  hardware is live but the software is ignoring it (or the other way
 *  round).
 *==========================================================================*/

/* The ADC analog watchdogs.  ENABLED - the watchdogs have always been
 * live in this firmware, and AWD2 is one of the two over-current
 * protection paths (COMP2 is the other).  Comment this out to take the
 * three watchdogs out of the build.
 *
 * The two-step form below is what makes that possible from a build.  A
 * bare "#ifdef USE_AWD" around the feature code cannot be switched OFF
 * from the compiler command line, because the header itself would have to
 * define the macro to make the default work - and once it is defined,
 * -UUSE_AWD on the command line has no effect (the header redefines it).
 *
 * So the header asks for a NEGATIVE switch and the feature code tests
 * the positive one:
 *      USE_COMP         - defined => the comparators are IN
 *      USE_AWD          - derived,  => the watchdogs are IN
 *
 * A build that wants the watchdogs out defines OPA2000_NO_AWD (or simply
 * comments the OPA2000_NO_AWD line out of the header).  The host test
 * suite uses exactly that to build a second copy of the watchdog setters
 * with the watchdogs switched off, and assert that the thresholds end up
 * parked where they cannot trip. */

#ifdef OPA2000_NO_AWD
/* #define USE_AWD */
#else
#define USE_AWD
#endif

#ifdef OPA2000_NO_COMP
/* #define USE_COMP */
#else
#define USE_COMP
#endif


/* The hardware comparators.  ENABLED.
 *
 * They were switched off during bring-up to stop a power-on overdrive
 * trip, and that switch was never thrown back.  The one-second settling
 * window in user_adc.c (STAB_COMPARATOR_WAIT) is exactly the fix the
 * original problem needed - the comparators are started only after the
 * input has settled and the references have been sitting at their real
 * values for a full second - so the reason for switching them off is
 * gone.
 *
 * They are also the SECOND of the two over-current paths (AWD2 is the
 * first), and the design spec calls for the two to be independent.
 */


/*==============================================================================
 *  COMP comparator references (DAC1 outputs)
 *
 *  The DAC is 12-bit and referenced to VDDA (3.3 V), NOT to the ADC's
 *  VREFBUF 2.5 V - the two references are different rails and must not be
 *  mixed.  One LSB is therefore 3300/4095 = 0.806 mV:
 *
 *      dac_code = ref_mV * 4095 / 3300
 *
 *  Both thresholds are stored as the reference VOLTAGE in mV rather than
 *  in watts / amps: the analog gain in front of each COMP input is a
 *  property of the board, calibrated on the bench, not a firmware
 *  constant.  See User/TODO-COMP1标定待办.txt.
 *==========================================================================*/
#define DAC_VDDA_MV                   3300U   /*!< DAC1 reference rail       */
#define DAC_FULL_SCALE                4095U   /*!< 12-bit maximum code       */

/*!< COMP1 ceiling.  Full scale is 3300 mV; leave headroom so a
 *    mis-typed config cannot pin the output at the rail. */
#define COMP1_REF_MV_MAX              3000U
/*!< COMP2 ceiling.  COMP2 watches the same node as ADC1_INP5, whose full
 *    scale is 2500 mV (VREFBUF), so a reference above that cannot be
 *    reached by the signal at all and 2500 is the real ceiling.
 *
 *    Note this is NOT the 30 A design limit any more.  The reference is
 *    derived from the sensor's MEASURED resting point
 *    (adc_im_comp2_ref_mV), and the tolerance band on that point spans
 *    roughly +/-190 mV, i.e. +/-7 A.  A sensor resting above 1.65 V
 *    therefore needs a reference above 1650 + 792 = 2442 mV to trip at
 *    30 A, and clamping it to 2442 would make the backstop fire
 *    EARLY - the exact failure the derivation exists to remove. */
#define COMP2_REF_MV_MAX              2500U

/*!< Ship default for COMP2: the 30 A design limit through the documented
 *    CC6920BSO-50A transfer function, V = 1650 + I * 26.4.  Used only as
 *    a stored fallback for an unrecognised sensor; on a fitted one the
 *    reference is always derived from the measured zero point. */
#define DEFAULT_COMP2_REF_MV          2442U
/*!< Ship default for COMP1: deliberately conservative (about 45 % of full
 *    scale).  The D5/R40 peak detector ahead of PB2 is uncalibrated, so
 *    this only has to sit far above the detector noise floor; the precise
 *    RF-input limit is enforced by AWD1 in firmware. 2378 = 68K + 2.2K @ 30W input */
#define DEFAULT_COMP1_REF_MV          2378U

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

#define DEFAULT_PC_BAUD_RATE      115200U
#define DEFAULT_RADIO_BAUD_RATE   115200U

/* Frequency detection (TIM1 ETR + TIM7 100us tick)
 * TIM7 ticks every 100us; counts accumulated; freq = count * 10000 */
#define FREQDET_MULTIPLIER         10000U  /*!< count * 10000 = Hz (1s/100us) */
#define FREQDET_MIN_VALID_FREQ_HZ  1000000U /*!< 1 MHz threshold  */
#define FREQDET_MAX_VALID_FREQ_HZ 60000000U /*!< 60 MHz upper bound */

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
    ALERT_FAN_FAULT,         /*!< 占位 - FAN1 tacho 暂未接线 */
} AlertType;

/* Alert severity - drives which hardware actions and which dismiss
 * behaviour protection.c takes.  Single source of truth is
 * alert_severity() in opa2000.c. */
typedef enum {
    ALERT_SEV_NONE = 0,
    ALERT_SEV_P0,            /*!< Emergency - full hard shutdown    */
    ALERT_SEV_P1,            /*!< Mild     - stop keying + alert    */
} AlertSeverity;

/* Fan speed state */
typedef enum {
    FAN_SPEED_LOW = 0,   /*!< 50 % */
    FAN_SPEED_MED,       /*!< 80 % */
    FAN_SPEED_HIGH,      /*!< 100 % */
} FanSpeedType;

/* Front-panel power state.
 *
 * This is the operator-visible three-position view, derived by power_control.c
 * from the two independent flags below.  g_psu_on and g_tx_active stay
 * separate because "supply on but not keyed" and "keyed" are genuinely
 * different conditions that the protection logic cares about individually;
 * the GUI just wants one label. */
typedef enum {
    POWER_STATE_OFF = 0,     /*!< supply disengaged                       */
    POWER_STATE_STANDBY,     /*!< supply on, PA disconnected             */
    POWER_STATE_TX,          /*!< supply on and transmitting            */
} PowerStateType;

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

    /* Multi-key resistor ladder code (ADC1_RANK_KEY_ADC), raw 16-bit. */
    volatile uint16_t key_adc;

    /* VSWR - derived from fwd2_mW and rev2_mW.
     * If fwd2_mW < 10W (10 000 mW), swr is forced to 100 (=1.0).
     * NOTE: this is the *instantaneous* value.  The GUI computes its own
     * SWR from the two peak-hold values below so that all three bar graphs
     * on screen agree with each other. */
    volatile int32_t swr_x100;           /*!< SWR * 100, instantaneous */

    /* Peak-hold (peak detector with exponential release) on the coupler the
     * operator actually sees.  Written only by adc_peak_hold_tick() in
     * user_adc.c; the GUI reads them and must never write.
     *
     * Semantics: attack is instantaneous (a new maximum replaces the held
     * value at once), release is an exponential decay with the time constant
     * PEAK_TAU_MS (user_adc.h), i.e. the held value falls to 1/e (36.8 %) of
     * its starting amplitude one tau after the input drops below it, and to
     * 5 % after three tau.  This is what makes a short power spike readable
     * on a bar graph without the needle flickering back to zero between
     * syllables.
     *
     * These are a measurement, not a function of the keying state: they
     * track the coupler unconditionally and are NOT gated on g_tx_active /
     * PTT.  If the amplifier is idle and the reading is zero, the holds
     * decay to zero on their own; if something non-zero is present while
     * idle (front-end noise, a stuck coupler, RF arriving from elsewhere)
     * that is exactly what the operator needs to see, so it is displayed
     * rather than hidden. */
    volatile int32_t fwd2_peak_mW;       /*!< coupler 2 forward, peak-hold, mW */
    volatile int32_t rev2_peak_mW;       /*!< coupler 2 reverse, peak-hold, mW */

    /* Sequence counter (increments on every fresh DMA batch) */
    volatile uint32_t seq;
} AdcDataType;

/* Touch screen transform.  Least-squares fit over CAL_POINT_COUNT samples:
 * screen = raw * gain + offset.  Floats, not fixed point - the MCU has an
 * FPU and these are only evaluated at calibration time and once per touch
 * report.  `calibrated` gates the transform: until the first calibration
 * the raw values pass through unchanged. */
typedef struct {
    float    x_gain;
    float    x_offset;
    float    y_gain;
    float    y_offset;
    uint8_t  calibrated;
    uint8_t  reserved[3];
} touch_cal_t;

/* Directional-coupler calibration, one per coupler / direction.
 *
 * The slope is kept as micro-Watt per ADC code and the intercept in
 * milli-Watt, so both are exact integers without any fixed-point
 * arithmetic:
 *
 *     power_mW = raw * gain_uW_per_code / 1000 + offset_mW
 *
 * Only the coupler calibration screen writes these; adc_calibration.c
 * uses them as a trim on top of the OPA1000 front-end model. */
typedef struct {
    int32_t  gain_uW_per_code;
    int32_t  offset_mW;
} coupler_t;

/* Persistent configuration blob.  Held in RAM as the global g_config;
 * config.c mirrors it to /fs/config.bin. */
typedef struct {
    uint32_t magic;                   /*!< CONFIG_MAGIC, see config.h       */
    uint8_t  version;                 /*!< CONFIG_VERSION, bump on layout change */
    uint8_t  reserved0[3];

    uint32_t pc_baud_rate;
    RadioType radio_type;
    uint32_t radio_baud_rate;

    /* Per-band protection thresholds (Hz / Watts / Amps / Celsius) */
    uint16_t band_rf_in_max_w[6];     /*!< AWD1 RF input threshold, by band */
    uint16_t band_rev1_max_w;         /*!< AWD3 coupler 1 reverse (PA->LPF) */
    uint16_t band_rev2_max_w;         /*!< software coupler 2 reverse       */
    uint16_t band_rev2_swr_w;         /*!< SWR-check coupler 2 reverse     */
    uint16_t band_swr_max_x100;       /*!< SWR threshold * 100             */
    uint16_t band_ntc_max_tenth_c;    /*!< NTC overheat threshold          */
    uint16_t band_fwd1_max_w;         /*!< coupler 1 forward software limit */
    uint16_t band_fwd2_max_w;         /*!< coupler 2 forward software limit */
    uint16_t band_im_max_mA;          /*!< main current threshold, milli-Amp */
    uint16_t band_vbias_max_mV;       /*!< VBIAS threshold                 */

    /* COMP comparator references - the DAC1 outputs that COMP1 / COMP2
     * compare the analog inputs against.  Stored as the reference
     * VOLTAGE in millivolts rather than in watts / amps, because the
     * DAC is 12-bit on VDDA and the analog gain in front of the inputs
     * is a hardware property the operator calibrates on the bench, not
     * a firmware constant.  GUI shows millivolts for COMP1 and also
     * the derived amps for COMP2 (CC6920BSO-50A: V = 1650 + I*26.4).
     * See User/TODO-COMP1标定待办.txt for the COMP1 calibration gap. */
    uint16_t comp1_ref_mV;            /*!< DAC1 CH1 -> COMP1_INP- (PB2)    */
    uint16_t comp2_ref_mV;            /*!< DAC1 CH2 -> COMP2_INP- (PE9)    */

    /* IM zero-offset calibration.
     *
     * im_zero_offset_adc is the measured resting code of the CC6920
     * (0 A reads 1650 mV = 43254 codes on a 2.5 V reference).  The
     * value is kept across reboots because it is useful as a starting
     * point, but it is re-measured on every boot.
     *
     * im_calibrated is a per-boot MEASUREMENT flag, not a setting: it is
     * true only after the calibration has succeeded in THIS session, and
     * it is never persisted (config_save() clears it).  It is also what
     * arms the main-current protection - AWD2's threshold and COMP2's
     * reference are both "zero offset plus the current limit", so until
     * this flag is set they are parked where they cannot trip.  See
     * adc_im_protection_valid() in adc_calibration.h. */
    uint16_t im_zero_offset_adc;
    uint8_t  im_calibrated;

    /* Touch screen calibration */
    touch_cal_t touch;

    /* Directional-coupler trims, written by the coupler calibration screen */
    coupler_t fwd1;
    coupler_t fwd2;
    coupler_t rev1;
    coupler_t rev2;

    /* Misc */
    LanguageType language;
    uint8_t  buzzer_enable;
    uint8_t  temp_fahrenheit;         /*!< display temperatures in degF     */

    /* Master protection enable.  Non-zero = protection armed (default);
     * zero = the firmware stops judging every threshold and stops taking
     * any protective action.  See protection_trigger_locked() for exactly
     * what that does and does not cover - the hardware watchdogs (AWD1/2/3),
     * the COMP comparators and the PB12 hard-fault line are NOT gated by
     * this and still act on their own.
     *
     * Carved out of the old reserved[] padding rather than appended, so the
     * struct layout and therefore the CRC coverage are unchanged and every
     * already-stored config record stays valid. */
    uint8_t  allow_protection;

    /* Which main-current sensor is fitted.  Detected automatically at
     * power-on from the sensor's own resting output, so the same firmware
     * runs on both board variants:
     *
     *   IM_SENSOR_UNKNOWN       not detected yet (or the reading was
     *                           nonsense - neither signature matched)
     *   IM_SENSOR_BIDIRECTIONAL the CC6920BSO-50A and relatives: 1.65 V
     *                           at 0 A, so the raw code rests near
     *                           43254 and a zero-offset calibration is
     *                           needed
     *   IM_SENSOR_UNIDIRECTIONAL  a one-way sensor: below 0.3 V at 0 A, so
     *                           the raw code rests near zero
     *
     * The two differ in what the resting level MEANS, not in how it is
     * measured: both are read the same way, and both need the resting
     * level known before the current can be read and before the
     * main-current protection can be armed.  See
     * adc_im_detect_sensor() in adc_calibration.h.
     *
     * Also carved out of reserved[] for the same reason as
     * allow_protection above. */
    uint8_t  im_sensor_type;

    uint8_t  reserved[3];

    /* MUST stay last: config.c computes the CRC over everything ahead of it. */
    uint32_t crc32;
} ConfigType;

/*============================================================================
 * Main-current sensor models
 *
 *  Two families are fitted to the same board and are told apart by their
 *  own resting output, which the firmware samples during the power-on
 *  zero-offset window.  See adc_im_detect_sensor() in adc_calibration.h.
 *============================================================================*/
#define IM_SENSOR_UNKNOWN        0U   /*!< not detected (or nonsense)     */
#define IM_SENSOR_BIDIRECTIONAL   1U   /*!< 1.65 V at 0 A (CC6920BSO-50A)  */
#define IM_SENSOR_UNIDIRECTIONAL  2U   /*!< below 0.3 V at 0 A (one-way)   */

/*============================================================================
 * Global state (declared; defined in opa2000.c)
 *============================================================================*/

extern volatile AdcDataType    g_adc_data;

/* Amplifier front-panel state.  These three are deliberately independent:
 * the main supply can be engaged while the HV rail is off, the HV rail can
 * be up while the amplifier is idle, and a fault can latch with everything
 * still on.
 *
 *   g_psu_on    Main supply engaged - PSU_ON_SIG (PG5) is latched.  The
 *               operator's master power switch; power_soft_off() releases
 *               it at shutdown.  Only this one and g_hv_on together mean
 *               "the PA can produce RF".
 *   g_hv_on     High-voltage rail engaged - PWR_ON_SIG (PG7) and
 *               RF_BIAS_ON (PG2) are high.  The front-panel HV button
 *               toggles this (power_hv_on / power_hv_off), independently of
 *               the relays.  The two pins are sequenced 10 ms apart, so
 *               this flag goes true one tick BEFORE the bias is up; use
 *               power_rail_busy() when the settled state matters.
 *   g_tx_active the transmit sequence is engaged: RELAY2 is closed, so the
 *               PA is still connected to the antenna side.  It stays true
 *               through the 15 ms stop delay after the pedal is released
 *               (RELAY1 is open but RELAY2 is not yet), so protection keeps
 *               running until the PA is fully disconnected.  Only
 *               meaningful while g_hv_on is true.
 *
 * Anything that means "no RF may be produced" is expressed by
 * g_allow_rf (no fault latched) or g_alert_flag (protection latched),
 * not by clearing g_hv_on. */
extern volatile bool           g_psu_on;
extern volatile bool           g_hv_on;
extern volatile bool           g_tx_active;

extern volatile BandType       g_current_band;
/*!< RF permission.  Set true once the ADC power-on silence is over, so
 *   the amplifier may be keyed; cleared by protection.c on a P0 fault
 *   and restored when the operator dismisses it.  Deliberately NOT tied
 *   to the main-current zero-offset calibration any more - that is a
 *   measurement step, and a sensor that will not settle must not be
 *   able to brick the amplifier. */
extern volatile bool           g_allow_rf;
extern volatile AlertType      g_alert_active;
extern volatile AlertSeverity  g_alert_severity;
extern volatile bool           g_alert_flag;
extern volatile FanSpeedType   g_fan_speed;
extern volatile uint32_t       g_radio_freq_hz;   /*!< last measured Hz */

/* Operator-visible power state, derived from g_psu_on / g_tx_active.
 * Written only by power_control.c; read by the GUI for its status banner. */
extern volatile PowerStateType g_power_state;

extern volatile ConfigType g_config;

/*============================================================================
 *  Boot-time flags (set by app_init_task, consumed by app_gui_task)
 *============================================================================
 *  g_calibration_required = true  -> the GUI must show the touch
 *      calibration screen before going to the main screen.  Set when
 *      config_init() reports no valid record (or the stored touch
 *      transform is the identity, see config_touch_needs_calibration()).
 *  g_touch_calibrated      = false -> touch.c has not yet finished its
 *      own calibration pass (host sample + the g_config.touch transform
 *      are still defaults).  Distinct from g_calibration_required:
 *      the former asks the GUI to PROMPT the user; the latter is the
 *      runtime "is the touch driver ready" flag.
 *  Both are volatile bool, single writer (init task) + multiple readers
 *  (GUI / comm) -- the 1-writer-N-readers rule applies; the reads
 *  may see a stale value for at most one task tick.
 *============================================================================*/
extern volatile bool g_calibration_required;
extern volatile bool g_touch_calibrated;

/*============================================================================
 *  Readiness gates (both default to "not ready" = safe)
 *============================================================================
 *  g_gui_ready  = true once the GUI has rendered a few frames, so the
 *      main screen's power numbers are actually on the glass.  The
 *      backlight is turned on at the same moment.
 *  g_ptt_ready  = true once the ADC power-on silence is over AND
 *      g_allow_rf is set (no fault latched) AND g_gui_ready.  Until
 *      then every PTT path is refused, because the ADC / COMP /
 *      protection chain is not live yet.
 *      Note what is NOT in this list: the IM zero-offset calibration.
 *      That is a measurement step, and it no longer holds the amplifier
 *      hostage - see g_allow_rf above.
 *  Both are volatile bool, single writer each, read by the monitor
 *  task and the PTT ISR path - the 1-writer-N-readers rule applies.
 *============================================================================*/
extern volatile bool g_gui_ready;
extern volatile bool g_ptt_ready;

/* FreeRTOS 致命错误标志（freertos_hooks.c 写、其他任何上下文读）。
 *   0          = 正常
 *   0xDEAD0001 = 栈溢出 (vApplicationStackOverflowHook)
 *   0xDEAD0002 = malloc 失败 (vApplicationMallocFailedHook)
 * 任务死循环前会先 __disable_irq()，所以读这个变量也建议在关中断上下文。 */
extern volatile uint32_t g_freertos_fault;

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
AlertSeverity alert_severity(AlertType a);

/* Initialization (called once at startup) */
void opa2000_init(void);

#ifdef __cplusplus
}
#endif

#endif /* OPA2000_H */
