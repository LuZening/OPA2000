/**
 * @file    test_protection.c
 * @brief   Unit tests for protection.c: the shutdown latch, the digital
 *          EXTI sources, the analog watchdog callbacks and the
 *          interrupt-context comparisons for the un-watched channels.
 *
 * @details protection.c is compiled into the test binary (see
 *          User/Tests/CMakeLists.txt).  Everything it touches is either a
 *          hw_pinmap.h macro (observed through the GPIO stub recorder) or
 *          a volatile in opa2000.h that the test sets up directly, so no
 *          real ADC / EXTI hardware is required.
 *
 *          TIM15 (the buzzer) is likewise stubbed: assertions look at
 *          pwm_started_ch_mask and ARR to prove the 3-tone loop starts and
 *          that protection_clear() silences it.
 *
 * @note    Every case starts from protection_clear() so the static latch
 *          inside protection.c is in a known state - there is no public
 *          reset otherwise, by design.
 */

#include "test_harness.h"
#include "opa2000.h"
#include "hw_pinmap.h"
#include "protection.h"
#include "power_control.h"
#include "buzzer.h"
#include "user_adc.h"
#include "stm32h7xx_hal_stub.h"

#include <stddef.h>
#include <stdbool.h>

extern TIM_HandleTypeDef htim15;

/* Recorded by comm_host_stub.c, which stands in for the CAT link's
 * alert-event publisher (see stubs/comm_host_stub.c). */
extern unsigned int       comm_stub_event_count;
extern volatile AlertSeverity comm_stub_last_severity;
extern volatile bool      comm_stub_last_on;

/*============================================================================
 *  Helpers
 *==========================================================================*/

/* HIGHERR_PWR is a fault *input*: drive it high to assert the alarm.
 * Declared before prot_establish_ready_state(), which uses it to park the
 * line at inactive. */
static void hardpwr_err_set(bool asserted);

/* Everything at rest: relays open, supply live, RF allowed, no latch. */
static void prot_establish_ready_state(void)
{
    /* A protection_clear() that had nothing latched arms no hold-off, so it
     * cannot leave one behind - but a PREVIOUS test in this binary may have
     * dismissed a real fault, and s_clear_holdoff_ms is static state inside
     * protection.c with no public reset.  protection_init() is the one that
     * clears it, so use that rather than assuming a clean slate. */
    protection_init();
    protection_clear();               /* guarantees an unlatched start */
    power_init_outputs();
    power_psu_on();                   /* main supply latch (PG5) */
    power_hv_on();                    /* HV rail: PWR_ON, bias one step later */
    (void)power_rail_step();          /* settle the bias so the rail is idle */
    /* power_hv_on() mutes the main-current protection for the inrush
     * window.  Every case here starts from a settled, already-energised
     * amplifier, so let the hold expire and release it - otherwise the
     * AWD2 / COMP2 cases would silently pass because the fault never had a
     * chance to latch. */
    test_wall_clock_advance(IM_MASK_HOLD_MS);
    protection_im_mask_tick();
    ptt_state    = PTT_STATE_IDLE;
    g_allow_rf   = true;
    g_alert_flag = false;
    g_tx_active  = false;

    /* protection.c reads every limit from g_config; opa2000_init() is not
     * called in the host build, so plant the production defaults here.
     * Keeping them in one place means a change to the defaults shows up as
     * a single edit rather than a scatter of magic numbers below. */
    g_config.band_rev2_max_w       = DEFAULT_PROTECT_REFLECT_POWER2_W;
    g_config.band_rev2_swr_w       = DEFAULT_PROTECT_REFLECT_POWER2_SWR;
    g_config.band_swr_max_x100     = DEFAULT_PROTECT_SWR_THRESHOLD_X100;
    g_config.band_ntc_max_tenth_c  = (uint16_t)(MAX_NTC_TEMP_C * 10U);
    g_config.band_fwd1_max_w       = MAX_TX_POWER1_W;
    g_config.band_fwd2_max_w       = MAX_TX_POWER2_W;
    g_config.band_vbias_max_mV     = 1300U;
    /* Master protection switch.  Every case in this file is about what
     * protection DOES, so the switch has to be armed for all of them - the
     * disarm cases below set it themselves. */
    g_config.allow_protection      = 1U;

    g_adc_data.rev2_mW          = 0;
    g_adc_data.rev1_mW          = 0;
    g_adc_data.fwd1_mW          = 0;
    g_adc_data.fwd2_mW          = 0;
    g_adc_data.im_mA            = 0;
    g_adc_data.vbias_mV         = 0;
    g_adc_data.swr_x100         = 100;
    g_adc_data.ntc_core_tenth_c = 250;   /* 25.0 C */

    /* Fault INPUTS back to inactive.  The GPIO stub keeps whatever level a
     * previous case drove, and arm_analog_chain() below ends the settling
     * window - which calls protection_arm_digital_faults(), which re-reads
     * PB12 and latches a HARDWARE_FAULT if it is still high.  A case that
     * drove the line and did not clear it would therefore leak into the
     * next one, which is exactly the kind of cross-test coupling that makes
     * a suite untrustworthy. */
    hardpwr_err_set(false);

    stub_gpio_reset_counters();
    buzzer_init();
}

/* HIGHERR_PWR is a fault *input*: drive it high to assert the alarm. */
static void hardpwr_err_set(bool asserted)
{
    HAL_GPIO_WritePin(HARDPWR_ERR_GPIO_Port, HARDPWR_ERR_Pin,
                      asserted ? GPIO_PIN_SET_E : GPIO_PIN_RESET_E);
}

static bool buzzer_sounding(void)
{
    return (htim15.pwm_started_ch_mask & (int)BUZZER_TIM_CHANNEL) != 0;
}

/*============================================================================
 *  COMP interrupt guards - the bring-up window
 *
 *  protection_comp1_isr() / protection_comp2_isr() fire straight from the
 *  comparator IRQ with no software gate in between, so the only thing that
 *  keeps a power-up transient from latching a P0 shutdown is the state of
 *  the stabilisation machine.
 *
 *  adc_stabilisation_tick() now keeps both comparators DISABLED until the
 *  analog side has settled, so in practice no edge can arrive during the
 *  window.  These guards are the belt to that braces: if a comparator were
 *  enabled early - by a future edit, or by a device whose COMP enable bit
 *  came up set - the ISR still refuses to latch.
 *==========================================================================*/

/* Drive the stabilisation machine past the comparator settling window so
 * adc_stabilisation_done() is true. */
static void arm_analog_chain(void)
{
    for (int i = 0; i < 64 && !adc_stabilisation_done(); i++) {
        (void)adc_stabilisation_tick();
    }
}

TEST_CASE(comp_isrs_ignore_edges_while_the_chain_is_still_arming)
{
    prot_establish_ready_state();

    /* adc_init() puts the machine back to STAB_ARMING. */
    adc_init();
    ASSERT_FALSE(adc_stabilisation_done());

    protection_comp1_isr();
    protection_comp2_isr();

    /* Neither latched: no alert and no buzzer.  (g_allow_rf is false here
     * because adc_init() blocks RF until the chain arms - that is the boot
     * gate, not a fault.) */
    ASSERT_FALSE(protection_is_active());
    ASSERT_FALSE(g_alert_flag);
    ASSERT_EQ(g_alert_active, ALERT_NONE);
    ASSERT_FALSE(buzzer_is_active());
    ASSERT_TRUE(adc_im_protection_valid() == false);   /* still uncalibrated */
}

TEST_CASE(comp1_latches_overdrive_once_the_chain_is_armed)
{
    prot_establish_ready_state();
    adc_init();
    arm_analog_chain();
    ASSERT_TRUE(adc_stabilisation_done());

    protection_comp1_isr();

    ASSERT_TRUE(protection_is_active());
    ASSERT_EQ(g_alert_active, ALERT_OVERDRIVE);
    ASSERT_EQ(g_alert_severity, ALERT_SEV_P0);
    ASSERT_FALSE(g_allow_rf);
}

TEST_CASE(comp2_latches_current_high_once_the_chain_is_armed)
{
    prot_establish_ready_state();
    adc_init();
    arm_analog_chain();
    ASSERT_TRUE(adc_stabilisation_done());

    protection_comp2_isr();

    ASSERT_TRUE(protection_is_active());
    ASSERT_EQ(g_alert_active, ALERT_CURRENT_HIGH);
    ASSERT_EQ(g_alert_severity, ALERT_SEV_P0);
    ASSERT_FALSE(g_allow_rf);
}

/*============================================================================
 *  A P0 fault must leave the panel out of Transmit
 *
 *  g_power_state is what the top banner reads.  It is only published by
 *  ptt_leave_tx(), which the protection path used to skip by assigning
 *  ptt_state directly - so after a fault the amplifier was dead but the
 *  banner still said "Transmit" and never recovered.
 *==========================================================================*/

static uint8_t ptt_out_level(void)
{
    return stub_gpio_get_last(PTT_OUT_GPIO_Port, PTT_OUT_Pin);
}

TEST_CASE(a_p0_fault_takes_the_panel_out_of_transmit)
{
    prot_establish_ready_state();

    /* Mid-transmit. */
    power_relays_tx_on();
    (void)power_relay_timer_callback();      /* -> TX_RUNNING */
    ASSERT_EQ(ptt_state, PTT_STATE_TX_RUNNING);
    ASSERT_EQ(g_power_state, POWER_STATE_TX);
    ASSERT_TRUE(g_tx_active);
    ASSERT_EQ(ptt_out_level(), 1);

    protection_trigger_locked(ALERT_CURRENT_HIGH);

    /* Relays open, keying feedback low, and - the part that used to be
     * missed - the published power state no longer says TX. */
    ASSERT_EQ(ptt_state, PTT_STATE_IDLE);
    ASSERT_FALSE(g_tx_active);
    ASSERT_NE(g_power_state, POWER_STATE_TX);
    ASSERT_EQ(ptt_out_level(), 0);
}

TEST_CASE(a_p0_fault_still_publishes_the_state_when_already_idle)
{
    prot_establish_ready_state();

    /* The state machine has already landed in IDLE, but the published state
     * is stale at TX.  This is the exact shape the old direct assignment
     * produced, and it has to self-heal on the next fault rather than leave
     * the banner stuck forever. */
    ptt_state     = PTT_STATE_IDLE;
    g_tx_active   = true;
    g_power_state = POWER_STATE_TX;

    protection_trigger_locked(ALERT_CURRENT_HIGH);

    ASSERT_FALSE(g_tx_active);
    ASSERT_NE(g_power_state, POWER_STATE_TX);
    ASSERT_EQ(ptt_out_level(), 0);
}

/*============================================================================
 *  protection_init() - power-on state
 *============================================================================*/

TEST_CASE(a_dismissed_buzzer_reprograms_its_note_on_the_next_alert)
{
    prot_establish_ready_state();

    protection_trigger_locked(ALERT_HARDWARE_FAULT);
    uint16_t first_note_arr = htim15.Instance->ARR;
    ASSERT_TRUE(first_note_arr > 0U);
    ASSERT_TRUE(buzzer_sounding());

    /* Dismissing silences the buzzer and clears ARR. */
    protection_clear();
    ASSERT_FALSE(buzzer_is_active());
    ASSERT_EQ(htim15.Instance->ARR, 0);

    /* A second, DIFFERENT fault (so the dismiss hold-off does not apply)
     * has to re-program the note.  buzzer_set_freq() now caches the
     * programmed note to avoid 10 000 HAL Start/Stop calls a second, and a
     * cache that could not be cleared would leave this alarm silent - which
     * is exactly the failure this pins. */
    protection_trigger_locked(ALERT_CURRENT_HIGH);
    ASSERT_TRUE(buzzer_is_active());
    ASSERT_TRUE(buzzer_sounding());
    ASSERT_EQ(htim15.Instance->ARR, first_note_arr);
}

TEST_CASE(init_leaves_every_output_disarmed)
{
    prot_establish_ready_state();
    protection_trigger_locked(ALERT_CURRENT_HIGH);
    ASSERT_TRUE(protection_is_active());

    protection_init();

    ASSERT_FALSE(protection_is_active());
    ASSERT_EQ(g_alert_active, ALERT_NONE);
    ASSERT_FALSE(g_alert_flag);
    ASSERT_EQ(stub_gpio_get_last(RF_IN_PROTECT_GPIO_Port, RF_IN_PROTECT_Pin),
              GPIO_PIN_RESET_E);
    ASSERT_FALSE(buzzer_is_active());
}

/*============================================================================
 *  protection_trigger_locked() - the single shutdown path
 *============================================================================*/

TEST_CASE(trigger_drops_everything_and_raises_rf_protect)
{
    prot_establish_ready_state();
    /* Pretend the operator keyed up: both relays closed, PTT_OUT high. */
    power_relays_tx_on();
    power_ptt_out_set(true);
    ptt_state = PTT_STATE_TX_RUNNING;
    g_tx_active = true;
    stub_gpio_reset_counters();

    protection_trigger_locked(ALERT_CURRENT_HIGH);

    ASSERT_EQ(stub_gpio_get_last(RELAY1_GPIO_Port, RELAY1_Pin), GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(RELAY2_GPIO_Port, RELAY2_Pin), GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(PWR_ON_GPIO_Port, PWR_ON_Pin),  GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(RF_BIAS_GPIO_Port, RF_BIAS_Pin), GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(PTT_OUT_GPIO_Port, PTT_OUT_Pin), GPIO_PIN_RESET_E);
    /* The RF board is told to run its own protection. */
    ASSERT_EQ(stub_gpio_get_last(RF_IN_PROTECT_GPIO_Port, RF_IN_PROTECT_Pin),
              GPIO_PIN_SET_E);
    ASSERT_EQ(ptt_state, PTT_STATE_IDLE);
}

TEST_CASE(trigger_publishes_alert_and_blocks_ptt)
{
    prot_establish_ready_state();

    protection_trigger_locked(ALERT_SWR_HIGH);

    ASSERT_TRUE(protection_is_active());
    ASSERT_EQ(g_alert_active, ALERT_SWR_HIGH);
    ASSERT_TRUE(g_alert_flag);
    /* PTT is blocked from now on: a task-initiated TX must be refused. */
    ASSERT_FALSE(g_allow_rf);
    ASSERT_FALSE(g_tx_active);
    power_tx_enable();
    ASSERT_EQ(ptt_state, PTT_STATE_IDLE);
    ASSERT_EQ(stub_gpio_get_last(RELAY2_GPIO_Port, RELAY2_Pin), GPIO_PIN_RESET_E);
}

TEST_CASE(trigger_starts_the_3_tone_buzzer)
{
    prot_establish_ready_state();

    protection_trigger_locked(ALERT_HARDWARE_FAULT);

    ASSERT_TRUE(buzzer_is_active());
    ASSERT_TRUE(buzzer_sounding());
    /* Note 1 of the warning is the high tone: ARR = 15.625 MHz / 2.5 kHz */
    ASSERT_EQ(htim15.Instance->ARR, BUZZER_TEST_TIM_CLOCK_HZ / BUZZER_NOTE_HIGH_HZ);
}

TEST_CASE(trigger_is_latched_first_fault_wins)
{
    prot_establish_ready_state();

    protection_trigger_locked(ALERT_SWR_HIGH);
    /* A second, different source fires while already latched: the cause
     * must not be overwritten and the shutdown must not be re-run. */
    stub_gpio_reset_counters();
    protection_trigger_locked(ALERT_NTC_OVERHEAT);

    ASSERT_EQ(g_alert_active, ALERT_SWR_HIGH);
    ASSERT_EQ(stub_gpio_set_count(RELAY1_GPIO_Port, RELAY1_Pin), 0U);
    ASSERT_EQ(stub_gpio_set_count(RF_IN_PROTECT_GPIO_Port, RF_IN_PROTECT_Pin), 0U);
}

/*============================================================================
 *  Digital sources - EXTI interrupt context
 *============================================================================*/

TEST_CASE(hardpwr_err_active_high_trips)
{
    prot_establish_ready_state();
    hardpwr_err_set(true);

    protection_hardpwr_err_isr();

    ASSERT_TRUE(protection_is_active());
    ASSERT_EQ(g_alert_active, ALERT_HARDWARE_FAULT);
}

TEST_CASE(hardpwr_err_active_low_edge_is_ignored)
{
    prot_establish_ready_state();
    /* Spurious / noise edge: the line reads inactive when the ISR runs. */
    hardpwr_err_set(false);

    protection_hardpwr_err_isr();

    ASSERT_FALSE(protection_is_active());
    ASSERT_FALSE(g_alert_flag);
}

TEST_CASE(hardpwr_err_level_stays_high_only_triggers_once)
{
    prot_establish_ready_state();
    hardpwr_err_set(true);

    protection_hardpwr_err_isr();
    stub_gpio_reset_counters();
    protection_hardpwr_err_isr();

    ASSERT_TRUE(protection_is_active());
    ASSERT_EQ(stub_gpio_set_count(RF_IN_PROTECT_GPIO_Port, RF_IN_PROTECT_Pin), 0U);
}

TEST_CASE(digital_isr_is_the_generic_rf_deck_entry)
{
    prot_establish_ready_state();

    protection_digital_isr(ALERT_FREQ_MISMATCH);

    ASSERT_TRUE(protection_is_active());
    ASSERT_EQ(g_alert_active, ALERT_FREQ_MISMATCH);
    ASSERT_EQ(stub_gpio_get_last(RF_IN_PROTECT_GPIO_Port, RF_IN_PROTECT_Pin),
              GPIO_PIN_SET_E);
}

/*============================================================================
 *  Analog sources with a hardware watchdog
 *============================================================================*/

TEST_CASE(awd_callbacks_map_to_their_fault_codes)
{
    prot_establish_ready_state();
    protection_awd1_cb();                 /* RF input overdrive */
    ASSERT_EQ(g_alert_active, ALERT_OVERDRIVE);
    protection_clear();

    protection_awd2_cb();                 /* main current       */
    ASSERT_EQ(g_alert_active, ALERT_CURRENT_HIGH);
    protection_clear();

    protection_awd3_cb();                 /* coupler 1 reverse */
    ASSERT_EQ(g_alert_active, ALERT_REFLECT_POWER_HIGH);
}

TEST_CASE(awd_shutdown_does_not_wait_for_the_task)
{
    prot_establish_ready_state();
    ptt_state   = PTT_STATE_TX_RUNNING;
    g_tx_active = true;
    power_relays_tx_on();
    power_ptt_out_set(true);

    /* Runs from the ADC IRQ, so everything the relay/power pins say must
     * already be true when this returns. */
    protection_awd2_cb();

    ASSERT_EQ(stub_gpio_get_last(RELAY1_GPIO_Port, RELAY1_Pin), GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(RELAY2_GPIO_Port, RELAY2_Pin), GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(PTT_OUT_GPIO_Port, PTT_OUT_Pin), GPIO_PIN_RESET_E);
    ASSERT_EQ(g_alert_active, ALERT_CURRENT_HIGH);
}

/*============================================================================
 *  Analog sources without a watchdog - ADC1 DMA-completion context
 *============================================================================*/

TEST_CASE(dma_rev2_over_hard_limit_trips)
{
    prot_establish_ready_state();
    g_tx_active       = true;
    g_adc_data.rev2_mW = 400001;   /* limit is 400 W, exclusive */

    protection_check_adc1_dma();

    ASSERT_TRUE(protection_is_active());
    ASSERT_EQ(g_alert_active, ALERT_REFLECT_POWER_HIGH);
}

TEST_CASE(dma_rev2_at_limit_does_not_trip)
{
    prot_establish_ready_state();
    g_tx_active        = true;
    /* Exactly at the limit, expressed from the constant rather than as a
     * literal 400000 W: DEFAULT_PROTECT_REFLECT_POWER2_W is a production
     * value that can move, and a hard-coded one stops testing the
     * boundary as soon as it does. */
    g_adc_data.rev2_mW = (int32_t)DEFAULT_PROTECT_REFLECT_POWER2_W * 1000;

    protection_check_adc1_dma();

    ASSERT_FALSE(protection_is_active());
}

TEST_CASE(dma_swr_rule_needs_both_reverse_and_swr)
{
    prot_establish_ready_state();
    g_tx_active = true;

    /* Reverse above 200 W but SWR fine -> no trip. */
    g_adc_data.rev2_mW  = 250000;
    g_adc_data.swr_x100 = 180;
    protection_check_adc1_dma();
    ASSERT_FALSE(protection_is_active());

    /* SWR above 2.0 but reverse below 200 W -> no trip. */
    g_adc_data.rev2_mW  = 150000;
    g_adc_data.swr_x100 = 250;
    protection_check_adc1_dma();
    ASSERT_FALSE(protection_is_active());

    /* Both -> trip as SWR, not as raw reverse power. */
    g_adc_data.rev2_mW  = 250000;
    g_adc_data.swr_x100 = 250;
    protection_check_adc1_dma();
    ASSERT_TRUE(protection_is_active());
    ASSERT_EQ(g_alert_active, ALERT_SWR_HIGH);
}

TEST_CASE(dma_forward_limits_trip)
{
    prot_establish_ready_state();
    g_tx_active         = true;
    g_adc_data.fwd1_mW = 1100001;   /* coupler 1 limit 1100 W */
    protection_check_adc1_dma();
    ASSERT_TRUE(protection_is_active());
    /* A dismiss holds the SAME cause off briefly (see
     * PROTECTION_REARM_HOLD_MS) so the popup really goes away - step past
     * that window before expecting the next fault to latch. */
    protection_clear();
    test_wall_clock_advance(1000U);

    g_tx_active         = true;
    g_adc_data.fwd1_mW = 0;
    g_adc_data.fwd2_mW = 1050001;   /* coupler 2 limit 1050 W */
    protection_check_adc1_dma();
    ASSERT_TRUE(protection_is_active());
}

TEST_CASE(dma_forward_at_limit_does_not_trip)
{
    prot_establish_ready_state();
    g_tx_active         = true;
    g_adc_data.fwd1_mW = 1100000;
    g_adc_data.fwd2_mW = 1050000;

    protection_check_adc1_dma();

    ASSERT_FALSE(protection_is_active());
}

TEST_CASE(dma_checks_run_even_when_not_keyed)
{
    prot_establish_ready_state();
    g_tx_active         = false;   /* idle, and it must not matter */
    g_adc_data.rev2_mW  = 900000;
    g_adc_data.fwd1_mW  = 900000;
    g_adc_data.fwd2_mW  = 900000;

    /* An idle amplifier sitting on a stray carrier, a wrongly connected
     * antenna or a failing LPF produces exactly these readings, and there
     * is no legitimate signal in flight to blame them on.  So the limits
     * apply in the idle state too. */
    protection_check_adc1_dma();

    ASSERT_TRUE(protection_is_active());
    ASSERT_EQ(g_alert_active, ALERT_REFLECT_POWER_HIGH);
}

TEST_CASE(dma_check_is_skipped_once_latched)
{
    prot_establish_ready_state();
    g_tx_active = true;
    protection_trigger_locked(ALERT_HARDWARE_FAULT);
    stub_gpio_reset_counters();

    /* Fresh garbage on every coupler channel must not re-run anything. */
    g_adc_data.rev2_mW = 900000;
    g_adc_data.fwd1_mW = 900000;
    protection_check_adc1_dma();

    ASSERT_EQ(g_alert_active, ALERT_HARDWARE_FAULT);
    ASSERT_EQ(stub_gpio_set_count(RELAY1_GPIO_Port, RELAY1_Pin), 0U);
}

/*============================================================================
 *  NTC - ADC2 injected-completion context, runs whether keyed or not
 *============================================================================*/

TEST_CASE(ntc_overheat_trips_even_when_idle)
{
    prot_establish_ready_state();
    g_tx_active          = false;
    /* One 0.1 C step above the limit: the smallest reading that must trip. */
    g_adc_data.ntc_core_tenth_c = (int32_t)(MAX_NTC_TEMP_C * 10) + 1;

    protection_check_ntc();

    ASSERT_TRUE(protection_is_active());
    ASSERT_EQ(g_alert_active, ALERT_NTC_OVERHEAT);
}

TEST_CASE(ntc_at_limit_does_not_trip)
{
    prot_establish_ready_state();
    /* Exactly at the limit. The comparison is strict, so this must NOT
     * trip—that is what makes the threshold "over N C" and not "N C or
     * above". */
    g_adc_data.ntc_core_tenth_c = (int32_t)(MAX_NTC_TEMP_C * 10);

    protection_check_ntc();

    ASSERT_FALSE(protection_is_active());
}

TEST_CASE(ntc_check_ignored_once_latched)
{
    prot_establish_ready_state();
    protection_trigger_locked(ALERT_SWR_HIGH);

    g_adc_data.ntc_core_tenth_c = 1200;
    protection_check_ntc();

    ASSERT_EQ(g_alert_active, ALERT_SWR_HIGH);
}

/*============================================================================
 *  protection_check() - 100 ms task backstop
 *============================================================================*/

TEST_CASE(backstop_catches_overheat_before_the_keyed_limits)
{
    prot_establish_ready_state();
    g_tx_active          = true;
    g_adc_data.ntc_core_tenth_c = 950;
    g_adc_data.rev2_mW   = 900000;   /* would trip on its own */

    protection_check();

    /* Temperature is reported as the cause: it is the condition that will
     * not clear by itself, so the operator must power-cycle. */
    ASSERT_EQ(g_alert_active, ALERT_NTC_OVERHEAT);
}

TEST_CASE(backstop_delegates_coupler_limits_to_the_dma_rules)
{
    prot_establish_ready_state();
    g_tx_active         = true;
    g_adc_data.fwd1_mW = 1200000;

    protection_check();

    ASSERT_TRUE(protection_is_active());
}

TEST_CASE(backstop_checks_vbias_whether_keyed_or_not)
{
    prot_establish_ready_state();
    g_tx_active        = true;
    g_adc_data.vbias_mV = 1400;      /* limit 1300 mV */

    protection_check();
    ASSERT_EQ(g_alert_active, ALERT_VBIAS_FAULT);
    /* Past the post-dismiss hold-off before the next one is expected. */
    protection_clear();
    test_wall_clock_advance(1000U);

    /* Idle: an out-of-limit bias is just as much a hardware fault when
     * there is no RF in flight to explain it, so the same limit applies. */
    g_tx_active        = false;
    g_adc_data.vbias_mV = 1400;
    protection_check();
    ASSERT_TRUE(protection_is_active());
    ASSERT_EQ(g_alert_active, ALERT_VBIAS_FAULT);
}

TEST_CASE(backstop_does_nothing_when_healthy)
{
    prot_establish_ready_state();
    g_tx_active          = true;
    g_adc_data.fwd1_mW   = 700000;
    g_adc_data.rev2_mW   = 50000;
    g_adc_data.swr_x100  = 130;
    g_adc_data.vbias_mV  = 1200;
    g_adc_data.ntc_core_tenth_c = 400;

    protection_check();

    ASSERT_FALSE(protection_is_active());
    ASSERT_FALSE(g_alert_flag);
    ASSERT_FALSE(buzzer_is_active());
}

/*============================================================================
 *  protection_clear() - the GUI / CAT / SPI6 ack path
 *============================================================================*/

TEST_CASE(clear_releases_the_latch_pin_and_buzzer)
{
    prot_establish_ready_state();
    protection_trigger_locked(ALERT_CURRENT_HIGH);
    ASSERT_TRUE(protection_is_active());

    protection_clear();

    ASSERT_FALSE(protection_is_active());
    ASSERT_EQ(g_alert_active, ALERT_NONE);
    ASSERT_FALSE(g_alert_flag);
    ASSERT_EQ(stub_gpio_get_last(RF_IN_PROTECT_GPIO_Port, RF_IN_PROTECT_Pin),
              GPIO_PIN_RESET_E);
    ASSERT_FALSE(buzzer_is_active());
    ASSERT_FALSE(buzzer_sounding());
    /* Dismiss re-arms RF: the CC6920 zero offset is unaffected by the
     * fault, so the original calibration stays valid. */
    ASSERT_TRUE(g_allow_rf);
}

TEST_CASE(clear_without_a_fault_is_harmless)
{
    prot_establish_ready_state();

    protection_clear();

    ASSERT_FALSE(protection_is_active());
    ASSERT_FALSE(buzzer_is_active());
}

/*============================================================================
 *  Severity-driven behaviour (P0 = emergency, P1 = mild)
 *==========================================================================*/

TEST_CASE(severity_mapping_classifies_each_cause)
{
    ASSERT_EQ(alert_severity(ALERT_SWR_HIGH),            ALERT_SEV_P0);
    ASSERT_EQ(alert_severity(ALERT_REFLECT_POWER_HIGH),  ALERT_SEV_P0);
    ASSERT_EQ(alert_severity(ALERT_CURRENT_HIGH),        ALERT_SEV_P0);
    ASSERT_EQ(alert_severity(ALERT_HARDWARE_FAULT),      ALERT_SEV_P0);
    ASSERT_EQ(alert_severity(ALERT_OVERDRIVE),           ALERT_SEV_P0);
    ASSERT_EQ(alert_severity(ALERT_FREQ_MISMATCH),       ALERT_SEV_P0);
    ASSERT_EQ(alert_severity(ALERT_VBIAS_FAULT),         ALERT_SEV_P0);
    ASSERT_EQ(alert_severity(ALERT_NTC_OVERHEAT),        ALERT_SEV_P1);
    ASSERT_EQ(alert_severity(ALERT_FAN_FAULT),           ALERT_SEV_P1);
    ASSERT_EQ(alert_severity(ALERT_NONE),                ALERT_SEV_NONE);
}

TEST_CASE(p1_thermal_fault_leaves_rail_up_and_only_blocks_ptt)
{
    prot_establish_ready_state();
    stub_gpio_reset_counters();
    unsigned int ev0 = comm_stub_event_count;

    protection_trigger_locked(ALERT_NTC_OVERHEAT);

    ASSERT_TRUE(protection_is_active());
    ASSERT_EQ(g_alert_active, ALERT_NTC_OVERHEAT);
    ASSERT_EQ(g_alert_severity, ALERT_SEV_P1);
    /* The CAT link must learn about the raise immediately, not on the next
     * 100 ms status poll. */
    ASSERT_EQ(comm_stub_event_count, ev0 + 1U);
    ASSERT_TRUE(comm_stub_last_on);
    ASSERT_EQ(comm_stub_last_severity, ALERT_SEV_P1);
    /* P1 must NOT drive the RF board protection bus - the PA temperature
     * is a local concern, no need to latch the RF deck. */
    ASSERT_EQ(stub_gpio_get_last(RF_IN_PROTECT_GPIO_Port, RF_IN_PROTECT_Pin),
              GPIO_PIN_RESET_E);
    /* P1 must NOT clobber g_allow_rf - the IMain zero offset is unaffected
     * by PA temperature. */
    ASSERT_TRUE(g_allow_rf);
    /* P1 must NOT cut PWR_ON or RF_BIAS - the PA stays biased and ready
     * to go the moment the operator dismisses.  prot_establish_ready_state
     * already engaged the rail (PG5/PG7/PG2), so the last-written level
     * for these pins is SET. */
    ASSERT_EQ(stub_gpio_get_last(PWR_ON_GPIO_Port, PWR_ON_Pin), GPIO_PIN_SET_E);
    ASSERT_EQ(stub_gpio_get_last(RF_BIAS_GPIO_Port, RF_BIAS_Pin), GPIO_PIN_SET_E);
    /* P1 drops PTT_OUT (the only thing it does, alongside clearing
     * g_tx_active below). */
    ASSERT_EQ(stub_gpio_get_last(PTT_OUT_GPIO_Port, PTT_OUT_Pin), GPIO_PIN_RESET_E);
    /* Buzzer started (P1 = single mid-tone, 1 s). */
    ASSERT_TRUE(buzzer_is_active());
    /* The g_alert_flag gate is what actually keeps the operator from
     * keying again while the alert is up. */
    ASSERT_TRUE(g_alert_flag);
    ASSERT_FALSE(g_tx_active);
}

TEST_CASE(p1_dismiss_does_not_touch_protect_pin)
{
    prot_establish_ready_state();
    unsigned int ev0 = comm_stub_event_count;
    protection_trigger_locked(ALERT_NTC_OVERHEAT);
    /* Drive the pin high by hand to simulate the surrounding hardware:
     * P1 path never touched it, so dismiss must not clear it. */
    HAL_GPIO_WritePin(RF_IN_PROTECT_GPIO_Port, RF_IN_PROTECT_Pin,
                      GPIO_PIN_SET_E);

    protection_clear();

    /* Pin still set - P1 dismiss did not toggle it. */
    ASSERT_EQ(stub_gpio_get_last(RF_IN_PROTECT_GPIO_Port, RF_IN_PROTECT_Pin),
              GPIO_PIN_SET_E);
    /* But the latch state is released. */
    ASSERT_FALSE(protection_is_active());
    ASSERT_FALSE(buzzer_is_active());
    /* And the CAT link got the matching "cleared" event. */
    ASSERT_EQ(comm_stub_event_count, ev0 + 2U);   /* raise + clear */
    ASSERT_FALSE(comm_stub_last_on);
}

TEST_CASE(buzzer_stops_after_10s_even_undismissed)
{
    prot_establish_ready_state();
    /* The default buzzer_start() path caps at 10 s.  Manually advance the
     * wall clock past the deadline and tick. */
    protection_trigger_locked(ALERT_CURRENT_HIGH);
    ASSERT_TRUE(buzzer_is_active());
    /* Simulate the deadline being reached by pushing it into the past:
     * the host test cannot wait 10 s. */
    buzzer_test_expire_deadline();
    buzzer_tick();
    ASSERT_FALSE(buzzer_is_active());
    ASSERT_FALSE(buzzer_sounding());
}

/*============================================================================
 *  Buzzer re-trigger guard
 *  Note: protection_trigger_locked() itself short-circuits on
 *  s_protection_active, so the "second trigger while latched" path is
 *  exercised directly against the buzzer API.
 *==========================================================================*/

TEST_CASE(buzzer_repeat_p0_does_not_reset_the_deadline)
{
    prot_establish_ready_state();
    buzzer_stop();                             /* drop any leftover state */
    buzzer_start();
    ASSERT_TRUE(buzzer_is_active());

    uint32_t first_deadline = buzzer_test_get_deadline();
    /* Pretend 3 s have passed: deadline is now first_deadline - 3000. */
    buzzer_test_set_deadline(first_deadline - 3000U);

    /* Second P0 call while the buzzer is already running.  Must NOT reset
     * the deadline back to "now + 10 s" - that would let a chatter source
     * keep the buzzer alive past 10 s forever. */
    buzzer_start();

    ASSERT_TRUE(buzzer_is_active());
    ASSERT_EQ(buzzer_test_get_deadline(), first_deadline - 3000U);
}

TEST_CASE(buzzer_repeat_p1_is_suppressed_while_p0_sounding)
{
    prot_establish_ready_state();
    buzzer_stop();                             /* drop any leftover state */
    buzzer_start();                            /* P0 owns the buzzer */
    ASSERT_TRUE(buzzer_is_active());
    extern TIM_HandleTypeDef htim15;
    fprintf(stderr, "[trace] buzzer_start: ARR=%u se=%u s_deadline=%u\n",
            htim15.Instance->ARR, (unsigned)comm_stub_last_severity, buzzer_test_get_deadline());
    /* P0 starts on the high note; capture ARR as the "P0 fingerprint". */
    uint16_t p0_arr = htim15.Instance->ARR;
    fprintf(stderr, "[trace] p0_arr=%u (expected %u)\n",
            p0_arr, BUZZER_TEST_TIM_CLOCK_HZ / BUZZER_NOTE_HIGH_HZ);
    ASSERT_EQ(p0_arr, BUZZER_TEST_TIM_CLOCK_HZ / BUZZER_NOTE_HIGH_HZ);

    /* P1 trying to come in while P0 owns the buzzer must NOT downgrade
     * the loop to a 1 s single tone. */
    buzzer_start_p1();

    ASSERT_TRUE(buzzer_is_active());
    ASSERT_EQ(htim15.Instance->ARR, p0_arr);
}

TEST_CASE(buzzer_p1_upgrades_to_p0_when_p0_fires)
{
    prot_establish_ready_state();
    buzzer_stop();                             /* drop any leftover state */
    buzzer_start_p1();                         /* P1 owns the buzzer */
    ASSERT_TRUE(buzzer_is_active());
    extern TIM_HandleTypeDef htim15;
    /* P1 is the single mid-tone. */
    ASSERT_EQ(htim15.Instance->ARR,
              BUZZER_TEST_TIM_CLOCK_HZ / BUZZER_NOTE_MID_HZ);

    /* P0 fires while P1 is sounding - the operator gets the full P0
     * 3-tone loop, starting on the high note. */
    buzzer_start();

    ASSERT_TRUE(buzzer_is_active());
    ASSERT_EQ(htim15.Instance->ARR,
              BUZZER_TEST_TIM_CLOCK_HZ / BUZZER_NOTE_HIGH_HZ);
}

/*============================================================================
 *  Master protection switch (g_config.allow_protection == 0)
 *
 *  Disarming the switch must silence EVERYTHING the firmware would do about
 *  a fault: the shutdown sequence, the buzzer, the latch that drives the
 *  GUI popup, and the alert= event on the CAT link.  Each source is driven
 *  with a genuine fault and asserted to leave no trace.
 *==========================================================================*/

TEST_CASE(disarmed_protection_ignores_a_direct_trigger)
{
    prot_establish_ready_state();
    g_config.allow_protection = 0U;

    protection_trigger_locked(ALERT_CURRENT_HIGH);

    ASSERT_FALSE(protection_is_active());
    ASSERT_EQ(g_alert_active, ALERT_NONE);
    /* g_alert_flag is the GUI task's cue to call gui_show_alert(); never
     * setting it is what suppresses the popup. */
    ASSERT_FALSE(g_alert_flag);
    /* Still true: the P0 branch that clears it never ran. */
    ASSERT_TRUE(g_allow_rf);
}

TEST_CASE(disarmed_protection_sounds_no_buzzer)
{
    prot_establish_ready_state();
    g_config.allow_protection = 0U;
    buzzer_stop();

    /* P0 gets the 3-tone, P1 the single mid-tone - neither may start. */
    protection_trigger_locked(ALERT_CURRENT_HIGH);
    ASSERT_FALSE(buzzer_is_active());

    buzzer_stop();
    protection_clear();
    protection_trigger_locked(ALERT_NTC_OVERHEAT);    /* P1 */
    ASSERT_FALSE(buzzer_is_active());
}

TEST_CASE(armed_protection_does_sound_the_buzzer)
{
    /* The counterpart to the case above: proves the buzzer assertions are
     * actually looking at something, so "no tone when disarmed" is not
     * passing simply because the helper is broken. */
    prot_establish_ready_state();
    ASSERT_EQ(g_config.allow_protection, 1U);
    buzzer_stop();

    protection_trigger_locked(ALERT_CURRENT_HIGH);
    ASSERT_TRUE(buzzer_is_active());
}

TEST_CASE(disarmed_protection_leaves_the_amplifier_energised)
{
    prot_establish_ready_state();
    g_config.allow_protection = 0U;
    g_tx_active = true;               /* keyed, so there is something to lose */
    stub_gpio_reset_counters();

    protection_trigger_locked(ALERT_CURRENT_HIGH);   /* a P0-class fault */

    /* The P0 shutdown sequence must not have run at all: the RF-board fault
     * latch is never raised and the HV rail is untouched.  Any one of these
     * moving would be a half-executed shutdown, which is worse than either
     * doing it or not doing it. */
    ASSERT_EQ(stub_gpio_get_last(RF_IN_PROTECT_GPIO_Port, RF_IN_PROTECT_Pin),
              GPIO_PIN_RESET_E);
    ASSERT_TRUE(g_psu_on);
    ASSERT_TRUE(g_hv_on);
    /* Keying is not interrupted either. */
    ASSERT_TRUE(g_tx_active);
}

TEST_CASE(disarmed_protection_stays_silent_over_the_cat_link)
{
    prot_establish_ready_state();
    g_config.allow_protection = 0U;
    comm_stub_event_count = 0U;

    protection_trigger_locked(ALERT_CURRENT_HIGH);
    protection_trigger_locked(ALERT_NTC_OVERHEAT);

    /* No alert=ON for the host either: the front panel is silent, so the PC
     * must not be told about faults that nothing is acting on. */
    ASSERT_EQ(comm_stub_event_count, 0U);
}

TEST_CASE(disarmed_protection_ignores_the_hardware_fault_line)
{
    prot_establish_ready_state();
    g_config.allow_protection = 0U;

    hardpwr_err_set(true);
    protection_hardpwr_err_isr();

    ASSERT_FALSE(protection_is_active());
    ASSERT_EQ(g_alert_active, ALERT_NONE);
}

TEST_CASE(disarmed_protection_ignores_the_watchdog_callbacks)
{
    prot_establish_ready_state();
    g_config.allow_protection = 0U;

    /* These are the callbacks the AWD1/2/3 hardware fires.  The switch
     * stops the FIRMWARE reacting; it cannot stop the peripheral tripping,
     * which is the whole reason those watchdogs exist. */
    protection_awd1_cb();
    protection_awd2_cb();
    protection_awd3_cb();
    protection_comp1_isr();
    protection_comp2_isr();

    ASSERT_FALSE(protection_is_active());
    ASSERT_EQ(g_alert_active, ALERT_NONE);
}

TEST_CASE(disarmed_protection_ignores_the_software_threshold_comparisons)
{
    prot_establish_ready_state();
    g_config.allow_protection = 0U;
    g_tx_active = true;

    g_adc_data.rev2_mW      = DEFAULT_PROTECT_REFLECT_POWER2_W * 1000U;
    g_adc_data.fwd2_mW      = MAX_TX_POWER2_W * 1000U;
    g_adc_data.vbias_mV     = 5000U;
    g_adc_data.ntc_core_tenth_c = (int32_t)(MAX_NTC_TEMP_C * 10U + 100U);

    protection_check_adc1_dma();     /* REV2 / FWD2 / VBIAS */
    protection_check_ntc();          /* over-temperature */

    ASSERT_FALSE(protection_is_active());
    ASSERT_EQ(g_alert_active, ALERT_NONE);
    ASSERT_FALSE(g_alert_flag);
}

TEST_CASE(disarmed_protection_ignores_the_task_side_backstop)
{
    prot_establish_ready_state();
    g_config.allow_protection = 0U;
    g_tx_active = true;

    g_adc_data.swr_x100 = 900;       /* far above any threshold */
    g_adc_data.rev2_mW = 5000U;

    protection_check();              /* the 100 ms task sweep */

    ASSERT_FALSE(protection_is_active());
    ASSERT_FALSE(g_alert_flag);
}

TEST_CASE(re_arming_protection_restores_the_full_behaviour)
{
    prot_establish_ready_state();

    g_config.allow_protection = 0U;
    protection_trigger_locked(ALERT_CURRENT_HIGH);
    ASSERT_FALSE(protection_is_active());

    /* Back on, and the very same fault now latches.  This is what makes the
     * switch a switch rather than a one-way door: the operator can always
     * put the unit back into a protected state without a power cycle. */
    g_config.allow_protection = 1U;
    protection_trigger_locked(ALERT_CURRENT_HIGH);
    ASSERT_TRUE(protection_is_active());
    ASSERT_EQ(g_alert_active, ALERT_CURRENT_HIGH);
}

TEST_CASE(a_latched_fault_survives_disarming_rather_than_vanishing)
{
    prot_establish_ready_state();

    /* Fault first, then disarm.  A latch that vanished on the switch being
     * thrown would leave an undismissable popup: gui_show_alert() runs from
     * the GUI task off g_alert_flag, while gui_dismiss_alert() is only called
     * when protection_is_active() goes false, and protection_clear() - the
     * only thing that clears the latch - would still be reachable, but the
     * latched P0 would also have dropped g_allow_rf with no way left to
     * re-arm it.  Keeping the latch means the operator recovers normally. */
    protection_trigger_locked(ALERT_CURRENT_HIGH);
    ASSERT_TRUE(protection_is_active());

    g_config.allow_protection = 0U;
    ASSERT_TRUE(protection_is_active());
    ASSERT_EQ(g_alert_active, ALERT_CURRENT_HIGH);
    ASSERT_FALSE(g_allow_rf);       /* P0 already latched it low */

    /* And it is still clearable, so recovery does not need a reboot. */
    protection_clear();
    ASSERT_FALSE(protection_is_active());
}

TEST_CASE(a_new_fault_after_disarming_does_not_re_latch)
{
    prot_establish_ready_state();
    g_config.allow_protection = 0U;

    protection_trigger_locked(ALERT_CURRENT_HIGH);
    protection_clear();              /* back to a clean, disarmed state */

    /* A different fault arrives with protection still off.  Nothing may
     * latch: the re-entry guard is not what stops this, the kill switch is,
     * and conflating the two would make the switch unreliable to reason
     * about. */
    protection_trigger_locked(ALERT_NTC_OVERHEAT);
    ASSERT_FALSE(protection_is_active());
    ASSERT_EQ(g_alert_active, ALERT_NONE);
    ASSERT_FALSE(g_alert_flag);
}

/*============================================================================
 *  Dismiss (protection_clear) followed by the next interrupt pass
 *==========================================================================*/

TEST_CASE(a_dismiss_survives_the_next_interrupt_pass_while_the_fault_is_real)
{
    prot_establish_ready_state();

    /* One milliwatt over the limit: the comparison is strict, so "at the
     * limit" would not trip at all and this case would pass for the wrong
     * reason. */
    g_adc_data.rev2_mW = DEFAULT_PROTECT_REFLECT_POWER2_W * 1000U + 1U;
    protection_trigger_locked(ALERT_REFLECT_POWER_HIGH);
    ASSERT_TRUE(protection_is_active());

    protection_clear();
    ASSERT_FALSE(protection_is_active());

    /* The ADC1 DMA-completion interrupt runs this thousands of times per
     * second, so without the hold-off the popup and the buzzer would be
     * back microseconds after the operator dismissed them. */
    protection_check_adc1_dma();

    ASSERT_FALSE(protection_is_active());
    ASSERT_FALSE(buzzer_is_active());
}

TEST_CASE(a_dismiss_survives_the_ntc_interrupt_pass_too)
{
    prot_establish_ready_state();

    g_adc_data.ntc_core_tenth_c = (int32_t)(MAX_NTC_TEMP_C * 10U + 100U);
    protection_trigger_locked(ALERT_NTC_OVERHEAT);
    ASSERT_TRUE(protection_is_active());

    protection_clear();
    ASSERT_FALSE(protection_is_active());

    protection_check_ntc();

    ASSERT_FALSE(protection_is_active());
}

TEST_CASE(a_fault_that_outlasts_the_hold_off_latches_again)
{
    prot_establish_ready_state();

    g_adc_data.rev2_mW = DEFAULT_PROTECT_REFLECT_POWER2_W * 1000U + 1U;
    protection_trigger_locked(ALERT_REFLECT_POWER_HIGH);
    protection_clear();

    /* A dismiss must not disarm the protection for good: a real
     * over-temperature has to come back and say so.  Push the wall clock
     * past the hold-off rather than sleeping through it. */
    test_wall_clock_advance(6000U);

    protection_check_adc1_dma();

    ASSERT_TRUE(protection_is_active());
    ASSERT_EQ(g_alert_active, ALERT_REFLECT_POWER_HIGH);
}

TEST_CASE(a_different_fault_is_not_swallowed_by_the_hold_off)
{
    prot_establish_ready_state();

    /* A coupler fault, dismissed. */
    g_adc_data.rev2_mW = DEFAULT_PROTECT_REFLECT_POWER2_W * 1000U + 1U;
    protection_trigger_locked(ALERT_REFLECT_POWER_HIGH);
    protection_clear();

    /* A genuinely different fault inside the hold-off window - a real
     * overdrive is a P0 emergency and must never be masked by the fact that
     * the operator dismissed something else a moment ago. */
    g_adc_data.rev2_mW = 0;
    protection_awd1_cb();

    ASSERT_TRUE(protection_is_active());
    ASSERT_EQ(g_alert_active, ALERT_OVERDRIVE);
}

TEST_CASE(a_dismiss_never_requests_a_latch_reset_when_no_fault_ever_latched_it)
{
    prot_establish_ready_state();

    /* A P1 thermal fault drives no hardware latch, so the RF deck owes us
     * no reset - and the 100 ms pulse would drop RF_BIAS on an amplifier
     * that is merely too hot. */
    protection_trigger_locked(ALERT_NTC_OVERHEAT);
    protection_clear();

    ASSERT_FALSE(protection_take_latch_clear_request());
}

TEST_CASE(a_dismiss_of_a_p0_still_requests_the_latch_reset)
{
    prot_establish_ready_state();

    protection_trigger_locked(ALERT_CURRENT_HIGH);
    protection_clear();

    /* The counterpart to the case above: the P0 path did raise PB13, so the
     * RF deck still needs its reset pulse. */
    ASSERT_TRUE(protection_take_latch_clear_request());
}

/*============================================================================
 *  Digital fault lines held off during the power-on settling window
 *==========================================================================*/

TEST_CASE(a_digital_fault_is_ignored_while_the_lines_are_held_off)
{
    prot_establish_ready_state();
    protection_hold_digital_faults();

    /* The power board pulls its fault line high while the supply is still
     * coming up.  That is not a fault, and acting on it would mean a P0
     * shutdown with a 10 s buzzer at every cold boot. */
    hardpwr_err_set(true);
    protection_hardpwr_err_isr();

    ASSERT_FALSE(protection_is_active());
    ASSERT_FALSE(g_alert_flag);
    ASSERT_FALSE(buzzer_is_active());
}

TEST_CASE(the_hold_off_masks_the_exti_lines_individually)
{
    prot_establish_ready_state();

    /* Both lines start armed, as CubeMX leaves them. */
    EXTI->IMR1 |= EXTI_LINE_BIT(12) | EXTI_LINE_BIT(15);

    protection_hold_digital_faults();

    /* Line 12 (PB12, the power board fault) and line 15 (PF15, the
     * backup-rail monitor) share EXTI15_10_IRQn.  Masking per LINE is what
     * keeps the backup monitor's own line state intact - only the two
     * fault lines are held off. */
    ASSERT_EQ(EXTI->IMR1 & EXTI_LINE_BIT(12), 0U);
    ASSERT_EQ(EXTI->IMR1 & EXTI_LINE_BIT(15), 0U);

    protection_arm_digital_faults();

    /* And the mask comes off again, per line. */
    ASSERT_NE(EXTI->IMR1 & EXTI_LINE_BIT(12), 0U);
    ASSERT_NE(EXTI->IMR1 & EXTI_LINE_BIT(15), 0U);
}

TEST_CASE(a_fault_still_asserted_when_the_lines_are_armed_is_reported)
{
    prot_establish_ready_state();
    protection_hold_digital_faults();

    /* The supply did NOT come up: the line is still high when the settling
     * window ends.  A line that never fell produces no new edge, so the
     * arm function has to look at the level itself - otherwise this fault
     * would be lost for good. */
    hardpwr_err_set(true);
    protection_arm_digital_faults();

    ASSERT_TRUE(protection_is_active());
    ASSERT_EQ(g_alert_active, ALERT_HARDWARE_FAULT);
}

TEST_CASE(a_transient_that_cleared_during_the_hold_off_is_forgotten)
{
    prot_establish_ready_state();
    protection_hold_digital_faults();

    /* The line pulsed and went back low while the supply was settling -
     * a startup artefact.  Nothing to report, and the amplifier must not
     * be shut down over it. */
    hardpwr_err_set(true);
    hardpwr_err_set(false);
    protection_arm_digital_faults();

    ASSERT_FALSE(protection_is_active());
    ASSERT_FALSE(g_alert_flag);
}

/*============================================================================
 *  Per-source trigger history
 *
 *  g_alert_active holds ONE cause and the re-entry guard refuses everything
 *  after it, so it cannot answer "which of my four current protections is
 *  misbehaving" - which is the question behind every nuisance CURRENT_HIGH
 *  report.  The history answers it: one sticky bit per source, since power-on.
 *==========================================================================*/

/* Read one source's bit out of the history. */
static bool src_fired(ProtectionSource src)
{
    return ((protection_source_fired_mask() >> (uint32_t)src) & 1U) != 0U;
}

TEST_CASE(the_history_starts_empty_and_is_cleared_only_at_power_on)
{
    prot_establish_ready_state();       /* calls protection_init() */

    /* Nothing has fired yet.  Assert the whole word rather than spot bits:
     * a stray stamp surviving init would otherwise go unnoticed. */
    ASSERT_EQ(protection_source_fired_mask(), 0U);

    protection_awd2_cb();
    ASSERT_TRUE(src_fired(PROT_SRC_AWD2));

    /* The dismiss must NOT clear it - that is the whole point of the
     * record.  An operator who dismissed the popup and then wants to know
     * whether it was the watchdog or the comparator needs the answer to
     * still be there. */
    protection_clear();
    ASSERT_TRUE(src_fired(PROT_SRC_AWD2));
    ASSERT_FALSE(protection_is_active());

    /* And a fresh unit clears it. */
    protection_init();
    ASSERT_EQ(protection_source_fired_mask(), 0U);
}

TEST_CASE(every_protection_source_keeps_its_own_bit)
{
    prot_establish_ready_state();
    adc_init();
    arm_analog_chain();

    /* One trigger per source, each a genuinely different subsystem, so a
     * mistake that funnels two of them into one bit shows up as two bits
     * colliding or one of them missing. */
    protection_awd1_cb();                             /* RF input       */
    protection_clear();
    protection_awd2_cb();                             /* main current   */
    protection_clear();
    protection_awd3_cb();                             /* reverse        */
    protection_clear();
    protection_comp1_isr();                           /* RF comparator  */
    protection_clear();
    protection_comp2_isr();                           /* I comparator   */
    protection_clear();
    g_adc_data.ntc_core_tenth_c = (int32_t)(MAX_NTC_TEMP_C * 10U + 100U);
    protection_check_ntc();                           /* over-temp      */
    protection_clear();
    g_adc_data.ntc_core_tenth_c = 250;
    g_adc_data.vbias_mV = 5000U;
    protection_check_adc1_dma();                      /* PA bias        */
    protection_clear();
    g_adc_data.vbias_mV = 0;
    g_adc_data.rev2_mW = DEFAULT_PROTECT_REFLECT_POWER2_W * 1000U + 1U;
    protection_check_adc1_dma();                      /* coupler        */
    protection_clear();
    protection_hold_digital_faults();
    hardpwr_err_set(true);
    protection_arm_digital_faults();                  /* power board    */

    /* Every source above, named one by one.  Deliberately NOT a literal
     * like 0xFF: a mask that silently skipped a bit would still look right
     * as a hex constant, and the whole value of this feature is that the
     * bits are trustworthy.  PROT_SRC_FREQ is absent by design - it is
     * stamped from freq_detect.c, which has its own test. */
    uint32_t expect = 0U;
    expect |= 1U << (uint32_t)PROT_SRC_AWD1;
    expect |= 1U << (uint32_t)PROT_SRC_AWD2;
    expect |= 1U << (uint32_t)PROT_SRC_AWD3;
    expect |= 1U << (uint32_t)PROT_SRC_COMP1;
    expect |= 1U << (uint32_t)PROT_SRC_COMP2;
    expect |= 1U << (uint32_t)PROT_SRC_HARDERR;
    expect |= 1U << (uint32_t)PROT_SRC_NTC;
    expect |= 1U << (uint32_t)PROT_SRC_VBIAS;
    expect |= 1U << (uint32_t)PROT_SRC_COUPLER;

    ASSERT_EQ(protection_source_fired_mask(), expect);
}

TEST_CASE(the_current_protections_report_which_line_actually_fired)
{
    prot_establish_ready_state();
    adc_init();
    arm_analog_chain();

    /* This is the case the feature exists for: AWD2 and COMP2 watch the SAME
     * node, so a CURRENT_HIGH popup is ambiguous - is the limit set too
     * low, or is the analog front end overdriving?  Only the history
     * separates them. */
    protection_awd2_cb();

    ASSERT_TRUE(src_fired(PROT_SRC_AWD2));
    ASSERT_FALSE(src_fired(PROT_SRC_COMP2));

    /* The other way round, on a fresh unit. */
    protection_init();
    protection_comp2_isr();

    ASSERT_TRUE(src_fired(PROT_SRC_COMP2));
    ASSERT_FALSE(src_fired(PROT_SRC_AWD2));
}

TEST_CASE(a_source_that_fires_while_another_fault_is_latched_is_still_recorded)
{
    prot_establish_ready_state();
    adc_init();
    arm_analog_chain();

    /* The overdrive latches. */
    protection_awd1_cb();
    ASSERT_EQ(g_alert_active, ALERT_OVERDRIVE);

    /* Now the comparator sees a real over-current.  It is refused by the
     * re-entry guard - and that refusal is exactly why the history has to
     * be stamped BEFORE the trigger call: without it this event would be
     * invisible, and "only ever shows OVERDRIVE" is the symptom that makes
     * a multi-fault fault hunt hard. */
    protection_comp2_isr();
    ASSERT_EQ(g_alert_active, ALERT_OVERDRIVE);      /* still not re-latched */
    ASSERT_TRUE(src_fired(PROT_SRC_COMP2));
    ASSERT_TRUE(src_fired(PROT_SRC_AWD1));
}

TEST_CASE(a_source_firing_while_protection_is_disarmed_is_still_recorded)
{
    prot_establish_ready_state();
    g_config.allow_protection = 0U;

    /* The kill switch suppresses everything the firmware would DO - the
     * shutdown, the buzzer, the popup.  It does not rewrite history: the
     * analog front end really did produce an out-of-limit reading, and an
     * operator switching protection off to see whether the popup stops is
     * exactly the person who needs to know it was still happening. */
    protection_awd2_cb();

    ASSERT_FALSE(protection_is_active());
    ASSERT_FALSE(g_alert_flag);
    ASSERT_TRUE(src_fired(PROT_SRC_AWD2));
}

TEST_CASE(a_comparator_edge_inside_the_settling_window_is_not_recorded)
{
    prot_establish_ready_state();
    adc_init();
    ASSERT_FALSE(adc_stabilisation_done());

    /* The comparator reference is still ramping, so an edge here says
     * nothing about the amplifier.  Stamping it would point the operator
     * at a comparator that never saw an overdrive - a false lead that is
     * worse than no record at all. */
    protection_comp1_isr();
    protection_comp2_isr();

    ASSERT_EQ(protection_source_fired_mask(), 0U);

    /* Once the chain is armed the very same call records. */
    arm_analog_chain();
    protection_comp2_isr();
    ASSERT_TRUE(src_fired(PROT_SRC_COMP2));
    ASSERT_FALSE(src_fired(PROT_SRC_COMP1));
}

TEST_CASE(a_masked_power_board_line_is_not_recorded)
{
    prot_establish_ready_state();
    protection_hold_digital_faults();

    /* Still inside the settling window: the supply is not up, so an
     * asserted fault line is not yet information about a fault. */
    hardpwr_err_set(true);
    protection_hardpwr_err_isr();

    ASSERT_FALSE(protection_is_active());
    ASSERT_EQ(protection_source_fired_mask(), 0U);

    /* The same level once the mask is off IS a real fault. */
    protection_arm_digital_faults();
    ASSERT_TRUE(protection_is_active());
    ASSERT_TRUE(src_fired(PROT_SRC_HARDERR));
}

TEST_CASE(clearing_the_history_needs_no_reboot)
{
    prot_establish_ready_state();

    protection_awd1_cb();
    protection_awd2_cb();
    ASSERT_NE(protection_source_fired_mask(), 0U);

    /* A bench session: wipe the record and start watching again.  This is
     * the only path that clears it besides power-on. */
    protection_clear_source_history();
    ASSERT_EQ(protection_source_fired_mask(), 0U);

    /* And it stays clear until something actually fires. */
    protection_awd3_cb();
    ASSERT_EQ(protection_source_fired_mask(), 1U << (uint32_t)PROT_SRC_AWD3);
}

TEST_CASE(a_digital_fault_acts_normally_once_the_lines_are_armed)
{
    prot_establish_ready_state();
    protection_hold_digital_faults();
    protection_arm_digital_faults();

    /* After the settling window the line is a real fault indicator again. */
    hardpwr_err_set(true);
    protection_hardpwr_err_isr();

    ASSERT_TRUE(protection_is_active());
    ASSERT_EQ(g_alert_active, ALERT_HARDWARE_FAULT);
}

/*============================================================================
 *  IM (main current) mute - the HV inrush window
 *
 *  Closing PWR_ON charges the supply's bulk capacitors, and that surge looks
 *  like a short circuit on the node AWD2 and COMP2 both watch.  The mute is
 *  one flag: raised by power_hv_on() before the switch, tested by both
 *  handlers, released by the pump task once IM_MASK_HOLD_MS has passed.
 *==========================================================================*/

TEST_CASE(hv_inrush_mutes_both_main_current_paths)
{
    prot_establish_ready_state();
    adc_init();
    arm_analog_chain();

    protection_mask_im_high();
    ASSERT_TRUE(protection_im_masked());
    /* The external latch clear is asserted with the mute: anything the
     * inrush latched has to be gone by the time the window closes. */
    ASSERT_EQ(stub_gpio_get_last(CLEAR_FAULTX_OC_GPIO_Port,
                                 CLEAR_FAULTX_OC_Pin), 0);

    /* Neither the watchdog callback nor the comparator may latch, and
     * neither may stamp the history: a mute is not a fault event. */
    protection_awd2_cb();
    protection_comp2_isr();

    ASSERT_FALSE(protection_is_active());
    ASSERT_EQ(g_alert_active, ALERT_NONE);
    ASSERT_FALSE(g_alert_flag);
    ASSERT_FALSE(buzzer_is_active());
    ASSERT_EQ(protection_source_fired_mask(), 0U);
}

TEST_CASE(the_pump_task_releases_the_mute_after_the_hold_window)
{
    prot_establish_ready_state();
    adc_init();
    arm_analog_chain();

    protection_mask_im_high();

    /* Early ticks change nothing. */
    protection_im_mask_tick();
    ASSERT_TRUE(protection_im_masked());
    ASSERT_EQ(stub_gpio_get_last(CLEAR_FAULTX_OC_GPIO_Port,
                                 CLEAR_FAULTX_OC_Pin), 0);

    test_wall_clock_advance(IM_MASK_HOLD_MS - 1U);
    protection_im_mask_tick();
    ASSERT_TRUE(protection_im_masked());
    /* Still holding the latch clear - the window has not closed yet. */
    ASSERT_EQ(stub_gpio_get_last(CLEAR_FAULTX_OC_GPIO_Port,
                                 CLEAR_FAULTX_OC_Pin), 0);

    /* One more millisecond and the next tick releases it. */
    test_wall_clock_advance(1U);
    protection_im_mask_tick();
    ASSERT_FALSE(protection_im_masked());
    /* ... and hands the latch bus back, so the next genuine fault can set
     * it again. */
    ASSERT_EQ(stub_gpio_get_last(CLEAR_FAULTX_OC_GPIO_Port,
                                 CLEAR_FAULTX_OC_Pin), 1);

    /* The very same edge now latches: the window ends where the inrush
     * transient does, not at the end of the transmission. */
    protection_comp2_isr();
    ASSERT_TRUE(protection_is_active());
    ASSERT_EQ(g_alert_active, ALERT_CURRENT_HIGH);
}

TEST_CASE(the_mute_stays_on_until_the_pump_task_clears_it)
{
    prot_establish_ready_state();
    adc_init();
    arm_analog_chain();

    protection_mask_im_high();

    /* The window is closed by the task, not by the clock alone - that is
     * what "cleared in the pump task" means, and it keeps the ISR side to a
     * single volatile read. */
    test_wall_clock_advance(5000U);
    ASSERT_TRUE(protection_im_masked());

    protection_awd2_cb();
    ASSERT_FALSE(protection_is_active());
}

TEST_CASE(the_mute_covers_only_the_main_current_paths)
{
    prot_establish_ready_state();
    adc_init();
    arm_analog_chain();

    protection_mask_im_high();

    /* RF input overdrive watches a different node and has nothing to do
     * with the supply's inrush, so it must still trip. */
    protection_awd1_cb();
    ASSERT_TRUE(protection_is_active());
    ASSERT_EQ(g_alert_active, ALERT_OVERDRIVE);
}

TEST_CASE(raising_the_hv_rail_mutes_the_main_current_paths)
{
    /* From a rail-off state so power_hv_on() really runs the sequence. */
    protection_init();
    power_init_outputs();
    ASSERT_FALSE(protection_im_masked());

    power_hv_on();

    /* Raised before PWR_ON is asserted - the comparator can see the surge on
     * the first millisecond, so the flag must not be a step behind. */
    ASSERT_TRUE(protection_im_masked());
    ASSERT_EQ(stub_gpio_get_last(PWR_ON_GPIO_Port, PWR_ON_Pin), GPIO_PIN_SET_E);
    /* And the external latch is held clear for the same window. */
    ASSERT_EQ(stub_gpio_get_last(CLEAR_FAULTX_OC_GPIO_Port,
                                 CLEAR_FAULTX_OC_Pin), 0);

    (void)power_rail_step();
}

TEST_CASE(power_on_clears_a_mute_left_behind)
{
    prot_establish_ready_state();
    protection_mask_im_high();
    ASSERT_TRUE(protection_im_masked());

    /* protection_init() is the power-on reset: a mute that somehow survived
     * a soft restart must not come up with the rail still unmuted. */
    protection_init();
    ASSERT_FALSE(protection_im_masked());
}

/*============================================================================
 *  Timed clear of the external fault latch
 *
 *  protection_clear_fault_safe() only raises a flag; the monitor task's
 *  service drives the pin.  These cases drive the service by hand, which is
 *  exactly what the monitor task does once per tick.
 *==========================================================================*/

static uint8_t clear_fault_level(void)
{
    return stub_gpio_get_last(CLEAR_FAULTX_OC_GPIO_Port, CLEAR_FAULTX_OC_Pin);
}

TEST_CASE(a_safe_clear_request_does_not_touch_the_pin_until_the_task_runs)
{
    prot_establish_ready_state();
    stub_gpio_reset_counters();

    /* The caller is an ISR or a command handler: it must not block, and it
     * must not drive the pin itself. */
    protection_clear_fault_safe(100U);

    /* Nothing on the pin yet - the monitor task has not run. */
    ASSERT_EQ(stub_gpio_set_count(CLEAR_FAULTX_OC_GPIO_Port,
                                  CLEAR_FAULTX_OC_Pin), 0U);

    /* The task's first pass pulls the line low. */
    protection_fault_clear_service();
    ASSERT_EQ(clear_fault_level(), 0);
}

TEST_CASE(the_service_releases_the_line_when_the_requested_time_is_up)
{
    prot_establish_ready_state();

    protection_clear_fault_safe(100U);
    protection_fault_clear_service();
    ASSERT_EQ(clear_fault_level(), 0);

    /* Still inside the window: the line stays low. */
    test_wall_clock_advance(50U);
    protection_fault_clear_service();
    ASSERT_EQ(clear_fault_level(), 0);

    /* One millisecond past the deadline and the next pass releases it. */
    test_wall_clock_advance(50U);
    protection_fault_clear_service();
    ASSERT_EQ(clear_fault_level(), 1);
}

TEST_CASE(a_zero_length_request_still_produces_a_pulse)
{
    prot_establish_ready_state();

    /* 0 means 1 ms: a zero-length pulse would give the latch network no
     * edge to react to. */
    protection_clear_fault_safe(0U);
    protection_fault_clear_service();
    ASSERT_EQ(clear_fault_level(), 0);

    test_wall_clock_advance(1U);
    protection_fault_clear_service();
    ASSERT_EQ(clear_fault_level(), 1);
}

TEST_CASE(a_request_that_arrives_during_a_pulse_is_not_swallowed)
{
    prot_establish_ready_state();

    protection_clear_fault_safe(100U);
    protection_fault_clear_service();
    ASSERT_EQ(clear_fault_level(), 0);

    /* A second request lands while the first pulse is still running.  The
     * service clears its pending flag BEFORE doing the pin work, so this
     * one is picked up on the next pass rather than lost. */
    protection_clear_fault_safe(100U);

    test_wall_clock_advance(100U);
    protection_fault_clear_service();
    /* The first pulse ended, but the second request is now running, so the
     * line is low again. */
    ASSERT_EQ(clear_fault_level(), 0);

    test_wall_clock_advance(100U);
    protection_fault_clear_service();
    ASSERT_EQ(clear_fault_level(), 1);
}

TEST_CASE(the_service_never_touches_the_pin_when_nothing_was_requested)
{
    prot_establish_ready_state();
    stub_gpio_reset_counters();

    protection_fault_clear_service();
    protection_fault_clear_service();

    ASSERT_EQ(stub_gpio_set_count(CLEAR_FAULTX_OC_GPIO_Port,
                                  CLEAR_FAULTX_OC_Pin), 0U);
    ASSERT_EQ(protection_fault_clear_remaining_ms(), 0U);
}

TEST_CASE(the_remaining_time_reports_how_long_the_pulse_still_has_to_run)
{
    prot_establish_ready_state();

    ASSERT_EQ(protection_fault_clear_remaining_ms(), 0U);

    protection_clear_fault_safe(100U);
    protection_fault_clear_service();
    ASSERT_EQ(protection_fault_clear_remaining_ms(), 100U);

    test_wall_clock_advance(40U);
    ASSERT_EQ(protection_fault_clear_remaining_ms(), 60U);

    /* Past the deadline: nothing left to wait for. */
    test_wall_clock_advance(100U);
    ASSERT_EQ(protection_fault_clear_remaining_ms(), 0U);
}

TEST_CASE(the_blocking_hold_and_the_timed_pulse_share_the_pin_safely)
{
    prot_establish_ready_state();

    /* The dismiss path holds the line low itself... */
    protection_fault_clear_hold();
    ASSERT_EQ(clear_fault_level(), 0);

    /* ... and a timed pulse runs on top of it. */
    protection_clear_fault_safe(50U);
    protection_fault_clear_service();
    ASSERT_EQ(clear_fault_level(), 0);

    /* The timed pulse ends first, but the line must stay low: the dismiss
     * pulse is still holding it. */
    test_wall_clock_advance(100U);
    protection_fault_clear_service();
    ASSERT_EQ(clear_fault_level(), 0);

    /* Now the dismiss pulse lets go, and the line is released. */
    protection_fault_clear_release();
    ASSERT_EQ(clear_fault_level(), 1);
}

TEST_CASE(the_im_mute_and_a_timed_pulse_share_the_pin_safely)
{
    prot_establish_ready_state();

    /* The HV inrush mute holds the line low for its own window... */
    protection_mask_im_high();
    ASSERT_EQ(clear_fault_level(), 0);

    /* ... and a timed clear runs on top of it. */
    protection_clear_fault_safe(50U);
    protection_fault_clear_service();

    test_wall_clock_advance(100U);
    protection_fault_clear_service();
    /* The timed pulse ended; the mute still holds the line. */
    ASSERT_EQ(clear_fault_level(), 0);

    /* The mute window closes and the line is finally released. */
    test_wall_clock_advance(IM_MASK_HOLD_MS);
    protection_im_mask_tick();
    ASSERT_EQ(clear_fault_level(), 1);
}

/*============================================================================
 *  Fault strings for the GUI popup
 *==========================================================================*/

TEST_CASE(fault_strings_are_distinct_and_non_empty)
{
    ASSERT_STR_EQ(protection_get_fault_string(ALERT_SWR_HIGH), "SWR Too High");
    ASSERT_STR_EQ(protection_get_fault_string(ALERT_REFLECT_POWER_HIGH),
                  "Reflected Power High");
    ASSERT_STR_EQ(protection_get_fault_string(ALERT_CURRENT_HIGH),
                  "Main Current Too High");
    ASSERT_STR_EQ(protection_get_fault_string(ALERT_HARDWARE_FAULT),
                  "Hardware Fault");
    ASSERT_STR_EQ(protection_get_fault_string(ALERT_OVERDRIVE),
                  "RF Input Overdrive");
    ASSERT_STR_EQ(protection_get_fault_string(ALERT_NTC_OVERHEAT),
                  "Core Temperature High");
    ASSERT_STR_EQ(protection_get_fault_string(ALERT_VBIAS_FAULT), "PA Bias Fault");
    ASSERT_STR_EQ(protection_get_fault_string(ALERT_NONE), "Unknown Fault");
}

/*============================================================================
 *  Suite table
 *============================================================================*/

const test_entry_t g_suite_protection[] = {
    TEST_ENTRY(comp_isrs_ignore_edges_while_the_chain_is_still_arming),
    TEST_ENTRY(comp1_latches_overdrive_once_the_chain_is_armed),
    TEST_ENTRY(comp2_latches_current_high_once_the_chain_is_armed),
    TEST_ENTRY(a_p0_fault_takes_the_panel_out_of_transmit),
    TEST_ENTRY(a_p0_fault_still_publishes_the_state_when_already_idle),
    TEST_ENTRY(a_dismissed_buzzer_reprograms_its_note_on_the_next_alert),
    TEST_ENTRY(init_leaves_every_output_disarmed),
    TEST_ENTRY(a_dismiss_survives_the_next_interrupt_pass_while_the_fault_is_real),
    TEST_ENTRY(a_dismiss_survives_the_ntc_interrupt_pass_too),
    TEST_ENTRY(a_fault_that_outlasts_the_hold_off_latches_again),
    TEST_ENTRY(a_different_fault_is_not_swallowed_by_the_hold_off),
    TEST_ENTRY(a_dismiss_never_requests_a_latch_reset_when_no_fault_ever_latched_it),
    TEST_ENTRY(a_dismiss_of_a_p0_still_requests_the_latch_reset),
    TEST_ENTRY(trigger_drops_everything_and_raises_rf_protect),
    TEST_ENTRY(trigger_publishes_alert_and_blocks_ptt),
    TEST_ENTRY(trigger_starts_the_3_tone_buzzer),
    TEST_ENTRY(trigger_is_latched_first_fault_wins),
    TEST_ENTRY(hardpwr_err_active_high_trips),
    TEST_ENTRY(hardpwr_err_active_low_edge_is_ignored),
    TEST_ENTRY(hardpwr_err_level_stays_high_only_triggers_once),
    TEST_ENTRY(digital_isr_is_the_generic_rf_deck_entry),
    TEST_ENTRY(awd_callbacks_map_to_their_fault_codes),
    TEST_ENTRY(awd_shutdown_does_not_wait_for_the_task),
    TEST_ENTRY(dma_rev2_over_hard_limit_trips),
    TEST_ENTRY(dma_rev2_at_limit_does_not_trip),
    TEST_ENTRY(dma_swr_rule_needs_both_reverse_and_swr),
    TEST_ENTRY(dma_forward_limits_trip),
    TEST_ENTRY(dma_forward_at_limit_does_not_trip),
    TEST_ENTRY(dma_checks_run_even_when_not_keyed),
    TEST_ENTRY(dma_check_is_skipped_once_latched),
    TEST_ENTRY(ntc_overheat_trips_even_when_idle),
    TEST_ENTRY(ntc_at_limit_does_not_trip),
    TEST_ENTRY(ntc_check_ignored_once_latched),
    TEST_ENTRY(backstop_catches_overheat_before_the_keyed_limits),
    TEST_ENTRY(backstop_delegates_coupler_limits_to_the_dma_rules),
    TEST_ENTRY(backstop_checks_vbias_whether_keyed_or_not),
    TEST_ENTRY(backstop_does_nothing_when_healthy),
    TEST_ENTRY(clear_releases_the_latch_pin_and_buzzer),
    TEST_ENTRY(clear_without_a_fault_is_harmless),
    TEST_ENTRY(severity_mapping_classifies_each_cause),
    TEST_ENTRY(p1_thermal_fault_leaves_rail_up_and_only_blocks_ptt),
    TEST_ENTRY(p1_dismiss_does_not_touch_protect_pin),
    TEST_ENTRY(buzzer_stops_after_10s_even_undismissed),
    TEST_ENTRY(buzzer_repeat_p0_does_not_reset_the_deadline),
    TEST_ENTRY(buzzer_repeat_p1_is_suppressed_while_p0_sounding),
    TEST_ENTRY(buzzer_p1_upgrades_to_p0_when_p0_fires),
    TEST_ENTRY(disarmed_protection_ignores_a_direct_trigger),
    TEST_ENTRY(disarmed_protection_sounds_no_buzzer),
    TEST_ENTRY(armed_protection_does_sound_the_buzzer),
    TEST_ENTRY(disarmed_protection_leaves_the_amplifier_energised),
    TEST_ENTRY(disarmed_protection_stays_silent_over_the_cat_link),
    TEST_ENTRY(disarmed_protection_ignores_the_hardware_fault_line),
    TEST_ENTRY(disarmed_protection_ignores_the_watchdog_callbacks),
    TEST_ENTRY(disarmed_protection_ignores_the_software_threshold_comparisons),
    TEST_ENTRY(disarmed_protection_ignores_the_task_side_backstop),
    TEST_ENTRY(re_arming_protection_restores_the_full_behaviour),
    TEST_ENTRY(a_latched_fault_survives_disarming_rather_than_vanishing),
    TEST_ENTRY(a_new_fault_after_disarming_does_not_re_latch),
    TEST_ENTRY(a_digital_fault_is_ignored_while_the_lines_are_held_off),
    TEST_ENTRY(the_hold_off_masks_the_exti_lines_individually),
    TEST_ENTRY(a_fault_still_asserted_when_the_lines_are_armed_is_reported),
    TEST_ENTRY(a_transient_that_cleared_during_the_hold_off_is_forgotten),
    TEST_ENTRY(the_history_starts_empty_and_is_cleared_only_at_power_on),
    TEST_ENTRY(every_protection_source_keeps_its_own_bit),
    TEST_ENTRY(the_current_protections_report_which_line_actually_fired),
    TEST_ENTRY(a_source_that_fires_while_another_fault_is_latched_is_still_recorded),
    TEST_ENTRY(a_source_firing_while_protection_is_disarmed_is_still_recorded),
    TEST_ENTRY(a_comparator_edge_inside_the_settling_window_is_not_recorded),
    TEST_ENTRY(a_masked_power_board_line_is_not_recorded),
    TEST_ENTRY(clearing_the_history_needs_no_reboot),
    TEST_ENTRY(a_digital_fault_acts_normally_once_the_lines_are_armed),
    TEST_ENTRY(hv_inrush_mutes_both_main_current_paths),
    TEST_ENTRY(the_pump_task_releases_the_mute_after_the_hold_window),
    TEST_ENTRY(the_mute_stays_on_until_the_pump_task_clears_it),
    TEST_ENTRY(the_mute_covers_only_the_main_current_paths),
    TEST_ENTRY(raising_the_hv_rail_mutes_the_main_current_paths),
    TEST_ENTRY(power_on_clears_a_mute_left_behind),
    TEST_ENTRY(a_safe_clear_request_does_not_touch_the_pin_until_the_task_runs),
    TEST_ENTRY(the_service_releases_the_line_when_the_requested_time_is_up),
    TEST_ENTRY(a_zero_length_request_still_produces_a_pulse),
    TEST_ENTRY(a_request_that_arrives_during_a_pulse_is_not_swallowed),
    TEST_ENTRY(the_service_never_touches_the_pin_when_nothing_was_requested),
    TEST_ENTRY(the_remaining_time_reports_how_long_the_pulse_still_has_to_run),
    TEST_ENTRY(the_blocking_hold_and_the_timed_pulse_share_the_pin_safely),
    TEST_ENTRY(the_im_mute_and_a_timed_pulse_share_the_pin_safely),
    TEST_ENTRY(fault_strings_are_distinct_and_non_empty),
    TEST_SUITE_END
};