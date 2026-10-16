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
 *============================================================================*/

/* Everything at rest: relays open, supply live, RF allowed, no latch. */
static void prot_establish_ready_state(void)
{
    protection_clear();               /* guarantees an unlatched start */
    power_init_outputs();
    power_psu_on();                   /* main supply latch (PG5) */
    power_hv_on();                    /* HV rail: PWR_ON, bias one step later */
    (void)power_rail_step();          /* settle the bias so the rail is idle */
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

    g_adc_data.rev2_mW          = 0;
    g_adc_data.rev1_mW          = 0;
    g_adc_data.fwd1_mW          = 0;
    g_adc_data.fwd2_mW          = 0;
    g_adc_data.im_mA            = 0;
    g_adc_data.vbias_mV         = 0;
    g_adc_data.swr_x100         = 100;
    g_adc_data.ntc_core_tenth_c = 250;   /* 25.0 C */

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
 *  protection_init() - power-on state
 *============================================================================*/

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
    /* Note 1 of the warning is the high tone: ARR = 12.5 MHz / 2.5 kHz */
    ASSERT_EQ(htim15.Instance->ARR, 12500000U / BUZZER_NOTE_HIGH_HZ);
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
    g_adc_data.rev2_mW = 400000;

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
    protection_clear();

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

TEST_CASE(dma_checks_are_skipped_when_not_keyed)
{
    prot_establish_ready_state();
    g_tx_active         = false;   /* idle: no RF to protect */
    g_adc_data.rev2_mW  = 900000;
    g_adc_data.fwd1_mW  = 900000;
    g_adc_data.fwd2_mW  = 900000;

    protection_check_adc1_dma();

    ASSERT_FALSE(protection_is_active());
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
    g_adc_data.ntc_core_tenth_c = 901;   /* 90.1 C, limit 90.0 C */

    protection_check_ntc();

    ASSERT_TRUE(protection_is_active());
    ASSERT_EQ(g_alert_active, ALERT_NTC_OVERHEAT);
}

TEST_CASE(ntc_at_limit_does_not_trip)
{
    prot_establish_ready_state();
    g_adc_data.ntc_core_tenth_c = 900;

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

TEST_CASE(backstop_checks_vbias_only_while_keyed)
{
    prot_establish_ready_state();
    g_tx_active        = true;
    g_adc_data.vbias_mV = 1400;      /* limit 1300 mV */

    protection_check();
    ASSERT_EQ(g_alert_active, ALERT_VBIAS_FAULT);
    protection_clear();

    /* Idle: a stale VBIAS reading is irrelevant. */
    g_tx_active        = false;
    g_adc_data.vbias_mV = 1400;
    protection_check();
    ASSERT_FALSE(protection_is_active());
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
            p0_arr, 12500000U / BUZZER_NOTE_HIGH_HZ);
    ASSERT_EQ(p0_arr, 12500000U / BUZZER_NOTE_HIGH_HZ);

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
              12500000U / BUZZER_NOTE_MID_HZ);

    /* P0 fires while P1 is sounding - the operator gets the full P0
     * 3-tone loop, starting on the high note. */
    buzzer_start();

    ASSERT_TRUE(buzzer_is_active());
    ASSERT_EQ(htim15.Instance->ARR,
              12500000U / BUZZER_NOTE_HIGH_HZ);
}

/*============================================================================
 *  Fault strings for the GUI popup
 *============================================================================*/

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
    TEST_ENTRY(init_leaves_every_output_disarmed),
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
    TEST_ENTRY(dma_checks_are_skipped_when_not_keyed),
    TEST_ENTRY(dma_check_is_skipped_once_latched),
    TEST_ENTRY(ntc_overheat_trips_even_when_idle),
    TEST_ENTRY(ntc_at_limit_does_not_trip),
    TEST_ENTRY(ntc_check_ignored_once_latched),
    TEST_ENTRY(backstop_catches_overheat_before_the_keyed_limits),
    TEST_ENTRY(backstop_delegates_coupler_limits_to_the_dma_rules),
    TEST_ENTRY(backstop_checks_vbias_only_while_keyed),
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
    TEST_ENTRY(fault_strings_are_distinct_and_non_empty),
    TEST_SUITE_END
};