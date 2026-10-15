/**
 * @file    test_power_control.c
 * @brief   Unit tests for power_control.c: switch_band() and the
 *          PTT relay-sequencing state machine.
 *
 * @details power_control.c is compiled normally into the test binary
 *          (see tests/CMakeLists.txt).  This file only includes the
 *          public header and drives the API, asserting on the GPIO
 *          stub recorder (User/Tests/stubs/stm32h7xx_hal_stub.c).
 *
 *          The TIM5 relay-delay one-shot is bypassed by invoking
 *          power_relay_timer_callback() directly, which is exactly
 *          what the real TIM5 update ISR does.
 *
 * @note    hw_pinmap.h maps the LPF / RELAY / PSU macros onto GPIO_TypeDef
 *          pointers.  The stub treats them as opaque identities, so the
 *          tests validate *which pin got which level*, not the physical
 *          port address.
 */

#include "test_harness.h"
#include "opa2000.h"
#include "hw_pinmap.h"
#include "power_control.h"
#include "stm32h7xx_hal_stub.h"

#include <stddef.h>

/*============================================================================
 *  Helpers
 *============================================================================*/

/* TIM handles owned by Core/Src/tim.c in the firmware; provided by the stub. */
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* Press or release the PTT input.  PTT_INX is active-low. */
static void ptt_set_pressed(bool pressed)
{
    HAL_GPIO_WritePin(PTT_IN_GPIO_Port, PTT_IN_Pin,
                      pressed ? GPIO_PIN_RESET_E : GPIO_PIN_SET_E);
}

/* Put the relay/PSU pins into the known idle state power_init_outputs()
 * produces and then bring the supply up to "ready", which is the state
 * every PTT test starts from.  Counters are cleared so the test only sees
 * its own writes. */
static void ptt_establish_idle_outputs(void)
{
    power_init_outputs();
    power_psu_on();               /* main supply latch (PG5) */
    power_hv_on();                /* HV rail: PWR_ON, bias one step later */
    /* power_hv_on() only raises PWR_ON now; RF_BIAS follows one
     * power_rail_step() later.  Settle it here so every PTT test starts
     * from a fully biased, keyable amplifier - otherwise the HV sequencing
     * interlock would refuse to key and the test would be measuring the
     * rail sequence instead of what it means to check. */
    (void)power_rail_step();
    stub_gpio_reset_counters();
    ptt_state    = PTT_STATE_IDLE;
    g_allow_rf   = true;
    g_alert_flag = false;
    /* Boot gate: the IM zero-offset calibration has run and the GUI has
     * painted a frame.  Without it ptt_apply_level() force-releases and
     * every test below would be measuring the gate, not the relays. */
    g_ptt_ready  = true;
}

/*============================================================================
 *  switch_band() - LPF is HIGH-active
 *============================================================================*/

/* Lower LPF index == shorter wavelength (LPF1 = 15-10 m ... LPF5 = 160 m). */
TEST_CASE(switch_band_15m_activates_lpf1_only)
{
    stub_gpio_reset_counters();
    switch_band(BAND_15M_10M);

    ASSERT_EQ(g_current_band, BAND_15M_10M);
    ASSERT_EQ(stub_gpio_get_last(LPF1_GPIO_Port, LPF1_Pin), GPIO_PIN_SET_E);
    ASSERT_EQ(stub_gpio_get_last(LPF2_GPIO_Port, LPF2_Pin), GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(LPF3_GPIO_Port, LPF3_Pin), GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(LPF4_GPIO_Port, LPF4_Pin), GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(LPF5_GPIO_Port, LPF5_Pin), GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(LPF6_GPIO_Port, LPF6_Pin), GPIO_PIN_RESET_E);
}

TEST_CASE(switch_band_160m_activates_lpf5_only)
{
    stub_gpio_reset_counters();
    switch_band(BAND_160M);

    ASSERT_EQ(g_current_band, BAND_160M);
    ASSERT_EQ(stub_gpio_get_last(LPF5_GPIO_Port, LPF5_Pin), GPIO_PIN_SET_E);
    ASSERT_EQ(stub_gpio_get_last(LPF1_GPIO_Port, LPF1_Pin), GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(LPF2_GPIO_Port, LPF2_Pin), GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(LPF3_GPIO_Port, LPF3_Pin), GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(LPF4_GPIO_Port, LPF4_Pin), GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(LPF6_GPIO_Port, LPF6_Pin), GPIO_PIN_RESET_E);
}

TEST_CASE(switch_band_80m_activates_lpf4_only)
{
    stub_gpio_reset_counters();
    switch_band(BAND_80M);
    ASSERT_EQ(stub_gpio_get_last(LPF4_GPIO_Port, LPF4_Pin), GPIO_PIN_SET_E);
    ASSERT_EQ(stub_gpio_get_last(LPF5_GPIO_Port, LPF5_Pin), GPIO_PIN_RESET_E);
}

TEST_CASE(switch_band_40m30m_activates_lpf3_only)
{
    stub_gpio_reset_counters();
    switch_band(BAND_40M_30M);
    ASSERT_EQ(stub_gpio_get_last(LPF3_GPIO_Port, LPF3_Pin), GPIO_PIN_SET_E);
    ASSERT_EQ(stub_gpio_get_last(LPF4_GPIO_Port, LPF4_Pin), GPIO_PIN_RESET_E);
}

TEST_CASE(switch_band_20m17m_activates_lpf2_only)
{
    stub_gpio_reset_counters();
    switch_band(BAND_20M_17M);
    ASSERT_EQ(stub_gpio_get_last(LPF2_GPIO_Port, LPF2_Pin), GPIO_PIN_SET_E);
    ASSERT_EQ(stub_gpio_get_last(LPF1_GPIO_Port, LPF1_Pin), GPIO_PIN_RESET_E);
}

TEST_CASE(switch_band_fault_falls_back_to_15m)
{
    stub_gpio_reset_counters();
    switch_band(BAND_FAULT);

    /* Per spec: config corruption or BAND_FAULT -> default to 15-10 m */
    ASSERT_EQ(g_current_band, BAND_15M_10M);
    ASSERT_EQ(stub_gpio_get_last(LPF1_GPIO_Port, LPF1_Pin), GPIO_PIN_SET_E);
}

TEST_CASE(switch_band_none_falls_back_to_15m)
{
    stub_gpio_reset_counters();
    switch_band(BAND_NONE);
    ASSERT_EQ(g_current_band, BAND_15M_10M);
}

TEST_CASE(switch_band_releases_previous_band)
{
    /* Start on 160 m (LPF5 high), then switch to 15 m: LPF5 must go low. */
    stub_gpio_reset_counters();
    switch_band(BAND_160M);
    ASSERT_EQ(stub_gpio_get_last(LPF5_GPIO_Port, LPF5_Pin), GPIO_PIN_SET_E);

    switch_band(BAND_15M_10M);
    ASSERT_EQ(stub_gpio_get_last(LPF5_GPIO_Port, LPF5_Pin), GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(LPF1_GPIO_Port, LPF1_Pin), GPIO_PIN_SET_E);
}

TEST_CASE(switch_band_index_increases_with_wavelength)
{
    /* Regression guard for the "lower index == shorter wavelength"
     * convention the mechanical LPF board relies on. */
    const BandType   bands[]  = { BAND_15M_10M, BAND_20M_17M,
                                   BAND_40M_30M, BAND_80M, BAND_160M };
    GPIO_TypeDef *const ports[] = { LPF1_GPIO_Port, LPF2_GPIO_Port,
                                    LPF3_GPIO_Port, LPF4_GPIO_Port,
                                    LPF5_GPIO_Port };
    const uint16_t pins[] = { LPF1_Pin, LPF2_Pin, LPF3_Pin, LPF4_Pin, LPF5_Pin };

    for (unsigned b = 0; b < 5; b++) {
        for (unsigned p = 0; p < 5; p++) {
            uint8_t lvl;
            stub_gpio_reset_counters();
            switch_band(bands[b]);
            lvl = stub_gpio_get_last(ports[p], pins[p]);
            ASSERT_EQ(lvl, (p == b) ? GPIO_PIN_SET_E : GPIO_PIN_RESET_E);
        }
    }
}

TEST_CASE(switch_band_never_activates_two_bands_at_once)
{
    /* Sweep all bands; after each switch exactly one LPF must be high. */
    const BandType bands[] = {
        BAND_160M, BAND_80M, BAND_40M_30M, BAND_20M_17M, BAND_15M_10M
    };
    const GPIO_TypeDef *ports[] = {
        LPF1_GPIO_Port, LPF2_GPIO_Port, LPF3_GPIO_Port,
        LPF4_GPIO_Port, LPF5_GPIO_Port, LPF6_GPIO_Port
    };
    const uint16_t pins[] = {
        LPF1_Pin, LPF2_Pin, LPF3_Pin, LPF4_Pin, LPF5_Pin, LPF6_Pin
    };

    for (unsigned b = 0; b < 5; b++) {
        stub_gpio_reset_counters();
        switch_band(bands[b]);

        int high_count = 0;
        for (unsigned p = 0; p < 6; p++) {
            if (stub_gpio_get_last((GPIO_TypeDef *)ports[p], pins[p])
                == GPIO_PIN_SET_E) {
                high_count++;
            }
        }
        ASSERT_EQ(high_count, 1);
    }
}

/*============================================================================
 *  PTT relay-sequencing state machine
 *============================================================================*/

TEST_CASE(ptt_press_from_idle_closes_relay2_only)
{
    ptt_establish_idle_outputs();

    ptt_set_pressed(true);
    power_control_ptt_isr();

    /* RELAY2 closes immediately (PA -> LPF), RELAY1 waits for TIM5 */
    ASSERT_EQ(ptt_state, PTT_STATE_TX_STARTING);
    ASSERT_EQ(ptt_event_flag, 1);
    ASSERT_EQ(stub_gpio_get_last(RELAY2_GPIO_Port, RELAY2_Pin),
              GPIO_PIN_SET_E);
    ASSERT_EQ(stub_gpio_get_last(RELAY1_GPIO_Port, RELAY1_Pin),
              GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_set_count(RELAY1_GPIO_Port, RELAY1_Pin), 0);
}

TEST_CASE(ptt_relay_timer_then_closes_relay1)
{
    ptt_establish_idle_outputs();
    ptt_state = PTT_STATE_TX_STARTING;

    power_relay_timer_callback();

    ASSERT_EQ(ptt_state, PTT_STATE_TX_RUNNING);
    ASSERT_EQ(stub_gpio_get_last(RELAY1_GPIO_Port, RELAY1_Pin),
              GPIO_PIN_SET_E);
}

TEST_CASE(ptt_release_from_running_opens_relay1_then_delayed_relay2)
{
    ptt_establish_idle_outputs();
    ptt_state = PTT_STATE_TX_RUNNING;

    ptt_set_pressed(false);
    power_control_ptt_isr();

    /* RELAY1 opens immediately, RELAY2 waits for TIM5 */
    ASSERT_EQ(ptt_state, PTT_STATE_TX_STOPPING);
    ASSERT_EQ(ptt_event_flag, 0);
    ASSERT_EQ(stub_gpio_get_last(RELAY1_GPIO_Port, RELAY1_Pin),
              GPIO_PIN_RESET_E);

    power_relay_timer_callback();
    ASSERT_EQ(ptt_state, PTT_STATE_IDLE);
    ASSERT_EQ(stub_gpio_get_last(RELAY2_GPIO_Port, RELAY2_Pin),
              GPIO_PIN_RESET_E);
}

TEST_CASE(ptt_repress_during_stopping_cancels_the_delay)
{
    ptt_establish_idle_outputs();
    ptt_state = PTT_STATE_TX_STOPPING;

    /* Operator re-presses before TIM5 expires */
    ptt_set_pressed(true);
    power_control_ptt_isr();

    ASSERT_EQ(ptt_state, PTT_STATE_TX_RUNNING);
    ASSERT_EQ(stub_gpio_get_last(RELAY1_GPIO_Port, RELAY1_Pin),
              GPIO_PIN_SET_E);
}

TEST_CASE(ptt_blocked_until_im_calibration_completes)
{
    ptt_establish_idle_outputs();
    g_allow_rf = false;                 /* IM zero-offset not done yet */

    ptt_set_pressed(true);
    power_control_ptt_isr();

    /* Nothing must close */
    ASSERT_EQ(ptt_state, PTT_STATE_IDLE);
    ASSERT_EQ(stub_gpio_get_last(RELAY2_GPIO_Port, RELAY2_Pin),
              GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_set_count(RELAY2_GPIO_Port, RELAY2_Pin), 0);
    ASSERT_EQ(stub_gpio_set_count(RELAY1_GPIO_Port, RELAY1_Pin), 0);
}

TEST_CASE(ptt_blocked_while_alert_active)
{
    ptt_establish_idle_outputs();
    ptt_state   = PTT_STATE_TX_RUNNING;
    g_alert_flag = true;                /* protection latched */

    ptt_set_pressed(true);
    power_control_ptt_isr();

    /* Fault path forces both relays open regardless of PTT */
    ASSERT_EQ(ptt_state, PTT_STATE_IDLE);
    ASSERT_EQ(stub_gpio_get_last(RELAY1_GPIO_Port, RELAY1_Pin),
              GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(RELAY2_GPIO_Port, RELAY2_Pin),
              GPIO_PIN_RESET_E);
}

TEST_CASE(ptt_press_while_idle_after_stop_is_noop)
{
    ptt_establish_idle_outputs();

    /* Release while idle -> must not touch any relay */
    ptt_set_pressed(false);
    power_control_ptt_isr();

    ASSERT_EQ(ptt_state, PTT_STATE_IDLE);
    ASSERT_EQ(stub_gpio_set_count(RELAY1_GPIO_Port, RELAY1_Pin), 0);
    ASSERT_EQ(stub_gpio_set_count(RELAY2_GPIO_Port, RELAY2_Pin), 0);
}

TEST_CASE(ptt_flag_is_cleared_by_task_api)
{
    ptt_establish_idle_outputs();

    ptt_set_pressed(true);
    power_control_ptt_isr();
    ASSERT_EQ(power_get_ptt_flag(), 1);

    power_clear_ptt_flag();
    ASSERT_EQ(power_get_ptt_flag(), -1);
}

/*============================================================================
 *  PTT drives g_tx_active
 *
 *  protection_check() and freq_detect() both gate on g_tx_active, so it
 *  has to follow the relay state machine or the software protection never
 *  runs.  g_hv_on is the operator's HV switch and must be unaffected
 *  by PTT.
 *==========================================================================*/

TEST_CASE(ptt_press_raises_tx_active)
{
    ptt_establish_idle_outputs();
    ASSERT_FALSE(g_tx_active);
    ASSERT_TRUE(g_hv_on);

    ptt_set_pressed(true);
    power_control_ptt_isr();
    ASSERT_TRUE(g_tx_active);
}

TEST_CASE(ptt_release_clears_tx_active)
{
    ptt_establish_idle_outputs();

    ptt_set_pressed(true);
    power_control_ptt_isr();
    power_relay_timer_callback();          /* -> TX_RUNNING */
    ASSERT_TRUE(g_tx_active);

    ptt_set_pressed(false);
    power_control_ptt_isr();
    ASSERT_TRUE(g_tx_active);              /* still stopping */

    power_relay_timer_callback();          /* RELAY2 opens -> IDLE */
    ASSERT_FALSE(g_tx_active);
}

TEST_CASE(ptt_never_touches_the_supply_flag)
{
    ptt_establish_idle_outputs();

    ptt_set_pressed(true);
    power_control_ptt_isr();
    power_relay_timer_callback();
    ASSERT_TRUE(g_hv_on);

    ptt_set_pressed(false);
    power_control_ptt_isr();
    power_relay_timer_callback();
    ASSERT_TRUE(g_hv_on);                  /* still up, just not keyed */
    ASSERT_FALSE(g_tx_active);
}

TEST_CASE(ptt_while_hv_off_only_moves_the_relays)
{
    ptt_establish_idle_outputs();
    /* Drop the HV rail (PWR_ON + RF_BIAS).  The pedal path must still run its
     * relay sequence - the RF switch is a separate subsystem from the rail -
     * but no RF can be produced because power_tx_enable() sees g_hv_on false.
     * This is the deliberate consequence of keeping the three rails apart. */
    power_hv_off();
    (void)power_rail_step();
    ASSERT_FALSE(g_hv_on);

    ptt_set_pressed(true);
    power_control_ptt_isr();
    power_relay_timer_callback();
    ASSERT_TRUE(g_tx_active);              /* relay sequence still ran */
    ASSERT_FALSE(g_hv_on);                  /* but the HV rail stays down */

    ptt_set_pressed(false);
    power_control_ptt_isr();
    power_relay_timer_callback();
    ASSERT_FALSE(g_tx_active);
    ASSERT_FALSE(g_hv_on);
}

TEST_CASE(ptt_blocked_by_alert_clears_tx_active)
{
    ptt_establish_idle_outputs();

    ptt_set_pressed(true);
    power_control_ptt_isr();
    power_relay_timer_callback();
    ASSERT_TRUE(g_tx_active);

    /* Protection latches while transmitting. */
    g_alert_flag = true;
    ptt_set_pressed(true);
    power_control_ptt_isr();

    ASSERT_EQ(ptt_state, PTT_STATE_IDLE);
    ASSERT_FALSE(g_tx_active);
    ASSERT_TRUE(g_hv_on);                  /* fault must not fake a power-off */
}

TEST_CASE(ptt_blocked_during_stop_sequence_also_clears_tx_active)
{
    ptt_establish_idle_outputs();

    ptt_set_pressed(true);
    power_control_ptt_isr();
    ptt_set_pressed(false);
    power_control_ptt_isr();
    ASSERT_EQ(ptt_state, PTT_STATE_TX_STOPPING);

    /* Alert arrives inside the 15 ms stop delay. */
    g_alert_flag = true;
    ptt_set_pressed(false);
    power_control_ptt_isr();

    ASSERT_EQ(ptt_state, PTT_STATE_IDLE);
    ASSERT_FALSE(g_tx_active);
    ASSERT_TRUE(g_hv_on);
    ASSERT_EQ(stub_gpio_get_last(RELAY1_GPIO_Port, RELAY1_Pin),
              GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(RELAY2_GPIO_Port, RELAY2_Pin),
              GPIO_PIN_RESET_E);
}

TEST_CASE(ptt_repress_during_stop_keeps_tx_active)
{
    ptt_establish_idle_outputs();

    ptt_set_pressed(true);
    power_control_ptt_isr();
    ptt_set_pressed(false);
    power_control_ptt_isr();
    ASSERT_EQ(ptt_state, PTT_STATE_TX_STOPPING);

    /* Re-press cancels the stop: still keyed, so TX must persist. */
    ptt_set_pressed(true);
    power_control_ptt_isr();
    ASSERT_EQ(ptt_state, PTT_STATE_TX_RUNNING);
    ASSERT_TRUE(g_tx_active);
}

TEST_CASE(relays_api_keeps_tx_active_in_step)
{
    ptt_establish_idle_outputs();

    power_relays_tx_on();
    ASSERT_TRUE(g_tx_active);

    power_relays_tx_off();
    power_relay_timer_callback();
    ASSERT_EQ(ptt_state, PTT_STATE_IDLE);
    ASSERT_FALSE(g_tx_active);
}

/*============================================================================
 *  PTT_OUT (PC15, ACTIVE HIGH) follows the RF path
 *
 *  It must stay low through the 15 ms break-before-make (RELAY1 still
 *  open, the PA is not reaching the antenna yet) and only rise once the
 *  state machine reaches TX_RUNNING.  It must fall immediately on release
 *  and on any protection trip.
 *==========================================================================*/

static uint8_t ptt_out_level(void)
{
    return stub_gpio_get_last(PTT_OUT_GPIO_Port, PTT_OUT_Pin);
}

TEST_CASE(ptt_out_starts_released_after_init)
{
    power_init();
    ASSERT_EQ(ptt_out_level(), GPIO_PIN_RESET_E);
}

TEST_CASE(ptt_out_stays_low_during_the_relay_delay)
{
    ptt_establish_idle_outputs();

    ptt_set_pressed(true);
    power_control_ptt_isr();

    /* TX_STARTING: RELAY1 still open, so no keying feedback yet. */
    ASSERT_EQ(ptt_state, PTT_STATE_TX_STARTING);
    ASSERT_EQ(ptt_out_level(), GPIO_PIN_RESET_E);

    power_relay_timer_callback();

    /* TX_RUNNING: RELAY1 closed, RF reaches the antenna. */
    ASSERT_EQ(ptt_state, PTT_STATE_TX_RUNNING);
    ASSERT_EQ(ptt_out_level(), GPIO_PIN_SET_E);
}

TEST_CASE(ptt_out_drops_immediately_on_release)
{
    ptt_establish_idle_outputs();
    ptt_set_pressed(true);
    power_control_ptt_isr();
    power_relay_timer_callback();
    ASSERT_EQ(ptt_out_level(), GPIO_PIN_SET_E);

    ptt_set_pressed(false);
    power_control_ptt_isr();

    /* No 15 ms wait: the radio must see the drop straight away. */
    ASSERT_EQ(ptt_state, PTT_STATE_TX_STOPPING);
    ASSERT_EQ(ptt_out_level(), GPIO_PIN_RESET_E);

    power_relay_timer_callback();
    ASSERT_EQ(ptt_out_level(), GPIO_PIN_RESET_E);
}

TEST_CASE(ptt_out_reasserts_when_a_stop_is_cancelled)
{
    ptt_establish_idle_outputs();
    ptt_set_pressed(true);
    power_control_ptt_isr();
    power_relay_timer_callback();

    ptt_set_pressed(false);
    power_control_ptt_isr();
    ASSERT_EQ(ptt_out_level(), GPIO_PIN_RESET_E);

    /* Re-press inside the stop delay puts the RF path back. */
    ptt_set_pressed(true);
    power_control_ptt_isr();
    ASSERT_EQ(ptt_state, PTT_STATE_TX_RUNNING);
    ASSERT_EQ(ptt_out_level(), GPIO_PIN_SET_E);
}

TEST_CASE(ptt_out_drops_when_an_alert_blocks_ptt)
{
    ptt_establish_idle_outputs();
    ptt_set_pressed(true);
    power_control_ptt_isr();
    power_relay_timer_callback();
    ASSERT_EQ(ptt_out_level(), GPIO_PIN_SET_E);

    g_alert_flag = true;
    ptt_set_pressed(true);
    power_control_ptt_isr();

    ASSERT_EQ(ptt_state, PTT_STATE_IDLE);
    ASSERT_EQ(ptt_out_level(), GPIO_PIN_RESET_E);
}

TEST_CASE(ptt_out_stays_asserted_while_the_hv_rail_sequences_down)
{
    ptt_establish_idle_outputs();
    ptt_set_pressed(true);
    power_control_ptt_isr();
    power_relay_timer_callback();
    ASSERT_EQ(ptt_out_level(), GPIO_PIN_SET_E);

    /* power_hv_off() deliberately does NOT open the relays: the HV rail and
     * the RF path are independent, so the operator asking to drop high
     * voltage must not also unkey.  PTT_OUT therefore stays high and the
     * relay state machine is untouched. */
    power_hv_off();
    ASSERT_FALSE(g_hv_on);
    ASSERT_EQ(ptt_out_level(), GPIO_PIN_SET_E);   /* relays never opened */

    (void)power_rail_step();
    ASSERT_EQ(ptt_out_level(), GPIO_PIN_SET_E);
}

TEST_CASE(ptt_out_drops_when_the_supply_is_switched_off)
{
    ptt_establish_idle_outputs();
    ptt_set_pressed(true);
    power_control_ptt_isr();
    power_relay_timer_callback();
    ASSERT_EQ(ptt_out_level(), GPIO_PIN_SET_E);

    /* power_standby() goes through power_relays_tx_off(), which arms the
     * 15 ms timer - PTT_OUT must not wait for it. */
    power_standby();
    ASSERT_EQ(ptt_out_level(), GPIO_PIN_SET_E);   /* still in TX_STOPPING */

    power_relay_timer_callback();
    ASSERT_EQ(ptt_out_level(), GPIO_PIN_RESET_E);
}

TEST_CASE(ptt_out_is_high_active)
{
    /* Guard against the polarity being flipped by mistake: SET = keyed. */
    power_ptt_out_set(false);
    ASSERT_EQ(ptt_out_level(), GPIO_PIN_RESET_E);
    power_ptt_out_set(true);
    ASSERT_EQ(ptt_out_level(), GPIO_PIN_SET_E);
}

/*============================================================================
 *  Front-panel power API
 *==========================================================================*/

TEST_CASE(power_init_leaves_the_external_psu_and_the_pa_off)
{
    power_init();

    /* The PSU is an external device the operator switches themselves, so
     * the firmware must NOT power it up unprompted.  PG5 stays at its
     * power-off level (low) after boot. */
    ASSERT_FALSE(g_psu_on);
    ASSERT_EQ(stub_gpio_get_last(PSU_ON_GPIO_Port, PSU_ON_Pin),
              GPIO_PIN_RESET_E);

    /* The PA is dark too - the front-panel button brings the HV rail up. */
    ASSERT_FALSE(g_hv_on);
    ASSERT_EQ(stub_gpio_get_last(PWR_ON_GPIO_Port, PWR_ON_Pin),
              GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(RF_BIAS_GPIO_Port, RF_BIAS_Pin),
              GPIO_PIN_RESET_E);
}

TEST_CASE(power_hv_enable_engages_the_rail_only_once)
{
    power_init();
    ASSERT_FALSE(g_hv_on);

    power_hv_enable();
    ASSERT_TRUE(g_hv_on);
    ASSERT_EQ(stub_gpio_get_last(PWR_ON_GPIO_Port, PWR_ON_Pin),
              GPIO_PIN_SET_E);
    ASSERT_FALSE(g_tx_active);

    /* The external PSU and the relays are NOT part of the HV request.  The
     * PSU stays off because the operator never switched it on. */
    ASSERT_FALSE(g_psu_on);
    ASSERT_EQ(stub_gpio_get_last(PSU_ON_GPIO_Port, PSU_ON_Pin),
              GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(RELAY1_GPIO_Port, RELAY1_Pin),
              GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(RELAY2_GPIO_Port, RELAY2_Pin),
              GPIO_PIN_RESET_E);

    stub_gpio_reset_counters();
    power_hv_enable();                           /* idempotent */
    ASSERT_EQ(stub_gpio_set_count(PWR_ON_GPIO_Port, PWR_ON_Pin), 0);
}

TEST_CASE(power_hv_disable_drops_the_rail_but_leaves_supply_and_relays)
{
    /* The external PSU and both relays are untouched.  Switch the PSU on
     * first so the assertion is meaningful: with it already on, "untouched"
     * means it must still be on afterwards. */
    power_psu_on();
    power_hv_enable();
    (void)power_rail_step();

    power_hv_disable();
    (void)power_rail_step();

    ASSERT_FALSE(g_hv_on);
    ASSERT_FALSE(g_tx_active);
    ASSERT_EQ(stub_gpio_get_last(RF_BIAS_GPIO_Port, RF_BIAS_Pin),
              GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(PWR_ON_GPIO_Port, PWR_ON_Pin),
              GPIO_PIN_RESET_E);

    ASSERT_TRUE(g_psu_on);
    ASSERT_EQ(stub_gpio_get_last(PSU_ON_GPIO_Port, PSU_ON_Pin),
              GPIO_PIN_SET_E);
    ASSERT_EQ(stub_gpio_get_last(RELAY1_GPIO_Port, RELAY1_Pin),
              GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(RELAY2_GPIO_Port, RELAY2_Pin),
              GPIO_PIN_RESET_E);
}

TEST_CASE(power_standby_keeps_the_hv_rail_but_drops_the_rf_path)
{
    power_init();
    g_ptt_ready = true;
    power_hv_enable();
    (void)power_rail_step();        /* settle HV: PWR_ON up, then bias */
    power_tx_enable();
    power_relay_timer_callback();
    ASSERT_TRUE(g_tx_active);

    power_standby();
    ASSERT_EQ(ptt_state, PTT_STATE_TX_STOPPING);
    ASSERT_TRUE(g_hv_on);                    /* rail stays engaged */

    power_relay_timer_callback();
    ASSERT_FALSE(g_tx_active);
    ASSERT_TRUE(g_hv_on);
    ASSERT_EQ(ptt_out_level(), GPIO_PIN_RESET_E);
}

TEST_CASE(ptt_is_refused_while_the_hv_rail_is_still_sequencing)
{
    power_init_outputs();
    ptt_state    = PTT_STATE_IDLE;
    g_allow_rf   = true;
    g_alert_flag = false;
    g_ptt_ready  = true;

    /* HV rail on, bias not yet raised: keying now would put the PA into
     * service during the very settling window the gap exists to protect.
     * This interlock is about sequencing only - note that g_psu_on is false
     * here and that no longer blocks anything. */
    power_hv_on();
    ASSERT_TRUE(power_rail_busy());

    power_tx_enable();
    ASSERT_EQ(ptt_state, PTT_STATE_IDLE);
    ASSERT_FALSE(g_tx_active);

    /* Once the sequence settles, the same request is honoured. */
    (void)power_rail_step();
    ASSERT_FALSE(power_rail_busy());
    power_tx_enable();
    ASSERT_EQ(ptt_state, PTT_STATE_TX_STARTING);
    ASSERT_TRUE(g_tx_active);
}

/*============================================================================
 *  Task-initiated transmit (CAT / serial / front panel)
 *==========================================================================*/

TEST_CASE(power_tx_enable_keys_the_amplifier)
{
    ptt_establish_idle_outputs();
    power_tx_enable();
    ASSERT_EQ(ptt_state, PTT_STATE_TX_STARTING);
    ASSERT_TRUE(g_tx_active);

    power_relay_timer_callback();
    ASSERT_EQ(ptt_state, PTT_STATE_TX_RUNNING);
    ASSERT_EQ(ptt_out_level(), GPIO_PIN_SET_E);

    power_tx_disable();
    power_relay_timer_callback();
    ASSERT_FALSE(g_tx_active);
    ASSERT_EQ(ptt_out_level(), GPIO_PIN_RESET_E);
}

TEST_CASE(power_tx_enable_is_refused_until_the_boot_gate_opens)
{
    /* Nothing about the PSU or the HV rail here - only g_ptt_ready, which
     * stays false until the IM zero-offset calibration has run and the GUI
     * has painted a frame. */
    power_init();
    g_allow_rf   = true;
    g_alert_flag = false;
    g_ptt_ready  = false;

    power_tx_enable();
    ASSERT_EQ(ptt_state, PTT_STATE_IDLE);
    ASSERT_FALSE(g_tx_active);
}

TEST_CASE(power_tx_enable_ignores_the_psu_and_hv_rail_state)
{
    /* The operator deliberately removed the power-state interlocks: whether
     * the PA is energised is not the firmware's call at request time.  This
     * test pins that down - with the PSU off AND the HV rail down, a valid
     * request must still key.  Keep it this way unless the interlock is
     * explicitly reinstated. */
    power_init();
    ASSERT_FALSE(g_psu_on);                   /* external PSU off */
    ASSERT_FALSE(g_hv_on);                    /* HV rail off     */
    g_allow_rf   = true;
    g_alert_flag = false;
    g_ptt_ready  = true;

    power_tx_enable();
    ASSERT_EQ(ptt_state, PTT_STATE_TX_STARTING);
    ASSERT_TRUE(g_tx_active);
}

TEST_CASE(power_tx_enable_is_refused_while_calibrating)
{
    ptt_establish_idle_outputs();
    g_allow_rf = false;                      /* IM zero-offset unfinished */

    power_tx_enable();
    ASSERT_EQ(ptt_state, PTT_STATE_IDLE);
    ASSERT_FALSE(g_tx_active);
}

TEST_CASE(power_tx_enable_is_refused_while_alert_latched)
{
    ptt_establish_idle_outputs();
    g_alert_flag = true;

    power_tx_enable();
    ASSERT_EQ(ptt_state, PTT_STATE_IDLE);
    ASSERT_FALSE(g_tx_active);
}

TEST_CASE(power_get_ptt_state_reads_the_active_low_input)
{
    ptt_establish_idle_outputs();

    ptt_set_pressed(false);                   /* pin high = released */
    ASSERT_FALSE(power_get_ptt_state());

    ptt_set_pressed(true);                    /* pin low = transmit */
    ASSERT_TRUE(power_get_ptt_state());
}

/*============================================================================
 *  Band follow
 *==========================================================================*/

TEST_CASE(power_update_band_selects_the_matching_lpf)
{
    power_init();

    power_update_band(21000000U);             /* 21 MHz -> 15-10 m */
    ASSERT_EQ(g_current_band, BAND_15M_10M);
    ASSERT_EQ(stub_gpio_get_last(LPF1_GPIO_Port, LPF1_Pin), GPIO_PIN_SET_E);

    power_update_band(7100000U);              /* 7.1 MHz -> 40-30 m */
    ASSERT_EQ(g_current_band, BAND_40M_30M);
    ASSERT_EQ(stub_gpio_get_last(LPF1_GPIO_Port, LPF1_Pin), GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(LPF3_GPIO_Port, LPF3_Pin), GPIO_PIN_SET_E);
}

TEST_CASE(power_update_band_ignores_out_of_band_frequencies)
{
    power_init();
    power_update_band(21000000U);             /* 15-10 m */
    ASSERT_EQ(g_current_band, BAND_15M_10M);

    power_update_band(1000000U);              /* below every band */
    ASSERT_EQ(g_current_band, BAND_15M_10M);

    power_update_band(3000000U);              /* between 160 m and 80 m */
    ASSERT_EQ(g_current_band, BAND_15M_10M);

    power_update_band(50000000U);             /* above every band */
    ASSERT_EQ(g_current_band, BAND_15M_10M);
    ASSERT_EQ(stub_gpio_get_last(LPF1_GPIO_Port, LPF1_Pin), GPIO_PIN_SET_E);
}

TEST_CASE(power_update_band_does_not_disturb_a_matching_band)
{
    power_init();
    power_update_band(3700000U);              /* 80 m */
    ASSERT_EQ(g_current_band, BAND_80M);

    stub_gpio_reset_counters();
    power_update_band(3800000U);              /* still 80 m */
    ASSERT_EQ(g_current_band, BAND_80M);
    ASSERT_EQ(stub_gpio_set_count(LPF4_GPIO_Port, LPF4_Pin), 0);
}

/*============================================================================
 *  PTT idempotency
 *
 *  Both edge sources (EXTI5 and the TIM7 poll) reconcile against the live
 *  pin level, so a duplicate edge must never move the state machine or
 *  re-arm the 15 ms timer.
 *==========================================================================*/

TEST_CASE(repeat_press_before_release_is_a_noop)
{
    ptt_establish_idle_outputs();
    ptt_set_pressed(true);

    power_control_ptt_isr();
    ASSERT_EQ(ptt_state, PTT_STATE_TX_STARTING);

    /* Bounce: several more falling edges while already keying. */
    for (int i = 0; i < 5; i++) {
        power_control_ptt_isr();
        ASSERT_EQ(ptt_state, PTT_STATE_TX_STARTING);
    }

    power_relay_timer_callback();
    ASSERT_EQ(ptt_state, PTT_STATE_TX_RUNNING);

    /* And again now that it is running. */
    for (int i = 0; i < 5; i++) {
        power_control_ptt_isr();
        ASSERT_EQ(ptt_state, PTT_STATE_TX_RUNNING);
    }
    ASSERT_TRUE(g_tx_active);
    ASSERT_EQ(ptt_out_level(), GPIO_PIN_SET_E);
}

TEST_CASE(repeat_release_before_press_is_a_noop)
{
    ptt_establish_idle_outputs();

    ptt_set_pressed(false);
    for (int i = 0; i < 5; i++) {
        power_control_ptt_isr();
        ASSERT_EQ(ptt_state, PTT_STATE_IDLE);
    }
    ASSERT_FALSE(g_tx_active);
    ASSERT_EQ(ptt_out_level(), GPIO_PIN_RESET_E);
    /* Nothing was keyed, so no relay should have been touched at all. */
    ASSERT_EQ(stub_gpio_set_count(RELAY1_GPIO_Port, RELAY1_Pin), 0);
    ASSERT_EQ(stub_gpio_set_count(RELAY2_GPIO_Port, RELAY2_Pin), 0);
}

TEST_CASE(repeat_release_during_the_stop_delay_does_not_rearm)
{
    ptt_establish_idle_outputs();
    ptt_set_pressed(true);
    power_control_ptt_isr();
    power_relay_timer_callback();

    ptt_set_pressed(false);
    power_control_ptt_isr();
    ASSERT_EQ(ptt_state, PTT_STATE_TX_STOPPING);

    for (int i = 0; i < 5; i++) {
        power_control_ptt_isr();
        ASSERT_EQ(ptt_state, PTT_STATE_TX_STOPPING);
    }

    power_relay_timer_callback();
    ASSERT_EQ(ptt_state, PTT_STATE_IDLE);
}

TEST_CASE(bouncing_press_release_ends_where_the_pin_says)
{
    ptt_establish_idle_outputs();

    /* Ten bounces, then leave the pedal down. */
    for (int i = 0; i < 10; i++) {
        ptt_set_pressed(true);
        power_control_ptt_isr();
        ptt_set_pressed(false);
        power_control_ptt_isr();
    }
    ptt_set_pressed(true);
    power_control_ptt_isr();

    /* RELAY2 is still closed from before, so re-pressing completes the RF
     * path straight away instead of waiting out the delay again. */
    ASSERT_EQ(ptt_state, PTT_STATE_TX_RUNNING);
    ASSERT_EQ(ptt_out_level(), GPIO_PIN_SET_E);

    /* And the opposite: leave it up. */
    ptt_set_pressed(false);
    power_control_ptt_isr();
    power_relay_timer_callback();
    ASSERT_EQ(ptt_state, PTT_STATE_IDLE);
    ASSERT_EQ(ptt_out_level(), GPIO_PIN_RESET_E);
    ASSERT_FALSE(g_tx_active);
}

/*============================================================================
 *  power_ptt_poll() - release-safety fallback
 *==========================================================================*/

TEST_CASE(poll_unkeys_when_the_release_edge_never_arrives)
{
    ptt_establish_idle_outputs();

    /* Press arrives as an interrupt, as designed. */
    ptt_set_pressed(true);
    power_control_ptt_isr();
    power_relay_timer_callback();
    ASSERT_EQ(ptt_state, PTT_STATE_TX_RUNNING);
    ASSERT_EQ(ptt_out_level(), GPIO_PIN_SET_E);

    /* The operator lets go but the rising edge is lost - no ISR call. */
    ptt_set_pressed(false);
    power_ptt_poll();

    ASSERT_EQ(ptt_state, PTT_STATE_TX_STOPPING);
    ASSERT_EQ(ptt_out_level(), GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(RELAY1_GPIO_Port, RELAY1_Pin),
              GPIO_PIN_RESET_E);
    /* RELAY2 is still closed, so protection must keep running. */
    ASSERT_TRUE(g_tx_active);

    power_relay_timer_callback();
    ASSERT_EQ(ptt_state, PTT_STATE_IDLE);
    ASSERT_FALSE(g_tx_active);
}

TEST_CASE(poll_unkeys_from_the_starting_state_too)
{
    ptt_establish_idle_outputs();
    ptt_set_pressed(true);
    power_control_ptt_isr();
    ASSERT_EQ(ptt_state, PTT_STATE_TX_STARTING);

    /* Released before RELAY1 ever closed. */
    ptt_set_pressed(false);
    power_ptt_poll();
    ASSERT_EQ(ptt_state, PTT_STATE_TX_STOPPING);

    power_relay_timer_callback();
    ASSERT_EQ(ptt_state, PTT_STATE_IDLE);
    ASSERT_FALSE(g_tx_active);
}

TEST_CASE(poll_does_nothing_while_the_pedal_is_still_down)
{
    ptt_establish_idle_outputs();
    ptt_set_pressed(true);
    power_control_ptt_isr();

    /* Many polls while the request stands: must not re-arm the delay. */
    for (int i = 0; i < 10; i++) {
        power_ptt_poll();
        ASSERT_EQ(ptt_state, PTT_STATE_TX_STARTING);
    }
    power_relay_timer_callback();
    ASSERT_EQ(ptt_state, PTT_STATE_TX_RUNNING);
}

TEST_CASE(poll_is_a_noop_on_an_idle_amplifier)
{
    ptt_establish_idle_outputs();

    stub_gpio_reset_counters();
    for (int i = 0; i < 10; i++) {
        power_ptt_poll();
    }
    ASSERT_EQ(ptt_state, PTT_STATE_IDLE);
    ASSERT_EQ(stub_gpio_set_count(RELAY1_GPIO_Port, RELAY1_Pin), 0);
    ASSERT_EQ(stub_gpio_set_count(RELAY2_GPIO_Port, RELAY2_Pin), 0);
    ASSERT_EQ(stub_gpio_set_count(PTT_OUT_GPIO_Port, PTT_OUT_Pin), 0);
}

TEST_CASE(poll_force_releases_when_the_alert_latched)
{
    ptt_establish_idle_outputs();
    ptt_set_pressed(true);
    power_control_ptt_isr();
    power_relay_timer_callback();
    ASSERT_TRUE(g_tx_active);

    /* Protection latches while the pedal is still held down. */
    g_alert_flag = true;
    power_ptt_poll();

    ASSERT_EQ(ptt_state, PTT_STATE_IDLE);
    ASSERT_FALSE(g_tx_active);
    ASSERT_EQ(ptt_out_level(), GPIO_PIN_RESET_E);
}

/*============================================================================
 *  HV rail sequencing (PWR_ON + RF_BIAS)
 *==========================================================================*/

TEST_CASE(hv_on_sequences_pwr_on_then_bias)
{
    power_init_outputs();
    stub_gpio_reset_counters();
    power_psu_on();                   /* main supply latch first */
    stub_gpio_reset_counters();
    power_hv_on();

    /* Stage 1: PWR_ON comes up, the bias must NOT be on yet - the PA needs
     * HV_STAGE_DELAY_MS of settling before it is biased. */
    ASSERT_EQ(stub_gpio_get_last(PWR_ON_GPIO_Port, PWR_ON_Pin),
              GPIO_PIN_SET_E);         /* PG7 */
    ASSERT_EQ(stub_gpio_get_last(RF_BIAS_GPIO_Port, RF_BIAS_Pin),
              GPIO_PIN_RESET_E);       /* PG2 still down */
    ASSERT_TRUE(power_rail_busy());
    ASSERT_TRUE(g_hv_on);

    /* The main supply latch is NOT part of this sequence - it was already
     * set before the call and must not be touched by it. */
    ASSERT_EQ(stub_gpio_get_last(PSU_ON_GPIO_Port, PSU_ON_Pin),
              GPIO_PIN_SET_E);
    ASSERT_TRUE(g_psu_on);

    /* Stage 2: the monitor task's next tick biases the PA. */
    ASSERT_TRUE(power_rail_step());
    ASSERT_EQ(stub_gpio_get_last(RF_BIAS_GPIO_Port, RF_BIAS_Pin),
              GPIO_PIN_SET_E);         /* PG2 now up */
    ASSERT_FALSE(power_rail_busy());

    /* Settled: further steps are no-ops. */
    ASSERT_FALSE(power_rail_step());
}

TEST_CASE(hv_off_sequences_bias_then_pwr_on)
{
    power_init_outputs();
    stub_gpio_reset_counters();
    ptt_state = PTT_STATE_IDLE;
    power_psu_on();
    power_hv_on();
    (void)power_rail_step();        /* settle the on sequence first */
    stub_gpio_reset_counters();

    power_hv_off();

    /* Stage 1: the bias drops, but PWR_ON stays up for the HV_STAGE_DELAY_MS
     * gap. */
    ASSERT_EQ(stub_gpio_get_last(RF_BIAS_GPIO_Port, RF_BIAS_Pin),
              GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(PWR_ON_GPIO_Port, PWR_ON_Pin),
              GPIO_PIN_SET_E);        /* still up during the gap */
    ASSERT_TRUE(power_rail_busy());
    ASSERT_FALSE(g_hv_on);

    /* Stage 2: PWR_ON follows. */
    ASSERT_TRUE(power_rail_step());
    ASSERT_EQ(stub_gpio_get_last(PWR_ON_GPIO_Port, PWR_ON_Pin),
              GPIO_PIN_RESET_E);
    ASSERT_FALSE(power_rail_busy());

    /* Neither the main supply latch nor the relays were part of this. */
    ASSERT_TRUE(g_psu_on);
    ASSERT_EQ(stub_gpio_get_last(PSU_ON_GPIO_Port, PSU_ON_Pin),
              GPIO_PIN_SET_E);
    ASSERT_EQ(stub_gpio_get_last(RELAY1_GPIO_Port, RELAY1_Pin),
              GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(RELAY2_GPIO_Port, RELAY2_Pin),
              GPIO_PIN_RESET_E);
}

TEST_CASE(rail_abort_drops_a_pending_stage_without_touching_pins)
{
    stub_gpio_reset_counters();
    power_hv_on();                   /* stage 1: PWR_ON up, bias pending */

    power_rail_abort();              /* what the fault ISR does */

    /* No further stage may fire and re-assert anything. */
    ASSERT_FALSE(power_rail_step());
    ASSERT_EQ(stub_gpio_get_last(RF_BIAS_GPIO_Port, RF_BIAS_Pin),
              GPIO_PIN_RESET_E);
    ASSERT_FALSE(power_rail_busy());
}

TEST_CASE(hv_on_is_idempotent_once_settled)
{
    power_hv_on();
    (void)power_rail_step();
    stub_gpio_reset_counters();

    power_hv_on();                   /* already on: must be a no-op */
    ASSERT_EQ(stub_gpio_set_count(PWR_ON_GPIO_Port, PWR_ON_Pin), 0U);
    ASSERT_FALSE(power_rail_busy());
}

TEST_CASE(psu_latch_is_independent_of_the_hv_rail)
{
    /* PG5 is a plain latch with no timing: on, off, and it never touches
     * PWR_ON / RF_BIAS.  This is the counterpart to the HV tests above and
     * the reason the two got separate names. */
    stub_gpio_reset_counters();
    power_init_outputs();

    power_psu_on();
    ASSERT_EQ(stub_gpio_get_last(PSU_ON_GPIO_Port, PSU_ON_Pin),
              GPIO_PIN_SET_E);
    ASSERT_EQ(stub_gpio_get_last(PWR_ON_GPIO_Port, PWR_ON_Pin),
              GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(RF_BIAS_GPIO_Port, RF_BIAS_Pin),
              GPIO_PIN_RESET_E);

    power_psu_off();
    ASSERT_EQ(stub_gpio_get_last(PSU_ON_GPIO_Port, PSU_ON_Pin),
              GPIO_PIN_RESET_E);
    ASSERT_FALSE(g_psu_on);
    ASSERT_FALSE(g_hv_on);
    ASSERT_FALSE(power_rail_busy());     /* no sequencing involved */
}

TEST_CASE(power_init_outputs_leaves_everything_inactive)
{
    stub_gpio_reset_counters();

    power_init_outputs();

    ASSERT_EQ(stub_gpio_get_last(RELAY1_GPIO_Port, RELAY1_Pin),
              GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(RELAY2_GPIO_Port, RELAY2_Pin),
              GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(PWR_ON_GPIO_Port,  PWR_ON_Pin),
              GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(RF_BIAS_GPIO_Port, RF_BIAS_Pin),
              GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(PSU_ON_GPIO_Port,  PSU_ON_Pin),
              GPIO_PIN_RESET_E);
    ASSERT_EQ(stub_gpio_get_last(RF_IN_PROTECT_GPIO_Port,
                                 RF_IN_PROTECT_Pin),
              GPIO_PIN_RESET_E);
    /* Default band is 15-10 m per spec */
    ASSERT_EQ(g_current_band, BAND_15M_10M);
}

/*============================================================================
 *  Fan speed hysteresis
 *============================================================================*/

TEST_CASE(fan_speed_starts_at_50_percent_below_60c)
{
    /* power_init_fan_pwm() sets TIM3 CCR3; verify mapping only. */
    power_init_fan_pwm();

    /* 59.9 C -> still 50 % */
    power_update_fan(599);
    ASSERT_EQ(g_fan_speed, FAN_SPEED_LOW);

    /* 60.0 C -> 80 % */
    power_update_fan(600);
    ASSERT_EQ(g_fan_speed, FAN_SPEED_MED);
}

TEST_CASE(fan_speed_reaches_100_above_70c)
{
    power_init_fan_pwm();
    power_update_fan(700);
    ASSERT_EQ(g_fan_speed, FAN_SPEED_HIGH);
}

TEST_CASE(fan_speed_holds_100_until_60c_on_the_way_down)
{
    power_init_fan_pwm();

    /* Ramp up to 100 % */
    power_update_fan(750);
    ASSERT_EQ(g_fan_speed, FAN_SPEED_HIGH);

    /* Coming down: 100 % -> 80 % only after 70-10 = 60 C */
    power_update_fan(650);
    ASSERT_EQ(g_fan_speed, FAN_SPEED_HIGH);   /* still 100 % */
    power_update_fan(599);
    ASSERT_EQ(g_fan_speed, FAN_SPEED_MED);     /* dropped to 80 % */
}

TEST_CASE(fan_speed_drops_to_50_below_50c_on_the_way_down)
{
    power_init_fan_pwm();
    power_update_fan(750);      /* 100 % */
    power_update_fan(700);      /* 100 % (still above the 60 C release point) */
    power_update_fan(500);      /* <= 50 C -> 50 % */
    ASSERT_EQ(g_fan_speed, FAN_SPEED_LOW);
}

/*============================================================================
 *  Fan PWM duty
 *==========================================================================*/

TEST_CASE(fan_pwm_duty_matches_speed_percent)
{
    power_init_fan_pwm();

    /* ARR = 999 -> 1000 ticks; pulse = (pct * 1000 + 50) / 100 */
    power_set_fan_speed(0);
    ASSERT_EQ(htim3.Instance->CCR3, 0);
    power_set_fan_speed(50);
    ASSERT_EQ(htim3.Instance->CCR3, 500);
    power_set_fan_speed(100);
    ASSERT_EQ(htim3.Instance->CCR3, 1000);
}

TEST_CASE(fan_speed_percent_is_clamped_to_100)
{
    power_init_fan_pwm();
    power_set_fan_speed(200);
    ASSERT_EQ(htim3.Instance->CCR3, 1000);
    ASSERT_EQ(g_fan_speed, FAN_SPEED_HIGH);
}

TEST_CASE(pump_pwm_duty_matches_speed_percent)
{
    power_init_fan_pwm();
    power_set_pump_speed(25);
    ASSERT_EQ(htim4.Instance->CCR4, 250);
    power_set_pump_speed(100);
    ASSERT_EQ(htim4.Instance->CCR4, 1000);
}

/*============================================================================
 *  Suite table
 *==========================================================================*/

const test_entry_t g_suite_power_control[] = {
    TEST_ENTRY(switch_band_15m_activates_lpf1_only),
    TEST_ENTRY(switch_band_160m_activates_lpf5_only),
    TEST_ENTRY(switch_band_80m_activates_lpf4_only),
    TEST_ENTRY(switch_band_40m30m_activates_lpf3_only),
    TEST_ENTRY(switch_band_20m17m_activates_lpf2_only),
    TEST_ENTRY(switch_band_fault_falls_back_to_15m),
    TEST_ENTRY(switch_band_none_falls_back_to_15m),
    TEST_ENTRY(switch_band_releases_previous_band),
    TEST_ENTRY(switch_band_index_increases_with_wavelength),
    TEST_ENTRY(switch_band_never_activates_two_bands_at_once),
    TEST_ENTRY(ptt_press_from_idle_closes_relay2_only),
    TEST_ENTRY(ptt_relay_timer_then_closes_relay1),
    TEST_ENTRY(ptt_release_from_running_opens_relay1_then_delayed_relay2),
    TEST_ENTRY(ptt_repress_during_stopping_cancels_the_delay),
    TEST_ENTRY(ptt_blocked_until_im_calibration_completes),
    TEST_ENTRY(ptt_blocked_while_alert_active),
    TEST_ENTRY(ptt_press_while_idle_after_stop_is_noop),
    TEST_ENTRY(ptt_flag_is_cleared_by_task_api),
    TEST_ENTRY(ptt_press_raises_tx_active),
    TEST_ENTRY(ptt_release_clears_tx_active),
    TEST_ENTRY(ptt_never_touches_the_supply_flag),
    TEST_ENTRY(ptt_while_hv_off_only_moves_the_relays),
    TEST_ENTRY(ptt_blocked_by_alert_clears_tx_active),
    TEST_ENTRY(ptt_blocked_during_stop_sequence_also_clears_tx_active),
    TEST_ENTRY(ptt_repress_during_stop_keeps_tx_active),
    TEST_ENTRY(relays_api_keeps_tx_active_in_step),
    TEST_ENTRY(ptt_out_starts_released_after_init),
    TEST_ENTRY(ptt_out_stays_low_during_the_relay_delay),
    TEST_ENTRY(ptt_out_drops_immediately_on_release),
    TEST_ENTRY(ptt_out_reasserts_when_a_stop_is_cancelled),
    TEST_ENTRY(ptt_out_drops_when_an_alert_blocks_ptt),
    TEST_ENTRY(ptt_out_stays_asserted_while_the_hv_rail_sequences_down),
    TEST_ENTRY(ptt_out_drops_when_the_supply_is_switched_off),
    TEST_ENTRY(ptt_out_is_high_active),
    TEST_ENTRY(power_init_leaves_the_external_psu_and_the_pa_off),
    TEST_ENTRY(power_hv_enable_engages_the_rail_only_once),
    TEST_ENTRY(power_hv_disable_drops_the_rail_but_leaves_supply_and_relays),
    TEST_ENTRY(power_standby_keeps_the_hv_rail_but_drops_the_rf_path),
    TEST_ENTRY(ptt_is_refused_while_the_hv_rail_is_still_sequencing),
    TEST_ENTRY(power_tx_enable_keys_the_amplifier),
    TEST_ENTRY(power_tx_enable_is_refused_until_the_boot_gate_opens),
    TEST_ENTRY(power_tx_enable_ignores_the_psu_and_hv_rail_state),
    TEST_ENTRY(power_tx_enable_is_refused_while_calibrating),
    TEST_ENTRY(power_tx_enable_is_refused_while_alert_latched),
    TEST_ENTRY(power_get_ptt_state_reads_the_active_low_input),
    TEST_ENTRY(power_update_band_selects_the_matching_lpf),
    TEST_ENTRY(power_update_band_ignores_out_of_band_frequencies),
    TEST_ENTRY(power_update_band_does_not_disturb_a_matching_band),
    TEST_ENTRY(repeat_press_before_release_is_a_noop),
    TEST_ENTRY(repeat_release_before_press_is_a_noop),
    TEST_ENTRY(repeat_release_during_the_stop_delay_does_not_rearm),
    TEST_ENTRY(bouncing_press_release_ends_where_the_pin_says),
    TEST_ENTRY(poll_unkeys_when_the_release_edge_never_arrives),
    TEST_ENTRY(poll_unkeys_from_the_starting_state_too),
    TEST_ENTRY(poll_does_nothing_while_the_pedal_is_still_down),
    TEST_ENTRY(poll_is_a_noop_on_an_idle_amplifier),
    TEST_ENTRY(poll_force_releases_when_the_alert_latched),
    TEST_ENTRY(hv_on_sequences_pwr_on_then_bias),
    TEST_ENTRY(hv_off_sequences_bias_then_pwr_on),
    TEST_ENTRY(rail_abort_drops_a_pending_stage_without_touching_pins),
    TEST_ENTRY(hv_on_is_idempotent_once_settled),
    TEST_ENTRY(psu_latch_is_independent_of_the_hv_rail),
    TEST_ENTRY(power_init_outputs_leaves_everything_inactive),
    TEST_ENTRY(fan_speed_starts_at_50_percent_below_60c),
    TEST_ENTRY(fan_speed_reaches_100_above_70c),
    TEST_ENTRY(fan_speed_holds_100_until_60c_on_the_way_down),
    TEST_ENTRY(fan_speed_drops_to_50_below_50c_on_the_way_down),
    TEST_ENTRY(fan_pwm_duty_matches_speed_percent),
    TEST_ENTRY(fan_speed_percent_is_clamped_to_100),
    TEST_ENTRY(pump_pwm_duty_matches_speed_percent),
    TEST_SUITE_END
};