/**
 * @file    test_fan_tach.c
 * @brief   Unit tests for the REAL fan_tach.c implementation.
 *
 * @details Covers
 *            - fan_tach_tick() counts exactly one pulse per falling edge
 *            - the edge detector does not double-count a level held low
 *            - fan_tach_calculate_rpm() converts pulses to RPM using the
 *              *measured* window, not a hardcoded period
 *            - a window that is too short (or absurdly long) is rejected
 *            - FAN2 mirrors FAN1 while the hardware cannot do otherwise
 *
 *          The tests drive the real ISR body by calling fan_tach_tick()
 *          directly after stubbing the pin levels; no logic is duplicated.
 */

#include "test_harness.h"
#include "fan_tach.h"

#include <stdint.h>

/* Must match FAN_POLES in fan_tach.c.  Two pulses per revolution is the
 * de-facto standard for 4-wire PC / axial fans (Hall sensor, one magnet
 * pair, two edges per turn). */
#define TEST_POLES            2U

/* The measured window must land inside [FAN_WINDOW_MIN_MS,
 * FAN_WINDOW_MAX_MS] or the reading is (correctly) not updated. */
#define TEST_WINDOW_MS        1000U

static void tach_reset(void)
{
    /* Idle high, as the pull-up leaves it between pulses. */
    stub_gpio_set(FAN1_SENSE_GPIO_Port, FAN1_SENSE_Pin, 1U);
    stub_gpio_set(PUMP_SENSE_GPIO_Port, PUMP_SENSE_Pin, 1U);
    test_wall_clock_reset();
    fan_tach_init();
}

/* One complete tach pulse: high -> low -> high.  The fan_tach_tick() at
 * the end is what re-arms the edge detector for the next pulse. */
static void tach_pulse_fan(void)
{
    stub_gpio_set(FAN1_SENSE_GPIO_Port, FAN1_SENSE_Pin, 0U);
    fan_tach_tick();
    stub_gpio_set(FAN1_SENSE_GPIO_Port, FAN1_SENSE_Pin, 1U);
    fan_tach_tick();
}

static void tach_pulse_pump(void)
{
    stub_gpio_set(PUMP_SENSE_GPIO_Port, PUMP_SENSE_Pin, 0U);
    fan_tach_tick();
    stub_gpio_set(PUMP_SENSE_GPIO_Port, PUMP_SENSE_Pin, 1U);
    fan_tach_tick();
}

/* Deliver `n` fan pulses spread over `window_ms`, then close the window.
 * Returns the RPM that fan_tach_calculate_rpm() published. */
static uint32_t tach_run_window(uint32_t n, uint32_t window_ms)
{
    for (uint32_t i = 0; i < n; i++) {
        tach_pulse_fan();
    }
    test_wall_clock_advance(window_ms);
    fan_tach_calculate_rpm();
    return fan_tach_get_fan_rpm();
}

/*============================================================================
 *  fan_tach_tick() - edge counting
 *==========================================================================*/

TEST_CASE(a_single_falling_edge_counts_one_pulse)
{
    tach_reset();

    stub_gpio_set(FAN1_SENSE_GPIO_Port, FAN1_SENSE_Pin, 0U);
    fan_tach_tick();                       /* high -> low: one pulse */
    stub_gpio_set(FAN1_SENSE_GPIO_Port, FAN1_SENSE_Pin, 1U);
    fan_tach_tick();                       /* low -> high: not counted */

    test_wall_clock_advance(TEST_WINDOW_MS);
    fan_tach_calculate_rpm();
    /* 1 pulse / 2 poles = half a revolution per window. */
    ASSERT_EQ(fan_tach_get_fan_rpm(), 60000U / (TEST_POLES * TEST_WINDOW_MS));
}

TEST_CASE(a_level_held_low_is_counted_only_once)
{
    tach_reset();

    /* The ISR samples the same low level many times - a real fan whose
     * pulse is wider than the sample period looks exactly like this.
     * Counting every sample would inflate the RPM without bound. */
    stub_gpio_set(FAN1_SENSE_GPIO_Port, FAN1_SENSE_Pin, 0U);
    for (int i = 0; i < 50; i++) {
        fan_tach_tick();
    }
    stub_gpio_set(FAN1_SENSE_GPIO_Port, FAN1_SENSE_Pin, 1U);
    fan_tach_tick();

    test_wall_clock_advance(TEST_WINDOW_MS);
    fan_tach_calculate_rpm();
    ASSERT_EQ(fan_tach_get_fan_rpm(), 60000U / (TEST_POLES * TEST_WINDOW_MS));
}

TEST_CASE(a_high_level_produces_no_pulses_at_all)
{
    tach_reset();

    for (int i = 0; i < 100; i++) {
        fan_tach_tick();                   /* pin never leaves idle */
    }
    test_wall_clock_advance(TEST_WINDOW_MS);
    fan_tach_calculate_rpm();
    ASSERT_EQ(fan_tach_get_fan_rpm(), 0U);
}

TEST_CASE(the_fan_and_the_pump_are_counted_separately)
{
    tach_reset();

    for (int i = 0; i < 20; i++) {
        tach_pulse_fan();
    }
    for (int i = 0; i < 60; i++) {
        tach_pulse_pump();
    }
    test_wall_clock_advance(TEST_WINDOW_MS);
    fan_tach_calculate_rpm();

    ASSERT_EQ(fan_tach_get_fan_rpm(),  20U * 60000U / (TEST_POLES * TEST_WINDOW_MS));
    ASSERT_EQ(fan_tach_get_pump_rpm(), 60U * 60000U / (TEST_POLES * TEST_WINDOW_MS));
}

/* A coolant pump is not a PC fan, so PUMP_POLES is a separate constant.
 * Feeding the same pulse count to both pins and getting the same answer
 * back is the only way to catch the two constants having been collapsed
 * into one - which would silently misreport the pump by exactly the
 * ratio between them.  Both default to 2 today, so this passes either
 * way; it is here to fail the moment someone edits one and not the other
 * without meaning to. */
TEST_CASE(each_channel_divides_by_its_own_pulse_per_revolution)
{
    tach_reset();

    for (int i = 0; i < 40; i++) {
        tach_pulse_fan();
        tach_pulse_pump();
    }
    test_wall_clock_advance(TEST_WINDOW_MS);
    fan_tach_calculate_rpm();

    /* Equal pulse counts must give answers proportional to 1/PPR, i.e.
     * the fan number scaled by FAN_POLES/PUMP_POLES.  Written as a ratio
     * so the expectation stays right when either constant changes. */
    uint32_t fan_rpm  = fan_tach_get_fan_rpm();
    uint32_t pump_rpm = fan_tach_get_pump_rpm();
    ASSERT_EQ((uint64_t)fan_rpm * PUMP_POLES,
              (uint64_t)pump_rpm * FAN_POLES);
}

/*============================================================================
 *  fan_tach_calculate_rpm() - the conversion
 *==========================================================================*/

TEST_CASE(pulses_convert_to_rpm_at_two_pulses_per_revolution)
{
    tach_reset();

    /* 3000 rpm x 2 pulses/rev = 100 pulses per second, so 100 pulses in
     * a 1 s window must come back as 3000.  This is the "divide by the
     * poles" case: a naive pulses*60/seconds would report 6000. */
    ASSERT_EQ(tach_run_window(100U, 1000U), 3000U);
}

TEST_CASE(the_window_length_is_measured_not_assumed)
{
    tach_reset();

    /* Same pulse count, half the window, twice the RPM.  The old code
     * divided by a compile-time 100 ms, so calling it on a 1 s tick
     * reported every reading 10x too high; measuring the elapsed time
     * is what makes the caller period irrelevant. */
    ASSERT_EQ(tach_run_window(50U, 1000U), 1500U);
    ASSERT_EQ(tach_run_window(50U, 500U),  3000U);
}

TEST_CASE(a_short_window_publishes_nothing_rather_than_noise)
{
    tach_reset();

    /* Prime with a good reading so there is something to protect. */
    ASSERT_EQ(tach_run_window(100U, TEST_WINDOW_MS), 3000U);

    /* 10 ms is below FAN_WINDOW_MIN_MS: one stray pulse in a window that
     * short would otherwise read as a few thousand rpm. */
    for (int i = 0; i < 5; i++) {
        tach_pulse_fan();
    }
    test_wall_clock_advance(10U);
    fan_tach_calculate_rpm();
    ASSERT_EQ(fan_tach_get_fan_rpm(), 3000U);
}

TEST_CASE(a_stalled_caller_does_not_publish_a_wild_reading)
{
    tach_reset();

    ASSERT_EQ(tach_run_window(100U, TEST_WINDOW_MS), 3000U);

    /* The task was starved for 30 s.  Dividing by the real elapsed time
     * would be *correct* but useless (the average over 30 s says nothing
     * about now), so the last good reading is held instead. */
    test_wall_clock_advance(30000U);
    fan_tach_calculate_rpm();
    ASSERT_EQ(fan_tach_get_fan_rpm(), 3000U);
}

TEST_CASE(the_counters_are_cleared_between_windows)
{
    tach_reset();

    ASSERT_EQ(tach_run_window(100U, TEST_WINDOW_MS), 3000U);
    /* Second window with no pulses at all: if the counter had not been
     * reset this would still report the first window's 3000. */
    ASSERT_EQ(tach_run_window(0U, TEST_WINDOW_MS), 0U);
}

TEST_CASE(a_fast_spinner_does_not_overflow_the_conversion)
{
    tach_reset();

    /* 20000 rpm x 2 = 667 pulses per second.  count * 60000 = 40.2e6,
     * which fits in 32 bits - but only just, and only because the window
     * is 1 s.  The uint64_t intermediate is what keeps this honest if
     * the window ever grows. */
    ASSERT_EQ(tach_run_window(667U, 1000U), 20010U);
}

/*============================================================================
 *  FAN2
 *==========================================================================*/

TEST_CASE(fan2_mirrors_fan1_while_they_share_a_pwm_and_a_tach_pin)
{
    tach_reset();

    /* The pump pin (PD5) is the one FAN2 could be wired to, so spin it
     * at a different rate: if FAN2 were reading its own pin it would
     * report the pump number, not the fan one. */
    for (int i = 0; i < 100; i++) {
        tach_pulse_fan();
    }
    for (int i = 0; i < 20; i++) {
        tach_pulse_pump();
    }
    test_wall_clock_advance(TEST_WINDOW_MS);
    fan_tach_calculate_rpm();

    ASSERT_EQ(fan_tach_get_fan_rpm(),  3000U);
    ASSERT_EQ(fan_tach_get_pump_rpm(),  600U);
#if FAN2_TACH_MIRRORS_FAN1
    ASSERT_EQ(fan_tach_get_fan2_rpm(), fan_tach_get_fan_rpm());
#else
    ASSERT_EQ(fan_tach_get_fan2_rpm(), fan_tach_get_pump_rpm());
#endif
}

TEST_CASE(fan2_reports_zero_alongside_a_stopped_fan1)
{
    tach_reset();

    ASSERT_EQ(tach_run_window(0U, TEST_WINDOW_MS), 0U);
    ASSERT_EQ(fan_tach_get_fan2_rpm(), 0U);
}

const test_entry_t g_suite_fan_tach[] = {
    TEST_ENTRY(a_single_falling_edge_counts_one_pulse),
    TEST_ENTRY(a_level_held_low_is_counted_only_once),
    TEST_ENTRY(a_high_level_produces_no_pulses_at_all),
    TEST_ENTRY(the_fan_and_the_pump_are_counted_separately),
    TEST_ENTRY(pulses_convert_to_rpm_at_two_pulses_per_revolution),
    TEST_ENTRY(the_window_length_is_measured_not_assumed),
    TEST_ENTRY(a_short_window_publishes_nothing_rather_than_noise),
    TEST_ENTRY(a_stalled_caller_does_not_publish_a_wild_reading),
    TEST_ENTRY(the_counters_are_cleared_between_windows),
    TEST_ENTRY(a_fast_spinner_does_not_overflow_the_conversion),
    TEST_ENTRY(fan2_mirrors_fan1_while_they_share_a_pwm_and_a_tach_pin),
    TEST_ENTRY(fan2_reports_zero_alongside_a_stopped_fan1),
    TEST_SUITE_END
};
