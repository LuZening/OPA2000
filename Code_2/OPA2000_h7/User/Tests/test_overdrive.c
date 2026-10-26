/**
 * @file    test_overdrive.c
 * @brief   Unit tests for overdrive.c: the 200 ms peak window, the brightness
 *          ramp, the trip point, the red flash and the one-shot beep request.
 *
 * @details overdrive.c has no hardware dependency at all - it takes a raw
 *          code and a millisecond count as arguments - so the tests drive it
 *          directly and the window arithmetic is checked exactly rather than
 *          through a sleep.
 */

#include "test_harness.h"
#include "overdrive.h"

/* The ADC task samples at 50 ms, so four passes make the 200 ms window. */
#define TICK_MS 50U

static void tick_n(uint32_t start_ms, uint16_t raw, uint32_t n)
{
    for (uint32_t i = 0U; i < n; i++) {
        overdrive_tick(start_ms + i * TICK_MS, raw);
    }
}

TEST_CASE(overdrive_starts_dark_and_silent)
{
    overdrive_init();

    ASSERT_EQ(overdrive_peak_raw(), 0U);
    ASSERT_EQ(overdrive_led_bright(), 0U);
    ASSERT_FALSE(overdrive_is_tripped());
    ASSERT_FALSE(overdrive_take_beep());
}

TEST_CASE(overdrive_led_stays_dark_below_the_lit_floor)
{
    overdrive_init();
    overdrive_tick(0U, OVERDRIVE_LED_MIN_RAW);

    /* At the floor the ramp contributes nothing: it is the point where the
     * LED is still considered off. */
    ASSERT_EQ(overdrive_led_bright(), 0U);
    ASSERT_FALSE(overdrive_is_tripped());
    ASSERT_FALSE(overdrive_take_beep());
}

TEST_CASE(overdrive_brightness_ramps_between_the_floor_and_the_trip_point)
{
    uint32_t mid = ((uint32_t)OVERDRIVE_LED_MIN_RAW +
                    (uint32_t)OVERDRIVE_TRIP_RAW) / 2U;

    overdrive_init();
    overdrive_tick(0U, (uint16_t)mid);

    /* Halfway through the range is roughly half brightness; the exact value
     * depends on integer division, so assert the band rather than a number. */
    ASSERT_GE(overdrive_led_bright(), 120U);
    ASSERT_LE(overdrive_led_bright(), 135U);
    ASSERT_FALSE(overdrive_is_tripped());
}

TEST_CASE(overdrive_peak_is_the_maximum_of_the_last_200_ms)
{
    overdrive_init();

    overdrive_tick(0U, OVERDRIVE_TRIP_RAW);

    /* Three more passes (150 ms) still have the old sample in the window. */
    tick_n(TICK_MS, 0U, 3U);
    ASSERT_EQ(overdrive_peak_raw(), OVERDRIVE_TRIP_RAW);

    /* The fourth zero pass evicts it: 200 ms have now elapsed. */
    tick_n(4U * TICK_MS, 0U, 1U);
    ASSERT_EQ(overdrive_peak_raw(), 0U);
    ASSERT_FALSE(overdrive_is_tripped());
}

TEST_CASE(overdrive_trip_lights_the_led_and_requests_one_beep)
{
    overdrive_init();
    overdrive_tick(0U, OVERDRIVE_TRIP_RAW);

    ASSERT_TRUE(overdrive_is_tripped());
    ASSERT_EQ(overdrive_led_bright(), 255U);

    ASSERT_TRUE(overdrive_take_beep());
    /* The request is consumed by the first read: the caller starts the
     * buzzer exactly once per excursion. */
    ASSERT_FALSE(overdrive_take_beep());
}

TEST_CASE(overdrive_does_not_rearm_the_beep_while_the_input_stays_hot)
{
    overdrive_init();

    overdrive_tick(0U, OVERDRIVE_TRIP_RAW);
    ASSERT_TRUE(overdrive_take_beep());

    /* Ten more passes at full level: still the same excursion, so no second
     * request - the buzzer cycle already started is allowed to finish. */
    tick_n(TICK_MS, OVERDRIVE_TRIP_RAW, 10U);
    ASSERT_FALSE(overdrive_take_beep());

    /* Dropping below the trip point re-arms the latch ... */
    tick_n(11U * TICK_MS, 0U, 4U);
    ASSERT_FALSE(overdrive_is_tripped());

    /* ... so the next excursion gets its own beep. */
    overdrive_tick(20U * TICK_MS, OVERDRIVE_TRIP_RAW);
    ASSERT_TRUE(overdrive_take_beep());
}

TEST_CASE(overdrive_led_flashes_red_at_two_hertz_while_tripped)
{
    overdrive_init();

    /* Bright in the first half of the 500 ms cycle ... */
    overdrive_tick(0U, OVERDRIVE_TRIP_RAW);
    ASSERT_EQ(overdrive_led_bright(), 255U);

    /* ... dark in the second half ... */
    overdrive_tick(OVERDRIVE_FLASH_HALF_MS, OVERDRIVE_TRIP_RAW);
    ASSERT_EQ(overdrive_led_bright(), 0U);

    /* ... and bright again one full cycle later. */
    overdrive_tick(2U * OVERDRIVE_FLASH_HALF_MS, OVERDRIVE_TRIP_RAW);
    ASSERT_EQ(overdrive_led_bright(), 255U);
}
