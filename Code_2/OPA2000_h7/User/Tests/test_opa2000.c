/**
 * @file    test_opa2000.c
 * @brief   Unit tests for opa2000.c pure algorithms:
 *            - frequency_to_band()
 *            - crc32_calculate()
 *            - calculate_swr_x100()
 *
 * @note    These tests run on the host (x86_64) without any STM32
 *          hardware or cross-compile toolchain.  opa2000_init() is NOT
 *          called because it touches volatile global state that
 *          requires FreeRTOS headers we deliberately don't include
 *          here.
 */

#include "test_harness.h"
#include "opa2000.h"

#include <string.h>

/*============================================================================
 *  frequency_to_band()
 *============================================================================*/

TEST_CASE(freq_to_band_below_160m_returns_none)
{
    ASSERT_EQ(frequency_to_band(0),       BAND_NONE);
    ASSERT_EQ(frequency_to_band(1000000),  BAND_NONE);          /* 1 MHz */
    ASSERT_EQ(frequency_to_band(1799999),  BAND_NONE);
}

TEST_CASE(freq_to_band_160m_window)
{
    ASSERT_EQ(frequency_to_band(1800000),  BAND_160M);
    ASSERT_EQ(frequency_to_band(1900000),  BAND_160M);
    ASSERT_EQ(frequency_to_band(2100000),  BAND_160M);
    /* below window still none, above is fault */
    ASSERT_EQ(frequency_to_band(1799999),  BAND_NONE);
    ASSERT_EQ(frequency_to_band(2100001),  BAND_FAULT);
}

TEST_CASE(freq_to_band_80m_window)
{
    /* Gap between 160m max (2.1 MHz) and 80m min (3.5 MHz) -> FAULT */
    ASSERT_EQ(frequency_to_band(3000000),  BAND_FAULT);
    ASSERT_EQ(frequency_to_band(3500000),  BAND_80M);
    ASSERT_EQ(frequency_to_band(3700000),  BAND_80M);
    ASSERT_EQ(frequency_to_band(3900000),  BAND_80M);
    ASSERT_EQ(frequency_to_band(3900001),  BAND_FAULT);
}

TEST_CASE(freq_to_band_40m_30m_window)
{
    ASSERT_EQ(frequency_to_band(3900001),  BAND_FAULT);
    ASSERT_EQ(frequency_to_band(7000000),  BAND_40M_30M);
    ASSERT_EQ(frequency_to_band(9000000),  BAND_40M_30M);
    ASSERT_EQ(frequency_to_band(10150000), BAND_40M_30M);
    ASSERT_EQ(frequency_to_band(10150001), BAND_FAULT);
}

TEST_CASE(freq_to_band_20m_17m_window)
{
    ASSERT_EQ(frequency_to_band(14000000), BAND_20M_17M);
    ASSERT_EQ(frequency_to_band(18168000), BAND_20M_17M);
    ASSERT_EQ(frequency_to_band(18168001), BAND_FAULT);
}

TEST_CASE(freq_to_band_15m_10m_window)
{
    ASSERT_EQ(frequency_to_band(21000000), BAND_15M_10M);
    ASSERT_EQ(frequency_to_band(29700000), BAND_15M_10M);
    ASSERT_EQ(frequency_to_band(29700001), BAND_FAULT);
}

TEST_CASE(freq_to_band_extreme_high_is_fault)
{
    ASSERT_EQ(frequency_to_band(100000000), BAND_FAULT);
    ASSERT_EQ(frequency_to_band(0xFFFFFFFFu), BAND_FAULT);
}

/*============================================================================
 *  crc32_calculate()
 *
 *  Standard IEEE 802.3 / RFC 3720 test vector: "123456789" -> 0xCBF43926
 *============================================================================*/

TEST_CASE(crc32_ieee_test_vector_1)
{
    const uint8_t data[] = "123456789";
    uint32_t got = crc32_calculate((const uint8_t *)data, 9);
    /* The classic IEEE 802.3 CRC32 of ASCII "123456789" */
    ASSERT_EQ(got, 0xCBF43926U);
}

TEST_CASE(crc32_empty_input)
{
    /* CRC of empty buffer = 0 */
    ASSERT_EQ(crc32_calculate(NULL, 0), 0x00000000U);
    ASSERT_EQ(crc32_calculate((const uint8_t *)"", 0), 0x00000000U);
}

TEST_CASE(crc32_single_zero)
{
    const uint8_t data = 0x00;
    ASSERT_EQ(crc32_calculate(&data, 1), 0xD202EF8DU);
}

TEST_CASE(crc32_single_ff)
{
    const uint8_t data = 0xFF;
    ASSERT_EQ(crc32_calculate(&data, 1), 0xFF000000U);
}

TEST_CASE(crc32_incremental_matches)
{
    /* CRC("abc") computed in one shot must equal CRC("a") + CRC("b") + CRC("c") */
    const uint8_t full[] = "abc";
    uint32_t crc_full = crc32_calculate(full, 3);

    uint32_t acc = 0xFFFFFFFFU;
    for (uint32_t i = 0; i < 3; i++) {
        acc ^= full[i];
        for (int b = 0; b < 8; b++) {
            acc = (acc >> 1) ^ (0xEDB88320U & (uint32_t)-(int32_t)(acc & 1));
        }
    }
    ASSERT_EQ(crc_full, acc ^ 0xFFFFFFFFU);
}

/*============================================================================
 *  calculate_swr_x100()
 *============================================================================*/

TEST_CASE(swr_zero_power_returns_one)
{
    /* Below 10W forward, SWR is forced to 1.0 (=100 x100) per spec */
    ASSERT_EQ(calculate_swr_x100(0, 0),       100);
    ASSERT_EQ(calculate_swr_x100(9999, 0),   100);   /* 9.999 W */
    ASSERT_EQ(calculate_swr_x100(9999, 500), 100);   /* even with reverse */
}

TEST_CASE(swr_perfect_match_returns_one)
{
    /* P_rev = 0 -> SWR = 1.0 exactly */
    ASSERT_EQ(calculate_swr_x100(100000, 0),    100);
    ASSERT_EQ(calculate_swr_x100(1500000, 0),  100);
}

TEST_CASE(swr_invalid_clamped_to_999)
{
    /* Rev >= Fwd (impossible) clamps to 999 (=9.99 SWR) */
    ASSERT_EQ(calculate_swr_x100(100000, 100000),  999);
    ASSERT_EQ(calculate_swr_x100(100000, 200000),  999);
    ASSERT_EQ(calculate_swr_x100(100000, -100),    100);  /* negative rev -> 100 */
}

TEST_CASE(swr_typical_value)
{
    /* Forward 100 W, reflected 11 W -> |Gamma| ~ sqrt(0.11) ~ 0.3317
     * SWR = (1+0.3317)/(1-0.3317) ~ 1.992 -> 199 x100 (within +-5) */
    int32_t swr = calculate_swr_x100(100000, 11000);
    ASSERT_GE(swr, 195);
    ASSERT_LE(swr, 205);
}

TEST_CASE(swr_monotonic_in_reflection)
{
    /* As reverse grows, SWR must grow */
    int32_t swr_low  = calculate_swr_x100(100000, 1000);
    int32_t swr_mid  = calculate_swr_x100(100000, 11000);
    int32_t swr_high = calculate_swr_x100(100000, 30000);
    ASSERT_LT(swr_low,  swr_mid);
    ASSERT_LT(swr_mid,  swr_high);
    /* And capped at 9.99 */
    ASSERT_LE(swr_high, 999);
}

/*============================================================================
 *  Suite table
 *============================================================================*/

const test_entry_t g_suite_opa2000[] = {
    TEST_ENTRY(freq_to_band_below_160m_returns_none),
    TEST_ENTRY(freq_to_band_160m_window),
    TEST_ENTRY(freq_to_band_80m_window),
    TEST_ENTRY(freq_to_band_40m_30m_window),
    TEST_ENTRY(freq_to_band_20m_17m_window),
    TEST_ENTRY(freq_to_band_15m_10m_window),
    TEST_ENTRY(freq_to_band_extreme_high_is_fault),
    TEST_ENTRY(crc32_ieee_test_vector_1),
    TEST_ENTRY(crc32_empty_input),
    TEST_ENTRY(crc32_single_zero),
    TEST_ENTRY(crc32_single_ff),
    TEST_ENTRY(crc32_incremental_matches),
    TEST_ENTRY(swr_zero_power_returns_one),
    TEST_ENTRY(swr_perfect_match_returns_one),
    TEST_ENTRY(swr_invalid_clamped_to_999),
    TEST_ENTRY(swr_typical_value),
    TEST_ENTRY(swr_monotonic_in_reflection),
    TEST_SUITE_END
};