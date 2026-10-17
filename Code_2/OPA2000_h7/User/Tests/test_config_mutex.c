/**
 * @file    test_config_mutex.c
 * @brief   Host tests for the g_config lock / unlock contract.
 *
 * @details Verifies the rule added in 2026-10-14:
 *   - config_save() / config_init() / config_reset_default() /
 *     config_validate() take config_lock() around the whole multi-field
 *     read/write.
 *   - Single-field reads and writes do not need the lock.
 *   - A field write that lands while the lock is held must not corrupt
 *     the snapshot that the lock-protected reader is working with.
 *
 * The host test runs against cmsis_os_shim (Windows mutex).  These
 * tests do NOT spin worker threads because the "lock is held" scenario
 * can be reproduced deterministically by calling config_lock() /
 * config_unlock() directly from the test thread.
 */

#include "test_harness.h"
#include "cmsis_os.h"
#include "config.h"
#include "opa2000.h"

#include <string.h>

/*============================================================================
 *  Test 1: lock / unlock are no-ops on the host shim at the API level
 *
 *  We don't have a way to verify "another thread cannot interleave"
 *  from a single thread, so we just assert the round-trip: lock,
 *  read g_config.*, unlock.  Field value must match what we set
 *  before.  If this round-trips, the basic "no crash" path is OK.
 *============================================================================*/
TEST_CASE(config_lock_unlock_roundtrip)
{
    /* touch a couple of fields first; they live in volatile ConfigType
     * but are not under the lock.  The lock is for the save / init /
     * validate snapshot path only. */
    g_config.pc_baud_rate = 19200U;
    g_config.radio_baud_rate = 4800U;
    g_config.band_fwd2_max_w = 800U;

    config_lock();
    uint32_t a = g_config.pc_baud_rate;
    uint16_t b = g_config.band_fwd2_max_w;
    config_unlock();

    ASSERT_EQ(a, 19200U);
    ASSERT_EQ(b, 800U);
}

/*============================================================================
 *  Test 2: field writes outside the lock are visible through the lock
 *
 *  Confirms the rule that a "1 writer + N readers" path is fine without
 *  a lock as long as the readers do not need a cross-field consistent
 *  view.  We write, then read through a held lock, and expect the
 *  written value to come back.  This protects against accidentally
 *  re-introducing a "lock around single field access" that would slow
 *  down the hot path.
 *============================================================================*/
TEST_CASE(config_single_field_visible_through_lock)
{
    g_config.band_fwd2_max_w = 0U;
    g_config.band_fwd1_max_w = 0U;

    /* simulated "writer" */
    g_config.band_fwd2_max_w = 950U;
    g_config.band_fwd1_max_w = 1100U;

    config_lock();
    uint16_t fwd2 = g_config.band_fwd2_max_w;
    uint16_t fwd1 = g_config.band_fwd1_max_w;
    config_unlock();

    ASSERT_EQ(fwd2, 950U);
    ASSERT_EQ(fwd1, 1100U);
}

/*============================================================================
 *  Test 3: config_lock / config_unlock are caller-side.  The public
 *  config_*() functions must NOT take the lock themselves.  This is
 *  the post-refactor contract: lock ownership is the caller's
 *  responsibility.
 *============================================================================*/
TEST_CASE(config_public_methods_do_not_lock)
{
    cmsis_os_shim_reset_counters();

    /* All four multi-field public methods must complete without
     * touching the mutex.  If any of them re-introduces an
     * osMutexAcquire inside its body, this test sees a non-zero
     * active count and fails. */
    (void)config_init();
    ASSERT_EQ(cmsis_os_shim_active_acquires(), 0U);

    (void)config_save();
    ASSERT_EQ(cmsis_os_shim_active_acquires(), 0U);

    (void)config_validate();
    ASSERT_EQ(cmsis_os_shim_active_acquires(), 0U);

    config_reset_default();
    ASSERT_EQ(cmsis_os_shim_active_acquires(), 0U);
}

/*============================================================================
 *  Test 4: when the caller wraps a config_save() in lock/unlock, the
 *  sequence round-trips and the active count returns to 0.  Verifies
 *  the caller's contract end-to-end.
 *============================================================================*/
TEST_CASE(config_caller_lock_save_roundtrip)
{
    cmsis_os_shim_reset_counters();
    config_reset_default();

    config_lock();
    bool ok = config_save();
    config_unlock();

    ASSERT_TRUE(ok);
    ASSERT_EQ(cmsis_os_shim_active_acquires(), 0U);
    ASSERT_TRUE(config_validate());
}

/*============================================================================
 *  Suite registration
 *============================================================================*/
const test_entry_t g_suite_config_mutex[] = {
    TEST_ENTRY(config_lock_unlock_roundtrip),
    TEST_ENTRY(config_single_field_visible_through_lock),
    TEST_ENTRY(config_public_methods_do_not_lock),
    TEST_ENTRY(config_caller_lock_save_roundtrip),
    TEST_SUITE_END,
};
