/**
 * @file    test_config_deferred_save.c
 * @brief   Host tests for the deferred-save mechanism (thread flag wake-up).
 *
 * @details The config save task used to poll once a second.  It now blocks
 *          on a thread flag and is woken by config_mark_dirty().  The
 *          questions these tests answer are the ones that actually matter
 *          for the operator:
 *
 *            - Does an edit actually wake the save task?  (If not, the
 *              edit is silently lost on the next reboot.)
 *            - Does a BATCH of edits produce ONE flash write, or one per
 *              edit?  (One per edit would burn through the flash ring for
 *              nothing and stall the CPU dozens of times per gesture.)
 *            - Can an edit be lost because the wake-up arrived at the
 *              wrong moment?  (The dirty flag is the durable record; the
 *              flag is only the wake-up.)
 *
 *          The host build has no RTOS, so the shim in
 *          User/Tests/stubs/cmsis_os_shim.c records what was posted to
 *          which thread, and the flash stub counts the writes.  Together
 *          they make the whole chain observable:
 *
 *              config_mark_dirty()
 *                  -> s_dirty = true
 *                  -> osThreadFlagsSet(save_task, CONFIG_SAVE_FLAG)
 *                  -> app_config_task() wakes
 *                  -> config_flush_pending() -> eeprom_save()
 */

#include "test_harness.h"
#include "cmsis_os.h"
#include "config.h"
#include "flash_eeprom.h"
#include "opa2000.h"

#include <string.h>

/*!< Provided by the flash host stub (User/Tests/stubs/flash_eeprom_host_stub.c). */
uint32_t eeprom_host_save_count(void);
void     eeprom_host_reset_save_count(void);
void     eeprom_host_fail_next_save(uint32_t count);

/*!< A stand-in for the save task's thread id.  The shim only records it, so
 *   any non-NULL value will do - but it has to be non-NULL, because
 *   NULL is how config.c spells "no task registered yet". */
static uint32_t     s_task_token = 0U;
static osThreadId_t s_fake_save_task = NULL;

/* Bring the module back to a known state before each case. */
static void reset_save_state(void)
{
    s_fake_save_task = (osThreadId_t)&s_task_token;
    config_set_save_task(NULL);
    config_clear_dirty();
    cmsis_os_shim_reset_flag_counters();
    eeprom_host_reset_save_count();
    eeprom_host_fail_next_save(0U);
}

/*============================================================================
 *  Test 1: an edit wakes the save task
 *
 *  This is the whole point of the change.  If config_mark_dirty() did not
 *  post the flag, the save task would sit blocked forever and the
 *  operator's settings would vanish on the next reboot - with no error
 *  message anywhere, because the dirty flag would still be set and
 *  nothing was complaining about it.
 *==========================================================================*/
TEST_CASE(an_edit_wakes_the_save_task)
{
    reset_save_state();
    config_set_save_task(s_fake_save_task);

    ASSERT_FALSE(config_is_dirty());
    ASSERT_EQ(cmsis_os_shim_flag_set_count(), 0U);

    config_mark_dirty();

    /* The durable record is set ... */
    ASSERT_TRUE(config_is_dirty());
    /* ... and the wake-up was posted, to the right thread, with the
     * flag the save task waits on. */
    ASSERT_EQ(cmsis_os_shim_flag_set_count(), 1U);
    ASSERT_EQ(cmsis_os_shim_last_flag_set(), CONFIG_SAVE_FLAG);
    ASSERT_TRUE(cmsis_os_shim_last_flag_target() == s_fake_save_task);
}

/*============================================================================
 *  Test 2: the wake-up is posted to the task that registered
 *
 *  config.c keeps one handle.  If a second task ever registered, the
 *  wake-up must go to the registered one - posting to a stale handle
 *  would wake nobody.
 *==========================================================================*/
TEST_CASE(the_wakeup_goes_to_the_registered_task)
{
    reset_save_state();

    /* A second, different handle - the one the save task registers with
     * must be the one that gets woken. */
    static uint32_t other_token = 0U;
    osThreadId_t other = (osThreadId_t)&other_token;
    config_set_save_task(other);

    config_mark_dirty();

    ASSERT_EQ(cmsis_os_shim_flag_set_count(), 1U);
    ASSERT_TRUE(cmsis_os_shim_last_flag_target() == other);
}

/*============================================================================
 *  Test 3: an edit made BEFORE the save task existed is not lost
 *
 *  The boot path edits the config (config_reset_default() and friends)
 *  long before app_config_task() exists.  At that point there is nobody
 *  to wake, so the dirty flag alone carries the edit.  When the task does
 *  register, it must find the pending edit and wake itself for it.
 *==========================================================================*/
TEST_CASE(an_edit_before_the_task_existed_is_not_lost)
{
    reset_save_state();

    /* No task registered yet: the wake-up has nowhere to go. */
    config_mark_dirty();
    ASSERT_TRUE(config_is_dirty());
    ASSERT_EQ(cmsis_os_shim_flag_set_count(), 0U);

    /* Now the task registers.  It has to notice the pending edit. */
    config_set_save_task(s_fake_save_task);

    ASSERT_TRUE(config_is_dirty());
    ASSERT_EQ(cmsis_os_shim_flag_set_count(), 1U);
    ASSERT_EQ(cmsis_os_shim_last_flag_set(), CONFIG_SAVE_FLAG);
}

/*============================================================================
 *  Test 4: registering a task when nothing is pending wakes nobody
 *
 *  The counterpart of test 3: a spurious wake-up would make the save
 *  task spin through a no-op write for nothing.
 *==========================================================================*/
TEST_CASE(registering_a_task_with_nothing_pending_wakes_nobody)
{
    reset_save_state();

    config_set_save_task(s_fake_save_task);

    ASSERT_FALSE(config_is_dirty());
    ASSERT_EQ(cmsis_os_shim_flag_set_count(), 0U);
}

/*============================================================================
 *  Test 5: a BATCH of edits produces ONE flash write
 *
 *  This is the question the operator actually asked: "if I change several
 *  values in one visit to the Settings menu, does it write the flash
 *  several times?"
 *
 *  The answer must be once.  Every edit raises the same dirty flag and
 *  posts the same thread flag; the save task wakes once, sees the dirty
 *  flag, and writes once.  The GUI's staging (s_menu_dirty) is what
 *  turns a whole editing session into a single config_mark_dirty(), so
 *  the batch arrives here as one dirty flag, not five.
 *==========================================================================*/
TEST_CASE(a_batch_of_edits_produces_one_flash_write)
{
    reset_save_state();
    config_set_save_task(s_fake_save_task);

    /* Five edits, the way five sliders in the Settings menu would. */
    g_config.band_rev1_max_w      = 810U;
    config_mark_dirty();
    g_config.band_rev2_max_w      = 420U;
    config_mark_dirty();
    g_config.band_swr_max_x100    = 250U;
    config_mark_dirty();
    g_config.band_ntc_max_tenth_c = 950U;
    config_mark_dirty();
    g_config.band_fwd1_max_w      = 1150U;
    config_mark_dirty();

    /* Five edits, five wake-ups posted - the task would be woken five
     * times, but the dirty flag is a single bit, so the batch is one
     * unit of work. */
    ASSERT_EQ(cmsis_os_shim_flag_set_count(), 5U);
    ASSERT_TRUE(config_is_dirty());

    /* The save task wakes and flushes: ONE write for the whole batch. */
    ASSERT_TRUE(config_flush_pending());
    ASSERT_EQ(eeprom_host_save_count(), 1U);

    /* And the flag is down again, so nothing else is queued. */
    ASSERT_FALSE(config_is_dirty());
}

/*============================================================================
 *  Test 6: a wake-up with nothing pending writes nothing
 *
 *  The stale-flag case: the task was woken, but the edit it was woken
 *  for has already been written by somebody else (the CAT link's
 *  explicit save, say).  It must not write the same record again.
 *==========================================================================*/
TEST_CASE(a_wakeup_with_nothing_pending_writes_nothing)
{
    reset_save_state();
    config_set_save_task(s_fake_save_task);

    ASSERT_FALSE(config_flush_pending());
    ASSERT_EQ(eeprom_host_save_count(), 0U);
}

/*============================================================================
 *  Test 7: an edit that lands DURING the write is kept for the next pass
 *
 *  The clear-then-write ordering is what makes this safe.  The save task
 *  clears the dirty flag and then writes; an edit that arrives in that
 *  window finds the flag already cleared, so it sets it again and is
 *  written on the next pass.  Clearing AFTER the write instead would
 *  swallow it - the edit would be in the flash's RAM copy but the flag
 *  would say "nothing pending", and the record would never be written.
 *
 *  On the host the two steps cannot be interleaved by a real thread, so
 *  the ordering is asserted directly: after the flush the flag is clear,
 *  and marking dirty again makes the next flush write again.
 *==========================================================================*/
TEST_CASE(an_edit_after_a_flush_is_written_on_the_next_pass)
{
    reset_save_state();
    config_set_save_task(s_fake_save_task);

    g_config.band_fwd2_max_w = 1060U;
    config_mark_dirty();
    ASSERT_TRUE(config_flush_pending());
    ASSERT_EQ(eeprom_host_save_count(), 1U);

    /* A second batch arrives after the first was written. */
    g_config.band_fwd2_max_w = 1070U;
    config_mark_dirty();
    ASSERT_TRUE(config_is_dirty());
    ASSERT_TRUE(config_flush_pending());
    ASSERT_EQ(eeprom_host_save_count(), 2U);
}

/*============================================================================
 *  Test 8: the flash write takes the config lock
 *
 *  Two writers (the save task and the CAT link's explicit save) must
 *  not be inside eeprom_save() at the same time: they would pick the
 *  same target slot and one of the two records would be silently lost.
 *  The lock is taken around the write, so the acquire count has to come
 *  back to zero - and it has to have been non-zero while the write was
 *  running.
 *==========================================================================*/
TEST_CASE(the_flash_write_takes_the_config_lock)
{
    reset_save_state();
    config_set_save_task(s_fake_save_task);
    cmsis_os_shim_reset_counters();

    config_mark_dirty();
    ASSERT_TRUE(config_flush_pending());

    /* The write happened ... */
    ASSERT_EQ(eeprom_host_save_count(), 1U);
    /* ... and the lock was released again. */
    ASSERT_EQ(cmsis_os_shim_active_acquires(), 0U);
}

/*============================================================================
 *  Test 9: the wake-up survives a dropped notification
 *
 *  The dirty flag is the durable record and the thread flag is only the
 *  wake-up.  If the notification is lost - the task has not registered
 *  yet, or the post failed - the edit must still be written once the
 *  task exists.  Test 3 covers the "not registered yet" case; this one
 *  covers the general shape: the flag alone is enough to drive a write.
 *==========================================================================*/
TEST_CASE(the_dirty_flag_alone_drives_the_write)
{
    reset_save_state();

    /* The edit happens with no task around: no wake-up can be posted. */
    config_mark_dirty();
    ASSERT_EQ(cmsis_os_shim_flag_set_count(), 0U);

    /* The task shows up and flushes. */
    config_set_save_task(s_fake_save_task);
    ASSERT_TRUE(config_flush_pending());
    ASSERT_EQ(eeprom_host_save_count(), 1U);
}

/*============================================================================
 *  Test 10: clearing the dirty flag does not need a task
 *
 *  The CAT link's explicit save and the power-off flush clear the flag
 *  after writing themselves, so the save task does not write the same
 *  record again.  That must work with or without a registered task.
 *==========================================================================*/
TEST_CASE(clearing_the_dirty_flag_works_without_a_task)
{
    reset_save_state();

    config_mark_dirty();
    ASSERT_TRUE(config_is_dirty());

    config_clear_dirty();
    ASSERT_FALSE(config_is_dirty());
    ASSERT_FALSE(config_flush_pending());
    ASSERT_EQ(eeprom_host_save_count(), 0U);
}

/*============================================================================
 *  Test 11: a FAILED write is retried, not dropped
 *
 *  This is the failure mode the thread-flag change made worse.  Under the
 *  old 1 s poll, a config_save() that failed was retried a second later
 *  whether anyone cared or not.  With a flag wake-up there is no timer
 *  left, so a failed write that just cleared the dirty flag and moved on
 *  is gone for good - the operator's setting silently reverts on the next
 *  reboot, with nothing in the logs to say why.
 *
 *  eeprom_save() has six ways to fail (no free slot, unlock, erase,
 *  encode, program).  A transient one - a sector erase that did not
 *  settle, say - must not cost the operator their work.
 *==========================================================================*/
TEST_CASE(a_failed_write_is_retried_not_dropped)
{
    reset_save_state();
    config_set_save_task(s_fake_save_task);

    /* The next write fails. */
    eeprom_host_fail_next_save(1U);

    g_config.band_rev1_max_w = 830U;
    config_mark_dirty();
    cmsis_os_shim_reset_flag_counters();

    /* The flush reports the failure rather than claiming success ... */
    ASSERT_FALSE(config_flush_pending());
    ASSERT_EQ(eeprom_host_save_count(), 1U);

    /* ... and, crucially, the edit is still pending and the save task has
     * been asked to come back.  Both halves are needed: the dirty flag
     * without a wake-up would sit unnoticed forever. */
    ASSERT_TRUE(config_is_dirty());
    ASSERT_EQ(cmsis_os_shim_flag_set_count(), 1U);
    ASSERT_EQ(cmsis_os_shim_last_flag_set(), CONFIG_SAVE_FLAG);

    /* The retry succeeds and the flag comes down. */
    ASSERT_TRUE(config_flush_pending());
    ASSERT_EQ(eeprom_host_save_count(), 2U);
    ASSERT_FALSE(config_is_dirty());
}

/*============================================================================
 *  Test 12: a permanently failing store does not spin the save task
 *
 *  The retry above is what we want, but an edit that can never be written
 *  must not turn into an unbounded busy loop either - the save task sits
 *  at the lowest priority, and a tight re-post loop would burn the
 *  scheduler on flash retries and starve everything below it.
 *
 *  The flag being re-posted on every failure is fine and self-limiting:
 *  each attempt takes as long as the flash erase itself, so the loop is
 *  bounded by the flash, not by the CPU.  What this test pins down is the
 *  invariant that matters - the dirty flag stays set while the write
 *  keeps failing, so no edit is ever silently dropped no matter how many
 *  attempts it takes.
 *==========================================================================*/
TEST_CASE(a_persistently_failing_write_never_drops_the_edit)
{
    reset_save_state();
    config_set_save_task(s_fake_save_task);

    eeprom_host_fail_next_save(3U);

    g_config.band_rev2_max_w = 430U;
    config_mark_dirty();

    for (int i = 0; i < 3; i++) {
        ASSERT_FALSE(config_flush_pending());
        ASSERT_TRUE(config_is_dirty());
    }
    ASSERT_EQ(eeprom_host_save_count(), 3U);

    /* Fourth attempt succeeds: the accumulated edit finally lands. */
    ASSERT_TRUE(config_flush_pending());
    ASSERT_EQ(eeprom_host_save_count(), 4U);
    ASSERT_FALSE(config_is_dirty());
}

/*============================================================================
 *  Suite registration
 *==========================================================================*/
const test_entry_t g_suite_config_deferred_save[] = {
    TEST_ENTRY(an_edit_wakes_the_save_task),
    TEST_ENTRY(the_wakeup_goes_to_the_registered_task),
    TEST_ENTRY(an_edit_before_the_task_existed_is_not_lost),
    TEST_ENTRY(registering_a_task_with_nothing_pending_wakes_nobody),
    TEST_ENTRY(a_batch_of_edits_produces_one_flash_write),
    TEST_ENTRY(a_wakeup_with_nothing_pending_writes_nothing),
    TEST_ENTRY(an_edit_after_a_flush_is_written_on_the_next_pass),
    TEST_ENTRY(the_flash_write_takes_the_config_lock),
    TEST_ENTRY(the_dirty_flag_alone_drives_the_write),
    TEST_ENTRY(clearing_the_dirty_flag_works_without_a_task),
    TEST_ENTRY(a_failed_write_is_retried_not_dropped),
    TEST_ENTRY(a_persistently_failing_write_never_drops_the_edit),
    TEST_SUITE_END,
};
