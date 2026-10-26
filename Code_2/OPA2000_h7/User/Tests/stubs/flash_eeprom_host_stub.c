/**
 * @file    flash_eeprom_host_stub.c
 * @brief   Host-build stand-ins for the EEPROM layer's hardware paths.
 *
 * @note    flash_eeprom.c deliberately compiles its hardware half out of the
 *          host build (#ifndef HOST_TEST_NATIVE), so on the host only the
 *          record codec exists.  config.c however calls eeprom_load(),
 *          eeprom_save() and eeprom_current_id() unconditionally, so the
 *          test binary needs those three symbols or the link fails.
 *
 *          The semantics implemented here are what config.c actually relies
 *          on: "the store starts empty, every save succeeds".  That is the
 *          honest default for a host run with no flash to talk to, and it
 *          makes config_init() take its documented empty-store path:
 *          defaults, then a save, then report "no valid config" so the boot
 *          flow asks for touch calibration - which is the path the GUI tests
 *          and the calibration decision depend on.
 *
 *          If a test ever needs to exercise a torn or pre-existing record
 *          through config.c rather than through the codec directly, this is
 *          the hook to extend (a settable "load returns false" flag).
 */

#include "flash_eeprom.h"

#include <string.h>

bool eeprom_load(void *out, uint32_t out_size)
{
    /* No flash on the host: report an empty store and leave the caller's
     * buffer untouched, exactly as the real one does when nothing valid is
     * found. */
    (void)out;
    (void)out_size;
    return false;
}

uint32_t eeprom_current_id(void)
{
    /* 0 is the real layer's "nothing written yet" value, and config_sdcard_
     * ready() keys off it. */
    return 0U;
}

/*!< How many times eeprom_save() has been called since the last reset.
 *   The deferred-save tests use this to assert that a batch of edits
 *   produces ONE write, not one per edit. */
static uint32_t s_save_count = 0U;

/*!< When non-zero, eeprom_save() reports failure and clears this on the
 *   call, so a test can make exactly one write fail and then watch the
 *   retry succeed.  The deferred-save tests use it to prove that a failed
 *   write is not silently dropped - with a thread-flag wake-up there is no
 *   timer left to retry on, so a dropped flag IS a permanently lost
 *   setting. */
static uint32_t s_fail_next_saves = 0U;

uint32_t eeprom_host_save_count(void)
{
    return s_save_count;
}

void eeprom_host_reset_save_count(void)
{
    s_save_count = 0U;
}

void eeprom_host_fail_next_save(uint32_t count)
{
    s_fail_next_saves = count;
}

bool eeprom_save(const void *src, uint32_t len)
{
    /* Pretend it stuck.  Nothing reads it back, so the contents do not
     * matter - only that config_reset_default() does not report failure. */
    (void)src;
    (void)len;
    s_save_count++;
    if (s_fail_next_saves > 0U) {
        s_fail_next_saves--;
        return false;
    }
    return true;
}
