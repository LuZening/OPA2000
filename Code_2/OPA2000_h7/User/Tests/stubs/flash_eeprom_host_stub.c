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

bool eeprom_save(const void *src, uint32_t len)
{
    /* Pretend it stuck.  Nothing reads it back, so the contents do not
     * matter - only that config_reset_default() does not report failure. */
    (void)src;
    (void)len;
    return true;
}
