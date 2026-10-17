/**
 * @file    test_config_layout.c
 * @brief   Layout contract for the on-flash configuration record.
 *
 * @note    The EEPROM emulation layer stores ConfigType verbatim into a
 *          sequence of 16-byte flash words, so two properties are
 *          load-bearing and must not drift silently:
 *
 *            1. sizeof(ConfigType) must stay within the payload cap the
 *               EEPROM layer advertises (EEPROM_MAX_PAYLOAD_BYTES).
 *            2. crc32 must remain the LAST member, because config.c hashes
 *               "everything ahead of it" by subtracting sizeof(uint32_t)
 *               from sizeof(ConfigType).
 *
 *          Both are asserted here rather than left to a comment, so that
 *          adding a field breaks the build instead of the firmware.
 */

#include "test_harness.h"
#include "opa2000.h"
#include "flash_eeprom.h"

#include <stddef.h>
#include <string.h>

/* Per-sector bookkeeping the wear-leveling scheme needs on top of the
 * payload: magic, record id, payload length, CRC, commit marker. */
#define EEPROM_RECORD_HEADER_BYTES 16U

/*============================================================================
 *  Record fits
 *==========================================================================*/

TEST_CASE(config_record_fits_in_the_eeprom_payload_cap)
{
    /* The whole point of the exercise: one contiguous payload that the
     * EEPROM layer accepts, with room left for the header and commit
     * word inside a record.  The authoritative cap lives in
     * flash_eeprom.h - duplicating it here is how this test drifted out
     * of sync with the real geometry in the first place. */
    ASSERT_TRUE(sizeof(ConfigType) <= EEPROM_MAX_PAYLOAD_BYTES);
    ASSERT_TRUE(EEPROM_MAX_PAYLOAD_BYTES
                > sizeof(ConfigType) + EEPROM_RECORD_HEADER_BYTES);
}

TEST_CASE(config_record_is_word_aligned_after_the_header)
{
    /* Payload starts right after the 16-byte header, so it must itself be
     * 16-byte aligned or the write would straddle two flash words - which
     * the H7B0 ECC makes impossible without erasing first. */
    unsigned payload = EEPROM_RECORD_HEADER_BYTES;
    ASSERT_EQ(payload % EEPROM_WORD_BYTES, 0U);
}

TEST_CASE(config_record_leaves_room_in_the_sector)
{
    /* Each sector holds many records; we rotate through four sectors, so
     * per-sector capacity only has to exceed one record plus bookkeeping. */
    ASSERT_TRUE(EEPROM_SECTOR_BYTES >= EEPROM_MAX_RECORD_WORDS * 2U);
}

/*============================================================================
 *  crc32 must stay last
 *============================================================================*/

TEST_CASE(crc32_is_the_last_member_of_config)
{
    /* config.c does:  crc32_calculate(&cfg, sizeof(ConfigType) - 4)
     * If crc32 were not the trailing member, the hash would silently cover
     * the wrong bytes and a corrupted record could still validate. */
    ASSERT_EQ(offsetof(ConfigType, crc32) + sizeof(uint32_t),
              sizeof(ConfigType));
}

TEST_CASE(crc32_really_is_a_trailing_uint32)
{
    /* Guards against someone changing crc32 to uint16_t or uint64_t: the
     * "- sizeof(uint32_t)" in config.c would then be wrong. */
    ASSERT_EQ(sizeof(((ConfigType *)0)->crc32), sizeof(uint32_t));
}

/*============================================================================
 *  Types the record embeds
 *============================================================================*/

TEST_CASE(coupler_trim_is_exactly_two_int32)
{
    /* power_mW = raw * gain_uW_per_code / 1000 + offset_mW  -- the int64
     * intermediate in gui.c is what makes this safe, but the stored form
     * has to stay two 32-bit signed values. */
    ASSERT_EQ(sizeof(coupler_t), 8U);
}

TEST_CASE(touch_cal_stays_float_based)
{
    /* The user asked explicitly for no fixed-point: x/y gain and offset are
     * four floats.  Guard it so a future "optimisation" back to Q-format
     * fails here. */
    ASSERT_EQ(sizeof(touch_cal_t), 4U * sizeof(float) + 4U);
}

/*============================================================================
 *  Printout helper (run manually, not part of the pass/fail set)
 *============================================================================*/

void config_layout_report(void)
{
    printf("sizeof(ConfigType)  = %u\n",  (unsigned)sizeof(ConfigType));
    printf("  magic        @%u\n",  (unsigned)offsetof(ConfigType, magic));
    printf("  version      @%u\n",  (unsigned)offsetof(ConfigType, version));
    printf("  pc_baud_rate @%u\n",  (unsigned)offsetof(ConfigType, pc_baud_rate));
    printf("  radio_type   @%u\n",  (unsigned)offsetof(ConfigType, radio_type));
    printf("  band_rf_in   @%u\n",  (unsigned)offsetof(ConfigType, band_rf_in_max_w));
    printf("  band_vbias   @%u\n",  (unsigned)offsetof(ConfigType, band_vbias_max_mV));
    printf("  im_zero_off  @%u\n",  (unsigned)offsetof(ConfigType, im_zero_offset_adc));
    printf("  touch        @%u\n",  (unsigned)offsetof(ConfigType, touch));
    printf("  fwd1         @%u\n",  (unsigned)offsetof(ConfigType, fwd1));
    printf("  language     @%u\n",  (unsigned)offsetof(ConfigType, language));
    printf("  temp_f       @%u\n",  (unsigned)offsetof(ConfigType, temp_fahrenheit));
    printf("  comp1_ref    @%u\n",  (unsigned)offsetof(ConfigType, comp1_ref_mV));
    printf("  comp2_ref    @%u\n",  (unsigned)offsetof(ConfigType, comp2_ref_mV));
    printf("  crc32        @%u\n",  (unsigned)offsetof(ConfigType, crc32));
    printf("header + payload rounded to word = %u\n",
           (unsigned)(((sizeof(ConfigType) + EEPROM_RECORD_HEADER_BYTES
                        + EEPROM_WORD_BYTES - 1U)
                       / EEPROM_WORD_BYTES) * EEPROM_WORD_BYTES));
}