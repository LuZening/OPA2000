/**
 * @file    test_flash_eeprom.c
 * @brief   Unit tests for the flash EEPROM record codec.
 *
 * @note    Only the codec is tested here - eeprom_encode() and
 *          eeprom_decode() are pure functions over a word array.  The
 *          hardware paths (erase, program, sector rotation) need a real
 *          H7B0 and are out of scope for the host build, but every
 *          correctness property they depend on lives in the codec and is
 *          covered here.
 *
 *          The properties that matter are all about surviving a reset in
 *          the middle of a save, because that is the failure mode this
 *          layer exists to prevent.
 */

#include "test_harness.h"
#include "flash_eeprom.h"
#include "opa2000.h"

#include <string.h>

#define ERASED_WORD  0xFFFFFFFFU

/*============================================================================
 *  Geometry
 *============================================================================*/

TEST_CASE(sector_is_8kb_and_word_is_16_bytes)
{
    /* Both are hardware facts about the H7B0 and the whole layout depends
     * on them: 137-bit words mean nothing below 16 bytes is writable, and
     * an erase always costs a whole 8 KB sector. */
    ASSERT_EQ(EEPROM_SECTOR_BYTES, 8192U);
    ASSERT_EQ(EEPROM_WORD_BYTES, 16U);
    ASSERT_EQ(EEPROM_WORDS_PER_SECTOR, 512U);
}

TEST_CASE(config_record_fits_the_payload_cap)
{
    /* ConfigType plus header and commit must stay inside the cap, otherwise
     * eeprom_save() would refuse to store it. */
    uint32_t words = eeprom_record_words(sizeof(ConfigType));
    ASSERT_TRUE(words <= EEPROM_MAX_RECORD_WORDS);
}

TEST_CASE(record_words_counts_header_payload_and_commit)
{
    /* 0-byte payload: header (1) + nothing + commit (1) = 2 words */
    ASSERT_EQ(eeprom_record_words(0U), 2U);
    /* 16 bytes fits exactly one payload word */
    ASSERT_EQ(eeprom_record_words(16U), 3U);
    /* 17 bytes spills into a second payload word */
    ASSERT_EQ(eeprom_record_words(17U), 4U);
    /* sizeof(ConfigType) = 124 -> 8 payload words -> 10 words total */
    ASSERT_EQ(eeprom_record_words(124U), 10U);
}

/*============================================================================
 *  Round trip
 *============================================================================*/

TEST_CASE(a_record_round_trips)
{
    static uint32_t words[EEPROM_MAX_RECORD_WORDS];
    ConfigType in;
    ConfigType out;

    memset(&in, 0, sizeof(in));
    in.pc_baud_rate = 115200U;
    in.language     = MULTILANG_ENGLISH;
    in.temp_fahrenheit = 1U;
    in.touch.x_gain = 1.2345f;

    uint32_t crc = crc32_calculate((const uint8_t *)&in, sizeof(in));
    uint32_t n = eeprom_encode(7U, &in, sizeof(in), crc, words,
                               EEPROM_MAX_RECORD_WORDS);
    ASSERT_EQ(n, eeprom_record_words(sizeof(ConfigType)));

    memset(&out, 0, sizeof(out));
    ASSERT_EQ(eeprom_decode(words, n, &out, sizeof(out)), sizeof(ConfigType));
    ASSERT_EQ(out.pc_baud_rate, 115200U);
    ASSERT_EQ(out.language, MULTILANG_ENGLISH);
    ASSERT_EQ(out.temp_fahrenheit, 1U);
    ASSERT_TRUE(out.touch.x_gain > 1.23f && out.touch.x_gain < 1.24f);
}

TEST_CASE(id_survives_the_round_trip)
{
    /* The id is what makes "newest wins" work on load, so it has to come
     * back out of the record rather than being inferred. */
    static uint32_t words[EEPROM_MAX_RECORD_WORDS];
    uint8_t payload[32];
    memset(payload, 0xA5, sizeof(payload));

    eeprom_encode(0x1234U, payload, sizeof(payload), 0U, words,
                  EEPROM_MAX_RECORD_WORDS);

    /* id is the upper 16 bits of word 1 */
    ASSERT_EQ(words[1] >> 16, 0x1234U);
    ASSERT_EQ(words[1] & 0xFFFFU, sizeof(payload));
}

TEST_CASE(encoding_is_deterministic)
{
    /* Saving the same config twice must produce identical bytes.  If the
     * zero padding were uninitialised garbage the sector would fill with
     * records that differ only in slack, and the CRC of "the same" config
     * would not be reproducible. */
    static uint32_t a[EEPROM_MAX_RECORD_WORDS];
    static uint32_t b[EEPROM_MAX_RECORD_WORDS];
    uint8_t payload[64];
    memset(payload, 0x5A, sizeof(payload));

    uint32_t na = eeprom_encode(3U, payload, sizeof(payload), 0xDEADU, a,
                                EEPROM_MAX_RECORD_WORDS);
    uint32_t nb = eeprom_encode(3U, payload, sizeof(payload), 0xDEADU, b,
                                EEPROM_MAX_RECORD_WORDS);

    ASSERT_EQ(na, nb);
    ASSERT_TRUE(memcmp(a, b, na * sizeof(uint32_t)) == 0);
}

TEST_CASE(padding_past_the_payload_is_ones_not_garbage)
{
    /* A 20-byte payload occupies 32 bytes of flash words.  The slack must
     * be a fixed value so a later read of that region is predictable. */
    static uint32_t words[EEPROM_MAX_RECORD_WORDS];
    uint8_t payload[20];
    memset(payload, 0x11, sizeof(payload));

    uint32_t n = eeprom_encode(1U, payload, sizeof(payload), 0U, words,
                               EEPROM_MAX_RECORD_WORDS);
    ASSERT_EQ(n, 4U);

    /* payload occupies words[4] .. words[8]; word[8] holds bytes 16..19 of
     * the payload plus 12 bytes of padding. */
    const uint8_t *raw = (const uint8_t *)&words[4];
    for (int i = 20; i < 32; i++) {
        ASSERT_EQ(raw[i], 0xFF);
    }
}

/*============================================================================
 *  Torn writes - the reason this layer exists
 *============================================================================*/

TEST_CASE(an_erased_slot_is_not_a_record)
{
    /* This is what a never-written region looks like.  It must be rejected,
     * not decoded as a record with garbage contents. */
    static uint32_t words[EEPROM_MAX_RECORD_WORDS];
    ConfigType out;

    for (unsigned i = 0; i < EEPROM_MAX_RECORD_WORDS; i++) {
        words[i] = ERASED_WORD;
    }
    ASSERT_EQ(eeprom_decode(words, 10U, &out, sizeof(out)), 0U);
}

TEST_CASE(a_record_without_its_commit_word_is_rejected)
{
    /* Power cut between the payload write and the commit write.  The
     * payload is fully there but the record must NOT be trusted, because
     * we cannot tell it apart from a half-written one. */
    static uint32_t words[EEPROM_MAX_RECORD_WORDS];
    ConfigType in;
    ConfigType out;

    memset(&in, 0, sizeof(in));
    in.pc_baud_rate = 9600U;
    uint32_t n = eeprom_encode(5U, &in, sizeof(in),
                               crc32_calculate((const uint8_t *)&in, sizeof(in)),
                               words, EEPROM_MAX_RECORD_WORDS);

    /* Simulate the interrupted save: everything written except the commit. */
    uint32_t commit = words[n - 1U];
    words[n - 1U] = ERASED_WORD;

    ASSERT_EQ(eeprom_decode(words, n, &out, sizeof(out)), 0U);

    /* And once the commit lands it becomes valid again. */
    words[n - 1U] = commit;
    ASSERT_EQ(eeprom_decode(words, n, &out, sizeof(out)), sizeof(ConfigType));
}

TEST_CASE(a_missing_header_is_rejected)
{
    static uint32_t words[EEPROM_MAX_RECORD_WORDS];
    uint8_t payload[32];
    uint8_t out[64];
    memset(payload, 0x3C, sizeof(payload));

    uint32_t n = eeprom_encode(2U, payload, sizeof(payload), 0U, words,
                               EEPROM_MAX_RECORD_WORDS);
    words[0] = ERASED_WORD;          /* header never landed */

    ASSERT_EQ(eeprom_decode(words, n, out, sizeof(out)), 0U);
}

TEST_CASE(commit_id_must_agree_with_the_header)
{
    /* A mismatch means the two halves came from different saves - the
     * signature of a record whose header was reused after a partial erase. */
    static uint32_t words[EEPROM_MAX_RECORD_WORDS];
    uint8_t payload[32];
    uint8_t out[64];
    memset(payload, 0x77, sizeof(payload));

    uint32_t n = eeprom_encode(9U, payload, sizeof(payload), 0U, words,
                               EEPROM_MAX_RECORD_WORDS);
    words[n - 1U] = (words[n - 1U] & 0x0000FFFFU) | (0x00AA0000U);

    ASSERT_EQ(eeprom_decode(words, n, out, sizeof(out)), 0U);
}

TEST_CASE(commit_length_must_agree_with_the_header)
{
    static uint32_t words[EEPROM_MAX_RECORD_WORDS];
    uint8_t payload[32];
    uint8_t out[64];
    memset(payload, 0x22, sizeof(payload));

    uint32_t n = eeprom_encode(4U, payload, sizeof(payload), 0U, words,
                               EEPROM_MAX_RECORD_WORDS);
    /* Header says 32 bytes, commit says 16 - the decoder must not guess. */
    words[1] = (words[1] & 0xFFFF0000U) | 16U;

    ASSERT_EQ(eeprom_decode(words, n, out, sizeof(out)), 0U);
}

TEST_CASE(a_length_that_disagrees_with_the_slot_is_rejected)
{
    /* Header claims a length whose record would need more words than the
     * slot holds.  Decoding must refuse rather than read past the record
     * into whatever follows it. */
    static uint32_t words[EEPROM_MAX_RECORD_WORDS];
    uint8_t payload[32];
    uint8_t out[64];
    memset(payload, 0x99, sizeof(payload));

    uint32_t n = eeprom_encode(6U, payload, sizeof(payload), 0U, words,
                               EEPROM_MAX_RECORD_WORDS);
    words[1] = (words[1] & 0xFFFF0000U) | 4000U;   /* absurd length */

    ASSERT_EQ(eeprom_decode(words, n, out, sizeof(out)), 0U);
}

TEST_CASE(a_record_longer_than_the_destination_is_refused)
{
    /* Writing into a smaller buffer must fail loudly instead of smashing
     * the caller's stack. */
    static uint32_t words[EEPROM_MAX_RECORD_WORDS];
    uint8_t payload[128];
    uint8_t small[16];
    memset(payload, 0x42, sizeof(payload));

    uint32_t n = eeprom_encode(8U, payload, sizeof(payload), 0U, words,
                               EEPROM_MAX_RECORD_WORDS);
    ASSERT_EQ(eeprom_decode(words, n, small, sizeof(small)), 0U);
}

/*============================================================================
 *  Capacity
 *============================================================================*/

TEST_CASE(four_sectors_hold_far_more_records_than_we_need)
{
    /* With a 124-byte ConfigType a record is 10 words = 160 bytes, so a
     * sector holds 32 and four sectors hold 128.  A sector is erased only
     * once all four are full, which multiplies the flash endurance by four.
     *
     * The point of this test is to catch the case where someone grows
     * ConfigType so much that the ring stops being practical. */
    uint32_t words = eeprom_record_words(sizeof(ConfigType));
    uint32_t per_sector = EEPROM_WORDS_PER_SECTOR / words;
    uint32_t total = per_sector * EEPROM_SECTOR_COUNT;

    ASSERT_TRUE(per_sector >= 16U);
    ASSERT_TRUE(total >= 64U);
}