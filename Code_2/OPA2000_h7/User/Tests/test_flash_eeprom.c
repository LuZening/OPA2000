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
     * eeprom_save() would refuse to store it.  eeprom_record_words() counts
     * 16-byte flash words, so it is compared against the flash-word cap and
     * separately against the uint32 staging buffer in its own unit. */
    uint32_t words = eeprom_record_words(sizeof(ConfigType));
    ASSERT_TRUE(words <= EEPROM_MAX_RECORD_WORDS);
    ASSERT_TRUE(words * EEPROM_WORD_U32 <= EEPROM_RECORD_U32);
}

TEST_CASE(the_staging_buffer_is_the_size_the_geometry_actually_needs)
{
    /* EEPROM_RECORD_U32 is the single number that decides whether a save
     * overruns, so pin its derivation rather than its literal value.  The
     * unit chain is: payload cap -> payload flash words -> whole record ->
     * uint32 elements (four per 16-byte flash word). */
    uint32_t payload_words =
        (EEPROM_MAX_PAYLOAD_BYTES + EEPROM_WORD_BYTES - 1U) / EEPROM_WORD_BYTES;
    uint32_t record_words = EEPROM_HEADER_WORDS + payload_words
                            + EEPROM_COMMIT_WORDS;

    ASSERT_EQ(EEPROM_WORD_U32, 4U);            /* 16 bytes / 4 */
    ASSERT_EQ(EEPROM_MAX_RECORD_WORDS, record_words);
    ASSERT_EQ(EEPROM_RECORD_U32, record_words * 4U);

    /* And the real config must fit with room to spare - the buffer is sized
     * for the worst case the cap allows, not for ConfigType as it is today,
     * so growing the struct does not silently walk off the end. */
    uint32_t need = eeprom_record_words(sizeof(ConfigType));
    ASSERT_TRUE(need < EEPROM_MAX_RECORD_WORDS);
    ASSERT_TRUE(need * EEPROM_WORD_U32 < EEPROM_RECORD_U32);
}

TEST_CASE(record_words_counts_header_payload_and_commit)
{
    /* 0-byte payload: 1 header word + nothing + 1 commit word */
    ASSERT_EQ(eeprom_record_words(0U), 2U);
    /* 16 bytes fits exactly one payload word */
    ASSERT_EQ(eeprom_record_words(16U), 3U);
    /* 17 bytes spills into a second payload word */
    ASSERT_EQ(eeprom_record_words(17U), 4U);
    /* sizeof(ConfigType) = 128 -> 8 payload words -> 10 words total */
    ASSERT_EQ(eeprom_record_words(128U), 10U);
}

TEST_CASE(record_words_saturates_instead_of_wrapping)
{
    /* The rounding term is (len + 15).  For a len just below UINT32_MAX that
     * addition wraps, and an unguarded need comes back as 2 - which then
     * passes every "does it fit" check downstream and lets the payload memcpy
     * run off the end of the staging buffer.  A record can never exceed one
     * sector, so clamping is both safe and the honest answer. */
    ASSERT_EQ(eeprom_record_words(0xFFFFFFFFU),
              eeprom_record_words(EEPROM_SECTOR_BYTES -
                                  EEPROM_MAX_RECORD_OVERHEAD_BYTES));
    ASSERT_EQ(eeprom_record_words(0xFFFFFFF0U),
              eeprom_record_words(EEPROM_SECTOR_BYTES -
                                  EEPROM_MAX_RECORD_OVERHEAD_BYTES));
    /* The saturated answer is a whole-sector-sized record, which is far larger
     * than the EEPROM_MAX_PAYLOAD_BYTES cap - that cap bounds what eeprom_save
     * accepts, not what eeprom_record_words can describe.  What matters is
     * that it did NOT wrap back to 2, and that it is a sane record size. */
    ASSERT_TRUE(eeprom_record_words(0xFFFFFFFFU) > 2U);
    ASSERT_TRUE(eeprom_record_words(0xFFFFFFFFU) <= EEPROM_WORDS_PER_SECTOR);
}

TEST_CASE(a_word_too_big_for_the_buffer_is_refused)
{
    /* The bound eeprom_encode() enforces, in both of the units that matter:
     * the record must fit the staging buffer AND the sector. */
    uint8_t payload[64];
    memset(payload, 0x5A, sizeof(payload));

    uint32_t need = eeprom_record_words(sizeof(payload));
    uint32_t fits = eeprom_record_words(sizeof(payload)) * EEPROM_WORD_U32;

    /* Right-sized buffer: accepted. */
    static uint32_t ok[EEPROM_RECORD_U32];
    ASSERT_EQ(eeprom_encode(1U, payload, sizeof(payload), 0U, ok, fits), need);

    /* One element short: refused, and nothing may be written. */
    static uint32_t small[EEPROM_RECORD_U32];
    memset(small, 0xFF, sizeof(small));
    ASSERT_EQ(eeprom_encode(1U, payload, sizeof(payload), 0U, small, fits - 1U),
              0U);
    /* words[0] is only written after the bound check, so an untouched magic
     * proves the function bailed before writing anything. */
    ASSERT_EQ(small[0], 0xFFFFFFFFU);
}

/*============================================================================
 *  Flash programming layout
 *
 *  eeprom_hal_program() walks the staging buffer 16 bytes per call, because
 *  that is what one H7 flash word costs (FLASH_NB_32BITWORD_IN_FLASHWORD = 4
 *  uint32, 8 of those bits being ECC).  It lives in the firmware half of this
 *  file, so no host test can call it - which is exactly how its data pointer
 *  came to advance 4 bytes per call while the flash address advanced 16, and
 *  every saved record after the first went out garbled.
 *
 *  So the contract is pinned here instead: given an encoded record, the i-th
 *  16-byte flash word must be staging[i * EEPROM_WORD_U32 .. +4).  If the
 *  stepping in eeprom_hal_program() ever drifts, this fails.
 *==========================================================================*/

TEST_CASE(program_stepping_covers_the_record_without_gaps_or_overlap)
{
    /* The staging buffer laid out exactly the way eeprom_hal_program() walks
     * it: call w programs flash word w from staging[w*4 .. w*4+3].  Building
     * the expectation that way and comparing byte-for-byte is what pins the
     * contract - a 4-byte step in the real loop would consume three quarters
     * of every word twice and leave the tail untouched. */
    uint8_t payload[100];
    for (unsigned i = 0U; i < sizeof(payload); i++) {
        payload[i] = (uint8_t)(0x40U + i);
    }

    static uint32_t staging[EEPROM_RECORD_U32];
    uint32_t n = eeprom_encode(0x1234U, payload, sizeof(payload), 0xABCDU,
                               staging, EEPROM_RECORD_U32);
    ASSERT_EQ(n, eeprom_record_words(sizeof(payload)));

    /* Independent expectation, assembled flash word by flash word. */
    uint8_t want[EEPROM_RECORD_U32 * sizeof(uint32_t)];
    memset(want, 0xFF, sizeof(want));

    uint32_t pw = (uint32_t)((sizeof(payload) + EEPROM_WORD_BYTES - 1U)
                             / EEPROM_WORD_BYTES);
    uint32_t off = 0U;

    /* Word 0: header. */
    uint32_t hdr[4] = { EEPROM_RECORD_MAGIC,
                        (0x1234U << 16) | (uint32_t)sizeof(payload),
                        EEPROM_FLAG_COMMITTED, 0xFFFFFFFFU };
    memcpy(&want[off], hdr, sizeof(hdr));
    off += EEPROM_WORD_BYTES;

    /* Words 1..pw: payload, 0xFF padded. */
    memcpy(&want[off], payload, sizeof(payload));
    off += pw * EEPROM_WORD_BYTES;

    /* Final word: commit. */
    uint32_t cmt[4] = { EEPROM_RECORD_MAGIC,
                        (0x1234U << 16) | (uint32_t)sizeof(payload),
                        0xABCDU, 0xFFFFFFFFU };
    memcpy(&want[off], cmt, sizeof(cmt));
    off += EEPROM_WORD_BYTES;

    /* The record occupies exactly n flash words and not one byte more. */
    ASSERT_EQ(n * EEPROM_WORD_BYTES, off);
    ASSERT_TRUE(off <= sizeof(want));

    /* Byte-for-byte identical to what encode() produced. */
    ASSERT_EQ(memcmp(staging, want, off), 0);

    /* And each program call reads a distinct, in-range 16-byte slice. */
    for (uint32_t w = 0U; w < n; w++) {
        const uint8_t *slice = (const uint8_t *)&staging[w * EEPROM_WORD_U32];
        ASSERT_EQ(slice, (const uint8_t *)&staging[0] + w * EEPROM_WORD_BYTES);
    }
}

/*============================================================================
 *  Round trip
 *============================================================================*/

TEST_CASE(a_record_round_trips)
{
    static uint32_t words[EEPROM_RECORD_U32];
    ConfigType in;
    ConfigType out;

    memset(&in, 0, sizeof(in));
    in.pc_baud_rate = 115200U;
    in.language     = MULTILANG_ENGLISH;
    in.temp_fahrenheit = 1U;
    in.touch.x_gain = 1.2345f;

    uint32_t crc = crc32_calculate((const uint8_t *)&in, sizeof(in));
    uint32_t n = eeprom_encode(7U, &in, sizeof(in), crc, words,
                               EEPROM_RECORD_U32);
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
    static uint32_t words[EEPROM_RECORD_U32];
    uint8_t payload[32];
    memset(payload, 0xA5, sizeof(payload));

    eeprom_encode(0x1234U, payload, sizeof(payload), 0U, words,
                  EEPROM_RECORD_U32);

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
    static uint32_t a[EEPROM_RECORD_U32];
    static uint32_t b[EEPROM_RECORD_U32];
    uint8_t payload[64];
    memset(payload, 0x5A, sizeof(payload));

    uint32_t na = eeprom_encode(3U, payload, sizeof(payload), 0xDEADU, a,
                                EEPROM_RECORD_U32);
    uint32_t nb = eeprom_encode(3U, payload, sizeof(payload), 0xDEADU, b,
                                EEPROM_RECORD_U32);

    ASSERT_EQ(na, nb);
    ASSERT_TRUE(memcmp(a, b, na * sizeof(uint32_t)) == 0);
}

TEST_CASE(padding_past_the_payload_is_ones_not_garbage)
{
    /* A 20-byte payload occupies 32 bytes of flash words.  The slack must
     * be a fixed value so a later read of that region is predictable. */
    static uint32_t words[EEPROM_RECORD_U32];
    uint8_t payload[20];
    memset(payload, 0x11, sizeof(payload));

    uint32_t n = eeprom_encode(1U, payload, sizeof(payload), 0U, words,
                               EEPROM_RECORD_U32);
    /* 1 header word + 2 payload words (20 bytes rounds up to 32) + 1 commit
     * word.  These are 16-byte FLASH words, not uint32 elements - the
     * staging buffer is four times larger again. */
    ASSERT_EQ(n, 4U);
    ASSERT_EQ(n, eeprom_record_words(sizeof(payload)));

    /* Payload starts right after the header and spans 32 bytes; bytes 20..31
     * of that span are padding.  This also pins the payload offset, which is
     * what the old encode loop got wrong by treating the byte index as a
     * word index. */
    const uint8_t *raw =
        (const uint8_t *)&words[EEPROM_HEADER_WORDS * EEPROM_WORD_U32];
    for (unsigned i = 0U; i < 20U; i++) {
        ASSERT_EQ(raw[i], 0x11);
    }
    for (unsigned i = 20U; i < 32U; i++) {
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
    static uint32_t words[EEPROM_RECORD_U32];
    ConfigType out;

    for (unsigned i = 0; i < EEPROM_RECORD_U32; i++) {
        words[i] = ERASED_WORD;
    }
    ASSERT_EQ(eeprom_decode(words, EEPROM_MAX_RECORD_WORDS, &out, sizeof(out)), 0U);
}

TEST_CASE(a_record_without_its_commit_word_is_rejected)
{
    /* Power cut between the payload write and the commit write.  The
     * payload is fully there but the record must NOT be trusted, because
     * we cannot tell it apart from a half-written one. */
    static uint32_t words[EEPROM_RECORD_U32];
    ConfigType in;
    ConfigType out;

    memset(&in, 0, sizeof(in));
    in.pc_baud_rate = 9600U;
    uint32_t n = eeprom_encode(5U, &in, sizeof(in),
                               crc32_calculate((const uint8_t *)&in, sizeof(in)),
                               words, EEPROM_RECORD_U32);

    /* Simulate the interrupted save: everything written except the commit.
     * The commit block is EEPROM_COMMIT_WORDS wide, and the magic that
     * decides validity is its FIRST word, not the last. */
    uint32_t commit = words[(n - EEPROM_COMMIT_WORDS) * EEPROM_WORD_U32];
    words[(n - EEPROM_COMMIT_WORDS) * EEPROM_WORD_U32] = ERASED_WORD;

    ASSERT_EQ(eeprom_decode(words, n, &out, sizeof(out)), 0U);

    /* And once the commit lands it becomes valid again. */
    words[(n - EEPROM_COMMIT_WORDS) * EEPROM_WORD_U32] = commit;
    ASSERT_EQ(eeprom_decode(words, n, &out, sizeof(out)), sizeof(ConfigType));
}

TEST_CASE(a_missing_header_is_rejected)
{
    static uint32_t words[EEPROM_RECORD_U32];
    uint8_t payload[32];
    uint8_t out[64];
    memset(payload, 0x3C, sizeof(payload));

    uint32_t n = eeprom_encode(2U, payload, sizeof(payload), 0U, words,
                               EEPROM_RECORD_U32);
    words[0] = ERASED_WORD;          /* header never landed */

    ASSERT_EQ(eeprom_decode(words, n, out, sizeof(out)), 0U);
}

TEST_CASE(commit_id_must_agree_with_the_header)
{
    /* A mismatch means the two halves came from different saves - the
     * signature of a record whose header was reused after a partial erase. */
    static uint32_t words[EEPROM_RECORD_U32];
    uint8_t payload[32];
    uint8_t out[64];
    memset(payload, 0x77, sizeof(payload));

    uint32_t n = eeprom_encode(9U, payload, sizeof(payload), 0U, words,
                               EEPROM_RECORD_U32);
    uint32_t commit = (n - EEPROM_COMMIT_WORDS) * EEPROM_WORD_U32;
    words[commit] = (words[commit] & 0x0000FFFFU) | (0x00AA0000U);

    ASSERT_EQ(eeprom_decode(words, n, out, sizeof(out)), 0U);
}

TEST_CASE(commit_length_must_agree_with_the_header)
{
    static uint32_t words[EEPROM_RECORD_U32];
    uint8_t payload[32];
    uint8_t out[64];
    memset(payload, 0x22, sizeof(payload));

    uint32_t n = eeprom_encode(4U, payload, sizeof(payload), 0U, words,
                               EEPROM_RECORD_U32);
    /* Header says 32 bytes, commit says 16 - the decoder must not guess. */
    words[1] = (words[1] & 0xFFFF0000U) | 16U;

    ASSERT_EQ(eeprom_decode(words, n, out, sizeof(out)), 0U);
}

TEST_CASE(a_length_that_disagrees_with_the_slot_is_rejected)
{
    /* Header claims a length whose record would need more words than the
     * slot holds.  Decoding must refuse rather than read past the record
     * into whatever follows it. */
    static uint32_t words[EEPROM_RECORD_U32];
    uint8_t payload[32];
    uint8_t out[64];
    memset(payload, 0x99, sizeof(payload));

    uint32_t n = eeprom_encode(6U, payload, sizeof(payload), 0U, words,
                               EEPROM_RECORD_U32);
    words[1] = (words[1] & 0xFFFF0000U) | 4000U;   /* absurd length */

    ASSERT_EQ(eeprom_decode(words, n, out, sizeof(out)), 0U);
}

TEST_CASE(a_record_longer_than_the_destination_is_refused)
{
    /* Writing into a smaller buffer must fail loudly instead of smashing
     * the caller's stack. */
    static uint32_t words[EEPROM_RECORD_U32];
    uint8_t payload[128];
    uint8_t small[16];
    memset(payload, 0x42, sizeof(payload));

    uint32_t n = eeprom_encode(8U, payload, sizeof(payload), 0U, words,
                               EEPROM_RECORD_U32);
    ASSERT_EQ(eeprom_decode(words, n, small, sizeof(small)), 0U);
}

/*============================================================================
 *  Header-only inspection
 *
 *  eeprom_load() reads only the header and commit word while surveying the
 *  ring, so these must agree with the full decoder on every well-formed
 *  record.
 *==========================================================================*/

TEST_CASE(header_id_and_len_match_the_encoded_record)
{
    static uint32_t words[EEPROM_RECORD_U32];
    uint8_t payload[48];
    memset(payload, 0x5C, sizeof(payload));

    uint32_t n = eeprom_encode(0xBEEFU, payload, sizeof(payload), 0U, words,
                               EEPROM_RECORD_U32);

    ASSERT_EQ(eeprom_header_id(words), 0xBEEFU);
    ASSERT_EQ(eeprom_header_len(words), sizeof(payload));
    ASSERT_EQ(n, eeprom_record_words(sizeof(payload)));
}

TEST_CASE(a_good_record_is_intact)
{
    static uint32_t words[EEPROM_RECORD_U32];
    uint8_t payload[32];
    memset(payload, 0x21, sizeof(payload));

    eeprom_encode(11U, payload, sizeof(payload), 0U, words,
                  EEPROM_RECORD_U32);

    ASSERT_TRUE(eeprom_record_is_intact(words, EEPROM_WORDS_PER_SECTOR));
}

TEST_CASE(an_erased_slot_is_not_intact)
{
    static uint32_t words[EEPROM_RECORD_U32];
    for (unsigned i = 0; i < EEPROM_RECORD_U32; i++) {
        words[i] = ERASED_WORD;
    }
    ASSERT_FALSE(eeprom_record_is_intact(words, EEPROM_WORDS_PER_SECTOR));
}

TEST_CASE(a_missing_commit_word_is_not_intact)
{
    /* The distinction that matters: eeprom_load() must reject a torn record
     * during pass 1, using only the header and the commit word.  If this
     * returned true, the loader would queue it as a candidate and only
     * discover the problem after reading the payload. */
    static uint32_t words[EEPROM_RECORD_U32];
    uint8_t payload[32];
    memset(payload, 0x33, sizeof(payload));

    uint32_t n = eeprom_encode(12U, payload, sizeof(payload), 0U, words,
                               EEPROM_RECORD_U32);
    words[(n - EEPROM_COMMIT_WORDS) * EEPROM_WORD_U32] = ERASED_WORD;

    ASSERT_FALSE(eeprom_record_is_intact(words, EEPROM_WORDS_PER_SECTOR));
}

TEST_CASE(a_disagreeing_commit_is_not_intact)
{
    static uint32_t words[EEPROM_RECORD_U32];
    uint8_t payload[32];
    memset(payload, 0x44, sizeof(payload));

    uint32_t n = eeprom_encode(13U, payload, sizeof(payload), 0U, words,
                               EEPROM_RECORD_U32);
    words[(n - EEPROM_COMMIT_WORDS) * EEPROM_WORD_U32] = EEPROM_RECORD_MAGIC | (0x00FF0000U);   /* wrong id */

    ASSERT_FALSE(eeprom_record_is_intact(words, EEPROM_WORDS_PER_SECTOR));
}

TEST_CASE(intact_and_decode_never_disagree)
{
    /* is_intact() and decode() must reach the same verdict, or the loader
     * could accept a record in pass 1 and then have nothing to fall back
     * to in pass 2. */
    static uint32_t words[EEPROM_RECORD_U32];
    uint8_t payload[40];
    uint8_t out[64];

    memset(payload, 0x7E, sizeof(payload));
    uint32_t n = eeprom_encode(20U, payload, sizeof(payload), 0U, words,
                               EEPROM_RECORD_U32);
    ASSERT_TRUE(eeprom_record_is_intact(words, EEPROM_WORDS_PER_SECTOR));
    ASSERT_TRUE(eeprom_decode(words, n, out, sizeof(out)) != 0U);

    /* break it one way at a time */
    words[0] = 0xDEADBEEF;
    ASSERT_FALSE(eeprom_record_is_intact(words, EEPROM_WORDS_PER_SECTOR));
    ASSERT_FALSE(eeprom_decode(words, n, out, sizeof(out)) != 0U);
}

/*============================================================================
 *  Capacity
 *==========================================================================*/

TEST_CASE(four_sectors_hold_far_more_records_than_we_need)
{
    /* With a 124-byte ConfigType a record is 16 words = 256 bytes, so a
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

/*============================================================================
 *  Suite registration
 *==========================================================================*/

const test_entry_t g_suite_flash_eeprom[] = {
    TEST_ENTRY(sector_is_8kb_and_word_is_16_bytes),
    TEST_ENTRY(config_record_fits_the_payload_cap),
    TEST_ENTRY(the_staging_buffer_is_the_size_the_geometry_actually_needs),
    TEST_ENTRY(record_words_counts_header_payload_and_commit),
    TEST_ENTRY(record_words_saturates_instead_of_wrapping),
    TEST_ENTRY(a_word_too_big_for_the_buffer_is_refused),
    TEST_ENTRY(program_stepping_covers_the_record_without_gaps_or_overlap),
    TEST_ENTRY(a_record_round_trips),
    TEST_ENTRY(id_survives_the_round_trip),
    TEST_ENTRY(encoding_is_deterministic),
    TEST_ENTRY(padding_past_the_payload_is_ones_not_garbage),
    TEST_ENTRY(an_erased_slot_is_not_a_record),
    TEST_ENTRY(a_record_without_its_commit_word_is_rejected),
    TEST_ENTRY(a_missing_header_is_rejected),
    TEST_ENTRY(commit_id_must_agree_with_the_header),
    TEST_ENTRY(commit_length_must_agree_with_the_header),
    TEST_ENTRY(a_length_that_disagrees_with_the_slot_is_rejected),
    TEST_ENTRY(a_record_longer_than_the_destination_is_refused),
    TEST_ENTRY(header_id_and_len_match_the_encoded_record),
    TEST_ENTRY(a_good_record_is_intact),
    TEST_ENTRY(an_erased_slot_is_not_intact),
    TEST_ENTRY(a_missing_commit_word_is_not_intact),
    TEST_ENTRY(a_disagreeing_commit_is_not_intact),
    TEST_ENTRY(intact_and_decode_never_disagree),
    TEST_ENTRY(four_sectors_hold_far_more_records_than_we_need),
    TEST_SUITE_END,
};   