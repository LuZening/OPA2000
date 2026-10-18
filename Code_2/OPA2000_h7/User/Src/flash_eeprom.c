/**
 * @file flash_eeprom.c
 * @brief Configuration storage on the STM32H7B0's internal flash.
 *
 * @see flash_eeprom.h for the geometry and the wear-leveling scheme.
 *
 * The record encode/decode logic is kept free of HAL calls so it can be
 * exercised on the host (User/Tests/test_flash_eeprom.c).  Everything that
 * actually touches the flash array goes through the eeprom_hal_* wrappers
 * at the bottom of this file.
 */

#include "flash_eeprom.h"
#include "opa2000.h"

#include <string.h>

/*============================================================================
 *  Record encoding
 *
 *  A record occupies consecutive 16-byte words inside one sector:
 *
 *      word 0        : header  (magic, id, payload length, flags)
 *      word 1..n-1   : payload, zero padded to a word boundary
 *      word n        : commit  (magic again, id again, payload CRC32)
 *
 *  The commit word is programmed last.  Until it lands the record reads as
 *  all-ones in that word, so a record interrupted by a reset is skipped
 * rather than mistaken for a valid one.
 *==========================================================================*/

static EepromStatsType s_stats;

/*============================================================================
 *  Unit conversion - the one place flash words become uint32 indices
 *==========================================================================
 *
 *  Two units meet in this file and confusing them is what caused the payload
 *  overflow: the record geometry is counted in 16-byte FLASH words (that is
 *  what the sector arithmetic and the ECC granularity both use), while every
 *  buffer in the program is a uint32_t array, where one index step is 4
 *  bytes.  Indexing a uint32_t buffer with a flash-word count silently
 *  quarters every offset.
 *
 *  So the conversion lives here, in one named function, and nowhere else is
 *  a flash-word count used as an index.  If you find yourself wanting to
 *  write words[n] with an n that came from eeprom_record_words(), what you
 *  actually want is eeprom_word_to_u32(n).
 *==========================================================================*/

/** @brief Flash-word index -> uint32_t element index.
 *  @param word_index  position in 16-byte flash words
 *  @return            the same position as a uint32_t array index
 */
static inline uint32_t eeprom_word_to_u32(uint32_t word_index)
{
    return word_index * EEPROM_WORD_U32;
}

/*============================================================================
 *  Host-testable helpers
 *==========================================================================*/

/* How many 16-byte flash words a record of \p len payload bytes needs,
 * including the header and the commit word.
 *
 * Saturates rather than wrapping.  The rounding term (len + 15) would
 * overflow for len > 0xFFFFFFF0 and hand back a need of 2, which then
 * passes every "does it fit" check downstream and lets a huge memcpy run off
 * the staging buffer.  The cap here is one sector: a record can never be
 * larger than that, so clamping to it is both safe and the honest answer. */
uint32_t eeprom_record_words(uint32_t len)
{
    /* header + payload rounded up + commit */
    uint32_t max_payload = EEPROM_SECTOR_BYTES - EEPROM_MAX_RECORD_OVERHEAD_BYTES;

    if (len > max_payload) {
        len = max_payload;
    }
    uint32_t payload_words = (len + EEPROM_WORD_BYTES - 1U) / EEPROM_WORD_BYTES;
    return EEPROM_HEADER_WORDS + payload_words + EEPROM_COMMIT_WORDS;
}

/* Serialise \p src into \p words, ready to be programmed.  Returns the
 * word count, or 0 if the record would not fit in a sector.
 *
 * The payload is copied verbatim - memcpy of the caller's raw bytes, no
 * byte-at-a-time loop and no per-field interpretation.  The caller hands us
 * a plain struct (ConfigType), which contains only fixed-width integers and
 * floats and no pointers, so its in-memory image IS its stored image. */
uint32_t eeprom_encode(uint32_t id,
                       const void *src, uint32_t len,
                       uint32_t crc,
                       uint32_t *words, uint32_t max_words)
{
    uint32_t need = eeprom_record_words(len);
    uint32_t payload_words = need - EEPROM_HEADER_WORDS - EEPROM_COMMIT_WORDS;
    /* Payload start and commit start, both as uint32_t indices into words[].
     * Every geometry count goes through eeprom_word_to_u32() so the
     * flash-word -> element conversion is done in exactly one place. */
    uint32_t pay = eeprom_word_to_u32(EEPROM_HEADER_WORDS);
    uint8_t *payload = (uint8_t *)&words[pay];
    uint32_t pad;

    if (words == NULL || src == NULL ||
        eeprom_word_to_u32(need) > max_words) {
        return 0U;
    }

    /* Header: magic | id | len | flags, then a spare word.  All four live in
     * the first flash word, i.e. the first EEPROM_WORD_U32 elements. */
    words[0] = EEPROM_RECORD_MAGIC;
    words[1] = ((uint32_t)id << 16) | (uint32_t)len;
    words[2] = EEPROM_FLAG_COMMITTED;
    words[3] = 0xFFFFFFFFU;   /* spare, kept erased-looking */

    /* Payload: one memcpy of the caller's raw bytes, then fill the tail of
     * the last word with 0xFF.  The fill must be deterministic - the same
     * config saved twice has to produce identical bytes, otherwise the
     * sector fills with records differing only in their slack. */
    memcpy(payload, src, len);
    pad = payload_words * EEPROM_WORD_BYTES - len;
    if (pad > 0U) {
        memset(&payload[len], 0xFF, pad);
    }

    /* Commit, programmed last so an interrupted save leaves it erased and
     * the record is skipped rather than half-believed. */
    {
        uint32_t cmt = eeprom_word_to_u32(EEPROM_HEADER_WORDS + payload_words);

        words[cmt + 0U] = EEPROM_RECORD_MAGIC;
        words[cmt + 1U] = ((uint32_t)id << 16) | (uint32_t)len;
        words[cmt + 2U] = crc;
        words[cmt + 3U] = 0xFFFFFFFFU;
    }

    return need;
}

/* Validate a serialised record and, if it is good, copy the payload out.
 * Returns the payload length, or 0 when the record is not usable. */
uint32_t eeprom_decode(const uint32_t *words, uint32_t n_words,
                       void *out, uint32_t out_size)
{
    uint32_t need;
    uint32_t len;
    uint32_t id;

    if (words == NULL || n_words < 3U) {
        return 0U;
    }

    /* Header must be present.  An erased word reads 0xFFFFFFFF, so this
     * rejects both "never written" and "torn header". */
    if (words[0] != EEPROM_RECORD_MAGIC) {
        return 0U;
    }
    if ((words[2] & EEPROM_FLAG_COMMITTED) == 0U) {
        return 0U;
    }

    id  = words[1] >> 16;
    len = words[1] & 0xFFFFU;

    /* Length has to agree with the slot size, otherwise a bad len would
     * make us read past the record into the next one. */
    need = eeprom_record_words(len);
    if (need != n_words) {
        return 0U;
    }
    if (out == NULL || len > out_size) {
        return 0U;
    }

    /* Commit word must agree with the header.  This is the torn-write
     * check: the commit is programmed last, so a record whose payload was
     * interrupted still has an erased (0xFFFFFFFF) commit word. */
    {
        uint32_t cmt = eeprom_word_to_u32(need - EEPROM_COMMIT_WORDS);

        if (words[cmt + 0U] != EEPROM_RECORD_MAGIC) {
            return 0U;
        }
        if ((words[cmt + 1U] >> 16) != id) {
            return 0U;
        }
        if ((words[cmt + 1U] & 0xFFFFU) != len) {
            return 0U;
        }
    }

    memcpy(out, (const uint8_t *)&words[eeprom_word_to_u32(EEPROM_HEADER_WORDS)],
           len);
    return len;
}

/*============================================================================
 *  Flash back end
 *
 *  Everything below drives real hardware and is compiled out of the host
 *  test build, which only exercises the codec above.
 *==========================================================================*/

#ifndef HOST_TEST_NATIVE

/* Must come first: it supplies the FLASH_CR_PSIZE family that the H7 flash
 * HAL and its legacy header reference unconditionally, and that this part's
 * CMSIS header omits.  See the file header for the full explanation. */
#include "h7b0_flash_compat.h"

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_flash_ex.h"

/*!< Absolute base of EEPROM_FIRST_SECTOR.  128 KB part, 8 KB sectors. */
#define EEPROM_FLASH_BASE   (0x08000000UL + (EEPROM_FIRST_SECTOR * EEPROM_SECTOR_BYTES))

/*!< All error flags, cleared before every operation.  A stale error left by
 *   an interrupted earlier save would otherwise make every subsequent
 *   HAL_FLASH_* call return HAL_ERROR. */
#define EEPROM_FLASH_ERROR_FLAGS  (FLASH_FLAG_ALL_ERRORS_BANK1 | FLASH_FLAG_ALL_ERRORS_BANK2)

static uint32_t eeprom_sector_base(uint32_t sector_index)
{
    return EEPROM_FLASH_BASE + sector_index * EEPROM_SECTOR_BYTES;
}

static bool eeprom_hal_unlock(void)
{
    if (HAL_FLASH_Unlock() != HAL_OK) {
        return false;
    }
    /* Clear whatever stale error flag a previous interrupted operation
     * left behind, otherwise every later call fails immediately. */
    __HAL_FLASH_CLEAR_FLAG(EEPROM_FLASH_ERROR_FLAGS);
    return true;
}

static bool eeprom_hal_erase_sector(uint32_t sector_index)
{
    FLASH_EraseInitTypeDef er = {0};

    er.TypeErase = FLASH_TYPEERASE_SECTORS;
    er.Banks        = FLASH_BANK_1;
    er.Sector       = EEPROM_FIRST_SECTOR + sector_index;
    er.NbSectors    = 1U;
    /* 32-bit parallel programming (PSIZE = 10).  This part always runs at
     * RANGE_3: FLASH_NB_32BITWORD_IN_FLASHWORD is 4, so a flash word is
     * 128 bits regardless, and RANGE_3 is what ST's own H7 examples use. */
    er.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    uint32_t sector_error = 0U;
    __disable_irq();
    bool r = HAL_FLASHEx_Erase(&er, &sector_error) == HAL_OK;
    __enable_irq();
    return r;
}

static bool eeprom_hal_program(uint32_t address, const uint32_t *words, uint32_t n)
{
    /* One HAL_FLASH_Program() call per 16-byte flash word, and the caller
     * guarantees each word is programmed exactly once.
     *
     * Why one call per word and not one call for the whole record: on this
     * part a flash word is 128 bits wide - FLASH_NB_32BITWORD_IN_FLASHWORD
     * is 4, so 4 x uint32 - of which 8 bits are ECC computed over the
     * other 120.  HAL_FLASH_Program() hard-codes that width: it copies
     * exactly FLASH_NB_32BITWORD_IN_FLASHWORD uint32 in its own loop
     * (stm32h7xx_hal_flash.c), and there is no variant that takes more.
     * The ECC granularity is the hardware's, so 16 bytes per call is not a
     * choice we can optimise away.
     *
     * It is also not safe to merge calls: programming part of a word and
     * then coming back for the rest corrupts the ECC, and the next read of
     * that address raises a bus fault that hardfaults the core.  Never
     * retry a failed program on the same address either - erase the sector
     * instead.
     *
     * The data pointer therefore has to advance EEPROM_WORD_U32 elements
     * per call, not one.  Each call reads a full 16 bytes out of it, and the
     * destination advances 16 bytes too, so the two stay in step.
     *
     * DataAddress is an ADDRESS, not a pointer to the data - the HAL casts
     * it straight back to a pointer and dereferences it
     * (stm32h7xx_hal_flash.c:164: `src_addr = (__IO uint32_t*)DataAddress`).
     * Passing `words` itself would hand the HAL the pointer *value*, so it
     * would read from wherever that number happens to be an address and
     * program that garbage into flash.  Hence the explicit cast of the
     * buffer to its own address. */
    bool r= true;
    __disable_irq();
    for (uint32_t i = 0U; i < n; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                              address + i * EEPROM_WORD_BYTES,
                              (uint32_t)(uintptr_t)&words[i * EEPROM_WORD_U32])
            != HAL_OK)
        {
            r = false;
            break;
        }
    }
    __enable_irq();
    return r;
}

#endif /* !HOST_TEST_NATIVE */

/* Read the record id out of a slot's header word 1, without touching the
 * payload. */
uint32_t eeprom_header_id(const uint32_t *words)
{
    return (words[1] >> 16) & 0xFFFFU;
}

/* Payload length from the header, without touching the payload. */
uint32_t eeprom_header_len(const uint32_t *words)
{
    return words[1] & 0xFFFFU;
}

/* Check just the framing: header magic, committed flag, and agreement
 * between the header and the commit word at the end of the record.
 *
 * \p avail_words is how many 16-byte words remain between \p words and the
 * end of the sector it lives in.  The length is stored in flash, so it is
 * untrusted input: without that ceiling this function would happily read
 * words[need-1] past the sector boundary to fetch the commit word, and on
 * the last sector that runs off the end of the EEPROM area entirely.  The
 * bound has to be enforced HERE rather than by the caller because this is
 * the function that does the far read.
 *
 * The payload CRC is deliberately NOT verified here - that would mean
 * reading the payload, which is exactly what the two-pass load avoids.
 * A record that passes this but fails its CRC is caught by the full
 * eeprom_decode() in pass 2. */
bool eeprom_record_is_intact(const uint32_t *words, uint32_t avail_words)
{
    uint32_t len;
    uint32_t need;

    if (words[0] != EEPROM_RECORD_MAGIC) {
        return false;
    }
    if ((words[2] & EEPROM_FLAG_COMMITTED) == 0U) {
        return false;
    }

    len  = words[1] & 0xFFFFU;
    need = eeprom_record_words(len);
    if (need > avail_words) {
        return false;       /*!< does not fit in what is left of the sector */
    }

    /* The commit word is programmed last, so an interrupted save leaves it
     * erased.  It must also agree with the header: a mismatch means the two
     * halves came from different saves. */
    {
        uint32_t cmt = eeprom_word_to_u32(need - EEPROM_COMMIT_WORDS);

        if (words[cmt + 0U] != EEPROM_RECORD_MAGIC) {
            return false;
        }
        if (eeprom_header_id(words) != (words[cmt + 1U] >> 16)) {
            return false;
        }
        if (len != (words[cmt + 1U] & 0xFFFFU)) {
            return false;
        }
    }
    return true;
}

static bool eeprom_word_is_erased(const uint32_t *words)
{
    return (words[0] == 0xFFFFFFFFU);
}

/*============================================================================
 *  Public API
 *
 *  These drive the real flash array through the eeprom_hal_* wrappers above,
 *  so they are compiled out of the host test build - which exercises the
 *  codec and the framing checks above instead.
 *==========================================================================*/

#ifndef HOST_TEST_NATIVE

bool eeprom_load(void *out, uint32_t out_size)
{
    /* Two passes, and the split is deliberate: it is what keeps the RAM
     * cost at zero.
     *
     * Pass 1 walks every slot reading ONLY the 4-word header.  It collects
     * the id and address of each framing-valid record.  The payload is
     * never touched, so a ring holding 128 records costs 128 bytes of flash
     * reads, not 16 KB.
     *
     * Pass 2 walks that list newest-first and stops at the first record
     * whose payload actually decodes.
     *
     * The single-pass alternative would copy every candidate payload into
     * `out` on the way past, overwriting the good copy with a worse one
     * whenever the newest turned out to be torn.  Here `out` is written
     * exactly once, by the record that survives.
     */
    /* Candidate list: one entry per framing-valid record.  Bounded by the
     * ring capacity (4 sectors x 32 records), so this is a fixed 1 KB worst
     * case and there is no allocation and no failure path. */
    static struct { uint32_t addr; uint32_t words; uint32_t id; }
           s_cand[EEPROM_MAX_RECORDS];
    uint32_t n_cand = 0U;

    if (out == NULL) {
        return false;
    }

    /* ---- Pass 1: locate every framing-valid record, headers only ---- */
    for (uint32_t sec = 0U; sec < EEPROM_SECTOR_COUNT; sec++) {
        uint32_t base = eeprom_sector_base(sec);

        for (uint32_t w = 0U; w < EEPROM_WORDS_PER_SECTOR; w += 2U) {
            uint32_t addr = base + w * EEPROM_WORD_BYTES;
            const uint32_t *words = (const uint32_t *)addr;
            uint32_t avail = EEPROM_WORDS_PER_SECTOR - w;
            uint32_t len;
            uint32_t need;

            if (eeprom_word_is_erased(words)) {
                /* Slots are filled in order, so once one is erased nothing
                 * further along in this sector can be valid. */
                break;
            }

            /* Header and commit word only - never the payload between
             * them.  This is the whole point of the two-pass split.
             * `avail` is what keeps the commit-word fetch inside the
             * sector: a length that would push it past the end is rejected
             * before any far read happens, not after. */
            if (!eeprom_record_is_intact(words, avail)) {
                s_stats.loads_corrupt++;
                continue;
            }

            len  = eeprom_header_len(words);
            need = eeprom_record_words(len);

            if (n_cand < EEPROM_MAX_RECORDS) {
                s_cand[n_cand].addr  = addr;
                s_cand[n_cand].words = need;
                s_cand[n_cand].id    = eeprom_header_id(words);
                n_cand++;
            }

            /* Jump past this record; the loop increments w by 2 on top.
             * need <= avail is guaranteed by eeprom_record_is_intact() above,
             * so w stays inside the sector and the next slot's header is
             * still 16-byte aligned (both terms are even multiples of 2). */
            w += (need - 1U) * 2U;
        }
    }

    if (n_cand == 0U) {
        s_stats.loads_empty++;
        return false;
    }

    /* ---- Pass 2: newest first, stop at the first that decodes ---- */
    /* Selection sort by id, descending, done in place.  At most a few
     * records are ever examined in practice, and insertion into an already
     * sorted list keeps the inner loop tiny. */
    for (uint32_t i = 1U; i < n_cand; i++) {
        uint32_t key_addr  = s_cand[i].addr;
        uint32_t key_words = s_cand[i].words;
        uint32_t key_id    = s_cand[i].id;
        uint32_t j = i;

        while (j > 0U && s_cand[j - 1U].id < key_id) {
            s_cand[j] = s_cand[j - 1U];
            j--;
        }
        s_cand[j].addr  = key_addr;
        s_cand[j].words = key_words;
        s_cand[j].id    = key_id;
    }

    for (uint32_t i = 0U; i < n_cand; i++) {
        if (eeprom_decode((const uint32_t *)s_cand[i].addr, s_cand[i].words,
                          out, out_size) != 0U) {
            if (i > 0U) {
                /* The newer ones were framing-valid but their payloads did
                 * not decode, i.e. bit rot or a marginal brown-out during
                 * the write.  Falling back to an older intact record is the
                 * whole reason several are kept. */
                s_stats.loads_corrupt += i;
            }
            s_stats.loads_ok++;
            return true;
        }
        s_stats.loads_corrupt++;
    }

    /* Every candidate failed its payload CRC: treat it as no configuration
     * rather than handing back a partially valid struct. */
    s_stats.loads_empty++;
    return false;
}

uint32_t eeprom_current_id(void)
{
    uint32_t best = 0U;
    bool found = false;

    for (uint32_t sec = 0U; sec < EEPROM_SECTOR_COUNT; sec++) {
        uint32_t base = eeprom_sector_base(sec);
        for (uint32_t w = 0U; w < EEPROM_WORDS_PER_SECTOR; w += 2U) {
            uint32_t addr = base + w * EEPROM_WORD_BYTES;
            const uint32_t *words = (const uint32_t *)addr;
            if (eeprom_word_is_erased(words)) {
                break;
            }
            if (words[0] != EEPROM_RECORD_MAGIC) {
                continue;
            }
            uint32_t id = eeprom_header_id(words);
            if (!found || id > best) {
                best = id;
                found = true;
            }
            /* Step to the next record rather than the next pair of words:
             * the length lives in the header, so this cannot land inside
             * the record we just read. */
            w += (eeprom_record_words(eeprom_header_len(words)) - 1U) * 2U;
        }
    }
    return best;
}

bool eeprom_save(const void *src, uint32_t len)
{
    uint32_t need = eeprom_record_words(len);
    uint32_t record_bytes = need * EEPROM_WORD_BYTES;
    uint32_t next_id;
    uint32_t target_sec = 0U;
    uint32_t target_slot = 0U;
    bool erased_first = true;

    if (src == NULL || len == 0U || record_bytes > EEPROM_SECTOR_BYTES) {
        return false;
    }

    /* Find the sector that still has room for one more record.  Slots are
     * walked in record-sized steps and stop at the first erased header:
     * writes fill a sector front to back, so the first erased header is
     * exactly where the next record belongs.  Stepping by the real record
     * size matters - a record is at least four words, so scanning word by
     * word would stop in the middle of the previous one. */
    for (uint32_t sec = 0U; sec < EEPROM_SECTOR_COUNT && erased_first; sec++) {
        uint32_t base = eeprom_sector_base(sec);
        for (uint32_t w = 0U; w < EEPROM_WORDS_PER_SECTOR; w += need) {
            const uint32_t *words =
                (const uint32_t *)(base + w * EEPROM_WORD_BYTES);
            if (eeprom_word_is_erased(words)) {
                target_sec  = sec;
                target_slot = w;
                erased_first = false;
                break;
            }
        }
    }

    if (erased_first) {
        /* Every sector is full.  Reclaim the oldest one: that is where the
         * highest id is NOT, so erasing it loses nothing that is still the
         * newest copy.  Scanning for the lowest id is what makes the wear
         * spread across all four sectors instead of always hitting the
         * same one. */
        uint32_t lowest = 0xFFFFFFFFU;
        for (uint32_t sec = 0U; sec < EEPROM_SECTOR_COUNT; sec++) {
            uint32_t base = eeprom_sector_base(sec);
            uint32_t sec_min = 0xFFFFFFFFU;
            bool any = false;
            for (uint32_t w = 0U; w < EEPROM_WORDS_PER_SECTOR; w += 2U) {
                uint32_t addr = base + w * EEPROM_WORD_BYTES;
                const uint32_t *words = (const uint32_t *)addr;
                if (eeprom_word_is_erased(words)) {
                    break;
                }
                if (words[0] != EEPROM_RECORD_MAGIC) {
                    continue;
                }
                uint32_t id = eeprom_header_id(words);
                if (!any || id < sec_min) {
                    sec_min = id;
                    any = true;
                }
            }
            if (any && sec_min < lowest) {
                lowest = sec_min;
                target_sec = sec;
            }
        }

        if (lowest == 0xFFFFFFFFU) {
            s_stats.save_failures++;
            return false;
        }

        if (!eeprom_hal_unlock() || !eeprom_hal_erase_sector(target_sec)) {
            s_stats.save_failures++;
            return false;
        }
        s_stats.sector_erases++;
        target_slot = 0U;
    }

    next_id = eeprom_current_id() + 1U;

    {
        /* Sized to the record, not the sector: ConfigType is 128 bytes so a
         * record is 1 + 8 + 1 = 10 flash words = 160 bytes = 40 uint32.  A
         * sector-sized buffer would be 2 KB of static RAM for nothing.
         *
         * EEPROM_RECORD_U32, not EEPROM_MAX_RECORD_WORDS: the array is
         * uint32_t[], so it needs four elements per 16-byte flash word. */
        static uint32_t staging[EEPROM_RECORD_U32];
        uint32_t words;

        if (need * EEPROM_WORD_U32 > EEPROM_RECORD_U32) {
            s_stats.save_failures++;
            return false;
        }

        words = eeprom_encode(next_id, src, len,
                              crc32_calculate((const uint8_t *)src, len),
                              staging, EEPROM_RECORD_U32);
        if (words != need) {
            s_stats.save_failures++;
            return false;
        }

        if (!eeprom_hal_unlock() ||
            !eeprom_hal_program(eeprom_sector_base(target_sec)
                                    + target_slot * EEPROM_WORD_BYTES,
                                staging, words)) {
            s_stats.save_failures++;
            return false;
        }
    }

    (void)HAL_FLASH_Lock();
    s_stats.saves_ok++;
    s_stats.saves_since_erase++;
    return true;
}

#endif /* !HOST_TEST_NATIVE */

void eeprom_get_stats(EepromStatsType *out)
{
    if (out != NULL) {
        *out = s_stats;
    }
}

void eeprom_clear_stats(void)
{
    memset(&s_stats, 0, sizeof(s_stats));
}
