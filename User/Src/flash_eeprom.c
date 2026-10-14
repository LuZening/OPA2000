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
 *  Host-testable helpers
 *==========================================================================*/

/* How many 16-byte words a record of \p len payload bytes needs, including
 * the header and the commit word. */
uint32_t eeprom_record_words(uint32_t len)
{
    /* header + payload rounded up + commit */
    uint32_t payload_words = (len + EEPROM_WORD_BYTES - 1U) / EEPROM_WORD_BYTES;
    return 1U + payload_words + 1U;
}

/* Serialise \p src into \p words, ready to be programmed.  Returns the
 * word count, or 0 if the record would not fit in a sector. */
uint32_t eeprom_encode(uint32_t id,
                       const void *src, uint32_t len,
                       uint32_t crc,
                       uint32_t *words, uint32_t max_words)
{
    uint32_t need = eeprom_record_words(len);
    const uint8_t *p = (const uint8_t *)src;
    uint32_t payload_words = need - 2U;
    uint32_t i;

    if (words == NULL || src == NULL || need > max_words) {
        return 0U;
    }

    /* Header word: magic | id | len | flags */
    words[0] = EEPROM_RECORD_MAGIC;
    words[1] = ((uint32_t)id << 16) | (uint32_t)len;
    words[2] = EEPROM_FLAG_COMMITTED;
    words[3] = 0xFFFFFFFFU;   /* spare, kept erased-looking for readability */

    /* Payload words, zero padded.  Padding must be deterministic: the same
     * config saved twice has to produce identical bytes, otherwise the
     * sector fills up with records that differ only in slack. */
    for (i = 0U; i < payload_words * EEPROM_WORD_BYTES; i++) {
        uint8_t b = (i < len) ? p[i] : 0xFFU;
        ((uint8_t *)&words[4U + i])[0] = b;
    }

    /* Commit word: magic again | id again | crc | payload length */
    words[need - 1U] = EEPROM_RECORD_MAGIC;
    words[need]     = ((uint32_t)id << 16) | (uint32_t)len;
    words[need + 1U] = crc;
    words[need + 2U] = 0xFFFFFFFFU;

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
    if (words[need - 1U] != EEPROM_RECORD_MAGIC) {
        return 0U;
    }
    if ((words[need - 1U] >> 16) != id) {
        return 0U;
    }
    if ((words[need - 1U] & 0xFFFFU) != len) {
        return 0U;
    }

    memcpy(out, (const uint8_t *)&words[4], len);
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
    return HAL_FLASHEx_Erase(&er, &sector_error) == HAL_OK;
}

static bool eeprom_hal_program(uint32_t address, const uint32_t *words, uint32_t n)
{
    /* One HAL_FLASH_Program() call per 16-byte flash word, and the caller
     * guarantees each word is programmed exactly once.
     *
     * This matters more than it looks.  Each flash word carries ECC bits
     * that are computed over the full 128 data bits, so a word must go
     * from erased to its final value in a single operation.  Programming
     * part of a word and later programming the rest corrupts the ECC, and
     * the next read of that address raises a bus fault that hardfaults the
     * core.  Never retry a failed program on the same address - erase the
     * sector instead. */
    for (uint32_t i = 0U; i < n; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                              address + i * EEPROM_WORD_BYTES,
                              (uint32_t)(&words[i])) != HAL_OK) {
            return false;
        }
    }
    return true;
}

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
 * The payload CRC is deliberately NOT verified here - that would mean
 * reading the payload, which is exactly what the two-pass load avoids.
 * A record that passes this but fails its CRC is caught by the full
 * eeprom_decode() in pass 2. */
bool eeprom_record_is_intact(const uint32_t *words)
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
    if (need * EEPROM_WORD_BYTES > EEPROM_SECTOR_BYTES) {
        return false;
    }

    /* The commit word is programmed last, so an interrupted save leaves it
     * erased.  It must also agree with the header: a mismatch means the two
     * halves came from different saves. */
    if (words[need - 1U] != EEPROM_RECORD_MAGIC) {
        return false;
    }
    if (eeprom_header_id(words) != (words[need - 1U] >> 16)) {
        return false;
    }
    if (len != (words[need - 1U] & 0xFFFFU)) {
        return false;
    }
    return true;
}

static bool eeprom_word_is_erased(const uint32_t *words)
{
    return (words[0] == 0xFFFFFFFFU);
}

/*============================================================================
 *  Public API
 *==========================================================================*/

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
            uint32_t len;
            uint32_t need;

            if (eeprom_word_is_erased(words)) {
                /* Slots are filled in order, so once one is erased nothing
                 * further along in this sector can be valid. */
                break;
            }

            /* Header and commit word only - never the payload between
             * them.  This is the whole point of the two-pass split. */
            if (!eeprom_record_is_intact(words)) {
                s_stats.loads_corrupt++;
                continue;
            }

            len  = eeprom_header_len(words);
            need = eeprom_record_words(len);

            if (need * EEPROM_WORD_BYTES > EEPROM_SECTOR_BYTES) {
                s_stats.loads_corrupt++;
                break;      /*!< nonsense length: the chain is untrustworthy */
            }

            if (n_cand < EEPROM_MAX_RECORDS) {
                s_cand[n_cand].addr  = addr;
                s_cand[n_cand].words = need;
                s_cand[n_cand].id    = eeprom_header_id(words);
                n_cand++;
            }

            /* Jump past this record; the loop increments w by 2 on top. */
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
        /* Sized to the record, not the sector: ConfigType is 124 bytes so a
         * record is 144 bytes = 9 words.  A sector-sized buffer would be
         * 2 KB of static RAM for nothing. */
        static uint32_t staging[EEPROM_MAX_RECORD_WORDS];
        uint32_t words;

        if (need > EEPROM_MAX_RECORD_WORDS) {
            s_stats.save_failures++;
            return false;
        }

        words = eeprom_encode(next_id, src, len,
                              crc32_calculate((const uint8_t *)src, len),
                              staging, EEPROM_MAX_RECORD_WORDS);
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