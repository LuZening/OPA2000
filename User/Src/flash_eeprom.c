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

#include "stm32h7xx_hal.h"
/*!< FLASH_VOLTAGE_RANGE_3 and FLASH_EraseInitTypeDef live here, not in
 *   stm32h7xx_hal_flash.h. */
#include "stm32h7xx_hal_flash_ex.h"

/*!< Absolute base ofEEPROM_FIRST_SECTOR.  128 KB part, 8 KB sectors. */
#define EEPROM_FLASH_BASE   (0x08000000UL + (EEPROM_FIRST_SECTOR * EEPROM_SECTOR_BYTES))

/*!< All error flags, cleared before every operation.  The H7 flash
 *   controller has separate flag sets per bank, and the bank field in
 *   FLASH_CCR is bank-specific on dual-bank parts, so both sets are
 *   cleared - a stale error from an interrupted earlier save would
 *   otherwise make every subsequent HAL_FLASH_* call return HAL_ERROR. */
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

    er.TypeErase    = FLASH_TYPEERASE_SECTORS;
    /* 32-bit parallel programming.  The HAL's FLASH_VOLTAGE_RANGE_* family is
     * only defined when the CMSIS header exposes FLASH_CR_PSIZE, which on
     * this part it does not; VOLTAGE_RANGE_3 is the same setting under the
     * name the CMSIS device header uses, and it is what
     * IS_FLASH_VOLTAGE_RANGE() checks. */
    er.VoltageRange = VOLTAGE_RANGE_3;
    er.Sector       = EEPROM_FIRST_SECTOR + sector_index;
#if defined(FLASH_BANK_1)
    er.Banks        = FLASH_BANK_1;
#else
    er.Banks        = 0U;   /*!< single bank: the HAL ignores the field */
#endif

    uint32_t sector_error = 0U;
    return HAL_FLASHEx_Erase(&er, &sector_error) == HAL_OK;
}

static bool eeprom_hal_program(uint32_t address, const uint32_t *words, uint32_t n)
{
    for (uint32_t i = 0U; i < n; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                              address + i * EEPROM_WORD_BYTES,
                              (uint32_t)(&words[i])) != HAL_OK) {
            return false;
        }
    }
    return true;
}

static bool eeprom_word_is_erased(uint32_t address)
{
    const uint32_t *p = (const uint32_t *)address;
    return (*p == 0xFFFFFFFFU);
}

/*============================================================================
 *  Public API
 *==========================================================================*/

bool eeprom_load(void *out, uint32_t out_size)
{
    uint32_t best_id = 0U;
    bool found = false;

    if (out == NULL) {
        return false;
    }

    /* Scan every slot of every sector.  Ids increase monotonically, so the
     * record carrying the highest valid id is the newest one no matter which
     * sector it happens to sit in. */
    for (uint32_t sec = 0U; sec < EEPROM_SECTOR_COUNT; sec++) {
        uint32_t base = eeprom_sector_base(sec);

        for (uint32_t w = 0U; w < EEPROM_WORDS_PER_SECTOR; w += 2U) {
            uint32_t addr = base + w * EEPROM_WORD_BYTES;
            const uint32_t *words = (const uint32_t *)addr;
            uint32_t len;
            uint32_t need;

            if (eeprom_word_is_erased(addr)) {
                /* Slots are filled in order, so once one is erased nothing
                 * further along in this sector can be valid. */
                break;
            }
            if (words[0] != EEPROM_RECORD_MAGIC) {
                s_stats.loads_corrupt++;
                continue;
            }

            /* The header carries the payload length, so there is no need to
             * guess: read it, then hand decode() the exact slot size. */
            len  = words[1] & 0xFFFFU;
            need = eeprom_record_words(len);

            if (need * EEPROM_WORD_BYTES > EEPROM_SECTOR_BYTES) {
                s_stats.loads_corrupt++;
                break;      /*!< nonsense length: the chain is untrustworthy */
            }

            if (eeprom_decode(words, need, out, out_size) == 0U) {
                s_stats.loads_corrupt++;
                /* Skip the rest of this record so a torn tail cannot be
                 * mistaken for the header of a following record. */
                w += (need - 1U) * 2U;
                continue;
            }

            {
                uint32_t id = words[1] >> 16;
                if (!found || id > best_id) {
                    best_id = id;
                    found = true;
                }
            }

            /* Jump past this record; the loop increments w by 2 on top. */
            w += (need - 1U) * 2U;
        }
    }

    if (found) {
        s_stats.loads_ok++;
    } else {
        s_stats.loads_empty++;
    }
    return found;
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
            if (eeprom_word_is_erased(addr)) {
                break;
            }
            if (words[0] != EEPROM_RECORD_MAGIC) {
                continue;
            }
            uint32_t id = words[1] >> 16;
            if (!found || id > best) {
                best = id;
                found = true;
            }
            /* Step to the next record rather than the next pair of words:
             * the length lives in the header, so this cannot land inside
             * the record we just read. */
            w += (eeprom_record_words(words[1] & 0xFFFFU) - 1U) * 2U;
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
            if (eeprom_word_is_erased(base + w * EEPROM_WORD_BYTES)) {
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
                if (eeprom_word_is_erased(addr)) {
                    break;
                }
                const uint32_t *words = (const uint32_t *)addr;
                if (words[0] != EEPROM_RECORD_MAGIC) {
                    continue;
                }
                uint32_t id = words[1] >> 16;
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