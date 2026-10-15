/**
 * @file    flash_eeprom.h
 * @brief   Configuration storage on the STM32H7B0's internal flash.
 *
 * @details The H7B0 has 128 KB of flash in 8 KB sectors, and the array is
 *          organised as 137-bit words (128 data bits + 9 ECC bits).  Two
 *          consequences drive the whole design:
 *
 *            - The programmable granularity is 16 bytes and the ECC cannot
 *              be switched off.  A word can only be written once, from all
 *              ones to the target value; a second write to the same address
 *              needs an erase first.  There is no read-modify-write.
 *            - Erasing costs a whole 8 KB sector.
 *
 *          So the last four sectors (SECTOR_12..15, 32 KB) hold a ring of
 *          append-only records.  A save appends one record; it never
 *          rewrites what is already there.  When the active sector fills
 *          up, the sector with the lowest id is erased and reused, which
 *          spreads wear evenly across all four.
 *
 *          On load, every slot of every sector is scanned, each candidate
 *          is validated (magic, length, payload CRC), and the record with
 *          the highest id wins.  A record that was interrupted by a reset
 *          or a power cut simply fails validation and is ignored, so a
 *          save can never destroy the previous good copy.
 *
 *          Layout, per 16-byte flash word:
 *
 *              word 0 : magic | id | len | flags        (header)
 *              word 1..N : payload (ConfigType, zero padded)
 *              last word : header duplicate + crc32   (commit marker)
 *
 *          The commit marker is written LAST.  Its presence is what makes
 *          a record "complete": a record whose payload is written but whose
 *          commit word is still erased was interrupted, and is discarded.
 *
 * @warning Saving blocks the CPU for the duration of the erase and program
 *          operations, because the core cannot fetch instructions from a
 *          busy flash.  Never call this from an interrupt, and never from
 *          a task with a tight deadline.  app_config_task() owns it.
 */

#ifndef FLASH_EEPROM_H
#define FLASH_EEPROM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 *  Geometry
 *============================================================================*/

/*!< H7B0 flash sector size: 8 KB. */
#define EEPROM_SECTOR_BYTES        8192U

/*!< Programmable granularity: 128 data bits + 9 ECC bits = 16 bytes. */
#define EEPROM_WORD_BYTES           16U

/*!< Words per sector. */
#define EEPROM_WORDS_PER_SECTOR    (EEPROM_SECTOR_BYTES / EEPROM_WORD_BYTES)

/*!< Four sectors give both endurance and history depth: a sector is erased
 *   only once all four are full. */
#define EEPROM_SECTOR_COUNT         4U

/*!< Index of the first sector used.  SECTOR_12..15 are the last four of the
 *   sixteen in the 128 KB part; the linker script shrinks FLASH to 96 KB so
 *   nothing the compiler emits can land on top of them. */
#define EEPROM_FIRST_SECTOR        12U

/*!< Largest payload this layer will store.  ConfigType is ~128 bytes,
 *   which needs 10 words including header and commit; the cap leaves
 *   room for the structure to grow without silently overflowing the
 *   staging buffer.  Kept in sync with the layout test in
 *   User/Tests/test_config_layout.c. */
#define EEPROM_MAX_PAYLOAD_BYTES   256U

/*!< Header + padded payload + commit, for EEPROM_MAX_PAYLOAD_BYTES. */
#define EEPROM_MAX_RECORD_WORDS \
    (1U + ((EEPROM_MAX_PAYLOAD_BYTES + EEPROM_WORD_BYTES - 1U) / EEPROM_WORD_BYTES) + 1U)

/*!< Highest number of records the ring can hold.  eeprom_load() uses this to
 *   size its candidate table, so it must not be raised past what the sectors
 *   can actually store. */
#define EEPROM_MAX_RECORDS \
    ((EEPROM_SECTOR_COUNT * EEPROM_WORDS_PER_SECTOR) / EEPROM_MAX_RECORD_WORDS)

/*============================================================================
 *  Record header (word 0)
 *============================================================================*/
#define EEPROM_RECORD_MAGIC         0x5041434FU   /*!< "OCAP" little-endian */
#define EEPROM_FLAG_COMMITTED       0x0001U

/*============================================================================
 *  API
 *============================================================================*/

/**
 * @brief Read the newest valid record into \p out
 * @return true if a valid record was found and \p out was filled;
 *         false if the store is empty or every candidate failed validation,
 *         in which case \p out is untouched.
 */
bool eeprom_load(void *out, uint32_t out_size);

/**
 * @brief Append \p src as a new record with id = newest + 1
 *
 * Erases the sector with the oldest id if the active one is full.
 *
 * @note  BLOCKS THE CPU.  Task context only, never an ISR.
 */
bool eeprom_save(const void *src, uint32_t len);

/**
 * @brief Highest record id currently stored (0 if the store is empty)
 */
uint32_t eeprom_current_id(void);

/**
 * @brief Statistics, for diagnostics over the comm link
 */
typedef struct {
    uint32_t loads_ok;
    uint32_t loads_empty;
    uint32_t loads_corrupt;    /*!< candidates rejected by magic/len/CRC   */
    uint32_t saves_ok;
    uint32_t save_failures;
    uint32_t sector_erases;
    uint32_t saves_since_erase;
} EepromStatsType;

void eeprom_get_stats(EepromStatsType *out);
void eeprom_clear_stats(void);

/*============================================================================
 *  Record codec - exposed for the host tests
 *
 *  These are pure functions over a word array and touch no hardware, so
 *  User/Tests/test_flash_eeprom.c can exercise the encode/decode contract
 *  (including torn writes) without an STM32 or a flash driver.
 *==========================================================================*/

/**
 * @brief Words needed to store \p len payload bytes (header + payload +
 *        commit word), and how many records fit in one sector.
 */
uint32_t eeprom_record_words(uint32_t len);

/**
 * @brief Serialise \p src into \p words
 * @return word count written, or 0 if it would not fit in \p max_words
 */
uint32_t eeprom_encode(uint32_t id, const void *src, uint32_t len,
                       uint32_t crc, uint32_t *words, uint32_t max_words);

/**
 * @brief Validate a serialised record and copy its payload out
 * @return payload length, or 0 if the record is unusable
 *         (erased, missing commit word, header/commit disagreement, or a
 *         length that does not match the slot)
 */
uint32_t eeprom_decode(const uint32_t *words, uint32_t n_words,
                       void *out, uint32_t out_size);

/*==============================================================================
 *  Header-only accessors
 *
 *  These read a slot's header and commit word but never its payload.  eeprom_load()
 *  uses them to survey the ring cheaply before committing to reading any
 *  record in full.  Exposed for the host tests.
 *===========================================================================*/

/** @brief Record id from a slot's header, without reading the payload. */
uint32_t eeprom_header_id(const uint32_t *words);

/** @brief Payload length from a slot's header, without reading the payload. */
uint32_t eeprom_header_len(const uint32_t *words);

/**
 * @brief Check a slot's framing only: header magic, committed flag, and
 *        agreement between header and commit word.
 *
 * @note  Does NOT verify the payload CRC - that would mean reading the
 *        payload, which is what eeprom_load() is avoiding.  A record that
 *        passes here but fails its CRC is caught by eeprom_decode().
 *
 * @param words First word of the slot, i.e. the header
 */
bool eeprom_record_is_intact(const uint32_t *words);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_EEPROM_H */