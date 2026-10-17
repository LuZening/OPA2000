/**
 * @file    h7b0_flash_compat.h
 * @brief   Fills the FLASH_CR_PSIZE hole in the CMSIS device header.
 *
 * @details STM32H7B0's CMSIS header (stm32h7b0xx.h) does not define
 *          FLASH_CR_PSIZE, but the H7 HAL driver is written on the
 *          assumption that it exists:
 *
 *            - stm32h7xx_hal_flash_ex.h wraps its entire
 *              FLASH_VOLTAGE_RANGE_1..4 family in
 *              `#if defined (FLASH_CR_PSIZE)`, so on this part none of
 *              those macros come into existence.
 *            - stm32h7xx_hal_def.h includes Legacy/stm32_hal_legacy.h
 *              unconditionally, and that file references
 *              FLASH_VOLTAGE_RANGE_3 at line 485 with no guard at all.
 *
 *          The result is that any translation unit which pulls in the flash
 *          HAL fails to compile with
 *          "'FLASH_VOLTAGE_RANGE_3' undeclared", even though nothing in
 *          that file uses it.
 *
 *          This header defines the missing macros BEFORE the HAL is
 *          included, so the HAL compiles unmodified.  Nothing in the
 *          driver tree is touched.
 *
 *          Values are from RM0490 (STM32H7B0 reference manual), section
 *          "Flash control register", FLASH_CR.PSIZE:
 *
 *              00 - byte programming  (8 bits)
 *              01 - half-word         (16 bits)
 *              10 - word              (32 bits)
 *              11 - double word      (64 bits)
 *
 *          PSIZE is bits [9:8] of FLASH_CR.
 *
 * @warning This header MUST be included before any stm32h7xx_hal*.h.  In
 *          practice that means including it first in flash_eeprom.c, before
 *          "stm32h7xx_hal.h".
 */

#ifndef H7B0_FLASH_COMPAT_H
#define H7B0_FLASH_COMPAT_H

#include "stm32h7b0xx.h"

/* Only define what is actually missing, so if a future CMSIS package fixes
 * this the block collapses to nothing and the real definitions win. */
#if !defined(FLASH_CR_PSIZE)

/*!< PSIZE field: bits [9:8] of FLASH_CR. */
#define FLASH_CR_PSIZE_Pos      (8U)
#define FLASH_CR_PSIZE_Msk      (0x3UL << FLASH_CR_PSIZE_Pos)   /*!< 0x00000300 */
#define FLASH_CR_PSIZE          FLASH_CR_PSIZE_Msk

/*!< Individual PSIZE encodings, used by the HAL's RANGE macros below. */
#define FLASH_CR_PSIZE_0        (0x1UL << FLASH_CR_PSIZE_Pos)   /*!< 16-bit  */
#define FLASH_CR_PSIZE_1        (0x2UL << FLASH_CR_PSIZE_Pos)   /*!< 32-bit  */

/* The four names stm32h7xx_hal_flash_ex.h defines inside its own
 * `#if defined (FLASH_CR_PSIZE)` block.  Reproduced verbatim here, which is
 * what makes that block become active. */
#define FLASH_VOLTAGE_RANGE_1   0x00000000U                     /*!< 8-bit   */
#define FLASH_VOLTAGE_RANGE_2   FLASH_CR_PSIZE_0
#define FLASH_VOLTAGE_RANGE_3   FLASH_CR_PSIZE_1
#define FLASH_VOLTAGE_RANGE_4   FLASH_CR_PSIZE

#endif /* !defined(FLASH_CR_PSIZE) */

#endif /* H7B0_FLASH_COMPAT_H */