/**
 * @file    ns2009.c
 * @brief   NS2009 I2C resistive touch driver (single-threaded).
 *
 * @details Calls into the project's I2C2 handle `hi2c2`.  The driver
 *          is designed to be used during the single-threaded boot
 *          window (touch_init) and from the GUI task; it does not
 *          take any mutex.  When called from a multi-task context
 *          (touch_process on the GUI task), the caller is
 *          responsible for serialising - in practice the I2C
 *          transaction itself is brief and the GUI task is the only
 *          one polling the touch, so no lock is needed.
 */

#include "ns2009.h"

#include "stm32h7xx_hal.h"

/* Provided by CubeMX in the firmware build; the host test never
 * links this file (the touch driver is exercised only on target),
 * so a forward declaration is enough. */
extern I2C_HandleTypeDef hi2c2;

/*============================================================================
 *  Probe
 *============================================================================*/
bool ns2009_probe(void)
{
    /* A single register read of zero bytes is a no-op that still
     * performs the address ACK.  If the chip is missing, HAL_I2C_Master_Receive
     * returns HAL_ERROR (HAL_BUSY/HAL_TIMEOUT), the I2C bus stays
     * clean.  We then issue a 1-byte read at REG_XH which is a guaranteed
     * "registers exist" probe. */
    uint8_t scratch = 0U;
    HAL_StatusTypeDef rc = HAL_I2C_Master_Receive(
        &hi2c2,
        NS2009_ADDR,
        &scratch,
        1U,
        NS2009_I2C_TIMEOUT_MS);
    return (rc == HAL_OK);
}

/*============================================================================
 *  Read
 *============================================================================*/
bool ns2009_read(uint16_t *x, uint16_t *y)
{
    uint8_t xh_xl[2] = {0U, 0U};
    uint8_t yh_yl[2] = {0U, 0U};

    /* Two short I2C reads, each starting at the high byte.  Repeated
     * START between the two reads is implicit because we re-issue the
     * transfer instead of holding the bus. */
    HAL_StatusTypeDef rc = HAL_I2C_Mem_Read(
        &hi2c2,
        NS2009_ADDR,
        NS2009_REG_XH,
        I2C_MEMADD_SIZE_8BIT,
        xh_xl,
        2U,
        NS2009_I2C_TIMEOUT_MS);
    if (rc != HAL_OK) {
        return false;
    }

    rc = HAL_I2C_Mem_Read(
        &hi2c2,
        NS2009_ADDR,
        NS2009_REG_YH,
        I2C_MEMADD_SIZE_8BIT,
        yh_yl,
        2U,
        NS2009_I2C_TIMEOUT_MS);
    if (rc != HAL_OK) {
        return false;
    }

    uint16_t raw_x = NS2009_COORD_FROM_HL(xh_xl[0], xh_xl[1]);
    uint16_t raw_y = NS2009_COORD_FROM_HL(yh_yl[0], yh_yl[1]);

    /* No-touch samples return ~0 in both axes.  Reject the obviously
     * invalid range and any signal that does not exceed the press
     * threshold on at least one axis. */
    if (raw_x < NS2009_PRESS_THRESHOLD && raw_y < NS2009_PRESS_THRESHOLD) {
        return false;
    }

    *x = raw_x;
    *y = raw_y;
    return true;
}
