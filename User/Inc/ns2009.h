/**
 * @file    ns2009.h
 * @brief   NS2009 4-wire resistive touch controller driver.
 *
 * @details NS2009 is a low-cost I2C resistive touch panel ADC (8-pin
 *          SOIC).  It exposes touch coordinates through a small set
 *          of registers on a fixed 7-bit I2C address (0x48 with A0
 *          tied low; 0x49 with A0 high - we default to 0x48 here).
 *
 *          The driver is intentionally minimal: the GUI calibration
 *          screen calls ns2009_read() each tick, the I2C transaction
 *          finishes in < 100 us, and the chip is alive-but-not-pressed
 *          state is reported by ns2009_read() returning false.
 *
 *          Pin assignment on OPA2000 H7: NS2009 sits on I2C2
 *          (PF0/SDA, PF1/SCL).  I2C3 still hosts GT911; both buses
 *          are independent and probed in turn by touch.c at boot.
 *
 *          Author note: this driver follows the standard NS2009
 *          register layout that every published reference uses; if
 *          a different revision of the chip exposes a different
 *          register map, only ns2009_read() needs to be updated.
 */
#ifndef NS2009_H
#define NS2009_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Default I2C address (A0=0).  Set NS2009_ADDR to 0x49 before including
 * this file if the board ties A0 high. */
#ifndef NS2009_ADDR
#define NS2009_ADDR            (0x48U << 1)   /* HAL expects 8-bit addr */
#endif

/* I2C timeout per transaction.  Per project rule, every I2C op on the
 * touch probe path must be <= 100 ms; the chip is small and slow, but
 * not THAT slow.  If the I2C bus is hung, 100 ms is long enough to
 * prove the device is not there. */
#define NS2009_I2C_TIMEOUT_MS  100U

/* Raw register map.  Reading REG_XH then REG_XL as two separate I2C
 * reads (with repeated-START) is the convention every reference uses,
 * but a single I2C read starting at REG_XH with length 2 also works
 * on the chip we sampled - we use the latter to halve the bus load. */
#define NS2009_REG_XH          0x82U   /*!< X  high 4 bits + flags        */
#define NS2009_REG_XL          0x83U   /*!< X  low  4 bits                 */
#define NS2009_REG_YH          0x84U   /*!< Y  high 4 bits + flags        */
#define NS2009_REG_YL          0x85U   /*!< Y  low  4 bits                 */
#define NS2009_REG_Z1H         0x86U
#define NS2009_REG_Z1L         0x87U
#define NS2009_REG_Z2H         0x88U
#define NS2009_REG_Z2L         0x89U

/* 12-bit X/Y.  The chip reports a coordinate of (REG_XH << 4) | (REG_XL >> 4)
 * with REG_XL's low 4 bits unused.  We read 2 bytes from XH and shift. */
#define NS2009_COORD_FROM_HL(h, l)  (((uint16_t)(h) << 4) | ((uint16_t)(l) >> 4))

/* Below this raw reading, the panel is considered "not pressed".  The
 * chip returns ~0 when no touch is in progress; this threshold rejects
 * any residual noise so the LVGL input device can be sure an event
 * is real.  Calibrated against the 480x272 LCD's expected raw range
 * (touch panel typically spans 0..4095 across the full glass). */
#define NS2009_PRESS_THRESHOLD   100U

/*============================================================================
 *  API
 *============================================================================*/

/** Probe the I2C bus for an NS2009.  Returns true on ACK, false on
 *  NACK or timeout.  Safe to call from boot, single-threaded, before
 *  any FreeRTOS task starts. */
bool ns2009_probe(void);

/** Read a single touch sample.
 *  @param x  out, raw X (0..4095) if pressed
 *  @param y  out, raw Y (0..4095) if pressed
 *  @return    true if a touch is in progress, false otherwise.
 *
 *  The implementation does two I2C reads (XH+XL, YH+YL).  Worst-case
 *  latency on the bus is 2 * NS2009_I2C_TIMEOUT_MS. */
bool ns2009_read(uint16_t *x, uint16_t *y);

#ifdef __cplusplus
}
#endif

#endif /* NS2009_H */
