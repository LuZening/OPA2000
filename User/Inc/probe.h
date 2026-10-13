/**
 * @file    probe.h
 * @brief   Lightweight logging / debug probe macros for OPA2000 firmware.
 *
 * @details Compiled to no-op when PROBE_ENABLE = 0.  When enabled,
 *          prints to ITM/SWO (default) or UART (PROBE_USE_UART=1).
 *
 *          Use `PROBE_LOGF(level, fmt, ...)` from any context;
 *          the macros handle the channel selection.
 *
 * @version 1.0
 * @date    2026-10
 * @author  Control Agent (wt-control branch)
 */

#ifndef PROBE_H
#define PROBE_H

#include <stdint.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===========================================================================
 *  1. Build-time switches
 * ===========================================================================*/
#ifndef PROBE_ENABLE
#define PROBE_ENABLE    0      /*!< 0 = no-op, 1 = enable PROBE_* macros */
#endif

#ifndef PROBE_USE_UART
#define PROBE_USE_UART  0      /*!< 0 = ITM/SWO, 1 = UART (LPUART1) */
#endif

/* ===========================================================================
 *  2. Levels
 * ===========================================================================*/
typedef enum {
    PROBE_LVL_ERROR = 0,
    PROBE_LVL_WARN  = 1,
    PROBE_LVL_INFO  = 2,
    PROBE_LVL_DEBUG = 3,
} probe_level_t;

/* ===========================================================================
 *  3. Public API (always declared, even if disabled)
 * ===========================================================================*/
void probe_init(void);
void probe_putc(char c);
int  probe_printf(probe_level_t lvl, const char *fmt, ...);

/* ===========================================================================
 *  4. Convenience macros
 * ===========================================================================*/
#if PROBE_ENABLE
  #define PROBE_LOGF(lvl, fmt, ...)  probe_printf((lvl), (fmt), ##__VA_ARGS__)
  #define PROBE_ERROR(fmt, ...)      PROBE_LOGF(PROBE_LVL_ERROR, "[ERR] " fmt "\r\n", ##__VA_ARGS__)
  #define PROBE_WARN(fmt, ...)       PROBE_LOGF(PROBE_LVL_WARN,  "[WRN] " fmt "\r\n", ##__VA_ARGS__)
  #define PROBE_INFO(fmt, ...)       PROBE_LOGF(PROBE_LVL_INFO,  "[INF] " fmt "\r\n", ##__VA_ARGS__)
  #define PROBE_DEBUG(fmt, ...)      PROBE_LOGF(PROBE_LVL_DEBUG, "[DBG] " fmt "\r\n", ##__VA_ARGS__)
#else
  #define PROBE_LOGF(lvl, fmt, ...)  do {} while (0)
  #define PROBE_ERROR(fmt, ...)      do {} while (0)
  #define PROBE_WARN(fmt, ...)       do {} while (0)
  #define PROBE_INFO(fmt, ...)       do {} while (0)
  #define PROBE_DEBUG(fmt, ...)      do {} while (0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* PROBE_H */