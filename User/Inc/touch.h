/**
 * @file touch.h
 * @brief Touch Screen Driver Header (NS2009 resistive & GT911 capacitive)
 *
 * @details 两个控制器都挂在 I2C2 上，靠 7 位地址区分（NS2009 = 0x48，
 *          GT911 = 0x14）。数据搬运走 DMA，传输完成中断里解码并发布
 *          volatile 样本，任务侧只读快照——见下面对 g_touch_raw_* 的说明。
 */

#ifndef TOUCH_H
#define TOUCH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "opa2000.h"

#ifndef LVGL_SIM
#include "main.h"

/*============================================================================
 * External Variables
 *============================================================================*/

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern I2C_HandleTypeDef hi2c3;
#endif

/*============================================================================
 * Touch Configuration
 *============================================================================*/

#define TOUCH_SPI_BUFFER_SIZE    8       /* TCS2046 response size */

/* Touch coordinates */
typedef struct {
    uint16_t x;
    uint16_t y;
    bool pressed;
} TouchPointType;

/* Which controller answered on the bus.  The board carries an NS2009
 * (resistive) on the PCB; the GT911 (capacitive) is the production panel
 * on the FPC connector.  Detection order is capacitive first. */
typedef enum {
    TOUCH_NONE = 0,
    TOUCH_RESISTIVE,      /* NS2009, on board   */
    TOUCH_CAPACITIVE,     /* GT911, on FPC1     */
} TouchType;

/* Calibration points (5-point) */
#define CAL_POINT_COUNT  5

typedef struct {
    uint16_t x;   /* Screen x position */
    uint16_t y;   /* Screen y position */
} CalPointType;

/*============================================================================
 * Global state
 *============================================================================*/

/* Which controller answered on the bus.  Set once by touch_init(). */
extern volatile TouchType g_touch_type;

/*============================================================================
 * ISR -> task hand-off: the raw sample
 *
 * touch.c drives I2C2 with DMA.  The transfer-complete interrupt decodes the
 * bytes and publishes them here; nothing in that interrupt touches an LVGL
 * widget or a FreeRTOS object.  The GUI task reads the snapshot through
 * touch_get_position(), which also applies the resistive calibration
 * transform - deliberately kept on the task side because it is float maths
 * and has no business running in an interrupt.
 *
 * Written only by the I2C2 DMA callback (single writer, always ISR context).
 * g_touch_raw_seq is a seqlock counter: it is incremented once after x / y /
 * pressed have all landed, so a reader that sees the same value before and
 * after its three loads knows it got a consistent triple.  Without that, a
 * reader could catch "new coordinates, stale pressed bit" on a press edge,
 * which is exactly where a phantom tap would come from.
 *
 * These are RAW panel values (NS2009: 0..4095; GT911: panel pixels).
 *============================================================================*/

extern volatile uint16_t g_touch_raw_x;
extern volatile uint16_t g_touch_raw_y;
extern volatile uint8_t  g_touch_raw_pressed;   /*!< 1 = pressed, 0 = released */
extern volatile uint8_t  g_touch_raw_seq;       /*!< bumped per completed sample */
/*!< Cumulative bus errors / aborted transactions.  Diagnostic only. */
extern volatile uint32_t g_touch_bus_faults;

/*============================================================================
 * Function Prototypes
 *============================================================================*/

void touch_init(void);
TouchType touch_detect_type(void);

/*! Latest sample, already calibrated for the resistive panel.
 *  Non-blocking: reads the ISR-published snapshot only, never touches I2C. */
bool touch_get_position(TouchPointType *point);

/*! Advance the DMA state machine by one step and police the watchdog.
 *  Call from the GUI task tick (~10 ms).  Kicks off a new poll when the
 *  previous one has settled and enough time has passed; never blocks. */
void touch_process(void);

void touch_calibrate_start(void);
bool touch_calibrate_add_point(uint16_t screen_x, uint16_t screen_y);
bool touch_calibrate_complete(void);
bool touch_is_pressed(void);
void touch_apply_calibration(uint16_t raw_x, uint16_t raw_y,
                             uint16_t *cal_x, uint16_t *cal_y);

/* Capacitive touch specific functions */
void touch_capacitive_init(void);

/*! Snapshot accessor kept for API compatibility.  It no longer performs an
 *  I2C read - it returns whatever the DMA callback last published, so it can
 *  never block.  Prefer touch_get_position() in new code. */
bool touch_capacitive_read(uint16_t *x, uint16_t *y, bool *pressed);

void touch_int_isr(void);

#ifdef __cplusplus
}
#endif

#endif /* TOUCH_H */
