/**
 * @file touch.h
 * @brief Touch Screen Driver Header (Resistive TCS2046 & Capacitive GT911)
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
 * (resistive); an external GT911 (capacitive) can be attached over the FPC.
 * Detection order is capacitive first, resistive as the fallback. */
typedef enum {
    TOUCH_NONE = 0,
    TOUCH_RESISTIVE,      /* NS2009, on board  */
    TOUCH_CAPACITIVE,     /* GT911, on FPC1    */
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
 * Function Prototypes
 *============================================================================*/

void touch_init(void);
TouchType touch_detect_type(void);
bool touch_get_position(TouchPointType *point);
void touch_process(void);
void touch_calibrate_start(void);
bool touch_calibrate_add_point(uint16_t screen_x, uint16_t screen_y);
bool touch_calibrate_complete(void);
bool touch_is_pressed(void);
void touch_apply_calibration(uint16_t raw_x, uint16_t raw_y,
                             uint16_t *cal_x, uint16_t *cal_y);

/* Capacitive touch specific functions */
void touch_capacitive_init(void);
bool touch_capacitive_read(uint16_t *x, uint16_t *y, bool *pressed);

void touch_int_isr(void);

#ifdef __cplusplus
}
#endif

#endif /* TOUCH_H */
