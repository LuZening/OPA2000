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

/*!< "有一个新样本待取走"标志，由 I2C 中断置位、由消费者清零。
 *
 *  存在的理由：LVGL 每 tick 调一次 indev 的 read_cb，而采样是 10 ms 一次，
 *  两个周期不同频，LVGL 必然连续多帧读到同一个样本。LVGL 自己会去重
 *  （坐标与状态都不变就不产生事件），所以功能上不会重复点击——但那依赖
 *  LVGL 内部实现。OPA1000 用同样的"取走即清"标志（main.c 的 TS_touched），
 *  不依赖下游的容忍度，这里照做。
 *
 *  与 g_touch_raw_seq 的分工：seq 是自增计数器，供 seqlock 校验快照一致性，
 *  读者**不会**清它；本标志是"取走即清"，消费者**必须**清，否则永远为真。
 *  两者不能互相替代。
 *
 *  一般直接用 touch_take_sample() 取，不要单独碰这个变量。 */
extern volatile uint8_t  g_touch_new_sample;

/*!< NS2009 最近一次算出的接触压力，范围 0..8190（|(4095-Z2)+Z1|）。
 *  pressed 就是拿它跟 NS2009_Z_PRESS_THRESHOLD（400）比出来的。
 *
 *  **这是唯一需要标定的数字。** 400 是从 LLMC_DAC 的 HR2046 路径沿用来的
 *  起步值（touch_HR2046.c: `*pZ = (ADC_Z > 400) ? 1 : 0`），而真实门限
 *  与面板电阻、驱动电流、玻璃厚度有关，每块屏都不一样。
 *
 *  上板后先读这个数：空屏时是什么基线、轻触时升到多少、重按时多少，
 *  然后按需调整那一个宏。若发现点击过于灵敏（空屏时也超阈值）或过于
 *  迟钝（要用力才响应），就是这一个数没定准。
 *
 *  只有电容屏（GT911）时它恒为 0。 */
extern volatile uint32_t g_touch_raw_z;

/*============================================================================
 * Diagnostics - 排查 I2C 问题用
 *============================================================================*/

/*!< 累计错误回调次数（含被中止的事务）。 */
extern volatile uint32_t g_touch_bus_faults;

/*!< 最近一次 HAL_I2C_ErrorCallback() 里 hi2c->ErrorCode 的原始值。
 *
 *  用它区分故障性质，对照 stm32h7xx_hal_i2c.h 的定义：
 *      0x00  NONE      正常
 *      0x01  BERR      总线错误：START/STOP 出现在错误的位置
 *      0x02  ARLO      仲裁丢失：总线上有第二个主机
 *      0x04  AF        ACK 失败（NACK）——**通常不是总线故障**，
 *                      而是从机没应答（器件没焊 / 地址不对 / 已复位）
 *      0x08  OVR       溢出：CPU/DMA 没及时取走数据
 *      0x10  DMA       DMA 传输错误
 *      0x20  TIMEOUT   超时
 *      0x40  SIZE      长度管理错误
 *      0x80  DMA_PARAM DMA 参数错误
 *  多个位可能同时置起。
 */
extern volatile uint32_t g_touch_last_error;

/*!< 总线恢复（SCL 打拍 + 重新挂载）累计执行次数。 */
extern volatile uint32_t g_touch_recoveries;

/*============================================================================
 * Function Prototypes
 *============================================================================*/

void touch_init(void);
TouchType touch_detect_type(void);

/*! Latest sample, already calibrated for the resistive panel.
 *  Non-blocking: reads the ISR-published snapshot only, never touches I2C.
 *
 *  注意：返回的是**当前状态**，不是"有新数据"。同一份样本会被读到多次
 *  ——LVGL 每 tick 读一次，采样是 10 ms 一次。要按 OPA1000 的方式只处理
 *  真正的新样本，用 touch_take_sample()。 */
bool touch_get_position(TouchPointType *point);

/*! 取走一个新样本：有新数据就填 \p point 并返回 true，并把
 *  "取走即清"标志清掉；没有新数据则返回 false 且**不碰** \p point。
 *
 *  这是 OPA1000 那个 TS_touched 标志的封装（main.c 的 lvGetTouchscreenXY）。
 *  适合"事件驱动"的消费者：LVGL 的 indev read_cb、标定点采集。
 *  只关心当前状态的地方（比如状态显示）用 touch_get_position() 就够。
 *
 *  @note 标志由 I2C 中断置位；清除它与中断置位可能相邻发生。这里读-清一次
 *        就收工，最多漏掉恰好落在这一瞬间的那个样本——下个周期会补上，
 *        对触摸来说一帧的延迟无感。不需要临界区。
 */
bool touch_take_sample(TouchPointType *point);

/*! Advance the DMA state machine and police the watchdog.
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
