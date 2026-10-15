/**
 * @file touch.c
 * @brief Touch Screen Driver Implementation (I2C2 + DMA, 非阻塞)
 *
 * @details 两个控制器都挂在同一条 I2C2（PF0/SDA、PF1/SCL）上，靠 7 位地址区分：
 *
 *   1. NS2009  RTOUCH  电阻  7 位 0b1001000 = 0x48
 *   2. GT911   CTOUCH  电容  7 位 0b0010100 = 0x14
 *
 *          电容优先探测：GT911 是量产屏，NS2009 是替代料。探测发生在
 *          main() 里、osKernelStart() 之前，单线程，所以那里用阻塞式 HAL
 *          调用是安全的，也只有在那个阶段才可以。
 *
 *          ==================== 运行期为什么不阻塞 ====================
 *
 *          原来 touch_process() 每 10 ms 同步读一次总线，芯片不应答时
 *          每次最多自旋 100 ms（HAL 的等待循环是纯轮询，不让出 CPU），
 *          屏幕刷新率会被拖到几 Hz。改成 DMA 之后：
 *
 *            - 发起传输后立刻返回，数据由 DMA 搬进不可缓存缓冲；
 *            - 传输完成中断里解码，写 volatile 快照（见 touch.h）；
 *            - 任务侧只读快照，永不碰总线。
 *
 *          ==================== 完成回调由谁触发 ====================
 *
 *          I2C 的 DMA 是"两段接力"，两个中断都参与，缺一不可：
 *
 *            DMA 流中断（DMA2_Stream6/7）搬完数据后关掉 RXDMAEN，并打开
 *              STOP 中断；
 *            然后 I2C 事件中断（I2C2_EV）收到 STOPF，才调用
 *              HAL_I2C_MemRxCpltCallback()。
 *
 *          所以 I2C2_EV / I2C2_ER 必须使能——见 OPA2000_h7.ioc 的
 *          NVIC.I2C2_EV_IRQn / NVIC.I2C2_ER_IRQn（本文件不定义中断处理
 *          函数，交给 CubeMX 生成的 stm32h7xx_it.c，避免重复定义）。
 *
 *          ==================== 两个必须守住的约束 ====================
 *
 *          1. 缓冲必须放 .dma_buffer（AXI SRAM 非缓存窗口）。DMA1/DMA2
 *             访问不到 DTCM，且缓存窗口需要手动 cache maintenance，本项目
 *             的 MPU region4 已把它标为不可缓存。
 *
 *          2. DMA 版 HAL 函数没有超时参数——总线挂死时传输永不完成，
 *             回调一次都不来。所以必须有软件看门狗：touch_process()
 *             用 HAL_GetTick() 比对，超时则 HAL_I2C_Master_Abort_IT()
 *             中止并复位状态机。
 */

#include "touch.h"
#include "config.h"
#include "stm32h7xx_hal.h"
#include "main.h"
#include "ns2009.h"

/* CubeMX 在 Core/Src/i2c.c 里定义；这里前向声明就够（ns2009.c 同样做法）。 */
extern I2C_HandleTypeDef hi2c2;

/*============================================================================
 * 常量
 *============================================================================*/

#define SCREEN_WIDTH    480
#define SCREEN_HEIGHT   272

/* GT911 (CTOUCH) 7 位地址 0b0010100 = 0x14，HAL 要 8 位地址（R/W 落在 bit0）。 */
#define GT911_I2C_ADDRESS     (0x14U << 1)   /* 0x28 */

#define GT911_REG_STATUS      0x814EU        /* 状态 / 点数          */
#define GT911_REG_POINT       0x8150U        /* 第一个触摸点数据      */

#define GT911_STATUS_BYTES    1U
#define GT911_POINT_BYTES     7U

/* NS2009 的 XH,XL,YH,YL 是连续的（0x82..0x85），但仍分两笔读：
 * 芯片每次读触发一次坐标转换，一次读 4 字节拿不到两次独立转换的结果。 */
#define NS_RX_BYTES           4U

/* 每笔事务的软件看门狗上限。400 kHz 下一笔 8 字节事务约 200 us，
 * 10 ms 已是 50 倍余量；再长只会让总线挂死时更晚被发现。 */
#define TOUCH_I2C_TIMEOUT_MS  10U

/* 轮询周期：一轮样本落定后才发起下一轮，所以这是"最快"周期。 */
#define TOUCH_POLL_PERIOD_MS  10U

/*============================================================================
 * DMA 缓冲
 *
 * .dma_buffer 落在 AXI SRAM 的不可缓存窗口（app.ld 的 .dma_no_cache，
 * 由 MPU region4 标为 NOT_CACHEABLE），DMA2 够得到，且无需手动 cache
 * maintenance。注意该段是 NOLOAD——不被 startup 赋初值或清零，所以
 * 发送缓冲必须在每次启动前显式赋值。
 *==========================================================================*/
#if defined(__GNUC__)
#define TOUCH_DMA_BUF  __attribute__((section(".dma_buffer"), aligned(32)))
#else
#define TOUCH_DMA_BUF
#endif

/*! NS2009 坐标载荷：[0..1] = XH,XL  [2..3] = YH,YL */
static uint8_t s_ns_rx[NS_RX_BYTES]      TOUCH_DMA_BUF;
/*! GT911 状态寄存器（1 字节） */
static uint8_t s_gt_status[GT911_STATUS_BYTES] TOUCH_DMA_BUF;
/*! GT911 第一个触摸点（7 字节） */
static uint8_t s_gt_point[GT911_POINT_BYTES]   TOUCH_DMA_BUF;
/*! GT911 清状态：往 0x814E 写 1 个 0。NOLOAD 段，每次用前显式赋值。 */
static uint8_t s_gt_clear_tx[1]                TOUCH_DMA_BUF;

/*============================================================================
 * ISR -> 任务 交接的 volatile 快照
 *==========================================================================*/

volatile uint16_t g_touch_raw_x       = 0U;
volatile uint16_t g_touch_raw_y       = 0U;
volatile uint8_t  g_touch_raw_pressed = 0U;
volatile uint8_t  g_touch_raw_seq     = 0U;
volatile uint32_t g_touch_bus_faults  = 0U;

/*============================================================================
 * 私有状态
 *==========================================================================*/

/* Current touch type */
static TouchType detected_touch_type = TOUCH_NONE;

/* True when the resistive panel is an NS2009 on I2C2.  Only one resistive
 * controller is supported now - the legacy XPT2046/TSC2046 SPI path was
 * dropped, so the resistive backend is always NS2009. */
static bool s_resistive_is_ns2009 = false;

/* Public view of the detected controller, so the GUI can skip drawing
 * capacitive-only affordances.  touch_init() is the only writer. */
volatile TouchType g_touch_type = TOUCH_NONE;

/* Interrupt flag for capacitive touch (GT911 INT pin, currently unused) */
static volatile uint8_t touch_interrupt_flag = 0;

/* Calibration data */
static bool calibrating = false;
static uint8_t cal_point_index = 0;
static struct {
    uint16_t raw_x;
    uint16_t raw_y;
    uint16_t screen_x;
    uint16_t screen_y;
} cal_data[CAL_POINT_COUNT];

/*============================================================================
 * DMA 状态机
 *
 * 一笔事务只能有一个在途。stage 既是"当前等的是什么"，也是回调里的
 * 分派依据。所有 stage 变量都在中断里推进，任务侧只在 IDLE 时发起新
 * 一轮，所以两边不会同时写同一笔事务。
 *==========================================================================*/

typedef enum {
    TS_IDLE = 0,     /*!< 没有在途事务                       */
    TS_NS_X,         /*!< 等 NS2009 X 读                     */
    TS_NS_Y,         /*!< 等 NS2009 Y 读                     */
    TS_GT_STATUS,    /*!< 等 GT911 状态读                     */
    TS_GT_POINT,     /*!< 等 GT911 触摸点读                   */
    TS_GT_CLEAR,     /*!< 等 GT911 清状态写                   */
} TouchStage;

static volatile TouchStage s_stage        = TS_IDLE;
static volatile uint32_t   s_stage_tick   = 0U;   /*!< 本阶段起始时刻          */
static volatile uint16_t   s_cur_addr     = 0U;   /*!< 在途事务的从机地址      */
static volatile bool       s_abort_pending = false;

/* 任务上下文专用，无需 volatile。 */
static uint32_t s_last_poll_tick = 0U;

/*============================================================================
 * 发布与解码（全部在中断上下文执行，不得调用任何 FreeRTOS API）
 *==========================================================================*/

/**
 * @brief 把一个完整样本发布到 volatile 快照。
 *
 * 写入顺序是有意为之：先落"拾起/按下"的坐标，再落 pressed 位；释放时
 * 反之。配合 g_touch_raw_seq 的 seqlock，读者绝不会把"新坐标"配上
 * "旧的按下状态"——那正是幻影点击的来源。
 */
static void touch_publish(bool pressed, uint16_t x, uint16_t y)
{
    if (pressed) {
        g_touch_raw_x = x;
        g_touch_raw_y = y;
        g_touch_raw_pressed = 1U;
    } else {
        g_touch_raw_pressed = 0U;
        g_touch_raw_x = 0U;
        g_touch_raw_y = 0U;
    }
    /* 三个字段都落定之后才自增：读者据此判断快照是否一致。 */
    g_touch_raw_seq = (uint8_t)(g_touch_raw_seq + 1U);
}

/**
 * @brief 发起一笔 NS2009 读。返回 false 表示没能启动，状态机已回 IDLE。
 * @note 可由任务上下文（首轮）或中断上下文（串联下一拍）调用。
 */
static bool touch_start_ns_read(uint16_t reg, uint8_t *dst, uint16_t len,
                                TouchStage st)
{
    s_cur_addr   = NS2009_ADDR;
    s_stage      = st;
    s_stage_tick = HAL_GetTick();

    if (HAL_I2C_Mem_Read_DMA(&hi2c2, NS2009_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             dst, len) != HAL_OK) {
        /* HAL 在启动前就拒绝了（句柄忙 / 参数非法）：没有回调会来，
         * 必须自己收回状态，否则状态机会永远卡在这一拍。 */
        s_stage = TS_IDLE;
        return false;
    }
    return true;
}

/** @brief 发起 GT911 状态寄存器读（1 字节 @ 0x814E）。 */
static bool touch_start_gt_status(void)
{
    s_cur_addr   = GT911_I2C_ADDRESS;
    s_stage      = TS_GT_STATUS;
    s_stage_tick = HAL_GetTick();

    if (HAL_I2C_Mem_Read_DMA(&hi2c2, GT911_I2C_ADDRESS, GT911_REG_STATUS,
                             I2C_MEMADD_SIZE_16BIT, s_gt_status,
                             GT911_STATUS_BYTES) != HAL_OK) {
        s_stage = TS_IDLE;
        return false;
    }
    return true;
}

/** @brief 发起 GT911 第一个触摸点读（7 字节 @ 0x8150）。 */
static bool touch_start_gt_point(void)
{
    s_cur_addr   = GT911_I2C_ADDRESS;
    s_stage      = TS_GT_POINT;
    s_stage_tick = HAL_GetTick();

    if (HAL_I2C_Mem_Read_DMA(&hi2c2, GT911_I2C_ADDRESS, GT911_REG_POINT,
                             I2C_MEMADD_SIZE_16BIT, s_gt_point,
                             GT911_POINT_BYTES) != HAL_OK) {
        s_stage = TS_IDLE;
        return false;
    }
    return true;
}

/**
 * @brief 发起 GT911 清状态写（往 0x814E 写 0）。
 *
 * GT911 读完点数据后必须清状态，否则下一帧读到的还是旧数据。
 * 原实现不分按下与否都会清，这里保持同样语义。
 */
static bool touch_start_gt_clear(void)
{
    s_cur_addr   = GT911_I2C_ADDRESS;
    s_stage      = TS_GT_CLEAR;
    s_stage_tick = HAL_GetTick();

    /* NOLOAD 段不清零，内容不可假设——每次显式赋值。 */
    s_gt_clear_tx[0] = 0U;

    if (HAL_I2C_Mem_Write_DMA(&hi2c2, GT911_I2C_ADDRESS, GT911_REG_STATUS,
                              I2C_MEMADD_SIZE_16BIT, s_gt_clear_tx, 1U)
        != HAL_OK) {
        s_stage = TS_IDLE;
        return false;
    }
    return true;
}

/*============================================================================
 * HAL 完成回调（中断上下文）
 *
 * 这些是 HAL 里的 __weak 定义，在这里给出强定义即可，与 CubeMX 生成
 * 的代码不冲突（CubeMX 不生成 HAL 回调）。回调触发时 hi2c 的 State 已经
 * 回到 READY、锁已释放，所以在这里直接串联下一笔 DMA 是安全的。
 *==========================================================================*/

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C2) {
        return;
    }

    switch (s_stage) {

    case TS_NS_X:
        /* X 到手，接着读 Y。两笔分开是因为每次读都触发一次坐标转换。 */
        (void)touch_start_ns_read(NS2009_REG_YH, &s_ns_rx[2], 2U, TS_NS_Y);
        break;

    case TS_NS_Y: {
        uint16_t rx = NS2009_COORD_FROM_HL(s_ns_rx[0], s_ns_rx[1]);
        uint16_t ry = NS2009_COORD_FROM_HL(s_ns_rx[2], s_ns_rx[3]);
        /* 无触摸时两轴都接近 0，两轴都低于阈值才算"未按下"（与原逻辑一致）。 */
        bool pressed = !((rx < NS2009_PRESS_THRESHOLD) &&
                         (ry < NS2009_PRESS_THRESHOLD));
        touch_publish(pressed, rx, ry);
        s_stage = TS_IDLE;
        break;
    }

    case TS_GT_STATUS:
        if ((s_gt_status[0] & 0x80U) == 0U) {
            /* 未按下：照原逻辑仍要清状态，然后收工。 */
            touch_publish(false, 0U, 0U);
            (void)touch_start_gt_clear();
        } else {
            (void)touch_start_gt_point();
        }
        break;

    case TS_GT_POINT: {
        uint16_t x = ((uint16_t)s_gt_point[1] << 8) | (uint16_t)s_gt_point[2];
        uint16_t y = ((uint16_t)s_gt_point[3] << 8) | (uint16_t)s_gt_point[4];
        /* 先发布，再清状态：样本早一拍可用，清状态只是收尾。 */
        touch_publish(true, x, y);
        (void)touch_start_gt_clear();
        break;
    }

    default:
        /* TS_IDLE / TS_GT_CLEAR 不该收到 Rx 完成（后者是写事务）。 */
        break;
    }
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C2) {
        return;
    }
    if (s_stage == TS_GT_CLEAR) {
        s_stage = TS_IDLE;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C2) {
        return;
    }

    /* 失败安全：宁可报"未按下"也不留下幻影按压。这与原来的行为一致
     * （旧代码在读失败时也是把 pressed 置 false）。代价是总线抖动会让
     * 一次按压中断一拍，对控制面板来说这个方向是正确的。 */
    touch_publish(false, 0U, 0U);

    s_abort_pending = false;
    s_stage         = TS_IDLE;
    g_touch_bus_faults++;
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C2) {
        return;
    }
    /* 看门狗发起的中止走这里收尾（部分 HAL 路径走 ErrorCallback，
     * 两个都复位，谁先到都不影响）。 */
    s_abort_pending = false;
    s_stage         = TS_IDLE;
}

/*============================================================================
 * Public Functions
 *============================================================================*/

/**
 * @brief Initialize touch screen
 */
void touch_init(void)
{
    detected_touch_type = touch_detect_type();
    g_touch_type = detected_touch_type;

    if (detected_touch_type == TOUCH_RESISTIVE) {
        s_resistive_is_ns2009 = true;
        /* NS2009 上电后无需配置，探测成功即代表就绪。 */
    } else if (detected_touch_type == TOUCH_CAPACITIVE) {
        s_resistive_is_ns2009 = false;
        touch_capacitive_init();
    }

    s_last_poll_tick = HAL_GetTick();
}

/**
 * @brief Detect touch screen type
 *
 * Both controllers hang off I2C2 and are distinguished purely by address:
 *
 *   1. GT911 on I2C2, 7-bit 0x14 (capacitive, production panel).
 *   2. NS2009 on I2C2, 7-bit 0x48 (resistive, RTOUCH).
 *
 * 这个函数只在 main() 的单线程启动阶段调用，那时还没有 DMA 状态机，
 * 用阻塞式 HAL 读反而更简单可靠，所以这里保留 50/100 ms 的超时。
 */
TouchType touch_detect_type(void)
{
    /* Step 1: GT911 over I2C2 (capacitive). */
    {
        uint8_t test_byte = 0;
        if (HAL_I2C_Mem_Read(&hi2c2, GT911_I2C_ADDRESS,
                             0x0000, I2C_MEMADD_SIZE_16BIT,
                             &test_byte, 1, 50) == HAL_OK) {
            if (test_byte == '9' || test_byte == '1') {
                return TOUCH_CAPACITIVE;
            }
        }
    }

    /* Step 2: NS2009 over I2C2 (resistive, RTOUCH). */
    if (ns2009_probe()) {
        return TOUCH_RESISTIVE;
    }

    /* Neither controller answered.  Fall back to resistive so the
     * GUI at least comes up; the read path will simply never report a
     * press.  This keeps a board with a broken I2C bus from bricking
     * the whole unit. */
    return TOUCH_RESISTIVE;
}

/**
 * @brief Get the latest touch position, calibrated for the resistive panel.
 *
 * 只读 volatile 快照，永不碰总线，所以不会阻塞。seqlock 校验保证拿到的是
 * 一致的三元组；极少数情况下快照正在被中断改写，重试几次即可，失败则
 * 用最后读到的值（对触摸而言一帧的偏差无感）。
 */
bool touch_get_position(TouchPointType *point)
{
    uint16_t x;
    uint16_t y;
    uint8_t  pressed;
    uint8_t  tries = 4U;

    if (point == NULL) {
        return false;
    }

    do {
        uint8_t s0 = g_touch_raw_seq;
        x = g_touch_raw_x;
        y = g_touch_raw_y;
        pressed = g_touch_raw_pressed;
        if (s0 == g_touch_raw_seq) {
            break;              /* 期间没有写入 → 快照一致 */
        }
    } while (--tries != 0U);

    /* 电容屏不需要标定；只有电阻屏套用 g_config 里的增益/偏置。 */
    if (detected_touch_type == TOUCH_RESISTIVE && pressed != 0U &&
        g_config.touch.calibrated) {
        touch_apply_calibration(x, y, &x, &y);
    }

    point->x       = x;
    point->y       = y;
    point->pressed = (pressed != 0U);
    return point->pressed;
}

/**
 * @brief Advance the DMA state machine and police the watchdog.
 *
 * 由 GUI 任务每 10 ms 调用一次。三件事：
 *   1. 有在途事务且超时 → 中止并复位（DMA 版 HAL 没有超时，只能自己守）；
 *   2. 在途事务未结束 → 直接返回，绝不重复发起；
 *   3. 空闲且到点 → 发起下一轮轮询。
 *
 * 发起前检查 hi2c.State，避开"任务正要发起、中断刚完成上一笔"的窄窗口。
 */
void touch_process(void)
{
    uint32_t now = HAL_GetTick();

    if (detected_touch_type == TOUCH_NONE) {
        return;
    }

    /* --- 1. 有在途事务：看门狗 + 等它结束 --- */
    if (s_stage != TS_IDLE) {
        if (!s_abort_pending &&
            (now - s_stage_tick) > TOUCH_I2C_TIMEOUT_MS) {
            if (HAL_I2C_Master_Abort_IT(&hi2c2, s_cur_addr) != HAL_OK) {
                /* 连中止都发不出去（句柄状态异常）：直接复位状态机。
                 * HAL 句柄若仍非 READY，下面的检查会继续拦住发起，
                 * 直到它自己回到可用的状态。 */
                s_stage = TS_IDLE;
            } else {
                s_abort_pending = true;   /* 等回调收尾 */
            }
        }
        return;
    }

    /* --- 2. 到点才发起新一轮 --- */
    if ((now - s_last_poll_tick) < TOUCH_POLL_PERIOD_MS) {
        return;
    }
    s_last_poll_tick = now;

    /* --- 3. HAL 必须就绪，否则这轮跳过 --- */
    if (hi2c2.State != HAL_I2C_STATE_READY) {
        return;
    }

    if (detected_touch_type == TOUCH_RESISTIVE) {
        (void)touch_start_ns_read(NS2009_REG_XH, s_ns_rx, 2U, TS_NS_X);
    } else {
        (void)touch_start_gt_status();
    }
}

/**
 * @brief Start calibration
 */
void touch_calibrate_start(void)
{
    calibrating = true;
    cal_point_index = 0;

    /* Reset calibration data */
    memset(cal_data, 0, sizeof(cal_data));
}

/**
 * @brief Add calibration point
 *
 * 消费最新的 volatile 样本，不再直接读总线——校准界面运行时 DMA 状态机
 * 照常在跑，这里等的是它发布的样本。
 */
bool touch_calibrate_add_point(uint16_t screen_x, uint16_t screen_y)
{
    TouchPointType tp;

    if (!calibrating || cal_point_index >= CAL_POINT_COUNT) {
        return false;
    }

    /* 只有电阻屏有校准界面；校准时 g_config.touch.calibrated 仍为 0，
     * 所以 touch_get_position() 返回的是未经变换的原始坐标。 */
    if (!touch_get_position(&tp) || !tp.pressed) {
        return false;
    }

    cal_data[cal_point_index].raw_x    = tp.x;
    cal_data[cal_point_index].raw_y    = tp.y;
    cal_data[cal_point_index].screen_x = screen_x;
    cal_data[cal_point_index].screen_y = screen_y;

    cal_point_index++;

    return (cal_point_index >= CAL_POINT_COUNT);
}

/**
 * @brief Complete calibration
 */
bool touch_calibrate_complete(void)
{
    float x1, y1;

    if (cal_point_index < CAL_POINT_COUNT) {
        calibrating = false;
        return false;
    }

    /* Linear regression for X: screen_x = raw_x * slope + offset */
    /* Using least squares: slope = (N*sum(xy) - sum(x)*sum(y)) / (N*sum(x^2) - sum(x)^2) */
    float sum_raw_x = 0, sum_screen_x = 0;
    float sum_raw_x2 = 0;
    float sum_raw_x_screen_x = 0;

    for (int i = 0; i < CAL_POINT_COUNT; i++) {
        sum_raw_x += cal_data[i].raw_x;
        sum_screen_x += cal_data[i].screen_x;
        sum_raw_x2 += (float)cal_data[i].raw_x * cal_data[i].raw_x;
        sum_raw_x_screen_x += (float)cal_data[i].raw_x * cal_data[i].screen_x;
    }

    x1 = (CAL_POINT_COUNT * sum_raw_x2 - sum_raw_x * sum_raw_x);
    if (fabsf(x1) < 0.001f) {
        calibrating = false;
        return false;
    }

    g_config.touch.x_gain = (CAL_POINT_COUNT * sum_raw_x_screen_x - sum_raw_x * sum_screen_x) / x1;
    g_config.touch.x_offset = (sum_screen_x - g_config.touch.x_gain * sum_raw_x) / CAL_POINT_COUNT;

    /* Linear regression for Y */
    float sum_raw_y = 0, sum_screen_y = 0;
    float sum_raw_y2 = 0;
    float sum_raw_y_screen_y = 0;

    for (int i = 0; i < CAL_POINT_COUNT; i++) {
        sum_raw_y += cal_data[i].raw_y;
        sum_screen_y += cal_data[i].screen_y;
        sum_raw_y2 += (float)cal_data[i].raw_y * cal_data[i].raw_y;
        sum_raw_y_screen_y += (float)cal_data[i].raw_y * cal_data[i].screen_y;
    }

    y1 = (CAL_POINT_COUNT * sum_raw_y2 - sum_raw_y * sum_raw_y);
    if (fabsf(y1) < 0.001f) {
        calibrating = false;
        return false;
    }

    g_config.touch.y_gain = (CAL_POINT_COUNT * sum_raw_y_screen_y - sum_raw_y * sum_screen_y) / y1;
    g_config.touch.y_offset = (sum_screen_y - g_config.touch.y_gain * sum_raw_y) / CAL_POINT_COUNT;

    /* Check center point residual */
    /* Center point is at index 4 */
    float predicted_x = cal_data[4].raw_x * g_config.touch.x_gain + g_config.touch.x_offset;
    float predicted_y = cal_data[4].raw_y * g_config.touch.y_gain + g_config.touch.y_offset;

    float residual_x = fabsf(predicted_x - cal_data[4].screen_x) / cal_data[4].screen_x;
    float residual_y = fabsf(predicted_y - cal_data[4].screen_y) / cal_data[4].screen_y;

    if (residual_x > 0.1f || residual_y > 0.1f) {
        /* Calibration failed - residual too high */
        calibrating = false;
        return false;
    }

    /* Calibration successful */
    g_config.touch.calibrated = 1U;
    calibrating = false;

    /* Save to config */
    config_save();

    return true;
}

/**
 * @brief Check if touch is pressed
 */
bool touch_is_pressed(void)
{
    return (g_touch_raw_pressed != 0U);
}

/**
 * @brief Apply calibration
 */
void touch_apply_calibration(uint16_t raw_x, uint16_t raw_y,
                             uint16_t *cal_x, uint16_t *cal_y)
{
    if (!g_config.touch.calibrated) {
        *cal_x = raw_x;
        *cal_y = raw_y;
        return;
    }

    *cal_x = (uint16_t)(raw_x * g_config.touch.x_gain + g_config.touch.x_offset);
    *cal_y = (uint16_t)(raw_y * g_config.touch.y_gain + g_config.touch.y_offset);

    /* Clamp to screen bounds */
    if (*cal_x > SCREEN_WIDTH) *cal_x = SCREEN_WIDTH;
    if (*cal_y > SCREEN_HEIGHT) *cal_y = SCREEN_HEIGHT;
}

/*============================================================================
 * Touch Type Specific Functions
 *============================================================================*/

/**
 * @brief Initialize capacitive touch
 */
void touch_capacitive_init(void)
{
    /* GT911 power-on reset.  CTouch_RSTX is PF3 per the .ioc (CTouch_RSTX);
     * this used to be a hardcoded PD10, which is LTDC_B3 - an LCD data
     * line - so the "reset" was pulling the panel bus down instead of the
     * touch controller. */
    HAL_GPIO_WritePin(CTouch_RSTX_GPIO_Port, CTouch_RSTX_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(CTouch_RSTX_GPIO_Port, CTouch_RSTX_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
}

/**
 * @brief Read capacitive touch
 *
 * 只返回最近一次 DMA 回调发布的样本，不再发 I2C 事务。
 */
bool touch_capacitive_read(uint16_t *x, uint16_t *y, bool *pressed)
{
    TouchPointType tp;
    bool ok = touch_get_position(&tp);

    if (x != NULL)       { *x = tp.x; }
    if (y != NULL)       { *y = tp.y; }
    if (pressed != NULL) { *pressed = tp.pressed; }

    return ok;
}

/**
 * @brief Touch interrupt ISR handler
 * @note Called from HAL_GPIO_EXTI_Callback
 *       For capacitive touch (GT911), this indicates touch event
 *       For resistive touch, this may not be used
 */
void touch_int_isr(void)
{
    /* Set flag to indicate touch interrupt occurred */
    /* The GUI task will process this in touch_process() */
    touch_interrupt_flag = 1;
}
