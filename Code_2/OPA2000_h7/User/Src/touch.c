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
 *          原来 touch_process() 每 10 ms 同步读一次总线，而 HAL 的等待循环
 *          是纯自旋（I2C_WaitOnFlagUntilTimeout 里 while 轮询，没有 osDelay
 *          也没有 taskYIELD，不让出 CPU）。芯片不应答时每次最多自旋 100 ms，
 *          GUI 任务这一圈就废掉，屏幕刷新率掉到几 Hz。改成 DMA 之后：
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
 *          只开 DMA 流中断的话，传输会启动但回调一次都不来。本文件不定义
 *          中断处理函数，交给 CubeMX 生成的 stm32h7xx_it.c（避免重复定义）。
 *
 *          ==================== 三个必须守住的约束 ====================
 *
 *          1. 缓冲必须放 .dma_buffer（AXI SRAM 非缓存窗口）。DMA1/DMA2 访问
 *             不到 DTCM，缓存窗口则需要手动 cache maintenance，本项目的
 *             MPU region4 已把它标为不可缓存。
 *
 *          2. DMA 版 HAL 函数没有超时参数——总线挂死时传输永不完成。所以有
 *             软件看门狗：touch_process() 用 HAL_GetTick() 比对，超时则
 *             HAL_I2C_Master_Abort_IT() 中止并复位状态机。
 *
 *          3. 中止不一定能把总线救回来（从机可能一直拉着 SDA 不放），所以
 *             还要有总线恢复：把 SCL/SDA 临时改成 GPIO 开漏，手工打 9 个
 *             时钟让从机把剩余的位发完，再补一个 STOP，最后重新挂载外设。
 *             触发条件是"连续若干笔事务失败"或"句柄长时间回不到 READY"。
 */

#include "touch.h"
#include "config.h"
#include "hw_pinmap.h"
#include "stm32h7xx_hal.h"
#include "main.h"
#include "ns2009.h"

/* CubeMX 在 Core/Src/i2c.c 里定义；这里前向声明就够（ns2009.c 同样做法）。 */
extern I2C_HandleTypeDef hi2c2;

/*! CubeMX 生成的 I2C2 初始化（含 MspInit：引脚 AF、DMA 链接、NVIC、滤波器）。
 *  总线恢复时用它重新挂载外设——只调 HAL_I2C_Init() 会漏掉两个滤波器配置。 */
extern void MX_I2C2_Init(void);

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

/* NS2009 每笔读事务只拿一个 12 位值（命令码见 ns2009.h），返回 2 字节。
 * 所以一次采样 = 4 笔事务：Z1 → Z2 → X → Y，每笔都是独立的一次转换。
 * 一次读多字节在这里不成立——那 2 字节就是"12 位 + 4 位冗余"的上限。 */
#define NS_READ_BYTES         NS2009_READ_BYTES   /* 2 */

/* 每笔事务的软件看门狗上限。400 kHz 下一笔 8 字节事务约 200 us，
 * 10 ms 已是 50 倍余量；再长只会让总线挂死时更晚被发现。 */
#define TOUCH_I2C_TIMEOUT_MS  10U

/* 轮询周期：一轮样本落定后才发起下一轮，所以这是"最快"周期。 */
#define TOUCH_POLL_PERIOD_MS  10U

/* 连续这么多笔事务失败 → 判定总线需要恢复。 */
#define TOUCH_RECOVER_AFTER_FAULTS   3U

/* 恢复的最低间隔：避免器件缺失（每轮都 NACK）时每秒恢复上千次。 */
#define TOUCH_RECOVER_MIN_INTERVAL_MS  1000U

/* 句柄持续不回到 READY 超过这么久 → 中止也没生效，直接硬恢复。 */
#define TOUCH_HAL_STUCK_MS    200U

/* SCL 打拍的半周期。ST 建议恢复时钟频率 ≤ 100 kHz；280 MHz 下这个
 * volatile 循环约 5 us，即 ≈ 100 kHz 以内。用循环而不是 DWT，避免依赖
 * 调试单元是否使能。 */
#define TOUCH_RECOVER_HALF_LOOPS  500U

/* 恢复时手工打多少个 SCL 时钟。一个字节 8 位 + 1 个应答位 = 9。 */
#define TOUCH_RECOVER_CLOCKS  9U

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

/*! NS2009 的 2 字节读回：[0] = 12 位的高 8 位，[1] = 低 4 位 + 冗余 4 位。
 *  四个值（Z1/Z2/X/Y）各用一份：它们分属不同的转换，DMA 搬完到回调
 *  触发之间缓冲不能被下一笔覆写。 */
static uint8_t s_ns_z1[NS_READ_BYTES]    TOUCH_DMA_BUF;
static uint8_t s_ns_z2[NS_READ_BYTES]    TOUCH_DMA_BUF;
static uint8_t s_ns_x[NS_READ_BYTES]     TOUCH_DMA_BUF;
static uint8_t s_ns_y[NS_READ_BYTES]     TOUCH_DMA_BUF;
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

/*!< "有一个新样本待取走"标志，由 I2C 中断置位、读者清零。
 *
 *  为什么需要它：LVGL 每 tick 调一次 indev 的 read_cb，而采样是 10 ms 一次
 *  ——两个周期不同频，必然出现 LVGL 连续多帧读到同一个样本。LVGL 自己会去重
 *  （坐标与状态都不变就不产生事件），所以功能上不出重复点击；但那是依赖 LVGL
 *  内部实现的行为。OPA1000 用一个"取走即清"的标志（main.c 的 TS_touched）
 *  来表达同一件事，不依赖下游的容忍度，这里照做。
 *
 *  与 g_touch_raw_seq 的分工：seq 是自增计数器，给 seqlock 校验用（判断快照
 *  在读取过程中有没有被改写）；本标志是"取走即清"，给消费者判断"这是不是
 *  新数据"。两者语义不同，不能互相替代——seqlock 的读者不会清 seq，而本标志
 *  的消费者必须清，否则标志永远为真。 */
volatile uint8_t  g_touch_new_sample   = 0U;

/*============================================================================
 * 诊断量
 *==========================================================================*/

volatile uint32_t g_touch_bus_faults  = 0U;
volatile uint32_t g_touch_last_error  = 0U;
volatile uint32_t g_touch_recoveries  = 0U;

/*!< NS2009 最近一次的压力原始值。0..8190；空闲时接近基线。
 *  调试用：把阈值调准之前，先看这个数在按与不按时的实际范围。 */
volatile uint32_t g_touch_raw_z       = 0U;

/*============================================================================
 * 私有状态
 *============================================================================*/

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
    TS_NS_Z1,        /*!< 等 NS2009 Z1 读（面板电压）          */
    TS_NS_Z2,        /*!< 等 NS2009 Z2 读（触摸点电压）        */
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

/*!< 连续失败笔数（由中断自增、任务清零）。原子性足够：中断只做读改写，
 *    任务只做一次 8 位写入，最坏情况丢一次自增，对启发式阈值无影响。 */
static volatile uint8_t    s_consec_faults = 0U;

/* 以下三个只在任务上下文访问，无需 volatile。 */
static uint32_t s_last_poll_tick    = 0U;
static uint32_t s_last_recover_tick = 0U;
static uint32_t s_hal_busy_since    = 0U;

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

    /* 顺带通知"有新样本了"。放在最后，使它天然成为发布完成信号——
     * 消费者若在清除本标志之前读快照，拿到的必然是这一次的完整数据。 */
    g_touch_new_sample = 1U;
}

/**
 * @brief 发起一笔 NS2009 读。返回 false 表示没能启动，状态机已回 IDLE。
 *
 * \p cmd 是**命令码**而非寄存器地址（0xC0/0xD0/0xE0/0xF0，见 ns2009.h）：
 * NS2009 用命令码配置 ADC 的 MUX，一笔事务只能取一个 12 位值。
 *
 * @note 可由任务上下文（首笔）或中断上下文（串联下一笔）调用。
 */
static bool touch_start_ns_read(uint8_t cmd, uint8_t *dst, TouchStage st)
{
    s_cur_addr   = NS2009_ADDR;
    s_stage      = st;
    s_stage_tick = HAL_GetTick();

    if (HAL_I2C_Mem_Read_DMA(&hi2c2, NS2009_ADDR, cmd, I2C_MEMADD_SIZE_8BIT,
                             dst, NS_READ_BYTES) != HAL_OK) {
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
 * 总线恢复
 *
 * 中止一笔事务只能让主机停下来，救不回被从机拉着 SDA 不放的总线：
 * 从机可能正好停在"发了一半的字节"上，它还在等第 N 个时钟沿，而主机已经
 * 放手了。唯一的办法是主机继续把时钟打完，让从机把剩下的位吐完，再补一个
 * STOP 让它回到空闲。
 *
 * 步骤（顺序有讲究，别调换）：
 *   1. 摘掉外设（DeInit）：释放引脚、停掉 DMA 与中断，引脚回到复位态；
 *   2. 把 SCL/SDA 临时配成开漏 GPIO：SDA 释放为高（靠上拉），SCL 由我们
 *      自己驱动，这样才有"手工打拍"的能力；
 *   3. SCL 打 9 个完整时钟（8 数据 + 1 应答），给从机机会把位发完；
 *   4. 补一个 STOP 时序：SDA 低 → SCL 高 → SDA 高；
 *   5. MX_I2C2_Init() 重新挂载外设，引脚回到 AF4、DMA 重新链接、NVIC 重新
 *      使能、两个滤波器重新配置。
 *
 * 全程约 100 us，跑在任务上下文（touch_process），不构成阻塞问题。
 *==========================================================================*/

/** @brief 送回约 5 us 的忙等，用于 SCL 打拍（≤ 100 kHz）。 */
static void touch_recover_half_delay(void)
{
    for (volatile uint32_t i = 0U; i < TOUCH_RECOVER_HALF_LOOPS; i++) {
        __NOP();
    }
}

/**
 * @brief 从被从机拉死的总线里恢复出来，并重新挂载 I2C2。
 * @note 只在任务上下文调用，且调用时不应有在途事务。
 */
static void touch_bus_recover(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* --- 1. 摘掉外设：释放引脚，停掉 DMA 与 NVIC --- */
    (void)HAL_I2C_DeInit(&hi2c2);

    /* --- 2. SCL / SDA 临时改成开漏 GPIO --- */
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;              /* 板上另有上拉，这里只是兜底 */
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = I2C2_SCL_Pin;
    HAL_GPIO_Init(I2C2_SCL_GPIO_Port, &gpio);
    gpio.Pin = I2C2_SDA_Pin;
    HAL_GPIO_Init(I2C2_SDA_GPIO_Port, &gpio);

    /* SDA 释放（高），SCL 也先抬起：空闲态 */
    HAL_GPIO_WritePin(I2C2_SDA_GPIO_Port, I2C2_SDA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C2_SCL_GPIO_Port, I2C2_SCL_Pin, GPIO_PIN_SET);
    touch_recover_half_delay();

    /* --- 3. 打 9 个 SCL 时钟 --- */
    for (uint32_t i = 0U; i < TOUCH_RECOVER_CLOCKS; i++) {
        HAL_GPIO_WritePin(I2C2_SCL_GPIO_Port, I2C2_SCL_Pin, GPIO_PIN_RESET);
        touch_recover_half_delay();
        HAL_GPIO_WritePin(I2C2_SCL_GPIO_Port, I2C2_SCL_Pin, GPIO_PIN_SET);
        touch_recover_half_delay();
    }

    /* --- 4. 补一个 STOP：SCL 高电平期间 SDA 由低跳高 --- */
    HAL_GPIO_WritePin(I2C2_SDA_GPIO_Port, I2C2_SDA_Pin, GPIO_PIN_RESET);
    touch_recover_half_delay();
    HAL_GPIO_WritePin(I2C2_SCL_GPIO_Port, I2C2_SCL_Pin, GPIO_PIN_SET);
    touch_recover_half_delay();
    HAL_GPIO_WritePin(I2C2_SDA_GPIO_Port, I2C2_SDA_Pin, GPIO_PIN_SET);
    touch_recover_half_delay();

    /* --- 5. 重新挂载外设 --- */
    MX_I2C2_Init();

    /* 状态机一并复位：DeInit 期间不可能有回调，任何在途痕迹都已失效。 */
    s_stage         = TS_IDLE;
    s_abort_pending = false;
    s_consec_faults = 0U;
    s_last_poll_tick = HAL_GetTick();

    g_touch_recoveries++;
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

    /* 有一笔成功完成 → 总线是活的，清零连续失败计数。 */
    s_consec_faults = 0U;

    switch (s_stage) {

    case TS_NS_Z1:
        /* Z1（面板电压）到手，接着取 Z2（触摸点电压）。两者合成压力，
         * 顺序不能颠倒：手册把它们定义为两次独立转换，谁先谁后不影响
         * 结果，但固定顺序便于排查哪一笔出错。 */
        if (!NS2009_IS_VALID(s_ns_z1[1])) {
            touch_publish(false, 0U, 0U);
            s_stage = TS_IDLE;
            break;
        }
        (void)touch_start_ns_read(NS2009_CMD_Z2_12BIT, s_ns_z2, TS_NS_Z2);
        break;

    case TS_NS_Z2:
        if (!NS2009_IS_VALID(s_ns_z2[1])) {
            touch_publish(false, 0U, 0U);
            s_stage = TS_IDLE;
            break;
        }
        /* 压力此刻已可算，先存下来供调试看；坐标读完后才决定按下与否，
         * 因为无接触时 X/Y 是漂移的，没有参考价值。 */
        g_touch_raw_z = NS2009_PRESSURE(NS2009_VALUE(s_ns_z1[0], s_ns_z1[1]),
                                       NS2009_VALUE(s_ns_z2[0], s_ns_z2[1]));
        (void)touch_start_ns_read(NS2009_CMD_X_12BIT, s_ns_x, TS_NS_X);
        break;

    case TS_NS_X:
        if (!NS2009_IS_VALID(s_ns_x[1])) {
            touch_publish(false, 0U, 0U);
            s_stage = TS_IDLE;
            break;
        }
        (void)touch_start_ns_read(NS2009_CMD_Y_12BIT, s_ns_y, TS_NS_Y);
        break;

    case TS_NS_Y: {
        /* 失败安全：任何一个值的冗余位非 0 都当作"未按下"。宁可漏掉一次
         * 点击，也不要发出一次位置错误的点击——对面板而言这个方向是安全
         * 的。注意这种失败不计入 g_touch_bus_faults（总线本身是好的，
         * 是数据无效），要发现它得看 g_touch_raw_z 有没有异常值。 */
        if (!NS2009_IS_VALID(s_ns_y[1])) {
            touch_publish(false, 0U, 0U);
            s_stage = TS_IDLE;
            break;
        }
        touch_publish(g_touch_raw_z > NS2009_Z_PRESS_THRESHOLD,
                      NS2009_VALUE(s_ns_x[0], s_ns_x[1]),
                      NS2009_VALUE(s_ns_y[0], s_ns_y[1]));
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
    s_consec_faults = 0U;
    if (s_stage == TS_GT_CLEAR) {
        s_stage = TS_IDLE;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C2) {
        return;
    }

    /* HAL 已经在 I2C_IsErrorOccurred() 里清掉了 BERR / ARLO / OVR / AF
     * 标志（见 stm32h7xx_hal_i2c.c 4680-4703、1516），所以这里不需要再清，
     * 只把错误码留下来供排查：值到含义的对照表见 touch.h。 */
    g_touch_last_error = (uint32_t)hi2c->ErrorCode;

    /* 失败安全：宁可报"未按下"也不留下幻影按压。这与原来的行为一致
     * （旧代码在读失败时也是把 pressed 置 false）。代价是总线抖动会让
     * 一次按压中断一拍，对控制面板来说这个方向是正确的。 */
    touch_publish(false, 0U, 0U);

    if (s_consec_faults < 0xFFU) {
        s_consec_faults++;
    }
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
 * @brief 读快照并填 \p point。返回 point->pressed。
 *
 * seqlock 校验保证拿到的是一致的三元组；极少数情况下快照正在被中断改写，
 * 重试几次即可，失败则用最后读到的值（对触摸而言一帧的偏差无感）。
 * 两次公共 API（取状态 / 取新样本）共用它，免得标定变换走偏。
 */
static bool touch_read_snapshot(TouchPointType *point)
{
    uint16_t x;
    uint16_t y;
    uint8_t  pressed;
    uint8_t  tries = 4U;

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
 * @brief Get the latest touch position, calibrated for the resistive panel.
 *
 * 只读 volatile 快照，永不碰总线，所以不会阻塞。返回的是**当前状态**——
 * 同一个样本会被读到多次（LVGL 每 tick 一次，采样 10 ms 一次）。要只处理
 * 新样本，用 touch_take_sample()。
 */
bool touch_get_position(TouchPointType *point)
{
    if (point == NULL) {
        return false;
    }
    return touch_read_snapshot(point);
}

/**
 * @brief 取走一个新样本（OPA1000 的 TS_touched 标志的封装）。
 *
 * 有新样本才填 point 并返回 true，并清掉"取走即清"标志；没有则返回 false
 * 且不碰 point。适合 LVGL 的 indev read_cb 这种"事件驱动"的消费者。
 *
 * 清除与中断置位可能相邻发生；这里读-清一次就收工，最多漏掉恰好落在这一
 * 瞬间的样本，下个周期会补上。不需要临界区。
 */
bool touch_take_sample(TouchPointType *point)
{
    if (point == NULL) {
        return false;
    }

    if (g_touch_new_sample == 0U) {
        return false;           /* 上一份已经被取走了 */
    }
    g_touch_new_sample = 0U;   /* 取走即清 */

    /* 这一次是真的新样本，但 pressed 可能为 0（抬手）。两种情况都算
     * "有新数据"，返回的是 pressed 的值，由调用方判断。 */
    touch_read_snapshot(point);
    return true;
}

/**
 * @brief Advance the DMA state machine, police the watchdog, recover the bus.
 *
 * 由 GUI 任务每 10 ms 调用一次。四件事，按优先级从上到下：
 *
 *   1. 有在途事务 → 看门狗（DMA 版 HAL 没有超时，只能自己守），然后返回；
 *   2. 连续失败够多，或句柄长期不 READY → 总线恢复，然后返回；
 *   3. 空闲且到点 → 发起下一轮轮询；
 *   4. 发起前检查 hi2c.State，避开"任务正要发起、中断刚完成上一笔"的窄窗口。
 *
 * 全程无等待，唯一可能耗时的是恢复（约 100 us，见 touch_bus_recover）。
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

    /* --- 2a. 句柄长期回不到 READY：中止没生效，只能硬恢复 --- */
    if (hi2c2.State != HAL_I2C_STATE_READY) {
        if (s_hal_busy_since == 0U) {
            s_hal_busy_since = now;
        } else if ((now - s_hal_busy_since) > TOUCH_HAL_STUCK_MS &&
                   (now - s_last_recover_tick) >= TOUCH_RECOVER_MIN_INTERVAL_MS) {
            s_hal_busy_since    = 0U;
            s_last_recover_tick = now;
            touch_bus_recover();
        }
        return;
    }
    s_hal_busy_since = 0U;

    /* --- 2b. 连续失败够多：总线多半被从机拉住了 --- */
    if (s_consec_faults >= TOUCH_RECOVER_AFTER_FAULTS &&
        (now - s_last_recover_tick) >= TOUCH_RECOVER_MIN_INTERVAL_MS) {
        s_last_recover_tick = now;
        touch_bus_recover();
        return;
    }

    /* --- 3. 到点才发起新一轮 --- */
    if ((now - s_last_poll_tick) < TOUCH_POLL_PERIOD_MS) {
        return;
    }
    s_last_poll_tick = now;

    if (detected_touch_type == TOUCH_RESISTIVE) {
        /* 从 Z1 开始：压力决定"按没按"，而 X/Y 在无接触时是漂移的，
         * 没有参考价值。 */
        (void)touch_start_ns_read(NS2009_CMD_Z1_12BIT, s_ns_z1, TS_NS_Z1);
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
