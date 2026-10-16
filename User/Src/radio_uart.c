/**
 * @file radio_uart.c
 * @brief UART4 电台 CAT 链路的双缓冲 DMA + IDLE line 接收层.
 *
 * @details 唯一直接接触 UART/DMA 的文件。协议逻辑在 radio_band.c，两者
 *          通过 radio_band_set_tx_hook / set_rx_hook 解耦。
 *
 *          ioc 里 UART4_RX 的 DMA 模式是 DMA_NORMAL，不是 Circular。
 *          Normal 模式下 DMA 传完即停，必须由软件重启
 *          HAL_UARTEx_ReceiveToIdle_DMA() 才能继续收。网上常见的
 *          "在回调里直接重启"的写法有一个隐患：重启的那几十个周期里
 *          到达的字节会丢——对 5~17 字节的 CAT 帧来说很容易整帧丢掉。
 *
 *          这里用双缓冲（ping-pong）消掉这个空窗：
 *
 *            启动时 DMA 写 A；A 收满或收到 IDLE 时，中断里把 A 交给
 *            协议层，同时立刻把 B 切成新的接收目标并重启 DMA。
 *            "交 A 给协议层"（纯内存操作，十几微秒）与"重启 DMA 写 B"
 *            顺序执行，之后 DMA 收 B 的同时 CPU 已经在处理 A 的内容，
 *            两个动作不存在互相等待。
 *
 *            因为事件到来时 DMA 必然已经停下（Normal 模式），A 中的数据
 *            是完整且稳定的，此刻读它永远安全——这是双缓冲相对
 *            "边收边读同一块内存"的关键优势。
 *
 *          三种事件都要处理，因为 IDLE 不一定每帧都来：
 *          · IDLE —— 一帧结束，用 size 取本段
 *          · HT   —— 收满一半，即使没有 IDLE 也要切段（连续流防丢帧）
 *          · TC   —— 收满整个缓冲
 *
 *          中断里只有指针算术、内存搬运和一次 HAL 调用，无解析、无动态
 *          分配、无阻塞等待。
 *
 *          ===================== ISR 与任务的分工 =====================
 *
 *          中断**不接触协议层**。协议识别（判断 ICOM / Kenwood / Yaesu、
 *          BCD 解码、是否切波段）逻辑不短，放进中断会拉长 UART4_IRQn 的
 *          占用时间，而保护相关的中断优先级更高，一旦被挤占就吃掉保护的
 *          响应余量。
 *
 *          所以 IDLE 中断只做三件事：
 *            1. 把本段字节 memcpy 进 s_staged[]（.bss，DMA 不参与）
 *            2. 切换乒乓缓冲并重启 ReceiveToIdle —— DMA 立刻继续收下一帧
 *            3. osSemaphoreRelease() 唤醒电台任务
 *
 *          协议推断与执行全在 app_comm_task() 里，经
 *          radio_uart_wait_frame() 取出后调用 radio_band_feed() 完成。
 *          这样"收"与"解析"彻底解耦：中断永不阻塞，任务的处理时长也
 *          不再影响接收。
 */

#include "radio_uart.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#ifndef LVGL_SIM
#include "cmsis_os.h"
#include "FreeRTOS.h"   /* pdMS_TO_TICKS */
#endif

extern UART_HandleTypeDef huart4;

/*============================================================================
 *  双缓冲
 *
 *  放在 AXI SRAM 的不可缓存窗口（app.ld 的 .dma_no_cache，0x24040000 起，
 *  由 MPU 标为 non-cacheable）。注意：H7 的 DMA1/DMA2 无法访问 DTCM/ITCM
 *  （只有 MDMA 可以），所以 DMA 缓冲绝不能放 DTCM；放进非缓存 SRAM 后
 *  DMA 与 CPU 看到的内容天然一致，无需手动 cache maintenance。
 *==========================================================================*/
#if defined(__GNUC__)
#define RADIO_UART_BUF_SECTION  __attribute__((section(".dma_buffer"), aligned(32)))
#else
#define RADIO_UART_BUF_SECTION
#endif

/* 32 字节 = DMA 半传输的长度，HAL 会用 RxXferSize/2 判HT，
 * 所以必须是偶数（实际取 32 已满足）。 */
static uint8_t s_buf_a[RADIO_UART_BUF_SIZE] RADIO_UART_BUF_SECTION;
static uint8_t s_buf_b[RADIO_UART_BUF_SIZE] RADIO_UART_BUF_SECTION;

/* 哪个缓冲是"当前 DMA 目标"，哪个是"可安全读取"的。
 * 启动时 active=0（A），spare=1（B）。 */
static volatile uint8_t s_active;

/* A 已经收了多少字节——HT 事件用它判断 A 是否真的收满了半段，
 * 避免把上一次残留的字节重复投递。 */
static volatile uint16_t s_active_len;

static volatile uint32_t s_started;   /*!< init 是否成功 */

static volatile RadioUartStatsType s_stats;

/*============================================================================
 *  ISR -> 任务 交接
 *
 *  中断不碰协议层，只把这一段字节搬到这里，然后 osSemaphoreRelease()
 *  叫醒任务。任务在 radio_uart_wait_frame() 里取走并喂给 radio_band。
 *
 *  为什么不用 radio_band_push_isr() 的逐字节环形缓冲：那个接口是为
 *  "UART 中断逐字节喂"设计的，而这里是 DMA 整段搬运，一次 memcpy 比
 *  逐字节入环更快，也让中断与任务的边界更清晰。
 *
 *  为什么交接缓冲要放在 .bss 而不像 s_buf_a/b 那样放 .dma_buffer：
 *  DMA 永远不会访问它，只有 CPU 写（中断）和读（任务），所以不需要
 *  non-cacheable 窗口，也就不必占用紧邻的 DMA 内存。
 *
 *  单槽 + 长度字段：radio_band_feed() 吃的是"一段字节"，天然适配。
 *  s_staged_len 必须在中断里先写、长度有效，再释放信号量——任务被唤醒
 *  时读到的必然是完整的这一段。volatile 保证两次访问不会被优化掉或
 *  重排。帧率极低（最多 100 ms 一帧）而任务 10 ms 一轮，单槽足够。
 *==========================================================================*/

static uint8_t  s_staged[RADIO_UART_BUF_SIZE];
static volatile uint16_t s_staged_len;
static volatile bool       s_staged_full;

/* 中断与任务共享，延迟创建：radio_uart_init() 在 osKernelInitialize()
 * 之后、osKernelStart() 之前被调用，那时 osSemaphoreNew 已经可用，
 * 但信号量必须在有任务运行时创建才安全，所以放到第一次 wait 时懒创建。 */
static osSemaphoreId_t s_rx_sem;

/*============================================================================
 *  钩子实现
 *==========================================================================*/

/* radio_band 通过这个把查询帧发出去。CAT 查询只有 3~7 字节，115200 baud
 * 下不到 1 ms，阻塞发完全可以接受，而且省下一个已经很紧张的 DMA 流
 * （DMA2 的流基本被 ADC/LTDC/SDMMC 占满）。 */
void radio_uart_tx(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U) {
        return;
    }
    /* 100 ms 上限：万一对端把线拉死，comm task 不会被卡在这里。 */
    if (HAL_UART_Transmit(&huart4, (uint8_t *)data, len, 100U) != HAL_OK) {
        s_stats.uart_errors++;
    }
}

/* radio_band 通过这个接收字节。HAL 回调与协议层都汇聚到这里。
 *
 * 注意：这是一个**任务上下文**接口，由 radio_uart_wait_frame() 调用。
 * 中断绝不能直接调用它 —— 那会把协议识别（BCD 解码、帧结构校验、
 * 换波段）搬进中断里。ISR 走的是 radio_uart_stage()。 */
void radio_uart_rx(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U) {
        return;
    }
    radio_band_feed(data, len);
}

/*============================================================================
 *  内部
 *==========================================================================*/

/* ISR 侧的唯一职责：把这一段字节放进交接缓冲，然后叫醒任务。
 * 无解析、无浮点、无阻塞，只有一次 memcpy 和一次信号量操作。
 *
 * 任务优先级低于本中断，所以释放信号量不会立刻抢占，ISR 的尾部开销
 * 就是一次 memcpy（最多 32 字节）加 osSemaphoreRelease() 内部的置位，
 * 通常在 2 us 以内。 */
static void radio_uart_stage(const uint8_t *src, uint16_t len)
{
    if (src == NULL || len == 0U) {
        return;
    }
    if (len > RADIO_UART_BUF_SIZE) {
        len = RADIO_UART_BUF_SIZE;
    }

    /* 上一次还没被取走就又来一帧：记一笔并覆盖。CAT 每 100 ms 最多一帧，
     * 任务 10 ms 一轮，理论上不会发生；真发生了说明任务被长时间阻塞，
     * 宁可丢掉一帧也不能在这里阻塞中断。 */
    if (s_staged_full) {
        s_stats.frame_dropped++;
    }

    memcpy(s_staged, src, len);

    /* 顺序重要：先写长度和标志，再释放信号量。这样任务被唤醒时一定
     * 能读到完整且长度有效的一段。 */
    s_staged_len  = len;
    s_staged_full = true;
    s_stats.handoffs++;

#ifndef LVGL_SIM
    if (s_rx_sem != NULL) {
        (void)osSemaphoreRelease(s_rx_sem);
    }
#endif
}

static inline uint8_t *active_buf(void)
{
    return (s_active == 0U) ? s_buf_a : s_buf_b;
}

/* 把当前缓冲里已收到的 len 字节放进交接缓冲，然后把两个 DMA 缓冲对调，
 * 并用另一个缓冲重新启动 DMA 接收。
 *
 * 调用时机：Normal 模式下 DMA 已经停下，当前缓冲的内容是稳定完整的，
 * 所以此刻读它不会与 DMA 竞争——这是双缓冲相对"边收边读同一块内存"的
 * 关键优势。
 *
 * 顺序也有讲究：先交接数据（任务可以开始解析了），再切缓冲重启 DMA。
 * 这样"重启 DMA"的重活落在任务已经拿到数据的时刻之后，两者不互相等待。 */
static void radio_uart_flip_and_restart(uint16_t len)
{
    if (len > RADIO_UART_BUF_SIZE) {
        len = RADIO_UART_BUF_SIZE;
    }

    /* 1) 交给任务：只搬运，解析留给任务上下文。 */
    if (len > 0U) {
        radio_uart_stage(active_buf(), len);
    }

    /* 2) 对调：另一个缓冲成为新的 DMA 目标。
     *    s_active_len 必须清零——新缓冲还没收到任何字节，下一次 IDLE
     *    报上来的 size 是从 1 开始数的，若沿用旧值会把新数据误判成
     *    "重复事件"而丢弃。 */
    s_active     = (s_active == 0U) ? 1U : 0U;
    s_active_len = 0U;

    /* 3) 立刻重启接收，让 DMA 去写那个还没被读过的缓冲。
     *    Normal 模式下这次调用之后 DMA 又开始跑。
     *    s_active_len 由下一次 IDLE 回调根据 size 重新建立。 */
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart4, active_buf(),
                                     RADIO_UART_BUF_SIZE) != HAL_OK) {
        /* 只有在 UART 状态机出问题时才会失败（例如重入）。记下来便于
         * 诊断，不在这里重试——重试只会掩盖真正的状态错乱。 */
        s_stats.restart_fail++;
    }
    else {
        s_stats.restarts++;
    }
}

/*============================================================================
 *  HAL 回调
 *
 *  入口名不是 HAL_UARTEx_RxEventCallback —— 那个 HAL 弱符号由 comm.c 独占，
 *  它按实例把 UART4 的事件转到这里。见 radio_uart.h 的说明。
 *==========================================================================*/

void radio_uart_rx_event(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != &huart4 || s_started == 0U) {
        return;
    }

    /* HAL 在每次回调前已经把 RxEventType 设好。Normal 模式下三种事件的
     * size 语义都是"本次传输累计已收"，且到回调时 DMA 已停，当前缓冲
     * 内容完整稳定，此刻读它不会与 DMA 竞争：
     *   IDLE —— size = 累计已收（< 缓冲长度）
     *   HT   —— size = 缓冲长度 / 2（前半段已满）
     *   TC   —— size = 缓冲长度（整段已满） */
    switch (huart->RxEventType) {

    case HAL_UART_RXEVENT_HT:
        s_stats.ht_events++;
        /* 前半段满了。直接交出并对调，不看 size——HT 时 size 恒等于
         * 半段长度，且此刻后半段可能已经又收了一些，一起交出去更安全。 */
        radio_uart_flip_and_restart((uint16_t)(RADIO_UART_BUF_SIZE / 2U));
        break;

    case HAL_UART_RXEVENT_TC:
        s_stats.tc_events++;
        radio_uart_flip_and_restart(RADIO_UART_BUF_SIZE);
        break;

    case HAL_UART_RXEVENT_IDLE:
        s_stats.idle_events++;
        /* IDLE 与 HT 可能紧邻发生：HT 已经把前半段交走并重启过了，
         * 紧接着来的 IDLE 描述的是"新那一轮"的数据。s_active_len 记录
         * 本轮已经交出去多少，size 不大于它就说明没有新内容，
         * 此时只重启接收、不重复投递（否则同一段字节会被解析两次）。 */
        if (size > s_active_len) {
            uint16_t new_len = size;
            s_active_len     = size;
            radio_uart_flip_and_restart(new_len);
        }
        else {
            /* 无新字节，但 DMA 仍处于停止状态，必须重启。 */
            s_stats.dup_idle++;
            (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart4, active_buf(),
                                                RADIO_UART_BUF_SIZE);
            s_stats.restarts++;
        }
        break;

    default:
        /* 理论上到不了这里。保守起见按 TC 处理，交出整个缓冲并重启，
         * 宁可多交一次让协议层丢弃，也不能停止接收。 */
        radio_uart_flip_and_restart(RADIO_UART_BUF_SIZE);
        break;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart4) {
        return;
    }
    /* CAT 线上偶发帧错误很常见（干扰、老化的 USB 转串口线）。
     * 只记录不动作：协议层的半帧超时机制会丢弃残帧，链路能自恢复。
     * 但若 UART 进入了错误态，必须重启接收，否则 DMA 再也不会推进。 */
    s_stats.uart_errors++;
    if (s_started != 0U) {
        (void)HAL_UART_AbortReceive(&huart4);
        s_active_len = 0U;
        if (HAL_UARTEx_ReceiveToIdle_DMA(&huart4, active_buf(),
                                         RADIO_UART_BUF_SIZE) != HAL_OK) {
            s_stats.restart_fail++;
        }
        else {
            s_stats.restarts++;
        }
    }
}

/*============================================================================
 *  Public API
 *==========================================================================*/

/* s_stats 是 volatile（中断与任务共享），memset 会丢 volatile 限定符，
 * 编译器会告警。逐字段清零即可，字段少，不需要花哨的写法。 */
static void radio_uart_clear_stats(void)
{
    s_stats.idle_events  = 0U;
    s_stats.ht_events    = 0U;
    s_stats.tc_events    = 0U;
    s_stats.restarts     = 0U;
    s_stats.restart_fail = 0U;
    s_stats.uart_errors  = 0U;
    s_stats.dup_idle     = 0U;
    s_stats.handoffs     = 0U;
    s_stats.frame_dropped = 0U;
}

bool radio_uart_init(void)
{
    /* 没有 DMA 句柄说明 ioc 里 UART4_RX 的 DMA 没生效，或者还没
     * 重新生成代码。与其让后面的 ReceiveToIdle 莫名失败，不如在这里
     * 明确报出来。 */
    if (huart4.hdmarx == NULL) {
        return false;
    }

    radio_uart_clear_stats();
    memset(s_buf_a, 0, sizeof s_buf_a);
    memset(s_buf_b, 0, sizeof s_buf_b);
    radio_uart_flush_pending();

    s_active     = 0U;      /* A 先收 */
    s_active_len = 0U;
    s_started    = 0U;

    /* 协议层先就位，这样 DMA 一旦开始收，钩子已经可用。 */
    radio_band_init(g_config.radio_type);
    radio_band_set_tx_hook(radio_uart_tx);
    radio_band_set_rx_hook(radio_uart_rx);

    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart4, s_buf_a,
                                     RADIO_UART_BUF_SIZE) != HAL_OK) {
        return false;
    }
    s_stats.restarts++;
    s_started = 1U;
    return true;
}

void radio_uart_deinit(void)
{
    s_started = 0U;
    (void)HAL_UART_AbortReceive(&huart4);
    s_active_len = 0U;
    radio_uart_flush_pending();
}

/* 改 UART4 波特率并立即生效。任务上下文调用。
 *
 * HAL_UART_Init() 不是"幂等地重新配一遍"——它会先把外设彻底关掉再按
 * 新的 InitStruct 重新打开。AbortReceive 是为了让"还没处理完的当前
 * DMA 传输"停下来，避免它在我们修改 InitStruct 期间把外设状态机搅
 * 乱。Abort 完成后再改 InitStruct、Init、最后重启 ReceiveToIdle。
 *
 * 失败的回滚策略：保留原 InitStruct 不动。调用方可以再读
 * g_config.radio_baud_rate 来确认实际生效的值是否就是用户要的。 */
bool radio_uart_set_baud(uint32_t baud)
{
    if (s_started == 0U) {
        return false;
    }
    if (baud == 0U) {
        return false;
    }

    (void)HAL_UART_AbortReceive(&huart4);

    uint32_t prev_baud = huart4.Init.BaudRate;
    huart4.Init.BaudRate = baud;

    if (HAL_UART_Init(&huart4) != HAL_OK) {
        /* 回滚：把 InitStruct 改回旧值再 Init 一次，确保外设至少能用。 */
        huart4.Init.BaudRate = prev_baud;
        if (HAL_UART_Init(&huart4) != HAL_OK) {
            s_stats.restart_fail++;
            return false;
        }
        s_stats.restart_fail++;
        return false;
    }

    /* 切到 A 缓冲重新启动。Init 之后 DMA 句柄 huart4.hdmarx 仍在
     * （MX 把它放在外设 handle 里），所以可以直接 ReceiveToIdle。 */
    s_active     = 0U;
    s_active_len = 0U;
    radio_uart_flush_pending();
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart4, s_buf_a,
                                     RADIO_UART_BUF_SIZE) != HAL_OK) {
        s_started = 0U;
        s_stats.restart_fail++;
        return false;
    }
    s_stats.restarts++;
    return true;
}

void radio_uart_get_stats(RadioUartStatsType *out)
{
    if (out == NULL) {
        return;
    }
    *out = s_stats;
}

void radio_uart_flush_pending(void)
{
    s_staged_len  = 0U;
    s_staged_full = false;
}

bool radio_uart_wait_frame(uint32_t timeout_ms)
{
#ifdef LVGL_SIM
    /* No RTOS on the PC simulator: the "ISR" path does not run there
     * anyway, so report "nothing waiting" rather than pretending. */
    (void)timeout_ms;
    return false;
#else
    /* 懒创建：radio_uart_init() 可能在 osKernelStart() 之前被调用，
     * 那时创建信号量不安全（还没有任务在跑，osSemaphoreNew 依赖内核
     * 已初始化）。真正等待一定发生在任务里，所以放这里是安全的。 */
    if (s_rx_sem == NULL) {
        /* Binary semaphore: max_count 1, initially empty.  CMSIS-RTOS2
         * takes (max_count, initial_count, attr), not the single-attr
         * form RTOS v1 used. */
        s_rx_sem = osSemaphoreNew(1U, 0U, NULL);
        if (s_rx_sem == NULL) {
            return false;               /* 内存不足：放弃这一轮，不死循环 */
        }
        /* 中断可能在信号量建好之前就投递过数据，这里补一次检查，
         * 免得那一份要等到下一个 CAT 帧才被处理。 */
        if (s_staged_full) {
            (void)osSemaphoreRelease(s_rx_sem);
        }
    }

    /* 先看有没有现成的，省掉一次不必要的等待。 */
    if (s_staged_full) {
        goto take;
    }

    if (timeout_ms == 0U) {
        return false;                   /* 非阻塞轮询模式 */
    }

    /* timeout_ms 是"最多等这么久"，不是"无限等"：comm task 还要按 10 ms
     * 周期干别的事（收 PC 命令、推进 CAT 轮询累加器），无限等会让那两件事
     * 饿死。osSemaphoreAcquire 只认 tick 数，所以先换算；亚 tick 的值向上
     * 取整到 1，否则会变成永不超时。 */
    {
        uint32_t ticks = pdMS_TO_TICKS(timeout_ms);
        if (ticks == 0U) {
            ticks = 1U;
        }
        if (osSemaphoreAcquire(s_rx_sem, ticks) != osOK) {
            return false;
        }
    }

take:
    /* 从这里开始不再有中断会改 s_staged_*（s_staged_full 在下面清零，
     * 中断的写入条件就是它为真），所以下面的读取是安全的。 */
    if (!s_staged_full) {
        return false;                   /* 信号量醒了但数据已被取走 */
    }

    {
        uint16_t len = s_staged_len;
        s_staged_full = false;          /* 先清标志，让下一次 ISR 可以投递 */
        s_staged_len  = 0U;
        radio_uart_rx(s_staged, len);    /* 解析全在这一步，任务上下文 */
    }
    return true;
#endif
}