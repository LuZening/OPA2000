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
 */

#include "radio_uart.h"
#include "stm32h7xx_hal.h"
#include <string.h>

extern UART_HandleTypeDef huart4;

/*============================================================================
 *  双缓冲
 *
 *  放 DTCMRAM：H7 的 D-cache 会让 DMA 与 CPU 看到的内容不一致，而
 *  CAT 缓冲很小很热，牺牲一点 DTCM 容量换零一致性最划算。
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

/* radio_band 通过这个接收字节。HAL 回调与协议层都汇聚到这里。 */
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

static inline uint8_t *active_buf(void)
{
    return (s_active == 0U) ? s_buf_a : s_buf_b;
}

/* 把当前缓冲里已收到的 len 字节交给协议层，然后把两个缓冲对调，
 * 并用另一个缓冲重新启动 DMA 接收。
 *
 * 调用时机：Normal 模式下 DMA 已经停下，当前缓冲的内容是稳定完整的，
 * 所以此刻读它不会与 DMA 竞争——这是双缓冲相对"边收边读同一块内存"的
 * 关键优势。 */
static void radio_uart_flip_and_restart(uint16_t len)
{
    if (len > RADIO_UART_BUF_SIZE) {
        len = RADIO_UART_BUF_SIZE;
    }

    /* 1) 先把收到的数据交出去（纯内存操作）。 */
    if (len > 0U) {
        radio_uart_rx(active_buf(), len);
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
 *==========================================================================*/

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
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
}

void radio_uart_get_stats(RadioUartStatsType *out)
{
    if (out == NULL) {
        return;
    }
    *out = s_stats;
}