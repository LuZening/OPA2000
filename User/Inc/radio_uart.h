/**
 * @file radio_uart.h
 * @brief UART4 电台 CAT 链路的双缓冲 DMA + IDLE line 接收层.
 *
 * @details 这是唯一直接接触 UART/DMA 的文件。协议逻辑全在 radio_band.c，
 *          两者通过 radio_band_set_tx_hook / set_rx_hook 解耦，因此协议层
 *          可以在 host 上完整单元测试。
 *
 *          ===================== 为什么用双缓冲 =====================
 *
 *          ioc 里 UART4_RX 的 DMA 模式是 **DMA_NORMAL**（不是 Circular）。
 *          Normal 模式下 DMA 传完就停，必须由软件重新调用
 *          HAL_UARTEx_ReceiveToIdle_DMA() 才会继续收。
 *
 *          网上多数例程是"在回调里直接重启"，那有一个隐患：重启的那几十
 *          个周期（函数调用 + HAL 状态机 + 重新配置 DMA 控制器）里到达的
 *          字节会丢失。对 CAT 来说这很容易吃掉一个 5~17 字节的帧。
 *
 *          双缓冲（ping-pong）把这个问题消掉：
 *            · 两个等大缓冲 A / B；
 *            · DMA 往"当前缓冲"写，软件在中断里处理"另一个缓冲"；
 *            · 交完立刻把另一个缓冲切成新的接收目标并重启 DMA；
 *            · 于是"重启 DMA"这件事发生在 CPU 处理另一个缓冲的期间，
 *              与新数据到达并行，不存在空窗。
 *
 *          CAT 帧最长 17 字节，缓冲取 32 字节已远超需求；115200 baud 下
 *          32 字节需要约 2.8 ms 收完，而重启只需几十微秒，余量充足。
 *
 *          ===================== 三种事件 =====================
 *
 *          HAL_UARTEx_ReceiveToIdle_DMA() 会在三个时机回调：
 *            IDLE —— 空闲且未收满：一帧结束，用 size 取本段
 *            HT   —— 半满：没有 IDLE 也照样切段，防止连续流丢帧
 *            TC   —— 满：整个缓冲收满
 *          三种都在 Normal 模式下正确处理，且都要重启接收。
 *
 *          ============== ISR 只搬运，解析全部在任务里 ==============
 *
 *          协议识别（判断这是 ICOM / Kenwood / Yaesu 哪一家、BCD 怎么
 *          解、要不要切波段）逻辑不短，放在中断里会明显拉长中断时间，
 *          而 UART4_IRQn 的优先级虽然不是最高，但保护相关的中断都挂在
 *          更高优先级上，拉长它等于挤占保护链路的响应余量。
 *
 *          所以这里的分工是：
 *            中断 —— 把本段字节 memcpy 进 .bss 的交接缓冲，切换 DMA
 *                    的乒乓目标并重启接收，释放一个信号量。仅此而已，
 *                    没有解析、没有浮点、没有阻塞。
 *            任务 —— 等信号量，取走交接缓冲的内容，交给 radio_band
 *                    切帧 / 识别 / 换波段。
 *
 *          交接缓冲是单槽的：CAT 帧率极低（电台每 100 ms 最多回一帧），
 *          任务 10 ms 一次足以排空，单槽不会丢。若真出现积压，
 *          stats.frame_dropped 会记下来，而不是无声覆盖。
 *
 * @note ioc 里 UART4 的配置（用户已在 CubeMX 配好）：
 *          UART4       PC10 = TX, PC11 = RX, 115200 8N1
 *          UART4_IRQn  抢占优先级 5
 *          UART4_RX    DMA2_Stream7, **DMA_NORMAL**, Byte, Low
 *
 * @warning 需要在 CubeMX 里重新 Generate Code，让 dma.h 产生
 *          hdma_usart4_rx 句柄；否则 radio_uart_init() 会因
 *          huart4.hdmarx == NULL 而返回 false。
 *
 * @author  Control Agent (wt-control branch)
 */

#ifndef RADIO_UART_H
#define RADIO_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "radio_band.h"

/*!< 单个缓冲的大小。CAT 帧最长 17 字节（ICOM CIV），32 字节留足余量。 */
#define RADIO_UART_BUF_SIZE     32U

/*!< 启动 UART4 的双缓冲 DMA + IDLE 接收，并向 radio_band 注册收发钩子。
 *   必须在 HAL_UART_Init() / MX_DMA_Init() 之后调用。
 *   返回 false 表示启动失败（DMA 句柄缺失，或首次 ReceiveToIdle 报错）。 */
bool radio_uart_init(void);

/*!< 停止接收。可重复调用。 */
void radio_uart_deinit(void);

/*!< radio_band 的发送钩子：阻塞发送一帧 CAT 查询。
 *   已注册给 radio_band，一般不直接调用。 */
void radio_uart_tx(const uint8_t *data, uint16_t len);

/*!< radio_band 的接收钩子：把一段字节喂给协议层。
 *   已注册给 radio_band，一般不直接调用。 */
void radio_uart_rx(const uint8_t *data, uint16_t len);

/*============================================================================
 *  ISR -> 任务 的交接（协议探测走这条路）
 *==========================================================================*/

/*! 等待"有一帧待解析"。任务上下文调用。
 *
 *  radio_uart_init() 之后必须有一个任务常驻调用它，否则 CAT 数据会在
 *  交接缓冲里堆积直到 frame_dropped 递增。返回 true 表示交接缓冲里有
 *  新数据，已经被取走并喂给协议层；返回 false 表示超时（正常情况，
 *  电台本来就只在被轮询时才回帧）。
 *
 *  典型用法：
 *      for (;;) {
 *          radio_uart_wait_frame();      // 阻塞直到有数据
 *          radio_band_tick();            // 累加器满 100 ms 则发查询
 *      }
 *
 *  @param timeout_ms  最长等待时间。给 0 表示只做一次非阻塞检查，
 *                     用于"还要兼顾别的周期任务"的场景。
 *  @return true = 取到并处理了一帧；false = 超时无数据
 */
bool radio_uart_wait_frame(uint32_t timeout_ms);

/*! 清空交接缓冲里未处理的数据。radio_uart_init() 会自动调用。 */
void radio_uart_flush_pending(void);

/*============================================================================
 *  HAL 回调
 *==========================================================================*/

/* UART4 的 IDLE / HT / TC 事件统一入口。huart 必须是 &huart4。
 *
 * 注意这里不叫 HAL_UARTEx_RxEventCallback：HAL 里那是个全局唯一的弱符号，
 * 一个工程只能有一处强定义。comm.c 提供了那个唯一入口，按实例分发 ——
 * LPUART1 自己处理，其余（即 UART4）转到这里。两个文件各定义一个同名
 * 强符号会直接链接失败（multiple definition）。 */
void radio_uart_rx_event(UART_HandleTypeDef *huart, uint16_t size);

/* DMA 错误（搬运失败 / FIFO error）。全工程只有这一处定义，可以直接
 * 沿用 HAL 的弱符号名。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

/* 诊断计数。 */
typedef struct {
    uint32_t idle_events;   /*!< IDLE 中断次数                */
    uint32_t ht_events;     /*!< 半传输中断次数               */
    uint32_t tc_events;     /*!< 全传输中断次数               */
    uint32_t restarts;      /*!< 重启 ReceiveToIdle 次数      */
    uint32_t restart_fail;  /*!< 重启失败次数（应恒为 0）      */
    uint32_t uart_errors;   /*!< UART 错误中断次数            */
    uint32_t dup_idle;      /*!< 无新字节的重复 IDLE 次数     */
    uint32_t handoffs;      /*!< ISR 成功交给任务的次数        */
    uint32_t frame_dropped; /*!< 任务没排空就被覆盖的次数      */
} RadioUartStatsType;

void radio_uart_get_stats(RadioUartStatsType *out);

#ifdef __cplusplus
}
#endif

#endif /* RADIO_UART_H */