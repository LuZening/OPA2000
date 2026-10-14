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
 *  HAL 回调（stm32h7xx_hal_uart.c 反向调用）
 *==========================================================================*/

/* IDLE / HT / TC 三种事件统一入口。huart 必须是 &huart4。 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);

/* DMA 错误（搬运失败 / FIFO error）。 */
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
} RadioUartStatsType;

void radio_uart_get_stats(RadioUartStatsType *out);

#ifdef __cplusplus
}
#endif

#endif /* RADIO_UART_H */