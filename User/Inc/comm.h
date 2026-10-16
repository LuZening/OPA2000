/**
 * @file comm.h
 * @brief LPUART1 / USB CDC PC CAT 协议（行式 ASCII，\r\n 终止）
 *
 * @details
 *  命令不区分大小写。三类动词：
 *    VAR=VAL    设置变量，立即生效
 *    VAR?       查询变量
 *    命令        动作：psu on/off, hv on/off, bias on/off, status, help, ...
 *
 *  comm 任务负责：
 *    comm_process_rx()       —— 排空 RX 队列，按行解析与执行
 *    comm_periodic_tick()    —— 100ms 主动上报一行 "status ..."
 *
 *  中断/任务分工：
 *    HAL_UARTEx_RxEventCallback（强定义在 comm.c）：
 *      LPUART1 -> comm_lpuart_rx_callback -> 入队
 *      UART4   -> radio_uart_rx_event
 *
 * 兼容旧 API（comm_execute_command / comm_get_radio_* /
 * comm_calibrate_*），新代码应该用 line 解析入口。
 */

#ifndef COMM_H
#define COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "opa2000.h"
#include <stdbool.h>

/*============================================================================
 *  Communication Configuration
 *============================================================================*/

#define COMM_RX_QUEUE_SIZE     256
#define COMM_TX_QUEUE_SIZE     256

/* Command buffer size - 单行最大长度，含结尾 \0 */
#define COMM_CMD_MAX_LEN       64

/*============================================================================
 *  Coupler Calibration (legacy - 兼容老 GUI/PC 入口)
 *============================================================================*/

typedef enum {
    COUPLER_CAL_IDLE = 0,
    COUPLER_CAL_START,
    COUPLER_CAL_STAGE1_5W_FWD,
    COUPLER_CAL_STAGE1_5W_REF,
    COUPLER_CAL_STAGE2_20W_FWD,
    COUPLER_CAL_STAGE2_20W_REF,
    COUPLER_CAL_COMPLETE,
    COUPLER_CAL_ABORTED
} CouplerCalState;

typedef enum {
    COUPLER_1 = 0,
    COUPLER_2,
    COUPLER_COUNT
} CouplerId;

typedef struct {
    float stage1_fwd_adc;
    float stage1_ref_adc;
    float stage2_fwd_adc;
    float stage2_ref_adc;
} CouplerCalData;

typedef struct {
    uint8_t data[COMM_RX_QUEUE_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} CommQueueType;

/*============================================================================
 *  Public API
 *============================================================================*/

/* 初始化与启动 */
void comm_init(void);
void comm_start(void);

/* 任务循环调用：处理已收到的字节，按行解析执行 */
void comm_process_rx(void);

/* 任务循环调用：每 10ms 调一次，100ms 主动发一行 "status ..." */
void comm_periodic_tick(void);

/* 任务循环调用：从 s_tx_ring 搬数据到 DMA 缓冲并启动 LPUART1 TX。
 * 若 DMA 还在跑或 ring 空就什么都不做。HAL_UART_TxCpltCallback 负责
 * 释放 busy 标志，下一次 pump 自动接上。 */
void comm_tx_pump(void);

/* 直接执行一条已拼好的命令（兼容旧入口） */
void comm_execute_command(const char *cmd);

/* 兼容旧 API：发送响应（任意二进制） */
void comm_send_response(const uint8_t *data, uint16_t len);
void comm_send_string(const char *str);

/*============================================================================
 *  告警事件主动推送
 *
 *  comm_post_alert_evt() 供 ISR 或任何上下文调用，把"alert 状态
 *  变化"事件压入内部 ring。comm 任务在 comm_alert_pump() 里把
 *  事件格式化成 "alert=ON <code> <p0|p1>\r\n" 或 "alert=OFF\r\n"
 *  送进 s_tx_ring，由 comm_tx_pump() 经 LPUART1 DMA 发出。
 *
 *  on = true  : 推送 ON 事件（cause/severity 必须有效）
 *  on = false : 推送 OFF 事件（cause/severity 忽略，输出 alert=OFF）
 *
 *  ring 满则丢最新的，记 g_alert_evt_overrun 计数。
 *==========================================================================*/
void comm_post_alert_evt(AlertType cause, AlertSeverity sev, bool on);

/* 任务循环调用：把已入队的事件 pump 进 s_tx_ring。
 * 替代 comm_periodic_tick() 的位置，每 10ms 调一次。 */
void comm_alert_pump(void);

/* 诊断：事件被覆盖的次数（ring 满） */
uint32_t comm_get_alert_evt_overrun(void);

/* 查询当前 radio 状态 */
uint32_t comm_get_radio_frequency(void);
RadioType comm_get_radio_type(void);

/* 字节入队（中断侧）。命令解析已用 ring buffer 模式，
 * 此函数保留给老调用方——新代码应直接调 comm_lpuart_rx_callback。 */
void comm_queue_rx_byte(uint8_t byte);
bool     comm_get_queued_byte(uint8_t *byte);

/* LPUART1 IDLE 回调入口（HAL 强定义按实例转发到这里） */
void comm_lpuart_rx_callback(uint16_t size);

/* 兼容：把整段字节喂给 radio 协议层 */
void comm_process_radio_data(const uint8_t *data, uint16_t len);

/* 兼容：耦合器标定（已废弃但保留符号） */
bool               comm_calibrate_coupler_start(CouplerId coupler_id);
bool               comm_calibrate_coupler_continue(void);
void               comm_calibrate_coupler_abort(void);
CouplerCalState    comm_calibrate_get_state(void);

#ifdef __cplusplus
}
#endif

#endif /* COMM_H */
