/**
 * @file radio_band.h
 * @brief 电台 CAT 协议识别、频率解析与轮询发送.
 *
 * @details 纯逻辑模块，不直接访问 UART / DMA / 中断，因此可在 host 上
 *          完整单元测试。与硬件的交互通过两个回调解耦：
 *
 *            radio_band_set_tx_hook()  注册"把这几个字节发到电台"
 *            radio_band_set_rx_hook()  注册"收到字节从这里进来"
 *
 *          典型用法（任务上下文）：
 *
 *            radio_band_init(g_config.radio_type);   // UNKNOWN = 自动识别
 *            radio_band_set_tx_hook(radio_uart_tx);  // radio_uart.c 提供
 *            radio_band_set_rx_hook(radio_uart_rx);
 *
 *            void comm_task_10ms(void) {
 *                radio_band_tick();    // 累加器，满 100 ms 置发送标志
 *                radio_band_poll();    // 收字节、切帧、解析、必要时发查询
 *            }
 *
 *          三种协议的识别特征（按置信度从强到弱）：
 *            ICOM CIV   FE FE 前导 + FD 帧尾 —— 最强，不会误判
 *            Kenwood    全 ASCII，首字节 'I'，以 ';' 收尾 —— 与裸 BCD 不相容
 *            Yaesu      5/6 字节裸 BCD，无定界符 —— 判定最宽松，兜底
 *
 * @warning 没有防抖：单个合法帧立即生效。这是刻意的，波段切换延迟就是
 *          CAT 链路一个往返。误切换的防御靠单帧严格校验（帧结构、命令字、
 *          BCD 半字节、频率范围四条），不靠多次采样投票。
 *
 * @author  Control Agent (wt-control branch)
 */

#ifndef RADIO_BAND_H
#define RADIO_BAND_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "opa2000.h"

/*============================================================================
 *  配置
 *==========================================================================*/

/*!< 内部重组缓冲区。CAT 帧最长 17 字节（ICOM CIV），留 32 字节足够。 */
#define RADIO_BAND_BUF_SIZE     32U

/*!< 半帧超时判据：超过这么久没有新字节，当前缓冲就是不完整帧，丢弃。
 *   115200 baud 下一个字节 8.7 us，17 字节帧约 150 us；100 ms 足够覆盖
 *   任何帧内间隔，又不会把"电台停顿一下才发下一帧"误判成丢帧。 */
#define RADIO_BAND_IDLE_MS      100U

/*!< 频率合法范围（Hz）。低于 1.8 MHz 或高于 30 MHz 一律判为无效帧。
 *   这条是防噪声误切波段的关键：10 Hz 步进下随机 10 位十进制数落在这个
 *   区间的概率只有约 1.5%。 */
#define RADIO_BAND_MIN_HZ       1800000UL
#define RADIO_BAND_MAX_HZ       30000000UL

/*!< 轮询间隔（tick 数）。与 radio_band_tick() 的调用频率配合：
 *   若挂在 10 ms tick 上则 20 = 200 ms；若挂在 1 ms tick 上则 200。
 *   CAT 查询每 200 ms 一次：足够频繁以保证频率显示与波段切换的
 *   响应延迟 < 0.5 s，又不至于把安静的电台淹没在轮询包里。 */
#define RADIO_BAND_POLL_PERIOD_MS   20U

/*============================================================================
 *  硬件回调
 *==========================================================================*/

/*!< 把 len 字节发到电台。任务上下文调用，可能阻塞（帧很短，10~20 字节）。 */
typedef void (*radio_band_tx_fn_t)(const uint8_t *data, uint16_t len);

/*!< 收到 len 字节。任务上下文调用。 */
typedef void (*radio_band_rx_fn_t)(const uint8_t *data, uint16_t len);

/*============================================================================
 *  公开 API
 *==========================================================================*/

/* 初始化。radio_type_hint 传 RADIO_TYPE_UNKNOWN 时自动识别，
 * 传具体类型则只认该协议。 */
void radio_band_init(RadioType radio_type_hint);

/* GUI/用户切换 radio_type 时调用：只改协议提示，不动已识别的状态。
 * 若已是自动识别出的协议，新 hint 若与之不一致则降回 UNKNOWN 让
 * 重新识别（避免把已经稳定的链路状态清空）。
 * 同时清空重组缓冲——切协议时旧协议的半帧没有意义。 */
void radio_band_set_hint(RadioType radio_type_hint);

/* 丢弃半帧缓冲（不动已识别的协议与频率）。 */
void radio_band_reset(void);

/* 注册硬件回调。必须在 radio_band_init() 之前或之后都可以，
 * 但 set_rx_hook 会重置 ISR 环形缓冲状态。 */
void radio_band_set_tx_hook(radio_band_tx_fn_t fn);
void radio_band_set_rx_hook(radio_band_rx_fn_t fn);

/* 喂入一段收到的字节。任务上下文调用；长度任意，可以是半帧。 */
void radio_band_feed(const uint8_t *data, uint16_t len);

/* ISR 入口：单字节入无锁环形缓冲。不做解析、不阻塞。
 * 满了返回 false 并计数，由上层决定是否报警。 */
bool radio_band_push_isr(uint8_t byte);

/* 定时器累加器。每次调用 +1，满 RADIO_BAND_POLL_PERIOD_MS 则置发送标志。
 * 发送或收到字节都会把累加器清零，所以"每满 100 ms 一次"是相对于
 * 最近一次收发活动而言的。 */
void radio_band_tick(void);

/* 处理：搬 ISR 缓冲 -> 切帧 -> 解析 -> 超时丢弃 -> 必要时发查询。
 * 任务上下文调用，建议 10 ms tick。 */
void radio_band_poll(void);

/* 立即发一次协议查询（不等累加器）。手动切波段时可用来立刻刷新。 */
void radio_band_send_poll(void);

/* 最近一次有效频率（Hz）；从未收到有效帧时为 0。 */
uint32_t radio_band_get_freq(void);

/* 当前协议（可能是自动识别出来的，也可能是用户锁定的）。 */
RadioType radio_band_get_type(void);

/* 该频率属于哪个波段；0 或越界返回 BAND_NONE。 */
BandType radio_band_freq_to_band(uint32_t freq_hz);

/* 诊断计数，不参与控制。 */
typedef struct {
    uint32_t rx_bytes;
    uint32_t frames_ok;
    uint32_t frames_bad;
    uint32_t timeouts;
    uint32_t overrun;         /*!< ISR 环形缓冲写满次数   */
    uint32_t frames_held_tx;  /*!< TX 期间收到的合法帧，
                                    协议解析通过但暂不更新频率 */
} RadioBandStatsType;

void radio_band_get_stats(RadioBandStatsType *out);

/*============================================================================
 *  纯函数（暴露以便单元测试与离线调试）
 *==========================================================================*/

/* 在 data 里找第一帧。found 时给出帧指针、帧长、消耗字节数；
 * consumed 为 0 表示还没凑齐。hint 传 UNKNOWN 表示全部协议都试。 */
bool radio_band_find_frame(const uint8_t *data, uint16_t len,
                           const uint8_t **frame, uint16_t *frame_len,
                           uint16_t *consumed, RadioType hint);

/* 按指定协议解析一帧频率。频率以 10 Hz 为单位返回（CAT 原生单位）。 */
bool radio_band_parse_icom_civ(const uint8_t *f, uint16_t n, uint32_t *freq_10hz);
bool radio_band_parse_kenwood(const uint8_t *f, uint16_t n, uint32_t *freq_10hz);
bool radio_band_parse_yaesu(const uint8_t *f, uint16_t n, uint32_t *freq_10hz);

#ifdef __cplusplus
}
#endif

#endif /* RADIO_BAND_H */