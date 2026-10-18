/**
 * @file comm.c
 * @brief LPUART1 / USB CDC PC CAT 协议 (TaikoSensor 风格)
 *
 * @details
 *  行式 ASCII 协议，\r\n 终止。命令不区分大小写。三类动词：
 *    VAR=VAL          设置变量，立即生效
 *    VAR?             查询变量
 *    命令 (no = no ?) 动作：psu on / psu off / hv on / hv off / bias on /
 *                      bias off / status / help / save / reset / reboot
 *
 *  ===================== 硬件 =====================
 *
 *  LPUART1 在 D3 域，DMA 是 BDMA2；缓冲只能落在 .d3_dma_buffer（SRAM4）。
 *
 *  RX：双缓冲 ping-pong + IDLE line 中断。跟 radio_uart.c 的设计一致——
 *  DMA 写 A，事件触发后 memcpy 交给 comm_dispatch_line 解析，再切 B 启
 *  DMA。避免"在中断里重启 DMA"导致的空窗丢帧。
 *
 *  TX：单缓冲 + DMA + TC 中断 + 任务泵。comm_send_bytes() 永远是把
 *  数据丢进 tx_ring；任务在 comm_tx_pump() 里把 ring 里的数据搬到
 *  s_tx_dma_buf 并启动 HAL_UART_Transmit_DMA()。完成后从 ring 拉一段
 *  继续——直到 ring 空。半双工 CAT 协议是命令/响应式，ring 不会积压。
 *
 *  USB CDC 仍是 blocking 短消息（数十字节），不影响实时性。
 *
 *  ===================== 主动上报 =====================
 *
 *  comm_periodic_tick() 每 100ms 调一次，格式化一行 status... 入
 *  tx_ring，PC 串口调试助手可直接看到仪表读数。100ms 频率由 comm 任务
 *  在 10ms tick 上累加。
 */

#include "comm.h"
#include "config.h"
#include "user_adc.h"
#include "power_control.h"
#include "protection.h"
#include "radio_band.h"
#include "radio_uart.h"
#include "opa2000.h"
#include "freq_detect.h"
#include "flash_eeprom.h"
#include "string.h"
#include "stdio.h"
#include <stdlib.h>
#include "main.h"
#include "usart.h"

/*============================================================================
 *  RX 双缓冲
 *
 *  64 字节比任何 CAT 帧（最长 "MCU_READY..." 的状态行 ~90）都长，
 *  但这里说的是"用户键入的命令"，'help' 一行约 230 字节才走第二段。
 *  取 96 字节：装得下所有已知命令，留余量给未来。32 字节对齐。
 *==========================================================================*/
#define LPUART_RX_BUFFER_SIZE   96U
#if defined(__GNUC__)
#define LPUART_BUF_SECTION  __attribute__((section(".d3_dma_buffer"), aligned(32)))
#else
#define LPUART_BUF_SECTION
#endif

static uint8_t s_rx_a[LPUART_RX_BUFFER_SIZE] LPUART_BUF_SECTION;
static uint8_t s_rx_b[LPUART_RX_BUFFER_SIZE] LPUART_BUF_SECTION;

/* 谁是 DMA 目标，谁是待读。0 = A 收 / B 待读；1 = B 收 / A 待读。 */
static volatile uint8_t  s_rx_active;
static volatile uint16_t s_rx_active_len;
static volatile bool     s_rx_started;

/* 交接给任务用的缓冲。comm_rx_event() 在中断里 memcpy，任务在
 * comm_process_rx() 里取走。任务从 s_rx_pending 看到数据并清 s_rx_pending_full，
 * 期间中断仍在搬运"另一段"——双缓冲的关键。 */
static uint8_t  s_rx_pending[LPUART_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_pending_len;
static volatile bool     s_rx_pending_full;
static volatile uint32_t s_rx_overrun;

/*============================================================================
 *  TX 队列
 *
 *  命令响应（短）和主动 status（中等长度）共用一个 ring。任务在
 *  comm_tx_pump() 里从 ring 搬到 s_tx_dma_buf 并启动 DMA。
 *
 *  满策略：新数据覆盖最旧的——CAT 是实时仪表，最新状态最有用，
 *  旧数据被覆盖不致命。计 s_tx_overrun 让诊断能看到。
 *==========================================================================*/

#define LPUART_TX_DMA_SIZE   256U
static uint8_t s_tx_dma_buf[LPUART_TX_DMA_SIZE] LPUART_BUF_SECTION;

#define LPUART_TX_RING_SIZE  512U
static uint8_t  s_tx_ring[LPUART_TX_RING_SIZE];
static volatile uint16_t s_tx_head;       /*!< 写指针（任务） */
static volatile uint16_t s_tx_tail;       /*!< 读指针（任务） */
static volatile bool     s_tx_dma_busy;
static volatile uint32_t s_tx_overrun;
static volatile bool     s_tx_started;

/*============================================================================
 *  Alert 事件 ring
 *
 *  与 s_tx_ring 分离：s_tx_ring 装"持续性输出"（status 100ms 一行），
 *  这里装"瞬时事件"（告警触发/解除）。事件在 ISR 上下文入队、任务
 *  pump 到 s_tx_ring，永远先于持续输出抵达 PC 端。
 *
 *  容量 8：P0 跳保护后立即会清，中断里多次重复触发在 s_protection_active
 *  上去重，所以 ring 满只在连发不同 P0/P1 时出现。满则覆盖最新的，
 *  丢 ON 比丢 OFF 严重（操作员应看到新保护），所以 ON 覆盖 OFF、OFF
 *  覆盖 ON，按"事件紧急度"排序。
 *==========================================================================*/
typedef struct {
    AlertType     cause;
    AlertSeverity sev;
    bool          on;        /*!< true = ON, false = OFF */
} AlertEvent;

#define COMM_ALERT_EVT_RING_SIZE   8U
static volatile AlertEvent  s_alert_evt_ring[COMM_ALERT_EVT_RING_SIZE];
static volatile uint8_t      s_alert_evt_head;
static volatile uint8_t      s_alert_evt_tail;
static volatile uint32_t     s_alert_evt_overrun;

/*============================================================================
 *  解析层（协议）
 *============================================================================*/

static char     comm_cmd_buffer[COMM_CMD_MAX_LEN];
static uint16_t comm_cmd_index   = 0U;

/* 主动上报节拍。100ms 一次。 */
static uint16_t comm_tx_period_counter = 0U;
#define COMM_TX_PERIOD_TICKS    10U   /* 调用 10ms tick，10 次 = 100ms */

/*============================================================================
 *  响应输出：USB CDC + LPUART1 TX 队列
 *============================================================================*/

static void comm_send_bytes(const char *data, uint16_t len)
{
    if (data == NULL || len == 0U) {
        return;
    }
    /* USB CDC：失败静默 */
//    (void)CDC_Transmit_HS((uint8_t *)data, len);

    /* LPUART1 TX ring：满则覆盖（牺牲老数据，保留最新的）。 */
#ifndef LVGL_SIM
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (uint16_t)((s_tx_head + 1U) % LPUART_TX_RING_SIZE);
        if (next == s_tx_tail) {
            s_tx_overrun++;
            /* 覆盖最旧：tail 前进一格 */
            s_tx_tail = (uint16_t)((s_tx_tail + 1U) % LPUART_TX_RING_SIZE);
        }
        s_tx_ring[s_tx_head] = (uint8_t)data[i];
        s_tx_head = next;
    }
#else
    (void)s_tx_ring; (void)s_tx_head; (void)s_tx_tail;
#endif
}

static void comm_send_line(const char *line)
{
    uint16_t len = (uint16_t)strlen(line);
    if (len + 2U >= (uint16_t)COMM_TX_QUEUE_SIZE) {
        len = (uint16_t)(COMM_TX_QUEUE_SIZE - 3U);
    }
    char buf[COMM_TX_QUEUE_SIZE];
    memcpy(buf, line, len);
    buf[len++] = '\r';
    buf[len++] = '\n';
    comm_send_bytes(buf, len);
}

/* TaikoSensor 风格的几个响应原语。 */
static void comm_send_ok(void)         { comm_send_line("OK"); }
static void comm_send_err(const char *msg)
{
    char buf[128];
    snprintf(buf, sizeof buf, "ERROR: %s", msg);
    comm_send_line(buf);
}

/*============================================================================
 * 解析 / 执行
 *============================================================================*/

/* 找 = 或 ? 的位置；前者表示 SET，后者表示 QUERY。都没有就是 ACTION。
 * 返回值：'S' = SET, 'Q' = QUERY, 'A' = ACTION.
 * 变量名 / 值 / 命令部分全部小写化后写回 cmd_lower，方便后续匹配。 */
static char comm_classify(const char *in, char *var_name, size_t var_cap,
                          const char **value, char *out_lower, size_t out_cap)
{
    if (in == NULL || in[0] == '\0') {
        return 'A';
    }
    /* Trim leading whitespace. */
    while (*in == ' ' || *in == '\t') {
        in++;
    }
    /* Lowercase into out_lower so later matchers don't have to. */
    size_t i = 0;
    while (in[i] != '\0' && i + 1U < out_cap) {
        char c = in[i];
        if (c >= 'A' && c <= 'Z') {
            c = (char)(c - 'A' + 'a');
        }
        out_lower[i] = c;
        i++;
    }
    out_lower[i] = '\0';

    /* Find '=' or '?'. */
    const char *eq = strchr(out_lower, '=');
    const char *qu = strchr(out_lower, '?');
    const char *sp = strchr(out_lower, ' ');

    /* Whichever comes first. */
    const char *first = NULL;
    char type = 'A';
    if (eq != NULL && (first == NULL || eq < first)) { first = eq; type = 'S'; }
    if (qu != NULL && (first == NULL || qu < first)) { first = qu; type = 'Q'; }
    if (sp != NULL && (first == NULL || sp < first)) { first = sp; type = 'A'; }
    if (first == NULL) {
        first = out_lower + strlen(out_lower);
    }

    size_t name_len = (size_t)(first - out_lower);
    if (name_len >= var_cap) {
        name_len = var_cap - 1U;
    }
    memcpy(var_name, out_lower, name_len);
    var_name[name_len] = '\0';

    if (type == 'S') {
        *value = first + 1;
        /* Skip leading whitespace in value. */
        while (**value == ' ' || **value == '\t') {
            (*value)++;
        }
    }
    return type;
}

/*============================================================================
 * 响应器：把"当前状态"格式化为一行可读文本
 *============================================================================*/

static const char *comm_power_state_str(void)
{
    if (g_alert_flag) {
        return "ALERT";
    }
    if (g_tx_active) {
        return "TX";
    }
    if (g_hv_on) {
        return "STBY";
    }
    return "OFF";
}

static const char *comm_band_str(BandType b)
{
    switch (b) {
    case BAND_160M:     return "160";
    case BAND_80M:      return "80";
    case BAND_40M_30M:  return "40";
    case BAND_20M_17M:  return "20";
    case BAND_15M_10M:  return "15";
    case BAND_NONE:     return "0";
    case BAND_FAULT:    return "FAULT";
    }
    return "?";
}

static void comm_format_status_line(char *buf, size_t cap)
{
    /* All of these arrive as integer engineering units (mW, x100, mV, mA,
     * tenths of a degree).  Scale them by hand into the fixed-point the
     * report format uses so printf never sees a float.  Power, voltage and
     * temperature report whole units; SWR and current carry one decimal. */
    int32_t fwd_w   = ((int32_t)g_adc_data.fwd2_mW + 500) / 1000;   /* whole W */
    int32_t rev_w   = ((int32_t)g_adc_data.rev2_mW + 500) / 1000;
    int32_t swr_d   = (int32_t)g_adc_data.swr_x100 / 10;           /* 0.1 */
    int32_t v       = ((int32_t)g_adc_data.vm_mV + 500) / 1000;     /* whole V */
    int32_t im_ca   = (int32_t)g_adc_data.im_mA;                    /* 0.01 A */
    int32_t temp_t  = ((int32_t)g_adc_data.ntc_core_tenth_c + 5) / 10;
    uint32_t freq   = g_radio_freq_hz;

    /* ptt: 1 = keyed, 0 = idle. freq_detect_task 在 PTT 时把 g_tx_active 置
     * true；这里直接读它。 */
    uint8_t ptt = g_tx_active ? 1U : 0U;
    uint8_t hv  = g_hv_on ? 1U : 0U;
    uint8_t psu = g_psu_on ? 1U : 0U;
    /* fan= 字段：mode|status|ntc_fault
     *   mode: 'A' = AUTO, 'M' = MANUAL
     *   status: 0/1/2 = LOW/MED/HIGH  (与现有 status banner 复用)
     *   ntc_fault: 1 = 传感器开路/短路，0 = 正常
     * 用 '|' 分隔避免与现有 ',' 风格冲突。 */
    char fan_buf[8];
    snprintf(fan_buf, sizeof fan_buf, "%c|%u|%u",
             (power_get_fan_mode() == FAN_MODE_AUTO) ? 'A' : 'M',
             (unsigned)g_fan_speed,
             (unsigned)(power_fan_ntc_fault() ? 1U : 0U));

    snprintf(buf, cap,
        "status pwr=%s band=%s freq=%lu fwd=%ld rev=%ld "
        "swr=%ld.%ld v=%ld im=%ld.%02ld temp=%ld "
        "ptt=%u hv=%u psu=%u fan=%s alert=%d",
        comm_power_state_str(),
        comm_band_str(g_current_band),
        (unsigned long)freq,
        (long)fwd_w, (long)rev_w,
        (long)(swr_d / 10), (long)((swr_d < 0 ? -swr_d : swr_d) % 10),
        (long)v,
        (long)(im_ca / 100), (long)((im_ca < 0 ? -im_ca : im_ca) % 100),
        (long)temp_t,
        (unsigned)ptt, (unsigned)hv, (unsigned)psu,
        fan_buf,
        (int)g_alert_active);
}

/*============================================================================
 *  SET handlers
 *============================================================================*/

static void comm_set_var(const char *name, const char *value)
{
    if (name == NULL || value == NULL) {
        comm_send_err("bad set");
        return;
    }

    /* 语言 */
    if (strcmp(name, "lang") == 0) {
        if (strcmp(value, "chinese") == 0 || strcmp(value, "zh") == 0) {
            g_config.language = MULTILANG_CHINESE;
        } else if (strcmp(value, "english") == 0 || strcmp(value, "en") == 0) {
            g_config.language = MULTILANG_ENGLISH;
        } else {
            comm_send_err("lang must be chinese|english|zh|en");
            return;
        }
        (void)config_save();
        comm_send_ok();
        return;
    }

    /* Radio 波特率（PC 端可改，但生效由 comm 任务下次循环完成）。
     * 这里只更新 g_config，不直接调 radio_uart_set_baud——后者
     * 不在中断可调用，comm_set_var 也是任务上下文，安全，但显式
     * 让 task pump 统一处理能让日志/状态显示更顺。 */
    if (strcmp(name, "radio_baud") == 0 || strcmp(name, "radiobaud") == 0) {
        uint32_t baud = (uint32_t)strtoul(value, NULL, 10);
        if (baud == 0U || baud > 921600U) {
            comm_send_err("radio_baud must be 300..921600");
            return;
        }
        g_config.radio_baud_rate = baud;
        comm_send_ok();
        return;
    }

    /* 保护阈值（直接写 g_config，不主动保存到 flash——PC 用 save 命令控） */
    if (strcmp(name, "swr_max") == 0) {
        uint32_t v = (uint32_t)strtoul(value, NULL, 10);
        if (v < 100U || v > 999U) { comm_send_err("swr_max 100..999"); return; }
        g_config.band_swr_max_x100 = (uint16_t)v;
        comm_send_ok();
        return;
    }
    if (strcmp(name, "rev2_max") == 0) {
        uint32_t v = (uint32_t)strtoul(value, NULL, 10);
        if (v < 1U || v > 1000U) { comm_send_err("rev2_max 1..1000 W"); return; }
        g_config.band_rev2_max_w = (uint16_t)v;
        comm_send_ok();
        return;
    }
    if (strcmp(name, "rev1_max") == 0) {
        uint32_t v = (uint32_t)strtoul(value, NULL, 10);
        if (v < 1U || v > 1000U) { comm_send_err("rev1_max 1..1000 W"); return; }
        g_config.band_rev1_max_w = (uint16_t)v;
        comm_send_ok();
        return;
    }
    if (strcmp(name, "im_max") == 0) {
        uint32_t v = (uint32_t)strtoul(value, NULL, 10);
        if (v < 1U || v > 50000U) { comm_send_err("im_max 1..50000 mA"); return; }
        g_config.band_im_max_mA = (uint16_t)v;
        comm_send_ok();
        return;
    }
    if (strcmp(name, "vbias_max") == 0) {
        uint32_t v = (uint32_t)strtoul(value, NULL, 10);
        if (v < 100U || v > 5000U) { comm_send_err("vbias_max 100..5000 mV"); return; }
        g_config.band_vbias_max_mV = (uint16_t)v;
        comm_send_ok();
        return;
    }
    if (strcmp(name, "ntc_max") == 0) {
        uint32_t v = (uint32_t)strtoul(value, NULL, 10);
        if (v < 100U || v > 1500U) { comm_send_err("ntc_max 100..1500 (0.1 C)"); return; }
        g_config.band_ntc_max_tenth_c = (uint16_t)v;
        comm_send_ok();
        return;
    }
    if (strcmp(name, "fwd1_max") == 0) {
        uint32_t v = (uint32_t)strtoul(value, NULL, 10);
        if (v < 1U || v > 2000U) { comm_send_err("fwd1_max 1..2000 W"); return; }
        g_config.band_fwd1_max_w = (uint16_t)v;
        comm_send_ok();
        return;
    }
    if (strcmp(name, "fwd2_max") == 0) {
        uint32_t v = (uint32_t)strtoul(value, NULL, 10);
        if (v < 1U || v > 2000U) { comm_send_err("fwd2_max 1..2000 W"); return; }
        g_config.band_fwd2_max_w = (uint16_t)v;
        comm_send_ok();
        return;
    }

    comm_send_err("unknown variable");
}

/*============================================================================
 *  QUERY handlers
 *============================================================================*/

static void comm_query_var(const char *name)
{
    char buf[96];
    if (strcmp(name, "lang") == 0) {
        comm_send_line(g_config.language == MULTILANG_CHINESE ? "lang=chinese" : "lang=english");
        return;
    }
    if (strcmp(name, "radio_baud") == 0 || strcmp(name, "radiobaud") == 0) {
        snprintf(buf, sizeof buf, "radio_baud=%lu", (unsigned long)g_config.radio_baud_rate);
        comm_send_line(buf);
        return;
    }
    if (strcmp(name, "swr_max") == 0) {
        snprintf(buf, sizeof buf, "swr_max=%u", (unsigned)g_config.band_swr_max_x100);
        comm_send_line(buf);
        return;
    }
    if (strcmp(name, "rev2_max") == 0) {
        snprintf(buf, sizeof buf, "rev2_max=%u", (unsigned)g_config.band_rev2_max_w);
        comm_send_line(buf);
        return;
    }
    if (strcmp(name, "rev1_max") == 0) {
        snprintf(buf, sizeof buf, "rev1_max=%u", (unsigned)g_config.band_rev1_max_w);
        comm_send_line(buf);
        return;
    }
    if (strcmp(name, "im_max") == 0) {
        snprintf(buf, sizeof buf, "im_max=%u", (unsigned)g_config.band_im_max_mA);
        comm_send_line(buf);
        return;
    }
    if (strcmp(name, "vbias_max") == 0) {
        snprintf(buf, sizeof buf, "vbias_max=%u", (unsigned)g_config.band_vbias_max_mV);
        comm_send_line(buf);
        return;
    }
    if (strcmp(name, "ntc_max") == 0) {
        snprintf(buf, sizeof buf, "ntc_max=%u", (unsigned)g_config.band_ntc_max_tenth_c);
        comm_send_line(buf);
        return;
    }
    if (strcmp(name, "fwd1_max") == 0) {
        snprintf(buf, sizeof buf, "fwd1_max=%u", (unsigned)g_config.band_fwd1_max_w);
        comm_send_line(buf);
        return;
    }
    if (strcmp(name, "fwd2_max") == 0) {
        snprintf(buf, sizeof buf, "fwd2_max=%u", (unsigned)g_config.band_fwd2_max_w);
        comm_send_line(buf);
        return;
    }
    if (strcmp(name, "band") == 0) {
        comm_send_line("band=auto");
        return;
    }
    if (strcmp(name, "freq") == 0) {
        snprintf(buf, sizeof buf, "freq=%lu", (unsigned long)g_radio_freq_hz);
        comm_send_line(buf);
        return;
    }
    if (strcmp(name, "protocol") == 0 || strcmp(name, "radio_type") == 0) {
        RadioType t = radio_band_get_type();
        const char *s = "auto";
        switch (t) {
        case RADIO_TYPE_ICOM_CIV: s = "icom"; break;
        case RADIO_TYPE_YAESU:    s = "yaesu"; break;
        case RADIO_TYPE_KENWOOD:  s = "kenwood"; break;
        case RADIO_TYPE_UNKNOWN:  s = "auto"; break;
        }
        snprintf(buf, sizeof buf, "protocol=%s", s);
        comm_send_line(buf);
        return;
    }
    if (strcmp(name, "config") == 0) {
        comm_format_status_line(buf, sizeof buf);
        comm_send_line(buf);
        return;
    }
    comm_send_err("unknown variable");
}

/*============================================================================
 *  ACTION handlers
 *============================================================================*/

static void comm_action_status(void)
{
    char buf[256];
    comm_format_status_line(buf, sizeof buf);
    comm_send_line(buf);
}

static void comm_action_help(void)
{
    /* 完整命令清单。每行 ≤ COMM_TX_QUEUE_SIZE-2 (= 254)，超过会被
     * comm_send_line() 截断——所以 help 行的设计宽度保守在 ≤70 字符。 */
    static const char *help_lines[] = {
        "=== OPA2000 PC CAT ===",
        "All commands case-insensitive, line-terminated by \\r or \\n.",
        "Response: each line ends with \\r\\n.",
        "",
        "-- Query (name?) --",
        "  status         one line: pwr band freq fwd rev swr v im temp ptt hv psu alert",
        "  config         alias of status",
        "  freq           last valid radio frequency in Hz",
        "  band / protocol   auto|icom|yaesu|kenwood",
        "  lang           chinese|english",
        "  radio_baud     current CAT baud in bps",
        "  swr_max / rev2_max / rev1_max / fwd1_max / fwd2_max",
        "  im_max / vbias_max / ntc_max",
        "",
        "-- Set (name=value) --",
        "  lang=chinese|english|zh|en      (auto-saves to flash)",
        "  radio_baud=300..921600          (effective on next 10 ms tick)",
        "  swr_max=100..999                (x100, e.g. 250 = 2.50)",
        "  rev2_max / rev1_max / fwd1_max / fwd2_max = 1..2000  (W)",
        "  im_max=1..50000                 (mA)",
        "  vbias_max=100..5000             (mV)",
        "  ntc_max=100..1500               (0.1 C)",
        "  NB: SET only updates RAM. Use 'save' to persist.",
        "",
        "-- Action --",
        "  psu on|off        main supply latch (PG5)",
        "  hv  on|off        high-voltage rail (PWR_ON + RF_BIAS)",
        "  bias on|off       alias of hv",
        "  alert off         clear latched fault, re-arm RF",
        "  save              persist g_config to flash",
        "  reset             restore factory defaults (RAM only)",
        "  reboot            NVIC system reset",
        "  help / h / ?      this text",
        "",
        "-- Unsolicited output (no command needed) --",
        "  Every 100 ms:    one 'status' line",
        "  On alert:        'alert=ON <code> p0|p1'  (within 10 ms)",
        "  On dismiss:      'alert=OFF <code>'",
        "  All event lines also end with \\r\\n.",
        "",
        "-- Examples --",
        "  status?       ->  status pwr=STBY band=20 freq=14074000 ... alert=0",
        "  psu off       ->  OK",
        "  alert off     ->  OK",
        "  radio_baud=19200  ->  OK",
        "  xyz           ->  ERROR: unknown variable",
    };
    for (size_t i = 0; i < sizeof(help_lines) / sizeof(help_lines[0]); i++) {
        comm_send_line(help_lines[i]);
    }
}

static void comm_action_psu(bool on)
{
    if (on) {
        power_psu_on();
        comm_send_ok();
    } else {
        power_psu_off();
        comm_send_ok();
    }
}

static void comm_action_hv(bool on)
{
    if (on) {
        power_hv_enable();
        comm_send_ok();
    } else {
        power_hv_disable();
        comm_send_ok();
    }
}

static void comm_action_alert_off(void)
{
    protection_clear();
    comm_send_ok();
}

static void comm_action_save(void)
{
    if (config_save()) {
        comm_send_ok();
    } else {
        comm_send_err("save failed");
    }
}

static void comm_action_reset_cfg(void)
{
    config_reset_default();
    comm_send_ok();
}

static void comm_action_reboot(void)
{
    /* 收到回车前先回 OK，否则对端看不到提示。 */
    comm_send_ok();
    /* 稍等把数据送出去。 */
    HAL_Delay(20U);
    NVIC_SystemReset();
}

static void comm_dispatch_action(char *lower, const char *first_token)
{
    /* lower = 已小写化的整行；first_token = lower 中第一个空格之前
     * 的子串（也是已小写化的）。 */
    if (first_token == NULL || first_token[0] == '\0') {
        comm_send_err("empty");
        return;
    }

    /* 单 token 命令 */
    if (strcmp(first_token, "status") == 0 || strcmp(lower, "status") == 0) {
        comm_action_status();
        return;
    }
    if (strcmp(first_token, "help") == 0 || strcmp(lower, "help") == 0 ||
        strcmp(first_token, "?") == 0 || strcmp(lower, "?") == 0 ||
        strcmp(first_token, "h") == 0 || strcmp(lower, "h") == 0) {
        comm_action_help();
        return;
    }
    if (strcmp(first_token, "save") == 0) { comm_action_save(); return; }
    if (strcmp(first_token, "reset") == 0) { comm_action_reset_cfg(); return; }
    if (strcmp(first_token, "reboot") == 0) { comm_action_reboot(); return; }
    if (strcmp(first_token, "alert") == 0) {
        const char *arg = strchr(lower, ' ');
        if (arg == NULL) {
            comm_send_err("alert needs arg");
            return;
        }
        arg++;
        while (*arg == ' ') arg++;
        if (strcmp(arg, "off") == 0) {
            comm_action_alert_off();
        } else {
            comm_send_err("alert only takes 'off'");
        }
        return;
    }

    /* 两 token: psu on / psu off / hv on / hv off / bias on / bias off */
    const char *arg = strchr(lower, ' ');
    if (arg == NULL) {
        char buf[64];
        snprintf(buf, sizeof buf, "unknown action '%s'", first_token);
        comm_send_err(buf);
        return;
    }
    arg++;
    while (*arg == ' ') arg++;
    bool on = (strcmp(arg, "on") == 0);
    bool off = (strcmp(arg, "off") == 0);
    if (!on && !off) {
        comm_send_err("on/off expected");
        return;
    }

    if (strcmp(first_token, "psu") == 0) { comm_action_psu(on); return; }
    if (strcmp(first_token, "hv") == 0)  { comm_action_hv(on);  return; }
    if (strcmp(first_token, "bias") == 0) { comm_action_hv(on); return; }

    char buf[64];
    snprintf(buf, sizeof buf, "unknown action '%s'", first_token);
    comm_send_err(buf);
}

/*============================================================================
 * 行分发：一条已收到的完整命令
 *============================================================================*/

static void comm_dispatch_line(const char *line)
{
    char lower[COMM_CMD_MAX_LEN];
    char name[64];
    const char *value = NULL;
    char type = comm_classify(line, name, sizeof name, &value, lower, sizeof lower);

    if (type == 'S') {
        comm_set_var(name, value);
        return;
    }
    if (type == 'Q') {
        comm_query_var(name);
        return;
    }
    /* ACTION: 取 first token（lower 字符串中第一个空白之前的部分） */
    const char *sp = strchr(lower, ' ');
    char first_token[32];
    const char *first = lower;
    if (sp != NULL) {
        size_t n = (size_t)(sp - lower);
        if (n >= sizeof first_token) n = sizeof first_token - 1U;
        memcpy(first_token, lower, n);
        first_token[n] = '\0';
        first = first_token;
    }
    comm_dispatch_action(lower, first);
}

/*============================================================================
 *  Alert 事件 ring：任意上下文入队 + 任务 pump
 *==========================================================================*/

/* 任意上下文可调（ISR / 任务 / 启动期）。只动 s_alert_evt_head + ring
 * 数组两项，都是 .bss 内存。
 *
 * 原子性：head / tail 的 +1 +模 不是原子的，极端并发下会丢 1~2 个事件。
 * 这没问题——status 每 100ms 还会带 alert=N，丢事件不会让 PC 看不到
 * 当前状态。
 *
 * 满策略：覆盖"最旧"那条，s_alert_evt_overrun 计数。覆盖时优先覆盖
 * OFF（让 ON 留下来）：PC 看到 alert=OFF 然后又来一个 alert=ON 比看到
 * 一次 alert=OFF 然后静默要好。 */
void comm_post_alert_evt(AlertType cause, AlertSeverity sev, bool on)
{
#ifndef LVGL_SIM
    uint8_t head = s_alert_evt_head;
    uint8_t next = (uint8_t)((head + 1U) % COMM_ALERT_EVT_RING_SIZE);
    if (next == s_alert_evt_tail) {
        /* 满：决定覆盖哪一条。如果队首是 OFF 而新事件是 ON，覆盖队首
         *（让 ON 留下来），否则丢新事件（旧 OFF 保留）。 */
        if (on && !s_alert_evt_ring[s_alert_evt_tail].on) {
            /* 推进 tail 一格覆盖旧 OFF */
            s_alert_evt_tail = (uint8_t)((s_alert_evt_tail + 1U) % COMM_ALERT_EVT_RING_SIZE);
            s_alert_evt_overrun++;
        }
        else {
            s_alert_evt_overrun++;
            return;
        }
    }
    s_alert_evt_ring[head].cause = cause;
    s_alert_evt_ring[head].sev   = sev;
    s_alert_evt_ring[head].on    = on;
    s_alert_evt_head = next;
#else
    (void)cause; (void)sev; (void)on;
#endif
}

/* 把 ring 里的事件格式化成一行 alert=... 塞进 s_tx_ring。满则丢
 * 整批——任务在 10ms 一次 pump 之间把事件消化掉，s_tx_ring 留够
 * 256B 容纳 2~3 条 alert 行 + 一条 status 行。 */
void comm_alert_pump(void)
{
#ifndef LVGL_SIM
    while (s_alert_evt_tail != s_alert_evt_head) {
        AlertEvent ev = s_alert_evt_ring[s_alert_evt_tail];
        s_alert_evt_tail = (uint8_t)((s_alert_evt_tail + 1U) % COMM_ALERT_EVT_RING_SIZE);

        char buf[48];
        int n;
        if (ev.on) {
            const char *sev_str = "?";
            switch (ev.sev) {
            case ALERT_SEV_P0: sev_str = "p0"; break;
            case ALERT_SEV_P1: sev_str = "p1"; break;
            default:           sev_str = "?";  break;
            }
            n = snprintf(buf, sizeof buf, "alert=ON %d %s", (int)ev.cause, sev_str);
        }
        else {
            /* OFF 也带 cause，让 PC 知道刚解的是哪个。
             * 0 表示"没记录故障就被 dismiss"的边界情况。 */
            n = snprintf(buf, sizeof buf, "alert=OFF %d", (int)ev.cause);
        }
        if (n <= 0) {
            continue;
        }
        if ((size_t)n >= sizeof buf) {
            n = (int)(sizeof buf - 1U);
        }
        /* 直接走 comm_send_bytes()，把数据加 \r\n 送入 s_tx_ring。 */
        char line[sizeof buf + 4];
        memcpy(line, buf, (size_t)n);
        line[n++] = '\r';
        line[n++] = '\n';
        comm_send_bytes(line, (uint16_t)n);
    }
#else
    (void)0;
#endif
}

uint32_t comm_get_alert_evt_overrun(void)
{
    return s_alert_evt_overrun;
}

/*============================================================================
 *  RX 内部
 *============================================================================*/

static uint8_t *rx_active_buf(void)
{
    return (s_rx_active == 0U) ? s_rx_a : s_rx_b;
}

/* 中断侧：把当前缓冲里已收到的 len 字节搬进交接缓冲，切段并重启 DMA。 */
static void comm_rx_flip_and_restart(uint16_t len)
{
    if (len > LPUART_RX_BUFFER_SIZE) {
        len = LPUART_RX_BUFFER_SIZE;
    }
    if (len > 0U) {
        if (s_rx_pending_full) {
            s_rx_overrun++;     /* 上次还没被取走就被覆盖 */
        }
        memcpy(s_rx_pending, rx_active_buf(), len);
        s_rx_pending_len  = len;
        s_rx_pending_full = true;
    }

    s_rx_active     = (s_rx_active == 0U) ? 1U : 0U;
    s_rx_active_len = 0U;

    if (HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, rx_active_buf(),
                                     LPUART_RX_BUFFER_SIZE) != HAL_OK) {
        s_rx_started = false;
    }
}

/* HAL 强定义：LPUART1 自己处理，UART4 转 radio_uart，其他忽略 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == LPUART1) {
        if (s_rx_started == false) {
            return;
        }
        /* 与 radio_uart.c 一致：三种事件都处理。Normal 模式下 DMA 已
         * 停下，s_rx_a / s_rx_b 的内容稳定可读。 */
        switch (huart->RxEventType) {
        case HAL_UART_RXEVENT_HT:
            comm_rx_flip_and_restart((uint16_t)(LPUART_RX_BUFFER_SIZE / 2U));
            break;
        case HAL_UART_RXEVENT_TC:
            comm_rx_flip_and_restart(LPUART_RX_BUFFER_SIZE);
            break;
        case HAL_UART_RXEVENT_IDLE:
            if (Size > s_rx_active_len) {
                s_rx_active_len = Size;
                comm_rx_flip_and_restart(Size);
            }
            else {
                /* HT 已经把前半段交走过 + DMA 已停：仅重启。 */
                (void)HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, rx_active_buf(),
                                                    LPUART_RX_BUFFER_SIZE);
            }
            break;
        default:
            comm_rx_flip_and_restart(LPUART_RX_BUFFER_SIZE);
            break;
        }
    }
    else {
        radio_uart_rx_event(huart, Size);
    }
}

/* 兼容旧 API：把这一帧的 size 给 comm（不实际用，留给老调用方） */
void comm_lpuart_rx_callback(uint16_t size)
{
    (void)size;   /* 已被 HAL 入口替代，留 stub 兼容老链接 */
}

/*============================================================================
 *  TX 内部
 *============================================================================*/

/* HAL 弱符号 TxCplt 的强定义。DMA 完成后清 busy + 标记下一段可发。 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == LPUART1) {
        s_tx_dma_busy = false;
    }
}

/* 从 s_tx_ring 搬运最长 n 字节到 s_tx_dma_buf，返回实际搬的字节数。
 * ring 为空返回 0。 */
static uint16_t comm_tx_ring_drain(uint8_t *dst, uint16_t cap)
{
    uint16_t n = 0U;
    while (n < cap && s_tx_tail != s_tx_head) {
        dst[n++] = s_tx_ring[s_tx_tail];
        s_tx_tail = (uint16_t)((s_tx_tail + 1U) % LPUART_TX_RING_SIZE);
    }
    return n;
}

/* 任务循环调用：若 ring 非空且 DMA 空闲，搬一段 + 启动 DMA。 */
void comm_tx_pump(void)
{
#ifndef LVGL_SIM
    if (s_tx_started == false) {
        return;
    }
    if (s_tx_dma_busy) {
        return;
    }
    if (s_tx_tail == s_tx_head) {
        return;     /* ring 空 */
    }
    uint16_t n = comm_tx_ring_drain(s_tx_dma_buf, LPUART_TX_DMA_SIZE);
    if (n == 0U) {
        return;
    }
    s_tx_dma_busy = true;
    if (HAL_UART_Transmit_DMA(&hlpuart1, s_tx_dma_buf, n) != HAL_OK) {
        /* 失败：清 busy，下一轮重试。 */
        s_tx_dma_busy = false;
    }
#endif
}

/*============================================================================
 * Public Functions
 *============================================================================*/

void comm_init(void)
{
    s_rx_active     = 0U;
    s_rx_active_len = 0U;
    s_rx_started    = false;
    s_rx_pending_full = false;
    s_rx_pending_len  = 0U;
    s_rx_overrun    = 0U;

    s_tx_head     = 0U;
    s_tx_tail     = 0U;
    s_tx_dma_busy = false;
    s_tx_overrun  = 0U;
    s_tx_started  = false;

    s_alert_evt_head     = 0U;
    s_alert_evt_tail     = 0U;
    s_alert_evt_overrun  = 0U;
    /* volatile 数组：逐项清零，避免 memset 报警告。 */
    for (uint8_t i = 0U; i < COMM_ALERT_EVT_RING_SIZE; i++) {
        s_alert_evt_ring[i].cause = ALERT_NONE;
        s_alert_evt_ring[i].sev   = ALERT_SEV_NONE;
        s_alert_evt_ring[i].on    = false;
    }

    comm_cmd_index = 0U;
    comm_tx_period_counter = 0U;
}

void comm_start(void)
{
    /* 启动 LPUART1 双缓冲 RX。Normal 模式，ReceiveToIdle 触发。 */
    memset(s_rx_a, 0, sizeof s_rx_a);
    memset(s_rx_b, 0, sizeof s_rx_b);
    s_rx_active     = 0U;
    s_rx_active_len = 0U;
    if (HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, s_rx_a,
                                     LPUART_RX_BUFFER_SIZE) == HAL_OK) {
        s_rx_started = true;
    }
    s_tx_started  = true;
}

void comm_process_rx(void)
{
    /* 把 s_rx_pending 里的字节按行解析。\r 或 \n 触发一行处理。
     * 这里只读 s_rx_pending_full + 长度，不与中断写同一缓冲，
     * 因为 comm_rx_flip_and_restart() 写的是"另一段" (rx_active_buf)，
     * 而 s_rx_pending 是 .bss 内存，与 DMA 无关。 */
    if (!s_rx_pending_full) {
        return;
    }
    /* 完整拷一份到栈上，避免长时间持有关键变量。s_rx_pending ≤ 96
     * 字节，栈上完全放得下。 */
    uint8_t  local[LPUART_RX_BUFFER_SIZE];
    uint16_t local_len = s_rx_pending_len;
    memcpy(local, s_rx_pending, local_len);
    s_rx_pending_full = false;
    s_rx_pending_len  = 0U;

    for (uint16_t i = 0; i < local_len; i++) {
        uint8_t byte = local[i];
        if (byte == '\n' || byte == '\r') {
            if (comm_cmd_index == 0U) {
                continue;
            }
            comm_cmd_buffer[comm_cmd_index] = '\0';
            comm_dispatch_line(comm_cmd_buffer);
            comm_cmd_index = 0U;
        }
        else {
            if (comm_cmd_index < (uint16_t)(COMM_CMD_MAX_LEN - 1U)) {
                comm_cmd_buffer[comm_cmd_index++] = (char)byte;
            }
            else {
                comm_cmd_index = 0U;
                comm_send_err("line too long");
            }
        }
    }
}

/* 兼容旧 API：直接执行一条已拼好的命令。 */
void comm_execute_command(const char *cmd)
{
    if (cmd == NULL) {
        return;
    }
    char buf[COMM_CMD_MAX_LEN];
    strncpy(buf, cmd, sizeof buf - 1U);
    buf[sizeof buf - 1U] = '\0';
    comm_dispatch_line(buf);
}

/* 100ms 主动上报。任务在 10ms tick 上调一次本函数即可。 */
void comm_periodic_tick(void)
{
    if (++comm_tx_period_counter >= COMM_TX_PERIOD_TICKS) {
        comm_tx_period_counter = 0U;
        char buf[256];
        comm_format_status_line(buf, sizeof buf);
        comm_send_line(buf);
    }
}

void comm_send_response(const uint8_t *data, uint16_t len)
{
    comm_send_bytes((const char *)data, len);
}

void comm_send_string(const char *str)
{
    comm_send_line(str);
}

uint32_t comm_get_radio_frequency(void)
{
    return g_radio_freq_hz;
}

RadioType comm_get_radio_type(void)
{
    return radio_band_get_type();
}

/* 旧 API：老调用方若仍想用字节入队的方式发命令，可借助
 * comm_execute_command() 直接执行。这里保留空实现以避免链接错误，
 * 但实际不做任何事——新的乒乓路径不再需要它。 */
void comm_queue_rx_byte(uint8_t byte)
{
    (void)byte;
}

bool comm_get_queued_byte(uint8_t *byte)
{
    (void)byte;
    return false;
}

/* 兼容旧 API：实际派发由 radio_band.c 自己做。 */
void comm_process_radio_data(const uint8_t *data, uint16_t len)
{
    radio_band_feed(data, len);
}

/*============================================================================
 *  Coupler calibration - 兼容老 GUI/PC 入口
 *============================================================================*/

static CouplerCalState coupler_cal_state = COUPLER_CAL_IDLE;
static CouplerId       coupler_cal_id     = COUPLER_1;
static CouplerCalData  coupler_cal_data;

bool comm_calibrate_coupler_start(CouplerId coupler_id)
{
    if (coupler_cal_state != COUPLER_CAL_IDLE) {
        comm_send_err("calibration already in progress");
        return false;
    }
    coupler_cal_id = coupler_id;
    memset(&coupler_cal_data, 0, sizeof coupler_cal_data);
    coupler_cal_state = COUPLER_CAL_START;
    comm_send_line("calibration: use cal coupler 0|1 + cal ok | cal abort");
    return true;
}

bool comm_calibrate_coupler_continue(void)
{
    /* 已废弃：耦合器标定已改为一次性两段记录 + 全局系数；保留 stub
     * 以避免破坏老 API 调用方。 */
    (void)coupler_cal_data;
    return false;
}

void comm_calibrate_coupler_abort(void)
{
    coupler_cal_state = COUPLER_CAL_IDLE;
}

CouplerCalState comm_calibrate_get_state(void)
{
    return coupler_cal_state;
}
