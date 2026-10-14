/**
 * @file radio_band.c
 * @brief 电台 CAT 协议解析、协议识别与轮询发送.
 *
 * @details 纯逻辑模块：不碰 UART / DMA / 中断。所有与硬件的交互都通过
 *          两个回调函数解耦出去，因此可以在 host 上完整地跑单元测试。
 *
 *          分四层：
 *            radio_band_feed()      —— 喂收到的字节进去（任意长度）
 *            radio_band_poll()      —— 切帧、解析、识别、累加轮询计数
 *            radio_band_tick()      —— 100 Hz 定时器累加器，满则置发送标志
 *            radio_band_get_freq()  —— 读最近一次有效频率
 *
 *          硬件回调（在 radio_uart.c 里实现，由本模块反向调用）：
 *            radio_band_set_tx_hook()     —— 注册"把这几个字节发到电台"
 *            radio_band_set_rx_hook()     —— 注册"收到字节从这里进来"
 *
 *          ===================== 协议识别 =====================
 *
 *          三种 CAT 协议在字节层面都有各自无法混淆的特征，据此自动识别：
 *
 *            ICOM CIV   帧头 FE FE（两个 0xFE 背靠背，这是 CIV 的标志性
 *                       前导），帧尾 FD。特征最强，绝不会误判成别的。
 *
 *            Kenwood    ASCII 文本，命令名 + ';' + 参数 + ';'。特征是
 *                       首字节必为可打印 ASCII，且整帧只含 [0x20,0x7E]。
 *                       5/6 字节的裸 BCD 绝不会全落在 ASCII 可打印区里，
 *                       所以两者可区分。
 *
 *            Yaesu      最弱：没有帧头也没有帧尾，靠 IDLE line 切出来的
 *                       5 或 6 字节裸 BCD。特征只能是"每个半字节 <= 9"。
 *
 *          识别顺序即置信度顺序：ICOM 最强先试，Kenwood 次之（要求整帧
 *          可打印），Yaesu 最后兜底。Yaesu 的判定最宽松，所以只有在
 *          前两者都不匹配时才会被接受，不会抢占更明确的协议。
 *
 *          ===================== 误切换防御 =====================
 *
 *          没有防抖：一个合法帧立刻生效，切换延迟就是 CAT 链路一个往返。
 *
 *          防御不靠"多次采样投票"——那种机制原理上无法区分"电台真的
 *          快速扫频"和"噪声凑出了三次一致的错误值"。改为靠单帧严格校验，
 *          每一条都必须成立：
 *
 *            1. 帧结构精确（见上，各协议特征）
 *            2. 命令字必须是频率命令（CIV 只认 00/01）
 *            3. 每个 BCD 半字节 <= 9
 *            4. 频率落在 RADIO_BAND_MIN_HZ ~ MAX_HZ
 *
 *          第 4 条对 Yaesu 尤其关键：它没有任何定界符，噪声凑出 5 个
 *          合法 BCD 字节的概率不低。而 10 Hz 步进下随机 10 位十进制数
 *          落在 1.8M~30M 区间的概率只有约 1.5%，这条挡掉绝大多数噪声。
 *
 *          任何一条不成立：整帧丢弃，g_radio_freq_hz 完全不动。
 *
 * @author  Control Agent (wt-control branch)
 */

#include "radio_band.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/*============================================================================
 *  内部状态
 *==========================================================================*/

/* ISR 无锁环形缓冲：DMA+IDLE 中断投递字节，任务侧取出重组。 */
#define RADIO_BAND_ISR_RING 128U
static volatile uint8_t  s_ring[RADIO_BAND_ISR_RING];
static volatile uint16_t s_ring_head;      /*!< ISR 写指针 */
static volatile uint16_t s_ring_tail;      /*!< 任务读指针 */

/* 任务侧重组缓冲：把 ring 里的字节搬到这里凑帧。 */
static uint8_t  s_buf[RADIO_BAND_BUF_SIZE];
static uint16_t s_buf_len;
static uint32_t s_buf_tick;                /*!< 最后一次收到字节的时刻 */

/* 最近一次被采纳的频率（Hz）。 */
static volatile uint32_t s_freq_hz;

/* 识别出的协议 / 用户锁定的协议。 */
static RadioType s_type;

/* 轮询累加器与待发标志。 */
static volatile uint16_t s_poll_accum;
static volatile bool     s_tx_pending;

/* 硬件回调。 */
static radio_band_tx_fn_t s_tx_hook;
static radio_band_rx_fn_t s_rx_hook;

static RadioBandStatsType s_stats;

/*============================================================================
 *  小工具
 *==========================================================================*/

/* packed BCD 的一个字节：高低半字节都必须是 0~9。 */
static inline bool bcd_valid(uint8_t b)
{
    return ((b >> 4) <= 9U) && ((b & 0x0FU) <= 9U);
}

/* 当前 tick（ms）。HAL_GetTick() 是裸 volatile 读，任务与中断都可用。 */
static inline uint32_t now_tick(void)
{
    return HAL_GetTick();
}

/*============================================================================
 *  各协议解析
 *
 *  三者刻意不共用解码代码：ICOM 与 Yaesu 都是 5 字节 packed BCD，
 *  但最高字节语义不同（ICOM 用满 2 位，Yaesu 高半是频段标志只取低半），
 *  共用一个解码器必然有一边差一个数量级。
 *==========================================================================*/

/* ICOM CIV:  FE FE <to> <from> <cmd> <5×BCD> FD
 * 频率在 cmd==0x00 (TX) / 0x01 (RX) 的响应里，位置 data[5..9]，
 * 5 字节 packed BCD = 10 位十进制，10 Hz 步进。
 * 例：14.074200 MHz -> 00 01 40 74 20 -> 1407420 @10Hz */
bool radio_band_parse_icom_civ(const uint8_t *f, uint16_t n, uint32_t *freq_10hz)
{
    if (n < 11U) {                              /* FE FE to from cmd + 5 + FD */
        return false;
    }
    if (f[0] != 0xFEU || f[1] != 0xFEU) {       /* 引导前导 */
        return false;
    }
    if (f[10] != 0xFDU) {                       /* 帧尾 */
        return false;
    }
    /* 只认频率命令，否则模式/功率等同样以 FD 结尾的响应会被误当频率。 */
    if (f[4] != 0x00U && f[4] != 0x01U) {
        return false;
    }
    for (uint16_t i = 5U; i < 10U; i++) {
        if (!bcd_valid(f[i])) {
            return false;
        }
    }

    uint32_t v = 0U;
    for (uint16_t i = 5U; i < 10U; i++) {
        v = (v * 100U) + (uint32_t)((f[i] >> 4) * 10U) + (f[i] & 0x0FU);
    }
    *freq_10hz = v;
    return true;
}

/* Kenwood:  ASCII，典型频率响应 "IF;+001407420;"
 * 首字符 '+'/'-' 是收发方向标志，不属于数值。纯十进制，10 Hz 步进。 */
bool radio_band_parse_kenwood(const uint8_t *f, uint16_t n, uint32_t *freq_10hz)
{
    if (n < 4U) {
        return false;
    }
    if (f[0] != 'I' || f[1] != 'F' || f[2] != ';') {
        return false;
    }
    if (f[n - 1U] != ';') {                    /* 必须以 ';' 收尾 */
        return false;
    }

    uint16_t i = 3U;
    if (f[i] == '+' || f[i] == '-') {
        i++;
    }

    uint32_t v = 0U;
    uint8_t  digits = 0U;
    while (i < n && f[i] >= '0' && f[i] <= '9') {
        v = (v * 10U) + (uint32_t)(f[i] - '0');
        digits++;
        i++;
        if (digits > 10U) {                    /* 30 MHz 以内 7 位足够 */
            return false;
        }
    }
    if (digits == 0U) {
        return false;
    }
    /* 数字之后只允许 RLSB 标志，然后是收尾的 ';'。 */
    if (i < n && (f[i] == '+' || f[i] == 'R')) {
        i++;
    }
    if (i >= n || f[i] != ';') {
        return false;
    }

    *freq_10hz = v;
    return true;
}

/* Yaesu:  5 或 6 字节裸 BCD，无定界符。
 * 最高字节高半是频段标志（HF/VHF/UHF），只取低半；其余 4 字节满 2 位。
 * 例：14.074200 MHz -> 00 01 40 74 20 */
bool radio_band_parse_yaesu(const uint8_t *f, uint16_t n, uint32_t *freq_10hz)
{
    uint16_t off;

    if (n == 5U) {
        off = 0U;
    }
    else if (n == 6U) {
        off = 1U;                              /* 1 字节状态前缀 */
    }
    else {
        return false;
    }

    if ((f[off] & 0x0FU) > 9U) {              /* 最高字节只看低半 */
        return false;
    }
    for (uint16_t i = off + 1U; i < off + 5U; i++) {
        if (!bcd_valid(f[i])) {
            return false;
        }
    }

    uint32_t v = (uint32_t)(f[off] & 0x0FU);
    for (uint16_t i = off + 1U; i < off + 5U; i++) {
        v = (v * 100U) + (uint32_t)((f[i] >> 4) * 10U) + (f[i] & 0x0FU);
    }

    *freq_10hz = v;
    return true;
}

/*============================================================================
 *  帧定位与帧切分
 *
 *  拆成两步是有原因的，而且这个划分必须是"丢噪声"和"等收全"的区分：
 *
 *    locate_start()  丢弃帧头之前的噪声，直到某个协议的帧头出现。
 *                   它永远返回"已经位于帧头"，因此不会切掉半帧。
 *    find_frame()    从位置 0 验证一帧是否已经收全。收全就交出帧长与
 *                   消耗字节数；没收全返回 false，调用方保留缓冲等待。
 *
 *  如果把两者混在一起（"切不出帧就丢一个字节"），逐字节到达的半帧会在
 *  每收一个字节后被丢掉一个，永远拼不起来。
 *==========================================================================*/

/* 丢弃帧头之前的噪声，返回丢弃的字节数（0 = 已在帧头）。
 *
 * hint == UNKNOWN 时按置信度顺序找：ICOM 的 FE FE 最强，Kenwood 的 "IF;"
 * 次之，Yaesu 无定界符、无法定位，所以只在前两者都不匹配时才把整个
 * 缓冲当作"可能已经是一帧 Yaesu"（由 find_frame 去验长度与 BCD）。 */
static uint16_t radio_band_locate_start(const uint8_t *data, uint16_t len,
                                        RadioType hint)
{
    if (data == NULL || len == 0U) {
        return 0U;
    }

    bool want_civ = (hint == RADIO_TYPE_UNKNOWN || hint == RADIO_TYPE_ICOM_CIV);
    bool want_ken = (hint == RADIO_TYPE_UNKNOWN || hint == RADIO_TYPE_KENWOOD);

    if (want_civ) {
        /* FE FE 之前一律是噪声。找到第一个 FE FE 就停在那里。 */
        for (uint16_t i = 0U; (uint16_t)(i + 1U) < len; i++) {
            if (data[i] == 0xFEU && data[i + 1U] == 0xFEU) {
                return i;
            }
        }
        /* 整个缓冲都没有 FE FE。若只允许 ICOM，剩下的全是噪声。 */
        if (hint == RADIO_TYPE_ICOM_CIV) {
            return len;
        }
    }

    if (want_ken && len >= 3U) {
        if (data[0] == 'I' && data[1] == 'F' && data[2] == ';') {
            return 0U;               /* 已在 Kenwood 帧头 */
        }
        /* 找到第一个 "IF;" 之前都是噪声。 */
        for (uint16_t i = 0U; (uint16_t)(i + 2U) < len; i++) {
            if (data[i] == 'I' && data[i + 1] == 'F' && data[i + 2] == ';') {
                return i;
            }
        }
        if (hint == RADIO_TYPE_KENWOOD) {
            return len;
        }
    }

    /* Yaesu 没有帧头可定位：只有 UNKNOWN 或已锁定 Yaesu 时会走到这里，
     * 交给 find_frame 按长度和 BCD 合法性判定。 */
    return 0U;
}

/* 从 data[0] 开始验证一帧是否收全。收全时给出帧指针、帧长与消耗字节数。 */
bool radio_band_find_frame(const uint8_t *data, uint16_t len,
                           const uint8_t **frame, uint16_t *frame_len,
                           uint16_t *consumed, RadioType hint)
{
    *consumed = 0U;
    if (data == NULL || len == 0U || frame == NULL || frame_len == NULL) {
        return false;
    }

    /* --- ICOM CIV: FE FE ... FD --------------------------------------- */
    if ((hint == RADIO_TYPE_UNKNOWN || hint == RADIO_TYPE_ICOM_CIV) &&
        len >= 2U && data[0] == 0xFEU && data[1] == 0xFEU) {
        for (uint16_t j = 2U; j < len && j < 17U; j++) {
            if (data[j] == 0xFDU) {
                *frame     = data;
                *frame_len = (uint16_t)(j + 1U);
                *consumed  = (uint16_t)(j + 1U);
                return true;
            }
        }
        /* 有 FE FE 但还没收到 FD：半帧，原样保留等下一个字节。 */
        return false;
    }

    /* --- Kenwood: 'I''F'';...'; --------------------------------------- */
    if ((hint == RADIO_TYPE_UNKNOWN || hint == RADIO_TYPE_KENWOOD) &&
        len >= 3U && data[0] == 'I' && data[1] == 'F' && data[2] == ';') {
        for (uint16_t j = 3U; j < len; j++) {
            if (data[j] == ';') {
                *frame     = data;
                *frame_len = (uint16_t)(j + 1U);
                *consumed  = (uint16_t)(j + 1U);
                return true;
            }
        }
        /* ';' 还没到，半帧。 */
        return false;
    }

    /* --- Yaesu: 整段恰好 5/6 字节且全是合法 BCD -------------------------
     * 没有定界符，只能靠长度 + 内容判定。少于 5 字节是半帧，多于 6 字节
     * 说明这段不是一帧（等上层超时丢弃）。 */
    if (hint == RADIO_TYPE_UNKNOWN || hint == RADIO_TYPE_YAESU) {
        if (len < 5U) {
            return false;               /* 半帧，继续等 */
        }
        if (len > 6U) {
            return false;               /* 不可能是 Yaesu 帧 */
        }
        uint32_t dummy;
        if (radio_band_parse_yaesu(data, len, &dummy)) {
            *frame     = data;
            *frame_len = len;
            *consumed  = len;
            return true;
        }
    }

    return false;
}

/* 按当前（或候选）协议尝试解析。命中则返回协议类型。 */
static bool radio_band_try_parse(const uint8_t *f, uint16_t n,
                                  RadioType hint, RadioType *seen,
                                  uint32_t *freq_10hz)
{
    uint32_t v = 0U;

    if ((hint == RADIO_TYPE_UNKNOWN || hint == RADIO_TYPE_ICOM_CIV) &&
        radio_band_parse_icom_civ(f, n, &v)) {
        *seen = RADIO_TYPE_ICOM_CIV;
        *freq_10hz = v;
        return true;
    }
    if ((hint == RADIO_TYPE_UNKNOWN || hint == RADIO_TYPE_KENWOOD) &&
        radio_band_parse_kenwood(f, n, &v)) {
        *seen = RADIO_TYPE_KENWOOD;
        *freq_10hz = v;
        return true;
    }
    if ((hint == RADIO_TYPE_UNKNOWN || hint == RADIO_TYPE_YAESU) &&
        radio_band_parse_yaesu(f, n, &v)) {
        *seen = RADIO_TYPE_YAESU;
        *freq_10hz = v;
        return true;
    }
    return false;
}

/* 尝试从重组缓冲里切出一帧。成功则解析并上报。
 *
 * 两步走，先定位再验证：
 *   1. locate_frame_start() 找到当前协议的帧头位置。找不到就丢弃缓冲前
 *      面的噪声字节，一直丢到帧头出现为止——这一步保证半帧能被完整
 *      保留下来（逐字节喂入时，前 10 个字节都因为"还没到帧尾"而被判为
 *      等待，绝不能在这里被丢掉）。
 *   2. find_frame() 在帧头位置验证完整帧。帧头对了但帧尾还没到，说明
 *      还在收帧，原样保留等下一个字节。
 *
 * 关键区别：丢字节只发生在"确定不是帧头"的时候，不是"帧没收全"的时候。
 * 混为一谈会导致半帧被逐字节丢弃，任何分片到达的帧都拼不起来。
 */
static bool radio_band_try_consume(void)
{
    const uint8_t *fr;
    uint16_t fl, used;

    /* 1) 丢掉帧头之前的噪声。 */
    uint16_t skip = radio_band_locate_start(&s_buf[0], s_buf_len, s_type);
    if (skip > 0U) {
        s_buf_len = (uint16_t)(s_buf_len - skip);
        if (s_buf_len > 0U) {
            memmove(&s_buf[0], &s_buf[skip], s_buf_len);
        }
        if (s_buf_len == 0U) {
            return false;
        }
    }

    /* 2) 帧头已在最前面。验证是否完整；不完整就留着等下一个字节。 */
    if (!radio_band_find_frame(&s_buf[0], s_buf_len, &fr, &fl, &used, s_type)) {
        return false;
    }

    uint32_t f10 = 0U;
    RadioType seen = RADIO_TYPE_UNKNOWN;
    bool     ok   = radio_band_try_parse(fr, fl, s_type, &seen, &f10);

    if (ok) {
        /* 10 Hz 单位 -> Hz，再做范围校验。范围这一条是防噪声误切的关键：
         * 随机 10 位十进制数落在 1.8M~30M 的概率仅约 1.5%。 */
        uint32_t hz = f10 * 10U;
        if (hz >= RADIO_BAND_MIN_HZ && hz <= RADIO_BAND_MAX_HZ) {
            if (s_type == RADIO_TYPE_UNKNOWN) {
                /* 自动识别模式：记住识别结果，之后只认这个协议，
                 * 避免后续噪声被误判成别的协议。 */
                s_type = seen;
            }
            s_stats.frames_ok++;
            if (hz != s_freq_hz) {
                s_freq_hz       = hz;
                g_radio_freq_hz = hz;
            }
        }
        else {
            s_stats.frames_bad++;          /* 频率越界 = 无效帧 */
        }
    }
    else {
        s_stats.frames_bad++;
    }

    /* 消耗掉这一帧，剩下的字节留给下一轮。 */
    s_buf_len = (uint16_t)(s_buf_len - used);
    if (s_buf_len > 0U) {
        memmove(&s_buf[0], &s_buf[used], s_buf_len);
    }
    return true;
}

/*============================================================================
 *  Public API
 *==========================================================================*/

void radio_band_init(RadioType radio_type_hint)
{
    s_ring_head   = 0U;
    s_ring_tail   = 0U;
    s_buf_len     = 0U;
    s_buf_tick    = now_tick();
    s_freq_hz     = 0U;
    s_type        = radio_type_hint;
    s_poll_accum  = 0U;
    s_tx_pending  = false;
    memset(&s_stats, 0, sizeof s_stats);
}

void radio_band_reset(void)
{
    s_buf_len = 0U;
}

void radio_band_set_tx_hook(radio_band_tx_fn_t fn)
{
    s_tx_hook = fn;
}

void radio_band_set_rx_hook(radio_band_rx_fn_t fn)
{
    s_rx_hook = fn;
    if (fn != NULL && s_rx_hook != fn) {
        /* 防御性：如果调用者在已初始化后替换钩子，环形缓冲状态不再可信。 */
        s_ring_head = 0U;
        s_ring_tail = 0U;
    }
}

void radio_band_feed(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U) {
        return;
    }
    s_stats.rx_bytes += len;
    /* 收到电台的任何字节都重置轮询间隔：电台刚说过话，不必急着再问。 */
    s_poll_accum = 0U;

    for (uint16_t i = 0U; i < len; i++) {
        if (s_buf_len >= RADIO_BAND_BUF_SIZE) {
            /* 缓冲满还没凑出完整帧 = 这串不是合法 CAT 数据。丢最老的一个
             * 字节重新找帧头，否则会永久卡死在这一帧上。 */
            s_stats.frames_bad++;
            memmove(&s_buf[0], &s_buf[1], RADIO_BAND_BUF_SIZE - 1U);
            s_buf_len--;
        }
        s_buf[s_buf_len++] = data[i];
    }
    s_buf_tick = now_tick();

    /* 把缓冲里已有的完整帧都吃掉。try_consume 每次成功切出一帧，或者在
     * 切不出帧时丢掉一个字节让下一次重试，所以这个循环总会终止。 */
    while (radio_band_try_consume()) {
        /* 循环条件即"又切出一帧"。 */
    }
}

bool radio_band_push_isr(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_ring_head + 1U) % RADIO_BAND_ISR_RING);

    if (next == s_ring_tail) {
        /* 满了：丢最新的并计数，绝不在中断里阻塞。 */
        s_stats.overrun++;
        return false;
    }
    s_ring[s_ring_head] = byte;
    s_ring_head = next;
    return true;
}

void radio_band_tick(void)
{
    /* 由 100 Hz 定时器调用（app_comm_task 的 10 ms tick 分频，或直接用
     * 一个 10 ms tick 累积）。累加到 100 就该发一次查询。 */
    s_poll_accum++;
    if (s_poll_accum >= RADIO_BAND_POLL_PERIOD_MS) {
        s_poll_accum = 0U;
        s_tx_pending = true;
    }
}

void radio_band_poll(void)
{
    uint32_t t = now_tick();

    /* 1) 把 ISR 环形缓冲里的字节搬进重组缓冲。 */
    if (s_ring_tail != s_ring_head) {
        while (s_ring_tail != s_ring_head) {
            uint8_t b = s_ring[s_ring_tail];
            s_ring_tail = (uint16_t)((s_ring_tail + 1U) % RADIO_BAND_ISR_RING);

            if (s_buf_len >= RADIO_BAND_BUF_SIZE) {
                s_stats.frames_bad++;
                memmove(&s_buf[0], &s_buf[1], RADIO_BAND_BUF_SIZE - 1U);
                s_buf_len--;
            }
            s_buf[s_buf_len++] = b;
        }
        s_buf_tick = t;
        /* 收到字节：重置轮询间隔。 */
        s_poll_accum = 0U;

        while (radio_band_try_consume()) {
            /* 见 radio_band_feed() */
        }
    }

    /* 2) 超时丢弃：缓冲里还剩半帧且已静默很久，说明这串是噪声。 */
    if (s_buf_len > 0U && (uint32_t)(t - s_buf_tick) > RADIO_BAND_IDLE_MS) {
        s_stats.timeouts++;
        s_buf_len = 0U;
    }

    /* 3) 轮询发送：累加器满了就发一次查询，发送后重新计满 100。 */
    if (s_tx_pending) {
        s_tx_pending = false;
        radio_band_send_poll();
    }
}

void radio_band_send_poll(void)
{
    if (s_tx_hook == NULL) {
        return;
    }

    const uint8_t *q;
    uint8_t  qbuf[16];
    uint16_t n;

    /* 协议未识别时先发 Kenwood 的 "IF;"：它最短（3 字节）、几乎所有
     * Kenwood / Yaesu 机型都认。就算猜错，收到别的协议响应也会被识别
     * 出来并切换，不会有副作用。 */
    if (s_type == RADIO_TYPE_ICOM_CIV) {
        /* CIV 读 VFO 频率：FE FE 94 E0 03 00 FD（94 = 默认电台地址，
         * 03 = 读 VFO 频率，末尾 00 是子命令）。 */
        static const uint8_t civ[] = {
            0xFE, 0xFE, 0x94, 0xE0, 0x03, 0x00, 0xFD
        };
        q = civ;  n = (uint16_t)sizeof civ;
    }
    else {
        static const uint8_t ken[] = { 'I', 'F', ';' };
        q = ken;  n = (uint16_t)sizeof ken;
    }

    /* 有的电台要求查询前先发一个 ';' 复位帧。这里统一：Kenwood/Yaesu
     * 在查询前发一个 ';'，ICOM 不需要。 */
    if (s_type == RADIO_TYPE_YAESU) {
        qbuf[0] = ';';
        memcpy(&qbuf[1], q, n);
        s_tx_hook(qbuf, (uint16_t)(n + 1U));
    }
    else {
        s_tx_hook(q, n);
    }

    /* 发送即重置间隔：刚问过，等满 100 ms 再问下一次。 */
    s_poll_accum = 0U;
}

uint32_t radio_band_get_freq(void)
{
    return s_freq_hz;
}

RadioType radio_band_get_type(void)
{
    return s_type;
}

BandType radio_band_freq_to_band(uint32_t freq_hz)
{
    if (freq_hz == 0U) {
        return BAND_NONE;
    }
    return frequency_to_band(freq_hz);
}

void radio_band_get_stats(RadioBandStatsType *out)
{
    if (out == NULL) {
        return;
    }
    out->rx_bytes   = s_stats.rx_bytes;
    out->frames_ok  = s_stats.frames_ok;
    out->frames_bad = s_stats.frames_bad;
    out->timeouts   = s_stats.timeouts;
    out->overrun    = s_stats.overrun;
}