/**
 * @file    ns2009.h
 * @brief   NS2009 4-wire resistive touch controller driver.
 *
 * @details NS2009 is a low-cost I2C resistive touch panel ADC.  It exposes
 *          touch coordinates and contact pressure on a fixed 7-bit I2C
 *          address (0x48 with A0 tied low; 0x49 with A0 high - we default
 *          to 0x48 here).
 *
 *          Pin assignment on OPA2000 H7: NS2009 sits on I2C2
 *          (PF0/SDA, PF1/SCL), alongside the GT911.  The GT911 (CTOUCH,
 *          capacitive) answers at 7-bit 0x14 and this chip (RTOUCH,
 *          resistive) at 7-bit 0x48, so the two coexist on one bus.
 *
 *          本文件只保留"探测 + 命令码 + 解码宏"。坐标与压力读取已经搬到
 *          touch.c 的 DMA 状态机里：I2C2 的完成回调是全局唯一的
 *          （HAL_I2C_MemRxCpltCallback 只认句柄不认从机），两个控制器
 *          必须共用同一个状态机，否则无法区分是谁的回调。
 *
 *          ====================== 一个曾经踩过的坑 ======================
 *
 *          NS2009 是 I2C 器件，但它**不是寄存器映射器件**——每笔读事务的
 *          第一个字节是**命令码**，芯片据此配置 ADC 的 MUX 和驱动开关，
 *          然后返回 2 字节（12 位数据 + 4 位冗余）。一次事务只能拿一个值。
 *
 *          别把它当成 TSC2046 那种 SPI 器件、用 0x82/0x84 那套"寄存器地址"
 *          ——那是另一类器件的寻址方式，在 NS2009 上发出去得不到数据。
 *
 *          命令码取自 Linux 内核的 NS2009 驱动（Icenowy Zheng, 2017,
 *          drivers/input/touchscreen/ns2009.c，Lichee Pi Zero 在用）：
 *
 *              X  = 0xC0     Y  = 0xD0     Z1 = 0xE0     Z2 = 0xF0
 *
 *          ==========================================================
 */

#ifndef NS2009_H
#define NS2009_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Default I2C address (A0=0).  Set NS2009_ADDR to 0x49 before including
 * this file if the board ties A0 high. */
#ifndef NS2009_ADDR
#define NS2009_ADDR            (0x48U << 1)   /* HAL expects 8-bit addr */
#endif

/* I2C timeout for the boot-time probe.  Per project rule, every I2C op on
 * the touch probe path must be <= 100 ms; the chip is small and slow, but
 * not THAT slow.  If the I2C bus is hung, 100 ms is long enough to
 * prove the device is not there.  (The runtime read path does not use a
 * HAL timeout at all - it has a software watchdog in touch.c, because a
 * DMA transfer never times out on its own.) */
#define NS2009_I2C_TIMEOUT_MS  100U

/*============================================================================
 *  命令码（每笔读事务一个值，返回 2 字节）
 *==========================================================================*/
#define NS2009_CMD_X_12BIT     0xC0U
#define NS2009_CMD_Y_12BIT     0xD0U
#define NS2009_CMD_Z1_12BIT    0xE0U
#define NS2009_CMD_Z2_12BIT    0xF0U

/* 一次读事务返回的字节数：12 位数据 + 4 位冗余。 */
#define NS2009_READ_BYTES      2U

/*============================================================================
 *  12 位数据 + 4 位冗余的排布
 *
 *  数据手册 "Read Command" 一节的原文是 "the next 2 bytes is the 12bit
 *  from NS2009..., redundant 4bits zero"，排布如下：
 *
 *      byte 0            byte 1
 *   ┌─────────────┬────────────────────┐
 *   │ D11..D4     │ D3 D2 D1 D0 │ X X X X │
 *   └─────────────┴────────────────────┘
 *                             ↑           ↑
 *                       有效的低 4 位   **最后发来的 4 位**
 *
 *  也就是说 byte1 的高 4 位是 12 位结果的最后 4 位，**低 4 位是芯片保证
 *  为 0 的冗余位**——它们是 I2C 上最后到达的 4 个 bit。
 *
 *  这 4 位不是浪费：它们天然是一次免费的完整性校验。若非 0，说明这次
 *  转换的数据不可信（位错位、SDA 上的噪声、转换没完成）。Linux 内核
 *  驱动就是据此整次丢弃：
 *
 *      if (unlikely(raw_data[1] & 0xf))
 *              return -EINVAL;
 *      *val = (raw_data[0] << 4) | (raw_data[1] >> 4);
 *
 *  我们在 touch.c 里做同样的检查。
 *==========================================================================*/

/** @brief 12 位值：byte0 = 高 8 位，byte1 = 高 4 位是低 4 位数据。 */
#define NS2009_VALUE(hi, lo) \
    ((uint16_t)(((uint16_t)(hi) << 4) | ((uint16_t)(lo) >> 4)))

/** @brief 冗余 4 位是否非 0（非 0 = 本次读无效）。 */
#define NS2009_IS_VALID(lo)    (((lo) & 0x0FU) == 0U)

/*!< Full-scale of the 12-bit ADC, used by the Z formula below. */
#define NS2009_ADC_MAX               4095U

/**
 * @brief 接触压力——手册方法一。
 *
 * 电阻屏测的是两个电压：Z1 取自屏板（受压后上升），Z2 取自触摸点
 * （接触后下降）。单个都不单调，但两者之差是：
 *
 *      Z = |(4095 - Z2) + Z1|
 *
 * 这与手册 "Pressure Measurement" 一节的方法一一致（该方法还要 X 位置，
 * 才能进一步换算成 R_touch）；LLMC_DAC 的 HR2046 驱动
 * （Core/Src/touch_HR2046.c）用的也是同一个表达式。
 *
 * @note 中间量用 int32_t：(4095 - Z2) + Z1 的绝对值可达 8190，且中间
 *       结果可能为负。写在一个更窄的类型里会溢出或回绕。
 */
#define NS2009_PRESSURE(z1, z2)                                     \
    ((uint32_t)((int32_t)((int32_t)NS2009_ADC_MAX - (int32_t)(z2))  \
                + (int32_t)(z1)))

/**
 * @brief 压力超过此值即判定为"按下"。
 *
 * 400 来自 LLMC_DAC 的 HR2046 路径（touch_HR2046.c:
 * `*pZ = (ADC_Z > 400) ? 1 : 0`）。两者压力公式与 12 位量程一致，所以
 * 这个数可以直接沿用。
 *
 * 为什么按压力而不是按坐标判按下：空闲的电阻屏会漂移，X/Y 随温度、涂层
 * 老化与附近噪声游走，"未触摸"时并没有一个可预测的读数——任何 X/Y 阈值都
 * 是对这块板特定漂移的猜测。压力才是真正对应**接触**的量，无接触时回到
 * 稳定基线。这也是原来那个"两轴都低于 100"判据形状不对的原因。
 *
 * 这仍是**起步值，不是通用参数**：真实数值与面板电阻、驱动电流、玻璃
 * 厚度相关。上板后先读 g_touch_raw_z，看它空闲时的基线与轻触时的值，
 * 再决定要不要改——要改就只改这一行。
 */
#define NS2009_Z_PRESS_THRESHOLD    400U

/*============================================================================
 *  API
 *============================================================================*/

/** Probe the I2C bus for an NS2009.  Returns true on ACK, false on
 *  NACK or timeout.  Safe to call from boot, single-threaded, before
 *  any FreeRTOS task starts. */
bool ns2009_probe(void);

#ifdef __cplusplus
}
#endif

#endif /* NS2009_H */
