/**
 * @file power_control.h
 * @brief Power / Relay / Band / Fan / Buzzer control header.
 *
 * @details This header is the Control agent's primary interface to the
 *          hardware outputs that put the amplifier into / out of TX.
 *
 *          Layering:
 *            - All GPIO definitions live in User/Inc/hw_pinmap.h
 *            - power_control.c uses macros from hw_pinmap.h
 *              (LPF_ACTIVE(), RELAY1_ON, PSU_ON_PWR_ON, ...)
 *            - The PTT state machine runs *without* any FreeRTOS API
 *              calls in the ISR (HAL_GPIO_EXTI_Callback).  The task
 *              side reads the volatile flags only.
 *
 *          Fan / pump TACH detection runs in TIM7 0.5 ms tick; see
 *          fan_tach.c.
 *
 *          Buzzer 3-tone warning is implemented as a state machine
 *          driven by TIM7 ISR (no DMA); see buzzer.c (todo).
 *
 * @version 1.0
 * @date    2026-10
 * @author  Control Agent (wt-control branch)
 */

#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "opa2000.h"
#include "hw_pinmap.h"

/*============================================================================
 *  PTT state machine - written from EXTI5 ISR, read from task
 *============================================================================*/
typedef enum {
    PTT_STATE_IDLE = 0,
    PTT_STATE_TX_STARTING,    /* RELAY2 closed, waiting to close RELAY1 */
    PTT_STATE_TX_RUNNING,     /* both RELAY1+2 closed, transmitting */
    PTT_STATE_TX_STOPPING,    /* RELAY1 open, waiting to open RELAY2 */
} PTTStateType;

extern volatile PTTStateType ptt_state;
extern volatile int          ptt_event_flag;  /*!< -1=none, 0=release, 1=press */

/*============================================================================
 *  Fan / pump control
 *============================================================================*/

/* Initialise fan PWM (TIM3_CH3) and pump PWM (TIM4_CH4) at 10 kHz. */
void power_init_fan_pwm(void);
/* Set fan duty 0..100 % (smooth start/stop allowed). */
void power_set_fan_speed(uint8_t speed_percent);
/* Set pump duty 0..100 %. */
void power_set_pump_speed(uint8_t speed_percent);
/* Update fan based on temperature + hysteresis (50/80/100 %). */
void power_update_fan(int16_t temperature_tenth_c);

/*============================================================================
 *  电源与射频通路
 *
 *  ---------------------------------------------------------------------
 *  先读这一段。三条独立的轨，各有各的函数名，不要用错。
 *
 *  【PG5 PSU】  外部电源设备，MCU 只控制它的开关
 *      power_psu_on() / power_psu_off()        单拍，无时序。高=开、低=关
 *      唯一由固件主动发起的是 power_soft_off()（关机时关掉）；
 *      power_init() 不调，PSU 主要由用户自己操作。远程控制是骨架，协议待定。
 *
 *  【PG7 + PG2 高压轨】  PA 的高压 + 偏置，面板电源按钮控制的就是它
 *      power_hv_enable() / power_hv_disable()  操作员层，带幂等判断
 *      power_hv_on() / power_hv_off()           底层，分两拍
 *      power_rail_step() / _busy() / _abort()   时序推进 / 查询 / 取消
 *
 *  【RELAY1 + RELAY2】  射频通路
 *      power_relays_tx_on() / _off()           只管继电器，不碰任何电源
 *
 *  三条轨**互不联动**：高压按钮不会断开继电器，也不会动主电源；继电器动作
 *  不会碰高压。这样"断开高压"和"断开射频"才能独立控制。
 *
 *  ---------------------------------------------------------------------
 *  高压轨为什么要分两拍（HV_STAGE_DELAY_MS = 10 ms）
 *
 *      PA 偏置网络必须等高压轨稳定后才能加偏置，否则偏置台阶与供电台阶
 *      重叠，功放管承受的不是设计允许的上电时序。断开时反过来，先撤偏置、
 *      等 10 ms、再断高压。
 *
 *          上高压： PWR_ON(PG7) → [10ms] → RF_BIAS(PG2)
 *          下高压： RF_BIAS(PG2) → [10ms] → PWR_ON(PG7)
 *
 *  这是**非阻塞**实现：power_hv_on() 抛完第一拍就返回，绝不 HAL_Delay
 *  或忙等；第二拍由 app_monitor_task 每 10 ms 调一次 power_rail_step()
 *  完成。所以调用后立刻返回时，电路还没到终态 —— 这期间发射被拒绝。
 *
 *  状态标志：
 *      g_psu_on   外部 PSU 已开机（PG5 高）
 *      g_hv_on    高压轨已投入（比偏置早一拍，用 power_rail_busy() 判断
 *                 是否已到终态）
 *      g_tx_active 继电器已吸合
 *
 *  注意：发射请求**不**检查前两个标志，见 power_tx_enable() 的说明。
 *  这三个标志只用于状态显示与诊断。
 *=======================================================================*/

/*! 强制所有输出到关机/默认态，并取消任何在途的高压时序。
 *  上电流程开始前、测试开始前调用。可重复调用。
 *
 *  注意：它**不**等同于 power_hv_disable() —— 这个函数不管当前处于什么
 *  状态、也不会走继电器时序，只是把引脚摆到已知的空闲态。正常运行中不要
 *  用它代替任何操作员层函数。
 */
void power_init_outputs(void);

/*============================================================================
 *  【PG5】主电源（PSU，外部设备）—— 单拍，与高压轨和继电器都无关
 *
 *  PSU = Power Supply Unit，是**外部设备**，PG5 只是 MCU 进去的控制线：
 *
 *      PG5 ──> NPN Q9（反相）──> PC13 = PWR_WKUP4
 *
 *  Q9 反相，所以从 MCU 的 GPIO 视角看极性是「高电平开机」：
 *      PG5 高 → Q9 导通 → PC13 拉到 GND  → PSU 开机
 *      PG5 低 → Q9 截止 → PC13 回上拉   → PSU 关机
 *
 *  这与设计要求 6 节和 hw_pinmap.h 一致。**别按"低有效"的直觉写反。**
 *
 *  目前谁会调：
 *    power_soft_off()   关机时关掉 PSU —— 唯一由固件主动发起的动作
 *    远程 CAT 命令      仅骨架，协议未定（见 comm.c CMD_PSU_* 的 TODO）
 *    power_init()       **不调**：PSU 主要由用户自己操作，固件不该擅自开机
 *==========================================================================*/

/*! 外部 PSU 开机（PG5 拉高）。幂等。
 *  **不**碰 PWR_ON / RF_BIAS / 继电器 / REAR_RESERVED1。
 *  power_init() 不会调它 —— 上电时 PSU 保持关机，等用户操作。
 */
void power_psu_on(void);

/*! 外部 PSU 关机（PG5 拉低）。幂等。
 *  **不**碰 PWR_ON / RF_BIAS / 继电器 —— 断高压用 power_hv_disable()，
 *  整机下电用 power_soft_off()。
 */
void power_psu_off(void);

/*============================================================================
 *  【PG7 + PG2】高压轨底层开关 —— 分两拍，由 power_rail_step() 收尾
 *==========================================================================*/

/*! 请求上高压。**只抛第一拍就返回**，不等待、不阻塞。
 *
 *  立即效果：PWR_ON(PG7) 拉高；RF_BIAS(PG2) 保持低。
 *  终    态：10 ms 后 RF_BIAS 由 power_rail_step() 拉高。
 *
 *  返回后 g_hv_on 已为 true，但 **偏置尚未加上**，所以此时还不能发射 ——
 *  请用 power_rail_busy() 判断是否已到终态。
 *
 *  幂等：已在稳定上电状态时直接返回（不会重复触发时序）。
 *  @note 不检查 g_ptt_ready / g_alert_flag —— 门禁在 power_hv_enable()
 *        与 PTT 汇合点那两处。直调本函数会绕开"允许上电"的判定。
 */
void power_hv_on(void);

/*! 请求下高压。**只抛第一拍就返回**，不等待、不阻塞。
 *
 *  立即效果：拉低 RF_BIAS(PG2)。
 *  终    态：10 ms 后 PWR_ON(PG7) 由 power_rail_step() 拉低。
 *
 *  返回后 g_hv_on 已为 false，PWR_ON 仍保持高电平直到第二拍。
 *
 *  @note **不动继电器**。若此刻正在发射，继电器保持吸合，PA 带射频直接掉
 *        高压 —— 这是操作员按钮的明确意图（要断高压，不是要退发射）。
 *        保护跳闸才会既断高压又断继电器，且它不走本函数。
 *  @note 保护跳闸也不走本函数：那条路径要求 0 延迟，直接拉引脚，并调
 *        power_rail_abort() 取消在途时序。
 */
void power_hv_off(void);

/*============================================================================
 *  时序推进 —— 由 app_monitor_task 每 10 ms 调一次，业务代码不需要碰
 *==========================================================================*/

/*! 推进上下电时序一拍。**必须每 10 ms 调用一次**，否则上下电会卡在中间
 *  态（偏置永远加不上，或高压永远断不开）。
 *
 *  由 app_monitor_task 唯一负责调用。无 HAL_Delay、无忙等、无 FreeRTOS
 *  依赖，因此在哪个任务上下文调用都行。
 *
 *  @return true = 刚刚推进了一个阶段（上电链在等偏置，或下电链在等断高压）；
 *          false = 已处于终态，本次什么都没做。
 */
bool power_rail_step(void);

/*! 高压轨是否还卡在两拍之间。
 *
 *  在 ptt_apply_level() 里作为发射门禁使用：偏置尚未稳定时拒绝发射，
 *  避免功放在上电时序中途被拉起来。所有 PTT 路径（踏板 EXTI、TIM7
 *  轮询兜底、CAT/串口请求）都汇合到那一个判定点。
 */
bool power_rail_busy(void);

/*! 取消在途的上下电时序，**不碰任何引脚**。
 *
 *  只有保护跳闸需要：protection_trigger_locked() 自己以 0 延迟拉低
 *  PWR_ON / RF_BIAS，若不清掉排队中的阶段，它会在故障后把刚切断的高压
 *  轨重新拉起来 —— 那是最不能接受的故障。
 *
 *  ISR 安全：单次 volatile 写，不含任何 FreeRTOS API。
 */
void power_rail_abort(void);

/*============================================================================
 *  射频通路继电器
 *==========================================================================*/

/*! 两个继电器都吸合（PA → LPF → 天线通）。内部状态机带 15 ms 断-合
 *  延时：RELAY2 立即吸合，RELAY1 由 TIM5 回调后再吸合。
 */
void power_relays_tx_on(void);
/*! 两个继电器都断开。RELAY1 立即断开，RELAY2 由 TIM5 回调后再断开。 */
void power_relays_tx_off(void);

/*============================================================================
 *  操作员层 —— 面板电源按钮、面板 POWER 键、CAT 命令都走这里
 *
 *  与底层 power_hv_on/off 的区别：这两个先看当前状态再决定动不动，幂等
 *  且不会打断已在进行的动作。
 *==========================================================================*/

/*! 一次性的上电初始化：所有输出归位 → 风扇/泵 PWM 就绪 → **锁存主电源**。
 *  只在调度器启动前调一次。
 *  注意它**不**接通高压：PWR_ON / RF_BIAS 保持低，等操作员按电源按钮。
 */
void power_init(void);

/*! 操作员按"开机"：投入高压轨（PWR_ON → 10 ms → RF_BIAS）。
 *
 *  相对 power_hv_on() 的额外语义：
 *    - 高压已投入时**直接返回**，不会重复触发时序
 *    - 置 g_power_state = POWER_STATE_STANDBY（就绪、未发射）
 *    - 不会打断已在进行的发射流程
 *
 *  **不碰主电源锁存，也不碰继电器。**
 *
 *  调用点：GUI 顶部电源按钮（gui_power_button_pressed）、面板 POWER 键
 *  （key_power_action）、CAT/SPI6 的开机命令。
 *
 *  @note 返回后电路**还没到终态**，偏置要等一个 10 ms tick。需要"已就绪"
 *        才能发射的判定请用 power_rail_busy()，不要用 g_hv_on。
 */
void power_hv_enable(void);

/*! 从发射退回就绪：只断开发射通路，**高压与偏置保持**。
 *
 *  继电器经 15 ms 断-合延迟断开（TIM5），函数本身不等待。g_hv_on 保持
 *  true，所以操作员可以立刻重新踩踏板。
 */
void power_standby(void);

/*! 操作员按"关机"：撤偏置 → 10 ms → 断高压。
 *
 *  **只动 PWR_ON 与 RF_BIAS**：主电源锁存与继电器都保持原样。
 *  总是执行（不像 power_hv_enable() 那样幂等），重复调用安全，但每次都会
 *  重新走一遍时序。
 *
 *  @note 若此刻正在发射，继电器保持吸合、PA 带射频直接掉高压 —— 这是
 *        操作员按钮的明确意图（要断高压，不是要退发射）。
 *  @note 与 power_soft_off() 完全不同：后者是整机软关机（拉低
 *        REAR_RESERVED1，MCU 随即停摆），且**永不返回**。
 */
void power_hv_disable(void);

/*============================================================================
 *  Transmit request from a task source
 *
 *  供 CAT / 串口 / 面板发起的发射请求使用。与踏板 EXTI 走的是同一套
 *  联锁，最终都汇合到 ptt_apply_level()，所以拒绝的原因完全一致。
 *==========================================================================*/

/*! 请求发射。满足全部条件才会吸合继电器，否则**静默拒绝**：
 *      - g_ptt_ready  （IM 零点校准完成 且 GUI 已出帧）
 *      - g_allow_rf   （未因故障被封锁）
 *      - !g_alert_flag（无保护告警锁存）
 *      - !power_rail_busy()  （偏置已稳定，不在上下电时序中途）
 *
 *  **刻意不检查 g_psu_on / g_hv_on**：PA 有没有电不归固件在"请求发射"这
 *  一刻裁决 —— 操作员（或 PSU 自身）随时可能上电，拒绝了只是徒增困扰。
 *  剩下两条与电源相关的联锁都是硬件性质的，不是策略问题：
 *  power_rail_busy() 拦的是偏置正在稳定的那 10 ms（真实硬件瞬态），
 *  g_allow_rf / g_alert_flag 拦的是安全原因。
 *
 *  不阻塞：继电器的 15 ms 断-合由 TIM5 状态机完成。
 *  @return 无返回值。要确认是否真的吸合，读 g_tx_active 或 ptt_state。
 */
void power_tx_enable(void);

/*! 释放发射。断开继电器（RELAY1 立即、RELAY2 经 15 ms 延迟）。幂等。
 *  不受任何联锁限制 —— 故障或发射中止时**必须**能执行，所以不要在这里
 *  加"是否允许停机"的判断。
 */
void power_tx_disable(void);

/* Live level of PTT_INX, for task-side polling when an EXTI edge may have
 * been missed.  True means "transmit requested" (pin low, active low). */
bool power_get_ptt_state(void);

/* Follow the radio: pick the LPF band that frequency_hz falls into.
 * Out-of-band frequencies leave the current band alone. */
void power_update_band(uint32_t frequency_hz);

/*============================================================================
 *  Band switching
 *============================================================================*/

/* All LPF outputs LOW (inactive); pull high only the selected band.
 * Default on invalid band: BAND_15M_10M. */
void switch_band(BandType band);

/*============================================================================
 *  LCD backlight (PWM on TIM4 CH3 -> U8 AP3019 FB pin)
 *============================================================================*/

/* Convenience on/off wrapper (0 % / 100 %). */
void power_set_lcd_backlight(bool enable);

/* Explicit duty, 0..100 %.  The backlight starts at 0 % and is only
 * raised by boot_backlight_on() once the GUI has painted a frame, so a
 * cold boot never shows a half-drawn screen. */
void power_set_backlight_duty(uint8_t percent);

/*============================================================================
 *  Soft power-off
 *==========================================================================*/

/*! 软关机：**与 power_hv_disable() 是两回事，务必分清**。
 *
 *      power_hv_disable()  只断高压轨（PWR_ON + RF_BIAS），主电源锁存
 *                        保持，MCU 继续运行，可再次 power_hv_enable()。
 *      power_soft_off()    整机下电。依次断继电器、撤偏置、断高压、释放
 *                        主电源锁存，最后拉低 REAR_RESERVED1(PF4) ——
 *                        该脚是"维持主轨"的保持信号，松开后电源监控器切到
 *                        机内备用电源，MCU 随即掉电停摆。
 *
 *  动作顺序（不经过 power_rail_* 时序，因为已经没有下一拍的时间了）：
 *      1. 若在发射，断开继电器（含 15 ms 延迟）
 *      2. 先 RF_BIAS，再 PWR_ON，再释放 PSU_ON 保持位
 *      3. 背光 PWM 归零，让操作员看到"确实关了"而不是画面冻住
 *      4. 拉低 REAR_RESERVED1 → 延时 500 ms 等主轨塌落 → __WFI() 停摆
 *
 *  @warning **永不返回**。只能从任务上下文调用；调用前需先保存 dirty
 *           config（见 gui_process_poweroff 的 PoweroffStage 状态机，
 *           设计要求「延迟 500 ms 再调用软关机」）。
 */
void power_soft_off(void);

/*============================================================================
 *  PTT ISR / task glue
 *============================================================================*/

/* Called from EXTI5 ISR for BOTH edges of PTT_INX (falling = press,
 * rising = release; the pin is active low).  Runs no FreeRTOS APIs, reads
 * the live pin level rather than trusting the edge, and is therefore safe
 * to call for a duplicate edge. */
void power_control_ptt_isr(void);

/* Release-safety fallback, called from the fast TIM7 tick.  Reconciles the
 * state machine against the live PTT_INX level so a release edge that never
 * arrives still unkeys the amplifier within one tick.  Level based and
 * idempotent; interrupt-safe (no FreeRTOS API, no FPU). */
void power_ptt_poll(void);

/* Returns then clears the PTT event flag. */
int  power_get_ptt_flag(void);
void power_clear_ptt_flag(void);

/* Drive the PTT_OUT keying feedback pin (PC15, active high).  Normally
 * called by the PTT state machine; protection calls it directly so the
 * feedback to the radio drops in the same instant the relays do. */
void power_ptt_out_set(bool asserted);

/*============================================================================
 *  RELAY delay timer (TIM5 one-shot 15 ms)
 *============================================================================*/

/* Called from TIM5 update ISR when the relay-delay expires. */
void power_relay_timer_callback(void);

#ifdef __cplusplus
}
#endif

#endif /* POWER_CONTROL_H */