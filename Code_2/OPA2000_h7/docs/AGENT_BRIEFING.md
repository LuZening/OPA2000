# OPA2000 H7 固件项目 — Agent Briefing

> **读者**：wt-gui / wt-networking 两个 worktree 的开发 Agent
> **目的**：让你们快速掌握 Control 端已经达成的硬件/固件契约，避免重复劳动与冲突
> **配套文档**：`OPA2000_H7_固件设计要求.txt`（设计规范）、`OPA2000_h7.ioc`（CubeMX 配置）、`.trae/all_nets.json`（netlist 导出）

---

## 1. 项目结构与 Git 工作流

### 1.1 仓库布局

```
E:\Projects\RADIO\Projects\OPA2000\Code_2\
├── OPA2000_h7\                  ← main 分支（共享基础，docs/ 在这里）
├── OPA2000_h7_control\          ← wt-control 分支（Control Agent = 我）
├── OPA2000_h7_gui\              ← wt-gui 分支（你）
├── OPA2000_h7_networking\       ← wt-networking 分支（你）
└── OPA2000_h7_MSC_TEMPLATE\     ← 模板参考
```

### 1.2 分支策略

- `main`：基础基线，只放 **共享文档 + 公共头文件骨架**（任何 agent 都依赖的接口）
- `wt-control`：Control 逻辑（ADC/AWD/COMP/RELAY/PWM/电源/保护/校准状态机）
- `wt-gui`：GUI（LVGL7 + 触屏 + 多语言 + 显示）
- `wt-networking`：通信（UART4 电台 + LPUART1/USB CDC PC + SD config + SI5351）

每个 agent 在自己的工作区写代码，commit 推自己的分支。定期从 main 拉公共头变更，向 main 提公共头变更（PR-style，由你 review）。

### 1.3 提交规范

- commit message 简洁 + 包含模块名（参考之前的提交风格）
- 严禁 push（仓库有 push 保护）
- 严禁修改其他 agent 工作区的文件

---

## 2. 硬件基线（最重要 — 你们改代码前必读）

### 2.1 MCU 与封装

- **STM32H7B0ZBT6**，LQFP144，**lceda schematic 中标号 U5**（不是 U1）
- VREF+ 内部 VREFBUF 2.5V（仅接 C31/C89 退耦）
- 主时钟 250MHz，APB1/2 = 125MHz

### 2.2 引脚汇总（仅列与 Control/GUI/Networking 相关的）

| 引脚 | 功能 | 注意事项 |
|---|---|---|
| PA0/PA1/PA2/PA3/PB0/PB1/PC4/PC5/PF12 | ADC1 9 路外部通道 | 见 §4 |
| PC2_C/PC3_C | ADC2 INP0/INP1（455kHz 中频采样） | GUI/Networking 别占用 |
| PB12 | HARDPWR_ERR（EXTI12 上升沿） | 与 Control 强耦合 |
| PB13 | RF_IN_PROTECT_CPU_SIG（故障输出，OD 输出拉高） | 别设为输入 |
| PC14/PE5/PE4/PE3/PE2/PE1 | LPF1~LPF6，**高电平有效** | Control 独享 |
| PG7 | PWR_ON_SIG | Control 独享 |
| PG2 | RF_BIAS_ON（高电平使能） | Control 独享 |
| PG3/PG4 | RELAY2/RELAY1（高电平使能） | Control 独享 |
| PG5 | MAIN_PSU_ON_SIG（拉高=Q9 拉低 PC13=PSU 开机） | Control 独享 |
| PG0 | ALC_PWM_OUT（200kHz PWM） | Control 独享（TIM2 OC + DMA） |
| PC8 | FAN1_PWM（TIM3_CH3） | Control 独享 |
| PD14 | BL_PWM（TIM4_CH3） | Control 独享 |
| PD15 | PUMP_PWM（TIM4_CH4） | Control 独享 |
| PD4/PD5 | FAN1_SENSEX/PUMP_SENSEX（**仅 GPIO，禁止 EXTI**） | Control 独享 |
| PE6 | BUZZER（TIM15_CH2） | Control 独享 |
| PF5 | PTT_INX（EXTI5） | Control 独享 |
| PC13 | KEY1_WKUP（PWR_WKUP4） | Control 独享（唤醒源） |
| PE9 | COMP2_INP | Control 独享 |
| PE8/PE12 | COMP2_OUT/COMP1_OUT | Control 独享（硬件锁存） |
| PF11/PF15 | SAI2 SD_B/LTDC DE | 已被 LTDC/SAI 占用 |
| PE7 | ETR_FREQDET（TIM1_ETR） | Control 独享 |
| **PF1/PF0** | I2C2 SCL/SDA（NS2000/GT911 触屏） | GUI 独享（I2C 轮询） |
| **PF2** | CTouch_INTX（GT911 中断） | GUI 独享 |
| **PF3** | CTouch_RSTX（GT911 复位） | GUI 独享 |
| **PF4** | REAR_RESERVED1（**仅 GPIO，禁止 EXTI**） | Control 独享（释放给 EXTI4 给触屏） |
| PB6/PB7 | LPUART1 TX/RX | Networking 独享 |
| PB10/PB11/PB12/PA8/PC9 | SPI1 / I2C3 / EXTI / 触摸 | 见具体使用方 |
| PC10/PC11 | UART4 TX/RX（电台） | Networking 独享 |
| PA9/PA10/PA11/PA12 | USB DM/DP/VBUS | Networking + USB 共享 |
| PD0/PD1 | FDCAN1 | Networking 可选 |
| PA13/PA14 | SWD | Debug 保留 |
| PB8/PB9 | LTDC B6 / LIGHTNING_PROTECTION_A | GUI/Control 各一 |
| PD8/PD9 | CPU_READY / LIGHTNING_PROTECTION_B | Control 独享 |
| PB9 | LIGHTNING_PROTECTION_A | Control（防雷跳针位，默认 Low） |

### 2.3 严禁配置冲突清单

- **PF4 严禁 EXTI**（已被 PTT_INX/REAR_RESERVED1 占位，文档 [第 16、141 行]）
- **PD4/PD5 严禁 EXTI**（GPIO_Input + 上拉）
- **PB12 必为 EXTI 上升沿**（HARDPWR_ERR 保护源）
- **PC13 必为 PWR_WKUP4 + 上拉**（系统唤醒源）
- **PF5 必为 EXTI**（PTT_INX）

---

## 3. NVIC 中断优先级分组（**已固化**，禁止改动）

| 中断 | 主优先级 | 用途 |
|---|---|---|
| ADC1_2_IRQn | **0** | AWD1/2/3 保护 + ADC1/2 全局 |
| COMP_IRQn | **0** | COMP1/2 过流/过驱 |
| EXTI15_10_IRQn | **0** | HARDPWR_ERR (PB12) |
| TIM2_IRQn | 1 | PTT 继电器延时（**也用于 ALC 输出但靠 DMA 不靠中断**） |
| TIM5_IRQn | 1 | PTT 继电器延时 |
| TIM7_IRQn | 1 | 风扇/泵转速 0.5ms 轮询 |
| EXTI9_5_IRQn | 1 | PTT_INX (PF5) |
| TIM1_UP_IRQn | 5 | 频率计（默认） |
| EXTI2_IRQn (PF2) | 默认 | GT911 中断 |
| 其他 | ioc 默认 | — |

**Priority Group = NVIC_PRIORITYGROUP_4**

ioc 现状没有 ADC1_2/COMP/EXTI15_10 的 NVIC 行 → **必须打开 CubeMX 手动勾选**（你或 Control agent 都需要）。

**严禁在 ISR 中调用 FreeRTOS API**（除 `port_yield_from_isr` 类 if necessary，AIC）。

---

## 4. ADC 拓扑（你可能要在 GUI 里画图，需要真值）

### 4.1 ADC1（9 路常规，DMA1_Stream6，Circular）

| Rank | Channel | Pin | 信号 | 单位 | 转换函数 |
|---|---|---|---|---|---|
| 1 | INP4 | PC4 | RF1 输入功率 | mW | adc1_raw_to_rf_in_mW |
| 2 | INP5 | PB1 | 主电流 IM | mA | adc1_raw_to_im_mA |
| 3 | INP6 | PF12 | 反射功率1 REV1（耦合器1，PA→LPF） | mW | adc1_raw_to_fwd_rev1_mW |
| 4 | INP8 | PC5 | 发射功率1 FWD1（耦合器1） | mW | adc1_raw_to_fwd1_mW |
| 5 | INP9 | PB0 | 反射功率2 REV2（耦合器2，LPF→天线） | mW | adc1_raw_to_fwd_rev2_mW |
| 6 | INP14 | PA2 | VBIAS_SENSE（H5-5 经 R32 限流） | mV | adc1_raw_to_vbias_sense_mV |
| 7 | INP15 | PA3 | 发射功率2 FWD2（耦合器2） | mW | adc1_raw_to_fwd2_mW |
| 8 | INP16 | PA0 | 主电压 VM | mV | adc1_raw_to_vm_mV |
| 9 | INP17 | PA1 | 按键电阻分压 | ADC | adc1_raw_to_key_adc |

### 4.2 ADC2（2 常规 + 3 注入）

- Rank 1 = INP0 (PC2_C)：PA 输出 455kHz 中频（瞬时采样，软件解调包络）
- Rank 2 = INP1 (PC3_C)：电台输入 455kHz 中频（瞬时采样）
- InjectedRank 1 = INP2 (PF13)：核心 NTC
- InjectedRank 2 = INP6 (PF14)：第二 NTC/预留
- InjectedRank 3 = TEMPSENSOR：MCU 内部温度

### 4.3 ADC1 看门狗

- AWD1 → CH4 (RF 输入功率)
- AWD2 → CH5 (主电流)
- AWD3 → CH6 (反射功率 1，硬件最后防线 800W)

### 4.4 主电流芯片 CC6920BSO-50A

- 0A 输出 1.65V，灵敏度 26.4mV/A，VCC=3.3V
- 公式：`I_mA = (ADC_raw × 2500 / 65535 − 1650) × 1000 / 26.4`
- **零点校准状态机**：开机 RF_BIAS/PWR 未使能时取 100~500ms 平均值存为 `zero_offset_ADC`
- 全局 `volatile bool allow_RF`（默认 false），校准完成前阻止 PWR_ON_SIG (PG7) 与 RF_BIAS_ON (PG2) 拉高

---

## 5. 保护逻辑（Control 已实现；GUI / Networking 需要配合的部分见 5.4 / 5.5）

> Control 侧已全部实现并有 host 单元测试覆盖（`User/Tests/test_protection.c`，27 个用例）。
> **你们不需要重写判定逻辑，只需要做"告知用户 + 解除"这两件事。**

### 5.1 触发源

**数字量**（EXTI 中断，高电平有效，中断里直接取消发射并断电）

| 来源 | 引脚 | 语义 |
|---|---|---|
| HARDPWR_ERR | PB12 / EXTI12 | 电源板报警，**高电平有效**；ISR 内会重读引脚，伪边沿不动作 |
| （预留）RF deck 故障线 | 待定 | `protection_digital_isr(cause)` 是通用入口，加线只调它，不改关断路径 |

**模拟量**

| 通道 | 判定方式 | 阈值 |
|---|---|---|
| IMain | AWD2（CH5）硬件看门狗中断 | 30 A |
| REV1 | AWD3（CH6）硬件看门狗中断 | 800 W |
| RF 输入 | AWD1（CH4）硬件看门狗中断 | 按波段表 |
| REV2 | **ADC1 DMA 完成中断**（无 AWD） | 400 W |
| FWD1 | **ADC1 DMA 完成中断**（无 AWD） | 1100 W |
| FWD2 | **ADC1 DMA 完成中断**（无 AWD） | 1050 W |
| NTC | **ADC2 注入完成中断**（无 AWD） | 90.0 ℃ |

组合规则：`REV2 > 200 W 且 SWR > 2.0` 也报 `ALERT_SWR_HIGH`。
耦合器编号约定：coupler 1 = PA→LPF，coupler 2 = LPF→天线；**GUI 上显示的是 coupler 2**。
所有门限都存在 `g_config`，`protection.c` 内不得出现硬编码常量。
COMP1 / COMP2 两路比较器的中断入口是 `protection_comp1_isr()` / `protection_comp2_isr()`，
由 `HAL_COMP_TriggerCallback()`（注意不是 `..._RawCpltCallback`，HAL 里没有后者）
按 `hcomp->Instance` 分派 —— `MX_COMPx_Init()` 只配置外设，必须另外调
`HAL_COMP_Start_IT()` 比较器才会工作。
最后 `protection_check()`（Control task 100 ms tick）把所有比较再兜一遍底。

### 5.1a COMP 比较器与 DAC 参考电压

COMP1 / COMP2 的门限是 **DAC1 输出的参考电压**，存在 config 里由用户在
Settings → 保护 页调整，改完立即生效（`prot_save_cb()` →
`boot_apply_comp_thresholds()`）。

硬件事实（勿凭印象改）：

| 项 | 值 |
|---|---|
| COMP 可选 DAC 源 | **只有 DAC1_CH1 / DAC1_CH2**。DAC2 仅能经 `TPSENS_DAC2CH1` 进 COMP，那是温度传感器专用通路，**不是**通用参考源 —— 所以「一路用 DAC1、一路用 DAC2」在硬件上不成立，两路都取自 DAC1 |
| 分辨率 | 12 位，满码 4095 |
| 参考电压 | **VDDA = 3.3 V**，1 LSB = 0.806 mV |
| ⚠ 与 ADC 不同 | ADC 用 VREFBUF **2.5 V**，DAC 用 VDDA **3.3 V**，两条轨不同不可混用 |
| 换算 | `dac_code = ref_mV * 4095 / 3300` |
| 输出配置 | 缓冲关闭 + `DAC_CHIPCONNECT_INTERNAL`（缓冲关闭时 HAL 不允许 `..._BOTH`） |

| | COMP1 | COMP2 |
|---|---|---|
| 正输入（被测） | PB2 = ADC1_INP4 同节点，RF1 经 D5/R40 峰值检波 | PE9 = ADC1_INP5 同节点，主电流 IM（CC6920BSO-50A → U20-B 缓冲） |
| 负输入（参考） | DAC1_OUT1 | DAC1_OUT2 |
| config 字段 | `comp1_ref_mV` | `comp2_ref_mV` |
| 出厂默认 | 1500 mV（**未标定**，兜底用） | 2442 mV（= 30 A 设计门限，上限即此值） |
| 上限 | `COMP1_REF_MV_MAX` 3000 mV | `COMP2_REF_MV_MAX` 2442 mV |

COMP2 可按 `V = 1650 + I*26.4` 换算显示电流（20A→2178mV，25A→2310mV，30A→2442mV）。
**COMP1 的 mV↔W 换算未标定**，D5/R40 增益未知，禁止在代码里臆造；详见
`User/TODO-COMP1标定待办.txt`。标定前 COMP1 只作兜底，精确的 RF 输入门限
由 AWD1 在软件里承担。

### 5.2 保护动作（中断上下文执行，**0 延迟**，只有第一次触发生效）

1. `RF_IN_PROTECT_RAISE()`（PB13 拉高，**最先执行**，通知 RF 板执行自己的保护）
2. `RELAY1_OFF` + `RELAY2_OFF` + `PTT_OUT` 拉低（PC15，高有效）+ `ptt_state = IDLE`
3. `PWR_ON_DISABLE()`（PG7，断高压）
4. `RF_BIAS_DISABLE()`（PG2，断 PA 偏置）
5. `buzzer_start()`（TIM15 PWM 三音阶警示，循环播放）
6. `g_alert_active = 原因`、`g_alert_flag = true`、`g_allow_rf = false`、`g_tx_active = false`

只要 `g_alert_flag` 为真，任何 PTT 路径都会被拒绝（EXTI PTT、轮询兜底、`power_tx_enable()` 全部拒动）。

### 5.3 故障锁存总线 RF_IN_PROTECT_MIXED

PE12 (COMP1_OUT) → D1、PE8 (COMP2_OUT) → D6、PB13 → D7，三只二极管线或。任何一路拉高都触发外部硬件锁存（功放板切断）。

### 5.4 【GUI Agent 必做】告警弹窗与解除

保护一旦触发就**锁存**，不会自动恢复。以下两步必须由 GUI 完成：

**第一步：弹窗**

- 在 GUI task 里轮询这两个 volatile（**不要**在 GUI 里重新判 ADC 数值）：
  ```c
  extern volatile bool      g_alert_flag;      /* 已锁存 */
  extern volatile AlertType g_alert_active;    /* 故障原因 */
  bool protection_is_active(void);
  const char *protection_get_fault_string(AlertType a);  /* 英文短语 */
  ```
- `g_alert_flag` 由 false 变 true 的瞬间弹一个**模态**告警框，标题/正文用
  `protection_get_fault_string(g_alert_active)`。
- 弹窗期间要**每帧持续重绘/保持**，因为蜂鸣器在后台循环鸣响，弹窗是用户唯一的静态提示。
- 多语言：这条短语要走 i18n 字典（§7.4），不要硬编码。
- 同一故障只弹一次：自己保存"已弹过"标志，否则 100 ms 一次的 `protection_check()`
  不会重复触发（锁存后函数直接 return），但你自己别每帧重建对象。
- **弹窗显示期间不要调用任何能改 `g_alert_*` 的函数**，否则用户还没点确定就被解除了。

**第二步：解除**

用户点"确定/知道了"之后：

```c
protection_clear();
```

`protection_clear()` 做了三件事，**你不用重复做**：
- 复位内部锁存 + `g_alert_active = ALERT_NONE`、`g_alert_flag = false`
- 拉低 PB13（`RF_IN_PROTECT_CLEAR()`），通知 RF 板保护已解除
- `buzzer_stop()`（TIM15 PWM 停，蜂鸣器静音）

**解除后 RF 路径自动恢复**：

- `g_allow_rf` 恢复 `true`——CC6920 零点与本次故障无关，原标定值继续有效；
  ADC 任务只在开机时跑一次 `adc_im_zero_offset_tick()`，不要求任何手动操作。
- `g_hv_on` **不会**被保护清掉，电源指示灯仍是"就绪"。如果你的"一键恢复"流程需要
  重新上高压，请显式调 `power_hv_disable()` → `power_hv_enable()`，不要去改 `g_hv_on`。

**如果你要做"一键复位"**：用户长按确认（防误触）后，顺序是 `protection_clear()` →
`power_hv_disable()` → `power_hv_enable()`。顺序反了会留下 PB13 仍拉高的状态。

### 5.4a 高压轨时序（**不要绕过**）

**三条轨互不联动**，各有各的函数名：

| 轨 | 引脚 | 函数 | 谁触发 |
|---|---|---|---|
| 主电源锁存 | PG5 `PSU_ON` | `power_psu_on()` / `power_psu_off()` | `power_init()` 启动时置位；`power_soft_off()` 关机时释放 |
| **高压轨** | PG7 `PWR_ON` + PG2 `RF_BIAS` | `power_hv_enable()` / `power_hv_disable()`（操作员层）<br>`power_hv_on()` / `power_hv_off()`（底层）<br>`power_rail_step/busy/abort()`（时序） | GUI 顶部横幅电源按钮、面板 POWER 键、CAT 命令 |
| 射频通路 | RELAY1 + RELAY2 | `power_relays_tx_on()` / `power_relays_tx_off()` | PTT |

高压按钮**不会**断开继电器，**不会**动主电源；继电器动作**不会**碰高压。三者独立可分别控制。

高压轨两条引脚**永远不同时切换**，中间强制 10 ms 间隔：

```
上高压：  PWR_ON(PG7) → [10 ms] → RF_BIAS(PG2)
下高压：  RF_BIAS(PG2) → [10 ms] → PWR_ON(PG7)
```

实现方式是**非阻塞状态机**：`power_hv_on()` 抛完第一拍就返回（绝不 `HAL_Delay` 或忙等），第二拍由 `app_monitor_task` 每 10 ms 调一次 `power_rail_step()` 完成。所以调用后立刻返回时电路还没到终态——这期间发射被拒绝。

状态标志是三个独立的：`g_psu_on`（主电源）、`g_hv_on`（高压轨，比偏置早一拍）、`g_tx_active`（继电器）。三者都成立才可能产生射频。判断"已到终态"请用 `power_rail_busy()`，不要用 `g_hv_on`。

为什么 `app_monitor_task` 的 tick 从 100 ms 改成 10 ms：只有 10 ms 的分辨率才能兑现这 10 ms 间隔。PTT 边沿、风扇曲线、转速测量用 `slow_div` 分频回 100 ms，不受影响。

**三条 PTT 路径都在 `power_rail_busy()` 时被拒绝**（`ptt_apply_level()` 是所有路径的汇合点），所以偏置未稳定时踩踏板无反应——这是刻意的，几十毫秒后再踩即可。

**下高压时继电器保持原样**。若此刻正在发射，继电器仍吸合、PA 带射频直接掉高压——这是操作员按钮的明确意图（要断高压，不是要退发射）。

**保护跳闸不走这条路**：`protection_trigger_locked()` 直接拉低引脚（0 延迟），并调 `power_rail_abort()` 清掉待执行阶段，否则排队中的阶段会在故障后把刚切断的轨重新拉起来。故障**不改** `g_hv_on`/`g_psu_on`，面板仍显示"就绪"，真正封锁射频的是 `g_allow_rf`/`g_alert_flag`。

`power_soft_off()` 同样直落引脚（它不返回，且随后主轨就要塌落），顺序是继电器 → `RF_BIAS` → `PWR_ON` → 释放 `PSU_ON` 锁存 → 拉低 `REAR_RESERVED1`。

### 5.5 【Networking Agent 必做】CAT 串口 / SPI6 解除命令

远程解除走命令通道，判定逻辑仍由 Control 负责，你只做透传：

- 收到 **CAT 串口（UART4）或 SPI6 数据包**里的"解除告警/复位故障"命令时，调：
  ```c
  protection_clear();
  ```
  效果与 §5.4 第二步完全相同：停蜂鸣器、拉低 PB13、清 `g_alert_flag`。
- **不要自己再去拉 PB13、不要自己去停 TIM15**，全部由 `protection_clear()` 收口，
  否则会和 ISR 里的关断动作打架。
- 解除后建议回一条确认帧（你定协议），让上位机知道告警已清。
- 同样地，`g_allow_rf` 解除后已恢复 `true`，CAT 状态查询帧可以直接回报"射频就绪"。
- CAT 侧还可以顺手做只读上报：把 `g_alert_active` 编进状态查询帧（SWR/电流类故障码）。
  **只读上报不要调任何 clear 类函数。**
- GUI 和 Networking 可能同时收到解除请求：`protection_clear()` 是幂等的，
  重复调用不会出错，但也别在同一次故障里反复发解除命令。

### 5.6 责任边界（避免重复劳动）

| 事情 | 谁做 |
|---|---|
| 判定阈值、DMA/AWD/EXTI 中断里关断、锁存、蜂鸣器启动 | **Control（已完成，勿重写）** |
| 弹窗显示、多语言文案、解除按钮 | **GUI（§5.4）** |
| CAT/SPI6 解除命令、状态上报 | **Networking（§5.5）** |
| IM 零点校准后恢复 `g_allow_rf` | Control 任务开机自动跑，**无 GUI 入口** |

---

## 6. 通信接口分工（Networking Agent 重点）

### 6.1 LPUART1 (PB6/PB7) + USB CDC 共享队列

- 速率默认 19200，可 [config] 改
- LPUART1 时钟源 ioc = CSI（4MHz），误差约 0.5%（用户认可现状）
- RX/TX 数据包共享循环队列；同一命令协议同时走 LPUART1 + USB CDC

### 6.2 UART4 (PC10/PC11) 电台通信

- 三种协议：ICOM CIV、Yaesu、Kenwood
- 速率/型号由 [config] 指定，或自动探测
- 文档 [第 102 行]：与 100us TIM1_ETR 频率计配合

### 6.3 SD 卡 + FatFS

- SDMMC2 4-bit bus
- FatFs mount 到 /fs
- config 二进制文件存在 /fs/config.bin

### 6.4 SI5351 (I2C3)

- CLK0 = LO1 (RF2 通道混频本振)
- CLK1 = LO2 (RF1 通道混频本振)
- 编程公式：f_RF ± 455kHz
- 信号消失时 LO 停靠安全频率

### 6.5 ES8388 (I2C3 + SAI2)

- LIN1 = D23 直接包络检波 → R136 → C33
- RIN1 = D22 直接包络检波 → R138 → C109
- AM 音频带宽约 5.5kHz（**R136/R138/C33/C109 阻容值待查**）
- 数字音频走 SAI2

---

## 7. 触屏与显示（GUI Agent 重点）

### 7.1 触屏（NS2009/GT911）

- **全部走 I2C2 周期轮询 20ms，PF4 不再作 PENIRQ**
- I2C2 总线已有上拉 + R108/R109 串联保护
- NS2009 寄存器格式/字节数查 datasheet
- GT911 INT 接 PF2 (EXTI2)、RST 接 PF3 (GPIO 输出，可选 I2C 地址)
- 默认使用电阻屏，电容屏检测不到时回退

### 7.2 显示分辨率 480×800×16bit (RGB565)

- LTDC 全尺寸缓冲 768KB
- LVGL partial buffer = 屏幕行数 1/5 ≈ 153KB
- 必须放在指定 SRAM（注释 `framebuffer section`）
- 用 lvgl 7（注意与 v8 不兼容），API 通过 context7 查

### 7.3 5 点触屏校准

- config 损坏时启动校准
- 4 个角点（距角 10%）+ 中心点
- X/Y 一元线性回归，检查中心点残差（>10% 重新校准）

### 7.4 多语言

- 文档 [第 134 行] 要求英文/中文可切换
- 每个显示屏字符串维护多语言条目
- 由 Control agent 共享 i18n 字典

---

## 8. 共同接口（main 分支会建的公共头）

以下文件由 Control agent 在 main 分支建好后给所有 agent：

- `User/Inc/hw_pinmap.h` — 引脚宏定义 + GPIO port/pin 集中宏
- `User/Inc/adc_calibration.h` — ADC 转换函数原型 + AWD/COMP 配置宏
- `User/Inc/config.h` — 配置项结构体
- `User/Inc/probe.h` — 调试探针
- `User/Inc/error.h` — 错误码

请在自己 worktree 拉取后即可使用，不要在自己分支再定义。

---

## 9. 关键避坑清单

1. **GUI 不要直接读 ADC 寄存器** — 全部通过 Control 提供的转换函数接口（GUI 线程只读 volatile 镜像或共享结构体）
2. **Networking 不要在 ISR 里 enqueue 命令** — 用 `osMessageQueuePut` from ISR + `FromISR` 标志
3. **GUI 显示字符串不要硬编码中文** — 走 i18n 字典
4. **RELAY 默认 Low，LPF 默认 Low，RF_BIAS 默认 Low** — 上电不可有突变
5. **AWD ISR 中不许用 printf** — 用 flag 通知 task
6. **HAL_Delay 不许在 ISR** — 文档 [第 122 行] 明令禁止
7. **CFG 数据要 CRC 校验** — Control agent 会在 config.c 里加
8. **ioc 修改必须重新生成 Core/ 目录** — 不要手动改 Core/Inc 或 Core/Src
9. **GPIO 输出冲突**：GUI 不要碰 PC14/PE5/PE4/PE3/PE2/PE1、PG2/PG3/PG4/PG5/PG7、PD4/PD5/PC8/PD14/PD15/PE6/PG0 — 这些都是 Control 的
10. **EXTI4 已被释放**（PF4 = REAR_RESERVED1 普通 GPIO），不要用 EXTI4 给触屏
11. **自己写的函数必须先有测试**（见 [GLOBAL_RULES.md](GLOBAL_RULES.md) 规则 0）。每个新功能要有 happy path + 边界测试，本地跑通后再 commit。

---

## 10. 沟通方式

- 我（Control Agent）在 `wt-control` 分支开发
- 你们在自己的 worktree 开发，不要碰 `wt-control` 分支
- main 分支由你 review 后才合并各分支
- 任务同步通过本聊天（你发出新指令我看到）

下一步等待 Control agent 在 main 分支提交公共头骨架后拉取。