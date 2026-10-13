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

## 5. 保护逻辑要点（GUI 显示弹窗时要知道）

### 5.1 触发源（ISR 中判定，task 中响应）

| 来源 | 阈值/动作 |
|---|---|
| AWD1（CH4）| RF 输入超限，按波段表 10~20W |
| AWD2（CH5）| 主电流 > 30A |
| AWD3（CH6）| REV1 > 800W（硬件最后防线）|
| COMP1 | RF1 过驱（COMP1 正输入=PB2=ADC1_INP4）|
| COMP2 | 主电流过流（COMP2 正输入=PE9=ADC1_INP5）|
| EXTI12 (HARDPWR_ERR PB12) | 电源故障 |
| 软件判读 | REV2 > 400W、REV2>200W 且 SWR>2.0、FWD1>1100W、FWD2>1050W、NTC>90℃ |

### 5.2 保护动作（ISR 中执行，**0 延迟**）

1. 释放 PTT（拒响应 PTT_INX）
2. 释放 RELAY1/RELAY2
3. 拉高 PB13 (RF_IN_PROTECT_CPU_SIG)
4. 拉低 PG7 (PWR_ON_SIG) 关闭高压
5. 置位 volatile 告警 flag（GUI task 后续读）

GUI 显示告警时直接读 `volatile` 标志位，**不要在 GUI task 里轮询 ADC 数值判保护**。

### 5.3 故障锁存总线 RF_IN_PROTECT_MIXED

PE12 (COMP1_OUT) → D1、PE8 (COMP2_OUT) → D6、PB13 → D7，三只二极管线或。任何一路拉高都触发外部硬件锁存（功放板切断）。

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