# 全局硬性约束（GLOBAL RULES）

> **必读 + 必遵守** — 适用于 wt-control / wt-gui / wt-networking 三个 Agent

---

## 规则 1：CubeMX 生成的代码只能动 Section 区间

STM32CubeMX 在每个自动生成文件里嵌入 `/* USER CODE BEGIN <name> */` … `/* USER CODE END <name> */` 段落。**CubeMX 重新生成代码时只覆盖 section 区间外的部分**；你写在 section 外的修改会被 ST 厂家代码覆盖丢失。

### 正确做法

只修改夹在 `/* USER CODE BEGIN <name> */` 和 `/* USER CODE END <name> */` 之间的内容。`BEGIN` 必须在 `END` 之前，**顺序不能反**。

实际章节覆盖示例见 [Core/Src/adc.c](../Core/Src/adc.c)、[Core/Src/comp.c](../Core/Src/comp.c)、[Core/Src/i2c.c](../Core/Src/i2c.c)。Section 命名示例：

- `/* USER CODE BEGIN Header */` … `/* USER CODE END Header */`
- `/* USER CODE BEGIN ADC1_Init 0 */` … `/* USER CODE END ADC1_Init 0 */`
- `/* USER CODE BEGIN 1 */` … `/* USER CODE END 1 */`（ISR 后置）

### 严禁

- ❌ 修改 ST 厂家在 section 外写的代码（MX_xx_Init 内函数体、HAL handler 模板等）
- ❌ 在文件末尾的 `/* USER CODE END <name> */` 之后追加代码
- ❌ 调换 `BEGIN` 与 `END` 的顺序
- ❌ 改 include 顺序或外层 wrapper 函数

### 涵盖范围

以下目录里的所有 `.c` / `.h` 都是自动生成：
- `Core/Inc/*.h`、`Core/Src/*.c`
- `Drivers/STM32H7xx_HAL_Driver/`（不允许改）
- `Middlewares/ST/STM32_USB_Device_Library/`（不允许改）
- ioc 配置的所有外设（ADC/TIM/UART/SPI/I2C/COMP/DAC/SAI/LTDC/SDMMC/DMA 等）

要新增 HAL 外设功能？开 CubeMX 添加并 re-code。

### 例外：自己新建的文件

如果你新建的文件（例如 `User/Src/my_feature.c`），可以随意编辑，没有 section 限制——因为 CubeMX 不会扫描你新建的文件。

---

## 规则 2：不要修改 ST HAL 驱动源码

`Drivers/STM32H7xx_HAL_Driver/Src/` 与 `Inc/` 下任何文件都禁止直接修改。如果发现需要修改 HAL 行为：

1. 在自己新建的文件里通过宏包装 HAL API
2. 在 `User/Inc/` 下提供自己的函数包装层
3. 在 main 分支文档化需要修改 HAL 的原因，等待 review 后改 ioc 重新生成

---

## 规则 3：ioc 重新生成

每次 ioc 修改后（即使是改 NVIC 优先级、加 DMA 通道、调采样时间），CubeMX 会重新生成 `Core/Inc/*.h` 与 `Core/Src/*.c`。**用户手动执行 ioc 重新生成。** 重新生成后：

1. 检查你自己的代码是否在 section 内（防止被你之前的 commits 覆盖）
2. 重新编译（CMake/Ninja 重新生成 build 目录）
3. 提交新生成的 Core 文件与你的代码一起 commit（**不要拆分 commit**，避免二进制不兼容）

---

## 规则 4：ST 厂家代码 vs 自有代码清单

| 路径 | 类型 | 是否可改 |
|---|---|---|
| `Core/Inc/*.h` | CubeMX 自动 | 仅 section 内 |
| `Core/Src/*.c` | CubeMX 自动 | 仅 section 内 |
| `Drivers/STM32H7xx_HAL_Driver/` | ST 官方 | ❌ |
| `Drivers/CMSIS/` | ARM/CMSIS 标准 | ❌ |
| `Middlewares/ST/STM32_USB_Device_Library/` | ST 官方 | ❌ |
| `Middlewares/Third_Party/FreeRTOS/` | FreeRTOS 官方 | ❌ |
| `Middlewares/Third_Party/lvgl/` | LVGL 官方 | ❌（除 port 文件） |
| `Middlewares/Third_Party/FatFs/` | FatFS 官方 | ❌ |
| `User/Inc/*.h` | **自有** | ✅ |
| `User/Src/*.c` | **自有** | ✅ |
| `User/USB_DEVICE/` | **自有副本**（可以修改，ioc 重新生成会保留，但需要你处理冲突） | ✅ |
| `startup_stm32h7b0xx.s` | 启动文件 | 仅 section 内 |
| `STM32H7B0XX_FLASH.ld` | 链接脚本 | ❌（除非加 section 内存布局） |
| `cmake/stm32cubemx/` | CMake 模板 | ❌ |

---

## 规则 5：section 注释格式严格匹配

写入 section 的代码示例：

```c
/* USER CODE BEGIN ADC1_Init 2 */
  /* 在这里写你的 ADC1 初始化扩展 */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
/* USER CODE END ADC1_Init 2 */
```

**格式细节：**
- `/* USER CODE BEGIN <name> */` 必须用 C 注释符号 `/* ... */`（注意是斜杠+星号，**不是双斜杠 `//`**）
- `name` 大写字母 + 下划线 + 数字（无空格）
- 每个 section 必须有完整配对的 BEGIN/END
- BEGIN 必须在 END 之前（位置在文件中的行号）

---

## 规则 6：违反的后果

如果你违反规则写了 section 外的代码：

- ✅ 当下能编译、运行
- ❌ 下次 ioc 重新生成时（哪怕只是改了 NVIC 优先级）你的代码会被**永久丢失**
- § 整个项目回滚到 CubeMX 默认状态

这是破坏性变更，请严格遵守。

---

## 规则 7：review 检查清单

每次 PR / merge 到 main 前确认：

- [ ] 没有改 `Drivers/`、`Middlewares/ST/`、`Middlewares/Third_Party/FreeRTOS/`、`Middlewares/Third_Party/FatFs/`、`Middlewares/Third_Party/lvgl/`（除 port 文件）
- [ ] `Core/Inc/*.h`、`Core/Src/*.c` 的 diff 只出现在 `/* USER CODE BEGIN/END */` 段落内
- [ ] 新增功能用自己新建的文件（`User/Inc/*.h`、`User/Src/*.c`）
- [ ] 修改 HAL 行为时优先改 ioc 重新生成而不是直接改 HAL

违反任一项的 PR 必须退回修改。

---

参考阅读：[docs/AGENT_BRIEFING.md](AGENT_BRIEFING.md) — 项目架构、硬件基线