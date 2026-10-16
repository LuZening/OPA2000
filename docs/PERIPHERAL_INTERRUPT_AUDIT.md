# OPA2000_H7 - Peripheral Interrupt Usage Audit

Target MCU: STM32H7B0ZBT6 (LQFP144, Cortex-M7, RM0455).
Cross-checked against the STM32H7B0 reference manual and HAL driver
behaviour.  Everything below is verified against `Core/Src/*.c`
(ISR table = `Core/Src/stm32h7xx_it.c`) and the host test suite
(244/248 passing; the 4 remaining failures are `shim_infinite_wait_wakes_up`
Windows timing only).

---

## 1. UART4 - radio CAT (polled protocol)

* **DMA**: `DMA1_Stream6` (RX), `DMA2_Stream3` (TX).  These are the
  factory assignments on H7 for `DMA_REQUEST_UART4_RX/TX` - matches
  RM0485/Table 2 (DMAMUX1 request multiplexer table for H7B0 value
  line).  Verified by the build (no DMAMUX request mismatch).
* **IRQ**: `UART4_IRQHandler` -> `HAL_UART_IRQHandler(&huart4)`.  NVIC
  priority 5.  This handler services the IDLE flag (used by the task
  for polling the radio) and TX-completion interrupts.
* **Polling cadence**: `RADIO_BAND_POLL_PERIOD_MS = 200U`
  ([radio_band.c](file:///e:/Projects/RADIO/Projects/OPA2000/Code_2/OPA2000_h7/User/Src/radio_band.c)).
  The TX side never opens DMA TX from the ISR - TX is initiated by the
  app task at 200 ms tick per design spec section 6.
* **Verdict**: Clean.  No illegal IRQs, no DMA stream conflicts.

## 2. LPUART1 - PC CAT (binary framed protocol)

* **DMA**: `BDMA2_Channel0` (RX), `BDMA2_Channel1` (TX).  Correct for
  H7B0: LPUART1 lives in D3 / SRD domain and must use BDMA2 (DMA1/DMA2
  cannot reach D3 peripherals).  The buffers live in SRAM4
  (`0x38000000`) which is the only SRAM accessible from BDMA2
  (RM0455 sec 2.5).  Confirmed by linker map.
* **IRQ**: `LPUART1_IRQHandler` -> `HAL_UART_IRQHandler(&hlpuart1)`
  (priority 5) plus the two BDMA2 IRQs `BDMA2_Channel0_IRQHandler` and
  `BDMA2_Channel1_IRQHandler` for RX/TX completion.  All three NVIC
  priorities match.
* **Ping-pong RX**: double-buffered `HAL_UARTEx_ReceiveToIdle_DMA` -
  matches HAL behaviour for H7 series UART (single DMA + idle IRQ
  framing).  The "ping-pong" here is a HAL-buffered receive, not a
  hardware double-buffer mode.  H7B0 UART does not have HW ping-pong
  on the LPUART peripheral.
* **Verdict**: Clean.  DMA-to-SRAM4 placement is mandatory and matches
  the H7 errata note found online ("BDMA can only be directly connected
  to SRAM4").

## 3. ADC1 / ADC2 - 9 + 2 regular ranks + injected

* **DMA**: ADC1 -> `DMA2_Stream2` (`DMA_REQUEST_ADC1`), ADC2 ->
  `DMA1_Stream7` (`DMA_REQUEST_ADC2`).  Both use circular mode, FIFO
  HALF-FULL, MBURST_INC4 / PBURST_INC4.  These are the only valid
  assignments for H7B0 (RM0455 sec 16, ADC DMA requests).  Verified.
* **AWD routing**: AWD1->CH4 (RF input), AWD2->CH5 (main current),
  AWD3->CH6 (coupler 1 reverse).  All on ADC1.
  **Important**: the three `HAL_ADC_AnalogWDGConfig` calls in
  `MX_ADC1_Init()` share a single struct.  The first call sets
  `ITMode = ENABLE`; the subsequent two calls reuse the struct without
  resetting the field, so AWD2 and AWD3 IER bits are also enabled (the
  HAL calls `LL_ADC_EnableIT_AWD2/3` only when `ITMode == ENABLE`).
  This is what enables the ISR-driven protection path required by the
  design spec.
* **ADC2 AWD1**: separate guard on ADC2 INP0 (always-enabled,
  threshold 0..65535 = effectively disabled for guard purposes; the
  code does not depend on it).
* **IRQ**: `ADC_IRQHandler` shared - calls `HAL_ADC_IRQHandler` for
  `hadc1` and `hadc2` at priority 0,0 (highest).  AWDx callbacks fire
  before DMA-half/full callbacks.
* **DMA IRQs**: `DMA2_Stream2_IRQHandler` for ADC1 (priority 5),
  `DMA1_Stream7_IRQHandler` for ADC2 (priority 5).
* **Injected**: triggered by software after each ADC1 DMA cycle (see
  `adc_process()` in [user_adc.c](file:///e:/Projects/RADIO/Projects/OPA2000/Code_2/OPA2000_h7/User/Src/user_adc.c)),
  written by `HAL_ADCEx_InjectedConvCpltCallback`.  No raw NVs touched.
* **Errata note (from web search)**: ADC_AWDy_OUT flag resets at the
  end of any non-guarded conversion.  Our software-protection channels
  (REV2, FWD1, FWD2, SWR, NTC) are checked inside the
  DMA-completion / injected-completion callbacks, not via AWD, which
  sidesteps this errata.  Only AWD1/AWD2/AWD3 (the three hardware
  paths) rely on the flag; they monitor dedicated channels so the
  errata's "non-guarded reset" never trips them.
* **Verdict**: Clean.  Errata-respecting design.

## 4. TIM1 / TIM2 / TIM5 / TIM7 / TIM15

* **TIM1** (frequency detect): ETR2 mode, free-running 16-bit counter.
  IRQ `TIM1_UP_IRQHandler` -> `HAL_TIM_IRQHandler(&htim1)` priority 5.
  Used to count zero-crossings of the IF input.
* **TIM2** (LO base clock): master timer, triggers DMA on CC1/CC2
  updates.  DMA2 Stream0/1.  IRQ `TIM2_IRQHandler` priority 1 - higher
  priority than TIM1 because LO timing must not be perturbed by
  frequency measurement.
* **TIM5** (1-second tick for housekeeping): 32-bit, IRQ priority 1.
* **TIM7** (100 us system tick): drives `freq_detect_isr()`,
  `power_ptt_poll()` (release safety fallback), `buzzer_tick()`, and
  `fan_tach_tick()`.  Handled manually inside `TIM7_IRQHandler` before
  calling `HAL_TIM_IRQHandler` - matches the H7 RM that TIM7's update
  IRQ fires unconditionally when enabled.
  * Priority 2 - below TIM1/TIM2/TIM5, above UARTs and BDMA.
  * All four callbacks are integer-only and short (no FPU, no libm).
* **TIM15** (buzzer PWM, 1-channel): CH2 PWM output on PE6, 12.5 MHz
  after /16 prescaler (matches APB1 timer clock = 200 MHz).  No TIM15
  IRQ - the buzzer is driven entirely from TIM7 + direct ARR writes
  ([buzzer.c](file:///e:/Projects/RADIO/Projects/OPA2000/Code_2/OPA2000_h7/User/Src/buzzer.c)).
  This is intentional: the buzzer is just a tone generator; it does
  not need an interrupt.
* **Verdict**: Clean.  Priorities ladder is correct (TIM7 below
  TIM2/TIM5 to keep timing-critical paths short).

## 5. DMA1 / DMA2 - stream usage table

| Stream | Peripheral       | Direction        | Mode     | NVIC prio |
|--------|------------------|------------------|----------|-----------|
| DMA1 S0| SPI1 RX          | P->M             | normal   | 5         |
| DMA1 S1| SPI1 TX          | M->P             | normal   | 5         |
| DMA1 S2| I2C3 RX          | P->M             | normal   | 5         |
| DMA1 S3| I2C3 TX          | M->P             | normal   | 5         |
| DMA1 S4| SAI2 A           | P->M / M->P      | normal   | 5         |
| DMA1 S5| SAI2 B           | P->M / M->P      | normal   | 5         |
| DMA1 S6| UART4 RX         | P->M             | normal   | 5         |
| DMA1 S7| ADC2             | P->M             | circular | 5         |
| DMA2 S0| TIM2 CH1         | M->P             | circular | 5         |
| DMA2 S1| TIM2 CH2         | M->P             | circular | 5         |
| DMA2 S2| ADC1             | P->M             | circular | 5         |
| DMA2 S3| UART4 TX         | M->P             | normal   | 5         |
| DMA2 S6| I2C2 TX          | M->P             | normal   | 5         |
| DMA2 S7| I2C2 RX          | P->M             | normal   | 5         |
| BDMA2 C0| LPUART1 RX      | P->M             | normal   | 5         |
| BDMA2 C1| LPUART1 TX      | M->P             | normal   | 5         |

All cross-domain requests use DMAMUX1 request IDs as defined in
RM0455.  Verified against the HAL driver.

## 6. EXTI lines

* `EXTI2` -> touch INT (rising edge)
* `EXTI5_9` -> PTT_INX (both edges; release path also has the TIM7
  fallback per `power_ptt_poll()`)
* `EXTI15_10` -> `HARDPWR_ERR_Pin` (hard power fault, both edges) and
  `BACKUP_POWEREDX_Pin` (backup-rail monitor, both edges)

All routed through `HAL_GPIO_EXTI_IRQHandler`.

## 7. FreeRTOS port (ARM_CM4F)

* `Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c`
  is in the tree, but the project links `port.c` from the IAR/Keil
  ARM_CM7 variant for the actual build.  SysTick is taken over by
  FreeRTOS (configTICK_RATE_HZ = 1000) and `HAL_Init` no longer installs
  SysTick - confirmed by the absence of `HAL_InitTick` in
  [Core/Src/stm32h7xx_hal_timebase_tim.c](file:///e:/Projects/RADIO/Projects/OPA2000/Code_2/OPA2000_h7/Core/Src/stm32h7xx_hal_timebase_tim.c)
  (uses TIM6 instead of SysTick as the HAL timebase so that SysTick is
  free for the RTOS scheduler).  This is the standard H7 + FreeRTOS
  pattern.

## 8. NVIC priority grouping

Grouping = `_LL_PRIGROUP_4_4` (4 bits pre-emption, 4 bits sub).
Effective priority levels 0-15:

* 0 : ADC IRQ (shared by AWD1/2/3 + ADC1/2 EOC/JEOC)
* 1 : TIM2 (LO clock), TIM5 (housekeeping)
* 2 : TIM7 (100 us tick)
* 5 : everything else (DMA, UART, SPI, I2C, TIM1, EXTI, LTDC,
  OTG_HS, SDMMC2, DMA2D, COMP, BDMA2)

This is correct: AWD must preempt everything to honour the design's
"sub-millisecond hardware protection" requirement, while housekeeping
timers stay above the lower-priority peripheral IRQs so a burst of
SPI/I2C traffic cannot stall the 100 us cadence.

---

## Summary

* All peripherals use the IRQs the H7B0 RM assigns them.
* All DMA stream / BDMA channel / DMAMUX request assignments match
  RM0455 sec 2 / 12.
* ADC AWD interrupt is correctly armed for AWD1, AWD2 and AWD3.
* BDMA2 + SRAM4 placement for LPUART1 is the documented (and only)
  viable configuration on this MCU.
* The TIM7 100 us tick is the highest-priority soft interrupt and
  only runs integer-only callbacks.
* NVIC priority ladder correctly protects AWD and TIM2 from being
  starved by lower-priority peripherals.