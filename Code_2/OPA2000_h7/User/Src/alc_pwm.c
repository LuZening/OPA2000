/**
 * @file alc_pwm.c
 * @brief ALC PWM implementation: TIM2 OC + DMA -> PG0 BSRR.
 *
 * @details TIM2 has two Output Compare (No Output) channels enabled:
 *   CH1 (CCR1) - DMA -> GPIOG->BSRR writes (1<<0)  => set PG0 (rising)
 *   CH2 (CCR2) - DMA -> GPIOG->BSRR writes (1<<16) => reset PG0 (falling)
 *
 * Combined output:
 *   - PWM frequency f_out = f_TCLK / (ARR + 1)
 *     where f_TCLK = APB1 timer clock = 250 MHz (SYSCLK 250 MHz, APB1
 *     125 MHz, prescaler != 1 so the timers see 2 x APB1)
 *   - Duty cycle = (CCR2 - CCR1) / (ARR + 1)
 *
 * For ~200 kHz: ARR = 250 MHz / 200 kHz - 1 = 1249
 *
 * GPIOG base address on STM32H7B0: GPIOA=0x58020000, port stride 0x400.
 *   GPIOA + 6 * 0x400 = 0x58020000 + 0x1800 = 0x58021800 -> GPIOG base
 *   GPIOG->BSRR offset within port = 0x18
 *   => GPIOG->BSRR absolute = 0x58021818
 *
 * NB: the DMA "destination" is the GPIO BSRR register (a 32-bit
 * memory-mapped I/O register).  We feed it 1-word payloads from RAM.
 */

#include "alc_pwm.h"
#include "stm32h7xx_hal.h"

/* BSRR payloads:
 *  bit 0  set -> PG0 high
 *  bit 16 set -> PG0 low
 *
 * 这两块内存是 TIM2 CH1/CH2 的 DMA 源，被 DMA2（D2 域）读取；DMA2 访问不到
 * DTCM(0x20000000)，所以放 app.ld 的 .dma_no_cache（AXI SRAM 非缓存窗口）。
 * 该段为 NOLOAD，startup 不会赋初值，故在 init/start 里显式赋值。 */
#if defined(__GNUC__)
#define ALC_BSRR_SECTION  __attribute__((section(".dma_no_cache"), aligned(4)))
#else
#define ALC_BSRR_SECTION
#endif
volatile uint32_t alc_bsrr_set   ALC_BSRR_SECTION;   /*!< PG0 = HIGH */
volatile uint32_t alc_bsrr_reset ALC_BSRR_SECTION;   /*!< PG0 = LOW  */

/* Hard-coded absolute address of GPIOG->BSRR.  ioc configured PG0 as
 * GPIO_Output, so the GPIO port is clocked and the register is alive. */
#define GPIOG_BSRR_ADDR  (0x58021818UL)

/*============================================================================
 *  Public API
 *============================================================================*/

void alc_pwm_init(void)
{
    /* 载荷放在 NOLOAD 段，需在运行时赋初值 */
    alc_bsrr_set   = 0x00000001U;   /*!< PG0 = HIGH */
    alc_bsrr_reset = 0x00010000U;   /*!< PG0 = LOW  */

    /* TIM2 clock = 250 MHz.  Prescaler = 0 -> counter runs at 250 MHz.
     * ARR 1249 -> 1250 ticks -> 200 kHz PWM. */
    htim2.Instance->PSC = 0U;
    htim2.Instance->ARR = 1249U;
    htim2.Instance->CCR1 = 0U;       /*!< rising edge */
    htim2.Instance->CCR2 = 625U;     /*!< falling edge -> 50% duty start */
}

void alc_pwm_start(void)
{
    /* 载荷在 NOLOAD 段，DMA 启动前确保已赋初值（防止未调 init 直接 start） */
    alc_bsrr_set   = 0x00000001U;
    alc_bsrr_reset = 0x00010000U;

    /* Each CCR match triggers its DMA channel to copy a single word
     * from RAM to the GPIOG BSRR register.  Data alignment is 32-bit.
     * The HAL_TIM_OC_Start_DMA signature is:
     *   htim, channel, pData (source), Length (words)
     * The destination is implied by the DMA request source (TIM2_CH1 /
     * TIM2_CH2 triggered DMA streams).  We must therefore use
     * HAL_DMA_Start directly to point to the BSRR register. */
    HAL_DMA_Start(&hdma_tim2_ch1,
                 (uint32_t)(uintptr_t)&alc_bsrr_set,   /* src */
                 (uint32_t)GPIOG_BSRR_ADDR,            /* dst */
                 1);                                   /* length 1 word */
    HAL_DMA_Start(&hdma_tim2_ch2,
                 (uint32_t)(uintptr_t)&alc_bsrr_reset,
                 (uint32_t)GPIOG_BSRR_ADDR,
                 1);

    HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_Base_Start(&htim2);
}

void alc_pwm_stop(void)
{
    HAL_TIM_Base_Stop(&htim2);
    HAL_TIM_OC_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_OC_Stop(&htim2, TIM_CHANNEL_2);
    /* Park PG0 low to avoid spurious output. */
    HAL_GPIO_WritePin(ALC_PWM_GPIO_Port, ALC_PWM_Pin, GPIO_PIN_RESET);
}

void alc_pwm_set_period(uint16_t arr)
{
    htim2.Instance->ARR = arr;
}

void alc_pwm_set_pulse(uint16_t rising_ccr, uint16_t falling_ccr)
{
    htim2.Instance->CCR1 = rising_ccr;
    htim2.Instance->CCR2 = falling_ccr;
}