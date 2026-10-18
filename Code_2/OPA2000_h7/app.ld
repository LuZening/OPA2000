/*
******************************************************************************
**  File        : app.ld
**  Abstract    : OPA2000 application linker script.
**
**  全部代码/只读数据/字库都放在外部 OSPI NOR FLASH（memory-mapped 到
**  0x90000000）中执行（XIP）。内部 128 KB FLASH 只留给 bootloader：由它把
**  OCTOSPI1 配成 memory-mapped 模式，然后跳转到 0x90000000。bootloader 工程
**  继续使用 STM32H7B0XX_FLASH.ld，本文件不要动那个。
**
**  SRAM 布局（与 CubeMX 生成的 MPU_Config / .ioc 中的 MPU region 对齐）：
**    DTCMRAM 0x20000000 : .data/.bss/heap/stack + LVGL 堆（CPU 专用、最快）
**    AXI SRAM 0x24000000: .framebuffer   <- LTDC 层缓冲，固定 256 KB 窗口
**             0x24040000: .dma_no_cache  <- DMA + LVGL 绘制缓冲（不可缓存窗口）
**    这两块由应用代码用绝对地址访问（lv_port_disp.c 的 LCD_FB_ADDR =
**    0x24000000，Core/Src/ltdc.c 的 FBStartAdress = 0x24000000），因此必须
**    落在 SRAM 起始处，并由 MPU region3（0x24000000 1MB，NOT_CACHEABLE）覆盖，
**    保证 DMA2D/LTDC 与 CPU 之间无需 cache maintenance。
**
**  Target      : STMicroelectronics STM32H7B0 (Cortex-M7)
**
**  参考：E:\Projects\You\LLMC\LLMC_DAC\app.ld（同款 MCU 的量产验证布局）
******************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Specify the memory areas */
MEMORY
{
ITCMRAM (xrw)      : ORIGIN = 0x00000000, LENGTH = 64K
DTCMRAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 128K
RAM (xrw)          : ORIGIN = 0x24000000, LENGTH = 1024K
/* D3 域 SRAM4：BDMA2 唯一能访问的存储（HDMA/DMA1/DMA2 都够不到 D3）。
 * LPUART1 挂 D3 域、走 BDMA2，其 DMA 缓冲必须放这里。
 * 容量按 CubeMX MPU region6 的 32KB 填。 */
SRAM4 (xrw)        : ORIGIN = 0x38000000, LENGTH = 32K

/* 外部 OSPI NOR（W25Q128 级别，16 MB）memory-mapped 基址。
 * !! LENGTH 需按实物芯片容量确认：本工程 octospi.c 里 DeviceSize=32 只是
 * CubeMX 占位值；LLMC_DAC_bootloader 的实际芯片为 W25Q128(16MB) -> 16M。
 * 链接只用到约 1 MB，任何 >= 2 MB 的芯片都不会越界。 */
FLASH (rx)         : ORIGIN = 0x90000000, LENGTH = 16M
}

/* Highest address of the user mode stack（栈放在最快的 DTCM 顶部） */
_estack = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM);    /* end of RAM */
_sstack = _estack - _Min_Stack_Size;
/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0x200;      /* required amount of heap  */
_Min_Stack_Size = 0x1000; /* required amount of stack */

/* Define output sections */
SECTIONS
{
  /* ================= 外部 OSPI FLASH (XIP) ================= */

  /* 向量表必须落在 0x90000000：bootloader 从这里取初始 MSP 与 Reset 向量，
   * 运行期 system_stm32h7xx.c 也从这里 memcpy 到 ITCMRAM 并改写 VTOR。 */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >FLASH

  /* The program code and other data goes into FLASH */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH

  /* Constant data goes into FLASH */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >FLASH

  /*user added for Fonts*/
  .font :
  {
    . = ALIGN(4);
    KEEP (*(.font*))
    . = ALIGN(4);
  } >FLASH

  .ARM.extab : /* The "READONLY" keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >FLASH

  .ARM : /* The "READONLY" keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } >FLASH

  .preinit_array : /* The "READONLY" keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .init_array : /* The "READONLY" keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .fini_array : /* The "READONLY" keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4);
  } >FLASH

  /* ================= ITCMRAM ================= */

  /* 向量表在 ITCMRAM 的镜像（NOLOAD：内容由 system_stm32h7xx.c 在上电时
   * 从 0x90000000 拷入；中断响应最快）。 */
  .isr_vector_ram (NOLOAD) :
  {
    . = ALIGN(4);
    _isr_vector_ram_start = .;
    KEEP(*(.isr_vector_ram))
    . = ALIGN(4);
    _isr_vector_ram_end = .;
  } >ITCMRAM

  /* ================= DTCMRAM（数据/堆栈） ================= */

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections goes into DTCMRAM, load LMA copy after code */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */
    *(.RamFunc)        /* .RamFunc sections */
    *(.RamFunc*)       /* .RamFunc* sections */

    . = ALIGN(4);
  } >DTCMRAM AT> FLASH

 /* Initialized TLS data section */
  .tdata : ALIGN(4)
  {
    *(.tdata .tdata.* .gnu.linkonce.td.*)
    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
    PROVIDE(__data_end = .);
    PROVIDE(__tdata_end = .);
  } >DTCMRAM AT> FLASH

  PROVIDE( __tdata_start = ADDR(.tdata) );
  PROVIDE( __tdata_size = __tdata_end - __tdata_start );

  PROVIDE( __data_start = ADDR(.data) );
  PROVIDE( __data_size = __data_end - __data_start );

  PROVIDE( __tdata_source = LOADADDR(.tdata) );
  PROVIDE( __tdata_source_end = LOADADDR(.tdata) + SIZEOF(.tdata) );
  PROVIDE( __tdata_source_size = __tdata_source_end - __tdata_source );

  PROVIDE( __data_source = LOADADDR(.data) );
  PROVIDE( __data_source_end = __tdata_source_end );
  PROVIDE( __data_source_size = __data_source_end - __data_source );

  /* Uninitialized data section */
  .tbss (NOLOAD) : ALIGN(4)
  {
     /* This is used by the startup in order to initialize the .bss secion */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.tbss .tbss.*)
    . = ALIGN(4);
    PROVIDE( __tbss_end = . );
  } >DTCMRAM

  PROVIDE( __tbss_start = ADDR(.tbss) );
  PROVIDE( __tbss_size = __tbss_end - __tbss_start );
  PROVIDE( __tbss_offset = ADDR(.tbss) - ADDR(.tdata) );

  PROVIDE( __tls_base = __tdata_start );
  PROVIDE( __tls_end = __tbss_end );
  PROVIDE( __tls_size = __tls_end - __tls_base );
  PROVIDE( __tls_align = MAX(ALIGNOF(.tdata), ALIGNOF(.tbss)) );
  PROVIDE( __tls_size_align = (__tls_size + __tls_align - 1) & ~(__tls_align - 1) );
  PROVIDE( __arm32_tls_tcb_offset = MAX(8, __tls_align) );
  PROVIDE( __arm64_tls_tcb_offset = MAX(16, __tls_align) );

  .bss (NOLOAD) : ALIGN(4)
  {
    *(.bss)
    *(.bss*)
    *(COMMON)

      . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
      PROVIDE( __bss_end = .);
  } >DTCMRAM
  PROVIDE( __non_tls_bss_start = ADDR(.bss) );

  PROVIDE( __bss_start = __tbss_start );
  PROVIDE( __bss_size = __bss_end - __bss_start );

  /* LVGL 自带的内存池数组（lv_conf.h：LV_MEM_ATTR = section(".CCMRAM_heap")）。
   * 只被 CPU 访问，放最快的 DTCM。 */
  .CCMRAM_heap (NOLOAD) :
  {
    . = ALIGN(8);
    _sCCMRAM_heap = .;
    KEEP(*(.CCMRAM_heap))
    . = ALIGN(8);
    _eCCMRAM_heap = .;
  } >DTCMRAM

  /* User_heap_stack section, used to check that there is enough RAM left */
  ._user_heap_stack (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >DTCMRAM

  /* ================= AXI SRAM（framebuffer / DMA） ================= */

  /* LTDC 层缓冲：固定钉在 SRAM 起始 0x24000000，与 lv_port_disp.c 的
   * LCD_FB_ADDR 和 Core/Src/ltdc.c 的 FBStartAdress 一致。
   * 480 x 272 x 2 = 261,120 B，取整预留 256 KB（262,144 B）。
   * NOLOAD：不占 FLASH，内容由 LTDC/DMA2D 自行写入。 */
  .framebuffer (NOLOAD) :
  {
    . = ORIGIN(RAM);
    _framebuffer_start = .;
    KEEP(*(.framebuffer))
    . = ORIGIN(RAM) + 256K;
    _framebuffer_end = .;
  } >RAM

  /* AXI SRAM 不可缓存窗口（D1 域；DMA1/DMA2/DMA2D/LTDC/USB-OTG 都能访问）：
   *   - .dma_buffer    : user_adc.c 的 ADC DMA 缓冲、radio_uart.c 的 CAT 双缓冲
   *   - .lvgl_buffer   : lv_port_disp.c 的两块局部绘制缓冲（CPU 写、DMA2D 读）
   *   - .usb_dma_buffer: USB CDC 收发缓冲（USB OTG HS 的内部 DMA 读写）
   * 32 字节对齐；由 MPU region4 标为不可缓存，无需手动 cache maintenance。
   * 注意：本段的缓冲都是 NOLOAD，不会被 startup 赋初值/清零——需要初值的
   *       变量请放 .data，或在运行时显式赋值（参见 alc_pwm.c 的 BSRR 载荷）。 */
  .dma_no_cache (NOLOAD) :
  {
    . = ALIGN(32);
    _dma_no_cache_start = .;
    KEEP(*(.dma_buffer))
    KEEP(*(.lvgl_buffer))
    KEEP(*(.usb_dma_buffer))
    KEEP(*(.dma_no_cache))
    . = ALIGN(32);
    . = ORIGIN(RAM) + 512K;
    _dma_no_cache_end = .;
  } >RAM
  
  .RAM (NOLOAD) :
  {
    . = ALIGN(32);
  
    KEEP(*(.ram))
    KEEP(*(.RAM))
    . = ALIGN(32);

  } >RAM
  

  
  /* D3 域 SRAM4：BDMA2 在 D3 域，只能访问 SRAM4/Backup SRAM，够不到 AXI SRAM
   * 与 DTCM。LPUART1 挂 D3 域、其 DMA 是 BDMA2，所以 LPUART 的 DMA 缓冲必须
   * 放这里。NOLOAD（无需初值），由 MPU region6 标为不可缓存。 */
  .d3_dma_buffer (NOLOAD) :
  {
    . = ALIGN(32);
    _d3_dma_buffer_start = .;
    KEEP(*(.d3_dma_buffer))
    . = ALIGN(32);
    _d3_dma_buffer_end = .;
  } >SRAM4

  /* Remove information from the standard libraries（保留，勿删：去掉后 libc
   * 的重复 section 会一起进镜像，导致 OSPI 占用暴涨） */
  /DISCARD/ :
  {
    libc.a:* ( * )
    libm.a:* ( * )
    libgcc.a:* ( * )
  }

  /* ================= 布局断言 ================= */
  /* framebuffer 与 dma 窗口都必须留在 MPU region3（0x24000000 起 1 MB）
   * 的不可缓存区内；一旦越界，DMA2D/LTDC 就可能读到陈旧缓存。 */
  ASSERT(_framebuffer_start == ORIGIN(RAM), "framebuffer 必须从 SRAM 起始 0x24000000 开始")
  ASSERT((_framebuffer_end - _framebuffer_start) == 256K, "framebuffer 预留窗口应为 256 KB")
  ASSERT(_dma_no_cache_start >= _framebuffer_end, "dma_no_cache 不能覆盖 framebuffer")
  ASSERT(_dma_no_cache_end <= ORIGIN(RAM) + 512K, "dma_no_cache 超出 512 KB 不可缓存窗口")
  ASSERT(_d3_dma_buffer_start == ORIGIN(SRAM4), "d3_dma_buffer 必须从 SRAM4 起始开始")
  ASSERT(_d3_dma_buffer_end <= ORIGIN(SRAM4) + LENGTH(SRAM4), "d3_dma_buffer 超出 SRAM4（按 32KB 计）")
  ASSERT(LOADADDR(.data) >= ORIGIN(FLASH), "data 的 LMA 必须落在 OSPI FLASH")
}
