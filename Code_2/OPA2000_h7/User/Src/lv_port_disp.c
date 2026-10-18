#include "lv_port_disp.h"

#ifndef LVGL_SIM
#include "dma2d.h"
#include "ltdc.h"
#include "FreeRTOS.h"
#include "task.h"     /* taskYIELD() - disp_wait_cb() */

/* Panel geometry.  Taken from the LTDC layer that CubeMX generated
 * (Core/Src/ltdc.c: WindowX1 480, WindowY1 272, RGB565, FB at 0x24000000)
 * rather than from lv_conf.h's LV_HOR_RES_MAX, which is only an upper bound
 * the library allocates against.  LV_HOR_RES / LV_VER_RES are runtime macros
 * (they call into the display driver) and cannot be used here - the buffers
 * below are sized at compile time and before any driver is registered. */


/*============================================================================
 *  LTDC framebuffer - the single full-screen copy LTDC scans out
 *
 *  480 x 272 x 2 B = 261,120 B, which is what a 16-bit RGB565 layer needs.
 *  The section is NOLOAD in app.ld, so this costs no FLASH and no .bss init;
 *  LTDC reads it and DMA2D writes it, nothing else does.
 *
 *  It has to be a real object rather than a bare address constant: taking
 *  its address here is what pins the array into the .framebuffer section,
 *  and every other module that needs the framebuffer address takes it from
 *  here instead of repeating the literal.  A literal duplicated across files
 *  is exactly the kind of thing that silently drifts out of sync with the
 *  linker script.
 *
 *  Cacheability: app.ld reserves this as part of the D1 non-cacheable
 *  window because DMA2D writes it behind the CPU's back on the D1 bus - a
 *  write-back cache line that DMA2D never invalidates would make LTDC read
 *  stale pixels.  See the MPU region notes in Core/Src/main.c.
 *==========================================================================*/
uint16_t __attribute__((section(".framebuffer"))) lcd_framebuffer[LCD_WIDTH * LCD_HEIGHT];

/* 双缓冲局部刷新：每块 480 x 34 x 2 = 32,640 B，两块共 65,280 B。
 * 34 = 272 / 8，即 1/8 屏高（设计要求 OPA2000_H7_固件设计要求.txt 第 5 行）。
 * 放在 AXI SRAM 的 .dma_no_cache 不可缓存窗口（app.ld，0x24040000 起），
 * 因为 CPU 写、DMA2D 读，非缓存才能免去手动 cache clean。 */
#define LVGL_BUF_HEIGHT  (34 * 2)
#define LVGL_BUF_PIXELS  (LCD_WIDTH * LVGL_BUF_HEIGHT)

static uint16_t __attribute__((section(".RAM"))) lvgl_draw_buf1[LVGL_BUF_PIXELS];
static uint16_t __attribute__((section(".RAM"))) lvgl_draw_buf2[LVGL_BUF_PIXELS];

static lv_disp_buf_t disp_buf;
static lv_disp_drv_t disp_drv;

/* DMA2D transfer state.  Both are written by the DMA2D ISR and read by the
 * GUI task, hence volatile. */
static volatile bool dma2d_busy = false;   /* a transfer is in flight */

/*
 * DMA2D transfer-complete callback - runs in the DMA2D ISR.
 *
 * It does nothing but clear the flag.  In particular it must NOT call
 * lv_disp_flush_ready(): LVGL 7 is not reentrant and not ISR-safe, and
 * the GUI task is almost always inside lv_task_handler() at this point.
 * Releasing LVGL is the task's job - see disp_wait_cb() below.
 */
static void dma2d_transfer_done(void)
{
    dma2d_busy = false;
}

void DMA2D_XferCpltCallback(DMA2D_HandleTypeDef *hdma2d)
{
    (void)hdma2d;
    dma2d_transfer_done();
}

void DMA2D_ErrorCallback(DMA2D_HandleTypeDef *hdma2d)
{
    (void)hdma2d;
    /* Release the bus so the wait loop below can make progress; the frame
     * is simply dropped.  dma2d_busy = false either way. */
    dma2d_transfer_done();
}

/*
 * LVGL's "the display is still busy, please wait" hook.
 *
 * LVGL 7 requires this whenever flush_cb is asynchronous: lv_refr.c waits
 * `while(vdb->flushing)` and calls wait_cb to break the wait.  Leaving
 * wait_cb NULL does not mean "no wait" - it means LVGL busy-spins in an
 * empty loop with no way to make progress, which on a lost DMA2D
 * completion is an unbreakable hang.
 *
 * This runs in TASK context (GUI task), which is the whole point: it is
 * the only place from which it is legal to touch LVGL's state.
 */
static void disp_wait_cb(lv_disp_drv_t *drv)
{
    /* DMA2D_IRQn is priority 1, well above configLIBRARY_MAX_SYSCALL_
     * INTERRUPT_PRIORITY (5), so it preempts this loop no matter what
     * FreeRTOS critical section we are inside, and clears dma2d_busy.
     * taskYIELD() lets same-priority tasks run meanwhile.
     *
     * This loop must NOT be commented out or short-circuited: `flushing`
     * stays set after the last area's flush_cb returns, and it is
     * lv_disp_flush_ready() below that clears it.  Return early and LVGL
     * spins on `while(vdb->flushing)` forever with nobody left to clear
     * the flag - the screen freezes on the next repaint and never
     * recovers. */
    int timeout = 100;
    while (dma2d_busy && (timeout > 0)) {
        taskYIELD();
        timeout--;
    }
    dma2d_busy = false;
    lv_disp_flush_ready(drv);
}

static void DMA2D_MemCopy(uint32_t pixelFormat, void * pSrc, void * pDst, int xSize, int ySize, int lineoffset_src, int lineoffset_dst)
{
    /* DMA2D配置 */
    DMA2D->CR      = 0x00000000UL;
    DMA2D->FGMAR   = (uint32_t)pSrc;
    DMA2D->OMAR    = (uint32_t)pDst;
    DMA2D->FGOR    = lineoffset_src;
    DMA2D->OOR     = lineoffset_dst;
    DMA2D->FGPFCCR = pixelFormat;
    DMA2D->NLR     = (uint32_t)(xSize << 16) | (uint16_t)ySize;

    /* 打开中断 */
    DMA2D->CR |= DMA2D_CR_TCIE | DMA2D_CR_TEIE | DMA2D_CR_CEIE;

    /* 启动传输 */
    DMA2D->CR   |= DMA2D_CR_START;

//    /* 等待DMA2D传输完成 */
//    while (DMA2D->CR & DMA2D_CR_START) {}
}


static void disp_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p)
{
    /* Wait for the PREVIOUS transfer to land before reprogramming DMA2D.
     *
     * LVGL does not wait between non-last areas (it only waits at the top of
     * lv_refr_vdb_flush(), and only if flushing is still set), so two areas
     * can be in flight back to back.  Without this wait the second call
     * would overwrite NLR/OMAR/FGPFCCR while the first transfer is still
     * reading them.
     *
     * The bound is a safety net, not a normal path: DMA2D_IRQn is priority
     * 1, far above configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5), so it
     * preempts this loop and clears dma2d_busy within a few microseconds.
     * Reaching the bound means the completion interrupt was lost, and the
     * only way out is to stop pretending a transfer is running.
     */
    uint32_t timeout = 1000000U;
    while (dma2d_busy && (timeout > 0U)) {
        timeout--;
    }
    if (timeout == 0U) {
        dma2d_busy = false;
    }

    uint16_t area_w = lv_area_get_width(area);
    uint16_t area_h = lv_area_get_height(area);

    /* Offsets are in PIXELS, and they apply only to the destination.
     *
     * The source (color_p) is LVGL's own draw buffer, packed with no gaps:
     * every row is exactly area_w pixels wide.  Giving it a skip would make
     * DMA2D step over data that is not there, so it stays 0.
     *
     * The destination is the framebuffer, whose rows really are LCD_WIDTH
     * pixels wide, so a partial area must skip the remainder of each line -
     * that is the whole point of OutputOffset. */
    uint16_t dst_line_offset = LCD_WIDTH - area_w;
    // note: framebuffer2 is cached, must invalidate first before DMA2D
    SCB_CleanDCache_by_Addr((uint8_t*)color_p, lv_area_get_size(area) * sizeof(uint16_t));
//    SCB_CleanDCache_by_Addr((uint8_t*)color_p, sizeof(lvgl_draw_buf2));
    uint32_t dst_addr = (uint32_t)&lcd_framebuffer[area->y1 * LCD_WIDTH + area->x1];
    DMA2D_MemCopy(LTDC_PIXEL_FORMAT_RGB565, (uint16_t*)color_p, (uint16_t*)dst_addr, area_w, area_h, 0, dst_line_offset);

    dma2d_busy = true;


    if (!lv_disp_flush_is_last(drv)) {
        /* Non-last: LVGL moves on to the next area immediately and renders
         * it into the OTHER draw buffer, so this DMA runs in parallel with
         * the CPU.  That is the whole point of double buffering, and it is
         * only sound because the two buffers are distinct (disp_buf holds
         * two non-NULL pointers). */
        lv_disp_flush_ready(drv);
    }
    /* Last: flushing stays set on purpose.  LVGL's next
     * `while(vdb->flushing)` calls disp_wait_cb(), which waits for the DMA
     * in TASK context and then releases LVGL.  Doing it here instead would
     * mean blocking the GUI task for the duration of the copy; doing it in
     * the ISR would mean touching LVGL from interrupt context, which LVGL 7
     * does not support. */
}
#endif /* LVGL_SIM */

void lv_port_disp_init(void)
{
#ifndef LVGL_SIM
    /* Wire up DMA2D callbacks.  Both only clear dma2d_busy - releasing LVGL
     * is disp_wait_cb()'s job, in task context. */
    hdma2d.XferCpltCallback = DMA2D_XferCpltCallback;
    hdma2d.XferErrorCallback = DMA2D_ErrorCallback;

    /* DMA2D_IRQn priority 1, far above configLIBRARY_MAX_SYSCALL_INTERRUPT_
     * PRIORITY (5).  It therefore preempts the GUI task even inside a
     * FreeRTOS critical section, which is what lets disp_wait_cb() and the
     * wait at the top of disp_flush_cb() always make progress.  Safe at this
     * level because the ISR touches no FreeRTOS API. */
    HAL_NVIC_SetPriority(DMA2D_IRQn, 1, 0);

    /* Double buffering: LVGL renders into one buffer while DMA copies from
     * the other.  Both pointers non-NULL is what selects this mode in v7. */
    lv_disp_buf_init(&disp_buf, lvgl_draw_buf1, lvgl_draw_buf2, LVGL_BUF_PIXELS);

    lv_disp_drv_init(&disp_drv);
    disp_drv.buffer   = &disp_buf;
    disp_drv.flush_cb = disp_flush_cb;
    disp_drv.wait_cb  = disp_wait_cb;
    disp_drv.hor_res  = LCD_WIDTH;
    disp_drv.ver_res  = LCD_HEIGHT;
    lv_disp_drv_register(&disp_drv);
#else
    /* Simulator provides its own display driver */
#endif
}
