#include "lv_port_disp.h"

#ifndef LVGL_SIM
#include "dma2d.h"
#include "ltdc.h"

/* Panel geometry.  Taken from the LTDC layer that CubeMX generated
 * (Core/Src/ltdc.c: WindowX1 480, WindowY1 272, RGB565, FB at 0x24000000)
 * rather than from lv_conf.h's LV_HOR_RES_MAX, which is only an upper bound
 * the library allocates against.  LV_HOR_RES / LV_VER_RES are runtime macros
 * (they call into the display driver) and cannot be used here - the buffers
 * below are sized at compile time and before any driver is registered. */
#define LCD_WIDTH       480
#define LCD_HEIGHT      272
#define LCD_FB_ADDR     0x24000000UL

/* 双缓冲局部刷新：每块 480 x 27 x 2 = 25,920 B，两块共 51,840 B。
 * 放在 AXI SRAM 的 .dma_no_cache 不可缓存窗口（app.ld，0x24040000 起），
 * 因为 CPU 写、DMA2D 读，非缓存才能免去手动 cache clean。 */
#define LVGL_BUF_HEIGHT  27
#define LVGL_BUF_PIXELS  (LCD_WIDTH * LVGL_BUF_HEIGHT)

static uint16_t __attribute__((section(".lvgl_buffer"))) lvgl_draw_buf1[LVGL_BUF_PIXELS];
static uint16_t __attribute__((section(".lvgl_buffer"))) lvgl_draw_buf2[LVGL_BUF_PIXELS];

static lv_disp_buf_t disp_buf;
static lv_disp_drv_t disp_drv;

/* DMA2D transfer state */
static volatile bool dma2d_busy = false;       /* DMA2D hardware is actively transferring */
static volatile int  dma_pending_count = 0;    /* number of DMA transfers still in flight */

/*
 * DMA2D transfer-complete callback.
 * Marks DMA2D as free. When the LAST area's DMA finishes
 * (counter reaches 0), signals LVGL that the display is ready.
 */
static void dma2d_transfer_done(void)
{
    dma2d_busy = false;
    if (dma_pending_count > 0) {
        dma_pending_count--;
    }
    if (dma_pending_count == 0) {
        lv_disp_flush_ready(&disp_drv);
    }
}

void DMA2D_XferCpltCallback(DMA2D_HandleTypeDef *hdma2d)
{
    (void)hdma2d;
    dma2d_transfer_done();
}

void DMA2D_ErrorCallback(DMA2D_HandleTypeDef *hdma2d)
{
    (void)hdma2d;
    /* On error, release the bus so we don't hang in the spin loop */
    dma2d_transfer_done();
}

static void disp_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p)
{
    /* Wait for any previous DMA2D transfer to finish before reprogramming.
     * DMA2D_IRQn (priority 6) preempts this task, so the ISR clears
     * dma2d_busy and breaks the loop. A timeout guards against DMA errors. */
    uint32_t timeout = 1000000;
    while (dma2d_busy && --timeout) { }
    if (!timeout) {
        dma2d_busy = false;
        dma_pending_count = 0;
    }

    uint16_t area_w = lv_area_get_width(area);
    uint16_t area_h = lv_area_get_height(area);
    uint16_t line_offset = LCD_WIDTH - area_w;

    uint32_t dst_addr = LCD_FB_ADDR
                      + ((uint32_t)area->y1 * LCD_WIDTH + area->x1) * sizeof(uint16_t);

    hdma2d.Init.OutputOffset = line_offset;
    hdma2d.LayerCfg[1].InputOffset = line_offset;

    dma2d_busy = true;
    dma_pending_count++;
    HAL_DMA2D_Start_IT(&hdma2d, (uint32_t)color_p, dst_addr, area_w, area_h);

    if (!lv_disp_flush_is_last(drv)) {
        /* Non-last: LVGL can prepare the next area while DMA runs in parallel */
        lv_disp_flush_ready(drv);
    }
    /* Last: flush_ready is called from DMA2D_XferCpltCallback */
}
#endif /* LVGL_SIM */

void lv_port_disp_init(void)
{
#ifndef LVGL_SIM
    /* Wire up DMA2D callbacks */
    hdma2d.XferCpltCallback = DMA2D_XferCpltCallback;
    hdma2d.XferErrorCallback = DMA2D_ErrorCallback;

    /* Raise DMA2D IRQ priority to 1 so it is NOT masked by FreeRTOS BASEPRI
     * during critical sections, preventing the spin-wait from hanging.
     * DMA2D_IRQn is safe at 1 - the ISR only sets flags, no FreeRTOS APIs. */
    HAL_NVIC_SetPriority(DMA2D_IRQn, 1, 0);

    /* Double buffering: LVGL renders into one buffer while DMA copies from the other */
    lv_disp_buf_init(&disp_buf, lvgl_draw_buf1, lvgl_draw_buf2, LVGL_BUF_PIXELS);

    lv_disp_drv_init(&disp_drv);
    disp_drv.buffer   = &disp_buf;
    disp_drv.flush_cb = disp_flush_cb;
    disp_drv.hor_res  = LCD_WIDTH;
    disp_drv.ver_res  = LCD_HEIGHT;
    lv_disp_drv_register(&disp_drv);
#else
    /* Simulator provides its own display driver */
#endif
}