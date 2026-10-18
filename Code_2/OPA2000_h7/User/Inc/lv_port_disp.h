#ifndef LV_PORT_DISP_H
#define LV_PORT_DISP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"
#define LCD_WIDTH       480
#define LCD_HEIGHT      272
/*============================================================================
 *  LTDC framebuffer - 480 x 272 x 16 bit, RGB565
 *
 *  The one full-screen buffer LTDC scans out.  It is declared (not merely
 *  reserved) so that every module needing its address takes it from this
 *  object: Core/Src/ltdc.c points FBStartAdress here, and lv_port_disp.c
 *  uses it as the DMA2D destination.  A bare address literal repeated in
 *  both places would drift out of sync with app.ld the first time the
 *  linker layout changed.
 *
 *  It lives in the .framebuffer section, which app.ld pins to the start of
 *  AXI SRAM (0x24000000, 256 KB) and marks NOLOAD.
 *==========================================================================*/
extern uint16_t lcd_framebuffer[LCD_WIDTH * LCD_HEIGHT];

void lv_port_disp_init(void);

#ifdef __cplusplus
}
#endif

#endif /* LV_PORT_DISP_H */
