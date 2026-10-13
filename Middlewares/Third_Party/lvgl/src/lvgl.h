/**
 * @file lvgl.h
 * @brief LVGL 7.x main header - includes all core headers
 */

#ifndef LVGL_H
#define LVGL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Core */
#include "lv_core/lv_obj.h"
#include "lv_core/lv_style.h"
#include "lv_core/lv_group.h"
#include "lv_core/lv_debug.h"

/* HAL */
#include "lv_hal/lv_hal.h"
#include "lv_hal/lv_hal_disp.h"
#include "lv_hal/lv_hal_indev.h"
#include "lv_hal/lv_hal_tick.h"

/* Widgets */
#include "lv_widgets/lv_btn.h"
#include "lv_widgets/lv_img.h"
#include "lv_widgets/lv_label.h"
#include "lv_widgets/lv_line.h"
#include "lv_widgets/lv_cont.h"
#include "lv_widgets/lv_bar.h"
#include "lv_widgets/lv_slider.h"
#include "lv_widgets/lv_led.h"
#include "lv_widgets/lv_checkbox.h"
#include "lv_widgets/lv_switch.h"
#include "lv_widgets/lv_chart.h"
#include "lv_widgets/lv_gauge.h"
#include "lv_widgets/lv_arc.h"
#include "lv_widgets/lv_spinbox.h"
#include "lv_widgets/lv_calendar.h"
#include "lv_widgets/lv_cpicker.h"
#include "lv_widgets/lv_imgbtn.h"
#include "lv_widgets/lv_list.h"
#include "lv_widgets/lv_msgbox.h"
#include "lv_widgets/lv_dropdown.h"
#include "lv_widgets/lv_btnmatrix.h"
#include "lv_widgets/lv_keyboard.h"
#include "lv_widgets/lv_textarea.h"
#include "lv_widgets/lv_page.h"
#include "lv_widgets/lv_win.h"
#include "lv_widgets/lv_tabview.h"
#include "lv_widgets/lv_tileview.h"
#include "lv_widgets/lv_canvas.h"
#include "lv_widgets/lv_table.h"
#include "lv_widgets/lv_objmask.h"
#include "lv_widgets/lv_roller.h"
#include "lv_widgets/lv_spinner.h"
#include "lv_widgets/lv_linemeter.h"

/* Draw */
#include "lv_draw/lv_draw.h"

/* Misc */
#include "lv_misc/lv_mem.h"
#include "lv_misc/lv_log.h"
#include "lv_misc/lv_math.h"
#include "lv_misc/lv_txt.h"
#include "lv_misc/lv_fs.h"
#include "lv_misc/lv_ll.h"
#include "lv_misc/lv_color.h"
#include "lv_misc/lv_utils.h"
#include "lv_misc/lv_gc.h"

/* Themes */
#include "lv_themes/lv_theme.h"

#ifdef __cplusplus
}
#endif

#endif /* LVGL_H */