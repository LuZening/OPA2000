/**
 * @file gui.c
 * @brief LVGL GUI Implementation — OPA1000-style layout
 */

#include "gui.h"
#include "lv_port_disp.h"
#include <math.h>
#ifndef LVGL_SIM
#include "config.h"
#include "touch.h"
#include "power_control.h"
#endif
#include "string.h"
#include "stdio.h"

/*============================================================================
 * Private Variables
 *============================================================================*/

/* Screen dimensions — 800x480 landscape */
#define HOR_RES  800
#define VER_RES  480

/* Layout heights */
#define BANNER_H    (VER_RES / 10)
#define FWD_PWR_H   (VER_RES / 4)
#define MID_ROW_H   (VER_RES / 6)
#define INFO_H      (VER_RES / 8 * 3)
#define STATUS_H    (VER_RES / 8)
/* Total = 48 + 120 + 80 + 180 + 52 = 480 */

/* Progress bar ranges */
#define BAR_MAX_FWD    1000
#define BAR_MAX_REF    300
#define BAR_MAX_SWR    1000

/* String IDs */
enum {
    STR_IDLE = 0,
    STR_STANDBY,
    STR_TRANSMIT,
    STR_TX_POWER,
    STR_REF_POWER,
    STR_SWR,
    STR_VOLTAGE,
    STR_CURRENT,
    STR_TEMPERATURE,
    STR_BAND,
    STR_ALERT,
    STR_SWR_HIGH,
    STR_REF_HIGH,
    STR_CURRENT_HIGH,
    STR_HARDWARE_FAULT,
    STR_OVERDRIVE,
    STR_FREQ_MISMATCH,
    STR_DISMISS,
    STR_CANCEL,
    STR_CALIBRATION,
    STR_CAL_POINT,
    STR_CAL_SUCCESS,
    STR_CAL_FAILED,
    STR_SETTINGS,
    STR_LANGUAGE,
    STR_POWER,
    STR_FAN,
    STR_CAL_RAW,
    STR_CAL_RESIDUAL,
    STR_CAL_ACCEPT,
    STR_CAL_RETRY,
    STR_SCREEN,
    STR_TOUCH_CAL,
    STR_CAL_SENSORS,
    STR_DIR_COUPLER_1,
    STR_DIR_COUPLER_2,
    STR_START_MEAS,
    STR_SENSOR_INSPECT,
    STR_AUTO,
    STR_SAVE,
};

/* Multi-language strings */
static const char* gui_strings[2][40] = {
    /* Chinese */
    {
        "待机", "预备", "发射",
        "发射功率", "反射功率", "驻波比", "电压", "电流", "温度", "波段",
        "警报", "驻波比过高", "反射功率过高", "电流过高", "硬件故障", "过驱动", "频率不匹配",
        "解除", "取消", "校准", "校准点", "校准成功", "校准失败", "设置", "语言", "电源",
        "风扇",
        "原始坐标", "残差", "确认", "重试",
        "屏幕", "触屏校准",
        "校准传感器", "耦合器1", "耦合器2", "开始测量",
        "传感器检查", "自动", "保存",
    },
    /* English */
    {
        "Idle", "Standby", "Transmit",
        "TX Power", "Reflected", "SWR", "Voltage", "Current", "Temperature", "Band",
        "Alert", "SWR High", "Reflected High", "Current High", "Hardware Fault", "Overdrive", "Freq Mismatch",
        "Dismiss", "Cancel", "Calibration", "Calibrate Point", "Calibrate Success", "Calibrate Failed", "Settings", "Language", "Power",
        "Fan",
        "Raw Coordinate", "Residual", "Accept", "Retry",
        "Screen", "Touch Calibrate",
        "Calibrate Sensors", "Coupler 1", "Coupler 2", "Start",
        "Sensor Inspect", "Auto", "Save",
    }
};

#define COLOR_BG         LV_COLOR_MAKE(37, 40, 42)
#define COLOR_BANNER     LV_COLOR_BLUE
#define COLOR_BORDER     LV_COLOR_MAKE(55, 58, 60)
#define COLOR_BAR_BG     LV_COLOR_MAKE(0x30, 0x30, 0x30)
#define COLOR_BAR_BAD    LV_COLOR_RED

static LanguageType current_language = MULTILANG_ENGLISH;

/* LVGL objects — main screen widgets */
static lv_obj_t *main_screen;
static lv_obj_t *banner;
static lv_obj_t *bottom_bar;
static lv_obj_t *pwr_btn;
static lv_obj_t *status_label;
static lv_obj_t *band_label;

/* Progress bars */
static lv_obj_t *bar_fwd;
static lv_obj_t *bar_ref;
static lv_obj_t *bar_swr;

/* Numeric value labels alongside bars */
static lv_obj_t *val_fwd;
static lv_obj_t *val_ref;
static lv_obj_t *val_swr;

/* Section title labels (need updating on language change) */
static lv_obj_t *lbl_tx_title;
static lv_obj_t *lbl_ref_title;
static lv_obj_t *lbl_swr_title;

/* Bottom status bar labels */
static lv_obj_t *lbl_voltage;
static lv_obj_t *lbl_voltage_cap;
static lv_obj_t *lbl_current;
static lv_obj_t *lbl_current_cap;
static lv_obj_t *lbl_temp;
static lv_obj_t *lbl_temp_cap;
static lv_obj_t *lbl_fan;
static lv_obj_t *lbl_fan_cap;
static lv_obj_t *lbl_settings_btn;
static lv_obj_t *lbl_radio_mode;

/* Misc */
static lv_obj_t *alert_msgbox;
static lv_obj_t *cal_screen;
static uint8_t current_screen = 0;
static bool alert_active = false;
static bool calibrating = false;

/* Calibration state */
#define CAL_POINT_COUNT 5
static uint8_t cal_state = 0;        /* 0=idle, 1=showing point, 2=results */
static uint8_t cal_point_idx = 0;
static lv_obj_t *cal_container = NULL;

/* Calibration target positions (10% from edges + center) on 800x480 */
static const lv_point_t cal_targets[CAL_POINT_COUNT] = {
    {HOR_RES / 10,     VER_RES / 10    },  /* top-left     */
    {HOR_RES * 9 / 10, VER_RES / 10    },  /* top-right    */
    {HOR_RES / 10,     VER_RES * 9 / 10},  /* bottom-left  */
    {HOR_RES * 9 / 10, VER_RES * 9 / 10},  /* bottom-right */
    {HOR_RES / 2,      VER_RES / 2     },  /* center       */
};

/* Calibration raw-data buffer */
static struct {
    uint16_t raw_x;
    uint16_t raw_y;
    uint16_t screen_x;
    uint16_t screen_y;
} cal_data[CAL_POINT_COUNT];

/* Band selector popup */
static volatile bool is_band_selector_shown = false;
static lv_obj_t *band_selector_msgbox = NULL;
/* btnmatrix map array — static so LVGL's stored pointer stays valid */
static const char *band_btns[8];

/*============================================================================
 * Private Functions
 *============================================================================*/

static uint8_t gui_get_lang_index(void)
{
    return (current_language == MULTILANG_CHINESE) ? 0 : 1;
}

/**
 * @brief Create a styled container
 */
static lv_obj_t* make_cont(lv_obj_t *parent, lv_coord_t x, lv_coord_t y,
                            lv_coord_t w, lv_coord_t h, lv_color_t bg)
{
    lv_obj_t *c = lv_obj_create(parent, NULL);
    lv_obj_set_pos(c, x, y);
    lv_obj_set_size(c, w, h);
    lv_obj_set_style_local_bg_color(c, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, bg);
    lv_obj_set_style_local_border_width(c, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, 0);
    lv_obj_set_style_local_pad_top(c, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, 0);
    lv_obj_set_style_local_pad_bottom(c, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, 0);
    lv_obj_set_style_local_pad_left(c, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, 0);
    lv_obj_set_style_local_pad_right(c, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, 0);
    return c;
}

/**
 * @brief Create a label inside parent with given text and position
 */
static lv_obj_t* make_label(lv_obj_t *parent, const char *text,
                             lv_coord_t x, lv_coord_t y,
                             lv_color_t color, const lv_font_t *font)
{
    lv_obj_t *l = lv_label_create(parent, NULL);
    lv_label_set_text(l, text);
    lv_obj_set_pos(l, x, y);
    lv_obj_set_style_local_text_color(l, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, color);
    if (font) lv_obj_set_style_local_text_font(l, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, font);
    return l;
}

/**
 * @brief Create a progress bar
 */
static lv_obj_t* make_bar(lv_obj_t *parent, lv_coord_t x, lv_coord_t y,
                           lv_coord_t w, lv_coord_t h, lv_coord_t range_max)
{
    lv_obj_t *bar = lv_bar_create(parent, NULL);
    lv_obj_set_pos(bar, x, y);
    lv_obj_set_size(bar, w, h);
    lv_bar_set_range(bar, 0, range_max);
    lv_bar_set_value(bar, 0, LV_ANIM_OFF);

    /* Indicator: aqua → orange horizontal gradient, matching OPA1000 */
    lv_obj_set_style_local_bg_color(bar, LV_BAR_PART_INDIC, LV_STATE_DEFAULT, LV_COLOR_AQUA);
    lv_obj_set_style_local_bg_grad_color(bar, LV_BAR_PART_INDIC, LV_STATE_DEFAULT, LV_COLOR_ORANGE);
    lv_obj_set_style_local_bg_grad_dir(bar, LV_BAR_PART_INDIC, LV_STATE_DEFAULT, LV_GRAD_DIR_HOR);

    return bar;
}

/*============================================================================
 * Event Callbacks
 *============================================================================*/

static void gui_power_button_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
        gui_power_button_pressed();
    }
}

static void settings_btn_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
        gui_toggle_menu();
    }
}

/*============================================================================
 * Band Selector Popup
 *============================================================================*/

static void bandsel_event_handler(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED && is_band_selector_shown) {
        uint16_t btn_id = lv_msgbox_get_active_btn(obj);
        if (btn_id < 5) {
#ifndef LVGL_SIM
            switch_band((BandType)(btn_id + 1));
#endif
        }
        is_band_selector_shown = false;
        lv_obj_del_async(obj);
        band_selector_msgbox = NULL;
    }
}

static void show_band_selector(void)
{
    band_btns[0] = "160m";
    band_btns[1] = "80m";
    band_btns[2] = "40-30m";
    band_btns[3] = "20-17m";
    band_btns[4] = "15-10m";
    band_btns[5] = "\n";
    band_btns[6] = gui_get_string(STR_CANCEL);
    band_btns[7] = "";

    lv_obj_t *scr = lv_scr_act();
    band_selector_msgbox = lv_msgbox_create(scr, NULL);
    lv_msgbox_set_text(band_selector_msgbox, gui_get_string(STR_BAND));
    lv_msgbox_add_btns(band_selector_msgbox, band_btns);

    lv_obj_set_size(band_selector_msgbox, HOR_RES * 5 / 6, 180);

    /* === Title: white text, CJK font, blue bg === */
    lv_obj_set_style_local_text_color(band_selector_msgbox, LV_MSGBOX_PART_BG,
                                      LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(band_selector_msgbox, LV_MSGBOX_PART_BG,
                                      LV_STATE_DEFAULT, &MyCJK_16);
    lv_obj_set_style_local_bg_color(band_selector_msgbox, LV_MSGBOX_PART_BG,
                                     LV_STATE_DEFAULT, LV_COLOR_BLUE);

    /* === Style the btnmatrix directly (bypasses msgbox part routing) === */
    lv_obj_t *btnm = lv_msgbox_get_btnmatrix(band_selector_msgbox);
    lv_obj_set_style_local_text_color(btnm, LV_BTNMATRIX_PART_BTN,
                                      LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(btnm, LV_BTNMATRIX_PART_BTN,
                                      LV_STATE_DEFAULT, &MyCJK_16);
    lv_obj_set_style_local_bg_color(btnm, LV_BTNMATRIX_PART_BTN,
                                     LV_STATE_DEFAULT, LV_COLOR_MAKE(0, 160, 225));
    lv_obj_set_style_local_border_color(btnm, LV_BTNMATRIX_PART_BTN,
                                         LV_STATE_DEFAULT, LV_COLOR_GRAY);
    lv_obj_set_style_local_radius(btnm, LV_BTNMATRIX_PART_BTN,
                                   LV_STATE_DEFAULT, 0);

    lv_obj_set_event_cb(band_selector_msgbox, bandsel_event_handler);
    lv_obj_align(band_selector_msgbox, scr, LV_ALIGN_CENTER, 0, -30);
}

static void info_area_event_handle(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED && !is_band_selector_shown) {
        is_band_selector_shown = true;
        show_band_selector();
    }
}

static lv_obj_t* make_status_seg(lv_obj_t *parent, lv_coord_t x,
                                  const char *caption, lv_obj_t **cap_ref)
{
    lv_obj_t *seg = lv_obj_create(parent, NULL);
    lv_obj_set_size(seg, HOR_RES / 5, STATUS_H);
    lv_obj_set_pos(seg, x, 0);
    lv_obj_set_style_local_bg_opa(seg, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_OPA_TRANSP);
    lv_obj_set_style_local_border_width(seg, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, 0);
    lv_obj_set_style_local_pad_top(seg, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, 0);
    lv_obj_set_style_local_pad_bottom(seg, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, 0);
    lv_obj_set_style_local_pad_left(seg, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, 0);
    lv_obj_set_style_local_pad_right(seg, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, 0);
    *cap_ref = make_label(seg, caption, 4, 4, LV_COLOR_WHITE, &MyCJK_16);
    return seg;
}

/**
 * @brief Create the main layout
 */
static void gui_create_main_screen(void)
{
    main_screen = lv_obj_create(NULL, NULL);
    lv_obj_set_size(main_screen, HOR_RES, VER_RES);
    lv_obj_set_style_local_bg_color(main_screen, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, COLOR_BG);

    /*=============================================
     * 1 — Top Banner (48px) — centered status button
     *=============================================*/
    banner = make_cont(main_screen, 0, 0, HOR_RES, BANNER_H, COLOR_BANNER);

    /* Centered clickable button — transparent bg, visible border */
    pwr_btn = lv_btn_create(banner, NULL);
    lv_obj_set_size(pwr_btn, 240, 36);
    lv_obj_set_style_local_bg_opa(pwr_btn, LV_BTN_PART_MAIN, LV_STATE_DEFAULT, LV_OPA_TRANSP);
    lv_obj_set_style_local_border_width(pwr_btn, LV_BTN_PART_MAIN, LV_STATE_DEFAULT, 2);
    lv_obj_set_style_local_border_color(pwr_btn, LV_BTN_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_radius(pwr_btn, LV_BTN_PART_MAIN, LV_STATE_DEFAULT, 4);
    lv_obj_align(pwr_btn, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_event_cb(pwr_btn, gui_power_button_cb);

    status_label = lv_label_create(pwr_btn, NULL);
    lv_label_set_text(status_label, gui_get_string(STR_IDLE));
    lv_obj_set_style_local_text_color(status_label, LV_LABEL_PART_MAIN,
                                       LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(status_label, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &MyCJK_16);
    lv_obj_align(status_label, NULL, LV_ALIGN_CENTER, 0, 0);

    /*=============================================
     * 2 — Forward Power (120px) — title + bar
     *=============================================*/
    lv_obj_t *c_fwd = make_cont(main_screen, 20, BANNER_H + 6,
                                 HOR_RES - 40, FWD_PWR_H, COLOR_BG);
    lv_obj_set_style_local_border_width(c_fwd, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, 1);
    lv_obj_set_style_local_border_color(c_fwd, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, COLOR_BORDER);

    lbl_tx_title = make_label(c_fwd, gui_get_string(STR_TX_POWER),
                              0, 4, LV_COLOR_WHITE, &MyCJK_16);

    val_fwd = make_label(c_fwd, "0W",
                         0, 4, LV_COLOR_WHITE, &lv_font_montserrat_24);
    lv_obj_align(val_fwd, NULL, LV_ALIGN_IN_TOP_RIGHT, -8, 0);

    bar_fwd = make_bar(c_fwd, 0, 36, HOR_RES - 40, 28, BAR_MAX_FWD);

    /*=============================================
     * 3 — Middle Row (80px) — Reflected | SWR
     *=============================================*/
    lv_coord_t mid_y = BANNER_H + FWD_PWR_H + 8;
    lv_coord_t half_w = (HOR_RES - 40) / 2 - 5;

    lv_obj_t *c_ref = make_cont(main_screen, 20, mid_y,
                                  half_w, MID_ROW_H, COLOR_BG);
    lv_obj_set_style_local_border_width(c_ref, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, 1);
    lv_obj_set_style_local_border_color(c_ref, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, COLOR_BORDER);
    lv_obj_t *c_swr = make_cont(main_screen, 30 + half_w, mid_y,
                                  half_w, MID_ROW_H, COLOR_BG);
    lv_obj_set_style_local_border_width(c_swr, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, 1);
    lv_obj_set_style_local_border_color(c_swr, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, COLOR_BORDER);

    lbl_ref_title = make_label(c_ref, gui_get_string(STR_REF_POWER), 0, 2, LV_COLOR_WHITE, &MyCJK_16);
    val_ref = make_label(c_ref, "0W", 0, 2, LV_COLOR_WHITE, &lv_font_montserrat_24);
    lv_obj_align(val_ref, NULL, LV_ALIGN_IN_TOP_RIGHT, -8, 0);
    bar_ref = make_bar(c_ref, 0, 30, half_w - 10, 22, BAR_MAX_REF);

    lbl_swr_title = make_label(c_swr, gui_get_string(STR_SWR), 0, 2, LV_COLOR_WHITE, &MyCJK_16);
    val_swr = make_label(c_swr, "1.0", 0, 2, LV_COLOR_WHITE, &lv_font_montserrat_24);
    lv_obj_align(val_swr, NULL, LV_ALIGN_IN_TOP_RIGHT, -8, 0);
    bar_swr = make_bar(c_swr, 0, 30, half_w - 10, 22, BAR_MAX_SWR);

    /*=============================================
     * 4 — Info Area (180px) — band + frequency
     *=============================================*/
    lv_coord_t info_y = mid_y + MID_ROW_H + 6;
    lv_obj_t *c_info = make_cont(main_screen, 20, info_y,
                                   HOR_RES - 40, INFO_H, COLOR_BG);
    lv_obj_set_style_local_border_width(c_info, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, 1);
    lv_obj_set_style_local_border_color(c_info, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, COLOR_BORDER);
    /* Thin red border when focused (keyboard/encoder navigation) */
    lv_obj_set_style_local_border_width(c_info, LV_OBJ_PART_MAIN, LV_STATE_FOCUSED, 1);
    lv_obj_set_style_local_border_color(c_info, LV_OBJ_PART_MAIN, LV_STATE_FOCUSED, LV_COLOR_RED);

    /* Frequency range — large yellow text (OPA1000-style) */
    band_label = lv_label_create(c_info, NULL);
    lv_label_set_text(band_label, "---");
    lv_label_set_align(band_label, LV_LABEL_ALIGN_CENTER);
    lv_obj_set_style_local_text_color(band_label, LV_LABEL_PART_MAIN,
                                       LV_STATE_DEFAULT, LV_COLOR_YELLOW);
    lv_obj_set_style_local_text_font(band_label, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &lv_font_montserrat_48);
    lv_obj_align(band_label, NULL, LV_ALIGN_CENTER, 0, -20);

    /* Radio/band decoder mode — shown below the frequency */
    lbl_radio_mode = lv_label_create(c_info, NULL);
    lv_label_set_text(lbl_radio_mode, "");
    lv_label_set_align(lbl_radio_mode, LV_LABEL_ALIGN_CENTER);
    lv_obj_set_style_local_text_color(lbl_radio_mode, LV_LABEL_PART_MAIN,
                                       LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(lbl_radio_mode, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &lv_font_montserrat_24);
    lv_obj_align(lbl_radio_mode, NULL, LV_ALIGN_CENTER, 0, 40);

    /* Tap to open band selector */
    lv_obj_set_event_cb(c_info, info_area_event_handle);

    /*=============================================
     * 5 — Bottom Status Bar (52px) — Voltage / Current / Temp / Fan / Menu
     *=============================================*/
    lv_coord_t status_y = VER_RES - STATUS_H;
    bottom_bar = make_cont(main_screen, 0, status_y,
                                     HOR_RES, STATUS_H, COLOR_BANNER);

    lv_coord_t seg_w = HOR_RES / 5;

#define MAKE_STATUS_VAL(var) \
    var = make_label(seg, "-", 4, 26, LV_COLOR_WHITE, &lv_font_montserrat_16);

    lv_obj_t *seg;

    seg = make_status_seg(bottom_bar, seg_w * 0, gui_get_string(STR_VOLTAGE), &lbl_voltage_cap);
    MAKE_STATUS_VAL(lbl_voltage);
    seg = make_status_seg(bottom_bar, seg_w * 1, gui_get_string(STR_CURRENT), &lbl_current_cap);
    MAKE_STATUS_VAL(lbl_current);
    seg = make_status_seg(bottom_bar, seg_w * 2, gui_get_string(STR_TEMPERATURE), &lbl_temp_cap);
    MAKE_STATUS_VAL(lbl_temp);
    seg = make_status_seg(bottom_bar, seg_w * 3, gui_get_string(STR_FAN), &lbl_fan_cap);
    MAKE_STATUS_VAL(lbl_fan);

    /* Segment 4: Menu button */
    {
        lv_obj_t *seg = lv_obj_create(bottom_bar, NULL);
        lv_obj_set_size(seg, seg_w, STATUS_H);
        lv_obj_set_pos(seg, seg_w * 4, 0);
        lv_obj_set_style_local_bg_opa(seg, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_OPA_TRANSP);
        lv_obj_set_style_local_border_width(seg, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, 0);

        lv_obj_t *btn = lv_btn_create(seg, NULL);
        lv_obj_set_size(btn, seg_w - 16, 36);
        lv_obj_align(btn, NULL, LV_ALIGN_CENTER, 0, 0);
        lbl_settings_btn = lv_label_create(btn, NULL);
        lv_label_set_text(lbl_settings_btn, gui_get_string(STR_SETTINGS));
        lv_obj_set_style_local_text_font(lbl_settings_btn, LV_LABEL_PART_MAIN,
                                          LV_STATE_DEFAULT, &MyCJK_16);
        lv_obj_set_event_cb(btn, settings_btn_cb);
    }
#undef MAKE_STATUS_VAL
}

/**
 * @brief Update all live data on the main screen
 */
static void gui_update_main_values(void)
{
    char buf[32];

#ifndef LVGL_SIM
    /* Status */
    lv_color_t banner_color;
    if (g_tx_active) {
        lv_label_set_text(status_label, gui_get_string(STR_TRANSMIT));
        banner_color = LV_COLOR_MAKE(0xdf, 0x01, 0x01);  /* OPA1000 dark red */
    } else if (g_psu_on) {
        lv_label_set_text(status_label, gui_get_string(STR_STANDBY));
        banner_color = LV_COLOR_MAKE(0xea, 0xb9, 0x06);  /* OPA1000 dark yellow */
    } else {
        lv_label_set_text(status_label, gui_get_string(STR_IDLE));
        banner_color = COLOR_BANNER;  /* blue */
    }
    lv_obj_set_style_local_bg_color(banner, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, banner_color);
    lv_obj_set_style_local_bg_color(bottom_bar, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, banner_color);
    /* Text always white on colored banner */
    lv_obj_set_style_local_text_color(status_label, LV_LABEL_PART_MAIN,
                                       LV_STATE_DEFAULT, LV_COLOR_WHITE);

    /* Forward power */
    float fwd = g_adc_data.tx_power2;
    snprintf(buf, sizeof(buf), "%.0fW", fwd);
    lv_label_set_text(val_fwd, buf);
    lv_bar_set_value(bar_fwd, (int16_t)(fwd * BAR_MAX_FWD / 1000.0f), LV_ANIM_OFF);

    /* Reflected power */
    float refp = g_adc_data.ref_power2;
    snprintf(buf, sizeof(buf), "%.0fW", refp);
    lv_label_set_text(val_ref, buf);
    lv_bar_set_value(bar_ref, (int16_t)(refp * BAR_MAX_REF / 300.0f), LV_ANIM_OFF);

    /* SWR */
    float swr = g_adc_data.swr;
    snprintf(buf, sizeof(buf), "%.1f", swr);
    lv_label_set_text(val_swr, buf);
    lv_bar_set_value(bar_swr, (int16_t)((swr - 1.0f) * BAR_MAX_SWR / 3.0f), LV_ANIM_OFF);
    if (swr > 2.0f) {
        lv_obj_set_style_local_text_color(val_swr, LV_LABEL_PART_MAIN,
                                           LV_STATE_DEFAULT, LV_COLOR_RED);
    } else {
        lv_obj_set_style_local_text_color(val_swr, LV_LABEL_PART_MAIN,
                                           LV_STATE_DEFAULT, LV_COLOR_WHITE);
    }

    /* Voltage */
    snprintf(buf, sizeof(buf), "%.1fV", g_adc_data.voltage);
    lv_label_set_text(lbl_voltage, buf);

    /* Current */
    snprintf(buf, sizeof(buf), "%.1fA", g_adc_data.current);
    lv_label_set_text(lbl_current, buf);

    /* Temperature — Celsius + Fahrenheit */
    float tc = g_adc_data.ntc_temp;
    snprintf(buf, sizeof(buf), "%.0fC/%.0fF", tc, tc * 9.0f / 5.0f + 32.0f);
    lv_label_set_text(lbl_temp, buf);

    /* Fan speed */
    const char *fan_str;
    switch (g_fan_speed) {
        case FAN_SPEED_LOW:  fan_str = "Low";  break;
        case FAN_SPEED_MED:  fan_str = "Med";  break;
        case FAN_SPEED_HIGH: fan_str = "High"; break;
        default:             fan_str = "-";    break;
    }
    lv_label_set_text(lbl_fan, fan_str);

    /* Band / Frequency range — indexed from BandType (BAND_160M=1, etc.) */
    if (g_current_band > BAND_NONE && g_current_band <= BAND_15M_10M) {
        lv_label_set_text(band_label, strFreqRanges[g_current_band - 1]);
    } else {
        lv_label_set_text(band_label, "---");
    }

    /* Radio type (band decoder mode) */
    {
        const char *radio_str;
        switch (g_config.radio_type) {
            case RADIO_TYPE_ICOM_CIV: radio_str = "ICOM";   break;
            case RADIO_TYPE_YAESU:    radio_str = "YAESU";   break;
            case RADIO_TYPE_KENWOOD:  radio_str = "KENWOOD"; break;
            default:                  radio_str = "";        break;
        }
        lv_label_set_text(lbl_radio_mode, radio_str);
    }

#else /* LVGL_SIM placeholder */
    (void)buf;
    lv_label_set_text(status_label, "Simulator");
    lv_obj_set_style_local_text_color(status_label, LV_LABEL_PART_MAIN,
                                       LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_bg_color(banner, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, COLOR_BANNER);
    lv_obj_set_style_local_bg_color(bottom_bar, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, COLOR_BANNER);
    lv_label_set_text(val_fwd, "0W");
    lv_label_set_text(val_ref, "0W");
    lv_label_set_text(val_swr, "1.0");
    lv_label_set_text(lbl_voltage, "13.8V");
    lv_label_set_text(lbl_current, "0A");
    lv_label_set_text(lbl_temp, "25C/77F");
    lv_label_set_text(lbl_fan, "-");
    lv_label_set_text(band_label, "Band: ---");
#endif
}

/*============================================================================
 * Calibration Screen — Step-by-step guided calibration
 *============================================================================*/

/* Forward declarations */
static void cal_cancel_cb(lv_obj_t *obj, lv_event_t e);
static void cal_accept_cb(lv_obj_t *obj, lv_event_t e);
static void cal_retry_cb(lv_obj_t *obj, lv_event_t e);
static void cal_del_container(void);
#ifdef LVGL_SIM
static void cal_sim_touch_cb(lv_obj_t *obj, lv_event_t e);
#endif
static void cal_build_point_ui(uint8_t idx);
static bool cal_calculate(float *x_slope, float *x_off,
                          float *y_slope, float *y_off,
                          float *residual);
static void cal_build_results_ui(void);
static void gui_calibration_advance(void);

/* Cancel button callback */
static void cal_cancel_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
        calibrating = false;
        cal_state = 0;
        lv_scr_load(main_screen);
    }
}

/* Accept button callback */
static void cal_accept_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
        calibrating = false;
        cal_state = 0;
#ifndef LVGL_SIM
        config_save();
#endif
        lv_scr_load(main_screen);
    }
}

/* Retry button callback */
static void cal_retry_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
        calibrating = true;
        cal_state = 1;
        cal_point_idx = 0;
        memset(cal_data, 0, sizeof(cal_data));
        lv_obj_clean(cal_screen);
        cal_container = NULL;
        gui_update_calibration(0);
    }
}

/* Remove the dynamic container */
static void cal_del_container(void)
{
    if (cal_container) {
        lv_obj_del(cal_container);
        cal_container = NULL;
    }
}

#ifdef LVGL_SIM
/* LVGL_SIM: clicking the red dot simulates a touch */
static void cal_sim_touch_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_PRESSED && calibrating && cal_state == 1) {
        uint8_t idx = cal_point_idx;
        if (idx >= CAL_POINT_COUNT) return;
        /* Simulated 12-bit raw coords from screen position + small noise */
        uint16_t raw_x = (uint16_t)(cal_targets[idx].x * 4000.0f / HOR_RES);
        uint16_t raw_y = (uint16_t)(cal_targets[idx].y * 4000.0f / VER_RES);
        raw_x += (uint16_t)((rand() % 41) - 20);
        raw_y += (uint16_t)((rand() % 41) - 20);
        cal_data[idx].raw_x = raw_x;
        cal_data[idx].raw_y = raw_y;
        cal_data[idx].screen_x = cal_targets[idx].x;
        cal_data[idx].screen_y = cal_targets[idx].y;
        gui_calibration_advance();
    }
}
#endif

/* Build the per-point UI: red dot + crosshair + instructions */
static void cal_build_point_ui(uint8_t idx)
{
    cal_del_container();
    cal_container = lv_obj_create(cal_screen, NULL);
    lv_obj_set_size(cal_container, HOR_RES, VER_RES);
    lv_obj_set_pos(cal_container, 0, 0);
    lv_obj_set_style_local_bg_opa(cal_container, LV_OBJ_PART_MAIN,
                                   LV_STATE_DEFAULT, LV_OPA_TRANSP);
    lv_obj_set_style_local_border_width(cal_container, LV_OBJ_PART_MAIN,
                                        LV_STATE_DEFAULT, 0);

    /* Title */
    lv_obj_t *title = lv_label_create(cal_container, NULL);
    lv_label_set_text(title, gui_get_string(STR_CALIBRATION));
    lv_obj_align(title, cal_container, LV_ALIGN_IN_TOP_MID, 0, 16);
    lv_obj_set_style_local_text_color(title, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, LV_COLOR_BLACK);
    lv_obj_set_style_local_text_font(title, LV_LABEL_PART_MAIN,
                                     LV_STATE_DEFAULT, &MyCJK_16);

    /* Step counter */
    char buf[32];
    snprintf(buf, sizeof(buf), "%d / %d", idx + 1, CAL_POINT_COUNT);
    lv_obj_t *step_lbl = lv_label_create(cal_container, NULL);
    lv_label_set_text(step_lbl, buf);
    lv_obj_align(step_lbl, cal_container, LV_ALIGN_IN_TOP_MID, 0, 44);
    lv_obj_set_style_local_text_color(step_lbl, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, LV_COLOR_BLACK);
    lv_obj_set_style_local_text_font(step_lbl, LV_LABEL_PART_MAIN,
                                     LV_STATE_DEFAULT, &lv_font_montserrat_24);

    /* Instruction */
    lv_obj_t *instr = lv_label_create(cal_container, NULL);
    lv_label_set_text(instr, gui_get_string(STR_CAL_POINT));
    lv_obj_align(instr, cal_container, LV_ALIGN_IN_BOTTOM_MID, 0, -100);
    lv_obj_set_style_local_text_color(instr, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, LV_COLOR_BLACK);
    lv_obj_set_style_local_text_font(instr, LV_LABEL_PART_MAIN,
                                     LV_STATE_DEFAULT, &MyCJK_16);

    /* Raw coordinate display */
    lv_obj_t *raw_lbl = lv_label_create(cal_container, NULL);
    lv_label_set_text(raw_lbl, "---");
    lv_obj_align(raw_lbl, cal_container, LV_ALIGN_IN_BOTTOM_MID, 0, -70);
    lv_obj_set_style_local_text_color(raw_lbl, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, LV_COLOR_MAKE(80, 80, 80));
    lv_obj_set_style_local_text_font(raw_lbl, LV_LABEL_PART_MAIN,
                                     LV_STATE_DEFAULT, &MyCJK_16);

    /* Cancel button — bottom center, away from corner calibration points */
    lv_obj_t *cancel_btn = lv_btn_create(cal_container, NULL);
    lv_obj_set_pos(cancel_btn, HOR_RES / 2 - 60, VER_RES - 50);
    lv_obj_set_size(cancel_btn, 120, 36);
    lv_obj_set_style_local_text_font(cancel_btn, LV_BTN_PART_MAIN,
                                     LV_STATE_DEFAULT, &MyCJK_16);
    lv_obj_t *cancel_lbl = lv_label_create(cancel_btn, NULL);
    lv_label_set_text(cancel_lbl, gui_get_string(STR_CANCEL));
    lv_obj_set_event_cb(cancel_btn, cal_cancel_cb);

    /* Red dot at target position */
    lv_coord_t tx = cal_targets[idx].x;
    lv_coord_t ty = cal_targets[idx].y;
    lv_obj_t *dot = lv_obj_create(cal_container, NULL);
    lv_obj_set_size(dot, 44, 44);
    lv_obj_set_pos(dot, tx - 22, ty - 22);
    lv_obj_set_style_local_bg_color(dot, LV_OBJ_PART_MAIN,
                                    LV_STATE_DEFAULT, LV_COLOR_RED);
    lv_obj_set_style_local_border_width(dot, LV_OBJ_PART_MAIN,
                                        LV_STATE_DEFAULT, 2);
    lv_obj_set_style_local_border_color(dot, LV_OBJ_PART_MAIN,
                                        LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_radius(dot, LV_OBJ_PART_MAIN,
                                  LV_STATE_DEFAULT, LV_RADIUS_CIRCLE);

    /* Crosshair lines */
    lv_obj_t *hl = lv_obj_create(cal_container, NULL);
    lv_obj_set_size(hl, 60, 2);
    lv_obj_set_pos(hl, tx - 30, ty - 1);
    lv_obj_set_style_local_bg_color(hl, LV_OBJ_PART_MAIN,
                                    LV_STATE_DEFAULT, LV_COLOR_RED);
    lv_obj_set_style_local_border_width(hl, LV_OBJ_PART_MAIN,
                                        LV_STATE_DEFAULT, 0);
    lv_obj_t *vl = lv_obj_create(cal_container, NULL);
    lv_obj_set_size(vl, 2, 60);
    lv_obj_set_pos(vl, tx - 1, ty - 30);
    lv_obj_set_style_local_bg_color(vl, LV_OBJ_PART_MAIN,
                                    LV_STATE_DEFAULT, LV_COLOR_RED);
    lv_obj_set_style_local_border_width(vl, LV_OBJ_PART_MAIN,
                                        LV_STATE_DEFAULT, 0);

#ifdef LVGL_SIM
    /* In simulator: dot is clickable */
    lv_obj_move_foreground(dot);
    lv_obj_set_event_cb(dot, cal_sim_touch_cb);
#endif
}

/* Linear regression on collected data; returns false if residual > 5% */
static bool cal_calculate(float *x_slope, float *x_off,
                          float *y_slope, float *y_off,
                          float *residual)
{
    float sx = 0, ssx = 0, sy = 0, sxy = 0;
    for (int i = 0; i < CAL_POINT_COUNT; i++) {
        sx  += cal_data[i].raw_x;
        ssx += (float)cal_data[i].raw_x * cal_data[i].raw_x;
        sy  += cal_data[i].screen_x;
        sxy += (float)cal_data[i].raw_x * cal_data[i].screen_x;
    }
    float denom = CAL_POINT_COUNT * ssx - sx * sx;
    if (fabsf(denom) < 0.001f) return false;
    *x_slope = (CAL_POINT_COUNT * sxy - sx * sy) / denom;
    *x_off   = (sy - *x_slope * sx) / CAL_POINT_COUNT;

    float sy_r = 0, ssy_r = 0, sy_s = 0, sxy_r = 0;
    for (int i = 0; i < CAL_POINT_COUNT; i++) {
        sy_r  += cal_data[i].raw_y;
        ssy_r += (float)cal_data[i].raw_y * cal_data[i].raw_y;
        sy_s  += cal_data[i].screen_y;
        sxy_r += (float)cal_data[i].raw_y * cal_data[i].screen_y;
    }
    denom = CAL_POINT_COUNT * ssy_r - sy_r * sy_r;
    if (fabsf(denom) < 0.001f) return false;
    *y_slope = (CAL_POINT_COUNT * sxy_r - sy_r * sy_s) / denom;
    *y_off   = (sy_s - *y_slope * sy_r) / CAL_POINT_COUNT;

    /* Center point residual (index 4) */
    float px = cal_data[4].raw_x * *x_slope + *x_off;
    float py = cal_data[4].raw_y * *y_slope + *y_off;
    float rx = fabsf(px - (float)cal_data[4].screen_x) / (float)cal_data[4].screen_x;
    float ry = fabsf(py - (float)cal_data[4].screen_y) / (float)cal_data[4].screen_y;
    *residual = (rx > ry ? rx : ry) * 100.0f;
    return (*residual <= 5.0f);
}

/* Build the results screen */
static void cal_build_results_ui(void)
{
    cal_del_container();
    cal_container = lv_obj_create(cal_screen, NULL);
    lv_obj_set_size(cal_container, HOR_RES, VER_RES);
    lv_obj_set_pos(cal_container, 0, 0);
    lv_obj_set_style_local_bg_opa(cal_container, LV_OBJ_PART_MAIN,
                                   LV_STATE_DEFAULT, LV_OPA_TRANSP);
    lv_obj_set_style_local_border_width(cal_container, LV_OBJ_PART_MAIN,
                                        LV_STATE_DEFAULT, 0);

    float x_slope, x_off, y_slope, y_off, residual;
    bool passed = cal_calculate(&x_slope, &x_off, &y_slope, &y_off, &residual);

#ifndef LVGL_SIM
    if (passed) {
        g_config.touch_cal.x_slope  = x_slope;
        g_config.touch_cal.x_offset = x_off;
        g_config.touch_cal.y_slope  = y_slope;
        g_config.touch_cal.y_offset = y_off;
        g_config.touch_cal.valid    = true;
    }
#endif

    lv_color_t result_c = passed ? LV_COLOR_GREEN : LV_COLOR_RED;
    lv_obj_t *result_lbl = lv_label_create(cal_container, NULL);
    lv_label_set_text(result_lbl, passed ? gui_get_string(STR_CAL_SUCCESS)
                                         : gui_get_string(STR_CAL_FAILED));
    lv_obj_align(result_lbl, cal_container, LV_ALIGN_IN_TOP_MID, 0, 20);
    lv_obj_set_style_local_text_color(result_lbl, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, result_c);
    lv_obj_set_style_local_text_font(result_lbl, LV_LABEL_PART_MAIN,
                                     LV_STATE_DEFAULT, &MyCJK_16);

    char buf[128];
    lv_obj_t *params_lbl = lv_label_create(cal_container, NULL);
    snprintf(buf, sizeof(buf),
        "X: y = %.3f * x + %.1f\n"
        "Y: y = %.3f * x + %.1f\n"
        "%s: %.1f%% (max 5%%)",
        x_slope, x_off, y_slope, y_off,
        gui_get_string(STR_CAL_RESIDUAL), residual);
    lv_label_set_text(params_lbl, buf);
    lv_obj_align(params_lbl, cal_container, LV_ALIGN_CENTER, 0, -20);
    lv_obj_set_style_local_text_color(params_lbl, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, LV_COLOR_BLACK);
    lv_obj_set_style_local_text_font(params_lbl, LV_LABEL_PART_MAIN,
                                     LV_STATE_DEFAULT, &MyCJK_16);

    if (passed) {
        lv_obj_t *accept_btn = lv_btn_create(cal_container, NULL);
        lv_obj_set_pos(accept_btn, HOR_RES / 2 - 130, VER_RES - 70);
        lv_obj_set_size(accept_btn, 120, 40);
        lv_obj_set_style_local_text_font(accept_btn, LV_BTN_PART_MAIN,
                                         LV_STATE_DEFAULT, &MyCJK_16);
        lv_obj_t *accept_lbl = lv_label_create(accept_btn, NULL);
        lv_label_set_text(accept_lbl, gui_get_string(STR_CAL_ACCEPT));
        lv_obj_set_event_cb(accept_btn, cal_accept_cb);
    }

    lv_obj_t *retry_btn = lv_btn_create(cal_container, NULL);
    lv_obj_set_pos(retry_btn, HOR_RES / 2 + 10, VER_RES - 70);
    lv_obj_set_size(retry_btn, 120, 40);
    lv_obj_set_style_local_text_font(retry_btn, LV_BTN_PART_MAIN,
                                     LV_STATE_DEFAULT, &MyCJK_16);
    lv_obj_t *retry_lbl = lv_label_create(retry_btn, NULL);
    lv_label_set_text(retry_lbl, gui_get_string(STR_CAL_RETRY));
    lv_obj_set_event_cb(retry_btn, cal_retry_cb);
}

/* Advance to next point or show results */
static void gui_calibration_advance(void)
{
    cal_point_idx++;
    if (cal_point_idx >= CAL_POINT_COUNT) {
        cal_state = 2;
        cal_build_results_ui();
    } else {
        cal_state = 1;
        cal_build_point_ui(cal_point_idx);
    }
}

static void gui_create_cal_screen(void)
{
    cal_screen = lv_obj_create(NULL, NULL);
    lv_obj_set_size(cal_screen, HOR_RES, VER_RES);
    lv_obj_set_style_local_bg_color(cal_screen, LV_OBJ_PART_MAIN,
                                     LV_STATE_DEFAULT, LV_COLOR_WHITE);
}

/*============================================================================
 * Public Functions
 *============================================================================*/

void gui_init(void)
{
    lv_init();

#ifndef LVGL_SIM
    lv_port_disp_init();
    current_language = g_config.language;
#else
    current_language = MULTILANG_ENGLISH;
#endif

    gui_create_main_screen();
    lv_scr_load(main_screen);
    current_screen = 0;
}

void gui_start(void) { }

void gui_tick(void)
{
    lv_tick_inc(5);
    lv_task_handler();
}

void gui_update_display(void)
{
    if (!alert_active && !calibrating) {
        gui_update_main_values();
    }
}

void gui_show_alert(AlertType alert_type)
{
    const char *alert_text;
    switch (alert_type) {
        case ALERT_SWR_HIGH:          alert_text = gui_get_string(STR_SWR_HIGH); break;
        case ALERT_REFLECT_POWER_HIGH: alert_text = gui_get_string(STR_REF_HIGH); break;
        case ALERT_CURRENT_HIGH:      alert_text = gui_get_string(STR_CURRENT_HIGH); break;
        case ALERT_HARDWARE_FAULT:    alert_text = gui_get_string(STR_HARDWARE_FAULT); break;
        case ALERT_OVERDRIVE:         alert_text = gui_get_string(STR_OVERDRIVE); break;
        case ALERT_FREQ_MISMATCH:     alert_text = gui_get_string(STR_FREQ_MISMATCH); break;
        default:                      alert_text = gui_get_string(STR_ALERT); break;
    }
    alert_msgbox = lv_msgbox_create(main_screen, NULL);
    lv_msgbox_set_text(alert_msgbox, alert_text);
    static const char *btn_txts[] = {"Dismiss", ""};
    lv_msgbox_add_btns(alert_msgbox, btn_txts);
    lv_obj_set_size(alert_msgbox, 300, 150);
    lv_obj_align(alert_msgbox, main_screen, LV_ALIGN_CENTER, 0, 0);
    alert_active = true;
}

void gui_dismiss_alert(void)
{
    if (alert_msgbox) {
        lv_obj_del(alert_msgbox);
        alert_msgbox = NULL;
    }
    alert_active = false;
}

bool gui_is_alert_active(void) { return alert_active; }

const char* gui_get_string(uint16_t str_id)
{
    if (str_id >= 40) return "";
    return gui_strings[gui_get_lang_index()][str_id];
}

/*
 * Array of labels that need text refresh when language changes.
 * To add a new multilingual label, just append an entry here.
 */
typedef struct {
    lv_obj_t **label;
    uint16_t str_id;
} LangRefreshEntry;

static LangRefreshEntry lang_refresh[] = {
    {&status_label,    STR_IDLE},
    {&lbl_tx_title,    STR_TX_POWER},
    {&lbl_ref_title,   STR_REF_POWER},
    {&lbl_swr_title,   STR_SWR},
    {&lbl_voltage_cap, STR_VOLTAGE},
    {&lbl_current_cap, STR_CURRENT},
    {&lbl_temp_cap,    STR_TEMPERATURE},
    {&lbl_fan_cap,     STR_FAN},
    {&lbl_settings_btn, STR_SETTINGS},
};

void gui_set_language(LanguageType lang)
{
    current_language = lang;
#ifndef LVGL_SIM
    g_config.language = lang;
    config_save();
#endif

    for (size_t i = 0; i < sizeof(lang_refresh) / sizeof(lang_refresh[0]); i++) {
        if (*lang_refresh[i].label) {
            lv_label_set_text(*lang_refresh[i].label,
                              gui_get_string(lang_refresh[i].str_id));
        }
    }
}

void gui_show_calibration(void)
{
    if (cal_screen == NULL) gui_create_cal_screen();
    calibrating = true;
    cal_state = 1;
    cal_point_idx = 0;
    memset(cal_data, 0, sizeof(cal_data));
#ifndef LVGL_SIM
    touch_calibrate_start();
#endif
    lv_scr_load(cal_screen);
    gui_update_calibration(0);
}

void gui_update_calibration(uint8_t current_point)
{
    (void)current_point;
    if (cal_state == 1) {
        cal_build_point_ui(cal_point_idx);
    }
}

void gui_process_calibration(void)
{
#ifndef LVGL_SIM
    if (!calibrating || cal_state != 1) return;

    uint16_t raw_x, raw_y;
    bool pressed;
    if (touch_resistive_read(&raw_x, &raw_y, &pressed) && pressed) {
        uint8_t idx = cal_point_idx;
        if (idx >= CAL_POINT_COUNT) return;

        cal_data[idx].raw_x = raw_x;
        cal_data[idx].raw_y = raw_y;
        cal_data[idx].screen_x = cal_targets[idx].x;
        cal_data[idx].screen_y = cal_targets[idx].y;

        gui_calibration_advance();
    }
#endif
}

void gui_power_button_pressed(void)
{
#ifndef LVGL_SIM
    if (g_psu_on) power_off();
    else power_on();
#else
    static bool sim_active = false;
    sim_active = !sim_active;
    if (sim_active) {
        lv_label_set_text(status_label, gui_get_string(STR_STANDBY));
        lv_obj_set_style_local_bg_color(banner, LV_OBJ_PART_MAIN,
                                         LV_STATE_DEFAULT, LV_COLOR_MAKE(0xea, 0xb9, 0x06));
        lv_obj_set_style_local_bg_color(bottom_bar, LV_OBJ_PART_MAIN,
                                         LV_STATE_DEFAULT, LV_COLOR_MAKE(0xea, 0xb9, 0x06));
    } else {
        lv_label_set_text(status_label, gui_get_string(STR_IDLE));
        lv_obj_set_style_local_bg_color(banner, LV_OBJ_PART_MAIN,
                                         LV_STATE_DEFAULT, COLOR_BANNER);
        lv_obj_set_style_local_bg_color(bottom_bar, LV_OBJ_PART_MAIN,
                                         LV_STATE_DEFAULT, COLOR_BANNER);
    }
    /* Text always white */
    lv_obj_set_style_local_text_color(status_label, LV_LABEL_PART_MAIN,
                                       LV_STATE_DEFAULT, LV_COLOR_WHITE);
#endif
}

uint8_t gui_get_current_screen(void) { return current_screen; }

static void show_language_selector(void);
static void show_screen_settings(void);
static void show_coupler_selector(void);
static void show_sensor_inspect(void);
static void show_fan_settings(void);
static void show_band_comm_settings(void);
static void show_protection_settings(void);

static void settings_menu_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
        uint16_t btn_id = lv_btnmatrix_get_active_btn(obj);
        if (btn_id == 0) {
            lv_obj_del_async(lv_obj_get_parent(obj));
            show_coupler_selector();
            return;
        }
        if (btn_id == 1) {
            lv_obj_del_async(lv_obj_get_parent(obj));
            show_sensor_inspect();
            return;
        }
        if (btn_id == 2) {
            lv_obj_del_async(lv_obj_get_parent(obj));
            show_fan_settings();
            return;
        }
        if (btn_id == 3) {
            lv_obj_del_async(lv_obj_get_parent(obj));
            show_band_comm_settings();
            return;
        }
        if (btn_id == 4) {
            lv_obj_del_async(lv_obj_get_parent(obj));
            show_protection_settings();
            return;
        }
        if (btn_id == 5) {
            lv_obj_del_async(lv_obj_get_parent(obj));
            show_screen_settings();
            return;
        }
        if (btn_id == 6) {
            lv_obj_del_async(lv_obj_get_parent(obj));
            show_language_selector();
            return;
        }
        lv_obj_del_async(lv_obj_get_parent(obj));
    }
}

/*============================================================================
 * Language Selector Popup
 *============================================================================*/

static const char *lang_btns[] = {"English", "中文", "\n", NULL, ""};

static void language_sel_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
        uint16_t id = lv_msgbox_get_active_btn(obj);
        if (id == 0) {
            gui_set_language(MULTILANG_ENGLISH);
        } else if (id == 1) {
            gui_set_language(MULTILANG_CHINESE);
        }
        lv_obj_del_async(obj);
    }
}

static void show_language_selector(void)
{
    lang_btns[3] = gui_get_string(STR_CANCEL);

    lv_obj_t *scr = lv_scr_act();
    lv_obj_t *mbox = lv_msgbox_create(scr, NULL);
    lv_msgbox_set_text(mbox, gui_get_string(STR_LANGUAGE));
    lv_msgbox_add_btns(mbox, lang_btns);
    lv_obj_set_size(mbox, 320, 180);
    lv_obj_align(mbox, scr, LV_ALIGN_CENTER, 0, 0);

    lv_obj_set_style_local_text_color(mbox, LV_MSGBOX_PART_BG,
                                       LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(mbox, LV_MSGBOX_PART_BG,
                                      LV_STATE_DEFAULT, &MyCJK_16);
    lv_obj_set_style_local_bg_color(mbox, LV_MSGBOX_PART_BG,
                                     LV_STATE_DEFAULT, LV_COLOR_BLUE);

    lv_obj_t *btnm = lv_msgbox_get_btnmatrix(mbox);
    lv_obj_set_style_local_text_color(btnm, LV_BTNMATRIX_PART_BTN,
                                      LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(btnm, LV_BTNMATRIX_PART_BTN,
                                      LV_STATE_DEFAULT, &MyCJK_16);
    lv_obj_set_style_local_bg_color(btnm, LV_BTNMATRIX_PART_BTN,
                                     LV_STATE_DEFAULT, LV_COLOR_MAKE(0, 160, 225));
    lv_obj_set_style_local_border_color(btnm, LV_BTNMATRIX_PART_BTN,
                                         LV_STATE_DEFAULT, LV_COLOR_GRAY);
    lv_obj_set_style_local_radius(btnm, LV_BTNMATRIX_PART_BTN,
                                   LV_STATE_DEFAULT, 0);
    lv_obj_set_event_cb(mbox, language_sel_cb);
}

/*============================================================================
 * Screen Settings Popup
 *============================================================================*/

static const char *screen_btns[] = {"Touch Cal", "\n", "Cancel", ""};

static void screen_settings_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
        uint16_t btn_id = lv_msgbox_get_active_btn(obj);
        if (btn_id == 0) {  /* Touch Calibration */
            lv_obj_del_async(obj);
            gui_show_calibration();
            return;
        }
        lv_obj_del_async(obj);
    }
}

static void show_screen_settings(void)
{
    screen_btns[0] = gui_get_string(STR_TOUCH_CAL);
    screen_btns[2] = gui_get_string(STR_CANCEL);

    lv_obj_t *scr = lv_scr_act();
    lv_obj_t *mbox = lv_msgbox_create(scr, NULL);
    lv_msgbox_set_text(mbox, gui_get_string(STR_SCREEN));
    lv_msgbox_add_btns(mbox, screen_btns);
    lv_obj_set_size(mbox, 320, 160);
    lv_obj_align(mbox, scr, LV_ALIGN_CENTER, 0, 0);

    lv_obj_set_style_local_text_color(mbox, LV_MSGBOX_PART_BG,
                                       LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(mbox, LV_MSGBOX_PART_BG,
                                      LV_STATE_DEFAULT, &MyCJK_16);
    lv_obj_set_style_local_bg_color(mbox, LV_MSGBOX_PART_BG,
                                     LV_STATE_DEFAULT, LV_COLOR_BLUE);

    lv_obj_t *btnm = lv_msgbox_get_btnmatrix(mbox);
    lv_obj_set_style_local_text_color(btnm, LV_BTNMATRIX_PART_BTN,
                                      LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(btnm, LV_BTNMATRIX_PART_BTN,
                                      LV_STATE_DEFAULT, &MyCJK_16);
    lv_obj_set_style_local_bg_color(btnm, LV_BTNMATRIX_PART_BTN,
                                     LV_STATE_DEFAULT, LV_COLOR_MAKE(0, 160, 225));
    lv_obj_set_style_local_border_color(btnm, LV_BTNMATRIX_PART_BTN,
                                         LV_STATE_DEFAULT, LV_COLOR_GRAY);
    lv_obj_set_style_local_radius(btnm, LV_BTNMATRIX_PART_BTN,
                                   LV_STATE_DEFAULT, 0);
    lv_obj_set_event_cb(mbox, screen_settings_cb);
}

/*============================================================================
 * Coupler Selector Popup
 *============================================================================*/

static const char *coupler_btns[] = {"C1", "C2", "\n", "Cancel", ""};

static void coupler_sel_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
        uint16_t btn_id = lv_msgbox_get_active_btn(obj);
        lv_obj_del_async(obj);
        if (btn_id == 0 || btn_id == 1) {
            gui_show_coupler_cal(btn_id + 1);
        }
    }
}

static void show_coupler_selector(void)
{
    coupler_btns[0] = gui_get_string(STR_DIR_COUPLER_1);
    coupler_btns[1] = gui_get_string(STR_DIR_COUPLER_2);
    coupler_btns[3] = gui_get_string(STR_CANCEL);

    lv_obj_t *scr = lv_scr_act();
    lv_obj_t *mbox = lv_msgbox_create(scr, NULL);
    lv_msgbox_set_text(mbox, gui_get_string(STR_CAL_SENSORS));
    lv_msgbox_add_btns(mbox, coupler_btns);
    lv_obj_set_size(mbox, 320, 180);
    lv_obj_align(mbox, scr, LV_ALIGN_CENTER, 0, 0);

    lv_obj_set_style_local_text_color(mbox, LV_MSGBOX_PART_BG,
                                       LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(mbox, LV_MSGBOX_PART_BG,
                                      LV_STATE_DEFAULT, &MyCJK_16);
    lv_obj_set_style_local_bg_color(mbox, LV_MSGBOX_PART_BG,
                                     LV_STATE_DEFAULT, LV_COLOR_BLUE);

    lv_obj_t *btnm = lv_msgbox_get_btnmatrix(mbox);
    lv_obj_set_style_local_text_color(btnm, LV_BTNMATRIX_PART_BTN,
                                      LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(btnm, LV_BTNMATRIX_PART_BTN,
                                      LV_STATE_DEFAULT, &MyCJK_16);
    lv_obj_set_style_local_bg_color(btnm, LV_BTNMATRIX_PART_BTN,
                                     LV_STATE_DEFAULT, LV_COLOR_MAKE(0, 160, 225));
    lv_obj_set_style_local_border_color(btnm, LV_BTNMATRIX_PART_BTN,
                                         LV_STATE_DEFAULT, LV_COLOR_GRAY);
    lv_obj_set_style_local_radius(btnm, LV_BTNMATRIX_PART_BTN,
                                   LV_STATE_DEFAULT, 0);
    lv_obj_set_event_cb(mbox, coupler_sel_cb);
}

/*============================================================================
 * Coupler Calibration — Step-by-step guided calibration
 *============================================================================*/

/* Forward declarations */
static void cc_build_ui(uint8_t step);
static void cc_build_results(void);
static void cc_start_cb(lv_obj_t *obj, lv_event_t e);
static void cc_cancel_cb(lv_obj_t *obj, lv_event_t e);
static uint16_t cc_read_adc(void);

/* State */
#define CC_STEPS 4
static bool cc_active = false;
static uint8_t cc_coupler_id = 0;
static uint8_t cc_step = 0;
static uint8_t cc_state = 0;     /* 0=idle, 1=showing, 2=results */
static lv_obj_t *cc_screen = NULL;
static lv_obj_t *cc_container = NULL;
static lv_obj_t *cc_adc_val_label = NULL;
static uint16_t cc_meas[CC_STEPS];

/* Step descriptions (built from template + existing strings) */
static void cc_step_desc(uint8_t step, char *buf, size_t len)
{
    const char *dir = (step < 2) ? "Forward" : "Reflected";
    float power = (step == 0 || step == 2) ? 5.0f : 20.0f;
    snprintf(buf, len, "%s %.0fW", dir, power);
}

/* Read ADC raw value for the current coupler + step */
static uint16_t cc_read_adc(void)
{
#ifndef LVGL_SIM
    uint8_t ch;
    if (cc_coupler_id == 1)
        ch = (cc_step < 2) ? ADC_CHANNEL_TX_POWER1 : ADC_CHANNEL_REF_POWER1;
    else
        ch = (cc_step < 2) ? ADC_CHANNEL_TX_POWER2 : ADC_CHANNEL_REF_POWER2;
    return g_adc_data.raw[ch];
#else
    /* Simulate ADC readings */
    switch (cc_step) {
        case 0: return 500 + (uint16_t)(rand() % 101);   /* 5W  fwd */
        case 1: return 2000 + (uint16_t)(rand() % 201);  /* 20W fwd */
        case 2: return 200 + (uint16_t)(rand() % 51);    /* 5W  ref */
        case 3: return 600 + (uint16_t)(rand() % 101);   /* 20W ref */
        default: return 0;
    }
#endif
}

/* Cancel button callback */
static void cc_cancel_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
        cc_active = false;
        cc_state = 0;
        lv_scr_load(main_screen);
    }
}

/* Accept button callback */
static void cc_accept_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
#ifndef LVGL_SIM
        config_save();
#endif
        cc_active = false;
        cc_state = 0;
        lv_scr_load(main_screen);
    }
}

/* Retry button callback */
static void cc_retry_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
        cc_step = 0;
        cc_state = 1;
        memset(cc_meas, 0, sizeof(cc_meas));
        lv_obj_clean(cc_screen);
        cc_container = NULL;
        cc_build_ui(0);
    }
}

/* Start measurement button */
static void cc_start_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_PRESSED && cc_active && cc_state == 1) {
        cc_meas[cc_step] = cc_read_adc();
        cc_step++;
        if (cc_step >= CC_STEPS) {
            cc_state = 2;
            cc_build_results();
        } else {
            cc_state = 1;
            cc_build_ui(cc_step);
        }
    }
}

/* Build the step UI */
static void cc_build_ui(uint8_t step)
{
    if (cc_container) {
        lv_obj_del(cc_container);
        cc_container = NULL;
    }
    cc_adc_val_label = NULL;

    cc_container = lv_obj_create(cc_screen, NULL);
    lv_obj_set_size(cc_container, HOR_RES, VER_RES);
    lv_obj_set_pos(cc_container, 0, 0);
    lv_obj_set_style_local_bg_opa(cc_container, LV_OBJ_PART_MAIN,
                                   LV_STATE_DEFAULT, LV_OPA_TRANSP);
    lv_obj_set_style_local_border_width(cc_container, LV_OBJ_PART_MAIN,
                                        LV_STATE_DEFAULT, 0);

    /* Title: "Coupler X Calibration" */
    char buf[64];
    snprintf(buf, sizeof(buf), "%s %d %s",
             gui_get_string(STR_DIR_COUPLER_1)[0] ? gui_get_string(STR_DIR_COUPLER_1) : "",
             cc_coupler_id,
             gui_get_string(STR_CALIBRATION));
    lv_obj_t *title = lv_label_create(cc_container, NULL);
    lv_label_set_text(title, buf);
    lv_obj_align(title, cc_container, LV_ALIGN_IN_TOP_MID, 0, 16);
    lv_obj_set_style_local_text_color(title, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, LV_COLOR_BLACK);
    lv_obj_set_style_local_text_font(title, LV_LABEL_PART_MAIN,
                                     LV_STATE_DEFAULT, &MyCJK_16);

    /* Step + direction */
    char step_str[32];
    cc_step_desc(step, step_str, sizeof(step_str));
    snprintf(buf, sizeof(buf), "%d/%d  %s", step + 1, CC_STEPS, step_str);
    lv_obj_t *step_lbl = lv_label_create(cc_container, NULL);
    lv_label_set_text(step_lbl, buf);
    lv_obj_align(step_lbl, cc_container, LV_ALIGN_IN_TOP_MID, 0, 46);
    lv_obj_set_style_local_text_color(step_lbl, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, LV_COLOR_BLACK);
    lv_obj_set_style_local_text_font(step_lbl, LV_LABEL_PART_MAIN,
                                     LV_STATE_DEFAULT, &lv_font_montserrat_24);

    /* Instruction */
    lv_obj_t *instr = lv_label_create(cc_container, NULL);
    snprintf(buf, sizeof(buf), "%s %.0fW AM  |  %s",
             gui_get_string(STR_CAL_POINT),
             (step == 0 || step == 2) ? 5.0f : 20.0f,
             gui_get_string(STR_START_MEAS));
    lv_label_set_text(instr, buf);
    lv_obj_align(instr, cc_container, LV_ALIGN_IN_TOP_MID, 0, 84);
    lv_obj_set_style_local_text_color(instr, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, LV_COLOR_BLACK);
    lv_obj_set_style_local_text_font(instr, LV_LABEL_PART_MAIN,
                                     LV_STATE_DEFAULT, &MyCJK_16);

    /* ADC real-time value box */
    lv_obj_t *adc_box = lv_obj_create(cc_container, NULL);
    lv_obj_set_size(adc_box, HOR_RES - 80, 100);
    lv_obj_set_pos(adc_box, HOR_RES / 20, VER_RES / 4 + 10);
    lv_obj_set_style_local_border_color(adc_box, LV_OBJ_PART_MAIN,
                                         LV_STATE_DEFAULT, COLOR_BORDER);
    lv_obj_set_style_local_radius(adc_box, LV_OBJ_PART_MAIN,
                                   LV_STATE_DEFAULT, 4);

    /* Channel name */
    lv_obj_t *ch_label = lv_label_create(adc_box, NULL);
    snprintf(buf, sizeof(buf), "%s: %s",
             (step < 2) ? "TX Power" : "REF Power",
             (cc_coupler_id == 1) ? "CH1" : "CH2");
    lv_label_set_text(ch_label, buf);
    lv_obj_align(ch_label, adc_box, LV_ALIGN_IN_TOP_LEFT, 10, 8);
    lv_obj_set_style_local_text_color(ch_label, LV_LABEL_PART_MAIN,
                                       LV_STATE_DEFAULT, LV_COLOR_BLACK);
    lv_obj_set_style_local_text_font(ch_label, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &MyCJK_16);

    /* ADC raw value */
    lv_obj_t *raw_label = lv_label_create(adc_box, NULL);
    lv_label_set_text(raw_label, "ADC: ---");
    lv_obj_align(raw_label, adc_box, LV_ALIGN_IN_LEFT_MID, 10, 0);
    lv_obj_set_style_local_text_color(raw_label, LV_LABEL_PART_MAIN,
                                       LV_STATE_DEFAULT, LV_COLOR_BLACK);
    lv_obj_set_style_local_text_font(raw_label, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &lv_font_montserrat_24);

    /* Calculated power */
    cc_adc_val_label = lv_label_create(adc_box, NULL);
    lv_label_set_text(cc_adc_val_label, "Power: ---");
    lv_obj_align(cc_adc_val_label, adc_box, LV_ALIGN_IN_RIGHT_MID, -10, 0);
    lv_obj_set_style_local_text_color(cc_adc_val_label, LV_LABEL_PART_MAIN,
                                       LV_STATE_DEFAULT, LV_COLOR_BLACK);
    lv_obj_set_style_local_text_font(cc_adc_val_label, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &lv_font_montserrat_24);

    /* Start button */
    lv_obj_t *start_btn = lv_btn_create(cc_container, NULL);
    lv_obj_set_pos(start_btn, HOR_RES / 2 - 130, VER_RES - 70);
    lv_obj_set_size(start_btn, 120, 40);
    lv_obj_set_style_local_text_font(start_btn, LV_BTN_PART_MAIN,
                                     LV_STATE_DEFAULT, &MyCJK_16);
    lv_obj_t *start_lbl = lv_label_create(start_btn, NULL);
    lv_label_set_text(start_lbl, gui_get_string(STR_START_MEAS));
    lv_obj_set_event_cb(start_btn, cc_start_cb);

    /* Cancel button */
    lv_obj_t *cancel_btn = lv_btn_create(cc_container, NULL);
    lv_obj_set_pos(cancel_btn, HOR_RES / 2 + 10, VER_RES - 70);
    lv_obj_set_size(cancel_btn, 120, 40);
    lv_obj_set_style_local_text_font(cancel_btn, LV_BTN_PART_MAIN,
                                     LV_STATE_DEFAULT, &MyCJK_16);
    lv_obj_t *cancel_lbl = lv_label_create(cancel_btn, NULL);
    lv_label_set_text(cancel_lbl, gui_get_string(STR_CANCEL));
    lv_obj_set_event_cb(cancel_btn, cc_cancel_cb);
}

/* Calculate calibration from 2-point data */
static void cc_calculate(float *fwd_slope, float *fwd_off,
                         float *ref_slope, float *ref_off)
{
    if (cc_meas[1] != cc_meas[0]) {
        *fwd_slope = (20.0f - 5.0f) / (float)(cc_meas[1] - cc_meas[0]);
        *fwd_off   = 5.0f - *fwd_slope * (float)cc_meas[0];
    } else {
        *fwd_slope = 0.0f;
        *fwd_off   = 0.0f;
    }
    if (cc_meas[3] != cc_meas[2]) {
        *ref_slope = (20.0f - 5.0f) / (float)(cc_meas[3] - cc_meas[2]);
        *ref_off   = 5.0f - *ref_slope * (float)cc_meas[2];
    } else {
        *ref_slope = 0.0f;
        *ref_off   = 0.0f;
    }
}

/* Results screen */
static void cc_build_results(void)
{
    if (cc_container) {
        lv_obj_del(cc_container);
        cc_container = NULL;
    }

    /* Calculate */
    float fwd_slope, fwd_off, ref_slope, ref_off;
    cc_calculate(&fwd_slope, &fwd_off, &ref_slope, &ref_off);

#ifndef LVGL_SIM
    if (cc_coupler_id == 1) {
        g_config.cal_tx_power1.forward_slope   = fwd_slope;
        g_config.cal_tx_power1.forward_offset  = fwd_off;
        g_config.cal_ref_power1.reflect_slope  = ref_slope;
        g_config.cal_ref_power1.reflect_offset = ref_off;
    } else {
        g_config.cal_tx_power2.forward_slope   = fwd_slope;
        g_config.cal_tx_power2.forward_offset  = fwd_off;
        g_config.cal_ref_power2.reflect_slope  = ref_slope;
        g_config.cal_ref_power2.reflect_offset = ref_off;
    }
#endif

    cc_container = lv_obj_create(cc_screen, NULL);
    lv_obj_set_size(cc_container, HOR_RES, VER_RES);
    lv_obj_set_pos(cc_container, 0, 0);
    lv_obj_set_style_local_bg_opa(cc_container, LV_OBJ_PART_MAIN,
                                   LV_STATE_DEFAULT, LV_OPA_TRANSP);
    lv_obj_set_style_local_border_width(cc_container, LV_OBJ_PART_MAIN,
                                        LV_STATE_DEFAULT, 0);

    char buf[200];
    snprintf(buf, sizeof(buf), "%s %d %s",
             gui_get_string(STR_DIR_COUPLER_1)[0] ? gui_get_string(STR_DIR_COUPLER_1) : "",
             cc_coupler_id,
             gui_get_string(STR_CAL_SUCCESS));
    lv_obj_t *result_lbl = lv_label_create(cc_container, NULL);
    lv_label_set_text(result_lbl, buf);
    lv_obj_align(result_lbl, cc_container, LV_ALIGN_IN_TOP_MID, 0, 20);
    lv_obj_set_style_local_text_color(result_lbl, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, LV_COLOR_GREEN);
    lv_obj_set_style_local_text_font(result_lbl, LV_LABEL_PART_MAIN,
                                     LV_STATE_DEFAULT, &MyCJK_16);

    /* Parameters */
    lv_obj_t *params_lbl = lv_label_create(cc_container, NULL);
    snprintf(buf, sizeof(buf),
        "FWD: slope=%.6f  offset=%.4f\n"
        "REF: slope=%.6f  offset=%.4f",
        fwd_slope, fwd_off, ref_slope, ref_off);
    lv_label_set_text(params_lbl, buf);
    lv_obj_align(params_lbl, cc_container, LV_ALIGN_CENTER, 0, -20);
    lv_obj_set_style_local_text_color(params_lbl, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, LV_COLOR_BLACK);
    lv_obj_set_style_local_text_font(params_lbl, LV_LABEL_PART_MAIN,
                                     LV_STATE_DEFAULT, &MyCJK_16);

    /* Accept */
    lv_obj_t *accept_btn = lv_btn_create(cc_container, NULL);
    lv_obj_set_pos(accept_btn, HOR_RES / 2 - 130, VER_RES - 70);
    lv_obj_set_size(accept_btn, 120, 40);
    lv_obj_set_style_local_text_font(accept_btn, LV_BTN_PART_MAIN,
                                     LV_STATE_DEFAULT, &MyCJK_16);
    lv_obj_t *accept_lbl = lv_label_create(accept_btn, NULL);
    lv_label_set_text(accept_lbl, gui_get_string(STR_CAL_ACCEPT));
    lv_obj_set_event_cb(accept_btn, cc_accept_cb);

    /* Retry */
    lv_obj_t *retry_btn = lv_btn_create(cc_container, NULL);
    lv_obj_set_pos(retry_btn, HOR_RES / 2 + 10, VER_RES - 70);
    lv_obj_set_size(retry_btn, 120, 40);
    lv_obj_set_style_local_text_font(retry_btn, LV_BTN_PART_MAIN,
                                     LV_STATE_DEFAULT, &MyCJK_16);
    lv_obj_t *retry_lbl = lv_label_create(retry_btn, NULL);
    lv_label_set_text(retry_lbl, gui_get_string(STR_CAL_RETRY));
    lv_obj_set_event_cb(retry_btn, cc_retry_cb);
}

/* Update real-time ADC display — called from main loop */
void gui_process_coupler_cal(void)
{
    if (!cc_active || cc_state != 1 || cc_adc_val_label == NULL) return;

    uint16_t adc = cc_read_adc();
    char buf[64];

#ifndef LVGL_SIM
    float power;
    float slope = 0.0f, offset = 0.0f;
    if (cc_step < 2) {
        CalDataType *cal = (cc_coupler_id == 1) ? &g_config.cal_tx_power1 : &g_config.cal_tx_power2;
        slope = cal->forward_slope;
        offset = cal->forward_offset;
    } else {
        CalDataType *cal = (cc_coupler_id == 1) ? &g_config.cal_ref_power1 : &g_config.cal_ref_power2;
        slope = cal->reflect_slope;
        offset = cal->reflect_offset;
    }
    power = (float)adc * slope + offset;
    snprintf(buf, sizeof(buf), "ADC: %u    %.2fW", adc, power);
#else
    snprintf(buf, sizeof(buf), "ADC: %u", adc);
#endif
    lv_label_set_text(cc_adc_val_label, buf);
}

/* Public: show coupler calibration screen */
void gui_show_coupler_cal(uint8_t coupler_id)
{
    if (cc_screen == NULL) {
        cc_screen = lv_obj_create(NULL, NULL);
        lv_obj_set_size(cc_screen, HOR_RES, VER_RES);
        lv_obj_set_style_local_bg_color(cc_screen, LV_OBJ_PART_MAIN,
                                         LV_STATE_DEFAULT, LV_COLOR_WHITE);
    }

    cc_active = true;
    cc_coupler_id = coupler_id;
    cc_step = 0;
    cc_state = 1;
    memset(cc_meas, 0, sizeof(cc_meas));
    lv_scr_load(cc_screen);
    cc_build_ui(0);
}

/*============================================================================
 * Sensor Inspect Overlay — dual-panel table with ADC channel info
 *============================================================================*/

static lv_obj_t *si_overlay = NULL;
static lv_obj_t *si_val_labels[ADC_CHANNEL_COUNT * 2]; /* each row: raw + value */

static void si_close_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
        lv_obj_del_async(si_overlay);
        si_overlay = NULL;
    }
}

/* Build one column in the sensor-inspect table — all text centered */
static void si_build_col(lv_obj_t *parent, lv_coord_t x, int start_ch, int count,
                         const char *title, lv_obj_t **raw_labels, lv_obj_t **val_labels)
{
    lv_coord_t hdr_y = 44;
    lv_coord_t row_h = 34;
    /* Column widths for centering */
    const lv_coord_t cw_name = 85, cw_adc = 60, cw_raw = 60, cw_val = 95;
    lv_coord_t name_cx = x + cw_name / 2;
    lv_coord_t adc_cx  = x + cw_name + cw_adc / 2;
    lv_coord_t raw_cx  = x + cw_name + cw_adc + cw_raw / 2;
    lv_coord_t val_cx  = x + cw_name + cw_adc + cw_raw + cw_val / 2;

    /* Helper: create a centered label */
#define SI_LABEL(_parent, _text, _cx, _y, _w, _color, _font) \
    do { \
        lv_obj_t *l__ = lv_label_create(_parent, NULL); \
        lv_label_set_text(l__, _text); \
        lv_obj_set_width(l__, _w); \
        lv_obj_set_pos(l__, _cx - _w / 2, _y); \
        lv_label_set_align(l__, LV_LABEL_ALIGN_CENTER); \
        lv_obj_set_style_local_text_color(l__, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, _color); \
        lv_obj_set_style_local_text_font(l__, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, _font); \
    } while(0)

    /* Column title */
    SI_LABEL(parent, title, x + 150, 10, 300, LV_COLOR_YELLOW, &MyCJK_16);

    /* Header row */
    static const char *hdr[] = {"Name", "ADC", "Raw", "Value"};
    lv_coord_t hcxs[] = {name_cx, adc_cx, raw_cx, val_cx};
    lv_coord_t hws[]  = {cw_name, cw_adc, cw_raw, cw_val};
    for (int c = 0; c < 4; c++)
        SI_LABEL(parent, hdr[c], hcxs[c], hdr_y, hws[c], LV_COLOR_GRAY, &MyCJK_16);

    /* ADC unit info per channel */
    static const char *adc_info[] = {
        "ADC1.1","ADC1.2","ADC1.3","ADC1.4","ADC1.5",
        "ADC2.1","ADC2.2","ADC2.3","ADC2.4"
    };
    static const char *ch_short[] = {
        "TX Pwr1","REF Pwr1","TX Pwr2","REF Pwr2",
        "RF In","Current","Voltage","Temp","Detector"
    };

    for (int i = 0; i < count; i++) {
        int ch = start_ch + i;
        lv_coord_t y = hdr_y + 22 + i * row_h;

        SI_LABEL(parent, ch_short[ch],  name_cx, y, cw_name, LV_COLOR_WHITE, &lv_font_montserrat_16);
        SI_LABEL(parent, adc_info[ch],  adc_cx,  y, cw_adc,  LV_COLOR_WHITE, &lv_font_montserrat_16);

        raw_labels[ch] = lv_label_create(parent, NULL);
        lv_label_set_text(raw_labels[ch], "---");
        lv_obj_set_width(raw_labels[ch], cw_raw);
        lv_obj_set_pos(raw_labels[ch], raw_cx - cw_raw / 2, y);
        lv_label_set_align(raw_labels[ch], LV_LABEL_ALIGN_CENTER);
        lv_obj_set_style_local_text_color(raw_labels[ch], LV_LABEL_PART_MAIN,
                                           LV_STATE_DEFAULT, LV_COLOR_WHITE);
        lv_obj_set_style_local_text_font(raw_labels[ch], LV_LABEL_PART_MAIN,
                                          LV_STATE_DEFAULT, &lv_font_montserrat_16);

        val_labels[ch] = lv_label_create(parent, NULL);
        lv_label_set_text(val_labels[ch], "---");
        lv_obj_set_width(val_labels[ch], cw_val);
        lv_obj_set_pos(val_labels[ch], val_cx - cw_val / 2, y);
        lv_label_set_align(val_labels[ch], LV_LABEL_ALIGN_CENTER);
        lv_obj_set_style_local_text_color(val_labels[ch], LV_LABEL_PART_MAIN,
                                           LV_STATE_DEFAULT, LV_COLOR_YELLOW);
        lv_obj_set_style_local_text_font(val_labels[ch], LV_LABEL_PART_MAIN,
                                          LV_STATE_DEFAULT, &lv_font_montserrat_16);
    }
#undef SI_LABEL
}

static void show_sensor_inspect(void)
{
    if (si_overlay) { lv_obj_del(si_overlay); si_overlay = NULL; }

    lv_coord_t o_w = HOR_RES * 4 / 5;
    lv_coord_t o_h = VER_RES * 4 / 5;
    si_overlay = lv_obj_create(lv_scr_act(), NULL);
    lv_obj_set_size(si_overlay, o_w, o_h);
    lv_obj_align(si_overlay, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_local_bg_color(si_overlay, LV_OBJ_PART_MAIN,
                                     LV_STATE_DEFAULT, COLOR_BG);
    lv_obj_set_style_local_radius(si_overlay, LV_OBJ_PART_MAIN,
                                   LV_STATE_DEFAULT, 0);

    lv_obj_t *title = lv_label_create(si_overlay, NULL);
    lv_label_set_text(title, gui_get_string(STR_SENSOR_INSPECT));
    lv_obj_align(title, si_overlay, LV_ALIGN_IN_TOP_MID, 0, 8);
    lv_obj_set_style_local_text_color(title, LV_LABEL_PART_MAIN,
                                       LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(title, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &MyCJK_16);

    /* Two panels: ADC1 (ch 0-4) left, ADC2 (ch 5-8) right */
    si_build_col(si_overlay, 10, 0, 5, "ADC1", &si_val_labels[0], &si_val_labels[ADC_CHANNEL_COUNT]);
    si_build_col(si_overlay, o_w / 2 + 10, 5, 4, "ADC2",
                 &si_val_labels[0], &si_val_labels[ADC_CHANNEL_COUNT]);

    lv_obj_t *close_btn = lv_btn_create(si_overlay, NULL);
    lv_obj_set_size(close_btn, 120, 36);
    lv_obj_align(close_btn, si_overlay, LV_ALIGN_IN_BOTTOM_MID, 0, -8);
    lv_obj_t *cl = lv_label_create(close_btn, NULL);
    lv_label_set_text(cl, gui_get_string(STR_CANCEL));
    lv_obj_set_style_local_text_font(cl, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &MyCJK_16);
    lv_obj_set_event_cb(close_btn, si_close_cb);
}

/* Update sensor inspect values — called from main loop */
void gui_update_sensor_inspect(void)
{
    if (!si_overlay) return;
    char buf[48];
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
#ifndef LVGL_SIM
        uint16_t raw = g_adc_data.raw[i];
        float val;
        const char *unit = "";
        switch (i) {
            case ADC_CHANNEL_TX_POWER1:    val = g_adc_data.tx_power1; unit = "W";  break;
            case ADC_CHANNEL_REF_POWER1:   val = g_adc_data.ref_power1; unit = "W";  break;
            case ADC_CHANNEL_TX_POWER2:    val = g_adc_data.tx_power2; unit = "W";  break;
            case ADC_CHANNEL_REF_POWER2:   val = g_adc_data.ref_power2; unit = "W";  break;
            case ADC_CHANNEL_RF_INPUT:     val = g_adc_data.rf_input;   unit = "W";  break;
            case ADC_CHANNEL_MAIN_CURRENT: val = g_adc_data.current;    unit = "A";  break;
            case ADC_CHANNEL_MAIN_VOLTAGE: val = g_adc_data.voltage;    unit = "V";  break;
            case ADC_CHANNEL_NTC_TEMP:     val = g_adc_data.ntc_temp;   unit = "C";  break;
            case ADC_CHANNEL_DETECTOR:     val = g_adc_data.detector;   unit = "V";  break;
            default: val = 0; break;
        }
        snprintf(buf, sizeof(buf), "%u", raw);
        if (si_val_labels[i]) lv_label_set_text(si_val_labels[i], buf);

        if (i == ADC_CHANNEL_NTC_TEMP) {
            float f = val * 9.0f / 5.0f + 32.0f;
            snprintf(buf, sizeof(buf), "%.0f%s/%.0fF", val, unit, f);
        } else {
            snprintf(buf, sizeof(buf), "%.1f%s", val, unit);
        }
        if (si_val_labels[i + ADC_CHANNEL_COUNT])
            lv_label_set_text(si_val_labels[i + ADC_CHANNEL_COUNT], buf);
#else
        uint16_t sim = 500 + (uint16_t)(rand() % 2001);
        snprintf(buf, sizeof(buf), "%u", sim);
        if (si_val_labels[i]) lv_label_set_text(si_val_labels[i], buf);
        if (i == ADC_CHANNEL_NTC_TEMP) {
            snprintf(buf, sizeof(buf), "45C/113F");
        } else {
            snprintf(buf, sizeof(buf), "%.1fW", sim * 0.01f);
        }
        if (si_val_labels[i + ADC_CHANNEL_COUNT])
            lv_label_set_text(si_val_labels[i + ADC_CHANNEL_COUNT], buf);
#endif
    }
}

/*============================================================================
 * Fan Settings Popup
 *============================================================================*/

static const char *fan_btns[] = {"Auto", "20%", "40%", "60%", "80%", "100%", "\n", "Cancel", ""};

static void fan_sel_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
        uint16_t btn_id = lv_msgbox_get_active_btn(obj);
        lv_obj_del_async(obj);
        if (btn_id <= 5) {
            static const uint8_t speeds[] = {0, 20, 40, 60, 80, 100};
#ifndef LVGL_SIM
            power_set_fan_speed(speeds[btn_id]);
#endif
        }
    }
}

static void show_fan_settings(void)
{
    fan_btns[0] = gui_get_string(STR_AUTO);
    fan_btns[7] = gui_get_string(STR_CANCEL);

    lv_obj_t *scr = lv_scr_act();
    lv_obj_t *mbox = lv_msgbox_create(scr, NULL);
    lv_msgbox_set_text(mbox, gui_get_string(STR_FAN));
    lv_msgbox_add_btns(mbox, fan_btns);
    lv_obj_set_size(mbox, 320, 220);
    lv_obj_align(mbox, scr, LV_ALIGN_CENTER, 0, 0);

    lv_obj_set_style_local_text_color(mbox, LV_MSGBOX_PART_BG,
                                       LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(mbox, LV_MSGBOX_PART_BG,
                                      LV_STATE_DEFAULT, &MyCJK_16);
    lv_obj_set_style_local_bg_color(mbox, LV_MSGBOX_PART_BG,
                                     LV_STATE_DEFAULT, LV_COLOR_BLUE);
    lv_obj_t *btnm = lv_msgbox_get_btnmatrix(mbox);
    lv_obj_set_style_local_text_color(btnm, LV_BTNMATRIX_PART_BTN,
                                      LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(btnm, LV_BTNMATRIX_PART_BTN,
                                      LV_STATE_DEFAULT, &MyCJK_16);
    lv_obj_set_style_local_bg_color(btnm, LV_BTNMATRIX_PART_BTN,
                                     LV_STATE_DEFAULT, LV_COLOR_MAKE(0, 160, 225));
    lv_obj_set_style_local_border_color(btnm, LV_BTNMATRIX_PART_BTN,
                                         LV_STATE_DEFAULT, LV_COLOR_GRAY);
    lv_obj_set_style_local_radius(btnm, LV_BTNMATRIX_PART_BTN,
                                   LV_STATE_DEFAULT, 0);
    lv_obj_set_event_cb(mbox, fan_sel_cb);
}

/*============================================================================
 * Band Communication Settings Popup
 *============================================================================*/

static const char *band_comm_btns[] = {"ICOM", "Yaesu", "Kenwood", "\n", "Cancel", ""};

static void band_comm_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
        uint16_t btn_id = lv_msgbox_get_active_btn(obj);
        lv_obj_del_async(obj);
#ifndef LVGL_SIM
        if (btn_id <= 2) {
            g_config.radio_type = (RadioType)(btn_id + 1);
            config_save();
        }
#endif
    }
}

static void show_band_comm_settings(void)
{
    band_comm_btns[4] = gui_get_string(STR_CANCEL);

    lv_obj_t *scr = lv_scr_act();
    lv_obj_t *mbox = lv_msgbox_create(scr, NULL);
    lv_msgbox_set_text(mbox, gui_get_string(STR_BAND));
    lv_msgbox_add_btns(mbox, band_comm_btns);
    lv_obj_set_size(mbox, 320, 180);
    lv_obj_align(mbox, scr, LV_ALIGN_CENTER, 0, 0);

    lv_obj_set_style_local_text_color(mbox, LV_MSGBOX_PART_BG,
                                       LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(mbox, LV_MSGBOX_PART_BG,
                                      LV_STATE_DEFAULT, &MyCJK_16);
    lv_obj_set_style_local_bg_color(mbox, LV_MSGBOX_PART_BG,
                                     LV_STATE_DEFAULT, LV_COLOR_BLUE);
    lv_obj_t *btnm = lv_msgbox_get_btnmatrix(mbox);
    lv_obj_set_style_local_text_color(btnm, LV_BTNMATRIX_PART_BTN,
                                      LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(btnm, LV_BTNMATRIX_PART_BTN,
                                      LV_STATE_DEFAULT, &MyCJK_16);
    lv_obj_set_style_local_bg_color(btnm, LV_BTNMATRIX_PART_BTN,
                                     LV_STATE_DEFAULT, LV_COLOR_MAKE(0, 160, 225));
    lv_obj_set_style_local_border_color(btnm, LV_BTNMATRIX_PART_BTN,
                                         LV_STATE_DEFAULT, LV_COLOR_GRAY);
    lv_obj_set_style_local_radius(btnm, LV_BTNMATRIX_PART_BTN,
                                   LV_STATE_DEFAULT, 0);
    lv_obj_set_event_cb(mbox, band_comm_cb);
}

/*============================================================================
 * Protection Settings Screen
 *============================================================================*/

static lv_obj_t *prot_screen = NULL;
static lv_obj_t *prot_container = NULL;
static lv_obj_t *prot_sliders[8];
static lv_obj_t *prot_val_labels[8];

#ifdef LVGL_SIM
/* Simulator local storage */
static float sim_prot_vals[8] = {400, 200, 2.0f, 600, 40, 90, 1100, 1050};
static float sim_prot_backup[8];
#else
static ConfigType prot_backup;
#endif

typedef struct {
    const char *name;
    float *ptr;
    float vmin, vmax;
    const char *unit;
    int scale;
} ProtField;

static const ProtField prot_fields[] = {
#ifndef LVGL_SIM
    {"REF Pwr2",    &g_config.protect_reflect_power2_w,     100, 600, "W",  1},
    {"REF Pwr2+SWR",&g_config.protect_reflect_power2_swr,    50, 400, "W",  1},
    {"SWR",         &g_config.protect_swr_threshold,          1,   5, "",  10},
    {"REF Pwr1",    &g_config.protect_reflect_power1_w,     100,1000, "W",  1},
    {"Main I",      &g_config.protect_main_current_a,        10,  80, "A",  1},
    {"Temp",        &g_config.protect_ntc_temp_c,            60, 120, "C",  1},
    {"TX Pwr1",     &g_config.protect_tx_power1_w,          500,1500, "W",  1},
    {"TX Pwr2",     &g_config.protect_tx_power2_w,          500,1500, "W",  1},
#else
    {"REF Pwr2",    &sim_prot_vals[0], 100, 600, "W",  1},
    {"REF Pwr2+SWR",&sim_prot_vals[1],  50, 400, "W",  1},
    {"SWR",         &sim_prot_vals[2],   1,   5, "",  10},
    {"REF Pwr1",    &sim_prot_vals[3], 100,1000, "W",  1},
    {"Main I",      &sim_prot_vals[4],  10,  80, "A",  1},
    {"Temp",        &sim_prot_vals[5],  60, 120, "C",  1},
    {"TX Pwr1",     &sim_prot_vals[6], 500,1500, "W",  1},
    {"TX Pwr2",     &sim_prot_vals[7], 500,1500, "W",  1},
#endif
};
#define PROT_COUNT (sizeof(prot_fields) / sizeof(prot_fields[0]))

static void prot_slider_cb(lv_obj_t *slider, lv_event_t e)
{
    if (e == LV_EVENT_VALUE_CHANGED) {
        int idx = -1;
        for (int i = 0; i < 8; i++) {
            if (prot_sliders[i] == slider) { idx = i; break; }
        }
        if (idx < 0 || !prot_val_labels[idx]) return;
        int16_t sv = lv_slider_get_value(slider);
        float val = (float)sv / (float)prot_fields[idx].scale;
        char buf[48];
        if (prot_fields[idx].unit[0] == 'C') {
            float f = val * 9.0f / 5.0f + 32.0f;
            snprintf(buf, sizeof(buf), "%.0f%s / %.0fF", val, prot_fields[idx].unit, f);
        } else {
            snprintf(buf, sizeof(buf), "%.1f%s", val, prot_fields[idx].unit);
        }
        lv_label_set_text(prot_val_labels[idx], buf);
    }
}

static void prot_cancel_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
#ifndef LVGL_SIM
        *(ConfigType*)&g_config = prot_backup;
#else
        for (int i = 0; i < 8; i++)
            *prot_fields[i].ptr = sim_prot_backup[i];
#endif
        prot_screen = NULL; lv_scr_load(main_screen);
    }
}

static void prot_save_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
#ifndef LVGL_SIM
        config_save();
#endif
        prot_screen = NULL; lv_scr_load(main_screen);
    }
}

static void prot_slider_create(lv_obj_t *parent, int idx, lv_coord_t y)
{
    const ProtField *pf = &prot_fields[idx];
    char buf[64];

    lv_obj_t *name_lbl = lv_label_create(parent, NULL);
    snprintf(buf, sizeof(buf), "%s (%s)", pf->name, pf->unit);
    lv_label_set_text(name_lbl, buf);
    lv_obj_set_pos(name_lbl, HOR_RES / 20, y);
    lv_obj_set_style_local_text_color(name_lbl, LV_LABEL_PART_MAIN,
                                       LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(name_lbl, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &MyCJK_16);

    lv_obj_t *val_lbl = lv_label_create(parent, NULL);
    float init_val = *pf->ptr;
    if (pf->unit[0] == 'C') {
        float f = init_val * 9.0f / 5.0f + 32.0f;
        snprintf(buf, sizeof(buf), "%.0f%s / %.0fF", init_val, pf->unit, f);
    } else {
        snprintf(buf, sizeof(buf), "%.1f%s", init_val, pf->unit);
    }
    lv_label_set_text(val_lbl, buf);
    lv_obj_set_pos(val_lbl, HOR_RES * 7 / 10, y);
    lv_obj_set_style_local_text_color(val_lbl, LV_LABEL_PART_MAIN,
                                       LV_STATE_DEFAULT, LV_COLOR_YELLOW);
    lv_obj_set_style_local_text_font(val_lbl, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &MyCJK_16);

    lv_obj_t *slider = lv_slider_create(parent, NULL);
    lv_obj_set_size(slider, HOR_RES * 6 / 10, 20);
    lv_obj_set_pos(slider, HOR_RES / 20, y + 22);
    lv_slider_set_range(slider, (int16_t)(pf->vmin * pf->scale),
                                 (int16_t)(pf->vmax * pf->scale));
    lv_slider_set_value(slider, (int16_t)(init_val * pf->scale), LV_ANIM_OFF);

    prot_sliders[idx] = slider;
    prot_val_labels[idx] = val_lbl;
    lv_obj_set_event_cb(slider, prot_slider_cb);
}

static void show_protection_settings(void)
{
    if (prot_screen) { lv_obj_del(prot_screen); prot_screen = NULL; }

#ifndef LVGL_SIM
    prot_backup = g_config;
#else
    for (int i = 0; i < 8; i++)
        sim_prot_backup[i] = *prot_fields[i].ptr;
#endif

    prot_screen = lv_obj_create(NULL, NULL);
    lv_obj_set_size(prot_screen, HOR_RES, VER_RES);
    lv_obj_set_style_local_bg_color(prot_screen, LV_OBJ_PART_MAIN,
                                     LV_STATE_DEFAULT, COLOR_BG);
    lv_scr_load(prot_screen);

    prot_container = lv_obj_create(prot_screen, NULL);
    lv_obj_set_size(prot_container, HOR_RES, VER_RES);
    lv_obj_set_pos(prot_container, 0, 0);
    lv_obj_set_style_local_bg_opa(prot_container, LV_OBJ_PART_MAIN,
                                   LV_STATE_DEFAULT, LV_OPA_TRANSP);
    lv_obj_set_style_local_border_width(prot_container, LV_OBJ_PART_MAIN,
                                        LV_STATE_DEFAULT, 0);

    lv_obj_t *title = lv_label_create(prot_container, NULL);
    lv_label_set_text(title, "Protection Settings");
    lv_obj_align(title, prot_container, LV_ALIGN_IN_TOP_MID, 0, 10);
    lv_obj_set_style_local_text_color(title, LV_LABEL_PART_MAIN,
                                       LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(title, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &MyCJK_16);

    for (int i = 0; i < (int)PROT_COUNT; i++) {
        prot_slider_create(prot_container, i, 40 + i * 50);
    }

    lv_obj_t *save_btn = lv_btn_create(prot_container, NULL);
    lv_obj_set_pos(save_btn, HOR_RES / 2 - 130, VER_RES - 60);
    lv_obj_set_size(save_btn, 120, 40);
    lv_obj_t *save_lbl = lv_label_create(save_btn, NULL);
    lv_label_set_text(save_lbl, gui_get_string(STR_SAVE));
    lv_obj_set_style_local_text_font(save_lbl, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &MyCJK_16);
    lv_obj_set_event_cb(save_btn, prot_save_cb);

    lv_obj_t *cancel_btn = lv_btn_create(prot_container, NULL);
    lv_obj_set_pos(cancel_btn, HOR_RES / 2 + 10, VER_RES - 60);
    lv_obj_set_size(cancel_btn, 120, 40);
    lv_obj_t *cancel_lbl = lv_label_create(cancel_btn, NULL);
    lv_label_set_text(cancel_lbl, gui_get_string(STR_CANCEL));
    lv_obj_set_style_local_text_font(cancel_lbl, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &MyCJK_16);
    lv_obj_set_event_cb(cancel_btn, prot_cancel_cb);
}

/* Static so lv_btnmatrix_set_map pointer stays valid after function return */
static const char *settings_zh[] = {
    "校准传感器", "\n",
    "检查传感器", "\n",
    "风扇", "\n",
    "波段通信", "\n",
    "保护", "\n",
    "屏幕", "\n",
    "语言 Language", "\n",
    "取消", ""
};
static const char *settings_en[] = {
    "Calibrate Sensors", "\n",
    "Sensors", "\n",
    "Fan", "\n",
    "Band Communication", "\n",
    "Protection", "\n",
    "Screen", "\n",
    "Language 语言", "\n",
    "Cancel", ""
};

void gui_toggle_menu(void)
{
    const char **btns = (gui_get_lang_index() == 0) ? settings_zh : settings_en;

    lv_obj_t *overlay = lv_obj_create(lv_scr_act(), NULL);
    lv_obj_set_size(overlay, HOR_RES * 3 / 4, VER_RES * 7 / 8);
    lv_obj_align(overlay, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_local_bg_color(overlay, LV_OBJ_PART_MAIN,
                                     LV_STATE_DEFAULT, COLOR_BG);
    lv_obj_set_style_local_radius(overlay, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, 0);

    lv_obj_t *btnm = lv_btnmatrix_create(overlay, NULL);
    lv_btnmatrix_set_map(btnm, btns);
    lv_obj_set_size(btnm, HOR_RES * 3 / 4 - 20, VER_RES * 7 / 8 - 20);
    lv_obj_align(btnm, NULL, LV_ALIGN_CENTER, 0, 0);

    /* Button matrix background */
    lv_obj_set_style_local_bg_color(btnm, LV_BTNMATRIX_PART_BG,
                                     LV_STATE_DEFAULT, COLOR_BG);
    lv_obj_set_style_local_pad_inner(btnm, LV_BTNMATRIX_PART_BG,
                                      LV_STATE_DEFAULT, 16);

    /* Button style (matching OPA1000 and band selector) */
    lv_obj_set_style_local_text_color(btnm, LV_BTNMATRIX_PART_BTN,
                                      LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(btnm, LV_BTNMATRIX_PART_BTN,
                                      LV_STATE_DEFAULT, &MyCJK_16);
    lv_obj_set_style_local_bg_color(btnm, LV_BTNMATRIX_PART_BTN,
                                     LV_STATE_DEFAULT, LV_COLOR_MAKE(0, 160, 225));
    lv_obj_set_style_local_border_color(btnm, LV_BTNMATRIX_PART_BTN,
                                         LV_STATE_DEFAULT, LV_COLOR_GRAY);
    lv_obj_set_style_local_radius(btnm, LV_BTNMATRIX_PART_BTN,
                                   LV_STATE_DEFAULT, 0);

    lv_obj_set_event_cb(btnm, settings_menu_cb);
}

void gui_toggle_band_popup(void) { }
void gui_focus_prev(void) { }
void gui_focus_next(void) { }
void gui_enter_pressed(void) { }
