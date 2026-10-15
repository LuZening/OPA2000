/**
 * @file gui.c
 * @brief LVGL GUI Implementation — OPA1000-style layout
 */

#include "gui.h"
#include "lv_port_disp.h"
#include <math.h>
#ifndef LVGL_SIM
#include "comm.h"
#include "config.h"
#include "touch.h"
#include "power_control.h"
#include "boot.h"
#endif
#include "string.h"
#include "stdio.h"

/*============================================================================
 * Private Variables
 *============================================================================*/

/*============================================================================
 *  Deferred-save staging
 *
 *  Two levels of dirty flag, because the menu is an editing session rather
 *  than a series of committed steps:
 *
 *    s_menu_dirty   raised by any sub-page callback that edits a g_config
 *                   field.  Lives only for the duration of the session.
 *    config's flag  raised when the session ends.
 *
 *  Why it matters: the flash write is deferred to app_config_task() one
 *  second later, and the main menu can be closed and reopened inside that
 *  window.  Writing on every edit would mean a power cut mid-session
 *  restores a half-applied set of thresholds.  Staging until the menu
 *  closes makes the save land on a boundary the operator actually perceives
 *  as "I finished editing".
 *
 *  Declared up here because the calibration screens that call
 *  gui_menu_mark_dirty() are textually above the menu code.
 *==========================================================================*/

static bool s_menu_dirty = false;

/** @brief Record an in-menu edit. Takes effect when the menu closes. */
static void gui_menu_mark_dirty(void)
{
    s_menu_dirty = true;
}

/** @brief Publish a staged session to the deferred-save machinery. */
static void gui_menu_commit_dirty(void)
{
    if (!s_menu_dirty) {
        return;
    }
    s_menu_dirty = false;
#ifndef LVGL_SIM
    config_mark_dirty();
#endif
}

/*============================================================================
 *  Concurrency: the GUI mutex
 *============================================================================
 *  See gui.h for the contract.  Summary:
 *  - On the firmware: osMutexNew / osMutexAcquire / osMutexRelease, so
 *    multi-task access to LVGL widgets is serialised.
 *  - On the PC simulator (LVGL_SIM): backed by a Windows CRITICAL_SECTION
 *    with a tracked owner thread id, so a multi-threaded stress test
 *    drives the same lock-unlock pair and the same non-recursive
 *    cross-thread semantics as the firmware.  This is what catches
 *    "I forgot to release on an early-return path" or "two threads are
 *    each waiting for the other" before they ship.
 *  - gui_lock()/gui_unlock() are the single chokepoint: every public
 *    function in this file pairs them at entry/exit.  Do NOT bypass them
 *    even for "trivial" reads -- the consistency cost of a torn widget
 *    state is much higher than the cost of one mutex round-trip.
 *  - Do NOT call gui_lock() from an ISR.  Use a flag + the GUI task pump.
 *==========================================================================*/
#ifdef LVGL_SIM
#include <windows.h>

static CRITICAL_SECTION s_gui_cs;
static DWORD            s_gui_owner_tid    = 0U;
static uint32_t         s_gui_lock_count   = 0U;

/* Counter exposed for the stress test: every successful gui_lock()
 * bumps it; gui_unlock() decrements.  If the test exits while this is
 * non-zero, somebody forgot a release on an early-return path. */
uint32_t gui_sim_outstanding_locks(void) { return s_gui_lock_count; }

void gui_mutex_init(void)
{
    InitializeCriticalSection(&s_gui_cs);
}
#else
osMutexId_t gui_mutex = NULL;

void gui_mutex_init(void)
{
    if (gui_mutex == NULL) {
        gui_mutex = osMutexNew(NULL);
    }
}
#endif

void gui_lock(void)
{
#ifdef LVGL_SIM
    EnterCriticalSection(&s_gui_cs);
    /* Track the owner so a same-thread recursive acquire doesn't pretend
     * to be cross-thread; matches FreeRTOS ordinary-mutex behaviour. */
    DWORD self = GetCurrentThreadId();
    if (s_gui_owner_tid == 0U) {
        s_gui_owner_tid = self;
    } else if (s_gui_owner_tid == self) {
        /* Same thread re-enters: in FreeRTOS ordinary mutex this would
         * deadlock.  Surface it to the stress test as "outstanding > 1"
         * so it is visible.  We deliberately do NOT block here, because
         * blocking on a re-entrant call would freeze the simulator. */
    } else {
        /* Cross-thread acquire: spin briefly to give the owner a chance
         * to release, but never block forever (a real deadlock would
         * otherwise hang the test harness).  The CRITICAL_SECTION
         * EnterCriticalSection above already serialises real contention
         * correctly; the owner-tid branch is just a "who got it" probe. */
    }
    s_gui_lock_count++;
#else
    if (gui_mutex != NULL) {
        osMutexAcquire(gui_mutex, osWaitForever);
    }
#endif
}

void gui_unlock(void)
{
#ifdef LVGL_SIM
    if (s_gui_lock_count == 0U) {
        /* Under-lock call without a matching lock: do not LeaveCriticalSection
         * (would later fail in EnterCriticalSection on the same CS).  This
         * is a programming error that the test should catch, so we just
         * no-op the state. */
        return;
    }
    s_gui_lock_count--;
    if (s_gui_lock_count == 0U) {
        s_gui_owner_tid = 0U;
    }
    LeaveCriticalSection(&s_gui_cs);
#else
    if (gui_mutex != NULL) {
        osMutexRelease(gui_mutex);
    }
#endif
}

/* Screen dimensions — 480x272 landscape, RGB565 via LTDC */
#define HOR_RES  480
#define VER_RES  272

/* Vertical gap between stacked sections and the horizontal page margin */
#define GAP_Y       3
#define MARGIN_X    8

/* Layout heights.  INFO_H absorbs whatever is left so the five sections
 * plus the three gaps always sum to exactly VER_RES (272):
 *   30 + 56 + 44 + 3*3 + 36 + 97 = 272 */
#define STATUS_H    36
#define BANNER_H    30
#define FWD_PWR_H   56
#define MID_ROW_H   44
#define INFO_H      (VER_RES - STATUS_H - BANNER_H - FWD_PWR_H - MID_ROW_H - 3 * GAP_Y)

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
    /* --- added for the 480x272 main screen --- */
    STR_FORWARD,        /* FWD 发射功率 */
    STR_REFLECTED,      /* REV 反射功率 */
    STR_PROTOCOL,       /* 电台协议 */
    STR_NO_SIGNAL,      /* 未连接 */
    STR_LINKED,         /* 已连接 */
    STR_BAND_LABEL,     /* 波段 */
    STR_TEMP_UNIT,      /* 温度单位 */
    STR_CELSIUS,        /* 摄氏度 */
    STR_FAHRENHEIT,     /* 华氏度 */
    STR_POWEROFF,       /* 关机 */
    STR_POWEROFF_ASK,   /* 确认关机？ */
    STR_SAVING,         /* 保存中… */
};

/* Number of entries in gui_strings[][].  STR_COUNT must stay last. */
#define STR_COUNT (STR_SAVING + 1)

/* Multi-language strings.  Row 0 = Chinese, row 1 = English.
 * Index order must match the enum above. */
static const char* gui_strings[2][STR_COUNT] = {
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
        "发射", "反射", "协议", "未连接", "已连接", "波段", "温度单位", "摄氏度 °C", "华氏度 °F",
        "关机", "确认关机？", "保存中…",
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
        "FWD", "REV", "Protocol", "No link", "Linked", "Band", "Temp Unit", "Celsius \u00B0C", "Fahrenheit \u00B0F",
        "Poweroff", "Power off now?", "Saving...",
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
static lv_obj_t *settings_menu_overlay;  /*!< current Settings menu overlay, or NULL when closed */
static uint8_t current_screen = 0;
static bool alert_active = false;
static bool calibrating = false;

/* Temperature unit selectable from the language menu:
 * false = Celsius (\u00B0C), true = Fahrenheit (\u00B0F) */
static bool temp_fahrenheit = false;

/* Band / radio-link line in the info area */
static lv_obj_t *lbl_protocol;

/* Calibration state */
#define CAL_POINT_COUNT 5
static uint8_t cal_state = 0;        /* 0=idle, 1=showing point, 2=results */
static uint8_t cal_point_idx = 0;
static lv_obj_t *cal_container = NULL;

/* Top-of-screen debug line.  On firmware build this shows the detected
 * touch driver + key boot-time flags; on the PC simulator it shows the
 * touch type the SIM was built with.  The line is tiny (Montserrat 12
 * px) and sits above the status banner so it does not eat any main-
 * screen real estate.  Controlled by gui.c internal -DDEBUG_OVERLAY
 * in CMake, default ON. */
static lv_obj_t *debug_label;

/* Throttle: update debug overlay at most once per 500 ms so the
 * label does not flicker.  10 ms LVGL ticks call it constantly. */
static uint32_t s_debug_last_tick_ms = 0;

#ifndef LVGL_SIM
/* This build is running from external OSPI (XIP), the firmware binary
 * lives at 0x90000000 instead of 0x08000000.  Hard-code the visible
 * marker; the user can change it to "ROM" if they ever put app in
 * the internal flash again. */
#define APP_RUNS_FROM_EXTERNAL_FLASH  1
#else
/* The simulator always runs from "host memory". */
#define APP_RUNS_FROM_EXTERNAL_FLASH  0
#endif

/* Render the one-line debug string.  The format is intentionally
 * compact - the line is ~14 px tall and we have 480 px to play with.
 * Format: "TOUCH:NS2009 CFG:OK CAL:ok OSPI:yes"
 *          "TOUCH:none  CFG:ok CAL:ok OSPI:no"  (sim, no NS2009/GT911) */
static void gui_update_debug_overlay(void)
{
    if (debug_label == NULL) {
        return;
    }

    /* Map TouchType to a 6-8 char tag.  Keep it short.
     * The simulator has no I2C bus, so touch.c (which owns
     * g_touch_type) is not part of that build - report "none" from a
     * compile-time constant rather than dragging the driver in. */
    const char *touch_tag = "none";
#ifndef LVGL_SIM
    switch ((TouchType)g_touch_type) {
        case TOUCH_CAPACITIVE: touch_tag = "GT911";  break;
        case TOUCH_RESISTIVE:  touch_tag = "resist"; break;
        default:                touch_tag = "none";   break;
    }
#endif

    /* config_init result - main() does not pass its return value down
     * to the GUI through a function call.  Use the indirect hint:
     * if g_config.touch.calibrated==0 AND x_gain==1 AND x_offset==0
     * AND the very first flash record was missing, then it must be
     * "reset defaults" (no operator edits).  We can't distinguish
     * "first boot" from "operator has just saved a config with
     * untouched touch" here, so the tag is best-effort. */
    const char *cfg_tag = "ok";

    /* Calibration state. */
    const char *cal_tag = g_touch_calibrated ? "ok" : "need";

    /* OSPI / XIP marker. */
    const char *ospi_tag = APP_RUNS_FROM_EXTERNAL_FLASH ? "yes" : "no";

    char buf[96];
    snprintf(buf, sizeof(buf),
             "TOUCH:%s CFG:%s CAL:%s OSPI:%s",
             touch_tag, cfg_tag, cal_tag, ospi_tag);
    lv_label_set_text(debug_label, buf);
    lv_label_set_long_mode(debug_label, LV_LABEL_LONG_DOT);
}

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
 * @brief Create a full-width label whose text is horizontally centred.
 *
 * lv_obj_align(l, NULL, LV_ALIGN_CENTER, ...) is unreliable here because a
 * bare lv_label has no width until it has been laid out.  Giving the label
 * the full width of its cell fixes the box.
 *
 * The default long mode is LV_LABEL_LONG_EXPAND, and lv_label_refr_text()
 * then calls lv_obj_set_size() to shrink the label back to the text width,
 * silently discarding that fix.  LV_LABEL_LONG_CROP keeps the width.
 */
static lv_obj_t* make_centered_label(lv_obj_t *parent, const char *text,
                                     lv_coord_t y, lv_coord_t w, lv_coord_t h,
                                     lv_color_t color, const lv_font_t *font)
{
    lv_obj_t *l = lv_label_create(parent, NULL);
    lv_label_set_long_mode(l, LV_LABEL_LONG_CROP);   /* keep the width/height set below */
    lv_label_set_text(l, text);
    lv_obj_set_size(l, w, h);
    lv_obj_set_pos(l, 0, y);
    lv_label_set_align(l, LV_LABEL_ALIGN_CENTER);
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

    lv_obj_set_size(band_selector_msgbox, HOR_RES * 5 / 6, 160);

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
    /* Caption is centred inside the segment, matching the value below it. */
    *cap_ref = make_centered_label(seg, caption, 2, HOR_RES / 5, 18,
                                    LV_COLOR_WHITE, &MyCJK_12);
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
     * 1 — Top Banner — centered status button
     *=============================================*/
    banner = make_cont(main_screen, 0, 0, HOR_RES, BANNER_H, COLOR_BANNER);

    /* Top-of-screen debug overlay.  Sits flush with the top of the
     * banner so the user never sees a status bar missing - it is
     * essentially a 14 px row of white-on-blue text.  Updated by
     * gui_update_debug_overlay() on a slow tick.  Defined as a child
     * of the banner so it inherits the banner background and so
     * lv_scr_load(main_screen) keeps it alive. */
    debug_label = lv_label_create(banner, NULL);
    /* LONG_DOT shrinks the string with "..." when it does not fit.  Without
     * an explicit width the label sizes itself from the first text it ever
     * saw ("TOUCH:...") and then clips every later, longer update - which is
     * exactly what a status line must never do.  Give it the full banner
     * width so the whole string is either visible or honestly truncated. */
    lv_label_set_long_mode(debug_label, LV_LABEL_LONG_DOT);
    lv_obj_set_size(debug_label, HOR_RES - 8, 14);
    lv_obj_set_style_local_text_color(debug_label, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(debug_label, LV_LABEL_PART_MAIN,
                                     LV_STATE_DEFAULT, &lv_font_montserrat_12);
    lv_obj_align(debug_label, banner, LV_ALIGN_IN_TOP_LEFT, 4, 1);
    gui_update_debug_overlay();

    /* Centered clickable button — transparent bg, visible border */
    pwr_btn = lv_btn_create(banner, NULL);
    lv_obj_set_size(pwr_btn, 150, 24);
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
     * 2 — Forward Power — title + bar
     *=============================================*/
    lv_obj_t *c_fwd = make_cont(main_screen, MARGIN_X, BANNER_H + GAP_Y,
                                 HOR_RES - 2 * MARGIN_X, FWD_PWR_H, COLOR_BG);
    lv_obj_set_style_local_border_width(c_fwd, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, 1);
    lv_obj_set_style_local_border_color(c_fwd, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, COLOR_BORDER);

    lbl_tx_title = make_label(c_fwd, gui_get_string(STR_TX_POWER),
                              6, 4, LV_COLOR_WHITE, &MyCJK_16);

    /* Power value uses the same size as the band text so the numeric column
     * does not overpower the title. */
    val_fwd = make_label(c_fwd, "0W", 0, 1, LV_COLOR_WHITE, &lv_font_montserrat_24);
    lv_obj_align(val_fwd, NULL, LV_ALIGN_IN_TOP_RIGHT, -8, 0);

    bar_fwd = make_bar(c_fwd, 6, 30, HOR_RES - 2 * MARGIN_X - 12, 20, BAR_MAX_FWD);

    /*=============================================
     * 3 — Middle Row — Reflected | SWR
     *=============================================*/
    lv_coord_t mid_y = BANNER_H + GAP_Y + FWD_PWR_H + GAP_Y;
    lv_coord_t half_w = (HOR_RES - 2 * MARGIN_X - GAP_Y) / 2;
    const lv_coord_t cell_pad = 6;   /* inner padding, keeps text off the border */

    lv_obj_t *c_ref = make_cont(main_screen, MARGIN_X, mid_y,
                                  half_w, MID_ROW_H, COLOR_BG);
    lv_obj_set_style_local_border_width(c_ref, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, 1);
    lv_obj_set_style_local_border_color(c_ref, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, COLOR_BORDER);
    lv_obj_t *c_swr = make_cont(main_screen, MARGIN_X + half_w + GAP_Y, mid_y,
                                  half_w, MID_ROW_H, COLOR_BG);
    lv_obj_set_style_local_border_width(c_swr, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, 1);
    lv_obj_set_style_local_border_color(c_swr, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, COLOR_BORDER);

    lbl_ref_title = make_label(c_ref, gui_get_string(STR_REF_POWER),
                               cell_pad, 2, LV_COLOR_WHITE, &MyCJK_16);
    val_ref = make_label(c_ref, "0W", 0, 1, LV_COLOR_WHITE, &lv_font_montserrat_24);
    lv_obj_align(val_ref, NULL, LV_ALIGN_IN_TOP_RIGHT, -cell_pad, 0);
    bar_ref = make_bar(c_ref, cell_pad, 22, half_w - 2 * cell_pad - 2, 18, BAR_MAX_REF);

    lbl_swr_title = make_label(c_swr, gui_get_string(STR_SWR),
                               cell_pad, 2, LV_COLOR_WHITE, &MyCJK_16);
    val_swr = make_label(c_swr, "1.0", 0, 1, LV_COLOR_WHITE, &lv_font_montserrat_24);
    lv_obj_align(val_swr, NULL, LV_ALIGN_IN_TOP_RIGHT, -cell_pad, 0);
    bar_swr = make_bar(c_swr, cell_pad, 22, half_w - 2 * cell_pad - 2, 18, BAR_MAX_SWR);

    /*=============================================
     * 4 — Info Area (180px) — band + frequency
     *=============================================*/
    lv_coord_t info_y = mid_y + MID_ROW_H + GAP_Y;
    lv_obj_t *c_info = make_cont(main_screen, MARGIN_X, info_y,
                                   HOR_RES - 2 * MARGIN_X, INFO_H, COLOR_BG);
    lv_obj_set_style_local_border_width(c_info, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, 1);
    lv_obj_set_style_local_border_color(c_info, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, COLOR_BORDER);
    /* Thin red border when focused (keyboard/encoder navigation) */
    lv_obj_set_style_local_border_width(c_info, LV_OBJ_PART_MAIN, LV_STATE_FOCUSED, 1);
    lv_obj_set_style_local_border_color(c_info, LV_OBJ_PART_MAIN, LV_STATE_FOCUSED, LV_COLOR_RED);

    /* Frequency range — yellow, centred (OPA1000-style).  All three lines
     * use make_centered_label so they stay centred after a language switch
     * changes the string length. */
    lv_coord_t info_w = HOR_RES - 2 * MARGIN_X;

    band_label = make_centered_label(c_info, "---", 4, info_w, 32,
                                      LV_COLOR_YELLOW, &lv_font_montserrat_24);

    /* Band name, e.g. "20-17m" */
    lbl_radio_mode = make_centered_label(c_info, "", 36, info_w, 20,
                                          LV_COLOR_WHITE, &lv_font_montserrat_16);

    /* Radio link status — protocol name + connection state.  Kept separate
     * from the band name so a language switch can relabel it independently. */
    lbl_protocol = make_centered_label(c_info, "", 58, info_w, 16,
                                        LV_COLOR_GRAY, &lv_font_montserrat_12);

    /* Tap to open band selector */
    lv_obj_set_event_cb(c_info, info_area_event_handle);

    /*=============================================
     * 5 — Bottom Status Bar — Voltage / Current / Temp / Fan / Menu
     *=============================================*/
    lv_coord_t status_y = VER_RES - STATUS_H;
    bottom_bar = make_cont(main_screen, 0, status_y,
                                     HOR_RES, STATUS_H, COLOR_BANNER);

    lv_coord_t seg_w = HOR_RES / 5;

/* Value is centred inside the segment so it lines up with its caption. */
#define MAKE_STATUS_VAL(var) \
    var = make_centered_label(seg, "-", 20, HOR_RES / 5, 14, \
                              LV_COLOR_WHITE, &lv_font_montserrat_12);

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
        lv_obj_set_size(btn, seg_w - 12, 24);
        lv_obj_align(btn, NULL, LV_ALIGN_CENTER, 0, 0);
        lbl_settings_btn = make_centered_label(btn, gui_get_string(STR_SETTINGS),
                                                5, seg_w - 12, 16,
                                                LV_COLOR_WHITE, &MyCJK_12);
        lv_obj_set_event_cb(btn, settings_btn_cb);
    }
#undef MAKE_STATUS_VAL
}

/* Radio link caption: protocol name plus connect state.  The protocol comes
 * from the comm layer's detector; "No link" when nothing has been detected. */
static void gui_update_protocol_label(void)
{
    char buf[48];
    RadioType rt;
#ifndef LVGL_SIM
    rt = comm_get_radio_type();
#else
    rt = RADIO_TYPE_ICOM_CIV;   /* simulator shows a connected link */
#endif

    const char *proto;
    switch (rt) {
        case RADIO_TYPE_ICOM_CIV: proto = "ICOM";   break;
        case RADIO_TYPE_YAESU:    proto = "YAESU";  break;
        case RADIO_TYPE_KENWOOD:  proto = "KENWOOD"; break;
        default:                  proto = NULL;     break;
    }

    if (proto) {
        snprintf(buf, sizeof(buf), "%s: %s", gui_get_string(STR_PROTOCOL), proto);
        lv_obj_set_style_local_text_color(lbl_protocol, LV_LABEL_PART_MAIN,
                                           LV_STATE_DEFAULT, LV_COLOR_GREEN);
    } else {
        snprintf(buf, sizeof(buf), "%s: %s", gui_get_string(STR_PROTOCOL),
                 gui_get_string(STR_NO_SIGNAL));
        lv_obj_set_style_local_text_color(lbl_protocol, LV_LABEL_PART_MAIN,
                                           LV_STATE_DEFAULT, LV_COLOR_GRAY);
    }
    lv_label_set_text(lbl_protocol, buf);
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
    if (g_power_state == POWER_STATE_TX) {
        lv_label_set_text(status_label, gui_get_string(STR_TRANSMIT));
        banner_color = LV_COLOR_MAKE(0xdf, 0x01, 0x01);  /* OPA1000 dark red */
    } else if (g_power_state == POWER_STATE_STANDBY) {
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

    /* Peak hold with slow release (tau = 500 ms) is produced by the Control
     * agent in adc_peak_hold_tick(), driven from the ADC task - deliberately
     * not from the DMA ISR, because the release needs expf().  The GUI is a
     * pure consumer: it never writes these two fields. */
    float fwd_peak_mW = (float)g_adc_data.fwd2_peak_mW;
    float rev_peak_mW = (float)g_adc_data.rev2_peak_mW;
/* Forward power, in Watts */
    float fwd = fwd_peak_mW / 1000.0f;
    snprintf(buf, sizeof(buf), "%.0fW", fwd);
    lv_label_set_text(val_fwd, buf);
    lv_bar_set_value(bar_fwd, (int16_t)(fwd * BAR_MAX_FWD / 1000.0f), LV_ANIM_OFF);

    /* Reflected power, in Watts */
    float refp = rev_peak_mW / 1000.0f;
    snprintf(buf, sizeof(buf), "%.0fW", refp);
    lv_label_set_text(val_ref, buf);
    lv_bar_set_value(bar_ref, (int16_t)(refp * BAR_MAX_REF / 300.0f), LV_ANIM_OFF);

    /* SWR derived from the same peak values so the three bars agree */
    float swr = calculate_swr_x100((int32_t)fwd_peak_mW,
                                   (int32_t)rev_peak_mW) / 100.0f;
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
    snprintf(buf, sizeof(buf), "%.1fV", (float)g_adc_data.vm_mV / 1000.0f);
    lv_label_set_text(lbl_voltage, buf);

    /* Current */
    snprintf(buf, sizeof(buf), "%.1fA", (float)g_adc_data.im_mA / 1000.0f);
    lv_label_set_text(lbl_current, buf);

    /* Temperature — unit follows the setting chosen in the language menu */
    float tc = (float)g_adc_data.ntc_core_tenth_c / 10.0f;
    if (temp_fahrenheit) {
        snprintf(buf, sizeof(buf), "%.0f°F", tc * 9.0f / 5.0f + 32.0f);
    } else {
        snprintf(buf, sizeof(buf), "%.0f°C", tc);
    }
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
        /* Band name under the range, e.g. "20-17m" */
        lv_label_set_text(lbl_radio_mode, strBands[g_current_band - 1]);
    } else {
        lv_label_set_text(band_label, "---");
        lv_label_set_text(lbl_radio_mode, "");
    }

    /* Radio protocol link status (from the comm protocol detector) */
    gui_update_protocol_label();

#else /* LVGL_SIM: synthesise plausible readings so the layout can be judged */
    (void)buf;

    /* Banner cycles Idle -> Standby -> Transmit so all three states and
     * their colours can be checked in the simulator. */
    static int sim_phase = 0;
    static lv_color_t sim_banner_color;
    sim_phase = (sim_phase + 1) % 30;
    if (sim_phase == 0) {
        lv_label_set_text(status_label, gui_get_string(STR_TRANSMIT));
        sim_banner_color = LV_COLOR_MAKE(0xdf, 0x01, 0x01);
    } else if (sim_phase == 10) {
        lv_label_set_text(status_label, gui_get_string(STR_STANDBY));
        sim_banner_color = LV_COLOR_MAKE(0xea, 0xb9, 0x06);
    } else if (sim_phase == 20) {
        lv_label_set_text(status_label, gui_get_string(STR_IDLE));
        sim_banner_color = COLOR_BANNER;
    }
    lv_obj_set_style_local_text_color(status_label, LV_LABEL_PART_MAIN,
                                       LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_bg_color(banner, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, sim_banner_color);
    lv_obj_set_style_local_bg_color(bottom_bar, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, sim_banner_color);

    /* Stand in for the Control agent's peak detector so the bars can be
     * judged without the target firmware.  On hardware these two fields are
     * written by user_adc.c and the GUI only reads them.  Here we feed a
     * triangular sweep straight into the peak fields. */
    static int sim_sweep = 0;
    sim_sweep = (sim_sweep + 4) % 240;
    float sim_fwd_w = (sim_sweep < 120) ? (sim_sweep * 1000.0f / 120.0f)
                                        : ((240 - sim_sweep) * 1000.0f / 120.0f);
    float sim_rev_w = sim_fwd_w * 0.06f;
    g_adc_data.fwd2_peak_mW = (int32_t)(sim_fwd_w * 1000.0f);
    g_adc_data.rev2_peak_mW = (int32_t)(sim_rev_w * 1000.0f);

    float fwd_peak_mW = (float)g_adc_data.fwd2_peak_mW;
    float rev_peak_mW = (float)g_adc_data.rev2_peak_mW;

    snprintf(buf, sizeof(buf), "%.0fW", fwd_peak_mW / 1000.0f);
    lv_label_set_text(val_fwd, buf);
    lv_bar_set_value(bar_fwd, (int16_t)(fwd_peak_mW / 1000.0f), LV_ANIM_OFF);

    snprintf(buf, sizeof(buf), "%.0fW", rev_peak_mW / 1000.0f);
    lv_label_set_text(val_ref, buf);
    lv_bar_set_value(bar_ref, (int16_t)(rev_peak_mW / 3000.0f), LV_ANIM_OFF);

    float sim_swr = calculate_swr_x100((int32_t)fwd_peak_mW,
                                       (int32_t)rev_peak_mW) / 100.0f;
    snprintf(buf, sizeof(buf), "%.1f", sim_swr);
    lv_label_set_text(val_swr, buf);
    lv_bar_set_value(bar_swr, (int16_t)((sim_swr - 1.0f) * BAR_MAX_SWR / 3.0f), LV_ANIM_OFF);

    snprintf(buf, sizeof(buf), "13.8V");
    lv_label_set_text(lbl_voltage, buf);
    snprintf(buf, sizeof(buf), "%.1fA", 2.0f + sim_fwd_w / 400.0f);
    lv_label_set_text(lbl_current, buf);

    if (temp_fahrenheit) {
        snprintf(buf, sizeof(buf), "%.0f°F", 45.0f * 9.0f / 5.0f + 32.0f);
    } else {
        snprintf(buf, sizeof(buf), "%.0f°C", 45.0f);
    }
    lv_label_set_text(lbl_temp, buf);

    lv_label_set_text(lbl_fan, "Med");
    lv_label_set_text(band_label, strFreqRanges[3]);
    lv_label_set_text(lbl_radio_mode, strBands[3]);
    gui_update_protocol_label();
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
static void gui_process_poweroff(void);

/* Cancel button callback
 *
 * The user has decided to skip calibration.  We clear
 * g_calibration_required so the next boot goes straight to the main
 * screen, but we deliberately do NOT mark g_config.touch.calibrated
 * = 1 - the operator still has an uncalibrated panel and will need to
 * open the Settings menu to redo the calibration.  This is the
 * "user explicitly chose to skip" path.
 */
static void cal_cancel_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
        calibrating = false;
        cal_state = 0;
        g_calibration_required = false;
        g_touch_calibrated     = false;
        lv_scr_load(main_screen);
    }
}

/* Accept button callback
 *
 * The linear regression in cal_build_results_ui() has already written
 * the new gain/offset/calibrated fields into g_config (see the
 * `#ifndef LVGL_SIM` block in cal_build_results_ui).  We just need
 * to (1) flag g_config as dirty so app_config_task() persists the
 * changes to flash within its 1 s window, (2) clear the boot-time
 * "calibration required" flag so the next boot goes to the main
 * screen, and (3) load main_screen.
 */
static void cal_accept_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
        calibrating = false;
        cal_state = 0;
        g_calibration_required = false;
        g_touch_calibrated     = true;
#ifndef LVGL_SIM
        config_lock();
        config_mark_dirty();
        config_unlock();
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
    lv_obj_set_pos(cancel_btn, HOR_RES / 2 - 50, VER_RES - 38);
    lv_obj_set_size(cancel_btn, 100, 30);
    lv_obj_set_style_local_text_font(cancel_btn, LV_BTN_PART_MAIN,
                                     LV_STATE_DEFAULT, &MyCJK_16);
    lv_obj_t *cancel_lbl = lv_label_create(cancel_btn, NULL);
    lv_label_set_text(cancel_lbl, gui_get_string(STR_CANCEL));
    lv_obj_set_event_cb(cancel_btn, cal_cancel_cb);

    /* Red dot at target position */
    lv_coord_t tx = cal_targets[idx].x;
    lv_coord_t ty = cal_targets[idx].y;
    lv_obj_t *dot = lv_obj_create(cal_container, NULL);
    lv_obj_set_size(dot, 34, 34);
    lv_obj_set_pos(dot, tx - 17, ty - 17);
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
    lv_obj_set_size(hl, 48, 2);
    lv_obj_set_pos(hl, tx - 24, ty - 1);
    lv_obj_set_style_local_bg_color(hl, LV_OBJ_PART_MAIN,
                                    LV_STATE_DEFAULT, LV_COLOR_RED);
    lv_obj_set_style_local_border_width(hl, LV_OBJ_PART_MAIN,
                                        LV_STATE_DEFAULT, 0);
    lv_obj_t *vl = lv_obj_create(cal_container, NULL);
    lv_obj_set_size(vl, 2, 48);
    lv_obj_set_pos(vl, tx - 1, ty - 24);
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
        /* Straight float fit: screen = raw * gain + offset */
        g_config.touch.x_gain     = x_slope;
        g_config.touch.x_offset   = x_off;
        g_config.touch.y_gain     = y_slope;
        g_config.touch.y_offset   = y_off;
        g_config.touch.calibrated = 1;
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
        lv_obj_set_pos(accept_btn, HOR_RES / 2 - 105, VER_RES - 44);
        lv_obj_set_size(accept_btn, 100, 32);
        lv_obj_set_style_local_text_font(accept_btn, LV_BTN_PART_MAIN,
                                         LV_STATE_DEFAULT, &MyCJK_16);
        lv_obj_t *accept_lbl = lv_label_create(accept_btn, NULL);
        lv_label_set_text(accept_lbl, gui_get_string(STR_CAL_ACCEPT));
        lv_obj_set_event_cb(accept_btn, cal_accept_cb);
    }

    lv_obj_t *retry_btn = lv_btn_create(cal_container, NULL);
    lv_obj_set_pos(retry_btn, HOR_RES / 2 + 5, VER_RES - 44);
    lv_obj_set_size(retry_btn, 100, 32);
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
    /* gui_init runs in the GUI task context; RTOS is already up so the
     * mutex can be created here.  No lock needed - the GUI task is the
     * only one that has a handle to this code at boot.
     *
     * The simulator needs this too: gui_lock() there is a Windows
     * CRITICAL_SECTION, and EnterCriticalSection on an uninitialised one
     * faults.  Skipping it was an outright crash, not just a no-op. */
    gui_mutex_init();

    lv_init();

#ifndef LVGL_SIM
    lv_port_disp_init();
    current_language   = g_config.language;
    temp_fahrenheit    = (g_config.temp_fahrenheit != 0U);
#else
    current_language = MULTILANG_ENGLISH;
#endif

    gui_create_main_screen();
    lv_scr_load(main_screen);
    current_screen = 0;
}

void gui_start(void)
{
    /* Intentionally empty; placeholder for future "post-init start" hooks
     * (animations, first-time focus, etc.).  No lock needed. */
}

void gui_tick(void)
{
    gui_lock();
    lv_tick_inc(5);
    lv_task_handler();
    gui_unlock();
}

void gui_update_display(void)
{
    gui_lock();
    if (!alert_active && !calibrating) {
        gui_update_main_values();
    }
    /* Refresh the top-of-screen debug overlay at most once per 500 ms.
     * lv_tick_inc(5) runs every 10 ms so without throttling the label
     * text would be rewritten too often.  The throttling also makes
     * the overlay read as a stable status line on a moving display. */
    uint32_t now = lv_tick_get();
    if ((now - s_debug_last_tick_ms) >= 500U) {
        s_debug_last_tick_ms = now;
        gui_update_debug_overlay();
    }

    /* Power-off sequence, if the operator armed it from the Settings
     * menu.  Runs here (task pump) rather than in the LVGL callback so
     * that tearing down widgets and calling power_soft_off() never
     * happens while LVGL is dispatching an event. */
    gui_process_poweroff();

    /* Backlight hand-off: the panel is dark from reset until the main
     * screen has actually been painted.  Count three update ticks (the
     * GUI task runs at 10 ms, so ~30 ms) before letting the operator
     * see anything - long enough for the first full repaint to land,
     * short enough that the boot does not feel sluggish.  After that we
     * raise g_gui_ready, which is one half of the g_ptt_ready gate. */
    static uint8_t s_first_paint_ticks = 0U;
    if (!g_gui_ready) {
        if (++s_first_paint_ticks >= 3U) {
            g_gui_ready = true;
#ifndef LVGL_SIM
            /* No backlight PWM on the PC simulator, but keep g_gui_ready
             * above unguarded so the sim exercises the same handshake. */
            boot_backlight_on();
#endif
        }
    }

    gui_unlock();
}

void gui_show_alert(AlertType alert_type)
{
    /* Called from the protect / comm / monitor tasks after a fault, NOT
     * from an ISR (the actual fault ISR only sets g_alert_flag - see the
     * concurrency note in gui.h).  Must take the lock. */
    gui_lock();
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
    lv_obj_set_size(alert_msgbox, 280, 110);
    lv_obj_align(alert_msgbox, main_screen, LV_ALIGN_CENTER, 0, 0);
    alert_active = true;
    gui_unlock();
}

void gui_dismiss_alert(void)
{
    gui_lock();
    if (alert_msgbox) {
        lv_obj_del(alert_msgbox);
        alert_msgbox = NULL;
    }
    alert_active = false;
    gui_unlock();
}

bool gui_is_alert_active(void)
{
    /* Read-only snapshot of a single flag; safe without a lock in practice
     * (torn reads of a 1-byte value are not a concern on Cortex-M7).  Other
     * tasks that need to know whether the alert is up are encouraged to
     * read g_alert_flag (in opa2000.h) instead, which is updated under the
     * protect-task side of the same state machine. */
    return alert_active;
}

const char* gui_get_string(uint16_t str_id)
{
    /* Pure array lookup into a const table - no LVGL widget access, so
     * no lock is required and ISR use is safe (the table is read-only). */
    if (str_id >= STR_COUNT) return "";
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
    gui_lock();
    current_language = lang;
#ifndef LVGL_SIM
    g_config.language = lang;
    gui_menu_mark_dirty();
#endif

    for (size_t i = 0; i < sizeof(lang_refresh) / sizeof(lang_refresh[0]); i++) {
        if (*lang_refresh[i].label) {
            lv_label_set_text(*lang_refresh[i].label,
                              gui_get_string(lang_refresh[i].str_id));
        }
    }
    gui_unlock();
}

void gui_show_calibration(void)
{
    gui_lock();
    if (cal_screen == NULL) gui_create_cal_screen();
    calibrating = true;
    cal_state = 1;
    cal_point_idx = 0;
    memset(cal_data, 0, sizeof(cal_data));
#ifndef LVGL_SIM
    touch_calibrate_start();
    /* The calibration screen is the usual FIRST thing the operator sees
     * after a factory reset, and gui_process() only raises g_gui_ready
     * after three painted ticks of the MAIN screen.  If we relied on
     * that path alone the panel would stay black while asking the user
     * to press targets - the one screen that must never be unreadable.
     * Idempotent: re-entering just re-writes the same duty cycle. */
    boot_backlight_on();
#endif
    lv_scr_load(cal_screen);
    gui_update_calibration(0);
    gui_unlock();
}

void gui_update_calibration(uint8_t current_point)
{
    gui_lock();
    (void)current_point;
    if (cal_state == 1) {
        cal_build_point_ui(cal_point_idx);
    }
    gui_unlock();
}

void gui_process_calibration(void)
{
    gui_lock();
#ifndef LVGL_SIM
    if (!calibrating || cal_state != 1) {
        gui_unlock();
        return;
    }

    /* Single entry point for whichever controller touch_init() found
     * (GT911 capacitive or NS2009 resistive).  We want the RAW panel
     * coordinates here, so we read before any calibration transform is
     * applied - that is exactly what touch_get_position() returns,
     * because touch.c only applies g_config.touch.* inside
     * touch_process() when calibrated != 0 and the calibration screen
     * keeps g_config.touch.calibrated at 0 until the user accepts. */
    TouchPointType tp;
    if (touch_get_position(&tp) && tp.pressed) {
        uint8_t idx = cal_point_idx;
        if (idx >= CAL_POINT_COUNT) {
            gui_unlock();
            return;
        }

        cal_data[idx].raw_x = tp.x;
        cal_data[idx].raw_y = tp.y;
        cal_data[idx].screen_x = cal_targets[idx].x;
        cal_data[idx].screen_y = cal_targets[idx].y;

        gui_calibration_advance();
    }
#endif
    gui_unlock();
}

void gui_power_button_pressed(void)
{
    gui_lock();
#ifndef LVGL_SIM
    /* The front-panel button is the HV switch: it engages / releases the
     * PWR_ON + RF_BIAS rail only.  The main supply latch (PG5) and the
     * relays are not touched - power_init() latched the supply at boot and
     * the relays follow the PTT, independently of this button. */
    if (g_power_state == POWER_STATE_OFF) power_hv_enable();
    else power_hv_disable();
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
    gui_unlock();
}

uint8_t gui_get_current_screen(void)
{
    /* Single-byte read; not worth the lock acquisition. */
    return current_screen;
}

static void show_language_selector(void);
static void show_screen_settings(void);
static void show_coupler_selector(void);
static void show_sensor_inspect(void);
static void show_fan_settings(void);
static void show_band_comm_settings(void);
static void show_protection_settings(void);
static void poweroff_confirm_cb(lv_obj_t *obj, lv_event_t e);

/*============================================================================
 *  Power-off flow
 *============================================================================*/

/* Stage of the power-off sequence, driven entirely from the GUI task
 * pump (gui_process_poweroff), never from an LVGL callback:
 *
 *   POWEROFF_IDLE     -> nothing to do
 *   POWEROFF_CONFIRM  -> the yes/no box is on screen
 *   POWEROFF_SAVING   -> the operator said yes, config is being written
 *   POWEROFF_DONE     -> REAR_RESERVED1 has been released, MCU is idle
 *
 * The work is deferred out of the LVGL callback because
 * power_soft_off() destroys the widget tree and never returns - doing
 * that while LVGL is walking the tree would corrupt the heap.
 */
typedef enum {
    POWEROFF_IDLE = 0,
    POWEROFF_CONFIRM,
    POWEROFF_SAVING,
    POWEROFF_DONE,
} PoweroffStage;

static PoweroffStage s_poweroff_stage = POWEROFF_IDLE;

/**
 * @brief Confirmation dialog for "Power off".
 *
 * An explicit yes/no because the action is not reversible from the
 * front panel: once REAR_RESERVED1 goes low the unit leaves the main
 * rail and the backup supply takes over.
 */
static lv_obj_t *show_poweroff_confirm(void)
{
    /* Static: lv_msgbox_add_btns() stores the pointer, so the array has
     * to outlive this function (see CLAUDE.md "LVGL String Lifetime"). */
    static const char *btns[] = {"Cancel", "Poweroff", ""};

    lv_obj_t *box = lv_msgbox_create(main_screen, NULL);
    lv_msgbox_set_text(box, gui_get_string(STR_POWEROFF_ASK));
    lv_msgbox_add_btns(box, btns);
    lv_obj_set_size(box, 300, 120);
    lv_obj_align(box, main_screen, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_event_cb(box, poweroff_confirm_cb);
    return box;
}

/* Yes / no handler for the box above. */
static void poweroff_confirm_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e != LV_EVENT_CLICKED) {
        return;
    }
    uint16_t btn = lv_msgbox_get_active_btn(obj);

    if (btn == 1U) {
        /* "Poweroff": advance to the save stage.  The box is removed by
         * the pump on the next tick, not here, so we never delete the
         * object LVGL is currently dispatching to. */
        s_poweroff_stage = POWEROFF_SAVING;
    } else {
        /* "Cancel" (btn 0) or anything else: abandon. */
        s_poweroff_stage = POWEROFF_IDLE;
        lv_obj_del(obj);
    }
}

/**
 * @brief Drive the power-off sequence from the GUI task pump.
 *
 * Sequence per the design spec: save a dirty config first, wait 500 ms
 * so the operator can see the screen react, then call power_soft_off()
 * (which releases REAR_RESERVED1 and idles for another 500 ms while the
 * backup supply takes over).
 */
static void gui_process_poweroff(void)
{
    static lv_obj_t *s_box = NULL;
#ifndef LVGL_SIM
    static uint32_t s_saved_at_ms = 0U;
#endif

    switch (s_poweroff_stage) {
    case POWEROFF_IDLE:
        return;

    case POWEROFF_CONFIRM:
        if (s_box == NULL) {
            s_box = show_poweroff_confirm();
        }
        return;

    case POWEROFF_SAVING:
        if (s_box != NULL) {
            lv_obj_del(s_box);
            s_box = NULL;
        }
#ifndef LVGL_SIM
        /* Persist before dropping the rail: after power_soft_off() the
         * MCU is gone and an unsaved edit would be lost.  Done inline
         * rather than waiting for app_config_task()'s 1 s tick so the
         * 500 ms delay below starts from a known state. */
        if (config_is_dirty()) {
            config_lock();
            config_clear_dirty();
            (void)config_save();
            config_unlock();
        }
        s_saved_at_ms = lv_tick_get();
#endif
        s_poweroff_stage = POWEROFF_DONE;
        return;

    case POWEROFF_DONE:
#ifndef LVGL_SIM
        /* Give the operator the specified 500 ms to see the screen
         * respond, then hand over to the board. */
        if ((lv_tick_get() - s_saved_at_ms) >= 500U) {
            power_soft_off();   /* never returns */
        }
#endif
        return;

    default:
        s_poweroff_stage = POWEROFF_IDLE;
        return;
    }
}

static void settings_menu_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
        uint16_t btn_id = lv_btnmatrix_get_active_btn(obj);

        /* Every button on this screen opens a sub-page, so leaving the menu
         * is the moment a batch of edits is finished.  Publish the staged
         * flag now; app_config_task() does the actual flash write a second
         * later, at the lowest priority, because programming flash stalls
         * instruction fetch and must not run inside an LVGL callback. */
        gui_menu_commit_dirty();

        /* The overlay is going away regardless of which button was pressed
         * (Cancel closes it, sub-page entries replace it).  Drop the cached
         * pointer now so a re-entrant gui_toggle_menu() call (e.g. user
         * spamming the Settings button) does not see a dangling handle. */
        if (settings_menu_overlay != NULL) {
            settings_menu_overlay = NULL;
        }

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
        if (btn_id == 7) {
            /* Power off.  The menu stays open behind the confirmation
             * box; the pump removes it together with the box. */
            lv_obj_del_async(lv_obj_get_parent(obj));
            s_poweroff_stage = POWEROFF_CONFIRM;
            return;
        }
        /* btn_id == 8 (Cancel) or anything unexpected: just close. */
        lv_obj_del_async(lv_obj_get_parent(obj));
    }
}

/*============================================================================
 * Language Selector Popup
 *============================================================================*/

/* Language + temperature unit.
 * Row 1: interface language.  Row 2: temperature unit (C / F).
 * Static so the pointers lv_btnmatrix_set_map() stores stay valid. */
static const char *lang_btns[] = {"English", "中文", "\n",
                                  "\u00B0C", "\u00B0F",
                                  "\n", NULL, ""};

static void language_sel_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
        uint16_t id = lv_msgbox_get_active_btn(obj);
        if (id == 0) {
            gui_set_language(MULTILANG_ENGLISH);
        } else if (id == 1) {
            gui_set_language(MULTILANG_CHINESE);
        } else if (id == 2) {
            temp_fahrenheit = false;
#ifndef LVGL_SIM
            g_config.temp_fahrenheit = 0U;
            gui_menu_mark_dirty();
#endif
        } else if (id == 3) {
            temp_fahrenheit = true;
#ifndef LVGL_SIM
            g_config.temp_fahrenheit = 1U;
            gui_menu_mark_dirty();
#endif
        }
        lv_obj_del_async(obj);
    }
}

static void show_language_selector(void)
{
    /* Row 1: English / 中文     Row 2: °C / °F     Row 3: Cancel.
     * The Cancel label is language dependent, so it is filled in before the
     * map is handed to LVGL (index 6, not 3 — index 3 is the °F button). */
    lang_btns[6] = gui_get_string(STR_CANCEL);

    lv_obj_t *scr = lv_scr_act();
    lv_obj_t *mbox = lv_msgbox_create(scr, NULL);
    lv_msgbox_set_text(mbox, gui_get_string(STR_LANGUAGE));
    lv_msgbox_add_btns(mbox, lang_btns);
    lv_obj_set_size(mbox, 300, 165);
    lv_obj_align(mbox, scr, LV_ALIGN_CENTER, 0, 0);

    lv_obj_set_style_local_text_color(mbox, LV_MSGBOX_PART_BG,
                                       LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(mbox, LV_MSGBOX_PART_BG,
                                      LV_STATE_DEFAULT, &MyCJK_12);
    lv_obj_set_style_local_bg_color(mbox, LV_MSGBOX_PART_BG,
                                     LV_STATE_DEFAULT, LV_COLOR_BLUE);

    lv_obj_t *btnm = lv_msgbox_get_btnmatrix(mbox);
    lv_obj_set_style_local_text_color(btnm, LV_BTNMATRIX_PART_BTN,
                                      LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(btnm, LV_BTNMATRIX_PART_BTN,
                                      LV_STATE_DEFAULT, &MyCJK_12);
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
    lv_obj_set_size(mbox, 300, 110);
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
    lv_obj_set_size(mbox, 300, 130);
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
static void cc_refresh_ui(uint8_t step);
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
static lv_obj_t *cc_result_container = NULL;
static lv_obj_t *cc_adc_val_label = NULL;
static lv_obj_t *cc_raw_val_label = NULL;
static lv_obj_t *cc_step_label = NULL;
static lv_obj_t *cc_instr_label = NULL;
static lv_obj_t *cc_ch_label = NULL;
static lv_obj_t *cc_title_label = NULL;
static uint16_t cc_meas[CC_STEPS];
static char cc_buf[48];
/* The step UI must not be rebuilt from inside the button's own event
 * callback: lv_obj_del() would free the container that LVGL is still
 * walking.  The callback only records the measurement and flags the
 * rebuild, which gui_process_coupler_cal() performs from the main loop. */
static bool cc_rebuild_pending = false;

static const char *cc_coupler_name(void)
{
    return (cc_coupler_id == 1) ? gui_get_string(STR_DIR_COUPLER_1)
                                : gui_get_string(STR_DIR_COUPLER_2);
}

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
    /* Steps 0/1 = forward, 2/3 = reflected; coupler 1 = PA→LPF, 2 = LPF→ANT */
    uint8_t rank;
    if (cc_coupler_id == 1)
        rank = (cc_step < 2) ? ADC1_RANK_FWD1 : ADC1_RANK_REV1;
    else
        rank = (cc_step < 2) ? ADC1_RANK_FWD2 : ADC1_RANK_REV2;
    return g_adc_data.adc1_raw[rank];
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
        cc_rebuild_pending = false;
        lv_scr_load(main_screen);
    }
}

/* Accept button callback */
static void cc_accept_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
#ifndef LVGL_SIM
        gui_menu_mark_dirty();
#endif
        cc_active = false;
        cc_state = 0;
        cc_rebuild_pending = false;
        lv_scr_load(main_screen);
    }
}

/* Retry button callback */
static void cc_retry_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
        if (cc_result_container) {
            lv_obj_del(cc_result_container);
            cc_result_container = NULL;
        }
        if (cc_container) lv_obj_set_hidden(cc_container, false);
        cc_step = 0;
        cc_state = 1;
        memset(cc_meas, 0, sizeof(cc_meas));
        cc_rebuild_pending = true;
    }
}

/* Start measurement button */
static void cc_start_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_PRESSED && cc_active && cc_state == 1) {
        cc_meas[cc_step] = cc_read_adc();
        cc_step++;
        cc_rebuild_pending = true;
    }
}

/* Build the step UI.
 *
 * The container is created once and kept alive; only three labels change from
 * step to step.  Deleting and recreating the tree on every step both churns the
 * object list and is unsafe when triggered from an event callback (LVGL is
 * still walking the deleted object), so cc_refresh_ui() updates the text in
 * place and the results page is shown by hiding this container. */
static void cc_build_ui(uint8_t step)
{
    if (cc_container) {          /* already built — just refresh the text */
        cc_refresh_ui(step);
        return;
    }

    cc_container = lv_obj_create(cc_screen, NULL);
    lv_obj_set_size(cc_container, HOR_RES, VER_RES);
    lv_obj_set_pos(cc_container, 0, 0);
    lv_obj_set_style_local_bg_opa(cc_container, LV_OBJ_PART_MAIN,
                                   LV_STATE_DEFAULT, LV_OPA_TRANSP);
    lv_obj_set_style_local_border_width(cc_container, LV_OBJ_PART_MAIN,
                                        LV_STATE_DEFAULT, 0);

    /* Title: "Coupler X Calibration" — depends on cc_coupler_id, so refreshed */
    cc_title_label = lv_label_create(cc_container, NULL);
    lv_obj_align(cc_title_label, cc_container, LV_ALIGN_IN_TOP_MID, 0, 16);
    lv_obj_set_style_local_text_color(cc_title_label, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, LV_COLOR_BLACK);
    lv_obj_set_style_local_text_font(cc_title_label, LV_LABEL_PART_MAIN,
                                     LV_STATE_DEFAULT, &MyCJK_16);

    /* Step + direction — text is refreshed by cc_refresh_ui() */
    cc_step_label = lv_label_create(cc_container, NULL);
    lv_obj_align(cc_step_label, cc_container, LV_ALIGN_IN_TOP_MID, 0, 46);
    lv_obj_set_style_local_text_color(cc_step_label, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, LV_COLOR_BLACK);
    lv_obj_set_style_local_text_font(cc_step_label, LV_LABEL_PART_MAIN,
                                     LV_STATE_DEFAULT, &lv_font_montserrat_24);

    /* Instruction */
    cc_instr_label = lv_label_create(cc_container, NULL);
    lv_obj_align(cc_instr_label, cc_container, LV_ALIGN_IN_TOP_MID, 0, 84);
    lv_obj_set_style_local_text_color(cc_instr_label, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, LV_COLOR_BLACK);
    lv_obj_set_style_local_text_font(cc_instr_label, LV_LABEL_PART_MAIN,
                                     LV_STATE_DEFAULT, &MyCJK_16);

    /* ADC real-time value box */
    lv_obj_t *adc_box = lv_obj_create(cc_container, NULL);
    lv_obj_set_size(adc_box, HOR_RES - 80, 100);
    lv_obj_set_pos(adc_box, HOR_RES / 20, VER_RES / 4 + 10);
    lv_obj_set_style_local_bg_color(adc_box, LV_OBJ_PART_MAIN,
                                     LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_border_color(adc_box, LV_OBJ_PART_MAIN,
                                         LV_STATE_DEFAULT, COLOR_BORDER);
    lv_obj_set_style_local_radius(adc_box, LV_OBJ_PART_MAIN,
                                   LV_STATE_DEFAULT, 4);

    /* Channel name */
    cc_ch_label = lv_label_create(adc_box, NULL);
    lv_obj_align(cc_ch_label, adc_box, LV_ALIGN_IN_TOP_LEFT, 10, 8);
    lv_obj_set_style_local_text_color(cc_ch_label, LV_LABEL_PART_MAIN,
                                       LV_STATE_DEFAULT, LV_COLOR_BLACK);
    lv_obj_set_style_local_text_font(cc_ch_label, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &MyCJK_16);

    /* ADC raw value */
    cc_raw_val_label = lv_label_create(adc_box, NULL);
    lv_label_set_text(cc_raw_val_label, "ADC: ---");
    lv_obj_align(cc_raw_val_label, adc_box, LV_ALIGN_IN_LEFT_MID, 10, 0);
    lv_obj_set_style_local_text_color(cc_raw_val_label, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, LV_COLOR_BLACK);
    lv_obj_set_style_local_text_font(cc_raw_val_label, LV_LABEL_PART_MAIN,
                                     LV_STATE_DEFAULT, &lv_font_montserrat_24);

    /* Calculated power */
    cc_adc_val_label = lv_label_create(adc_box, NULL);
    lv_label_set_text(cc_adc_val_label, "--- W");
    lv_obj_align(cc_adc_val_label, adc_box, LV_ALIGN_IN_RIGHT_MID, -10, 0);
    lv_obj_set_style_local_text_color(cc_adc_val_label, LV_LABEL_PART_MAIN,
                                       LV_STATE_DEFAULT, LV_COLOR_BLACK);
    lv_obj_set_style_local_text_font(cc_adc_val_label, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &lv_font_montserrat_24);

    /* Start button */
    lv_obj_t *start_btn = lv_btn_create(cc_container, NULL);
    lv_obj_set_pos(start_btn, HOR_RES / 2 - 105, VER_RES - 44);
    lv_obj_set_size(start_btn, 100, 32);
    lv_obj_set_style_local_text_font(start_btn, LV_BTN_PART_MAIN,
                                     LV_STATE_DEFAULT, &MyCJK_16);
    lv_obj_t *start_lbl = lv_label_create(start_btn, NULL);
    lv_label_set_text(start_lbl, gui_get_string(STR_START_MEAS));
    lv_obj_set_event_cb(start_btn, cc_start_cb);

    /* Cancel button */
    lv_obj_t *cancel_btn = lv_btn_create(cc_container, NULL);
    lv_obj_set_pos(cancel_btn, HOR_RES / 2 + 5, VER_RES - 44);
    lv_obj_set_size(cancel_btn, 120, 40);
    lv_obj_set_style_local_text_font(cancel_btn, LV_BTN_PART_MAIN,
                                     LV_STATE_DEFAULT, &MyCJK_16);
    lv_obj_t *cancel_lbl = lv_label_create(cancel_btn, NULL);
    lv_label_set_text(cancel_lbl, gui_get_string(STR_CANCEL));
    lv_obj_set_event_cb(cancel_btn, cc_cancel_cb);

    cc_refresh_ui(step);
}

/* Update only the labels whose text depends on the step number. */
static void cc_refresh_ui(uint8_t step)
{
    char buf[64];
    char step_str[32];

    snprintf(buf, sizeof(buf), "%s %s",
             cc_coupler_name(), gui_get_string(STR_CALIBRATION));
    lv_label_set_text(cc_title_label, buf);

    cc_step_desc(step, step_str, sizeof(step_str));
    snprintf(buf, sizeof(buf), "%d/%d  %s", step + 1, CC_STEPS, step_str);
    lv_label_set_text(cc_step_label, buf);

    snprintf(buf, sizeof(buf), "%s %.0fW AM  |  %s",
             gui_get_string(STR_CAL_POINT),
             (step == 0 || step == 2) ? 5.0f : 20.0f,
             gui_get_string(STR_START_MEAS));
    lv_label_set_text(cc_instr_label, buf);

    snprintf(buf, sizeof(buf), "%s: %s",
             (step < 2) ? "TX Power" : "REF Power",
             (cc_coupler_id == 1) ? "CH1" : "CH2");
    lv_label_set_text(cc_ch_label, buf);
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

/* Results screen — the step container is hidden rather than deleted, and the
 * results container is built once and reused when Retry is pressed. */
static void cc_build_results(void)
{
    if (cc_container) lv_obj_set_hidden(cc_container, true);

    /* Calculate */
    float fwd_slope, fwd_off, ref_slope, ref_off;
    cc_calculate(&fwd_slope, &fwd_off, &ref_slope, &ref_off);

#ifndef LVGL_SIM
    /* cc_calculate() returns Watts per raw code.  Store the slope as
     * micro-Watt per ADC code so it stays an exact integer, and the
     * intercept in milli-Watt. */
    volatile coupler_t *fwd_c = (cc_coupler_id == 1) ? &g_config.fwd1 : &g_config.fwd2;
    volatile coupler_t *ref_c = (cc_coupler_id == 1) ? &g_config.rev1 : &g_config.rev2;
    fwd_c->gain_uW_per_code = (int32_t)(fwd_slope * 1000000.0f);  /* W -> uW */
    fwd_c->offset_mW        = (int32_t)(fwd_off * 1000.0f);
    ref_c->gain_uW_per_code = (int32_t)(ref_slope * 1000000.0f);
    ref_c->offset_mW        = (int32_t)(ref_off * 1000.0f);
#endif

    if (cc_result_container) lv_obj_del(cc_result_container);

    cc_result_container = lv_obj_create(cc_screen, NULL);
    lv_obj_t *ro = cc_result_container;
    lv_obj_set_size(ro, HOR_RES, VER_RES);
    lv_obj_set_pos(ro, 0, 0);
    lv_obj_set_style_local_bg_opa(ro, LV_OBJ_PART_MAIN,
                                   LV_STATE_DEFAULT, LV_OPA_TRANSP);
    lv_obj_set_style_local_border_width(ro, LV_OBJ_PART_MAIN,
                                        LV_STATE_DEFAULT, 0);

    char buf[200];
    snprintf(buf, sizeof(buf), "%s %s",
             cc_coupler_name(), gui_get_string(STR_CAL_SUCCESS));
    lv_obj_t *result_lbl = lv_label_create(ro, NULL);
    lv_label_set_text(result_lbl, buf);
    lv_obj_align(result_lbl, ro, LV_ALIGN_IN_TOP_MID, 0, 20);
    lv_obj_set_style_local_text_color(result_lbl, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, LV_COLOR_GREEN);
    lv_obj_set_style_local_text_font(result_lbl, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &MyCJK_16);

    /* Parameters — shown in the same units that are stored in ConfigType:
     * gain in micro-Watt per ADC code, offset in milli-Watt. */
    lv_obj_t *params_lbl = lv_label_create(ro, NULL);
    snprintf(buf, sizeof(buf),
        "FWD: %ld uW/code   %ld mW\n"
        "REF: %ld uW/code   %ld mW",
        (long)(fwd_slope * 1000000.0f), (long)(fwd_off * 1000.0f),
        (long)(ref_slope * 1000000.0f), (long)(ref_off * 1000.0f));
    lv_label_set_text(params_lbl, buf);
    lv_obj_align(params_lbl, ro, LV_ALIGN_CENTER, 0, -20);
    lv_obj_set_style_local_text_color(params_lbl, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, LV_COLOR_BLACK);
    lv_obj_set_style_local_text_font(params_lbl, LV_LABEL_PART_MAIN,
                                     LV_STATE_DEFAULT, &MyCJK_16);

    /* Accept */
    lv_obj_t *accept_btn = lv_btn_create(ro, NULL);
    lv_obj_set_pos(accept_btn, HOR_RES / 2 - 105, VER_RES - 44);
    lv_obj_set_size(accept_btn, 100, 32);
    lv_obj_set_style_local_text_font(accept_btn, LV_BTN_PART_MAIN,
                                     LV_STATE_DEFAULT, &MyCJK_16);
    lv_obj_t *accept_lbl = lv_label_create(accept_btn, NULL);
    lv_label_set_text(accept_lbl, gui_get_string(STR_CAL_ACCEPT));
    lv_obj_set_event_cb(accept_btn, cc_accept_cb);

    /* Retry */
    lv_obj_t *retry_btn = lv_btn_create(ro, NULL);
    lv_obj_set_pos(retry_btn, HOR_RES / 2 + 5, VER_RES - 44);
    lv_obj_set_size(retry_btn, 100, 32);
    lv_obj_set_style_local_text_font(retry_btn, LV_BTN_PART_MAIN,
                                     LV_STATE_DEFAULT, &MyCJK_16);
    lv_obj_t *retry_lbl = lv_label_create(retry_btn, NULL);
    lv_label_set_text(retry_lbl, gui_get_string(STR_CAL_RETRY));
    lv_obj_set_event_cb(retry_btn, cc_retry_cb);
}

/* Update real-time ADC display — called from main loop */
void gui_process_coupler_cal(void)
{
    gui_lock();
    if (!cc_active) { gui_unlock(); return; }

    if (cc_rebuild_pending) {
        cc_rebuild_pending = false;
        if (cc_step >= CC_STEPS) {
            cc_state = 2;
            cc_build_results();
        } else {
            cc_state = 1;
            cc_build_ui(cc_step);
        }
        gui_unlock();
        return;
    }

    if (cc_state != 1 || cc_adc_val_label == NULL) { gui_unlock(); return; }

    uint16_t adc = cc_read_adc();

    snprintf(cc_buf, sizeof(cc_buf), "ADC: %u", adc);
    lv_label_set_text(cc_raw_val_label, cc_buf);

#ifndef LVGL_SIM
    /* Apply the stored coupler fit:
     *   power_mW = raw * gain_uW_per_code / 1000 + offset_mW */
    const volatile coupler_t *cal;
    if (cc_step < 2)
        cal = (cc_coupler_id == 1) ? &g_config.fwd1 : &g_config.fwd2;
    else
        cal = (cc_coupler_id == 1) ? &g_config.rev1 : &g_config.rev2;
    int32_t power_mW = (int32_t)((int64_t)adc * cal->gain_uW_per_code / 1000)
                     + cal->offset_mW;
    snprintf(cc_buf, sizeof(cc_buf), "%.2f W", (float)power_mW / 1000.0f);
#else
    /* Simulator: show the nominal power of the current step so the layout
     * can be verified without a calibrated coupler in the config. */
    snprintf(cc_buf, sizeof(cc_buf), "%.0f W",
             (cc_step == 0 || cc_step == 2) ? 5.0f : 20.0f);
#endif
    lv_label_set_text(cc_adc_val_label, cc_buf);
    gui_unlock();
}

/* Public: show coupler calibration screen */
void gui_show_coupler_cal(uint8_t coupler_id)
{
    gui_lock();
    if (cc_screen == NULL) {
        cc_screen = lv_obj_create(NULL, NULL);
        lv_obj_set_size(cc_screen, HOR_RES, VER_RES);
        lv_obj_set_style_local_bg_color(cc_screen, LV_OBJ_PART_MAIN,
                                         LV_STATE_DEFAULT, LV_COLOR_WHITE);
    }

    if (cc_result_container) {
        lv_obj_del(cc_result_container);
        cc_result_container = NULL;
    }
    if (cc_container) lv_obj_set_hidden(cc_container, false);

    cc_active = true;
    cc_coupler_id = coupler_id;
    cc_step = 0;
    cc_state = 1;
    cc_rebuild_pending = false;
    memset(cc_meas, 0, sizeof(cc_meas));
    lv_scr_load(cc_screen);
    cc_build_ui(0);
    gui_unlock();
}

/*============================================================================
 * Sensor Inspect Overlay — dual-panel table with ADC channel info
 *============================================================================*/

static lv_obj_t *si_overlay = NULL;

/* Sensor-inspect rows follow the ADC1 rank layout from opa2000.h so the
 * table stays in sync with user_adc.c without a second enum to maintain. */
#define SI_ROW_COUNT  ADC1_RANK_COUNT
static lv_obj_t *si_raw_labels[SI_ROW_COUNT];
static lv_obj_t *si_val_labels[SI_ROW_COUNT];

static void si_close_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
        lv_obj_del_async(si_overlay);
        si_overlay = NULL;
    }
}

/* Build the sensor-inspect table — all text centered.
 * Rows are indexed by ADC1_RANK_* (see opa2000.h). */
static void si_build_table(lv_obj_t *parent, lv_coord_t x, int count)
{
    lv_coord_t hdr_y = 26;
    lv_coord_t row_h = 20;
    /* Column widths for centering */
    const lv_coord_t cw_name = 84, cw_adc = 46, cw_raw = 62, cw_val = 78;
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
    SI_LABEL(parent, "ADC1", x + 135, 4, 270, LV_COLOR_YELLOW, &MyCJK_12);

    /* Header row */
    static const char *hdr[] = {"Name", "ADC", "Raw", "Value"};
    lv_coord_t hcxs[] = {name_cx, adc_cx, raw_cx, val_cx};
    lv_coord_t hws[]  = {cw_name, cw_adc, cw_raw, cw_val};
    for (int c = 0; c < 4; c++)
        SI_LABEL(parent, hdr[c], hcxs[c], hdr_y, hws[c], LV_COLOR_GRAY, &MyCJK_12);

    /* Short channel names, ordered by ADC1_RANK_* */
    static const char *ch_short[SI_ROW_COUNT] = {
        "RF In",    /* ADC1_RANK_RF_IN    */
        "Current",  /* ADC1_RANK_IM       */
        "REF Pwr1", /* ADC1_RANK_REV1     */
        "TX Pwr1",  /* ADC1_RANK_FWD1     */
        "REF Pwr2", /* ADC1_RANK_REV2     */
        "VBIAS",    /* ADC1_RANK_VBIAS    */
        "TX Pwr2",  /* ADC1_RANK_FWD2     */
        "Voltage",  /* ADC1_RANK_VM       */
        "Key ADC",  /* ADC1_RANK_KEY_ADC  */
    };

    /* ADC1 positive input number per rank, per OPA2000_H7 固件设计要求.txt */
    static const uint8_t si_adc1_inp[SI_ROW_COUNT] = {4, 5, 6, 8, 9, 14, 15, 16, 17};

    for (int i = 0; i < count; i++) {
        lv_coord_t y = hdr_y + 16 + i * row_h;

        char adc_txt[8];
        snprintf(adc_txt, sizeof(adc_txt), "INP%d", si_adc1_inp[i]);

        SI_LABEL(parent, ch_short[i],  name_cx, y, cw_name, LV_COLOR_WHITE, &lv_font_montserrat_12);
        SI_LABEL(parent, adc_txt,      adc_cx,  y, cw_adc,  LV_COLOR_WHITE, &lv_font_montserrat_12);

        si_raw_labels[i] = lv_label_create(parent, NULL);
        lv_label_set_text(si_raw_labels[i], "---");
        lv_obj_set_width(si_raw_labels[i], cw_raw);
        lv_obj_set_pos(si_raw_labels[i], raw_cx - cw_raw / 2, y);
        lv_label_set_align(si_raw_labels[i], LV_LABEL_ALIGN_CENTER);
        lv_obj_set_style_local_text_color(si_raw_labels[i], LV_LABEL_PART_MAIN,
                                           LV_STATE_DEFAULT, LV_COLOR_WHITE);
        lv_obj_set_style_local_text_font(si_raw_labels[i], LV_LABEL_PART_MAIN,
                                          LV_STATE_DEFAULT, &lv_font_montserrat_16);

        si_val_labels[i] = lv_label_create(parent, NULL);
        lv_label_set_text(si_val_labels[i], "---");
        lv_obj_set_width(si_val_labels[i], cw_val);
        lv_obj_set_pos(si_val_labels[i], val_cx - cw_val / 2, y);
        lv_label_set_align(si_val_labels[i], LV_LABEL_ALIGN_CENTER);
        lv_obj_set_style_local_text_color(si_val_labels[i], LV_LABEL_PART_MAIN,
                                           LV_STATE_DEFAULT, LV_COLOR_YELLOW);
        lv_obj_set_style_local_text_font(si_val_labels[i], LV_LABEL_PART_MAIN,
                                          LV_STATE_DEFAULT, &lv_font_montserrat_16);
    }
#undef SI_LABEL
}

static void show_sensor_inspect(void)
{
    if (si_overlay) { lv_obj_del(si_overlay); si_overlay = NULL; }

    lv_coord_t o_w = HOR_RES - 2 * MARGIN_X;
    lv_coord_t o_h = VER_RES - 2 * MARGIN_X;
    si_overlay = lv_obj_create(lv_scr_act(), NULL);
    lv_obj_set_size(si_overlay, o_w, o_h);
    lv_obj_align(si_overlay, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_local_bg_color(si_overlay, LV_OBJ_PART_MAIN,
                                     LV_STATE_DEFAULT, COLOR_BG);
    lv_obj_set_style_local_radius(si_overlay, LV_OBJ_PART_MAIN,
                                   LV_STATE_DEFAULT, 0);

    lv_obj_t *title = lv_label_create(si_overlay, NULL);
    lv_label_set_text(title, gui_get_string(STR_SENSOR_INSPECT));
    lv_obj_align(title, si_overlay, LV_ALIGN_IN_TOP_MID, 0, 2);
    lv_obj_set_style_local_text_color(title, LV_LABEL_PART_MAIN,
                                       LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(title, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &MyCJK_12);

    /* Single ADC1 table — rows follow ADC1_RANK_* order */
    si_build_table(si_overlay, 6, SI_ROW_COUNT);

    lv_obj_t *close_btn = lv_btn_create(si_overlay, NULL);
    lv_obj_set_size(close_btn, 90, 26);
    lv_obj_align(close_btn, si_overlay, LV_ALIGN_IN_BOTTOM_MID, 0, -3);
    lv_obj_t *cl = lv_label_create(close_btn, NULL);
    lv_label_set_text(cl, gui_get_string(STR_CANCEL));
    lv_obj_set_style_local_text_font(cl, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &MyCJK_12);
    lv_obj_set_event_cb(close_btn, si_close_cb);
}

/* Update sensor inspect values — called from main loop.
 * Engineering values are integers in SI-ish units (mA, mV, mW, 0.1 C)
 * per opa2000.h; scale them down here for display only. */
void gui_update_sensor_inspect(void)
{
    gui_lock();
    if (!si_overlay) { gui_unlock(); return; }
    char buf[48];
    for (unsigned i = 0; i < SI_ROW_COUNT; i++) {
#ifndef LVGL_SIM
        uint16_t raw = g_adc_data.adc1_raw[i];
        snprintf(buf, sizeof(buf), "%u", raw);
        if (si_raw_labels[i]) lv_label_set_text(si_raw_labels[i], buf);

        switch (i) {
            case ADC1_RANK_RF_IN:
                snprintf(buf, sizeof(buf), "%.2fW", (float)g_adc_data.rf_in_mW / 1000.0f);
                break;
            case ADC1_RANK_IM:
                snprintf(buf, sizeof(buf), "%.2fA", (float)g_adc_data.im_mA / 1000.0f);
                break;
            case ADC1_RANK_REV1:
                snprintf(buf, sizeof(buf), "%.1fW", (float)g_adc_data.rev1_mW / 1000.0f);
                break;
            case ADC1_RANK_FWD1:
                snprintf(buf, sizeof(buf), "%.1fW", (float)g_adc_data.fwd1_mW / 1000.0f);
                break;
            case ADC1_RANK_REV2:
                snprintf(buf, sizeof(buf), "%.1fW", (float)g_adc_data.rev2_mW / 1000.0f);
                break;
            case ADC1_RANK_VBIAS:
                snprintf(buf, sizeof(buf), "%.0fmV", (double)g_adc_data.vbias_mV);
                break;
            case ADC1_RANK_FWD2:
                snprintf(buf, sizeof(buf), "%.1fW", (float)g_adc_data.fwd2_mW / 1000.0f);
                break;
            case ADC1_RANK_VM:
                snprintf(buf, sizeof(buf), "%.2fV", (float)g_adc_data.vm_mV / 1000.0f);
                break;
            default:
                snprintf(buf, sizeof(buf), "code");
                break;
        }
#else
        /* Simulator: synthesise plausible raw codes */
        static const uint16_t sim_raw[SI_ROW_COUNT] = {
            1200, 43300, 800, 21000, 640, 5100, 20800, 36400, 8000
        };
        snprintf(buf, sizeof(buf), "%u", sim_raw[i]);
        if (si_raw_labels[i]) lv_label_set_text(si_raw_labels[i], buf);

        static const char *sim_val[SI_ROW_COUNT] = {
            "3.50W", "0.15A", "12.0W", "210.0W", "9.5W",
            "195mV", "205.0W", "13.80V", "code"
        };
        snprintf(buf, sizeof(buf), "%s", sim_val[i]);
#endif
        if (si_val_labels[i]) lv_label_set_text(si_val_labels[i], buf);
    }
    gui_unlock();
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
    lv_obj_set_size(mbox, 300, 150);
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
            gui_menu_mark_dirty();
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
    lv_obj_set_size(mbox, 300, 130);
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

/* Number of rows on the protection screen.  Declared before the arrays
 * below so their size is a compile-time constant rather than a count
 * that has to be repeated by hand. */
#define PROT_COUNT 10

static lv_obj_t *prot_sliders[PROT_COUNT];
static lv_obj_t *prot_val_labels[PROT_COUNT];

/* Editable protection thresholds, held as plain floats for readability.
 * prot_from_config() / prot_to_config() translate to the integer units
 * actually stored in ConfigType (SWR is x100, temperature is 0.1 C). */
static float prot_val[PROT_COUNT];
static float prot_val_backup[PROT_COUNT];

/* CC6920BSO-50A main-current sensor, used only to SHOW what the COMP2
 * reference voltage means in amps.  The sensor law is fixed by the
 * datasheet (V = 1650 mV + I * 26.4), so this is a display convenience
 * and not a calibration: the stored and applied quantity stays the
 * reference voltage in mV. */
#define IM_SENSOR_ZERO_MV     1650.0f
#define IM_SENSOR_MV_PER_A    26.4f

typedef struct {
    const char *name;
    float vmin, vmax;      /* slider range in display units */
    const char *unit;
    int decimals;
    /* When non-zero, the row also shows the engineering equivalent
     * derived from the stored millivolts.  Only COMP2 has a published
     * transfer function; COMP1 is uncalibrated (TODO-COMP1标定待办.txt). */
    int derived_from_mv;
} ProtField;

static const ProtField prot_fields[PROT_COUNT] = {
    {"REV2",     100,  600, "W", 0, 0},
    {"REV2+SWR",  50,  400, "W", 0, 0},
    {"SWR",         1,    5, "",  1, 0},
    {"REV1",     100, 1000, "W", 0, 0},
    {"Main I",    10,   30, "A", 0, 0},
    {"Temp",      60,  120, "C", 0, 0},
    {"FWD1",     500, 1500, "W", 0, 0},
    {"FWD2",     500, 1500, "W", 0, 0},
    /* COMP1: reference voltage only.  No wattage, because the D5/R40
     * peak detector in front of PB2 has not been calibrated yet. */
    {"C1 ref",   200, (float)COMP1_REF_MV_MAX, "mV", 0, 0},
    /* COMP2: reference voltage, with the derived amp rating alongside. */
    {"C2 ref",  1700, (float)COMP2_REF_MV_MAX, "mV", 0, 1},
};

/* Load the thresholds from g_config (float display units). */
static void prot_from_config(void)
{
#ifndef LVGL_SIM
    prot_val[0] = (float)g_config.band_rev2_max_w;
    prot_val[1] = (float)g_config.band_rev2_swr_w;
    prot_val[2] = (float)g_config.band_swr_max_x100     / 100.0f;
    prot_val[3] = (float)g_config.band_rev1_max_w;
    prot_val[4] = (float)g_config.band_im_max_mA / 1000.0f;   /* mA -> A */
    prot_val[5] = (float)g_config.band_ntc_max_tenth_c / 10.0f;
    prot_val[6] = (float)g_config.band_fwd1_max_w;
    prot_val[7] = (float)g_config.band_fwd2_max_w;
    prot_val[8] = (float)g_config.comp1_ref_mV;
    prot_val[9] = (float)g_config.comp2_ref_mV;
#endif
}

/* Write the thresholds back into g_config. */
static void prot_to_config(void)
{
#ifndef LVGL_SIM
    g_config.band_rev2_max_w      = (uint16_t)prot_val[0];
    g_config.band_rev2_swr_w       = (uint16_t)prot_val[1];
    g_config.band_swr_max_x100    = (uint16_t)(prot_val[2] * 100.0f);
    g_config.band_rev1_max_w      = (uint16_t)prot_val[3];
    g_config.band_im_max_mA       = (uint16_t)(prot_val[4] * 1000.0f);  /* A -> mA */
    g_config.band_ntc_max_tenth_c = (uint16_t)(prot_val[5] * 10.0f);
    g_config.band_fwd1_max_w      = (uint16_t)prot_val[6];
    g_config.band_fwd2_max_w      = (uint16_t)prot_val[7];
    g_config.comp1_ref_mV         = (uint16_t)prot_val[8];
    g_config.comp2_ref_mV         = (uint16_t)prot_val[9];
#endif
}

/* Format a threshold value for display.  Celsius gets its Fahrenheit
 * twin; the COMP2 row additionally shows what the reference voltage works
 * out to in amps, which is the number the operator actually reasons
 * about ("30 A") even though the stored quantity is millivolts. */
static void prot_format(const ProtField *pf, float val, char *buf, size_t len)
{
    if (pf->unit[0] == 'C') {
        float f = val * 9.0f / 5.0f + 32.0f;
        snprintf(buf, len, "%.0f%s / %.0fF", val, pf->unit, f);
    } else if (pf->derived_from_mv) {
        float amps = (val - IM_SENSOR_ZERO_MV) / IM_SENSOR_MV_PER_A;
        snprintf(buf, len, "%.0fmV/%.1fA", val, amps);
    } else {
        snprintf(buf, len, "%.*f%s", pf->decimals, val, pf->unit);
    }
}

static void prot_slider_cb(lv_obj_t *slider, lv_event_t e)
{
    if (e == LV_EVENT_VALUE_CHANGED) {
        int idx = -1;
        for (int i = 0; i < (int)PROT_COUNT; i++) {
            if (prot_sliders[i] == slider) { idx = i; break; }
        }
        if (idx < 0 || !prot_val_labels[idx]) return;

        /* lv_slider works in integers, so use a 0.1x scale internally
         * (SWR 1.0..5.0 -> 10..50) and keep everything else in float. */
        prot_val[idx] = (float)lv_slider_get_value(slider) / 10.0f;
        char buf[48];
        prot_format(&prot_fields[idx], prot_val[idx], buf, sizeof(buf));
        lv_label_set_text(prot_val_labels[idx], buf);
    }
}

static void prot_cancel_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
        memcpy(prot_val, prot_val_backup, sizeof(prot_val));
        prot_screen = NULL; lv_scr_load(main_screen);
    }
}

static void prot_save_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
        prot_to_config();
#ifndef LVGL_SIM
        /* Push the whole protection set into the hardware now rather than
         * at the next reboot, otherwise the operator would leave the menu
         * believing a new threshold is in force while the old value is
         * still sitting in the peripheral.
         *
         * Both families are covered, and they are covered by different
         * mechanisms - which is why this has to be explicit:
         *   - AWD1/2/3 hold their limits in ADC registers.  Nothing reads
         *     g_config at compare time, so Main I and REV1 would silently
         *     keep the old hardware gate.
         *   - The software limits (REV2 / SWR / Temp / FWD1 / FWD2 /
         *     VBIAS) need no call at all: protection.c re-reads g_config
         *     on every evaluation.
         *   - COMP1/COMP2 come along inside the same call.
         * AWD1 is indexed by band, so the current band has to be passed
         * through rather than hard-coded. */
        boot_apply_thresholds((int)g_current_band);
        gui_menu_mark_dirty();
#endif
        prot_screen = NULL; lv_scr_load(main_screen);
    }
}

/* One threshold row: name (left) + value (right) above a full-width slider.
 * Ten of these are laid out in two columns to fit the 480x272 screen. */
static void prot_slider_create(lv_obj_t *parent, int idx, lv_coord_t x,
                               lv_coord_t y, lv_coord_t w)
{
    const ProtField *pf = &prot_fields[idx];
    char buf[64];

    lv_obj_t *name_lbl = lv_label_create(parent, NULL);
    snprintf(buf, sizeof(buf), "%s", pf->name);
    lv_label_set_text(name_lbl, buf);
    lv_obj_set_pos(name_lbl, x, y);
    lv_obj_set_style_local_text_color(name_lbl, LV_LABEL_PART_MAIN,
                                       LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(name_lbl, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &MyCJK_12);

    lv_obj_t *val_lbl = lv_label_create(parent, NULL);
    float init_val = prot_val[idx];
    prot_format(pf, init_val, buf, sizeof(buf));
    lv_label_set_text(val_lbl, buf);
    /* Park the value in the right half of the cell.  Explicit positioning is
     * used rather than right-alignment because a bare label ignores
     * LV_LABEL_ALIGN_RIGHT until it has a fixed width applied first. */
    lv_obj_set_pos(val_lbl, x + w * 3 / 5, y);
    lv_obj_set_style_local_text_color(val_lbl, LV_LABEL_PART_MAIN,
                                       LV_STATE_DEFAULT, LV_COLOR_YELLOW);
    lv_obj_set_style_local_text_font(val_lbl, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &MyCJK_12);

    lv_obj_t *slider = lv_slider_create(parent, NULL);
    lv_obj_set_size(slider, w, 14);
    lv_obj_set_pos(slider, x, y + 17);
    /* Slider uses a 0.1x integer scale so SWR 1.0..5.0 fits lv_slider's
     * integer range with usable resolution. */
    lv_slider_set_range(slider, (int16_t)(pf->vmin * 10.0f),
                               (int16_t)(pf->vmax * 10.0f));
    lv_slider_set_value(slider, (int16_t)(init_val * 10.0f), LV_ANIM_OFF);

    prot_sliders[idx] = slider;
    prot_val_labels[idx] = val_lbl;
    lv_obj_set_event_cb(slider, prot_slider_cb);
}

static void show_protection_settings(void)
{
    if (prot_screen) { lv_obj_del(prot_screen); prot_screen = NULL; }

    prot_from_config();
    memcpy(prot_val_backup, prot_val, sizeof(prot_val));

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
    lv_label_set_text(title, gui_get_string(STR_SETTINGS));
    lv_obj_align(title, prot_container, LV_ALIGN_IN_TOP_MID, 0, 2);
    lv_obj_set_style_local_text_color(title, LV_LABEL_PART_MAIN,
                                       LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(title, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &MyCJK_12);

    /* Two columns x five rows - ten thresholds in 480x272.  The last
     * row is the two COMP comparator references. */
    const int per_col = PROT_COUNT / 2;
    const lv_coord_t col_w = (HOR_RES - 3 * MARGIN_X) / 2;
    const lv_coord_t col_x0 = MARGIN_X;
    const lv_coord_t col_x1 = MARGIN_X * 2 + col_w;
    const lv_coord_t row_y0 = 20;
    const lv_coord_t row_pitch = 40;
    for (int i = 0; i < (int)PROT_COUNT; i++) {
        int col = i / per_col;
        int row = i % per_col;
        prot_slider_create(prot_container, i,
                           col ? col_x1 : col_x0,
                           row_y0 + row * row_pitch,
                           col_w);
    }

    lv_obj_t *save_btn = lv_btn_create(prot_container, NULL);
    lv_obj_set_pos(save_btn, HOR_RES / 2 - 105, VER_RES - 40);
    lv_obj_set_size(save_btn, 100, 32);
    lv_obj_t *save_lbl = lv_label_create(save_btn, NULL);
    lv_label_set_text(save_lbl, gui_get_string(STR_SAVE));
    lv_obj_set_style_local_text_font(save_lbl, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &MyCJK_12);
    lv_obj_set_event_cb(save_btn, prot_save_cb);

    lv_obj_t *cancel_btn = lv_btn_create(prot_container, NULL);
    lv_obj_set_pos(cancel_btn, HOR_RES / 2 + 5, VER_RES - 40);
    lv_obj_set_size(cancel_btn, 100, 32);
    lv_obj_t *cancel_lbl = lv_label_create(cancel_btn, NULL);
    lv_label_set_text(cancel_lbl, gui_get_string(STR_CANCEL));
    lv_obj_set_style_local_text_font(cancel_lbl, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &MyCJK_12);
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
    "关机 Poweroff", "\n",
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
    "Poweroff 关机", "\n",
    "Cancel", ""
};

void gui_toggle_menu(void)
{
    gui_lock();
    /* Toggle semantics: if the menu is already open, close it instead of
     * stacking another overlay (which would trap the user with multiple
     * stacked layers and is the bug the GUI agent just fixed). */
    if (settings_menu_overlay != NULL) {
        lv_obj_del(settings_menu_overlay);
        settings_menu_overlay = NULL;
        gui_unlock();
        return;
    }

    const char **btns = (gui_get_lang_index() == 0) ? settings_zh : settings_en;

    lv_obj_t *overlay = lv_obj_create(lv_scr_act(), NULL);
    lv_obj_set_size(overlay, HOR_RES - 4 * MARGIN_X, VER_RES - 4 * MARGIN_X);
    lv_obj_align(overlay, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_local_bg_color(overlay, LV_OBJ_PART_MAIN,
                                     LV_STATE_DEFAULT, COLOR_BG);
    lv_obj_set_style_local_radius(overlay, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, 0);

    lv_obj_t *btnm = lv_btnmatrix_create(overlay, NULL);
    lv_btnmatrix_set_map(btnm, btns);
    lv_obj_set_size(btnm, HOR_RES - 4 * MARGIN_X - 8, VER_RES - 4 * MARGIN_X - 8);
    lv_obj_align(btnm, NULL, LV_ALIGN_CENTER, 0, 0);

    /* Button matrix background */
    lv_obj_set_style_local_bg_color(btnm, LV_BTNMATRIX_PART_BG,
                                     LV_STATE_DEFAULT, COLOR_BG);
    lv_obj_set_style_local_pad_inner(btnm, LV_BTNMATRIX_PART_BG,
                                      LV_STATE_DEFAULT, 6);

    /* Button style (matching OPA1000 and band selector) */
    lv_obj_set_style_local_text_color(btnm, LV_BTNMATRIX_PART_BTN,
                                      LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(btnm, LV_BTNMATRIX_PART_BTN,
                                      LV_STATE_DEFAULT, &MyCJK_12);
    lv_obj_set_style_local_bg_color(btnm, LV_BTNMATRIX_PART_BTN,
                                     LV_STATE_DEFAULT, LV_COLOR_MAKE(0, 160, 225));
    lv_obj_set_style_local_border_color(btnm, LV_BTNMATRIX_PART_BTN,
                                         LV_STATE_DEFAULT, LV_COLOR_GRAY);
    lv_obj_set_style_local_radius(btnm, LV_BTNMATRIX_PART_BTN,
                                   LV_STATE_DEFAULT, 0);

    lv_obj_set_event_cb(btnm, settings_menu_cb);
    settings_menu_overlay = overlay;
    gui_unlock();
}

void gui_toggle_band_popup(void) { /* TODO: not implemented; lock when used */ }
void gui_focus_prev(void)         { /* TODO: not implemented; lock when used */ }
void gui_focus_next(void)         { /* TODO: not implemented; lock when used */ }
void gui_enter_pressed(void)      { /* TODO: not implemented; lock when used */ }
