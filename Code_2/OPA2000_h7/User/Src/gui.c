/**
 * @file gui.c
 * @brief LVGL GUI Implementation — OPA1000-style layout
 */

#include "gui.h"
#include "lv_port_disp.h"
#include <math.h>
/* protection.h is needed OUTSIDE the LVGL_SIM guard: alert_dismiss_cb()
 * calls protection_clear() so that dismissing the popup actually releases
 * the fault latch.  That header only declares prototypes plus the
 * protection_get_fault_string() helper - no HAL or hardware dependency -
 * so it is safe in the simulator build too. */
#include "protection.h"
#ifndef LVGL_SIM
#include "comm.h"
#include "config.h"
#include "radio_band.h"
#include "touch.h"
#include "power_control.h"
#include "boot.h"
#include "user_adc.h"
#include "fan_tach.h"
#endif
#include "string.h"
#include "stdio.h"

/*============================================================================
 * Integer-only number formatting
 *
 * printf's %f pulls in newlib's float printf support, which costs several KB
 * of code and drags in soft-float helpers we otherwise never call.  Every
 * value on these screens comes from integer milliwatts / milliamps / tenths of
 * a degree, so format them as integers with a decimal point inserted by hand.
 *==========================================================================*/

/** @brief Format scaled_int / 10^decimals with a fixed number of decimals.
 *  @param scaled_int  value already multiplied by 10^decimals
 *  @param decimals    0..3; how many digits follow the point
 *  @param buf         destination
 *  @param len         destination size
 */
static void gui_fmt_fixed(char *buf, size_t len, int32_t scaled_int,
                          uint8_t decimals)
{
    uint32_t mag;
    bool negative = (scaled_int < 0);

    if (negative) {
        scaled_int = -scaled_int;
    }
    mag = (uint32_t)scaled_int;

    if (decimals == 0U) {
        snprintf(buf, len, negative ? "-%lu" : "%lu",
                 (unsigned long)mag);
    } else if (decimals == 1U) {
        snprintf(buf, len, negative ? "-%lu.%lu" : "%lu.%lu",
                 (unsigned long)(mag / 10U), (unsigned long)(mag % 10U));
    } else if (decimals == 2U) {
        snprintf(buf, len, negative ? "-%lu.%02lu" : "%lu.%02lu",
                 (unsigned long)(mag / 100U), (unsigned long)(mag % 100U));
    } else {
        snprintf(buf, len, negative ? "-%lu.%03lu" : "%lu.%03lu",
                 (unsigned long)(mag / 1000U), (unsigned long)(mag % 1000U));
    }
}

/*============================================================================
 *  Change-guarded label writes
 *
 *  LVGL 7 的 lv_label_set_text() 第一行就是 lv_obj_invalidate()，无条件：
 *
 *      void lv_label_set_text(lv_obj_t * label, const char * text)
 *      {
 *          LV_ASSERT_OBJ(label, LV_OBJX_NAME);
 *          lv_obj_invalidate(label);        <-- 文字没变也照样标脏
 *
 *  而 gui_update_main_values() 每轮要写 20 多个 label（功率、温度、电压、
 *  频段……）。无条件标脏的后果是每 30 ms 把整个主界面重画一次，而
 *  LV_USE_GPU_STM32_DMA2D = 0 —— 全部走 CPU 软件光栅化，中文字形还要逐
 *  像素混合。LVGL 的 perf monitor 实测就是 91% CPU。
 *
 *  真正的浪费在于：这些值绝大多数时候根本没变。功率还是那个功率，温度还是
 *  那个温度，但 LVGL 无从知道。所以这里在写入前先比一次，相同就整条跳过，
 *  连 invalidate 都不产生。
 *
 *  用法：每个 label 配一个缓存槽，写入时把 label 指针一并传进来（指针
 *  同时充当"这个槽属于哪个 label"的标识，也让重复注册不可能出错）。
 *==========================================================================*/

/*!< 缓存槽里保存的上次写入文本。32 B 够装下本文件所有格式化结果
 *  （最长的是协议行 "Protocol: KENWOOD"，约 18 字节）。 */
typedef struct {
    lv_obj_t *label;              /*!< 归属的 label；NULL = 空槽 */
    char      text[32];
} GuiLabelCache;

#define GUI_LABEL_CACHE_SLOTS \
    (sizeof(s_lblcache) / sizeof(s_lblcache[0]))

static GuiLabelCache s_lblcache[24];

/**
 * @brief Only call lv_label_set_text() when the text actually changed
 *
 * @param label  Target label
 * @param text   New text (must stay valid for the call, LVGL copies it)
 */
static void gui_label_set_cached(lv_obj_t *label, const char *text)
{
    /* 首个空槽；等于槽数表示没有空槽。 */
    size_t free_slot = GUI_LABEL_CACHE_SLOTS;

    if (label == NULL || text == NULL) {
        return;
    }

    for (size_t i = 0U; i < GUI_LABEL_CACHE_SLOTS; i++) {
        if (s_lblcache[i].label == label) {
            if (strcmp(s_lblcache[i].text, text) == 0) {
                return;             /* 无变化：连 invalidate 都不产生 */
            }
            lv_label_set_text(label, text);
            strncpy(s_lblcache[i].text, text, sizeof(s_lblcache[i].text) - 1U);
            s_lblcache[i].text[sizeof(s_lblcache[i].text) - 1U] = '\0';
            return;
        }
        if (s_lblcache[i].label == NULL && free_slot == GUI_LABEL_CACHE_SLOTS) {
            free_slot = i;
        }
    }

    lv_label_set_text(label, text);

    if (free_slot < GUI_LABEL_CACHE_SLOTS) {
        s_lblcache[free_slot].label = label;
        strncpy(s_lblcache[free_slot].text, text,
                sizeof(s_lblcache[free_slot].text) - 1U);
        s_lblcache[free_slot].text[sizeof(s_lblcache[free_slot].text) - 1U] = '\0';
    }
    /* else: 槽位耗尽。写入照常发生（正确性优先），只是不再缓存，
     * 下一轮起会重新走上面的查找分支。 */
}

/**
 * @brief Same as gui_label_set_cached(), for bar values
 *
 * lv_bar_set_value() already early-returns when the value is unchanged
 * (lv_bar.c: `if(ext->cur_value == value) return;`), so bars need no cache
 * of their own.  This wrapper exists only so the intent is obvious at the
 * call sites: the write is already change-guarded by LVGL itself.
 */
static void gui_bar_set_cached(lv_obj_t *bar, int16_t value)
{
    lv_bar_set_value(bar, value, LV_ANIM_OFF);
}

/**
 * @brief Drop all cached label text, so the next write goes through
 *
 * Needed whenever a label's text is changed by something other than
 * gui_label_set_cached() — a language switch rewrites every string, and
 * without this the cache would report "unchanged" and the new language
 * would never appear.
 */
static void gui_label_cache_invalidate(void)
{
    for (size_t i = 0U; i < GUI_LABEL_CACHE_SLOTS; i++) {
        s_lblcache[i].label = NULL;
        s_lblcache[i].text[0] = '\0';
    }
}

/*============================================================================
 * Private Variables
 *==========================================================================*/

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
    /*!< Shown after a failed touch calibration pass.  The screen restarts
     *  from the first target on its own, so this line exists purely to stop
     *  the operator reading the transition as a hang.  Touch calibration no
     *  longer has Accept/Retry buttons - those two IDs are still used by the
     *  coupler calibration result page. */
    STR_CAL_RESTART,
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
    /* --- alerts that protection.c can raise but had no string ---
     * ALERT_NTC_OVERHEAT / ALERT_VBIAS_FAULT were previously unmapped and
     * fell through to the generic "警报" / "Alert". Appended at the end so
     * the existing indices (and STR_COUNT) stay put. */
    STR_NTC_OVERHEAT,   /* 核心温度过高 */
    STR_VBIAS_FAULT,    /* PA 偏置异常 */
    STR_FAN_FAULT,      /* 风扇异常 (占位，未接线) */
};

/* Number of entries in gui_strings[][].  STR_COUNT must stay last - it
 * anchors to the enum's final value, so adding a string above it is
 * enough to grow the table with no second edit here. */
#define STR_COUNT (STR_FAN_FAULT + 1)

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
        "原始坐标", "残差", "确认", "重试", "重新开始标定",
        "屏幕", "触屏校准",
        "校准传感器", "耦合器1", "耦合器2", "开始测量",
        "传感器检查", "自动", "保存",
        "发射", "反射", "协议", "未连接", "已连接", "波段", "温度单位", "摄氏度 °C", "华氏度 °F",
        "关机", "确认关机？", "保存中…",
        "核心温度过高", "PA 偏置异常",
        "风扇异常",
    },
    /* English */
    {
        "Idle", "Standby", "Transmit",
        "TX Power", "Reflected", "SWR", "Voltage", "Current", "Temperature", "Band",
        "Alert", "SWR High", "Reflected High", "Current High", "Hardware Fault", "Overdrive", "Freq Mismatch",
        "Dismiss", "Cancel", "Calibration", "Calibrate Point", "Calibrate Success", "Calibrate Failed", "Settings", "Language", "Power",
        "Fan",
        "Raw Coordinate", "Residual", "Accept", "Retry", "Restarting calibration",
        "Screen", "Touch Calibrate",
        "Calibrate Sensors", "Coupler 1", "Coupler 2", "Start",
        "Sensor Inspect", "Auto", "Save",
        "FWD", "REV", "Protocol", "No link", "Linked", "Band", "Temp Unit", "Celsius \u00B0C", "Fahrenheit \u00B0F",
        "Poweroff", "Power off now?", "Saving...",
        "Core Overheat", "PA Bias Fault",
        "Fan Fault",
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

/*!< 防连点：记下标定点后锁这么久（毫秒）。按住不放时采样每 10 ms 就来一个
 *  新样本，不锁的话 5 个点会在 50 ms 内记满。减法写法对 tick 回绕安全。 */
#define CAL_LOCKOUT_MS 500U

/*!< 一轮 5 点标定失败后，在"残差/系数"画面上停留多久再自动回到第 1 点。
 *  留这一段是因为那几个数字是唯一能看出"为什么失败"的线索（通常是某个点
 *  敲偏了），一闪而过等于没显示。3 s 够读完五个数，又不至于让"想重试"
 *  的人觉得程序卡住。 */
#define CAL_RESTART_DELAY_MS 1000U

static uint8_t cal_state = 0;        /* 0=idle, 1=showing point, 2=results */
static uint8_t cal_point_idx = 0;
static lv_obj_t *cal_container = NULL;
static uint32_t s_cal_lockout_tick = 0U;   /*!< 上次记点的时刻 */
static uint32_t s_cal_restart_tick = 0U;  /*!< 失败画面开始展示的时刻 */

/* Top-of-screen debug line.  On firmware build this shows the detected
 * touch driver + key boot-time flags; on the PC simulator it shows the
 * touch type the SIM was built with.  The line is tiny (Montserrat 12
 * px) and sits above the status banner so it does not eat any main-
 * screen real estate.  Controlled by gui.c internal -DDEBUG_OVERLAY
 * in CMake, default ON. */
static lv_obj_t *debug_label;

/*============================================================================
 *  Touch raw readout - bottom-left corner, red
 *
 *  Shows the last sample exactly as the driver published it: panel X, panel
 *  Y, and the NS2009 contact pressure.  No calibration, no transform - this
 *  is the number you want when the calibrated position is wrong and you
 *  need to know what the panel itself said.
 *
 *  Compile-time switch: -DDEBUG_TOUCH_RAW=0 (the default) compiles the
 *  whole feature out, including the label and the per-click formatting, so
 *  a release build carries none of it.  The #if is written as
 *  `#if DEBUG_TOUCH_RAW` so an -DNDEBUG-only build (which does NOT define
 *  this macro) also compiles it out - that is deliberate, this is a
 *  bring-up aid, not a diagnostic that should survive into the field.
 *
 *  Placement: bottom-left, aligned inside the screen by 4 px so it never
 *  overlaps the Settings button on the right or the status banner on top.
 *==========================================================================*/
#ifndef DEBUG_TOUCH_RAW
#define DEBUG_TOUCH_RAW 0
#endif

#if DEBUG_TOUCH_RAW

/*!< The readout label.  Created in gui_create_main_screen() as a child of
 *  main_screen (not of a sub-panel) so it floats over whatever is on
 *  screen - useful when the thing being debugged *is* a popup. */
static lv_obj_t *lbl_touch_raw;

/*!< New sample published by the driver, waiting for the GUI task pump to
 *  pick it up.  Set from gui_indev_read_cb(), which runs INSIDE
 *  lv_task_handler() - LVGL is mid-refresh there and touching a widget
 *  from that context is unsafe.  The pump in gui_update_display() does the
 *  actual label write, in task context, exactly like every other widget
 *  update in this file.
 *
 *  Written by the GUI task and read by the GUI task (read_cb is called by
 *  lv_task_handler, which the same task runs), so no volatile and no lock
 *  are needed here either. */
static bool     s_touch_raw_dirty;
static uint16_t s_touch_raw_x;
static uint16_t s_touch_raw_y;
static uint32_t s_touch_raw_z;

#endif /* DEBUG_TOUCH_RAW */


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
    /* bg_opa must be COVER or the block is invisible.  This project installs
     * no theme (no lv_theme_*_init() call, and this LVGL 7.0's
     * lv_disp_drv_t has no theme field at all), so every object falls back to
     * lv_obj_get_style_prop()'s hardcoded default for an unset
     * LV_STYLE_BG_OPA, which is LV_OPA_TRANSP.  Setting bg_color alone then
     * paints a fully transparent rectangle. */
    lv_obj_set_style_local_bg_opa(c, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_OPA_COVER);
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
    /* No theme in this project, so an unset bg_opa falls back to LV_OPA_TRANSP
     * (see make_cont).  Without this the whole screen is see-through and the
     * framebuffer's initial contents show instead. */
    lv_obj_set_style_local_bg_opa(main_screen, LV_OBJ_PART_MAIN,
                                  LV_STATE_DEFAULT, LV_OPA_COVER);

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
    val_fwd = make_label(c_fwd, "0W", 0, 1, LV_COLOR_WHITE, &lv_font_montserrat_16);
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
    val_ref = make_label(c_ref, "0W", 0, 1, LV_COLOR_WHITE, &lv_font_montserrat_16);
    lv_obj_align(val_ref, NULL, LV_ALIGN_IN_TOP_RIGHT, -cell_pad, 0);
    bar_ref = make_bar(c_ref, cell_pad, 22, half_w - 2 * cell_pad - 2, 18, BAR_MAX_REF);

    lbl_swr_title = make_label(c_swr, gui_get_string(STR_SWR),
                               cell_pad, 2, LV_COLOR_WHITE, &MyCJK_16);
    val_swr = make_label(c_swr, "1.0", 0, 1, LV_COLOR_WHITE, &lv_font_montserrat_16);
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

#if DEBUG_TOUCH_RAW
    /* Touch raw readout: bottom-left, red, one line.
     *
     * Child of main_screen rather than of any section, so it stays put and
     * on top no matter which screen is loaded - including the popups, which
     * is exactly when you want to see what the panel actually reported.
     * Created hidden: nothing shows until the first sample arrives, so an
     * idle panel is not carrying a "0 0 0" that reads like a fault. */
    lbl_touch_raw = lv_label_create(main_screen, NULL);
    lv_obj_set_style_local_text_color(lbl_touch_raw, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, LV_COLOR_RED);
    lv_obj_set_style_local_text_font(lbl_touch_raw, LV_LABEL_PART_MAIN,
                                     LV_STATE_DEFAULT, &lv_font_montserrat_12);
    lv_obj_align(lbl_touch_raw, main_screen, LV_ALIGN_IN_BOTTOM_LEFT, 4, -2);
    lv_obj_set_hidden(lbl_touch_raw, true);
#endif
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

    /* The colour is set before the text so it is already correct on the
     * very first paint.  set_style_local_text_color() invalidates too, but
     * the protocol rarely changes, so the write itself is cheap; the text
     * write is the one that must be guarded. */
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
    gui_label_set_cached(lbl_protocol, buf);
}

/**
 * @brief Update all live data on the main screen
 */
static void gui_update_main_values(void)
{
    char buf[32];

#ifndef LVGL_SIM
    /* Status.  Every write below goes through gui_label_set_cached() /
     * gui_bar_set_cached(), both of which skip the LVGL call when the value
     * is unchanged.  That matters because lv_label_set_text() invalidates
     * unconditionally (lv_label.c, first statement), and with 20+ labels on
     * this screen an unguarded refresh marks the whole screen dirty on every
     * single loop - measured at 91% CPU with software rasterisation. */
    lv_color_t banner_color;
    if (g_power_state == POWER_STATE_TX) {
        gui_label_set_cached(status_label, gui_get_string(STR_TRANSMIT));
        banner_color = LV_COLOR_MAKE(0xdf, 0x01, 0x01);  /* OPA1000 dark red */
    } else if (g_power_state == POWER_STATE_STANDBY) {
        gui_label_set_cached(status_label, gui_get_string(STR_STANDBY));
        banner_color = LV_COLOR_MAKE(0xea, 0xb9, 0x06);  /* OPA1000 dark yellow */
    } else {
        gui_label_set_cached(status_label, gui_get_string(STR_IDLE));
        banner_color = COLOR_BANNER;  /* blue */
    }
    lv_obj_set_style_local_bg_color(banner, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, banner_color);
    lv_obj_set_style_local_bg_color(bottom_bar, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, banner_color);
    /* Text always white on colored banner */
    lv_obj_set_style_local_text_color(status_label, LV_LABEL_PART_MAIN,
                                       LV_STATE_DEFAULT, LV_COLOR_WHITE);

    /* 上电稳定等待期：零点校准还没做完之前，ADC 读数是没有意义的
     * （模拟量还在爬升，读出来会乱跳）。这一段时间里主界面一律显示
     * "--"，而不是把一屏噪声当成真实功率给操作员看。校准一完成
     * （adc_stabilisation_tick() 走到 ARMED）下面照常显示。
     * 这里只替换测量值，状态条/频段/协议等照旧刷新，操作员看得出
     * 机器活着、只是还没进入可发射状态。 */
    if (!adc_stabilisation_done()) {
        gui_label_set_cached(val_fwd, "--W");
        gui_label_set_cached(val_ref, "--W");
        gui_label_set_cached(val_swr, "--");
        gui_bar_set_cached(bar_fwd, 0);
        gui_bar_set_cached(bar_ref, 0);
        gui_bar_set_cached(bar_swr, 0);
        lv_obj_set_style_local_text_color(val_swr, LV_LABEL_PART_MAIN,
                                           LV_STATE_DEFAULT, LV_COLOR_WHITE);
        gui_label_set_cached(lbl_voltage, "--V");
        gui_label_set_cached(lbl_current, "--A");
        gui_label_set_cached(lbl_temp, "--\xC2\xB0" "C");

        /* The fan is already spinning here (power_init() commands 50 %),
         * so its TACH reading is meaningful even while the ADC readings
         * are not - leave it running rather than blanking it. */
        gui_fmt_fixed(buf, sizeof(buf), (int32_t)fan_tach_get_fan_rpm(), 0U);
        strncat(buf, "RPM", sizeof(buf) - strlen(buf) - 1U);
        gui_label_set_cached(lbl_fan, buf);
        gui_label_set_cached(lbl_temp_cap, "--");

        /* Band / Frequency range — indexed from BandType (BAND_160M=1, etc.) */
        if (g_current_band > BAND_NONE && g_current_band <= BAND_15M_10M) {
            gui_label_set_cached(band_label, strFreqRanges[g_current_band - 1]);
            gui_label_set_cached(lbl_radio_mode, strBands[g_current_band - 1]);
        } else {
            gui_label_set_cached(band_label, "---");
            gui_label_set_cached(lbl_radio_mode, "");
        }
        gui_update_protocol_label();
        return;
    }

    /* Peak hold with slow release (tau = 500 ms) is produced by the Control
     * agent in adc_peak_hold_tick(), driven from the ADC task - deliberately
     * not from the DMA ISR, because the release needs expf().  The GUI is a
     * pure consumer: it never writes these two fields. */
    int32_t fwd_peak_mW = (int32_t)g_adc_data.fwd2_peak_mW;
    int32_t rev_peak_mW = (int32_t)g_adc_data.rev2_peak_mW;
/* Forward power, in Watts - round to nearest before dropping the fraction */
    int32_t fwd_w = (fwd_peak_mW + 500) / 1000;
    gui_fmt_fixed(buf, sizeof(buf), fwd_w, 0U);
    strncat(buf, "W", sizeof(buf) - strlen(buf) - 1U);
    gui_label_set_cached(val_fwd, buf);
    gui_bar_set_cached(bar_fwd, (int16_t)((fwd_peak_mW * BAR_MAX_FWD) / (1000 * 1000)));

    /* Reflected power, in Watts */
    int32_t ref_w = (rev_peak_mW + 500) / 1000;
    gui_fmt_fixed(buf, sizeof(buf), ref_w, 0U);
    strncat(buf, "W", sizeof(buf) - strlen(buf) - 1U);
    gui_label_set_cached(val_ref, buf);
    gui_bar_set_cached(bar_ref, (int16_t)((rev_peak_mW * BAR_MAX_REF) / (300 * 1000)));

    /* SWR derived from the same peak values so the three bars agree.
     * calculate_swr_x100 returns hundredths; one decimal means /10. */
    int32_t swr_x100 = calculate_swr_x100(fwd_peak_mW, rev_peak_mW);
    gui_fmt_fixed(buf, sizeof(buf), swr_x100 / 10, 1U);
    gui_label_set_cached(val_swr, buf);
    gui_bar_set_cached(bar_swr, (int16_t)(((swr_x100 - 100) * BAR_MAX_SWR) / 300));
    if (swr_x100 > 200) {
        lv_obj_set_style_local_text_color(val_swr, LV_LABEL_PART_MAIN,
                                           LV_STATE_DEFAULT, LV_COLOR_RED);
    } else {
        lv_obj_set_style_local_text_color(val_swr, LV_LABEL_PART_MAIN,
                                           LV_STATE_DEFAULT, LV_COLOR_WHITE);
    }

    /* Voltage - vm_mV is millivolts, whole volts means /1000 */
    gui_fmt_fixed(buf, sizeof(buf), ((int32_t)g_adc_data.vm_mV + 500) / 1000, 0U);
    strncat(buf, "V", sizeof(buf) - strlen(buf) - 1U);
    gui_label_set_cached(lbl_voltage, buf);

    /* Current - one decimal amp means /100 of the milliamp reading */
    gui_fmt_fixed(buf, sizeof(buf), (int32_t)g_adc_data.im_mA, 2U);
    strncat(buf, "A", sizeof(buf) - strlen(buf) - 1U);
    gui_label_set_cached(lbl_current, buf);

    /* Temperature — unit follows the setting chosen in the language menu.
     * ntc_core_tenth_c is tenths of a degree Celsius; whole degrees. */
    int32_t tc_tenth = (int32_t)g_adc_data.ntc_core_tenth_c;
    if (temp_fahrenheit) {
        /* F = C * 9 / 5 + 32, kept in tenths so this stays integer. */
        int32_t f_tenth = (tc_tenth * 9L) / 5L + 320L;
        gui_fmt_fixed(buf, sizeof(buf), (f_tenth + 5) / 10, 0U);
        strncat(buf, "\xC2\xB0" "F", sizeof(buf) - strlen(buf) - 1U);
    } else {
        gui_fmt_fixed(buf, sizeof(buf), (tc_tenth + 5) / 10, 0U);
        strncat(buf, "\xC2\xB0" "C", sizeof(buf) - strlen(buf) - 1U);
    }
    gui_label_set_cached(lbl_temp, buf);

    /* Fan speed - measured RPM from the TACH pin, not the commanded
     * duty.  The two are not the same thing (a stalled or unplugged
     * fan still reports whatever duty we asked for), and the whole
     * point of counting pulses is to catch that difference.  Spelling out
     * "RPM" rather than a bare "R": the segment is 96 px wide, so there
     * is room, and an unexplained "R" next to "13.8V" reads as a typo. */
    gui_fmt_fixed(buf, sizeof(buf), (int32_t)fan_tach_get_fan_rpm(), 0U);
    strncat(buf, "RPM", sizeof(buf) - strlen(buf) - 1U);
    gui_label_set_cached(lbl_fan, buf);

    /* Band / Frequency range — indexed from BandType (BAND_160M=1, etc.) */
    if (g_current_band > BAND_NONE && g_current_band <= BAND_15M_10M) {
        gui_label_set_cached(band_label, strFreqRanges[g_current_band - 1]);
        /* Band name under the range, e.g. "20-17m" */
        gui_label_set_cached(lbl_radio_mode, strBands[g_current_band - 1]);
    } else {
        gui_label_set_cached(band_label, "---");
        gui_label_set_cached(lbl_radio_mode, "");
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
    gui_label_set_cached(val_fwd, buf);
    gui_bar_set_cached(bar_fwd, (int16_t)(fwd_peak_mW / 1000.0f));

    snprintf(buf, sizeof(buf), "%.0fW", rev_peak_mW / 1000.0f);
    gui_label_set_cached(val_ref, buf);
    gui_bar_set_cached(bar_ref, (int16_t)(rev_peak_mW / 3000.0f));

    float sim_swr = calculate_swr_x100((int32_t)fwd_peak_mW,
                                       (int32_t)rev_peak_mW) / 100.0f;
    snprintf(buf, sizeof(buf), "%.1f", sim_swr);
    gui_label_set_cached(val_swr, buf);
    gui_bar_set_cached(bar_swr, (int16_t)((sim_swr - 1.0f) * BAR_MAX_SWR / 3.0f));

    snprintf(buf, sizeof(buf), "13.8V");
    gui_label_set_cached(lbl_voltage, buf);
    snprintf(buf, sizeof(buf), "%.1fA", 2.0f + sim_fwd_w / 400.0f);
    gui_label_set_cached(lbl_current, buf);

    if (temp_fahrenheit) {
        snprintf(buf, sizeof(buf), "%.0f°F", 45.0f * 9.0f / 5.0f + 32.0f);
    } else {
        snprintf(buf, sizeof(buf), "%.0f°C", 45.0f);
    }
    gui_label_set_cached(lbl_temp, buf);

    gui_label_set_cached(lbl_fan, "2450RPM");
    gui_label_set_cached(band_label, strFreqRanges[3]);
    gui_label_set_cached(lbl_radio_mode, strBands[3]);
    gui_update_protocol_label();
#endif
}

/*============================================================================
 * Calibration Screen — Step-by-step guided calibration
 *============================================================================*/

/* Forward declarations */
static void cal_cancel_cb(lv_obj_t *obj, lv_event_t e);
static void cal_del_container(void);
static void cal_finish(void);
static void cal_restart_from_first_point(void);
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

/* Leave the calibration screen for good, with the panel calibrated.
 *
 * Split out of what used to be cal_accept_cb() so cal_build_results_ui()
 * can call it the moment the fit passes, without the round trip through a
 * button the operator no longer has to press. */
static void cal_finish(void)
{
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

/* cal_accept_cb() and cal_retry_cb() used to live here, backing the
 * Accept / Retry buttons on the result screen.  Both buttons are gone: a
 * passing fit now commits and leaves on the spot (cal_finish, called from
 * cal_build_results_ui), and a failing one restarts from point 0 by itself
 * (cal_restart_from_first_point, driven by gui_process_cal_restart).  There
 * is deliberately no remaining tap target that can leave the calibration
 * half-finished - g_config.touch.calibrated would stay 0 and the next boot
 * would ask the same five questions again. */

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

    /* Target marker: a blue cross.
     *
     * bg_opa has to be set explicitly on every one of these.  LVGL 7's
     * fallback for an unset LV_STYLE_BG_OPA is LV_OPA_TRANSP (lv_obj.c
     * lv_obj_get_style_prop()), and no theme is installed in this project
     * (lv_theme_*_init() is never called and disp_drv.theme stays NULL),
     * so there is nothing to raise that opacity.  Setting only bg_color on a
     * plain lv_obj_create() therefore draws a fully transparent rectangle -
     * the marker is invisible, and the colour choice is irrelevant.
     *
     * Three overlapping bars rather than one graphic: the two 48 px arms
     * plus the 14 px centre block read as a single cross, and each bar is a
     * plain lv_obj so nothing depends on line-draw support.
     */
    lv_coord_t tx = cal_targets[idx].x;
    lv_coord_t ty = cal_targets[idx].y;

    /* Horizontal arm */
    lv_obj_t *hl = lv_obj_create(cal_container, NULL);
    lv_obj_set_size(hl, 48, 4);
    lv_obj_set_pos(hl, tx - 24, ty - 2);
    lv_obj_set_style_local_bg_color(hl, LV_OBJ_PART_MAIN,
                                    LV_STATE_DEFAULT, LV_COLOR_BLUE);
    lv_obj_set_style_local_bg_opa(hl, LV_OBJ_PART_MAIN,
                                  LV_STATE_DEFAULT, LV_OPA_COVER);
    lv_obj_set_style_local_border_width(hl, LV_OBJ_PART_MAIN,
                                        LV_STATE_DEFAULT, 0);

    /* Vertical arm */
    lv_obj_t *vl = lv_obj_create(cal_container, NULL);
    lv_obj_set_size(vl, 4, 48);
    lv_obj_set_pos(vl, tx - 2, ty - 24);
    lv_obj_set_style_local_bg_color(vl, LV_OBJ_PART_MAIN,
                                    LV_STATE_DEFAULT, LV_COLOR_BLUE);
    lv_obj_set_style_local_bg_opa(vl, LV_OBJ_PART_MAIN,
                                  LV_STATE_DEFAULT, LV_OPA_COVER);
    lv_obj_set_style_local_border_width(vl, LV_OBJ_PART_MAIN,
                                        LV_STATE_DEFAULT, 0);

    /* Centre block, so the exact pixel the operator has to hit is
     * unambiguous even at a glancing angle. */
    lv_obj_t *dot = lv_obj_create(cal_container, NULL);
    lv_obj_set_size(dot, 14, 14);
    lv_obj_set_pos(dot, tx - 7, ty - 7);
    lv_obj_set_style_local_bg_color(dot, LV_OBJ_PART_MAIN,
                                    LV_STATE_DEFAULT, LV_COLOR_BLUE);
    lv_obj_set_style_local_bg_opa(dot, LV_OBJ_PART_MAIN,
                                  LV_STATE_DEFAULT, LV_OPA_COVER);
    lv_obj_set_style_local_border_width(dot, LV_OBJ_PART_MAIN,
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

/* Build the results screen
 *
 * No buttons, and that is the whole point of the change: the operator never
 * confirms anything.  Either the fit passes the residual check, in which
 * case it is committed to g_config, flagged for the flash write, and
 * cal_finish() drops straight back to the main screen - or it fails, in
 * which case cal_restart_from_first_point() wipes the samples and asks for
 * point 0 again.
 *
 * A screen the operator can leave half-done is worse than one that insists
 * on finishing: g_config.touch.calibrated would stay 0 and the next power
 * cycle would ask the same five questions all over again.
 *
 * The coefficients stay on screen for a moment on failure because they are
 * the only clue to WHY it failed.  A residual just over the limit almost
 * always means one target got tapped slightly off, and seeing the fitted
 * line lets the person aim differently on the next pass.
 */
static void cal_build_results_ui(void)
{
    cal_del_container();

    /* cal_calculate() has two early "return false" paths (degenerate input,
     * residual > 5%) that leave the outputs untouched, and the display below
     * prints them unconditionally.  Seed everything to a defined value so a
     * failed calibration shows "0.000" instead of whatever was on the stack. */
    float x_slope = 0.0f, x_off = 0.0f, y_slope = 0.0f, y_off = 0.0f,
          residual = 0.0f;
    bool passed = cal_calculate(&x_slope, &x_off, &y_slope, &y_off, &residual);

    if (passed) {
#ifndef LVGL_SIM
        /* Straight float fit: screen = raw * gain + offset */
        g_config.touch.x_gain     = x_slope;
        g_config.touch.x_offset   = x_off;
        g_config.touch.y_gain     = y_slope;
        g_config.touch.y_offset   = y_off;
        g_config.touch.calibrated = 1;
#endif
        /* cal_finish() flags g_config dirty so app_config_task() persists the
         * new coefficients, clears the boot-time "calibration required"
         * flag, and loads the main screen.  No green screen, no OK button:
         * if the numbers are good, the calibration is simply done. */
        cal_finish();
        return;
    }

    /* Failure: show why, then restart.  Build the container only now - on
     * the success path there is nothing to look at. */
    cal_container = lv_obj_create(cal_screen, NULL);
    lv_obj_set_size(cal_container, HOR_RES, VER_RES);
    lv_obj_set_pos(cal_container, 0, 0);
    lv_obj_set_style_local_bg_opa(cal_container, LV_OBJ_PART_MAIN,
                                   LV_STATE_DEFAULT, LV_OPA_TRANSP);
    lv_obj_set_style_local_border_width(cal_container, LV_OBJ_PART_MAIN,
                                        LV_STATE_DEFAULT, 0);

    lv_obj_t *result_lbl = lv_label_create(cal_container, NULL);
    lv_label_set_text(result_lbl, gui_get_string(STR_CAL_FAILED));
    lv_obj_align(result_lbl, cal_container, LV_ALIGN_IN_TOP_MID, 0, 20);
    lv_obj_set_style_local_text_color(result_lbl, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, LV_COLOR_RED);
    lv_obj_set_style_local_text_font(result_lbl, LV_LABEL_PART_MAIN,
                                     LV_STATE_DEFAULT, &MyCJK_16);

    char buf[128];
    lv_obj_t *params_lbl = lv_label_create(cal_container, NULL);
    /* The least-squares fit itself is genuinely floating point; only the
     * rendering is converted to fixed point, so printf never sees a float. */
    char slope_x[16], off_x[16], slope_y[16], off_y[16], resid[16];
    gui_fmt_fixed(slope_x, sizeof(slope_x), (int32_t)(x_slope * 1000.0f), 3U);
    gui_fmt_fixed(off_x,   sizeof(off_x),   (int32_t)(x_off   *  10.0f), 1U);
    gui_fmt_fixed(slope_y, sizeof(slope_y), (int32_t)(y_slope * 1000.0f), 3U);
    gui_fmt_fixed(off_y,   sizeof(off_y),   (int32_t)(y_off   *  10.0f), 1U);
    gui_fmt_fixed(resid,   sizeof(resid),   (int32_t)(residual *  10.0f), 1U);
    snprintf(buf, sizeof(buf),
        "X: y = %s * x + %s\n"
        "Y: y = %s * x + %s\n"
        "%s: %s%% (max 5%%)",
        slope_x, off_x, slope_y, off_y,
        gui_get_string(STR_CAL_RESIDUAL), resid);
    lv_label_set_text(params_lbl, buf);
    lv_obj_align(params_lbl, cal_container, LV_ALIGN_CENTER, 0, -20);
    lv_obj_set_style_local_text_color(params_lbl, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, LV_COLOR_BLACK);
    lv_obj_set_style_local_text_font(params_lbl, LV_LABEL_PART_MAIN,
                                     LV_STATE_DEFAULT, &MyCJK_16);

    /* Say what happens next, so a restart is not mistaken for a hang. */
    lv_obj_t *hint_lbl = lv_label_create(cal_container, NULL);
    lv_label_set_text(hint_lbl, gui_get_string(STR_CAL_RESTART));
    lv_obj_align(hint_lbl, cal_container, LV_ALIGN_IN_BOTTOM_MID, 0, -50);
    lv_obj_set_style_local_text_color(hint_lbl, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, LV_COLOR_RED);
    lv_obj_set_style_local_text_font(hint_lbl, LV_LABEL_PART_MAIN,
                                     LV_STATE_DEFAULT, &MyCJK_16);
}

/* Wipe the samples and ask for point 0 again.  Exactly what the Retry
 * button used to do, minus the button. */
static void cal_restart_from_first_point(void)
{
    calibrating   = true;
    cal_state     = 1;
    cal_point_idx = 0;
    memset(cal_data, 0, sizeof(cal_data));
    lv_obj_clean(cal_screen);
    cal_container = NULL;
    gui_update_calibration(0);
}

/* Advance to next point, or finish the pass.
 *
 * All five points collected: run the fit.  cal_build_results_ui() either
 * commits and leaves (cal_finish) or shows the failed fit - and in the
 * latter case arms the auto-restart below, so the operator gets to read the
 * coefficients before being asked to tap point 0 again.
 */
static void gui_calibration_advance(void)
{
    cal_point_idx++;
    if (cal_point_idx >= CAL_POINT_COUNT) {
        cal_state = 2;
        cal_build_results_ui();

        /* Only arm the countdown when the fit actually failed.  On success
         * cal_build_results_ui() has already called cal_finish(), which
         * clears `calibrating` and loads the main screen - arming here would
         * leave a stray timer about to yank the operator off the main
         * screen. */
        if (calibrating) {
            s_cal_restart_tick = lv_tick_get();
        }
    } else {
        cal_state = 1;
        cal_build_point_ui(cal_point_idx);
    }
}

/* Service the failed-pass countdown.  Called from gui_process_calibration()
 * on every GUI tick.
 *
 * Why a delay at all: the fitted coefficients are the only clue to why the
 * pass failed, and restarting instantly would flash them past before anyone
 * could read them.  3 s is long enough to read five numbers and short enough
 * that a deliberate retry does not feel like a hang. */
static void gui_process_cal_restart(void)
{
    if (!calibrating || cal_state != 2) {
        return;
    }
    /* Unsigned subtraction, so the lv_tick_get() wrap is a non-issue. */
    if ((lv_tick_get() - s_cal_restart_tick) < CAL_RESTART_DELAY_MS) {
        return;
    }
    cal_restart_from_first_point();
}

static void gui_create_cal_screen(void)
{
    cal_screen = lv_obj_create(NULL, NULL);
    lv_obj_set_size(cal_screen, HOR_RES, VER_RES);
    lv_obj_set_style_local_bg_color(cal_screen, LV_OBJ_PART_MAIN,
                                     LV_STATE_DEFAULT, LV_COLOR_WHITE);
    /* Without this the screen is transparent (no theme -> bg_opa defaults to
     * LV_OPA_TRANSP) and the black calibration text has nothing to read on. */
    lv_obj_set_style_local_bg_opa(cal_screen, LV_OBJ_PART_MAIN,
                                  LV_STATE_DEFAULT, LV_OPA_COVER);
}

/*============================================================================
 *  LVGL 触屏输入设备（indev）
 *
 *  不注册 indev，LVGL 就根本不会去读触摸，点击对任何控件都不起作用 ——
 *  界面能画、能刷新，但点不动。
 *
 *  read_cb 由 LVGL 在 lv_task_handler() 内部调用（也就是 gui_tick() 里
 *  那一处），每个 tick 一次。它只读 touch.c 发布的 volatile 快照，既不碰
 *  I2C 也不阻塞——采样节奏由 touch_process() 与 I2C 中断决定，LVGL 只是
 *  消费者。这也是为什么 DMA 那一层改成非阻塞之后界面能跟上。
 *
 *  范式照 OPA1000（Code/Src/LVGL_GUI.c 的 lvGetTouchscreenXY），同样的
 *  LVGL v7 API：
 *      lv_indev_drv_init / .type / .read_cb / lv_indev_drv_register
 *  OPA1000 还注册了旋钮（LV_INDEV_TYPE_ENCODER），本项目无旋钮，略。
 *==========================================================================*/

/* 整块只在固件构建：PC 模拟器没有 touch.c（依赖 STM32 HAL），由
 * LVGL.Simulator.cpp 自己的输入模拟驱动 indev。见 gui_indev_register()
 * 的 @note。 */
#ifndef LVGL_SIM

/*!< LVGL 输入设备描述符与句柄。static：只在本文件用。 */
static lv_indev_drv_t s_indev_drv;
static lv_indev_t    *s_indev_touch = NULL;

/*!< 最近一次**真正取到**的样本，喂给 LVGL 用。
 *
 *  LVGL v7 的 read_cb 只能填 data->point / data->state，没有"这帧没新
 *  数据"的表达方式（lv_indev_data_t 里只有 continue_reading，而它是用来
 *  让 LVGL 继续问下一路设备的，不是"我无数据"）。所以驱动必须把当前状态
 *  一直喂回去，于是就得在本地留一份缓存：touch_take_sample() 返回 false
 *  时就沿用上一帧的值，而不是重读快照或清零。
 *
 *  只有 GUI 任务会碰这三个变量（read_cb 由 lv_task_handler() 在 gui_tick()
 *  里调用，而 gui_tick() 已持有 gui 锁），所以不需要 volatile 或加锁。 */
static uint16_t s_indev_x  = 0U;
static uint16_t s_indev_y  = 0U;
static bool     s_indev_pr = false;

/**
 * @brief LVGL 读取触摸回调。
 *
 * LVGL v7 的签名：填 data->point / data->state。返回值恒为 false——本项目
 * 只有一路输入设备，不需要 LVGL 接着去轮询别的设备。
 */
static bool gui_indev_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    TouchPointType tp;

    (void)drv;

    if (data == NULL) {
        return false;
    }
    if(calibrating)
        return false; // do not race with calibrating

    /* LVGL 每 tick 调一次这里，而采样是 10 ms 一次——两个周期不同频，
     * 绝大多数 tick 拿到的都是同一份样本。所以用 touch_take_sample()：
     * 没有新样本时它返回 false，data 保持**上一帧的取值不变**。
     *
     * 这一点很关键：LVGL v7 的 read_cb 没有"我这帧没数据"的表达方式（只有
     * continue_reading），驱动必须把当前状态一直喂回去。因此这里维护一份
     * 缓存，只有真拿到新样本才更新——既能避免反复上报同一个历史值，也不
     * 依赖 LVGL 内部的去重实现。范式同 OPA1000 的 lvGetTouchscreenXY
     * （main.c），那里是 TS_touched 取走即清。
     */
    if (touch_take_sample(&tp)) {
        s_indev_x = tp.x;
        s_indev_y = tp.y;
        s_indev_pr = tp.pressed;

#if DEBUG_TOUCH_RAW
        /* Latch the panel's own numbers, not tp.*.  tp has already been
         * through touch_apply_calibration(), so it is exactly the value
         * that is suspected of being wrong - useless for telling you how
         * wrong.  g_touch_raw_* is what the ISR decoded straight off the
         * I2C bus (NS2009: 0..4095; GT911: panel pixels).
         *
         * g_touch_raw_z is the NS2009 contact pressure NS2009_PRESSURE()
         * derives from the panel's Z1/Z2 pair, i.e. the single number the
         * press threshold NS2009_Z_PRESS_THRESHOLD is compared against.
         * For a capacitive panel it stays 0 - there is no resistive Z.
         *
         * Only latched here; the label write happens in the GUI task pump.
         * Updating a widget from inside read_cb means updating it from
         * inside lv_task_handler() while LVGL is iterating its object
         * tree. */
        s_touch_raw_x = g_touch_raw_x;
        s_touch_raw_y = g_touch_raw_y;
        s_touch_raw_z = g_touch_raw_z;
        s_touch_raw_dirty = true;
#endif
    }

    /* 按下与抬起都要填坐标：LVGL 靠 state 判断这一帧算不算"按下"，而拖动
     * 与松手判断依赖最后一帧的坐标。把坐标清零反而会让控件在抬手瞬间收到
     * 一次跳到 (0,0) 的坐标，造成误点击。 */
    data->point.x = s_indev_x;
    data->point.y = s_indev_y;
    data->state   = s_indev_pr ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;

    return false;
}

/**
 * @brief 注册触摸输入设备。gui_init() 里调用一次。
 *
 * 失败时 s_indev_touch 保持 NULL：gui_tick() 照样跑，只是界面点不动。这
 * 比在初始化阶段直接死掉好——GUI 失去输入是能看见的状态，panic 不是。
 */
static void gui_indev_register(void)
{
    lv_indev_drv_init(&s_indev_drv);
    s_indev_drv.type    = LV_INDEV_TYPE_POINTER;
    s_indev_drv.read_cb = gui_indev_read_cb;
    s_indev_touch       = lv_indev_drv_register(&s_indev_drv);
}

#endif /* !LVGL_SIM */

/*============================================================================
 * Public Functions
 *==========================================================================*/

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

#ifndef LVGL_SIM
    /* Register the touch input device.  Must come after lv_init() (LVGL
     * has to exist) but the exact position relative to the screen build
     * does not matter - lv_indev_drv_register() just adds an entry to
     * LVGL's indev list, which lv_task_handler() walks from its first call
     * in gui_tick(). */
    gui_indev_register();
    boot_backlight_on();
#endif
}

void gui_start(void)
{
    /* Intentionally empty; placeholder for future "post-init start" hooks
     * (animations, first-time focus, etc.).  No lock needed. */
}

void gui_tick(void)
{

    /* No lv_tick_inc() here on purpose.  lv_conf.h sets LV_TICK_CUSTOM = 1,
     * so LVGL reads its millisecond base straight from the FreeRTOS tick
     * counter (1 kHz, so the count IS milliseconds - see lv_tick_port.h).
     *
     * Adding lv_tick_inc() on top would create a second time base that only
     * advances when this task gets scheduled, and this task is priority 3 -
     * below ADC (4), Monitor (5) and Protect (6).  Any preemption would make
     * LVGL's clock fall permanently behind real time, slowing every animation
     * and timer by an amount that varies with scheduler load. */


}

void gui_update_display(void)
{

#if DEBUG_TOUCH_RAW
    /* Touch raw readout.  gui_indev_read_cb() only latched the numbers (it
     * runs inside lv_task_handler(), where touching a widget is unsafe);
     * the label write happens here, in the task pump, with every other
     * widget update. */
    if (s_touch_raw_dirty && lbl_touch_raw != NULL) {
        s_touch_raw_dirty = false;
        char raw[48];
        snprintf(raw, sizeof(raw), "X:%u Y:%u Z:%lu",
                 (unsigned)s_touch_raw_x, (unsigned)s_touch_raw_y,
                 (unsigned long)s_touch_raw_z);
        /* Not gui_label_set_cached(): that cache exists to suppress
         * redraws of the main screen's live values, and this label is
         * already only written when the sample actually changed. */
        lv_label_set_text(lbl_touch_raw, raw);
        /* Reveal on first sample.  Creating it hidden keeps an untouched
         * panel from showing "X:0 Y:0 Z:0", which reads like a fault. */
        lv_obj_set_hidden(lbl_touch_raw, false);
        lv_obj_move_foreground(lbl_touch_raw);
    }
#endif

    if (!alert_active && !calibrating) {
        gui_update_main_values();
    }
    /* Refresh the top-of-screen debug overlay only once . */
    uint32_t now = lv_tick_get();
    if (s_debug_last_tick_ms == 0) {
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
        }
    }


}

/*============================================================================
 *  保护告警弹窗
 *==========================================================================*/

/**
 * @brief Dismiss 按钮的回调。
 *
 * 关键：这里**必须**调 protection_clear()，不能只调 gui_dismiss_alert()。
 * 后者只删控件，不动 protection.c 里的锁存——那样会出现更糟的状态：
 * 弹窗关了，但 s_protection_active 仍是 true，下一次 protection_check()
 * 立刻重新触发，弹窗反复弹出，而且 g_allow_rf 永远是 false，操作员
 * 再也发射不了。
 *
 * protection_clear() 一次性做四件事：清锁存、拉低 PB13（通知 RF 板保护
 * 已解除）、停蜂鸣器、清 g_alert_flag。g_allow_rf 保持 false —— 那是设计
 * 意图：操作员必须显式重跑一次 IM 零点标定才能重新武装射频。
 *
 * 不在 LVGL 回调里直接删 msgbox（LVGL 正在遍历这棵树，删了就悬空），
 * 由 GUI 任务的泵去关——见 gui_dismiss_alert() 的调用点。
 */
static void alert_dismiss_cb(lv_obj_t *obj, lv_event_t e)
{
    (void)obj;

    if (e != LV_EVENT_CLICKED) {
        return;
    }
    protection_clear();
}

void gui_show_alert(AlertType alert_type)
{
    /* Called from the GUI task pump, NOT from an ISR: the fault path only
     * sets g_alert_flag (see protection_trigger_locked() and the "no gui_*
     * from ISR" rule in gui.h).  Must take the lock. */


    /* Re-entrancy guard: one popup per fault.  protection_trigger_locked()
     * already suppresses repeat triggers until cleared, but a stray second
     * call here would otherwise leak the previous msgbox. */
    if (alert_msgbox != NULL) {

        return;
    }

    /* Text.  All eight causes are mapped - ALERT_NTC_OVERHEAT and
     * ALERT_VBIAS_FAULT used to fall through to the generic STR_ALERT,
     * which protection_get_fault_string() in protection.c does handle, so
     * the two files disagreed about what the operator should be told. */
    const char *alert_text;
    switch (alert_type) {
        case ALERT_SWR_HIGH:            alert_text = gui_get_string(STR_SWR_HIGH); break;
        case ALERT_REFLECT_POWER_HIGH:  alert_text = gui_get_string(STR_REF_HIGH); break;
        case ALERT_CURRENT_HIGH:        alert_text = gui_get_string(STR_CURRENT_HIGH); break;
        case ALERT_HARDWARE_FAULT:      alert_text = gui_get_string(STR_HARDWARE_FAULT); break;
        case ALERT_OVERDRIVE:           alert_text = gui_get_string(STR_OVERDRIVE); break;
        case ALERT_FREQ_MISMATCH:       alert_text = gui_get_string(STR_FREQ_MISMATCH); break;
        case ALERT_NTC_OVERHEAT:        alert_text = gui_get_string(STR_NTC_OVERHEAT); break;
        case ALERT_VBIAS_FAULT:         alert_text = gui_get_string(STR_VBIAS_FAULT); break;
        case ALERT_FAN_FAULT:           alert_text = gui_get_string(STR_FAN_FAULT); break;
        default:                        alert_text = gui_get_string(STR_ALERT); break;
    }

    /* static: lv_msgbox_add_btns() keeps the pointer, so the array has to
     * outlive this function (see CLAUDE.md "LVGL String Lifetime"). */
    static const char *btn_txts[] = { NULL, "" };   /* filled below */

    alert_msgbox = lv_msgbox_create(main_screen, NULL);
    lv_msgbox_set_text(alert_msgbox, alert_text);

    /* Button label is language-dependent, like the other popups in this
     * file (show_language_selector etc.) - "解除" reads better than a bare
     * "Dismiss" in Chinese. */
    btn_txts[0] = gui_get_string(STR_DISMISS);
    lv_msgbox_add_btns(alert_msgbox, btn_txts);
    lv_obj_set_size(alert_msgbox, 280, 110);
    lv_obj_align(alert_msgbox, main_screen, LV_ALIGN_CENTER, 0, 0);

    /* Style: without an explicit font the CJK glyphs render as boxes, and
     * the default msgbox background does not match the rest of the UI. */
    lv_obj_set_style_local_text_color(alert_msgbox, LV_MSGBOX_PART_BG,
                                      LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(alert_msgbox, LV_MSGBOX_PART_BG,
                                      LV_STATE_DEFAULT, &MyCJK_12);
    lv_obj_set_style_local_bg_color(alert_msgbox, LV_MSGBOX_PART_BG,
                                     LV_STATE_DEFAULT, LV_COLOR_BLUE);

    lv_obj_t *btnm = lv_msgbox_get_btnmatrix(alert_msgbox);
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

    /* The click that ends it.  Without this the button is decorative. */
    lv_obj_set_event_cb(alert_msgbox, alert_dismiss_cb);

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

    /* The loop above writes labels directly, bypassing the change-guard
     * cache.  Without dropping the cache, the next gui_update_main_values()
     * pass would compare each label against its *pre-switch* text, find a
     * match whenever the string happens to be language-neutral ("12W",
     * "ICOM"), and skip the write - so parts of the screen would keep the
     * old language indefinitely. */
    gui_label_cache_invalidate();
}

void gui_show_calibration(void)
{

    if (cal_screen == NULL) gui_create_cal_screen();
    calibrating = true;
    cal_state = 1;
    cal_point_idx = 0;
    memset(cal_data, 0, sizeof(cal_data));
    /* 清掉连点锁定：重新进入校准（含标定失败后点 Retry）时不该被上一轮
     * 残留的窗口挡住第一下操作。减法写法对回绕安全——即使 now 曾回绕到比
     * 旧值小，(now - old) 是一个巨大的正数，判定自然通过。 */
    s_cal_lockout_tick -= CAL_LOCKOUT_MS;
#ifndef LVGL_SIM
    touch_calibrate_start();
    /* The calibration screen is the usual FIRST thing the operator sees
     * after a factory reset, and gui_process() only raises g_gui_ready
     * after three painted ticks of the MAIN screen.  If we relied on
     * that path alone the panel would stay black while asking the user
     * to press targets - the one screen that must never be unreadable.
     * Idempotent: re-entering just re-writes the same duty cycle. */

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
    /* Auto-restart after a failed pass runs before the tap handling below:
     * it must fire even though cal_state is 2, not 1. */
    gui_process_cal_restart();

    if (!calibrating || cal_state != 1) {

        return;
    }

    /* 防连点锁定：见 CAL_LOCKOUT_MS。用 lv_tick_get() 作时基（本文件其它
     * 地方亦然），单位毫秒。检查完立即返回，不占 CPU、不破坏 10 ms 节拍。 */
    {
        uint32_t now = lv_tick_get();

        if ((now - s_cal_lockout_tick) < CAL_LOCKOUT_MS) {

            return;
        }

        /* 要原始坐标：标定期间 g_config.touch.calibrated 仍为 0，所以
         * touch_take_sample() 返回的未经变换。 */
        TouchPointType tp;
        if (touch_take_sample(&tp) && tp.pressed) {
            uint8_t idx = cal_point_idx;
            if (idx >= CAL_POINT_COUNT) {

                return;
            }

            cal_data[idx].raw_x = tp.x;
            cal_data[idx].raw_y = tp.y;
            cal_data[idx].screen_x = cal_targets[idx].x;
            cal_data[idx].screen_y = cal_targets[idx].y;

            s_cal_lockout_tick = now;   /* 立刻上锁，丢掉按住期间的余波 */
            gui_calibration_advance();
        }
    }
#endif

}

void gui_power_button_pressed(void)
{

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
        gui_label_set_cached(status_label, gui_get_string(STR_STANDBY));
        lv_obj_set_style_local_bg_color(banner, LV_OBJ_PART_MAIN,
                                         LV_STATE_DEFAULT, LV_COLOR_MAKE(0xea, 0xb9, 0x06));
        lv_obj_set_style_local_bg_color(bottom_bar, LV_OBJ_PART_MAIN,
                                         LV_STATE_DEFAULT, LV_COLOR_MAKE(0xea, 0xb9, 0x06));
    } else {
        gui_label_set_cached(status_label, gui_get_string(STR_IDLE));
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

static const char *screen_btns[] = {"Touch Cal", "Radio Baud", "\n", "Cancel", ""};

static void show_screen_radio_baud(void);
static void show_screen_cat_baud(void);

static void screen_settings_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
        uint16_t btn_id = lv_msgbox_get_active_btn(obj);
        if (btn_id == 0) {  /* Touch Calibration */
            lv_obj_del_async(obj);
            gui_show_calibration();
            return;
        }
        if (btn_id == 1) {  /* Radio Baud */
            lv_obj_del_async(obj);
            show_screen_radio_baud();
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

/*============================================================================
 * Baud-rate Selection Popups
 *============================================================================*/

/* 5 个标准 CAT 波特率（1200 / 2400 / 4800 / 9600 / 19200 / 38400 / 57600 /
 * 115200）。显示用 lv_spinbox 而不是 btnmatrix：操作员点 +/- 按钮
 * 在合法值列表里循环，title 区实时显示当前值，"Set" 按钮把当前值
 * 写回 g_config 并标 dirty（立即生效；comm 任务下一次循环调
 * radio_uart_set_baud 或 pc_uart_set_baud 把值写到对应 UART），"Back"
 * 关闭弹窗而不保存。 */
static const int32_t k_baud_table[] = {
    1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200
};
#define BAUD_TABLE_LEN  ((int32_t)(sizeof(k_baud_table)/sizeof(k_baud_table[0])))

/* Returns the index of the table entry that matches `baud`, or 0 if
 * no exact match (the caller treats 0 as "use the spinner's first
 * legal value" - the spinner only ever shows table entries, so this
 * is only a cold-start edge case).  Currently unused since the
 * roller pre-select uses an inline loop; kept commented for the
 * next reader who needs to look up an index. */
#if 0
static int32_t baud_table_index(uint32_t baud)
{
    for (int32_t i = 0; i < BAUD_TABLE_LEN; i++) {
        if ((uint32_t)k_baud_table[i] == baud) return i;
    }
    return 0;
}
#endif

/* Build a roller popup.  `title_zh / title_en` label the heading,
 * `field` is the destination (radio or PC).  The popup owns its
 * scratch value (s_pending_baud) and only commits it to `field` /
 * g_config when the operator presses Save.  Back discards the
 * scratch - which is the whole point of adding the Save button:
 * scrolling the roller around while exploring the table must NOT
 * touch g_config or schedule a flash write per sample.
 *
 * Layout:
 *   +----------------------------+
 *   | <title>                    |
 *   |                            |
 *   |   <prev>                   |
 *   |   <sel>    <-- selected    |
 *   |   <next>                   |
 *   |                            |
 *   |  [Save]      [Back]        |
 *   +----------------------------+
 */
static lv_obj_t *baud_spinner_popup = NULL;
static lv_obj_t *baud_spinner_roller = NULL;
static volatile uint32_t *baud_spinner_field = NULL;
static lv_obj_t *baud_spinner_title = NULL;

/* Scratch value updated only on Save.  Kept in sync with the current
 * g_config value at popup-open time; roller events update this, but
 * never touch g_config. */
static uint32_t s_pending_baud = 0U;

/* Save button - copy the scratch value into g_config, mark the menu
 * dirty so the config task flushes to flash, and close. */
static void baud_roller_save_cb(lv_obj_t *obj, lv_event_t e)
{
    (void)obj;
    if (e != LV_EVENT_CLICKED) return;
    if (!baud_spinner_popup || !baud_spinner_field) return;

#ifndef LVGL_SIM
    /* Commit the staged value.  The comm task will reprogram the UART
     * within the next 10 ms tick (it compares against last_applied_*_baud)
     * and the config task flushes g_config to flash on its 1 s window. */
    *baud_spinner_field = s_pending_baud;
    gui_menu_mark_dirty();
#else
    (void)baud_spinner_field;
#endif

    lv_obj_del_async(baud_spinner_popup);
    baud_spinner_popup  = NULL;
    baud_spinner_roller = NULL;
    baud_spinner_field  = NULL;
    baud_spinner_title  = NULL;
}

/* Back button - close without persisting.  s_pending_baud is
 * discarded along with the popup. */
static void baud_roller_back_cb(lv_obj_t *obj, lv_event_t e)
{
    (void)obj;
    if (e != LV_EVENT_CLICKED) return;
    if (!baud_spinner_popup) return;
    lv_obj_del_async(baud_spinner_popup);
    baud_spinner_popup  = NULL;
    baud_spinner_roller = NULL;
    baud_spinner_field  = NULL;
    baud_spinner_title  = NULL;
}

/* Roller value-changed handler.  Only updates s_pending_baud; the
 * config field is touched exclusively by the Save button.  Scrolling
 * the roller is therefore free (no flash writes) - the operator can
 * sample the whole table without paying a write per tick. */
static void baud_roller_event_cb(lv_obj_t *obj, lv_event_t e)
{
    (void)obj;
    if (e != LV_EVENT_VALUE_CHANGED) return;
    if (!baud_spinner_roller) return;
    uint16_t sel = lv_roller_get_selected(baud_spinner_roller);
    if (sel >= BAUD_TABLE_LEN) return;
    s_pending_baud = (uint32_t)k_baud_table[sel];
}

static void show_baud_spinbox(const char *title_zh,
                              const char *title_en,
                              volatile uint32_t *field)
{
    /* Tear down any previous roller instance so we never end up with
     * a stale popup when the operator enters twice. */
    if (baud_spinner_popup) {
        lv_obj_del_async(baud_spinner_popup);
        baud_spinner_popup = NULL;
    }

    lv_obj_t *scr = lv_scr_act();
    baud_spinner_popup = lv_obj_create(scr, NULL);
    lv_obj_set_size(baud_spinner_popup, 280, 200);
    lv_obj_align(baud_spinner_popup, scr, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_local_bg_color(baud_spinner_popup, LV_OBJ_PART_MAIN,
                                    LV_STATE_DEFAULT, COLOR_BG);
    lv_obj_set_style_local_bg_opa(baud_spinner_popup, LV_OBJ_PART_MAIN,
                                  LV_STATE_DEFAULT, LV_OPA_COVER);
    lv_obj_set_style_local_border_width(baud_spinner_popup, LV_OBJ_PART_MAIN,
                                         LV_STATE_DEFAULT, 2);
    lv_obj_set_style_local_border_color(baud_spinner_popup, LV_OBJ_PART_MAIN,
                                         LV_STATE_DEFAULT, LV_COLOR_BLUE);

    baud_spinner_field = field;
    /* Seed the scratch from the current g_config so the roller's
     * initial selection matches reality and a no-op Save is a real
     * no-op. */
    s_pending_baud = *field;

    /* Title (yellow).  CJK string "Radio 波特率" / "PC 波特率" is wider
     * than the popup width / 2, so we use MyCJK_12 (smaller) and
     * LV_LABEL_LONG_CROP so a too-wide title clips inside the popup
     * instead of overflowing the box. */
    baud_spinner_title = lv_label_create(baud_spinner_popup, NULL);
    lv_label_set_text(baud_spinner_title,
                      (gui_get_lang_index() == 0) ? title_zh : title_en);
    lv_label_set_long_mode(baud_spinner_title, LV_LABEL_LONG_CROP);
    lv_obj_set_width(baud_spinner_title, 260);
    lv_obj_align(baud_spinner_title, baud_spinner_popup, LV_ALIGN_IN_TOP_MID,
                 0, 4);
    lv_obj_set_style_local_text_color(baud_spinner_title, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, LV_COLOR_YELLOW);
    lv_obj_set_style_local_text_font(baud_spinner_title, LV_LABEL_PART_MAIN,
                                     LV_STATE_DEFAULT, &MyCJK_12);

    /* Build the roller option list out of k_baud_table[].  Same
     * "\n"-separated format OPA1000 uses (LVGL_items.c optsRollerBaudrates). */
    static char opts[BAUD_TABLE_LEN * 8];
    opts[0] = '\0';
    for (int32_t i = 0; i < BAUD_TABLE_LEN; i++) {
        char tmp[12];
        snprintf(tmp, sizeof tmp, "%lu", (unsigned long)k_baud_table[i]);
        if (i > 0) strncat(opts, "\n", sizeof(opts) - strlen(opts) - 1);
        strncat(opts, tmp, sizeof(opts) - strlen(opts) - 1);
    }

    baud_spinner_roller = lv_roller_create(baud_spinner_popup, NULL);
    /* visible row count = 3 -> one row above the selected, the
     * selected, one row below.  anim time 0 so the roller snaps
     * immediately on every keystroke / touch. */
    lv_roller_set_visible_row_count(baud_spinner_roller, 3);
    lv_roller_set_anim_time(baud_spinner_roller, 0);
    lv_roller_set_options(baud_spinner_roller, opts, LV_ROLLER_MODE_NORMAL);
    lv_roller_set_align(baud_spinner_roller, LV_LABEL_ALIGN_CENTER);
    lv_obj_set_size(baud_spinner_roller, 240, 80);
    lv_obj_align(baud_spinner_roller, baud_spinner_popup, LV_ALIGN_CENTER,
                 0, 14);
    /* Highlight the selected row.  We use the default LV_ROLLER_PART_BG
     * + LV_ROLLER_PART_SELECTED styling but tighten the colours to
     * match the rest of the popup. */
    lv_obj_set_style_local_bg_color(baud_spinner_roller, LV_ROLLER_PART_BG,
                                     LV_STATE_DEFAULT,
                                     LV_COLOR_MAKE(20, 24, 28));
    lv_obj_set_style_local_text_color(baud_spinner_roller, LV_ROLLER_PART_BG,
                                      LV_STATE_DEFAULT,
                                      LV_COLOR_MAKE(160, 170, 180));
    lv_obj_set_style_local_bg_color(baud_spinner_roller, LV_ROLLER_PART_SELECTED,
                                     LV_STATE_DEFAULT,
                                     LV_COLOR_MAKE(0, 90, 130));
    lv_obj_set_style_local_text_color(baud_spinner_roller, LV_ROLLER_PART_SELECTED,
                                      LV_STATE_DEFAULT,
                                      LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(baud_spinner_roller, LV_LABEL_PART_MAIN,
                                     LV_STATE_DEFAULT, &lv_font_montserrat_16);

    /* Pre-select the current baud.  If it is not in the table, fall
     * back to the closest lower entry. */
    int32_t start_idx = 0;
    for (int32_t i = 0; i < BAUD_TABLE_LEN; i++) {
        if ((uint32_t)k_baud_table[i] == *field) { start_idx = i; break; }
        if ((uint32_t)k_baud_table[i] < *field) start_idx = i;
    }
    lv_roller_set_selected(baud_spinner_roller, start_idx, LV_ANIM_OFF);

    /* The roller fires VALUE_CHANGED on every selection but only
     * updates s_pending_baud; the config field is touched exclusively
     * by the Save button below.  Scrolling is free. */
    lv_obj_set_event_cb(baud_spinner_roller, baud_roller_event_cb);

    /* Save / Back row at the bottom of the popup. */
    lv_obj_t *save_btn = lv_btn_create(baud_spinner_popup, NULL);
    lv_obj_set_size(save_btn, 80, 30);
    lv_obj_align(save_btn, baud_spinner_popup, LV_ALIGN_IN_BOTTOM_LEFT, 20, -6);
    lv_obj_t *save_lbl = lv_label_create(save_btn, NULL);
    lv_label_set_text(save_lbl, "Save");
    lv_obj_set_style_local_text_font(save_lbl, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &MyCJK_12);
    lv_obj_set_event_cb(save_btn, baud_roller_save_cb);

    lv_obj_t *back_btn = lv_btn_create(baud_spinner_popup, NULL);
    lv_obj_set_size(back_btn, 80, 30);
    lv_obj_align(back_btn, baud_spinner_popup, LV_ALIGN_IN_BOTTOM_RIGHT,
                 -20, -6);
    lv_obj_t *back_lbl = lv_label_create(back_btn, NULL);
    lv_label_set_text(back_lbl, "Back");
    lv_obj_set_style_local_text_font(back_lbl, LV_LABEL_PART_MAIN,
                                       LV_STATE_DEFAULT, &MyCJK_12);
    lv_obj_set_event_cb(back_btn, baud_roller_back_cb);
}

/* Public entry points used by the Band Communication popup.  "Radio
 * Baud" drives g_config.radio_baud_rate (UART4 -> transceiver CAT),
 * "CAT Baud" drives g_config.pc_baud_rate (LPUART1 -> PC CAT link). */
static void show_screen_radio_baud(void)
{
    show_baud_spinbox("Radio 波特率", "Radio Baud",
                      &g_config.radio_baud_rate);
}

static void show_screen_cat_baud(void)
{
    show_baud_spinbox("PC 波特率", "CAT Baud",
                      &g_config.pc_baud_rate);
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
    const char *watts = ((step == 0 || step == 2) ? "5W" : "20W");
    snprintf(buf, len, "%s %s", dir, watts);
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
    lv_obj_set_style_local_bg_opa(adc_box, LV_OBJ_PART_MAIN,
                                  LV_STATE_DEFAULT, LV_OPA_COVER);
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

    snprintf(buf, sizeof(buf), "%s %s AM  |  %s",
             gui_get_string(STR_CAL_POINT),
             ((step == 0 || step == 2) ? "5" : "20"),
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
    if (!cc_active) {  return; }

    if (cc_rebuild_pending) {
        cc_rebuild_pending = false;
        if (cc_step >= CC_STEPS) {
            cc_state = 2;
            cc_build_results();
        } else {
            cc_state = 1;
            cc_build_ui(cc_step);
        }

        return;
    }

    if (cc_state != 1 || cc_adc_val_label == NULL) {  return; }

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
    gui_fmt_fixed(cc_buf, sizeof(cc_buf), (power_mW + 500) / 1000, 0U);
    strncat(cc_buf, " W", sizeof(cc_buf) - strlen(cc_buf) - 1U);
#else
    /* Simulator: show the nominal power of the current step so the layout
     * can be verified without a calibrated coupler in the config. */
    snprintf(cc_buf, sizeof(cc_buf), "%s W",
             (cc_step == 0 || cc_step == 2) ? "5" : "20");
#endif
    lv_label_set_text(cc_adc_val_label, cc_buf);
}

/* Public: show coupler calibration screen */
void gui_show_coupler_cal(uint8_t coupler_id)
{
    if (cc_screen == NULL) {
        cc_screen = lv_obj_create(NULL, NULL);
        lv_obj_set_size(cc_screen, HOR_RES, VER_RES);
        lv_obj_set_style_local_bg_color(cc_screen, LV_OBJ_PART_MAIN,
                                         LV_STATE_DEFAULT, LV_COLOR_WHITE);
        lv_obj_set_style_local_bg_opa(cc_screen, LV_OBJ_PART_MAIN,
                                      LV_STATE_DEFAULT, LV_OPA_COVER);
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

/* ADC2 has 2 regular ranks (IF PA / IF RADIO) plus 3 injected ranks (core
 * NTC, aux NTC, MCU internal).  The Sensor Inspect overlay shows all five
 * so the operator can verify both ADCs at once. */
#define SI_ADC2_ROW_COUNT  5U
static lv_obj_t *si_adc2_raw_labels[SI_ADC2_ROW_COUNT];
static lv_obj_t *si_adc2_val_labels[SI_ADC2_ROW_COUNT];

static void si_close_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
        lv_obj_del_async(si_overlay);
        si_overlay = NULL;
    }
}

/* Render one channel row at vertical position y on parent.  Shared by
 * the ADC1 and ADC2 sub-tables so the column geometry stays consistent
 * across sections.  raw_lbl / val_lbl are stored so the live update
 * loop can rewrite them every tick. */
static void si_draw_row(lv_obj_t *parent,
                        lv_coord_t y,
                        const char *name, const char *adc_txt,
                        lv_coord_t name_cx, lv_coord_t adc_cx,
                        lv_coord_t raw_cx, lv_coord_t val_cx,
                        lv_coord_t cw_name, lv_coord_t cw_adc,
                        lv_coord_t cw_raw,  lv_coord_t cw_val,
                        lv_obj_t **raw_lbl, lv_obj_t **val_lbl)
{
    /* name + adc text labels (static for the lifetime of the overlay) */
    lv_obj_t *n = lv_label_create(parent, NULL);
    lv_label_set_text(n, name);
    lv_obj_set_width(n, cw_name);
    lv_obj_set_pos(n, name_cx - cw_name / 2, y);
    lv_label_set_align(n, LV_LABEL_ALIGN_CENTER);
    lv_obj_set_style_local_text_color(n, LV_LABEL_PART_MAIN,
                                       LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(n, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &lv_font_montserrat_12);

    lv_obj_t *a = lv_label_create(parent, NULL);
    lv_label_set_text(a, adc_txt);
    lv_obj_set_width(a, cw_adc);
    lv_obj_set_pos(a, adc_cx - cw_adc / 2, y);
    lv_label_set_align(a, LV_LABEL_ALIGN_CENTER);
    lv_obj_set_style_local_text_color(a, LV_LABEL_PART_MAIN,
                                       LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(a, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &lv_font_montserrat_12);

    /* raw + value labels (mutated each tick by gui_update_sensor_inspect) */
    *raw_lbl = lv_label_create(parent, NULL);
    lv_label_set_text(*raw_lbl, "---");
    lv_obj_set_width(*raw_lbl, cw_raw);
    lv_obj_set_pos(*raw_lbl, raw_cx - cw_raw / 2, y);
    lv_label_set_align(*raw_lbl, LV_LABEL_ALIGN_CENTER);
    lv_obj_set_style_local_text_color(*raw_lbl, LV_LABEL_PART_MAIN,
                                       LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(*raw_lbl, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &lv_font_montserrat_16);

    *val_lbl = lv_label_create(parent, NULL);
    lv_label_set_text(*val_lbl, "---");
    lv_obj_set_width(*val_lbl, cw_val);
    lv_obj_set_pos(*val_lbl, val_cx - cw_val / 2, y);
    lv_label_set_align(*val_lbl, LV_LABEL_ALIGN_CENTER);
    lv_obj_set_style_local_text_color(*val_lbl, LV_LABEL_PART_MAIN,
                                       LV_STATE_DEFAULT, LV_COLOR_YELLOW);
    lv_obj_set_style_local_text_font(*val_lbl, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &lv_font_montserrat_16);
}

/* Draw the section title (yellow) and the four-column header (gray).
 * Reused for the ADC1 and ADC2 tables. */
static void si_draw_header(lv_obj_t *parent,
                           lv_coord_t title_y,
                           lv_coord_t hdr_y,
                           lv_coord_t x_offset,
                           const char *title,
                           lv_coord_t name_cx, lv_coord_t adc_cx,
                           lv_coord_t raw_cx, lv_coord_t val_cx,
                           lv_coord_t cw_name, lv_coord_t cw_adc,
                           lv_coord_t cw_raw,  lv_coord_t cw_val)
{
    /* Section title */
    lv_obj_t *t = lv_label_create(parent, NULL);
    lv_label_set_text(t, title);
    lv_obj_set_width(t, 270);
    lv_obj_set_pos(t, x_offset + 135, title_y);
    lv_label_set_align(t, LV_LABEL_ALIGN_CENTER);
    lv_obj_set_style_local_text_color(t, LV_LABEL_PART_MAIN,
                                       LV_STATE_DEFAULT, LV_COLOR_YELLOW);
    lv_obj_set_style_local_text_font(t, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &MyCJK_12);

    static const char *hdr[] = {"Name", "ADC", "Raw", "Value"};
    lv_coord_t hcxs[] = {name_cx, adc_cx, raw_cx, val_cx};
    lv_coord_t hws[]  = {cw_name, cw_adc, cw_raw, cw_val};
    for (int c = 0; c < 4; c++) {
        lv_obj_t *l = lv_label_create(parent, NULL);
        lv_label_set_text(l, hdr[c]);
        lv_obj_set_width(l, hws[c]);
        lv_obj_set_pos(l, hcxs[c] - hws[c] / 2, hdr_y);
        lv_label_set_align(l, LV_LABEL_ALIGN_CENTER);
        lv_obj_set_style_local_text_color(l, LV_LABEL_PART_MAIN,
                                           LV_STATE_DEFAULT, LV_COLOR_GRAY);
        lv_obj_set_style_local_text_font(l, LV_LABEL_PART_MAIN,
                                          LV_STATE_DEFAULT, &MyCJK_12);
    }
}

/* Column widths shared by both sections so the two tables line up when
 * the operator scrolls between them. */
#define SI_CW_NAME  78
#define SI_CW_ADC   38
#define SI_CW_RAW   58
#define SI_CW_VAL   78

/* ADC1 sub-table — 9 rows, indexed by ADC1_RANK_* (see opa2000.h). */
static void si_build_adc1_table(lv_obj_t *parent, lv_coord_t x, int count)
{
    lv_coord_t hdr_y = 26;
    lv_coord_t row_h = 16;

    lv_coord_t name_cx = x + SI_CW_NAME / 2;
    lv_coord_t adc_cx  = x + SI_CW_NAME + SI_CW_ADC / 2;
    lv_coord_t raw_cx  = x + SI_CW_NAME + SI_CW_ADC + SI_CW_RAW / 2;
    lv_coord_t val_cx  = x + SI_CW_NAME + SI_CW_ADC + SI_CW_RAW + SI_CW_VAL / 2;

    si_draw_header(parent, 4, hdr_y, x, "ADC1",
                   name_cx, adc_cx, raw_cx, val_cx,
                   SI_CW_NAME, SI_CW_ADC, SI_CW_RAW, SI_CW_VAL);

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
        si_draw_row(parent, y, ch_short[i], adc_txt,
                    name_cx, adc_cx, raw_cx, val_cx,
                    SI_CW_NAME, SI_CW_ADC, SI_CW_RAW, SI_CW_VAL,
                    &si_raw_labels[i], &si_val_labels[i]);
    }
}

/* ADC2 sub-table — 2 regular ranks (455 kHz IF samples for the PA /
 * radio paths) and 3 injected ranks (core NTC, aux NTC, MCU internal
 * temperature).  Channels per OPA2000_H7 固件设计要求.txt sec [ADC2
 * channel mapping]. */
static void si_build_adc2_table(lv_obj_t *parent, lv_coord_t x, lv_coord_t y_start)
{
    lv_coord_t row_h = 16;

    lv_coord_t name_cx = x + SI_CW_NAME / 2;
    lv_coord_t adc_cx  = x + SI_CW_NAME + SI_CW_ADC / 2;
    lv_coord_t raw_cx  = x + SI_CW_NAME + SI_CW_ADC + SI_CW_RAW / 2;
    lv_coord_t val_cx  = x + SI_CW_NAME + SI_CW_ADC + SI_CW_RAW + SI_CW_VAL / 2;

    lv_coord_t hdr_y = y_start + 22;
    si_draw_header(parent, y_start, hdr_y, x, "ADC2",
                   name_cx, adc_cx, raw_cx, val_cx,
                   SI_CW_NAME, SI_CW_ADC, SI_CW_RAW, SI_CW_VAL);

    /* Two regular ranks first (IF PA, IF RADIO), then three injected
     * temperature ranks (core NTC, aux NTC, MCU internal).  The "ADC"
     * column shows the input number; the live update path distinguishes
     * them by row index 0..4. */
    static const char *adc2_name[SI_ADC2_ROW_COUNT] = {
        "PA IF",     /* ADC2_RANK_IF_PA      */
        "Radio IF",  /* ADC2_RANK_IF_RADIO   */
        "Core NTC",  /* inj rank 1, INP2     */
        "Aux NTC",   /* inj rank 2, INP6     */
        "MCU Temp",  /* inj rank 3, TEMPSENSOR */
    };
    static const char *adc2_ch[SI_ADC2_ROW_COUNT] = {
        "INP0", "INP1", "INP2", "INP6", "TSENSE",
    };

    for (unsigned i = 0U; i < SI_ADC2_ROW_COUNT; i++) {
        lv_coord_t y = hdr_y + 16 + (lv_coord_t)i * row_h;
        si_draw_row(parent, y, adc2_name[i], adc2_ch[i],
                    name_cx, adc_cx, raw_cx, val_cx,
                    SI_CW_NAME, SI_CW_ADC, SI_CW_RAW, SI_CW_VAL,
                    &si_adc2_raw_labels[i], &si_adc2_val_labels[i]);
    }
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
    lv_obj_set_style_local_bg_opa(si_overlay, LV_OBJ_PART_MAIN,
                                  LV_STATE_DEFAULT, LV_OPA_COVER);
    lv_obj_set_style_local_radius(si_overlay, LV_OBJ_PART_MAIN,
                                   LV_STATE_DEFAULT, 0);

    lv_obj_t *title = lv_label_create(si_overlay, NULL);
    lv_label_set_text(title, gui_get_string(STR_SENSOR_INSPECT));
    lv_obj_align(title, si_overlay, LV_ALIGN_IN_TOP_MID, 0, 2);
    lv_obj_set_style_local_text_color(title, LV_LABEL_PART_MAIN,
                                       LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_text_font(title, LV_LABEL_PART_MAIN,
                                      LV_STATE_DEFAULT, &MyCJK_12);

    /* Scrollable page that holds both sub-tables.  272 px tall screen
     * cannot fit ADC1 (9 rows) + ADC2 (5 rows) at 16 px/row, so the
     * page grows beyond the overlay and the operator scrolls within
     * it.  The page sits between the title (top) and the close button
     * (bottom). */
    lv_obj_t *si_page = lv_page_create(si_overlay, NULL);
    lv_obj_set_size(si_page, o_w, o_h - 16 - 30);  /* leave space for close */
    lv_obj_align(si_page, si_overlay, LV_ALIGN_IN_TOP_MID, 0, 16);
    lv_page_set_scrollbar_mode(si_page, LV_SCROLLBAR_MODE_AUTO);
    /* Allow the page background to show the dark overlay color so the
     * scroll looks seamless. */
    lv_obj_set_style_local_bg_color(si_page, LV_PAGE_PART_BG,
                                    LV_STATE_DEFAULT, COLOR_BG);

    /* ADC1 sub-table — rows follow ADC1_RANK_* order. */
    si_build_adc1_table(si_page, 6, SI_ROW_COUNT);

    /* ADC2 sub-table sits below ADC1.  ADC1 ends at:
     *   hdr_y (26) + 16 (first row offset) + 9 rows * 16 px
     *   = 26 + 16 + 144 = 186
     * leaving a small gap before the ADC2 title.  186 + 6 = 192. */
    si_build_adc2_table(si_page, 6, 192);

    /* Close button stays on the overlay, anchored to the bottom so it
     * does not scroll with the tables above. */
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

    if (!si_overlay) {  return; }
    char buf[48];

    /* ADC1 rows. */
    for (unsigned i = 0; i < SI_ROW_COUNT; i++) {
#ifndef LVGL_SIM
        uint16_t raw = g_adc_data.adc1_raw[i];
        snprintf(buf, sizeof(buf), "%u", raw);
        if (si_raw_labels[i]) lv_label_set_text(si_raw_labels[i], buf);

        switch (i) {
            case ADC1_RANK_RF_IN:
                gui_fmt_fixed(buf, sizeof(buf),
                              ((int32_t)g_adc_data.rf_in_mW + 500) / 1000, 0U);
                strncat(buf, "W", sizeof(buf) - strlen(buf) - 1U);
                break;
            case ADC1_RANK_IM:
                gui_fmt_fixed(buf, sizeof(buf), (int32_t)g_adc_data.im_mA, 2U);
                strncat(buf, "A", sizeof(buf) - strlen(buf) - 1U);
                break;
            case ADC1_RANK_REV1:
                gui_fmt_fixed(buf, sizeof(buf),
                              ((int32_t)g_adc_data.rev1_mW + 500) / 1000, 0U);
                strncat(buf, "W", sizeof(buf) - strlen(buf) - 1U);
                break;
            case ADC1_RANK_FWD1:
                gui_fmt_fixed(buf, sizeof(buf),
                              ((int32_t)g_adc_data.fwd1_mW + 500) / 1000, 0U);
                strncat(buf, "W", sizeof(buf) - strlen(buf) - 1U);
                break;
            case ADC1_RANK_REV2:
                gui_fmt_fixed(buf, sizeof(buf),
                              ((int32_t)g_adc_data.rev2_mW + 500) / 1000, 0U);
                strncat(buf, "W", sizeof(buf) - strlen(buf) - 1U);
                break;
            case ADC1_RANK_VBIAS:
                gui_fmt_fixed(buf, sizeof(buf), (int32_t)g_adc_data.vbias_mV, 0U);
                strncat(buf, "mV", sizeof(buf) - strlen(buf) - 1U);
                break;
            case ADC1_RANK_FWD2:
                gui_fmt_fixed(buf, sizeof(buf),
                              ((int32_t)g_adc_data.fwd2_mW + 500) / 1000, 0U);
                strncat(buf, "W", sizeof(buf) - strlen(buf) - 1U);
                break;
            case ADC1_RANK_VM:
                gui_fmt_fixed(buf, sizeof(buf),
                              ((int32_t)g_adc_data.vm_mV + 500) / 1000, 0U);
                strncat(buf, "V", sizeof(buf) - strlen(buf) - 1U);
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

    /* ADC2 rows.  Two regular (IF PA / IF RADIO) plus three injected
     * (core NTC, aux NTC, MCU internal).  Row index 0 = PA IF, 1 = Radio
     * IF, 2 = Core NTC, 3 = Aux NTC, 4 = MCU Temp - matches the order
     * in si_build_adc2_table(). */
    for (unsigned i = 0; i < SI_ADC2_ROW_COUNT; i++) {
        uint16_t raw;
#ifndef LVGL_SIM
        switch (i) {
            case 0: raw = g_adc_data.adc2_if_pa_raw;       break;
            case 1: raw = g_adc_data.adc2_if_radio_raw;    break;
            case 2: raw = g_adc_data.adc2_inj_ntc_core_raw;break;
            case 3: raw = g_adc_data.adc2_inj_ntc_aux_raw; break;
            default:raw = g_adc_data.adc2_inj_tsense_raw;  break;
        }
        snprintf(buf, sizeof(buf), "%u", raw);
        if (si_adc2_raw_labels[i]) lv_label_set_text(si_adc2_raw_labels[i], buf);

        switch (i) {
            case 0:
                /* 455 kHz IF samples - no calibrated engineering unit
                 * yet, show raw code as the engineering value too. */
                snprintf(buf, sizeof(buf), "%u", raw);
                break;
            case 1:
                snprintf(buf, sizeof(buf), "%u", raw);
                break;
            case 2:
                gui_fmt_fixed(buf, sizeof(buf),
                              ((int32_t)g_adc_data.ntc_core_tenth_c + 5) / 10, 0U);
                strncat(buf, "C", sizeof(buf) - strlen(buf) - 1U);
                break;
            case 3:
                gui_fmt_fixed(buf, sizeof(buf),
                              ((int32_t)g_adc_data.ntc_aux_tenth_c + 5) / 10, 0U);
                strncat(buf, "C", sizeof(buf) - strlen(buf) - 1U);
                break;
            default:
                gui_fmt_fixed(buf, sizeof(buf),
                              ((int32_t)g_adc_data.mcu_int_tenth_c + 5) / 10, 0U);
                strncat(buf, "C", sizeof(buf) - strlen(buf) - 1U);
                break;
        }
#else
        /* Simulator: synthesise plausible ADC2 codes */
        static const uint16_t sim_raw[SI_ADC2_ROW_COUNT] = {
            32500, 31200, 20500, 21500, 17000
        };
        snprintf(buf, sizeof(buf), "%u", sim_raw[i]);
        if (si_adc2_raw_labels[i]) lv_label_set_text(si_adc2_raw_labels[i], buf);
        static const char *sim_val[SI_ADC2_ROW_COUNT] = {
            "32500", "31200", "28.5C", "27.0C", "32.4C"
        };
        snprintf(buf, sizeof(buf), "%s", sim_val[i]);
#endif
        if (si_adc2_val_labels[i]) lv_label_set_text(si_adc2_val_labels[i], buf);
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
#ifndef LVGL_SIM
        if (btn_id == 0U) {
            /* "Auto": hand the duty back to the temperature loop.
             * power_set_fan_speed() alone would only write the PWM and
             * leave s_fan_mode alone, so the very next power_update_fan()
             * tick would overwrite it - and passing 0 would stop the fan
             * instead of handing it back. */
            power_set_fan_mode(FAN_MODE_AUTO);
        } else if (btn_id <= 5U) {
            static const uint8_t speeds[] = {0, 20, 40, 60, 80, 100};
            /* Must go through power_set_fan_manual(), not
             * power_set_fan_speed(): only the former latches MANUAL, and
             * without that latch the operator's choice is overwritten by
             * the NTC curve on the next tick. */
            power_set_fan_manual(speeds[btn_id]);
        }
#else
        (void)btn_id;
#endif
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

/* Layout, in row-major order:
 *   row 0: ICOM | Yaesu | Kenwood
 *   row 1: Auto | Radio Baud | CAT Baud
 *   row 2: Cancel (full-width)
 *
 * Radio Baud pops up the spinner bound to g_config.radio_baud_rate
 * (UART4 -> transceiver CAT, defaults to 115200).
 * CAT Baud pops up the spinner bound to g_config.pc_baud_rate (LPUART1
 * -> PC CAT link, defaults to 115200).  Both use the same spinner
 * infrastructure (show_baud_spinbox).
 *
 * Both spinner popups stay alive while their caller is open so the
 * operator can come back and pick another baud or change protocol.
 * The band_comm popup is only closed by a protocol selection, Auto
 * or Cancel - never by the baud sub-popups. */
static const char *band_comm_btns[] = {
    "ICOM", "Yaesu", "Kenwood", "\n",
    "Auto", "Radio Baud", "CAT Baud", "\n",
    "Cancel", "", "", ""
};

/* Spinner button labels - populated at show-band-comm-time based on
 * current language so the operator always sees the right glyph set. */
static const char *band_comm_radio_baud_label_zh = "Radio 波特率";
static const char *band_comm_radio_baud_label_en = "Radio Baud";
static const char *band_comm_cat_baud_label_zh   = "PC 波特率";
static const char *band_comm_cat_baud_label_en   = "CAT Baud";

/* Button matrix layout for band_comm_btns (after LVGL strips "\n"):
 *
 *   idx 0 1 2       (ICOM Yaesu Kenwood)
 *       3 4 5       (Auto  Radio Baud  CAT Baud)
 *       6           (Cancel)
 *
 * LVGL drops the two "\n" placeholders before numbering, hence no gaps
 * in the active-btn ids. */
#define BAND_COMM_BTN_ICOM       0U
#define BAND_COMM_BTN_YAESU      1U
#define BAND_COMM_BTN_KENWOOD    2U
#define BAND_COMM_BTN_AUTO       3U
#define BAND_COMM_BTN_RADIO_BAUD 4U
#define BAND_COMM_BTN_CAT_BAUD   5U
#define BAND_COMM_BTN_CANCEL     6U

static void band_comm_cb(lv_obj_t *obj, lv_event_t e)
{
    if (e == LV_EVENT_CLICKED) {
        uint16_t btn_id = lv_msgbox_get_active_btn(obj);
        if (btn_id == BAND_COMM_BTN_RADIO_BAUD) {
            /* Pop the radio-baud spinner on top of the current popup. */
            show_screen_radio_baud();
            return;
        }
        if (btn_id == BAND_COMM_BTN_CAT_BAUD) {
            /* Pop the PC-CAT-link-baud spinner on top of the current
             * popup. */
            show_screen_cat_baud();
            return;
        }
        /* Anything else (protocol selection or Auto/Cancel) closes the
         * popup immediately as before. */
        lv_obj_del_async(obj);
#ifndef LVGL_SIM
        if (btn_id <= BAND_COMM_BTN_KENWOOD) {
            /* 具体协议 (ICOM=0, Yaesu=1, Kenwood=2) */
            g_config.radio_type = (RadioType)(btn_id + 1);
        } else if (btn_id == BAND_COMM_BTN_AUTO) {
            /* Auto: 让协议层自己识别 */
            g_config.radio_type = RADIO_TYPE_UNKNOWN;
        } else {
            /* Cancel / 其他 - 不动 g_config。*/
        }
        /* 协议变更路径：通知 radio_band + dirty。Cancel / 两个 Baud
         * 按钮不动协议。 */
        if (btn_id <= BAND_COMM_BTN_AUTO) {
            radio_band_set_hint(g_config.radio_type);
            gui_menu_mark_dirty();
        }
#endif
    }
}

static void show_band_comm_settings(void)
{
    band_comm_btns[4] = gui_get_string(STR_AUTO);
    /* btn 5 = "Radio Baud" / btn 6 = "CAT Baud" - keep English labels
     * so the operator can recognise the same entry across both
     * languages; the spinner itself already follows the active language.
     * btn 8 = Cancel (the trailing empty strings are btnmatrix padding). */
    band_comm_btns[5] = (gui_get_lang_index() == 0)
                        ? band_comm_radio_baud_label_zh
                        : band_comm_radio_baud_label_en;
    band_comm_btns[6] = (gui_get_lang_index() == 0)
                        ? band_comm_cat_baud_label_zh
                        : band_comm_cat_baud_label_en;
    band_comm_btns[8] = gui_get_string(STR_CANCEL);

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

/* Number of rows on the protection screen.  Declared before the arrays
 * below so their size is a compile-time constant rather than a count
 * that has to be repeated by hand. */
#define PROT_COUNT 10

static lv_obj_t *prot_sliders[PROT_COUNT];
static lv_obj_t *prot_val_labels[PROT_COUNT];

/* Editable protection thresholds, held as plain floats for readability.
 * prot_from_config() / prot_to_config() translate to the integer units
 * actually stored in ConfigType (SWR is x100, temperature is 0.1 C).
 *
 * prot_val is int32 in tenths of the display unit.  lv_slider works in
 * integers anyway (that is why it was divided by 10 on the way in), and
 * keeping the whole chain integral means the labels can be rendered without
 * any float printf. */
static int32_t prot_val[PROT_COUNT];
static int32_t prot_val_backup[PROT_COUNT];

/* CC6920BSO-50A main-current sensor, used only to SHOW what the COMP2
 * reference voltage means in amps.  The sensor law is fixed by the
 * datasheet (V = 1650 mV + I * 26.4), so this is a display convenience
 * and not a calibration: the stored and applied quantity stays the
 * reference voltage in mV.  Both constants are scaled by 10 to match the
 * tenths used by prot_val / prot_format. */
#define IM_SENSOR_ZERO_MV_TENTH   16500   /* 1650 mV in tenths */
#define IM_SENSOR_MV_PER_A_TENTH   264    /*  26.4 mV/A in tenths */

typedef struct {
    const char *name;
    int32_t vmin, vmax;      /* slider range in tenths of the display unit */
    const char *unit;
    int decimals;
    /* When non-zero, the row also shows the engineering equivalent
     * derived from the stored millivolts.  Only COMP2 has a published
     * transfer function; COMP1 is uncalibrated (TODO-COMP1标定待办.txt).*/
    int derived_from_mv;
} ProtField;

static const ProtField prot_fields[PROT_COUNT] = {
    {"REV2",     1000,  6000, "W", 0, 0},
    {"REV2+SWR",  500,  4000, "W", 0, 0},
    {"SWR",        10,    50, "",  1, 0},
    {"REV1",     1000, 10000, "W", 0, 0},
    {"Main I",    100,   300, "A", 0, 0},
    {"Temp",      600,  1200, "C", 0, 0},
    {"FWD1",     5000, 15000, "W", 0, 0},
    {"FWD2",     5000, 15000, "W", 0, 0},
    /* COMP1: reference voltage only.  No wattage, because the D5/R40
     * peak detector in front of PB2 has not been calibrated yet. */
    {"C1 ref",   2000, COMP1_REF_MV_MAX * 10, "mV", 0, 0},
    /* COMP2: reference voltage, with the derived amp rating alongside. */
    {"C2 ref", 17000, COMP2_REF_MV_MAX * 10, "mV", 0, 1},
};

/* Load the thresholds from g_config (tenths of the display unit). */
static void prot_from_config(void)
{
#ifndef LVGL_SIM
    prot_val[0] = (int32_t)g_config.band_rev2_max_w      * 10;
    prot_val[1] = (int32_t)g_config.band_rev2_swr_w      * 10;
    prot_val[2] = (int32_t)g_config.band_swr_max_x100         / 10;  /* x100 -> tenths */
    prot_val[3] = (int32_t)g_config.band_rev1_max_w      * 10;
    prot_val[4] = (int32_t)g_config.band_im_max_mA          / 100; /* mA -> A */
    prot_val[5] = (int32_t)g_config.band_ntc_max_tenth_c     ;      /* already tenths */
    prot_val[6] = (int32_t)g_config.band_fwd1_max_w      * 10;
    prot_val[7] = (int32_t)g_config.band_fwd2_max_w      * 10;
    prot_val[8] = (int32_t)g_config.comp1_ref_mV         * 10;
    prot_val[9] = (int32_t)g_config.comp2_ref_mV         * 10;
#endif
}

/* Write the thresholds back into g_config. */
static void prot_to_config(void)
{
#ifndef LVGL_SIM
    g_config.band_rev2_max_w      = (uint16_t)(prot_val[0] / 10);
    g_config.band_rev2_swr_w       = (uint16_t)(prot_val[1] / 10);
    g_config.band_swr_max_x100    = (uint16_t)(prot_val[2] * 10);
    g_config.band_rev1_max_w      = (uint16_t)(prot_val[3] / 10);
    g_config.band_im_max_mA       = (uint16_t)(prot_val[4] * 100);  /* A -> mA */
    g_config.band_ntc_max_tenth_c = (uint16_t)prot_val[5];
    g_config.band_fwd1_max_w      = (uint16_t)(prot_val[6] / 10);
    g_config.band_fwd2_max_w      = (uint16_t)(prot_val[7] / 10);
    g_config.comp1_ref_mV         = (uint16_t)(prot_val[8] / 10);
    g_config.comp2_ref_mV         = (uint16_t)(prot_val[9] / 10);
#endif
}

/* Format a threshold value for display.  val is in tenths of the display
 * unit.  Celsius gets its Fahrenheit twin; the COMP2 row additionally shows
 * what the reference voltage works out to in amps, which is the number the
 * operator actually reasons about ("30 A") even though the stored quantity
 * is millivolts. */
static void prot_format(const ProtField *pf, int32_t val_tenth,
                        char *buf, size_t len)
{
    if (pf->unit[0] == 'C') {
        /* F = C * 9 / 5 + 32, kept in tenths so this stays integer. */
        int32_t f_tenth = (val_tenth * 9L) / 5L + 320L;
        gui_fmt_fixed(buf, len, val_tenth, 0U);
        strncat(buf, pf->unit, len - strlen(buf) - 1U);
        strncat(buf, " / ", len - strlen(buf) - 1U);
        {
            char tmp[24];
            gui_fmt_fixed(tmp, sizeof(tmp), f_tenth, 0U);
            strncat(buf, tmp, len - strlen(buf) - 1U);
        }
        strncat(buf, "F", len - strlen(buf) - 1U);
    } else if (pf->derived_from_mv) {
        /* A = (mV - 1650) / 26.4, carried in tenths. */
        int32_t amps_tenth =
            (val_tenth - IM_SENSOR_ZERO_MV_TENTH) / IM_SENSOR_MV_PER_A_TENTH;
        gui_fmt_fixed(buf, len, val_tenth, 0U);
        strncat(buf, "mV/", len - strlen(buf) - 1U);
        {
            char tmp[24];
            gui_fmt_fixed(tmp, sizeof(tmp), amps_tenth, 1U);
            strncat(buf, tmp, len - strlen(buf) - 1U);
        }
        strncat(buf, "A", len - strlen(buf) - 1U);
    } else {
        gui_fmt_fixed(buf, len, val_tenth, (uint8_t)pf->decimals);
        strncat(buf, pf->unit, len - strlen(buf) - 1U);
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

        /* prot_val is in tenths, and so is the slider range, so the two
         * are the same integer - no scaling needed in either direction. */
        prot_val[idx] = (int32_t)lv_slider_get_value(slider);
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
    prot_format(pf, prot_val[idx], buf, sizeof(buf));
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
    /* prot_fields already carries tenths (SWR 1.0..5.0 -> 10..50), which is
     * what gives lv_slider usable resolution in its integer range. */
    lv_slider_set_range(slider, (int16_t)pf->vmin, (int16_t)pf->vmax);
    lv_slider_set_value(slider, (int16_t)prot_val[idx], LV_ANIM_OFF);

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
    lv_obj_set_style_local_bg_opa(prot_screen, LV_OBJ_PART_MAIN,
                                  LV_STATE_DEFAULT, LV_OPA_COVER);
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

    /* Toggle semantics: if the menu is already open, close it instead of
     * stacking another overlay (which would trap the user with multiple
     * stacked layers and is the bug the GUI agent just fixed). */
    if (settings_menu_overlay != NULL) {
        lv_obj_del(settings_menu_overlay);
        settings_menu_overlay = NULL;

        return;
    }

    const char **btns = (gui_get_lang_index() == 0) ? settings_zh : settings_en;

    lv_obj_t *overlay = lv_obj_create(lv_scr_act(), NULL);
    lv_obj_set_size(overlay, HOR_RES - 4 * MARGIN_X, VER_RES - 4 * MARGIN_X);
    lv_obj_align(overlay, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_local_bg_color(overlay, LV_OBJ_PART_MAIN,
                                     LV_STATE_DEFAULT, COLOR_BG);
    lv_obj_set_style_local_bg_opa(overlay, LV_OBJ_PART_MAIN,
                                  LV_STATE_DEFAULT, LV_OPA_COVER);
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

}

void gui_toggle_band_popup(void) { /* TODO: not implemented; lock when used */ }
void gui_focus_prev(void)         { /* TODO: not implemented; lock when used */ }
void gui_focus_next(void)         { /* TODO: not implemented; lock when used */ }
void gui_enter_pressed(void)      { /* TODO: not implemented; lock when used */ }
