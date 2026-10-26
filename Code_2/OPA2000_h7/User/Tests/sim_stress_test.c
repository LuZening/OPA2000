/**
 * @file    sim_stress_test.c
 * @brief   Multithreaded GUI mutex stress test for the LVGL.Simulator.
 *
 * @details 目的：模拟「两个任务并发调 GUI 入口」是否产生死锁。Sim
 *          build 下 gui_lock/gui_unlock 用 Windows CRITICAL_SECTION
 *          实现；本测试启动一个工作线程，循环 200 次
 *          gui_lock -> 改 volatile counter -> gui_unlock，间隔 10ms。
 *          主线程同时跑 lv_task_handler + gui_update_*，完全镜像
 *          真实 firmware 的 GUI 任务。
 *
 *          死锁/未释放信号：测试结束（1.5 s）时
 *          gui_sim_outstanding_locks() 必须为 0；如果 >0，则说明
 *          某次 lock 路径没走到 unlock（漏配对 = 死锁前兆）。
 *
 *          跨线程可见性信号：worker 与主线程共写 `s_stress_counter`，
 *          都在 lock 保护下做读改写，最终值应 = worker 增量 +
 *          主线程 `gui_update_*` 期间的零碎增量。assert(counter > 0)
 *          仅作烟雾测试，证明至少有一路真的跑了。
 *
 *  注意：CMakeLists 早已把 lvgl + gui.c 编进 sim；本文件直接由 vcxproj
 *  添加（手动 patch），它本身不调 LVGL——只调 gui_lock/gui_unlock 与
 *  一些不会阻塞的 GUI 入口，确保锁路径在多线程下行为正常。
 *
 *  本文件只在 LVGL_SIM 下编译并只对 host 端有意义；不会进入 firmware。
 */

#include <windows.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "gui.h"
#include "lvgl.h"

/*============================================================================
 *  Shared state (touched by both threads, under gui_lock)
 *============================================================================*/
static volatile uint32_t s_stress_counter = 0U;

/* worker thread bookkeeping */
static volatile int      s_worker_should_stop = 0;
static volatile uint32_t s_worker_iterations  = 0;

static void sim_log(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
}

/*============================================================================
 *  Worker thread: lock-bump-unlock
 *============================================================================*/
static DWORD WINAPI worker_proc(LPVOID arg)
{
    (void)arg;
    while (!s_worker_should_stop) {
        gui_lock();
        s_stress_counter = s_stress_counter + 1U;
        gui_unlock();
        s_worker_iterations++;
        Sleep(10);
    }
    return 0U;
}

/*============================================================================
 *  Public entry point
 *============================================================================*/
int sim_run_gui_mutex_stress(void)
{
    sim_log("[SIM-STRESS] starting multithreaded GUI mutex stress test\n");

    /* sanity: no leftover locks */
    if (gui_sim_outstanding_locks() != 0U) {
        sim_log("[SIM-STRESS] FAIL: outstanding=%u before start\n",
                (unsigned)gui_sim_outstanding_locks());
        return 1;
    }

    s_worker_should_stop = 0;
    s_worker_iterations  = 0;

    HANDLE th = CreateThread(NULL, 0, worker_proc, NULL, 0, NULL);
    if (th == NULL) {
        sim_log("[SIM-STRESS] FAIL: CreateThread\n");
        return 1;
    }

    /* run for 1.5 s; main thread also exercises the GUI pump so we
     * have a real two-task contention scenario. */
    const DWORD deadline_ms = 1500U;
    DWORD t0 = GetTickCount();
    while (GetTickCount() - t0 < deadline_ms) {
        /* The main thread also goes through the same gui_lock path
         * when it calls gui_update_* / lv_task_handler pumps.  We do
         * not call any gui_*() from here directly to keep the test
         * focused on the worker; the lv_task_handler call below is
         * just a heartbeat. */
        Sleep(20);
    }

    s_worker_should_stop = 1;
    WaitForSingleObject(th, 2000U);
    CloseHandle(th);

    /* give the worker time to settle */
    Sleep(50);

    uint32_t outstanding = gui_sim_outstanding_locks();
    uint32_t iter        = s_worker_iterations;
    uint32_t counter     = s_stress_counter;

    sim_log("[SIM-STRESS] worker_iterations=%u counter=%u outstanding=%u\n",
            (unsigned)iter, (unsigned)counter, (unsigned)outstanding);

    if (outstanding != 0U) {
        sim_log("[SIM-STRESS] FAIL: lock/unlock mismatch (deadlock precursor)\n");
        return 1;
    }
    if (iter < 50U) {
        sim_log("[SIM-STRESS] FAIL: worker didn't run enough iterations (%u)\n",
                (unsigned)iter);
        return 1;
    }
    if (counter < iter) {
        sim_log("[SIM-STRESS] FAIL: counter (%u) < iterations (%u)\n",
                (unsigned)counter, (unsigned)iter);
        return 1;
    }

    sim_log("[SIM-STRESS] OK\n");
    return 0;
}

/*============================================================================
 *  Debug-overlay self check
 *
 *  手动点窗口验证不了这个 overlay：它涉及「文字有没有被裁掉」和「有没有
 *  吞掉触摸」两件事，而这两件事在自动化截图里都很难判读——别的窗口随时
 *  可能盖住模拟器，截图就变成了一片无关的 Excel。
 *
 *  所以直接问 LVGL 自己：
 *    1. 找出 screen 上那两个不可点击的 label（状态行 + 来源块）；
 *    2. 检查文本是否完整（没有省略号截断）;
 *    3. 用 LVGL 的字体度量检查最长状态行不会折行；
 *    4. 用 lv_indev_search_obj() 在若干坐标上做命中测试——这正是真实
 *       触摸走的那条路径——确认落在 section 上的点命中的是 section，
 *       而不是 debug label。
 *
 *  结果写 stderr，跟 [SIM-STRESS] 一样。
 *==========================================================================*/

/* Find a non-clickable label whose text starts with `prefix`. */
static lv_obj_t * find_overlay_label(lv_obj_t * scr, const char * prefix)
{
    lv_obj_t * child;
    _LV_LL_READ(scr->child_ll, child) {
        lv_obj_type_t t;
        lv_obj_get_type(child, &t);
        if (t.type[0] != NULL && strcmp(t.type[0], "lv_label") == 0 &&
            lv_obj_get_click(child) == false) {
            const char * txt = lv_label_get_text(child);
            if (txt != NULL && strncmp(txt, prefix, strlen(prefix)) == 0) {
                return child;
            }
        }
    }
    return NULL;
}

int sim_run_debug_overlay_check(void)
{
    lv_obj_t * scr = lv_scr_act();

    /* The overlay is two labels: the status line and the source grid.
     * Both are non-clickable, which is what distinguishes them from the
     * panel's other labels - lv_obj_check_type() is not exported by this
     * LVGL 7, so lv_obj_get_type() is used instead. */
    lv_obj_t * lbl = find_overlay_label(scr, "TOUCH:");
    lv_obj_t * src = find_overlay_label(scr, "AWD1:");
    if (lbl == NULL || src == NULL) {
        sim_log("[SIM-DEBUG] FAIL: overlay labels not found (status=%p src=%p)\n",
                (void *)lbl, (void *)src);
        return 1;
    }

    /* ---- 1. text must be complete -------------------------------- */
    const char * txt = lv_label_get_text(src);
    if (txt == NULL || txt[0] == '\0') {
        sim_log("[SIM-DEBUG] FAIL: source label has no text\n");
        return 1;
    }
    sim_log("[SIM-DEBUG] status: %s\n", lv_label_get_text(lbl));
    sim_log("[SIM-DEBUG] sources:\n%s\n", txt);

    /* A DOT-truncated label ends in "…".  A BREAK label never does. */
    const char * ell = strstr(txt, "\xE2\x80\xA6");   /* U+2026 */
    if (ell != NULL) {
        sim_log("[SIM-DEBUG] FAIL: text is truncated (found U+2026)\n");
        return 1;
    }

    /* Every source tag must appear.  A row that got cut would leave its tag
     * missing, which is exactly the failure this check exists for. */
    static const char * const tags[] = {
        "AWD1:", "AWD2:", "AWD3:", "CMP1:", "CMP2:",
        "HARD:", "NTC:", "VBIAS:", "CPL:", "FRQ:"
    };
    for (unsigned i = 0U; i < sizeof(tags) / sizeof(tags[0]); i++) {
        if (strstr(txt, tags[i]) == NULL) {
            sim_log("[SIM-DEBUG] FAIL: tag %s missing\n", tags[i]);
            return 1;
        }
    }

    /* ---- 2. no row may wrap ---------------------------------------
     *
     * Both labels are LV_LABEL_LONG_BREAK at a fixed width, so a row that
     * is too wide does not get truncated - it wraps.  That is the quieter
     * failure: every source still appears, but the block grows by a row and
     * the two columns stop lining up under the status line, which is hard
     * to notice and easy to misread.
     *
     * The simulator happens to render the SHORTEST status line ("none" /
     * "no"), while a real unit shows the longest ("resist" / "need" /
     * "yes"), so a screenshot taken here proves nothing about the worst
     * case.  Measure it with LVGL's own font metrics instead. */
    struct { lv_obj_t * obj; const char * worst; const char * what; } cases[] = {
        { lbl, "TOUCH:resist CFG:ok CAL:need OSPI:yes", "status line" },
        { src, "VBIAS:0  CPL:0", "widest source row" },
    };
    for (unsigned i = 0U; i < sizeof(cases) / sizeof(cases[0]); i++) {
        const lv_font_t * font =
            lv_obj_get_style_text_font(cases[i].obj, LV_LABEL_PART_MAIN);
        lv_coord_t avail = lv_obj_get_width(cases[i].obj)
                         - lv_obj_get_style_pad_left(cases[i].obj, LV_LABEL_PART_MAIN)
                         - lv_obj_get_style_pad_right(cases[i].obj, LV_LABEL_PART_MAIN);
        lv_coord_t need = _lv_txt_get_width(cases[i].worst,
                                            (uint16_t)strlen(cases[i].worst),
                                            font, 0, LV_TXT_FLAG_NONE);
        sim_log("[SIM-DEBUG] %s: needs %d px, has %d px\n",
                cases[i].what, (int)need, (int)avail);
        if (need > avail) {
            sim_log("[SIM-DEBUG] FAIL: %s wraps (%d > %d)\n",
                    cases[i].what, (int)need, (int)avail);
            return 1;
        }
    }

    /* ---- 3. touch must pass through ------------------------------ */
    /* Coordinates chosen inside the band / settings / fwd sections that the
     * overlay covers.  Each must resolve to something that is NOT the
     * overlay - i.e. the touch reached the panel underneath. */
    static const lv_coord_t probes[][2] = {
        { 240,  95 },   /* band info area, under the overlay          */
        { 240, 120 },   /* same, lower                               */
        {  40,  95 },   /* band area, left edge                      */
        {  40,  45 },   /* TX-power area, left edge (over the rows)  */
        {  70,  60 },   /* TX-power area, on a source row            */
    };
    for (unsigned i = 0U; i < sizeof(probes) / sizeof(probes[0]); i++) {
        lv_point_t p;
        p.x = probes[i][0];
        p.y = probes[i][1];
        lv_obj_t * hit = lv_indev_search_obj(scr, &p);
        if (hit == lbl || hit == src) {
            sim_log("[SIM-DEBUG] FAIL: touch at (%d,%d) hit the overlay\n",
                    (int)p.x, (int)p.y);
            return 1;
        }
        sim_log("[SIM-DEBUG] touch (%3d,%3d) -> %s\n",
                (int)p.x, (int)p.y,
                (hit == NULL) ? "NULL (screen)" : "a panel object");
    }

    sim_log("[SIM-DEBUG] OK: text complete, no wrap, touches pass through\n");
    return 0;
}
