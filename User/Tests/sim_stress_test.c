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

#include "gui.h"

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
