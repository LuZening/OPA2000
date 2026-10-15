/**
 * @file    test_cmsis_os_shim.c
 * @brief   Host-side tests for the Windows-mutex-backed CMSIS osMutex shim.
 *
 * @details 目的（来自设计要求）：在 host 上验证 GUI mutex 桩的语义，
 *          确保 (a) 独占、(b) acquire 不会无限期死锁、(c) Release 计数
 *          与 Acquire 严格匹配、(d) 错误参数返回合适的状态码。
 *          Windows mutex (CreateMutexW / WaitForSingleObject) 与 CMSIS
 *          普通 mutex 的语义足够接近，可以直接用来跑。
 *
 *          这些测试不依赖 lvgl，不依赖 FreeRTOS；只 link cmsis_os_shim。
 *          CMake 端把这一组测试编进 host_tests。
 *
 *  死锁/超时策略：
 *    涉及「锁已被另一线程持有」的用例，acquire 全部传 **有限 timeout**
 *    (例如 50ms)，并断言返回 osErrorTimeout。这样如果桩 bug 写成
 *    INFINITE 等待也不会卡住 host 测试；超时本身也是「不死锁」的反证。
 */

#include "test_harness.h"
#include "cmsis_os_shim.h"

#include <windows.h>
#include <process.h>     /* _beginthreadex */
#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 *  Test 1: 基础生命周期
 *  create -> acquire -> release -> acquire 应当全部成功
 *============================================================================*/
TEST_CASE(shim_basic_lifecycle)
{
    cmsis_os_shim_reset_counters();
    osMutexId_t m = osMutexNew(NULL);
    ASSERT_NOT_NULL(m);
    ASSERT_EQ(cmsis_os_shim_active_acquires(), 0U);

    ASSERT_EQ(osMutexAcquire(m, osWaitForever), osOK);
    ASSERT_EQ(cmsis_os_shim_active_acquires(), 1U);
    ASSERT_EQ(osMutexRelease(m), osOK);
    ASSERT_EQ(cmsis_os_shim_active_acquires(), 0U);

    /* second round on the same handle */
    ASSERT_EQ(osMutexAcquire(m, 100U), osOK);
    ASSERT_EQ(osMutexRelease(m), osOK);
    ASSERT_EQ(osMutexDelete(m), osOK);
}

/*============================================================================
 *  Test 2: 参数错误
 *  NULL handle 必须返回 osErrorParameter
 *============================================================================*/
TEST_CASE(shim_null_handle)
{
    ASSERT_EQ(osMutexAcquire(NULL, osWaitForever), osErrorParameter);
    ASSERT_EQ(osMutexRelease(NULL), osErrorParameter);
    ASSERT_EQ(osMutexDelete(NULL), osErrorParameter);
}

/*============================================================================
 *  Test 3: 超时返回 osErrorTimeout 而不是死锁
 *  关键死锁守卫：桩绝对不能 INFINITE 等待另一个线程占用的锁。
 *============================================================================*/

/* 共享状态：让子线程拿到锁后短暂持锁 */
static volatile int    s_helper_got_lock   = 0;
static volatile int    s_helper_release    = 0;
static osMutexId_t     s_helper_mutex       = NULL;

static unsigned __stdcall helper_thread(void *arg)
{
    (void)arg;
    osMutexAcquire(s_helper_mutex, osWaitForever);
    s_helper_got_lock = 1;
    while (!s_helper_release) {
        Sleep(1);
    }
    osMutexRelease(s_helper_mutex);
    return 0U;
}

TEST_CASE(shim_acquire_timeout_does_not_deadlock)
{
    cmsis_os_shim_reset_counters();
    s_helper_mutex = osMutexNew(NULL);
    ASSERT_NOT_NULL(s_helper_mutex);

    s_helper_got_lock = 0;
    s_helper_release  = 0;

    HANDLE th = (HANDLE)_beginthreadex(NULL, 0, helper_thread, NULL, 0, NULL);
    ASSERT_NE(th, (HANDLE)NULL);

    /* wait until the helper owns the lock */
    for (int i = 0; i < 200 && !s_helper_got_lock; i++) {
        Sleep(1);
    }
    ASSERT_TRUE(s_helper_got_lock);

    /* Now try to acquire from main with a 50ms timeout.  Must return
     * osErrorTimeout, NOT block forever.  This is the anti-deadlock
     * guarantee. */
    uint32_t t0 = GetTickCount();
    osStatus_t s = osMutexAcquire(s_helper_mutex, 50U);
    uint32_t dt = GetTickCount() - t0;
    ASSERT_EQ(s, osErrorTimeout);
    /* must have actually waited close to 50ms; loose bound to avoid CI flake */
    ASSERT_TRUE(dt >= 30U);
    ASSERT_TRUE(dt < 2000U);

    /* let helper release, then we should be able to grab it */
    s_helper_release = 1;
    WaitForSingleObject(th, INFINITE);
    CloseHandle(th);

    ASSERT_EQ(osMutexAcquire(s_helper_mutex, 200U), osOK);
    ASSERT_EQ(osMutexRelease(s_helper_mutex), osOK);
    ASSERT_EQ(osMutexDelete(s_helper_mutex), osOK);
}

/*============================================================================
 *  Test 4: Release 计数与 Acquire 严格匹配
 *  Acquire 两次后必须 Release 两次才能让 acquire_count 归零；
 *  Release 一个未持有的 mutex 必须返回 osErrorResource
 *============================================================================*/
TEST_CASE(shim_release_counting)
{
    cmsis_os_shim_reset_counters();
    osMutexId_t m = osMutexNew(NULL);
    ASSERT_NOT_NULL(m);

    ASSERT_EQ(osMutexAcquire(m, osWaitForever), osOK);
    ASSERT_EQ(osMutexAcquire(m, 100U), osOK);
    ASSERT_EQ(cmsis_os_shim_active_acquires(), 2U);

    /* first release succeeds */
    ASSERT_EQ(osMutexRelease(m), osOK);
    ASSERT_EQ(cmsis_os_shim_active_acquires(), 1U);

    /* second release succeeds */
    ASSERT_EQ(osMutexRelease(m), osOK);
    ASSERT_EQ(cmsis_os_shim_active_acquires(), 0U);

    /* now mutex is free -> another acquire works (counter resets) */
    ASSERT_EQ(osMutexAcquire(m, 100U), osOK);
    ASSERT_EQ(osMutexRelease(m), osOK);

    /* over-release must report an error, not crash */
    ASSERT_EQ(osMutexRelease(m), osErrorResource);
    ASSERT_EQ(cmsis_os_shim_active_acquires(), 0U);

    ASSERT_EQ(osMutexDelete(m), osOK);
}

/*============================================================================
 *  Test 5: 模拟 GUI 的 "锁-改-释放" 闭环
 *  这就是把 gui_lock/gui_unlock 抽象成 acquire/release 后的真实形态：
 *  在 1000 次「取锁 -> 改一个全局计数器 -> 释放」循环里，主线程永远
 *  不能让计数器被破坏；本测试断言计数稳定上升 + 每次循环结束后
 *  active_acquires 归零。如果桩在某些路径上忘了 release，计数器
 *  会持续涨。
 *============================================================================*/
TEST_CASE(shim_gui_pattern_does_not_leak)
{
    cmsis_os_shim_reset_counters();
    osMutexId_t m = osMutexNew(NULL);
    ASSERT_NOT_NULL(m);

    volatile uint32_t counter = 0U;
    for (int i = 0; i < 1000; i++) {
        ASSERT_EQ(osMutexAcquire(m, 100U), osOK);
        /* "GUI 修改"：写一次 */
        counter = counter + 1U;
        ASSERT_EQ(osMutexRelease(m), osOK);
        /* 每次循环结束 active_acquires 必须回到 0，证实 lock/unlock 严格成对 */
        ASSERT_EQ(cmsis_os_shim_active_acquires(), 0U);
    }
    ASSERT_EQ(counter, 1000U);
    ASSERT_EQ(osMutexDelete(m), osOK);
}

/*============================================================================
 *  Test 6: osWaitForever == INFINITE 真的能无限等
 *  这是「死锁不应当发生」的另一方面：桩本身的 INFINITE 路径必须能
 *  正常唤醒。我们让子线程短暂持锁后释放；主线程 INFINITE 等，
 *  应当在 1 秒内成功拿到锁。如果桩的 INFINITE 实现错成「0 立即返
 *  回 osErrorTimeout」，这个测试会立即失败。
 *============================================================================*/
static volatile int s_release2 = 0;
static osMutexId_t  s_m2       = NULL;

static unsigned __stdcall holder_thread(void *arg)
{
    (void)arg;
    osMutexAcquire(s_m2, osWaitForever);
    Sleep(50);
    s_release2 = 1;
    osMutexRelease(s_m2);
    return 0U;
}

TEST_CASE(shim_infinite_wait_wakes_up)
{
    cmsis_os_shim_reset_counters();
    s_m2 = osMutexNew(NULL);
    ASSERT_NOT_NULL(s_m2);

    s_release2 = 0;
    HANDLE th = (HANDLE)_beginthreadex(NULL, 0, holder_thread, NULL, 0, NULL);
    ASSERT_NE(th, (HANDLE)NULL);

    uint32_t t0 = GetTickCount();
    osStatus_t s = osMutexAcquire(s_m2, osWaitForever);
    uint32_t dt = GetTickCount() - t0;

    ASSERT_EQ(s, osOK);
    ASSERT_TRUE(s_release2 == 1);          /* helper must have run */
    ASSERT_TRUE(dt >= 30U);                /* waited until helper released */
    ASSERT_TRUE(dt < 5000U);               /* but did not hang */

    ASSERT_EQ(osMutexRelease(s_m2), osOK);
    WaitForSingleObject(th, INFINITE);
    CloseHandle(th);
    ASSERT_EQ(osMutexDelete(s_m2), osOK);
}

/*============================================================================
 *  Suite registration
 *============================================================================*/
const test_entry_t g_suite_cmsis_os_shim[] = {
    TEST_ENTRY(shim_basic_lifecycle),
    TEST_ENTRY(shim_null_handle),
    TEST_ENTRY(shim_acquire_timeout_does_not_deadlock),
    TEST_ENTRY(shim_release_counting),
    TEST_ENTRY(shim_gui_pattern_does_not_leak),
    TEST_ENTRY(shim_infinite_wait_wakes_up),
    TEST_SUITE_END,
};
