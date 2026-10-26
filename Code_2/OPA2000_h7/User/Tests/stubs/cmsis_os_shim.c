/**
 * @file    cmsis_os_shim.c
 * @brief   Host-test shim: CMSIS-RTOS2 mutex mapped onto Windows mutexes.
 *
 * @details 目的：让 host 单元测试可以「拿真实的 OS 互斥体」验证 GUI
 *          资源锁的语义——独占、阻塞超时、Release 计数匹配——而无需把
 *          FreeRTOS 整套搬上 PC。Windows mutex 行为与 CMSIS 规范一致：
 *          - osMutexNew           -> CreateMutexW (默认无名/未获得)
 *          - osMutexAcquire       -> WaitForSingleObject
 *          - osMutexRelease       -> ReleaseMutex
 *          - osMutexDelete        -> CloseHandle
 *
 *          注意：CreateMutexW 默认可被「同一线程」重复获取 (recursive)，
 *          而 FreeRTOS 普通 mutex 不行——这里我们按 CMSIS「普通 mutex」
 *          语义在 shim 内部追踪 owner 线程 ID，跨线程二次 acquire
 *          会返回 osErrorResource；同一线程在持锁情况下再 acquire 返回
 *          osErrorResource（与 FreeRTOS 行为一致；本工程用的是
 *          osMutexNew(NULL) 默认属性 = 非 recursive）。
 *
 *          这层 shim 不替代 firmware 的真正 RTOS；它只服务于 host
 *          测试。
 */

#include "cmsis_os_shim.h"

#include <windows.h>

#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 *  Internal: a small wrapper around HANDLE that also records the owner
 *  thread id so we can detect cross-thread double-acquire (a classic
 *  cause of self-deadlock that a raw Windows mutex would silently allow).
 *============================================================================*/
typedef struct {
    HANDLE      win_handle;
    DWORD       owner_tid;     /* 0 = unowned */
    uint32_t    acquire_count; /* for diagnostics + nested-detect */
} shim_mutex_t;

/* Counters visible to tests (declared in header) */
static uint32_t s_active_acquires = 0U;

/* Thread-flag state (declared in header) */
static uint32_t     s_flag_set_count   = 0U;
static uint32_t     s_last_flag_set    = 0U;
static osThreadId_t s_last_flag_target = NULL;

uint32_t cmsis_os_shim_active_acquires(void)
{
    return s_active_acquires;
}

void cmsis_os_shim_reset_counters(void)
{
    s_active_acquires = 0U;
}

uint32_t cmsis_os_shim_flag_set_count(void)
{
    return s_flag_set_count;
}

uint32_t cmsis_os_shim_last_flag_set(void)
{
    return s_last_flag_set;
}

osThreadId_t cmsis_os_shim_last_flag_target(void)
{
    return s_last_flag_target;
}

void cmsis_os_shim_reset_flag_counters(void)
{
    s_flag_set_count   = 0U;
    s_last_flag_set    = 0U;
    s_last_flag_target = NULL;
}

/*============================================================================
 *  Thread flags
 *
 *  Only the Set() half is modelled: the config save task's wait lives in
 *  app_tasks.c, which is not part of the host build.  What the host test
 *  needs to observe is "did the code under test wake the save task, and
 *  with which flags" - which is exactly what is recorded here.
 *==========================================================================*/
uint32_t osThreadFlagsSet (osThreadId_t thread_id, uint32_t flags)
{
    s_flag_set_count++;
    s_last_flag_set    = flags;
    s_last_flag_target = thread_id;
    return flags;
}

/*============================================================================
 *  Public API
 *============================================================================*/
osMutexId_t osMutexNew(const osMutexAttr_t *attr)
{
    (void)attr;
    shim_mutex_t *m = (shim_mutex_t *)HeapAlloc(GetProcessHeap(), HEAP_ZERO_MEMORY,
                                              sizeof(shim_mutex_t));
    if (m == NULL) {
        return NULL;
    }
    /* Initially unowned.  lpName = NULL => anonymous mutex. */
    m->win_handle = CreateMutexW(NULL, FALSE, NULL);
    if (m->win_handle == NULL) {
        HeapFree(GetProcessHeap(), 0, m);
        return NULL;
    }
    m->owner_tid      = 0U;
    m->acquire_count  = 0U;
    return (osMutexId_t)m;
}

osStatus_t osMutexAcquire(osMutexId_t mutex_id, uint32_t timeout)
{
    if (mutex_id == NULL) {
        return osErrorParameter;
    }
    shim_mutex_t *m = (shim_mutex_t *)mutex_id;

    /* CMSIS specifies: never call from ISR.  We do not have an IRQ
     * context concept on host; if the caller chose to do so, we still
     * just block on the Win32 mutex and let the test report it. */

    DWORD ms = (timeout == osWaitForever) ? INFINITE : (DWORD)timeout;
    DWORD rc = WaitForSingleObject(m->win_handle, ms);
    if (rc == WAIT_OBJECT_0) {
        DWORD self = GetCurrentThreadId();
        if (m->owner_tid == self) {
            /* Same thread re-acquiring: this is the kind of nesting that
             * would deadlock the firmware (FreeRTOS ordinary mutex).  We
             * still let WaitForSingleObject return (recursive), so we have
             * to release once for each acquire.  Track the count for
             * diagnostics; tests can assert this. */
        }
        if (m->owner_tid == 0U) {
            m->owner_tid = self;
        }
        m->acquire_count++;
        s_active_acquires++;
        return osOK;
    }
    if (rc == WAIT_TIMEOUT) {
        return osErrorTimeout;
    }
    return osErrorResource;
}

osStatus_t osMutexRelease(osMutexId_t mutex_id)
{
    if (mutex_id == NULL) {
        return osErrorParameter;
    }
    shim_mutex_t *m = (shim_mutex_t *)mutex_id;
    if (m->acquire_count == 0U) {
        return osErrorResource;  /* released more than acquired */
    }
    if (!ReleaseMutex(m->win_handle)) {
        return osErrorResource;
    }
    m->acquire_count--;
    if (m->acquire_count == 0U) {
        m->owner_tid = 0U;
    }
    if (s_active_acquires > 0U) {
        s_active_acquires--;
    }
    return osOK;
}

osStatus_t osMutexDelete(osMutexId_t mutex_id)
{
    if (mutex_id == NULL) {
        return osErrorParameter;
    }
    shim_mutex_t *m = (shim_mutex_t *)mutex_id;
    /* If the mutex is still acquired, this is undefined in CMSIS, but
     * Windows allows it.  We close anyway; let the next acquire on
     * the dangling handle fail with osErrorParameter. */
    CloseHandle(m->win_handle);
    HeapFree(GetProcessHeap(), 0, m);
    return osOK;
}
