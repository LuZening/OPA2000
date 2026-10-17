/**
 * @file    cmsis_os_shim.h
 * @brief   Host-test shim: minimal CMSIS-RTOS2 mutex types/constants
 *          matching the production firmware's cmsis_os.h / freertos_os2.h.
 *
 * @details 目的：让 host 单元测试可以 link 一份「Windows mutex 包装版」的
 *          osMutexNew/Acquire/Release，独立验证 GUI mutex 桩的正确性
 *          （独占语义、超时不死锁、Release 计数、删除等），而不必把
 *          FreeRTOS 整套搬上 host。
 *
 *          这个 shim 仅在 User/Tests/ 编译单元里被引用；firmware 不
 *          看见它，也不应该看见。
 *
 *  字段命名与数值与 LLMC_DAC/STM32CubeMX 默认一致，FreeRTOS/CMSIS
 *  文档化口径：
 *   - osStatus_t  : osOK = 0, osErrorTimeout, osErrorResource, osErrorParameter, osErrorISR
 *   - osMutexId_t : opaque handle
 *   - osWaitForever : 0xFFFFFFFFU
 */
#ifndef CMSIS_OS_SHIM_H
#define CMSIS_OS_SHIM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 *  Status codes
 *============================================================================*/
typedef enum {
    osOK                    =  0,
    osError                 = -1,
    osErrorTimeout          = -2,
    osErrorResource         = -3,
    osErrorParameter        = -4,
    osErrorNoMemory         = -5,
    osErrorISR              = -6,
    osErrorReserved         = 0x7FFFFFFF
} osStatus_t;

/*============================================================================
 *  Opaque mutex handle
 *============================================================================*/
typedef void *osMutexId_t;

/*============================================================================
 *  Wait constants
 *============================================================================*/
#define osWaitForever    0xFFFFFFFFU
#define osNoWait         0U

/*============================================================================
 *  Optional attributes (we ignore the body, but the typedef is referenced
 *  by osMutexNew's signature so the firmware source can be compiled
 *  against the shim without modification).
 *============================================================================*/
typedef struct {
    const char *name;
    uint32_t    attr_bits;
    void       *cb_mem;
    uint32_t    cb_size;
} osMutexAttr_t;

/*============================================================================
 *  API
 *============================================================================*/
osMutexId_t osMutexNew (const osMutexAttr_t *attr);
osStatus_t  osMutexAcquire(osMutexId_t mutex_id, uint32_t timeout);
osStatus_t  osMutexRelease(osMutexId_t mutex_id);
osStatus_t  osMutexDelete (osMutexId_t mutex_id);

/*============================================================================
 *  Test-only diagnostics (not part of CMSIS).  The shim counts the active
 *  acquisitions so test cases can assert that Release was actually called
 *  the right number of times.
 *============================================================================*/
uint32_t cmsis_os_shim_active_acquires(void);
void     cmsis_os_shim_reset_counters(void);

#ifdef __cplusplus
}
#endif

#endif /* CMSIS_OS_SHIM_H */
