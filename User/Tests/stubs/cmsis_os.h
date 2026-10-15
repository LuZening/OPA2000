/**
 * @file    cmsis_os.h  (host-test shim)
 * @brief   Redirects <cmsis_os.h> to the Windows-backed shim for host
 *          unit tests.
 *
 * @details User sources such as config.c include "cmsis_os.h" to reach
 *          the real FreeRTOS / CubeMX-supplied CMSIS-RTOS2 header.  On
 *          the host we only need the mutex subset, so this shim wins
 *          via the test-only include path (User/Tests/stubs first).
 *
 *          Never visible to the firmware build.
 */
#ifndef CMSIS_OS_HOST_TEST_SHIM_H
#define CMSIS_OS_HOST_TEST_SHIM_H

#include "cmsis_os_shim.h"

#endif /* CMSIS_OS_HOST_TEST_SHIM_H */
