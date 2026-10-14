/**
 * @file    main.h  (host-test shim)
 * @brief   Redirects <main.h> to the HAL stub for the host unit tests.
 *
 * @details User sources such as user_adc.h include "main.h" to reach the
 *          real CubeMX-generated header.  On the host we only need the HAL
 *          stub, so this file shadows the generated Core/Inc/main.h when
 *          User/Tests/stubs is first on the include path.
 *
 *          Never visible to the firmware build.
 */

#ifndef MAIN_HOST_TEST_SHIM_H
#define MAIN_HOST_TEST_SHIM_H

#include "stm32h7xx_hal_stub.h"

#endif /* MAIN_HOST_TEST_SHIM_H */