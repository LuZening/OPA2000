/**
 * @file    stm32h7xx_hal.h
 * @brief   Host-test shim: redirect <stm32h7xx_hal.h> to the stub.
 *
 * @details The Control sources (#include "stm32h7xx_hal.h") expect the
 *          real STM32 HAL.  For the host test build we put this
 *          directory FIRST in the include path, so this file wins and
 *          simply pulls in the stub definitions instead.
 *
 *          This file is ONLY used by the host test target; the firmware
 *          build never sees it.
 */

#ifndef STM32H7XX_HAL_HOST_SHIM_H
#define STM32H7XX_HAL_HOST_SHIM_H

#include "stm32h7xx_hal_stub.h"

#endif /* STM32H7XX_HAL_HOST_SHIM_H */