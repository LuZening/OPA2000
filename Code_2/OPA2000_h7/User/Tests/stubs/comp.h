/**
 * @file    comp.h
 * @brief   Host-test stand-in for the CubeMX-generated Core/Inc/comp.h.
 *
 * user_adc.c includes "comp.h" to reach hcomp1 / hcomp2, which it starts
 * once the power-on stabilisation gate arms.  The stub include directory
 * comes first in the include path, so this file wins over the real one and
 * the host build resolves the handles without dragging in the CubeMX
 * peripheral header.
 *
 * The COMP_TypeDef / COMP_HandleTypeDef / HAL_COMP_Start_IT declarations
 * themselves live in stm32h7xx_hal_stub.h, which every Control source
 * already reaches through stm32h7xx_hal.h.
 */

#ifndef STUB_COMP_H
#define STUB_COMP_H

#include "stm32h7xx_hal_stub.h"

#endif /* STUB_COMP_H */
