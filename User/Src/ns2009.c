/**
 * @file    ns2009.c
 * @brief   NS2009 I2C resistive touch driver (probe only).
 *
 * @details 本文件只剩探测。坐标读取已经搬到 touch.c 的 I2C2 DMA 状态机：
 *          HAL_I2C_MemRxCpltCallback() 是全局唯一的（只认句柄不认从机），
 *          NS2009 与 GT911 共用同一个回调，所以两家的读序列必须写在同
 *          一个状态机里，不能各留一份。
 *
 *          探测只在 main() 的单线程启动阶段跑一次，用阻塞式 HAL 读最简单
 *          且足够——那时还没有 DMA 状态机，也不存在与任务抢总线的问题。
 */

#include "ns2009.h"

#include "stm32h7xx_hal.h"

/* Provided by CubeMX in the firmware build; the host test never
 * links this file (the touch driver is exercised only on target),
 * so a forward declaration is enough. */
extern I2C_HandleTypeDef hi2c2;

/*============================================================================
 *  Probe
 *============================================================================*/
bool ns2009_probe(void)
{
    /* A single-byte read of REG_XH is a guaranteed "registers exist"
     * probe that still performs the address ACK.  If the chip is
     * missing, HAL_I2C_Master_Receive returns HAL_ERROR (NACK/timeout)
     * and the I2C bus stays clean. */
    uint8_t scratch = 0U;
    HAL_StatusTypeDef rc = HAL_I2C_Master_Receive(
        &hi2c2,
        NS2009_ADDR,
        &scratch,
        1U,
        NS2009_I2C_TIMEOUT_MS);
    return (rc == HAL_OK);
}
