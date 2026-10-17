/**
 * @file    error.h
 * @brief   OPA2000 firmware-wide error codes and result type.
 *
 * @details All three agents (Control / GUI / Networking) use these
 *          error codes when crossing module boundaries (function
 *          return values, queue messages, log lines, UI dialogs).
 *
 *          Negative values are errors, zero is success, positive
 *          values are non-error status flags.
 *
 * @version 1.0
 * @date    2026-10
 * @author  Control Agent (wt-control branch)
 */

#ifndef ERROR_H
#define ERROR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===========================================================================
 *  1. Generic result type
 * ===========================================================================*/
typedef int32_t opa_result_t;

/* Standard return values */
#define OPA_OK                  0
#define OPA_ERR                (-1)
#define OPA_ERR_BUSY           (-2)
#define OPA_ERR_TIMEOUT        (-3)
#define OPA_ERR_INVALID_ARG    (-4)
#define OPA_ERR_NOT_SUPPORTED  (-5)
#define OPA_ERR_HW_FAULT       (-6)
#define OPA_ERR_NOT_READY      (-7)
#define OPA_ERR_CRC            (-8)
#define OPA_ERR_NO_MEM         (-9)
#define OPA_ERR_OVERFLOW       (-10)

/* ===========================================================================
 *  2. ADC / power conversion specific errors
 * ===========================================================================*/
#define OPA_ERR_ADC_CALIBRATION     (-100)  /*!< IM zero-offset failed        */
#define OPA_ERR_ADC_RANGE           (-101)  /*!< raw out of expected range    */
#define OPA_ERR_ADC_AWD_CFG         (-102)  /*!< AWD threshold invalid        */

/* ===========================================================================
 *  3. Protection / fault codes
 * ===========================================================================*/
#define OPA_FAULT_REV1_OVER         (-200)  /*!< reflected 1 above AWD3 limit */
#define OPA_FAULT_REV2_OVER         (-201)  /*!< reflected 2 above software limit */
#define OPA_FAULT_FWD1_OVER         (-202)  /*!< forward 1 above 1100W          */
#define OPA_FAULT_FWD2_OVER         (-203)  /*!< forward 2 above 1050W          */
#define OPA_FAULT_IM_OVER           (-204)  /*!< main current above 30A         */
#define OPA_FAULT_VBIAS_OVER        (-205)  /*!< PA bias above limit            */
#define OPA_FAULT_NTC_OVER          (-206)  /*!< core temperature above 90C    */
#define OPA_FAULT_RFIN_OVER         (-207)  /*!< RF input above band-threshold */
#define OPA_FAULT_HARDPWR           (-208)  /*!< supply fault (HARDPWR_ERR)    */
#define OPA_FAULT_SWR               (-209)  /*!< SWR > 2.0 with REV > 200W     */

/* ===========================================================================
 *  4. Communication / config errors
 * ===========================================================================*/
#define OPA_ERR_PROTO              (-300)  /*!< protocol parse error          */
#define OPA_ERR_CHECKSUM           (-301)  /*!< checksum mismatch             */
#define OPA_ERR_RADIO_TIMEOUT      (-302)  /*!< no radio frame in time window  */
#define OPA_ERR_CONFIG_INVALID     (-303)  /*!< config file magic / CRC failed */

#ifdef __cplusplus
}
#endif

#endif /* ERROR_H */