/**
 * @file    config.h
 * @brief   Persistent configuration structure for OPA2000 H7.
 *
 * @details The configuration is stored as a binary blob in FatFS
 *          (/fs/config.bin) and survives reboot.  All thresholds
 *          marked [config] in OPA2000_H7_固件设计要求.txt live
 *          here.
 *
 *          The Control agent owns the layout and CRC; GUI / Networking
 *          agents read it through `config_get()` accessor only.
 *
 * @version 1.0
 * @date    2026-10
 * @author  Control Agent (wt-control branch)
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONFIG_MAGIC            0x4F504131U   /*!< "OPA1" */
#define CONFIG_VERSION          1U
#define CONFIG_FILE_PATH        "/fs/config.bin"
#define CONFIG_MAX_BANDS        8U             /*!< upper bound, 5 used now */

/* ===========================================================================
 *  1. Protection thresholds (per-band)
 * ===========================================================================*/
typedef struct {
    uint16_t rf_in_max_w;          /*!< RF input power threshold for AWD1 (W) */
    uint16_t rev2_max_w;          /*!< reflected 2 software threshold (W)     */
    uint16_t rev2_swr_w;           /*!< REV2 threshold for SWR check (W)       */
    uint16_t swr_max_x100;         /*!< SWR max, x100 (e.g. 200 = 2.0)         */
    uint16_t fwd1_max_w;           /*!< forward 1 software limit (W)            */
    uint16_t fwd2_max_w;           /*!< forward 2 software limit (W)            */
    uint16_t ntc_max_tenth_c;      /*!< core NTC max temperature (0.1C units)    */
} band_protection_t;

/* ===========================================================================
 *  2. Coupler calibration (linear fit y = a*x + b)
 * ===========================================================================*/
typedef struct {
    int32_t fwd1_a_q16;            /*!< Q16.16 fixed-point slope  */
    int32_t fwd1_b_mW;             /*!< intercept in mW            */
    int32_t fwd2_a_q16;
    int32_t fwd2_b_mW;
    int32_t rev1_a_q16;
    int32_t rev1_b_mW;
    int32_t rev2_a_q16;
    int32_t rev2_b_mW;
    int16_t rev1_awd3_threshold_mW; /*!< AWD3 hardware threshold     */
    int16_t reserved[3];
} coupler_calib_t;

/* ===========================================================================
 *  3. Touch / display calibration
 * ===========================================================================*/
typedef struct {
    int16_t x_a_q10;
    int16_t x_b_q10;
    int16_t y_a_q10;
    int16_t y_b_q10;
    uint8_t calibrated;
    uint8_t reserved[3];
} touch_calib_t;

/* ===========================================================================
 *  4. Radio link parameters
 * ===========================================================================*/
typedef enum {
    RADIO_PROTO_ICOM_CIV = 0,
    RADIO_PROTO_YAESU,
    RADIO_PROTO_KENWOOD,
    RADIO_PROTO_AUTO,        /*!< auto-detect */
} radio_protocol_t;

typedef struct {
    radio_protocol_t proto;
    uint32_t baud;
    uint16_t rx_timeout_ms;
    uint8_t  reserved[2];
} radio_link_t;

/* ===========================================================================
 *  5. Persistent configuration blob (lives in FatFS /fs/config.bin)
 * ===========================================================================*/
typedef struct __attribute__((packed)) {
    uint32_t           magic;                 /*!< CONFIG_MAGIC */
    uint8_t            version;               /*!< CONFIG_VERSION */
    uint8_t            reserved0[3];
    uint32_t           crc32;                 /*!< CRC32 over all fields below */

    /* Communication */
    uint32_t           pc_baud;               /*!< LPUART1 / USB CDC baudrate */
    radio_link_t       radio;

    /* Per-band protection */
    band_protection_t  band_prot[CONFIG_MAX_BANDS];

    /* Calibration */
    coupler_calib_t    coupler;
    touch_calib_t      touch;
    uint16_t           im_zero_offset_adc;    /*!< CC6920BSO-50A zero offset */
    uint8_t            im_calibrated;
    uint8_t            reserved1;

    /* Misc */
    uint8_t            language;              /*!< 0=EN, 1=ZH */
    uint8_t            buzzer_enable;
    uint8_t            reserved2[6];
} config_t;

/* ===========================================================================
 *  6. Access API
 * ===========================================================================*/
opa_result_t config_load  (void);                /*!< load from FatFS to RAM */
opa_result_t config_save  (void);                /*!< persist RAM to FatFS */
opa_result_t config_init_default(void);         /*!< populate RAM with safe defaults */

const config_t *config_get(void);               /*!< read-only accessor */
config_t       *config_get_mutable(void);       /*!< read-write accessor (control only) */

/* Field helpers */
uint16_t config_get_rf_in_max_w(uint8_t band);
void     config_set_rf_in_max_w(uint8_t band, uint16_t w);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H */