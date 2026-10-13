/**
 * @file comm.h
 * @brief Communication Protocol Header (USB CDC, LPUART1, Radio)
 */

#ifndef COMM_H
#define COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "opa2000.h"

/*============================================================================
 * Communication Configuration
 *============================================================================*/

#define COMM_RX_QUEUE_SIZE     256
#define COMM_TX_QUEUE_SIZE     256

/* Command buffer size */
#define COMM_CMD_MAX_LEN       64

/*============================================================================
 * Command Types
 *============================================================================*/

typedef enum {
    CMD_NONE = 0,
    CMD_GET_STATUS,
    CMD_SET_FREQ,
    CMD_GET_POWER,
    CMD_GET_SWR,
    CMD_GET_VOLTAGE,
    CMD_GET_CURRENT,
    CMD_GET_TEMP,
    CMD_GET_BAND,
    CMD_GET_FREQ,
    CMD_ALERT_OFF,
    CMD_SET_CONFIG,
    CMD_GET_CONFIG,
    CMD_SAVE_CONFIG,
    CMD_RESET_CONFIG,
    CMD_CALIBRATE,
    CMD_SET_LANGUAGE,
    CMD_GET_LANGUAGE,
    CMD_CALIBRATE_COUPLER,
    CMD_UNKNOWN
} CommandType;

/*============================================================================
 * Coupler Calibration State Machine
 *============================================================================*/

typedef enum {
    COUPLER_CAL_IDLE = 0,           /* Not in calibration mode */
    COUPLER_CAL_START,              /* Calibration started, waiting for OK */
    COUPLER_CAL_STAGE1_5W_FWD,      /* Stage 1: 5W forward power, waiting for OK */
    COUPLER_CAL_STAGE1_5W_REF,      /* Stage 1: 5W reflected power, waiting for OK */
    COUPLER_CAL_STAGE2_20W_FWD,     /* Stage 2: 20W forward power, waiting for OK */
    COUPLER_CAL_STAGE2_20W_REF,     /* Stage 2: 20W reflected power, waiting for OK */
    COUPLER_CAL_COMPLETE,           /* Calibration complete */
    COUPLER_CAL_ABORTED             /* Calibration aborted */
} CouplerCalState;

typedef enum {
    COUPLER_1 = 0,    /* Coupler 1: Between PA and LPF */
    COUPLER_2,        /* Coupler 2: Between LPF and Antenna */
    COUPLER_COUNT
} CouplerId;

/* Calibration data storage */
typedef struct {
    float stage1_fwd_adc;      /* ADC reading at 5W forward */
    float stage1_ref_adc;      /* ADC reading at 5W reflected */
    float stage2_fwd_adc;      /* ADC reading at 20W forward */
    float stage2_ref_adc;      /* ADC reading at 20W reflected */
} CouplerCalData;

/*============================================================================
 * Communication Structures
 *============================================================================*/

typedef struct {
    uint8_t data[COMM_RX_QUEUE_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} CommQueueType;

typedef struct {
    CommandType type;
    char args[COMM_CMD_MAX_LEN];
} CommandTypeDef;

/*============================================================================
 * Function Prototypes
 *============================================================================*/

/**
 * @brief Initialize communication (USB and LPUART)
 */
void comm_init(void);

/**
 * @brief Start communication (enable USB and UART)
 */
void comm_start(void);

/**
 * @brief Process received data
 * @note Call from communication task
 */
void comm_process_rx(void);

/**
 * @brief Parse and execute command
 * @param cmd Command string
 */
void comm_execute_command(const char *cmd);

/**
 * @brief Send response to both USB and LPUART
 * @param data Data to send
 * @param len Data length
 */
void comm_send_response(const uint8_t *data, uint16_t len);

/**
 * @brief Send string response
 * @param str String to send
 */
void comm_send_string(const char *str);

/**
 * @brief Get radio frequency from communication
 * @return Frequency in Hz
 */
uint32_t comm_get_radio_frequency(void);

/**
 * @brief Get radio type
 * @return Radio type
 */
RadioType comm_get_radio_type(void);

/**
 * @brief Queue received byte
 * @param byte Received byte
 */
void comm_queue_rx_byte(uint8_t byte);

/**
 * @brief LPUART Rx Event Callback (called on IDLE line detection)
 * @param size Number of bytes received
 */
void comm_lpuart_rx_callback(uint16_t size);

/**
 * @brief Get queued byte
 * @param byte Pointer to store byte
 * @return true if byte was available
 */
bool comm_get_queued_byte(uint8_t *byte);

/**
 * @brief Process radio communication (ICOM/Yaesu/Kenwood)
 * @param data Received data
 * @param len Data length
 */
void comm_process_radio_data(const uint8_t *data, uint16_t len);

/**
 * @brief Start coupler calibration
 * @param coupler_id Which coupler to calibrate (0=TX1, 1=TX2, 2=REF1, 3=REF2)
 * @return true if calibration started successfully
 */
bool comm_calibrate_coupler_start(CouplerId coupler_id);

/**
 * @brief Continue calibration with current ADC reading
 * @return true if calibration continued (more steps needed)
 */
bool comm_calibrate_coupler_continue(void);

/**
 * @brief Abort calibration
 */
void comm_calibrate_coupler_abort(void);

/**
 * @brief Get current calibration state
 * @return Current state
 */
CouplerCalState comm_calibrate_get_state(void);

#ifdef __cplusplus
}
#endif

#endif /* COMM_H */