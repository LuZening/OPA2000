/**
 * @file comm.c
 * @brief Communication Protocol Implementation
 */

#include "comm.h"
#include "config.h"
#include "user_adc.h"
#include "power_control.h"
#include "protection.h"
#include "radio_band.h"
#include "radio_uart.h"
#include "opa2000.h"
#include "string.h"
#include "stdio.h"
#include "main.h"
#include "USB_DEVICE/App/usbd_cdc_if.h"
#include "usart.h"
#include "usbd_cdc_if.h"

/*============================================================================
 * Private Variables
 *============================================================================*/

/* Remote PSU control over the CAT link is a skeleton: the packet format
 * has not been decided, so comm_parse_command() recognises no PSU keyword
 * and the CMD_PSU_ON / CMD_PSU_OFF branches are unreachable.  Flip this to
 * 1 once the protocol is agreed - the call sites are already in place.
 * See User/TODO-PSU远程控制.txt. */
#define COMM_PSU_REMOTE_ENABLED   0

/* RX Queue - shared between USB and LPUART */
static CommQueueType comm_rx_queue;

/* LPUART DMA Receive Buffer
 *
 * LPUART1 在 D3 域，其 DMA 是 BDMA2；BDMA 只能访问 D3 的 SRAM4
 * (0x38000000)/Backup SRAM，够不到 DTCM，也够不到 AXI SRAM。因此这块 DMA
 * 缓冲必须落在 app.ld 的 .d3_dma_buffer（映射到 SRAM4，非缓存），32B 对齐。 */
#define LPUART_RX_BUFFER_SIZE  256
#if defined(__GNUC__)
#define LPUART_BUF_SECTION  __attribute__((section(".d3_dma_buffer"), aligned(32)))
#else
#define LPUART_BUF_SECTION
#endif
static uint8_t lpuart_rx_buffer[LPUART_RX_BUFFER_SIZE] LPUART_BUF_SECTION;


/* Radio information */
static uint32_t g_radio_frequency = 0;
static RadioType g_radio_type = RADIO_TYPE_UNKNOWN;

/* Last command parse attempt result */
static char comm_cmd_buffer[COMM_CMD_MAX_LEN];
static uint8_t comm_cmd_index = 0;

/* Response format buffer */
static char comm_response_buffer[256];

/* Coupler calibration state machine */
static CouplerCalState coupler_cal_state = COUPLER_CAL_IDLE;
static CouplerId coupler_cal_id = COUPLER_1;
static CouplerCalData coupler_cal_data;

/* Coupler names for display */
static const char* const coupler_names[] = {
    "Coupler 1 (PA->LPF)",
    "Coupler 2 (LPF->Antenna)"
};

/*============================================================================
 * Private Functions
 *============================================================================*/

/**
 * @brief Parse command string
 */
static CommandType comm_parse_command(const char *cmd)
{
    if (cmd == NULL || strlen(cmd) == 0) {
        return CMD_NONE;
    }

    /* Skip leading spaces */
    while (*cmd == ' ') cmd++;

    /* Compare commands (case insensitive) */
    if (strcasecmp(cmd, "STATUS") == 0 || strcasecmp(cmd, "STATUS?") == 0) {
        return CMD_GET_STATUS;
    }
    if (strcasecmp(cmd, "POWER?") == 0) {
        return CMD_GET_POWER;
    }
    if (strcasecmp(cmd, "SWR?") == 0) {
        return CMD_GET_SWR;
    }
    if (strcasecmp(cmd, "VOLTAGE?") == 0) {
        return CMD_GET_VOLTAGE;
    }
    if (strcasecmp(cmd, "CURRENT?") == 0) {
        return CMD_GET_CURRENT;
    }
    if (strcasecmp(cmd, "TEMP?") == 0) {
        return CMD_GET_TEMP;
    }
    if (strcasecmp(cmd, "BAND?") == 0) {
        return CMD_GET_BAND;
    }
    if (strcasecmp(cmd, "FREQ?") == 0) {
        return CMD_GET_FREQ;
    }
    /* SET FREQ <frequency> - set frequency in Hz (e.g., SET FREQ 14100000) */
    if (strncasecmp(cmd, "SET FREQ ", 9) == 0 || strncasecmp(cmd, "FREQ ", 5) == 0) {
        return CMD_SET_FREQ;
    }
    if (strcasecmp(cmd, "ALERT OFF") == 0 || strcasecmp(cmd, "ALERT OFF") == 0) {
        return CMD_ALERT_OFF;
    }
    if (strncasecmp(cmd, "SET ", 4) == 0) {
        return CMD_SET_CONFIG;
    }
    if (strcasecmp(cmd, "CONFIG?") == 0 || strcasecmp(cmd, "GET CONFIG") == 0) {
        return CMD_GET_CONFIG;
    }
    if (strcasecmp(cmd, "SAVE CONFIG") == 0) {
        return CMD_SAVE_CONFIG;
    }
    if (strcasecmp(cmd, "RESET CONFIG") == 0) {
        return CMD_RESET_CONFIG;
    }
    if (strncasecmp(cmd, "CAL", 3) == 0) {
        return CMD_CALIBRATE;
    }
    if (strncasecmp(cmd, "LANG ", 5) == 0) {
        return CMD_SET_LANGUAGE;
    }
    if (strcasecmp(cmd, "LANG?") == 0) {
        return CMD_GET_LANGUAGE;
    }
    if (strncasecmp(cmd, "CAL COUPLER ", 12) == 0) {
        return CMD_CALIBRATE_COUPLER;
    }
    if (strcasecmp(cmd, "OK") == 0 && coupler_cal_state != COUPLER_CAL_IDLE) {
        /* OK command during calibration - continue */
        return CMD_CALIBRATE_COUPLER;
    }
    if ((strcasecmp(cmd, "ABORT") == 0 || strcasecmp(cmd, "CAL ABORT") == 0) &&
        coupler_cal_state != COUPLER_CAL_IDLE) {
        return CMD_CALIBRATE_COUPLER;
    }

    return CMD_UNKNOWN;
}

/**
 * @brief Execute command and send response
 */
static void comm_execute(CommandType cmd_type, const char *args)
{
    float value;
    uint32_t uvalue;

    switch (cmd_type) {
        case CMD_GET_STATUS:
            snprintf(comm_response_buffer, sizeof(comm_response_buffer),
                "PWR=%s,Band=%d,Freq=%lu,Power=%.1f,Reflect=%.1f,SWR=%.1f,Volt=%.1f,Current=%.1f,Temp=%.1f,Alert=%d",
                g_hv_on ? (g_tx_active ? "TX" : "STBY") : "OFF",
                g_current_band,
                g_radio_frequency,
                (double)g_adc_data.fwd2_mW / 1000.0,
                (double)g_adc_data.rev2_mW / 1000.0,
                (double)g_adc_data.swr_x100 / 100.0,
                (double)g_adc_data.vm_mV / 1000.0,
                (double)g_adc_data.im_mA / 1000.0,
                (double)g_adc_data.ntc_core_tenth_c / 10.0,
                g_alert_active);
            comm_send_string(comm_response_buffer);
            break;

        case CMD_GET_POWER:
            snprintf(comm_response_buffer, sizeof(comm_response_buffer),
                "POWER=%.1f", (double)g_adc_data.fwd2_mW / 1000.0);
            comm_send_string(comm_response_buffer);
            break;

        case CMD_GET_SWR:
            snprintf(comm_response_buffer, sizeof(comm_response_buffer),
                "SWR=%.2f", (double)g_adc_data.swr_x100 / 100.0);
            comm_send_string(comm_response_buffer);
            break;

        case CMD_GET_VOLTAGE:
            snprintf(comm_response_buffer, sizeof(comm_response_buffer),
                "VOLTAGE=%.1f", (double)g_adc_data.vm_mV / 1000.0);
            comm_send_string(comm_response_buffer);
            break;

        case CMD_GET_CURRENT:
            snprintf(comm_response_buffer, sizeof(comm_response_buffer),
                "CURRENT=%.1f", (double)g_adc_data.im_mA / 1000.0);
            comm_send_string(comm_response_buffer);
            break;

        case CMD_GET_TEMP:
            snprintf(comm_response_buffer, sizeof(comm_response_buffer),
                "TEMP=%.1f", (double)g_adc_data.ntc_core_tenth_c / 10.0);
            comm_send_string(comm_response_buffer);
            break;

        case CMD_GET_BAND:
            comm_send_string(g_current_band == BAND_NONE ? "BAND=0" :
                (g_current_band == BAND_160M ? "BAND=160" :
                 g_current_band == BAND_80M ? "BAND=80" :
                 g_current_band == BAND_40M_30M ? "BAND=40" :
                 g_current_band == BAND_20M_17M ? "BAND=20" :
                 g_current_band == BAND_15M_10M ? "BAND=15" :
                 g_current_band == BAND_FAULT ? "BAND=0" : "BAND=0"));
            break;

        case CMD_GET_FREQ:
            snprintf(comm_response_buffer, sizeof(comm_response_buffer), "FREQ=%lu", g_radio_frequency);
            comm_send_string(comm_response_buffer);
            break;

        case CMD_SET_FREQ: {
            /* Parse frequency in kHz (user input) and convert to Hz */
            uint32_t freq_khz = 0;
            if (args != NULL && strlen(args) > 0) {
                freq_khz = atoi(args);
            }
            /* Convert kHz to Hz */
            uint32_t freq = freq_khz * 1000;

            if (freq_khz == 0 || freq < 1800000 || freq > 30000000) {
                comm_send_string("ERROR: Invalid frequency (1800-30000 kHz)");
                break;
            }

            /* Set frequency and auto-switch band */
            g_radio_frequency = freq;
            BandType band = frequency_to_band(freq);

            if (band == BAND_FAULT) {
                comm_send_string("ERROR: Frequency out of ham bands");
                break;
            }

            /* Switch to the selected band */
            switch_band(band);

            /* Return confirmation with frequency range */
            const char *range_str = NULL;
            switch (band) {
                case BAND_160M: range_str = "1.800-2.100 MHz"; break;
                case BAND_80M:  range_str = "3.500-3.900 MHz"; break;
                case BAND_40M_30M: range_str = "7.000-10.150 MHz"; break;
                case BAND_20M_17M: range_str = "14.000-18.168 MHz"; break;
                case BAND_15M_10M: range_str = "21.000-29.700 MHz"; break;
                default: range_str = "Unknown"; break;
            }
            snprintf(comm_response_buffer, sizeof(comm_response_buffer),
                "FREQ=%lu,BAND=%s,RANGE=%s", freq_khz, strBands[band - 1], range_str);
            comm_send_string(comm_response_buffer);
            break;
        }

        case CMD_ALERT_OFF:
            /* Clearing the two flags is not enough: the latch inside
             * protection.c would stay set, PB13 would stay high and the
             * buzzer would keep running while the PC believed the fault
             * was gone.  protection_clear() does all three. */
            protection_clear();
            comm_send_string("ALERT OFF OK");
            break;

        case CMD_SET_LANGUAGE:
            if (args != NULL) {
                if (strcasecmp(args, "CHINESE") == 0 || strcasecmp(args, "ZH") == 0) {
                    g_config.language = MULTILANG_CHINESE;
                    config_save();
                    comm_send_string("LANG=CHINESE");
                } else if (strcasecmp(args, "ENGLISH") == 0 || strcasecmp(args, "EN") == 0) {
                    g_config.language = MULTILANG_ENGLISH;
                    config_save();
                    comm_send_string("LANG=ENGLISH");
                } else {
                    comm_send_string("ERROR: Unknown language");
                }
            }
            break;

        case CMD_GET_LANGUAGE:
            comm_send_string(g_config.language == MULTILANG_CHINESE ? "LANG=CHINESE" : "LANG=ENGLISH");
            break;

        case CMD_GET_CONFIG:
            snprintf(comm_response_buffer, sizeof(comm_response_buffer),
                "CONFIG: PC_BAUD=%lu, RADIO=%s, RADIO_BAUD=%lu, "
                "REV2=%u, REV2_SWR=%u, SWR=%u.%02u, REV1=%u, IM=%u",
                (unsigned long)g_config.pc_baud_rate,
                config_get_radio_type_string(g_config.radio_type),
                (unsigned long)g_config.radio_baud_rate,
                (uint32_t)g_config.band_rev2_max_w,
                (uint32_t)g_config.band_rev2_swr_w,
                (uint32_t)(g_config.band_swr_max_x100 / 100U),
                (uint32_t)(g_config.band_swr_max_x100 % 100U),
                (uint32_t)g_config.band_rev1_max_w,
                (uint32_t)g_config.band_im_max_mA);
            comm_send_string(comm_response_buffer);
            break;

        case CMD_SAVE_CONFIG:
            if (config_save()) {
                comm_send_string("CONFIG SAVED");
            } else {
                comm_send_string("ERROR: Save failed");
            }
            break;

        case CMD_RESET_CONFIG:
            config_reset_default();
            comm_send_string("CONFIG RESET");
            break;

        case CMD_CALIBRATE:
            /* This would start the calibration procedure */
            comm_send_string("CALIBRATION: Use menu on display");
            break;

        case CMD_CALIBRATE_COUPLER:
            {
                /* Handle calibration commands */
                bool handled = false;

                /* Check for abort commands */
                if (strcasecmp(args, "ABORT") == 0 || strncasecmp(args, "CAL ABORT", 9) == 0) {
                    comm_calibrate_coupler_abort();
                    comm_send_string("CALIBRATION ABORTED");
                    handled = true;
                }
                /* Check for OK command during calibration */
                else if (strcasecmp(args, "OK") == 0 && coupler_cal_state != COUPLER_CAL_IDLE) {
                    handled = comm_calibrate_coupler_continue();
                }
                /* Start new calibration */
                else if (strncasecmp(args, "CAL COUPLER ", 12) == 0) {
                    int coupler_id = args[12] - '0';
                    if (coupler_id >= 0 && coupler_id < COUPLER_COUNT) {
                        handled = comm_calibrate_coupler_start((CouplerId)coupler_id);
                    } else {
                        comm_send_string("ERROR: Invalid coupler ID (0-3)");
                        handled = true;
                    }
                }
                else if (coupler_cal_state != COUPLER_CAL_IDLE) {
                    /* Any other command during calibration - continue */
                    handled = comm_calibrate_coupler_continue();
                }
                else {
                    /* Show help */
                    comm_send_string("ERROR: Unknown calibration command");
                    comm_send_string("Usage: CAL COUPLER <0-1> where 0=Coupler1(PA->LPF), 1=Coupler2(LPF->Ant)");
                    handled = true;
                }
            }
            break;

        /* ------------------------------------------------------------------
         *  Remote PSU control - SKELETON ONLY.
         *
         *  TODO(PROTOCOL): the CAT packet format for these is not defined
         *  yet, so comm_parse_command() recognises no keyword that reaches
         *  here and both branches below are unreachable.  They are written
         *  out so the call site and the reply convention are settled once the
         *  framing is agreed, and so the intent is greppable.
         *
         *  Nothing else in the firmware drives the PSU: power_init() leaves
         *  it off, and power_soft_off() is the only path that switches it.
         *  See User/TODO-PSU远程控制.txt.
         * ---------------------------------------------------------------- */
        case CMD_PSU_ON:
        case CMD_PSU_OFF:
            /* The body is compiled but the feature stays off until the
             * protocol is agreed: flip COMM_PSU_REMOTE_ENABLED then.  Using a
             * macro rather than a literal false keeps the compiler from
             * warning about unreachable code while the skeleton sits here. */
            if (COMM_PSU_REMOTE_ENABLED) {
                if (cmd_type == CMD_PSU_ON) {
                    power_psu_on();
                    comm_send_string("PSU ON");
                } else {
                    power_psu_off();
                    comm_send_string("PSU OFF");
                }
            } else {
                comm_send_string("ERROR: PSU control not implemented");
            }
            break;

        default:
            comm_send_string("ERROR: Unknown command");
            break;
    }
}

/**
 * @brief Process ICOM CIV protocol
 */
static bool comm_parse_icom_civ(const uint8_t *data, uint16_t len)
{
    /* ICOM CIV format: FE FE <address> E0 <cmd> <data...> FD */
    /* We look for the frequency data in commands 00, 01, 02 */

    if (len < 5) return false;
    if (data[0] != 0xFE || data[1] != 0xFE) return false;

    /* Check if this is a frequency command response */
    /* Command 00 = TX Frequency, 01 = RX Frequency */
    if (data[3] == 0x00 || data[3] == 0x01) {
        /* Parse frequency from bytes 4-8 (5 bytes BCD) */
        uint32_t freq = 0;
        for (int i = 4; i < len && i < 9; i++) {
            freq = freq * 100 + (data[i] & 0x0F) * 10 + ((data[i] >> 4) & 0x0F) * 1;
        }
        g_radio_frequency = freq;
        g_radio_type = RADIO_TYPE_ICOM_CIV;
        return true;
    }

    return false;
}

/**
 * @brief Process Yaesu protocol
 */
static bool comm_parse_yaesu(const uint8_t *data, uint16_t len)
{
    /* Yaesu CAT format varies by model - typically 5-byte responses
     * Common format: the radio returns BCD-coded frequency
     * For most Yaesu models (FT-857/897/991/450 etc.):
     * - Response to frequency query is 5 bytes
     * - Bytes [1..4] contain BCD frequency (MHz in 100Hz steps)
     *
     * Example: 14.074200 MHz = 0x14 0x07 0x42 0x00 (BCD)
     * Byte 0 is usually command echo or status
     */

    if (len < 5) return false;

    /* Check for Yaesu IF response (read frequency response)
     * Typically: first byte is 0x00 or 0x01 or command echo
     * Following 4 bytes are BCD frequency (big-endian)
     * Frequency = BCD bytes 1-4 in format HHMMSSKK (100Hz units)
     */

    /* Try to decode 4 bytes of BCD frequency starting at byte 1 */
    uint32_t freq = 0;
    for (int i = 1; i < 5 && i < (int)len; i++) {
        uint8_t hi = (data[i] >> 4) & 0x0F;
        uint8_t lo = data[i] & 0x0F;
        if (hi > 9 || lo > 9) return false;  /* Not valid BCD */
        freq = freq * 100 + (hi * 10 + lo);
    }

    /* Sanity check: valid amateur radio frequency (1.8MHz - 30MHz) */
    /* freq is in 100Hz units, so 18000 = 1.8MHz, 300000 = 30MHz */
    if (freq < 18000 || freq > 300000) return false;

    /* Convert from 100Hz units to Hz */
    g_radio_frequency = freq * 100;
    g_radio_type = RADIO_TYPE_YAESU;
    return true;
}

/**
 * @brief Process Kenwood protocol
 */
static bool comm_parse_kenwood(const uint8_t *data, uint16_t len)
{
    /* Kenwood format: ; commands end with semicolon */
    /* Like IF; returns frequency */

    if (len < 2) return false;
    if (data[len-1] != ';') return false;

    /* Look for IF response (frequency) */
    if (len >= 3 && data[0] == 'I' && data[1] == 'F') {
        /* Parse frequency from position 2-10 */
        uint32_t freq = 0;
        for (int i = 2; i < len && i < 11; i++) {
            if (data[i] >= '0' && data[i] <= '9') {
                freq = freq * 10 + (data[i] - '0');
            }
        }
        g_radio_frequency = freq;
        g_radio_type = RADIO_TYPE_KENWOOD;
        return true;
    }

    return false;
}

/**
 * @brief Auto-detect radio type from data
 */
static void comm_detect_radio_type(const uint8_t *data, uint16_t len)
{
    /* Try each protocol parser */
    if (comm_parse_icom_civ(data, len)) return;
    if (comm_parse_kenwood(data, len)) return;
    if (comm_parse_yaesu(data, len)) return;
}

/*============================================================================
 * Public Functions
 *============================================================================*/

/**
 * @brief Initialize communication
 */
void comm_init(void)
{
    /* Initialize RX queue */
    comm_rx_queue.head = 0;
    comm_rx_queue.tail = 0;
    comm_rx_queue.count = 0;

    comm_cmd_index = 0;
    g_radio_frequency = 0;
    g_radio_type = g_config.radio_type;
}

/**
 * @brief Start communication
 */
void comm_start(void)
{
    /* Start DMA reception for LPUART */
    HAL_UART_Receive_DMA(&hlpuart1, lpuart_rx_buffer, LPUART_RX_BUFFER_SIZE);
}

/**
 * @brief LPUART Rx Event Callback (called on IDLE line detection)
 * This is called from stm32h7xx_it.c
 */
void comm_lpuart_rx_callback(uint16_t size)
{
    /* Process received data - copy from DMA buffer to queue */
    for (uint16_t i = 0; i < size; i++) {
        comm_queue_rx_byte(lpuart_rx_buffer[i]);
    }

    /* Restart DMA reception (stop first to reset HAL state after IDLE) */
    HAL_UART_DMAStop(&hlpuart1);
    HAL_UART_Receive_DMA(&hlpuart1, lpuart_rx_buffer, LPUART_RX_BUFFER_SIZE);
}

/**
 * @brief HAL UART Rx Event Callback override (called on IDLE line detection)
 * This overrides the __weak default in stm32h7xx_hal_uart.c
 *
 * This is the ONLY definition of this symbol in the project.  HAL declares it
 * once, weakly, for all UART instances, so a second definition in radio_uart.c
 * was a link error (multiple definition).  Instead this callback dispatches by
 * instance: LPUART1 is handled here, everything else (i.e. UART4, the radio CAT
 * link) is forwarded to radio_uart.c.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == LPUART1) {
        comm_lpuart_rx_callback(Size);
    } else {
        radio_uart_rx_event(huart, Size);
    }
}

/**
 * @brief Process received data
 */
void comm_process_rx(void)
{
    uint8_t byte;

    while (comm_get_queued_byte(&byte)) {
        /* Add to command buffer */
        if (comm_cmd_index < COMM_CMD_MAX_LEN - 1) {
            comm_cmd_buffer[comm_cmd_index++] = byte;
        }

        /* Check for command terminator */
        if (byte == '\n' || byte == '\r') {
            /* Null terminate and process command */
            comm_cmd_buffer[comm_cmd_index] = '\0';

            /* Skip empty commands */
            if (comm_cmd_index > 1) {
                /* Remove trailing whitespace */
                while (comm_cmd_index > 0 &&
                       (comm_cmd_buffer[comm_cmd_index-1] == '\n' ||
                        comm_cmd_buffer[comm_cmd_index-1] == '\r' ||
                        comm_cmd_buffer[comm_cmd_index-1] == ' ')) {
                    comm_cmd_buffer[--comm_cmd_index] = '\0';
                }

                if (comm_cmd_index > 0) {
                    /* Parse and execute command */
                    CommandType cmd = comm_parse_command(comm_cmd_buffer);
                    char *args = strchr(comm_cmd_buffer, ' ');
                    if (args) args++;

                    comm_execute(cmd, args);
                }
            }

            /* Reset command buffer */
            comm_cmd_index = 0;
        }
    }
}

/**
 * @brief Execute command
 */
void comm_execute_command(const char *cmd)
{
    CommandType cmd_type = comm_parse_command(cmd);
    char *args = strchr(cmd, ' ');
    if (args) args++;

    comm_execute(cmd_type, args);
}

/**
 * @brief Send response to both USB and LPUART
 */
void comm_send_response(const uint8_t *data, uint16_t len)
{
    /* Send over LPUART1 (polling mode) */
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)data, len, 100);

    /* Send over USB CDC */
    CDC_Transmit_HS((uint8_t*)data, len);
}

/**
 * @brief Send string response
 */
void comm_send_string(const char *str)
{
    uint16_t len = strlen(str);

    /* Build response with CR LF */
    char buffer[COMM_TX_QUEUE_SIZE];
    uint16_t buflen = 0;

    if (len + 2 < COMM_TX_QUEUE_SIZE) {
        memcpy(buffer, str, len);
        buflen = len;
        buffer[buflen++] = '\r';
        buffer[buflen++] = '\n';
    }

    if (buflen == 0) return;

    /* Send over LPUART1 (polling mode) */
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)buffer, buflen, 100);

    /* Send over USB CDC */
    CDC_Transmit_HS((uint8_t*)buffer, buflen);
}

/**
 * @brief Get radio frequency
 */
uint32_t comm_get_radio_frequency(void)
{
    return g_radio_frequency;
}

/**
 * @brief Get radio type
 * @note  Reports what radio_band.c actually detected on the CAT line, not
 *        the configured default: until a frame is recognised the answer is
 *        RADIO_TYPE_UNKNOWN, which is what the GUI shows as "not connected".
 */
RadioType comm_get_radio_type(void)
{
    return radio_band_get_type();
}

/**
 * @brief Queue received byte
 */
void comm_queue_rx_byte(uint8_t byte)
{
    if (comm_rx_queue.count < COMM_RX_QUEUE_SIZE) {
        comm_rx_queue.data[comm_rx_queue.tail] = byte;
        comm_rx_queue.tail = (comm_rx_queue.tail + 1) % COMM_RX_QUEUE_SIZE;
        comm_rx_queue.count++;
    }
}

/**
 * @brief Get queued byte
 */
bool comm_get_queued_byte(uint8_t *byte)
{
    if (comm_rx_queue.count == 0) {
        return false;
    }

    *byte = comm_rx_queue.data[comm_rx_queue.head];
    comm_rx_queue.head = (comm_rx_queue.head + 1) % COMM_RX_QUEUE_SIZE;
    comm_rx_queue.count--;

    return true;
}

/**
 * @brief Process radio communication data
 */
void comm_process_radio_data(const uint8_t *data, uint16_t len)
{
    /* First, try auto-detection if radio type unknown */
    if (g_radio_type == RADIO_TYPE_UNKNOWN) {
        comm_detect_radio_type(data, len);
    } else {
        /* Use known protocol */
        switch (g_radio_type) {
            case RADIO_TYPE_ICOM_CIV:
                comm_parse_icom_civ(data, len);
                break;
            case RADIO_TYPE_YAESU:
                comm_parse_yaesu(data, len);
                break;
            case RADIO_TYPE_KENWOOD:
                comm_parse_kenwood(data, len);
                break;
            default:
                /* Try all */
                comm_detect_radio_type(data, len);
                break;
        }
    }
}

/*============================================================================
 * Coupler Calibration Functions
 *============================================================================*/

/**
 * @brief Start coupler calibration
 */
bool comm_calibrate_coupler_start(CouplerId coupler_id)
{
    if (coupler_cal_state != COUPLER_CAL_IDLE) {
        comm_send_string("ERROR: Calibration already in progress");
        return false;
    }

    coupler_cal_id = coupler_id;

    /* Initialize calibration data */
    coupler_cal_data.stage1_fwd_adc = 0;
    coupler_cal_data.stage1_ref_adc = 0;
    coupler_cal_data.stage2_fwd_adc = 0;
    coupler_cal_data.stage2_ref_adc = 0;

    coupler_cal_state = COUPLER_CAL_START;

    /* Send welcome message and instructions */
    comm_send_string("========================================");
    snprintf(comm_response_buffer, sizeof(comm_response_buffer),
        "CALIBRATION: %s Coupler", coupler_names[coupler_id]);
    comm_send_string(comm_response_buffer);
    comm_send_string("========================================");
    comm_send_string("");
    comm_send_string("Step 1: Connect radio to amp, set to AM mode");
    comm_send_string("Step 2: Set radio power to 5W");
    comm_send_string("Step 3: Press PTT and confirm forward power");
    comm_send_string("Step 4: Type OK and press Enter when ready");

    return true;
}

/**
 * @brief Continue calibration - read current ADC and advance state
 */
bool comm_calibrate_coupler_continue(void)
{
    float current_adc = 0;
    bool forward_stage = (coupler_cal_state == COUPLER_CAL_STAGE1_5W_FWD ||
                          coupler_cal_state == COUPLER_CAL_STAGE2_20W_FWD);

    /* Get current ADC value based on coupler being calibrated.  Coupler 1 is
     * PA->LPF (ADC1_RANK_FWD1 / ADC1_RANK_REV1), coupler 2 is LPF->antenna
     * (ADC1_RANK_FWD2 / ADC1_RANK_REV2). */
    uint8_t rank;
    switch (coupler_cal_id) {
        case COUPLER_1:
            rank = forward_stage ? ADC1_RANK_FWD1 : ADC1_RANK_REV1;
            break;
        case COUPLER_2:
            rank = forward_stage ? ADC1_RANK_FWD2 : ADC1_RANK_REV2;
            break;
        default:
            return false;
    }
    current_adc = (float)g_adc_data.adc1_raw[rank];

    switch (coupler_cal_state) {
        case COUPLER_CAL_START:
            /* User confirmed ready - enter stage 1 */
            coupler_cal_state = COUPLER_CAL_STAGE1_5W_FWD;
            comm_send_string("");
            comm_send_string(">>> Recording 5W FORWARD power...");
            snprintf(comm_response_buffer, sizeof(comm_response_buffer),
                "ADC=%d", (int)current_adc);
            comm_send_string(comm_response_buffer);
            coupler_cal_data.stage1_fwd_adc = current_adc;

            comm_send_string("");
            comm_send_string("Now DISCONNECT the antenna (or set RELAY2 to reflect)");
            comm_send_string("Press PTT and confirm reflected power");
            comm_send_string("Type OK and press Enter when ready");
            break;

        case COUPLER_CAL_STAGE1_5W_FWD:
            /* Stage 1 forward recorded, now record reflected */
            coupler_cal_state = COUPLER_CAL_STAGE1_5W_REF;
            comm_send_string("");
            comm_send_string(">>> Recording 5W REFLECTED power...");
            snprintf(comm_response_buffer, sizeof(comm_response_buffer),
                "ADC=%d, Reflected=%.1fW", (int)current_adc,
                (double)g_adc_data.rev2_mW / 1000.0);
            comm_send_string(comm_response_buffer);
            coupler_cal_data.stage1_ref_adc = current_adc;

            comm_send_string("");
            comm_send_string("Step 1 complete!");
            comm_send_string("========================================");
            comm_send_string("Step 2: Increase radio power to 20W");
            comm_send_string("Step 3: Press PTT and confirm forward power");
            comm_send_string("Step 4: Type OK and press Enter when ready");
            break;

        case COUPLER_CAL_STAGE1_5W_REF:
            /* Stage 1 reflected recorded, now start stage 2 */
            coupler_cal_state = COUPLER_CAL_STAGE2_20W_FWD;
            comm_send_string("");
            comm_send_string(">>> Recording 20W FORWARD power...");
            snprintf(comm_response_buffer, sizeof(comm_response_buffer),
                "ADC=%d", (int)current_adc);
            comm_send_string(comm_response_buffer);
            coupler_cal_data.stage2_fwd_adc = current_adc;

            comm_send_string("");
            comm_send_string("Now DISCONNECT the antenna (or set RELAY2 to reflect)");
            comm_send_string("Press PTT and confirm reflected power");
            comm_send_string("Type OK and press Enter when ready");
            break;

        case COUPLER_CAL_STAGE2_20W_FWD:
            /* Stage 2 forward recorded, now record reflected and report */
            coupler_cal_state = COUPLER_CAL_COMPLETE;
            comm_send_string("");
            comm_send_string(">>> Recording 20W REFLECTED power...");
            snprintf(comm_response_buffer, sizeof(comm_response_buffer),
                "ADC=%d, Reflected=%.1fW", (int)current_adc,
                (double)g_adc_data.rev2_mW / 1000.0);
            comm_send_string(comm_response_buffer);
            coupler_cal_data.stage2_ref_adc = current_adc;

            /* Report the two-point fit through the 5W / 20W reference
             * points.  The forward-power conversion itself is NOT adjusted
             * here: the coupler front end is modelled exactly like OPA1000
             * (see adc_calibration.h section 1b), so there are no per-channel
             * slope / offset coefficients in g_config to store - the numbers
             * below are for the operator to sanity-check, nothing more. */
            {
                float fwd_x1 = coupler_cal_data.stage1_fwd_adc;
                float fwd_x2 = coupler_cal_data.stage2_fwd_adc;
                float fwd_slope = (fwd_x2 != fwd_x1) ? (20.0f - 5.0f) / (fwd_x2 - fwd_x1) : 0.0f;

                float ref_x1 = coupler_cal_data.stage1_ref_adc;
                float ref_x2 = coupler_cal_data.stage2_ref_adc;
                float ref_slope = (ref_x2 != ref_x1) ? (20.0f - 5.0f) / (ref_x2 - ref_x1) : 0.0f;

                comm_send_string("");
                comm_send_string("========================================");
                comm_send_string("CALIBRATION COMPLETE!");
                comm_send_string("========================================");
                snprintf(comm_response_buffer, sizeof(comm_response_buffer),
                    "Forward: ADC %.0f -> %.0f  (%.6f W/code)",
                    (double)fwd_x1, (double)fwd_x2, (double)fwd_slope);
                comm_send_string(comm_response_buffer);
                snprintf(comm_response_buffer, sizeof(comm_response_buffer),
                    "Reflect: ADC %.0f -> %.0f  (%.6f W/code)",
                    (double)ref_x1, (double)ref_x2, (double)ref_slope);
                comm_send_string(comm_response_buffer);
                comm_send_string("Conversion coefficients are fixed in firmware; nothing stored.");
            }
            coupler_cal_state = COUPLER_CAL_IDLE;
            break;

        default:
            comm_send_string("ERROR: Invalid calibration state");
            coupler_cal_state = COUPLER_CAL_IDLE;
            return false;
    }

    return true;
}

/**
 * @brief Abort calibration
 */
void comm_calibrate_coupler_abort(void)
{
    coupler_cal_state = COUPLER_CAL_IDLE;
}

/**
 * @brief Get current calibration state
 */
CouplerCalState comm_calibrate_get_state(void)
{
    return coupler_cal_state;
}

/**
 * @brief UART RX callback
 * @note  Removed: the radio link is handled by radio_uart.c (DMA + IDLE
 *        line interrupt) and radio_band.c (framing + protocol), driven
 *        from app_comm_task.  The old per-character path called
 *        comm_process_radio_data() once per byte, which re-ran the whole
 *        protocol decoder N times for an N-byte frame and could not tell
 *        where a frame started or ended.
 */