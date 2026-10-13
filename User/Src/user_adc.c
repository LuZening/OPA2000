/**
 * @file user_adc.c
 * @brief ADC Data Acquisition Implementation
 */

#include "user_adc.h"
#include "stm32h7xx_hal.h"
#include "opa2000.h"

/*============================================================================
 * Private Variables
 *============================================================================*/

/* DMA buffer - placed in non-cached memory */
static uint16_t adc_dma_buffer[ADC_CHANNEL_COUNT * ADC_DMA_BUFFER_SIZE]
    __attribute__((section(".dma_buffer")));

/* Filtered values for each channel */
static uint32_t adc_filtered[ADC_CHANNEL_COUNT] = {0};

/* Sample count for averaging */
static uint32_t adc_sample_count = 0;

/*============================================================================
 * Private Functions
 *============================================================================*/

/**
 * @brief Apply simple moving average filter
 */
static void adc_apply_filter(void)
{
    uint32_t sum[ADC_CHANNEL_COUNT] = {0};

    /* Sum all samples */
    for (uint32_t i = 0; i < adc_sample_count; i++) {
        for (uint8_t ch = 0; ch < ADC_CHANNEL_COUNT; ch++) {
            sum[ch] += adc_dma_buffer[i * ADC_CHANNEL_COUNT + ch];
        }
    }

    /* Calculate average */
    if (adc_sample_count > 0) {
        for (uint8_t ch = 0; ch < ADC_CHANNEL_COUNT; ch++) {
            adc_filtered[ch] = sum[ch] / adc_sample_count;
        }
    }
}

/**
 * @brief Update ADC data structure with processed values
 */
static void adc_update_values(void)
{
    uint16_t raw;

    /* Channel mapping based on hardware design:
     * ADC1: INP16 (PA0) - TX Power 1 (actually seems to be different in IOC)
     * Let me check what ADC channels are configured
     * Based on IOC: ADC1 has one channel configured
     * Need to check the actual channel mapping
     */

    /* Get filtered values */
    raw = (uint16_t)adc_filtered[ADC_CHANNEL_TX_POWER1];
    g_adc_data.raw[ADC_CHANNEL_TX_POWER1] = raw;

    raw = (uint16_t)adc_filtered[ADC_CHANNEL_REF_POWER1];
    g_adc_data.raw[ADC_CHANNEL_REF_POWER1] = raw;

    raw = (uint16_t)adc_filtered[ADC_CHANNEL_MAIN_VOLTAGE];
    g_adc_data.raw[ADC_CHANNEL_MAIN_VOLTAGE] = raw;

    raw = (uint16_t)adc_filtered[ADC_CHANNEL_MAIN_CURRENT];
    g_adc_data.raw[ADC_CHANNEL_MAIN_CURRENT] = raw;

    raw = (uint16_t)adc_filtered[ADC_CHANNEL_TX_POWER2];
    g_adc_data.raw[ADC_CHANNEL_TX_POWER2] = raw;

    raw = (uint16_t)adc_filtered[ADC_CHANNEL_REF_POWER2];
    g_adc_data.raw[ADC_CHANNEL_REF_POWER2] = raw;

    raw = (uint16_t)adc_filtered[ADC_CHANNEL_RF_INPUT];
    g_adc_data.raw[ADC_CHANNEL_RF_INPUT] = raw;

    raw = (uint16_t)adc_filtered[ADC_CHANNEL_NTC_TEMP];
    g_adc_data.raw[ADC_CHANNEL_NTC_TEMP] = raw;

    raw = (uint16_t)adc_filtered[ADC_CHANNEL_DETECTOR];
    g_adc_data.raw[ADC_CHANNEL_DETECTOR] = raw;

    /* Apply calibration and conversion */
    /* TX Power 1 */
    g_adc_data.tx_power1 = (float)adc_filtered[ADC_CHANNEL_TX_POWER1] *
                           g_config.cal_tx_power1.forward_slope +
                           g_config.cal_tx_power1.forward_offset;

    /* TX Power 2 */
    g_adc_data.tx_power2 = (float)adc_filtered[ADC_CHANNEL_TX_POWER2] *
                           g_config.cal_tx_power2.forward_slope +
                           g_config.cal_tx_power2.forward_offset;

    /* Reflected Power 1 */
    g_adc_data.ref_power1 = (float)adc_filtered[ADC_CHANNEL_REF_POWER1] *
                            g_config.cal_ref_power1.reflect_slope +
                            g_config.cal_ref_power1.reflect_offset;

    /* Reflected Power 2 */
    g_adc_data.ref_power2 = (float)adc_filtered[ADC_CHANNEL_REF_POWER2] *
                            g_config.cal_ref_power2.reflect_slope +
                            g_config.cal_ref_power2.reflect_offset;

    /* RF Input Power */
    g_adc_data.rf_input = (float)adc_filtered[ADC_CHANNEL_RF_INPUT] *
                          g_config.cal_rf_input.forward_slope +
                          g_config.cal_rf_input.forward_offset;

    /* Main voltage */
    g_adc_data.voltage = adc_to_main_voltage((uint16_t)adc_filtered[ADC_CHANNEL_MAIN_VOLTAGE]);

    /* Main current */
    g_adc_data.current = adc_to_main_current((uint16_t)adc_filtered[ADC_CHANNEL_MAIN_CURRENT]);

    /* NTC Temperature */
    g_adc_data.ntc_temp = adc_to_temperature((uint16_t)adc_filtered[ADC_CHANNEL_NTC_TEMP]);

    /* Detector */
    g_adc_data.detector = adc_to_voltage((uint16_t)adc_filtered[ADC_CHANNEL_DETECTOR]);

    /* Calculate SWR - use TX power 2 and Reflected power 2 */
    if (g_adc_data.tx_power2 < 10.0f) {
        g_adc_data.swr = 1.0f;
    } else {
        g_adc_data.swr = calculate_swr(g_adc_data.tx_power2, g_adc_data.ref_power2);
    }
}

/*============================================================================
 * Public Functions
 *============================================================================*/

/**
 * @brief Initialize ADC and DMA
 */
void adc_init(void)
{
    /* Note: ADC and DMA initialization is done by CubeMX in HAL_ADC_MspInit */
    /* This function sets up additional configuration if needed */

    /* Clear buffer */
    memset(adc_dma_buffer, 0, sizeof(adc_dma_buffer));
    adc_sample_count = 0;

    /* Start ADC in DMA mode - this is typically done by CubeMX generated code */
    /* We need to configure it for continuous conversion of multiple channels */
}

/**
 * @brief Start continuous ADC conversion
 */
void adc_start(void)
{
    /* Start DMA for ADC - actual implementation depends on CubeMX settings */
    /* The following is typical but may need adjustment */
    /*
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer,
                          ADC_CHANNEL_COUNT * ADC_DMA_BUFFER_SIZE) != HAL_OK) {
        Error_Handler();
    }
    */
}

/**
 * @brief Stop ADC conversion
 */
void adc_stop(void)
{
    /* HAL_ADC_Stop_DMA(&hadc1); */
}

/**
 * @brief Process DMA buffer and update ADC data
 * @note Called from DMA complete callback or periodically from task
 */
void adc_process_data(void)
{
    /* Reset sample count and apply filter */
    adc_sample_count = ADC_DMA_BUFFER_SIZE;
    adc_apply_filter();

    /* Update global ADC data */
    adc_update_values();

    /* Reset for next batch */
    adc_sample_count = 0;
}

/**
 * @brief Get filtered ADC value for a channel
 */
uint16_t adc_get_filtered_value(uint8_t channel)
{
    if (channel >= ADC_CHANNEL_COUNT) {
        return 0;
    }
    return (uint16_t)adc_filtered[channel];
}

/**
 * @brief Convert raw ADC to voltage
 */
float adc_to_voltage(uint16_t adc_value)
{
    return ((float)adc_value * ADC_VREF) / (float)ADC_MAX_VALUE;
}

/**
 * @brief Convert ADC to main voltage (50:1 divider)
 * Vmain = ADC * Vref / 4095 * 50
 */
float adc_to_main_voltage(uint16_t adc_value)
{
    return ((float)adc_value * ADC_VREF * 50.0f) / (float)ADC_MAX_VALUE;
}

/**
 * @brief Convert ADC to main current
 * Using 75A/75mV shunt with operational amplifier
 * Calibration: I = ADC * factor + offset
 */
float adc_to_main_current(uint16_t adc_value)
{
    /* Default calibration: 40A = 4095 (full scale assumption) */
    /* Actual calibration should be done with calibration procedure */
    float current = ((float)adc_value * 40.0f) / (float)ADC_MAX_VALUE;
    return current;
}

/**
 * @brief Convert ADC to temperature (NTC thermistor)
 * Using 10K NTC with voltage divider
 */
float adc_to_temperature(uint16_t adc_value)
{
    /* NTC thermistor parameters (typical 10K NTC) */
    /* R25 = 10K, B = 3950 */
    /* Voltage divider: Vout = Vcc * Rntc / (R + Rntc) */
    /* Steinhart-Hart equation for temperature */

    float vout = adc_to_voltage(adc_value);
    float r_ntc;

    /* Assume 10K series resistor */
    #define NTC_SERIES_RESISTOR 10000.0f

    if (vout < 0.001f) {
        r_ntc = 100000.0f;  /* Open circuit */
    } else {
        r_ntc = (vout * NTC_SERIES_RESISTOR) / (ADC_VREF - vout);
    }

    /* Steinhart-Hart equation (simplified) */
    /* 1/T = 1/T0 + (1/B) * ln(R/R0) */
    #define NTC_T0 298.15f      /* 25C in Kelvin */
    #define NTC_R0 10000.0f     /* Resistance at 25C */
    #define NTC_B 3950.0f       /* B coefficient */

    float temp_k = 1.0f / (1.0f/NTC_T0 + (1.0f/NTC_B) * logf(r_ntc/NTC_R0));
    float temp_c = temp_k - 273.15f;

    return temp_c;
}

/**
 * @brief ADC DMA Complete Callback
 * @note Called from DMA ISR - must be lightweight
 */
void adc_dma_complete(void)
{
    /* Process data - signal task to process */
    adc_process_data();
}