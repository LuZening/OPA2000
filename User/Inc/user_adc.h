/**
 * @file user_adc.h
 * @brief ADC Data Acquisition Header
 */

#ifndef USER_ADC_H
#define USER_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "opa2000.h"
#include "main.h"

/*============================================================================
 * External Variables
 *============================================================================*/

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc1;

/*============================================================================
 * ADC Configuration
 *============================================================================*/

/* DMA buffer size - enough for multiple samples */
#define ADC_DMA_BUFFER_SIZE    32

/* ADC values */
#define ADC_MAX_VALUE          4095    /* 12-bit ADC */
#define ADC_VREF               3.3f    /* Reference voltage */

/* Current measurement: 75A/75mV shunt = 1000:1 ratio */
/* Operational amplifier 3.3x gain */
/* ADC: V = I * 0.075 / 1000 * 3.3 / 3.3 = I * 0.000075 */
/* Actually: Vshunt * gain = ADC * Vref / 4095 */
/* I = ADC * Vref / 4095 / (gain * Rshunt) */

/* Voltage measurement: 50:1 divider */
/* ADC: V = ADC * Vref / 4095 * 50 */

/*============================================================================
 * Function Prototypes
 *============================================================================*/

/**
 * @brief Initialize ADC and DMA
 */
void adc_init(void);

/**
 * @brief Start continuous ADC conversion
 */
void adc_start(void);

/**
 * @brief Stop ADC conversion
 */
void adc_stop(void);

/**
 * @brief Process DMA buffer and update ADC data
 */
void adc_process_data(void);

/**
 * @brief Get filtered ADC value for a channel
 * @param channel Channel index
 * @return Filtered value
 */
uint16_t adc_get_filtered_value(uint8_t channel);

/**
 * @brief Convert raw ADC to voltage
 * @param adc_value Raw ADC value
 * @return Voltage in volts
 */
float adc_to_voltage(uint16_t adc_value);

/**
 * @brief Convert ADC to main voltage
 * @param adc_value Raw ADC value
 * @return Voltage in volts (50:1 divider compensation)
 */
float adc_to_main_voltage(uint16_t adc_value);

/**
 * @brief Convert ADC to main current
 * @param adc_value Raw ADC value
 * @return Current in amperes
 */
float adc_to_main_current(uint16_t adc_value);

/**
 * @brief Convert ADC to temperature (NTC thermistor)
 * @param adc_value Raw ADC value
 * @return Temperature in celsius
 */
float adc_to_temperature(uint16_t adc_value);

#ifdef __cplusplus
}
#endif

#endif /* ADC_H */