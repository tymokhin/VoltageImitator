/*
 * adc.h
 *
 *  Created on: May 10, 2025
 *      Author: Александр
 */

#ifndef HAL_ADC_H_
#define HAL_ADC_H_

extern ADC_HandleTypeDef hadc1;

#define ADC_BUFFER_LENGTH  128  // Кратне 2, бо half-complete/complete

extern uint16_t adc_dma_buffer[2][ADC_BUFFER_LENGTH];

#endif /* HAL_ADC_H_ */
