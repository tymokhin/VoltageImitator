/*
 * adc.c
 *
 *  Created on: May 10, 2025
 *      Author: Александр
 */

#include "main.h"

#include "adc.h"

extern HRTIM_HandleTypeDef hhrtim1;
extern DMA_HandleTypeDef hdma_adc1;

uint16_t adc_dma_buffer[2][ADC_BUFFER_LENGTH]; // Структура [CH1_0, CH2_0, ..., CH1_N, CH2_N]

extern void ProcessAdcData(uint16_t* adc_buffer, uint16_t buf_length);

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        // Перша половина буфера готова: adc_dma_buffer[0] ... [63]
    	ProcessAdcData(&adc_dma_buffer[0][ADC_BUFFER_LENGTH], sizeof(adc_dma_buffer[0]) / 2);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        // Друга половина буфера готова: adc_dma_buffer[64] ... [127]
        ProcessAdcData(&adc_dma_buffer[1][ADC_BUFFER_LENGTH], sizeof(adc_dma_buffer[0]) / 2);
    }
}

///* ADC обробка (наприклад у HAL_ADC_ConvCpltCallback) */
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
//    if (hadc->Instance == ADC1) {
//        update_pid(adc_buffer, signal_params);
//    }
//}
