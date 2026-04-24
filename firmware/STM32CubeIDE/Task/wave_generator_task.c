/*
 * wave_generator.c
 *
 *  Created on: May 9, 2025
 *      Author: Александр
 */

/* Ядро керування PWM+DMA+ADC+PID для STM32F334 */

#include "main.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "semphr.h"

/* Defines --------------------------------------------------------------*/
#define ADC_BUFFER_LENGTH 128
#define PWM_BUFFER_LEN 128
#define PWM_CHANNELS      4    // 4 канали
#define SAMPLE_RATE       100000UL  // частота дискретизації / PWM, Гц

/* Variables ------------------------------------------------------------*/

extern ADC_HandleTypeDef hadc1;
extern HRTIM_HandleTypeDef hhrtim1;

// Семафори для DMA callback'ів
SemaphoreHandle_t adcHalfBufSemaphore;
SemaphoreHandle_t adcFullBufSemaphore;

extern HRTIM_HandleTypeDef hhrtim1;
extern DMA_HandleTypeDef hdma_hrtim1_a;

#define RAMP_SAMPLES  500
#define TOTAL_SAMPLES (2 * RAMP_SAMPLES)

typedef struct{
	uint16_t l;
	uint16_t r;
} pwm_t;

//static uint16_t cmp_lut[2 * TOTAL_SAMPLES]; // [CMP1, CMP2] *
static pwm_t cmp_lut[TOTAL_SAMPLES] = {0};

/* Головний loop або FreeRTOS task */
void pwm_task(void *argument) {
//
//	/* FreeRTOS Semaphores */
//	adcHalfBufSemaphore = xSemaphoreCreateBinary();
//	adcFullBufSemaphore = xSemaphoreCreateBinary();
//
//	hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].TIMxDIER |= HRTIM_TIMDIER_UPDDE;
//
//	// 1) сформувати LUT
//	uint16_t per = hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].PERxR;
//	pwm_t* left = &cmp_lut[0];
//	pwm_t* left_back = &cmp_lut[RAMP_SAMPLES - 1];
//	pwm_t* right = &cmp_lut[RAMP_SAMPLES];
//	pwm_t* right_back = &cmp_lut[TOTAL_SAMPLES - 1];
//	/* ---- 1. TA1 = пилка, TA2 = 0 ---- */
////	for (uint32_t i = 0; i < RAMP_SAMPLES / 2; i++)
////	{
////		uint16_t ramp = (uint16_t)((per * 2 * i) / RAMP_SAMPLES);
////		ramp = per - ramp -1;
////
////		(left++)->l = ramp;
////		(left_back--)->l = ramp;
////		(right++)->r = ramp;
////		(right_back--)->r = ramp;
////	}
//
//	for (uint32_t i = 0; i < RAMP_SAMPLES / 2; i++)
//	{
//	    // i пробігає 0..(RAMP_SAMPLES/2 - 1)
//	    // це чверть періоду: 0..pi/2
//	    float a = (float)M_PI_2 * (float)i / (float)((RAMP_SAMPLES / 2) - 1);
//
//	    // амплітуда модуляції: 0..1 (можеш зробити 0.9 щоб не впиратись)
//	    float k = 0.95f;
//
//	    // "something" у діапазоні 0..k*per
//	    uint16_t s = (uint16_t)(k * (float)(per - 1) * sinf(a) + 0.5f);
//
//	    //if (i < RAMP_SAMPLES/10) s = (uint16_t)((1.0f + 0.5f*(1 - 10 * i / RAMP_SAMPLES )) * s);
//
//	    s = s > 0.05*per ? s : 0.05*per;
//
//	    // перетворюємо в CMP для твоєї логіки SET@CMP / RESET@PER
//	    uint16_t cmp = (uint16_t)((per - 1) - s);
//
//
//
//	    (left++)->l      = cmp;
//	    (left_back--)->l = cmp;
//
//	    (right++)->r      = cmp;
//	    (right_back--)->r = cmp;
//	}
//
//	// 2) запустити DMA: mem -> HRTIM BDMAR
//	if (HAL_DMA_Start(
//	      &hdma_hrtim1_a,
//	      (uint32_t)cmp_lut,
//	      (uint32_t)&hhrtim1.Instance->sCommonRegs.BDMADR,
//	      2 * TOTAL_SAMPLES) != HAL_OK)
//	{
//	  Error_Handler();
//	}
//
//	// 3) запустити PWM
//	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
//	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A);

	// Основний цикл: оновлюємо буфер лише при отриманні нового DMA пакета
	for (;;) {

	}
}

///* Головний loop або FreeRTOS task */
//void pwm_task(void *argument) {
//
//	/* FreeRTOS Semaphores */
//	adcHalfBufSemaphore = xSemaphoreCreateBinary();
//	adcFullBufSemaphore = xSemaphoreCreateBinary();
//
//	hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].TIMxDIER |= HRTIM_TIMDIER_UPDDE;
//
//
//	// 1) сформувати LUT
//	uint16_t per = hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].PERxR;
//	/* ---- 1. TA1 = пилка, TA2 = 0 ---- */
//	for (uint32_t i = 0; i < RAMP_SAMPLES; i++)
//	{
//		uint16_t ramp = (uint16_t)((per * i) / RAMP_SAMPLES);
//
//		cmp_lut[2*i + 0] = ramp; // CMP1 → TA1
//		cmp_lut[2*i + 1] = 0;    // CMP2 → TA2
//	}
//
//	/* ---- 2. TA1 = 0, TA2 = пилка ---- */
//	for (uint32_t i = 0; i < RAMP_SAMPLES; i++)
//	{
//		uint16_t ramp = (uint16_t)((per * i) / RAMP_SAMPLES);
//
//		uint32_t idx = RAMP_SAMPLES + i;
//		cmp_lut[2*idx + 0] = 0;    // CMP1 → TA1
//		cmp_lut[2*idx + 1] = ramp; // CMP2 → TA2
//	}
//
//	// 2) запустити DMA: mem -> HRTIM BDMAR
//	if (HAL_DMA_Start(
//	      &hdma_hrtim1_a,
//	      (uint32_t)cmp_lut,
//	      (uint32_t)&hhrtim1.Instance->sCommonRegs.BDMADR,
//	      2 * TOTAL_SAMPLES) != HAL_OK)
//	{
//	  Error_Handler();
//	}
//
//	// 3) запустити PWM
//	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
//	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A);
//
//	// Основний цикл: оновлюємо буфер лише при отриманні нового DMA пакета
//	volatile uint32_t d1, d2;
//	for (;;) {
//
//		d1 = hdma_hrtim1_a.Instance->CNDTR;
//		vTaskDelay(5);
//		d2 = hdma_hrtim1_a.Instance->CNDTR;
//		vTaskDelay(5);
////		// Перша половина буфера
////		if (xSemaphoreTake(adcHalfBufSemaphore, portMAX_DELAY) == pdTRUE) {
////			generate_pwm_buffer(pwm_buffer[0]);
////		}
////		// Друга половина буфера
////		if (xSemaphoreTake(adcFullBufSemaphore, portMAX_DELAY) == pdTRUE) {
////			generate_pwm_buffer(pwm_buffer[1]);
////		}
//	}
//}

void CreatePWMTask(){
	xTaskCreate(pwm_task, "PWM", configMINIMAL_STACK_SIZE * 5, NULL, configMAX_PRIORITIES - 1, NULL);
}



/* === Callbacks ADC DMA === */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(adcHalfBufSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(adcFullBufSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
