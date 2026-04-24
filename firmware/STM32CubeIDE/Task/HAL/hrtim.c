/*
 * hrtim.c
 *
 *  Created on: May 10, 2025
 *      Author: Александр
 */

#include "main.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "hrtim.h"

#define VREF               3.3f
#define ADC_MAX            4095
#define ADC_MID_REF        (uint16_t)(ADC_MAX / 2)

extern HRTIM_HandleTypeDef hhrtim1;
extern DMA_HandleTypeDef hdma_hrtim1_a;

// Буфери PWM для DMA (double-buffer)
uint16_t pwm_buffer[2][PWM_CHANNELS][PWM_BUFFER_LEN];
volatile bool pwm_buffer_ready[2] = {false, false};

SignalParams signal_params[PWM_CHANNELS];
float pid_error[PWM_CHANNELS];
float pid_integral[PWM_CHANNELS];














//uint8_t active_pwm_buf = 0;  // 0 або 1
//
//void HRTIM1_Master_IRQHandler(void)
//{
//    if (__HAL_HRTIM_GET_IT(&hhrtim1, HRTIM_IT_REP)) {
//        __HAL_HRTIM_CLEAR_IT(&hhrtim1, HRTIM_IT_REP);
//
//        // Вимкнути попередній PWM вихід
//        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TC1);
//
//        // Перемкнути DMA буфер
//        active_pwm_buf ^= 1;
//
//        // Перезапустити DMA Burst Transfer
//        HAL_HRTIM_WaveformDMABurstWriteStart(
//            &hhrtim1,
//            HRTIM_TIMERID_TIMER_C,
//            HRTIM_DMABURST_UPDATE_CMP1,
//            HRTIM_BURSTMODE_BASEADDR_CMP1,
//            (uint32_t*)pwm_buffer[active_pwm_buf][0],
//            PWM_BUFFER_LEN
//        );
//
//        // Запустити PWM на новому буфері
//        HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TC1);
//    }
//}
//
//
///* Callback DMA HRTIM */
//void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx) {
//    static uint8_t active_buf = 0;
//    if (pwm_buffer_ready[active_buf]) {
//        // Перемикаємо DMA на інший буфер
//        active_buf ^= 1;
//        __HAL_DMA_DISABLE(hhrtim->hdma[TIMER_A]);
//        hhrtim->Instance->sTimerxRegs[0].CMP1xR = (uint32_t)pwm_buffer[active_buf][0];
//        __HAL_DMA_ENABLE(hhrtim->hdma[TIMER_A]);
//        pwm_buffer_ready[active_buf] = false;
//    }
//}

