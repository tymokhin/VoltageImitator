/*
 * hrtim.h
 *
 *  Created on: May 10, 2025
 *      Author: Александр
 */

#ifndef HAL_HRTIM_H_
#define HAL_HRTIM_H_

#define PWM_BUFFER_LEN     128  // Кількість точок в одному періоді
#define PWM_CHANNELS       4     // Кількість каналів

extern HRTIM_HandleTypeDef hhrtim1;

static inline void StartHRTIM_DMA(uint32_t Timers) { HAL_HRTIM_WaveformCounterStart_DMA(&hhrtim1, Timers ); }
static inline void StartHRTIMoutput(uint32_t Output) { HAL_HRTIM_WaveformOutputStart(&hhrtim1, Output);  }

// Структура параметрів сигналу
typedef struct {
    float A;
    float alpha;
    float phi;
    float freq;
    float dt;
} SignalParams;


#endif /* HAL_HRTIM_H_ */
