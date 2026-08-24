/*
 * adc.c
 *
 *  Created on: May 10, 2025
 *      Author: Oleksandr
 *
 * Буфер adc_dma_buffer[2][128] прибраний: реальний буфер DMA живе в
 * wave_generator_task.c і має рівно 2 слова - стільки, скільки dual regular
 * simultaneous віддає за один тригер. Старий буфер на 512 байт ніде не
 * заповнювався.
 */

#include "main.h"
#include "adc.h"
