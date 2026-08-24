#ifndef UART_TASK_H_
#define UART_TASK_H_

#include "main.h"

typedef struct
{
    UART_HandleTypeDef *uart;
    IRQn_Type irqn;
} UartControlHardwareConfig;

void UartControl_BindHardware(const UartControlHardwareConfig *config);
void UartControl_Init(void);

#endif /* UART_TASK_H_ */
