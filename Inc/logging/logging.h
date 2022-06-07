#ifndef LOGGING_H_FILE
#define LOGGING_H_FILE

#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"

#include "device_drivers/File_Handling_RTOS.h"

void logging_Init(UART_HandleTypeDef *uart_handle);

void logging_SendData(UART_HandleTypeDef *uart_handle, int angle_x, int angle_y, int m1, int m2, int m3, int m4);

#endif
