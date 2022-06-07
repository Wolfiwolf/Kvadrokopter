#ifndef DATA_RECEIVER_H_FILE
#define DATA_RECEIVER_H_FILE

#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"

void dataReceiver_Init(SPI_HandleTypeDef *hspi);

uint8_t dataReceiver_ReceiveData(SPI_HandleTypeDef *hspi, uint8_t *data);

#endif
