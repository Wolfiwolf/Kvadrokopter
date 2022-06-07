#ifndef NRF24l01_H_FILE
#define NRF24l01_H_FILE

#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"

void nrf24l01_Init(SPI_HandleTypeDef *hspi);

void nrf24l01_Select_tx_mode(SPI_HandleTypeDef *hspi, uint8_t *address, uint8_t channel);

void nrf24l01_Select_rx_mode(SPI_HandleTypeDef *hspi, uint8_t *address, uint8_t channel);

uint8_t nrf24l01_Transmit(SPI_HandleTypeDef *hspi, uint8_t *data);

uint8_t nrf24l01_IsDataAvailable(SPI_HandleTypeDef *hspi, uint32_t pipeNum);

void nrf24l01_Receive(SPI_HandleTypeDef *hspi, uint8_t *data);

#endif
