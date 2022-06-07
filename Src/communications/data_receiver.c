#include "communications/data_receiver.h"
#include "device_drivers/nrf24l01.h"

void dataReceiver_Init(SPI_HandleTypeDef *hspi) {
	uint8_t address[] = { 0xEE, 0xDD, 0xCC, 0xBB, 0xAA };

	nrf24l01_Init(hspi);
	nrf24l01_Select_rx_mode(hspi, address, 10);
}

uint8_t dataReceiver_ReceiveData(SPI_HandleTypeDef *hspi, uint8_t *data) {

	if (nrf24l01_IsDataAvailable(hspi, 1) == 1) {
		nrf24l01_Receive(hspi, data);
		return 1;
	}

	return 0;
}
