#include "device_drivers/nrf24l01.h"

static void dissable_chip() {
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_Delay(20);
}

static void enable_chip() {
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_Delay(20);
}

static void select_chip() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_Delay(5);
}

static void unselect_chip() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_Delay(5);
}

static void write_to_reg(SPI_HandleTypeDef *hspi, uint8_t regAddress,
		uint8_t *data, uint8_t dataSize) {
	uint8_t buffer[2];
	buffer[0] = regAddress | 1 << 5;

	select_chip();
	HAL_SPI_Transmit(hspi, buffer, 1, 1000);
	HAL_SPI_Transmit(hspi, data, dataSize, 1000);
	unselect_chip();
}

static void read_reg(SPI_HandleTypeDef *hspi, uint8_t regAddress, uint8_t *data,
		uint8_t dataSize) {
	select_chip();
	HAL_SPI_Transmit(hspi, &regAddress, 1, 1000);
	HAL_SPI_Receive(hspi, data, dataSize, 1000);
	unselect_chip();
}

void nrf24l01_Init(SPI_HandleTypeDef *hspi) {
	dissable_chip();
	uint8_t data = 0x00;

	uint8_t fifoReg = 0x00;
	read_reg(hspi, 0x17, &fifoReg, 1);

	data = 0x00;
	write_to_reg(hspi, 0x00, &data, 1);

	write_to_reg(hspi, 0x01, &data, 1);

	write_to_reg(hspi, 0x02, &data, 1);

	data = 0x03;
	write_to_reg(hspi, 0x03, &data, 1);

	data = 0x00;
	write_to_reg(hspi, 0x04, &data, 1);

	data = 0x00;
	write_to_reg(hspi, 0x05, &data, 1);

	data = 0x0E;
	write_to_reg(hspi, 0x06, &data, 1);

	enable_chip();
}

void nrf24l01_Select_tx_mode(SPI_HandleTypeDef *hspi, uint8_t *address,
		uint8_t channel) {

	dissable_chip();
	unselect_chip();

	write_to_reg(hspi, 0x05, &channel, 1);
	write_to_reg(hspi, 0x10, address, 5);

	uint8_t configReg;
	read_reg(hspi, 0x00, &configReg, 1);
	configReg = configReg | 1 << 1;
	write_to_reg(hspi, 0x00, &configReg, 1);

	enable_chip();
}

uint8_t nrf24l01_Transmit(SPI_HandleTypeDef *hspi, uint8_t *data) {
	select_chip();

	uint8_t cmd = 0xA0;
	HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(hspi, data, 32, HAL_MAX_DELAY);

	unselect_chip();

	HAL_Delay(2);

	uint8_t fifoReg = 0x00;
	read_reg(hspi, 0x17, &fifoReg, 1);

	if (fifoReg & (1 << 4) && (!(fifoReg & (1 << 3)))) {
		cmd = 0xE1;
		HAL_SPI_Transmit(hspi, &cmd, 1, 1000);
		return 1;
	}

	return 0;
}

void nrf24l01_Select_rx_mode(SPI_HandleTypeDef *hspi, uint8_t *address,
		uint8_t channel) {

	dissable_chip();
	unselect_chip();

	write_to_reg(hspi, 0x05, &channel, 1);

	uint8_t rxRegister;
	read_reg(hspi, 0x02, &rxRegister, 1);
	rxRegister = rxRegister | (1 << 1);
	write_to_reg(hspi, 0x02, &rxRegister, 1);

	write_to_reg(hspi, 0x0B, address, 5);

	uint8_t data = 32;
	write_to_reg(hspi, 0x12, &data, 1);

	uint8_t configReg;
	read_reg(hspi, 0x00, &configReg, 1);
	configReg = configReg | (1 << 1) | (1 << 0);
	write_to_reg(hspi, 0x00, &configReg, 1);

	enable_chip();
}

uint8_t nrf24l01_IsDataAvailable(SPI_HandleTypeDef *hspi, uint32_t pipeNum) {
	uint8_t statusReg = 0x00;
	read_reg(hspi, 0x07, &statusReg, 1);

	if (statusReg & (1 << 6) && (statusReg & (pipeNum << 1))) {
		uint8_t data = 0x00 | (1 << 6);
		write_to_reg(hspi, 0x07, &data, 1);
		return 1;
	}

	return 0;
}

void nrf24l01_Receive(SPI_HandleTypeDef *hspi, uint8_t *data) {
	select_chip();

	uint8_t cmd = 0x61;
	HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(hspi, data, 32, HAL_MAX_DELAY);

	unselect_chip();

	HAL_Delay(2);

	cmd = 0xE2;
	HAL_SPI_Transmit(hspi, &cmd, 1, 1000);
}
