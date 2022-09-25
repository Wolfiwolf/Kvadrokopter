#include "device_drivers/ms5611.h"

extern I2C_HandleTypeDef hi2c2;

#define MS5611_I2C_ADDRESS 0x3B<<1

void MS5611_Init() {
	uint8_t data = 0x1E;
	HAL_I2C_Master_Transmit(&hi2c2, MS5611_I2C_ADDRESS, &data, 1, 1000);
	HAL_Delay(2000);
}

float MS5611_get_altitude() {

}
