#ifndef L3LSM303AGR_H_FILE
#define L3LSM303AGR_H_FILE

#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"

/*
*/
struct AccelerometerAxisData {
	float x, y, z;
};

void lsm303agr_init(I2C_HandleTypeDef *hi2c);
struct AccelerometerAxisData lsm303agr_get_axis_data(I2C_HandleTypeDef *hi2c);

#endif
