#ifndef L3GD20_H_FILE
#define L3GD20_H_FILE

#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"

struct GyroAxisData {
	float x, y, z;
};

void l3g4200d_init(I2C_HandleTypeDef *i2c_handle);

struct GyroAxisData l3g4200d_get_axis_data(I2C_HandleTypeDef *i2c_handle);

#endif
