#include "device_drivers/lsm303agr.h"
#include <stdio.h>
void lsm303agr_init(I2C_HandleTypeDef *hi2c) {
	uint8_t address = 0x19 << 1;
	uint8_t buffer[3];

	HAL_Delay(2000);

	buffer[0] = 0x0;
	HAL_I2C_Mem_Write(hi2c, address, 0x21, 1, buffer, 1, 1000);

	buffer[0] = 0x00;
	HAL_I2C_Mem_Write(hi2c, address, 0x22, 1, buffer, 1, 1000);

	buffer[0] = 0x81;
	HAL_I2C_Mem_Write(hi2c, address, 0x23, 1, buffer, 1, 1000);

	buffer[0] = 0x97;
	HAL_I2C_Mem_Write(hi2c, address, 0x20, 1, buffer, 1, 1000);
	HAL_Delay(90);
}

static double x_Kg = 0.0;
static double x_Ee = 0.5;
static double x_prev_Ee = 0.5;
static double x_Em = 0.03;
static double x_prev_val = 0.0;

static double y_Kg = 0.0;
static double y_Ee = 0.5;
static double y_prev_Ee = 0.5;
static double y_Em = 0.03;
static double y_prev_val = 0.0;

static double z_Kg = 0.0;
static double z_Ee = 0.5;
static double z_prev_Ee = 0.5;
static double z_Em = 0.03;
static double z_prev_val = 0.0;

struct AccelerometerAxisData lsm303agr_get_axis_data(I2C_HandleTypeDef *hi2c) {
	uint8_t address = 0x19 << 1;
	uint8_t buffer[3];

	uint8_t test_buffer[1];

	test_buffer[0] = 0x0;
	HAL_I2C_Mem_Read(hi2c, address, 0x0F, 1, test_buffer, 1, 1000);

	buffer[0] = 0x0;
	buffer[1] = 0x0;
	HAL_I2C_Mem_Read(hi2c, address, 0x28, 1, buffer, 1, 1000);
	HAL_I2C_Mem_Read(hi2c, address, 0x29, 1, &buffer[1], 1, 1000);

	int16_t xAxis = buffer[1];
	xAxis <<= 8;
	xAxis |= buffer[0];
	xAxis >>= 6;

	float x = xAxis;
	x /= 252.0f;

	buffer[0] = 0x0;
	buffer[1] = 0x0;
	HAL_I2C_Mem_Read(hi2c, address, 0x2A, 1, buffer, 1, 1000);
	HAL_I2C_Mem_Read(hi2c, address, 0x2B, 1, &buffer[1], 1, 1000);

	int16_t yAxis = buffer[1];
	yAxis <<= 8;
	yAxis |= buffer[0];
	yAxis >>= 6;

	float y = yAxis;
	y /= 252.0f;

	buffer[0] = 0x0;
	buffer[1] = 0x0;
	HAL_I2C_Mem_Read(hi2c, address, 0x2C, 1, buffer, 1, 1000);
	HAL_I2C_Mem_Read(hi2c, address, 0x2D, 1, &buffer[1], 1, 1000);

	int16_t zAxis = buffer[1];
	zAxis <<= 8;
	zAxis |= buffer[0];
	zAxis >>= 6;

	float z = zAxis;
	z /= 252.0f;


	x_Kg = x_Ee / (x_Ee + x_Em);
	float x_new_val = x_prev_val + x_Kg * (x - x_prev_val);
	double x_temp_prev_Ee = x_prev_Ee;
	x_prev_Ee = x_Ee;
	x_Ee = (1 - x_Kg) * x_temp_prev_Ee;
	x_prev_val = x_new_val;

	y_Kg = y_Ee / (y_Ee + y_Em);
	float y_new_val = y_prev_val + y_Kg * ((y + 0.03f) - y_prev_val);
	double y_temp_prev_Ee = y_prev_Ee;
	y_prev_Ee = y_Ee;
	y_Ee = (1 - y_Kg) * y_temp_prev_Ee;
	y_prev_val = y_new_val;

	z_Kg = z_Ee / (z_Ee + z_Em);
	float z_new_val = z_prev_val + z_Kg * (z - z_prev_val);
	double z_temp_prev_Ee = z_prev_Ee;
	z_prev_Ee = z_Ee;
	z_Ee = (1 - z_Kg) * z_temp_prev_Ee;
	z_prev_val = z_new_val;

	struct AccelerometerAxisData axisData;
	axisData.x = x_new_val;
	axisData.y = y_new_val;
	axisData.z = z_new_val;

	return axisData;
}
/*
 */
