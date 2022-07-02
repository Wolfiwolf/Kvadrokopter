#include <device_drivers/l3g4200d.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define L3G4200D_SLAVE_ADDRESS_WRITE	0xD2
#define L3G4200D_SLAVE_ADDRESS_READ		0xD3

void l3g4200d_init(I2C_HandleTypeDef *i2c_handle) {
	HAL_Delay(500);
	uint8_t data[2];

	// CTRL_REG1
	data[0] = 0x20;
	data[1] = 0xCF;

	HAL_I2C_Master_Transmit(i2c_handle, L3G4200D_SLAVE_ADDRESS_WRITE, data, 2,
			500);

	// CTRL_REG2
	data[0] = 0x21;
	data[1] = 0x00;

	HAL_I2C_Master_Transmit(i2c_handle, L3G4200D_SLAVE_ADDRESS_WRITE, data, 2,
			500);

	// CTRL_REG3
	data[0] = 0x22;
	data[1] = 0x00;

	HAL_I2C_Master_Transmit(i2c_handle, L3G4200D_SLAVE_ADDRESS_WRITE, data, 2,
			500);

	// CTRL_REG4
	data[0] = 0x23;
	data[1] = 0x30;

	HAL_I2C_Master_Transmit(i2c_handle, L3G4200D_SLAVE_ADDRESS_WRITE, data, 2,
			500);

	// CTRL_REG5
	data[0] = 0x24;
	// data[1] = 0x10;  // HIGH PASS FILTERED
	// data[1] = 1 << 1;  // LOW PASS FILTERED
	data[1] = (1 << 1) | (1 << 2);  // HIGH AND LOW PASS FILTERED

	HAL_I2C_Master_Transmit(i2c_handle, L3G4200D_SLAVE_ADDRESS_WRITE, data, 2,
			500);
}

static double x_Kg = 0.0;
static double x_Ee = 0.5;
static double x_prev_Ee = 0.5;
static double x_Em = 0.01;
static double x_prev_val = 0.0;

static double y_Kg = 0.0;
static double y_Ee = 0.5;
static double y_prev_Ee = 0.5;
static double y_Em = 0.01;
static double y_prev_val = 0.0;

static double z_Kg = 0.0;
static double z_Ee = 0.5;
static double z_prev_Ee = 0.5;
static double z_Em = 0.01;
static double z_prev_val = 0.0;

struct GyroAxisData l3g4200d_get_axis_data(I2C_HandleTypeDef *i2c_handle) {
	struct GyroAxisData axisData;

	uint8_t data[7];

	// CHECKING STATUS REGISTER
	data[0] = 0x27;
	data[1] = 0x00;

	while ((data[1] & 1 << 3) != 1 << 3) {
		HAL_I2C_Master_Transmit(i2c_handle, L3G4200D_SLAVE_ADDRESS_WRITE, data,
				1, 500);
		HAL_I2C_Master_Receive(i2c_handle, L3G4200D_SLAVE_ADDRESS_READ,
				&data[1], 1, 500);
	}

	// GETTING X AXIS
	data[0] = 0x28;
	data[1] = 0x00;
	HAL_I2C_Master_Transmit(i2c_handle, L3G4200D_SLAVE_ADDRESS_WRITE, data, 1,
			500);
	HAL_I2C_Master_Receive(i2c_handle, L3G4200D_SLAVE_ADDRESS_READ, &data[1], 1,
			500);

	data[0] = 0x29;
	data[2] = 0x00;
	HAL_I2C_Master_Transmit(i2c_handle, L3G4200D_SLAVE_ADDRESS_WRITE, data, 1,
			500);
	HAL_I2C_Master_Receive(i2c_handle, L3G4200D_SLAVE_ADDRESS_READ, &data[2], 1,
			500);

	// GETTING Y AXIS
	data[0] = 0x2A;
	data[3] = 0x00;
	HAL_I2C_Master_Transmit(i2c_handle, L3G4200D_SLAVE_ADDRESS_WRITE, data, 1,
			500);
	HAL_I2C_Master_Receive(i2c_handle, L3G4200D_SLAVE_ADDRESS_READ, &data[3], 1,
			500);

	data[0] = 0x2B;
	data[4] = 0x00;
	HAL_I2C_Master_Transmit(i2c_handle, L3G4200D_SLAVE_ADDRESS_WRITE, data, 1,
			500);
	HAL_I2C_Master_Receive(i2c_handle, L3G4200D_SLAVE_ADDRESS_READ, &data[4], 1,
			500);

	// GETTING Z AXIS
	/*
	 data[0] = 0x2C;
	 data[3] = 0x00;
	 HAL_I2C_Master_Transmit(i2c_handle, L3G4200D_SLAVE_ADDRESS_WRITE, data, 1,
	 500);
	 HAL_I2C_Master_Receive(i2c_handle, L3G4200D_SLAVE_ADDRESS_READ, &data[5], 1,
	 500);

	 data[0] = 0x2D;
	 data[4] = 0x00;
	 HAL_I2C_Master_Transmit(i2c_handle, L3G4200D_SLAVE_ADDRESS_WRITE, data, 1,
	 500);
	 HAL_I2C_Master_Receive(i2c_handle, L3G4200D_SLAVE_ADDRESS_READ, &data[6], 1,
	 500);
	 */

	int16_t raw_x = (data[2] << 8) | data[1];
	int16_t raw_y = (data[4] << 8) | data[3];
	// int16_t raw_z = (data[6] << 8) | data[5];

	float x = (0.07f * raw_x) + 0.5f;
	float y = (0.07f * raw_y) - 0.1f;
	// float z = (0.07f * raw_z) + 0.0f;

	x_Kg = x_Ee / (x_Ee + x_Em);
	float x_new_val = x_prev_val + x_Kg * (x - x_prev_val);
	double x_temp_prev_Ee = x_prev_Ee;
	x_prev_Ee = x_Ee;
	x_Ee = (1 - x_Kg) * x_temp_prev_Ee;
	x_prev_val = x_new_val;

	y_Kg = y_Ee / (y_Ee + y_Em);
	float y_new_val = y_prev_val + y_Kg * (y - y_prev_val);
	double y_temp_prev_Ee = y_prev_Ee;
	y_prev_Ee = y_Ee;
	y_Ee = (1 - y_Kg) * y_temp_prev_Ee;
	y_prev_val = y_new_val;

	/*
	 z_Kg = z_Ee / (z_Ee + z_Em);
	 float z_new_val = z_prev_val + z_Kg * ((z) - z_prev_val);
	 double z_temp_prev_Ee = z_prev_Ee;
	 z_prev_Ee = z_Ee;
	 z_Ee = (1 - z_Kg) * z_temp_prev_Ee;
	 z_prev_val = z_new_val;
	 */

	axisData.x = x - 1.0f;
	axisData.y = y - 0.15f;
	axisData.z = 0.0f;

	return axisData;
}

