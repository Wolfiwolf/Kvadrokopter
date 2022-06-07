#include "device_drivers/bno055.h"

extern I2C_HandleTypeDef hi2c1;

#define BNO055_ADDRESS 0x28<<1



void BNO055_Init() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
	HAL_Delay(500);

	// I2C
	uint8_t data[1];
	data[0] = 0x00;
	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDRESS, 0x00, 1, data, 1, 1000);

	if (data[0] != 160) {
		return;
	}

	// POWER MODE -> NORMAL MODE
	data[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, BNO055_ADDRESS, 0x3E, 1, data, 1, 1000);

	// SETTING WINDOWS MODE
	data[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, BNO055_ADDRESS, 0x3B, 1, data, 1, 1000);

	// OPERATING MODE -> NDOF
	data[0] = (1<<3) | (1<<2);
	HAL_I2C_Mem_Write(&hi2c1, BNO055_ADDRESS, 0x3D, 1, data, 1, 1000);
	HAL_Delay(15); // WAIT FOR OPERATING MODE SWITCH
	/*
	*/

}


static float x_vals[5] = {0};
static float y_vals[5] = {0};

static void order_array(float *arr) {
	for(int i = 0; i < 3; ++i) {
		for(int p = 0; p < 2; ++p) {
			if (arr[i + 1] > arr[i]) {
				float temp = arr[i + 1];
				arr[i + 1] = arr[i];
				arr[i] = temp;
			}
		}
	}
}

static void copy_array(float *arr_src, float *arr_dest) {
	for(int i = 0; i < 3; ++i) {
		arr_dest[i] = arr_src[i];
	}
}

struct Angles BNO055_Get_angles() {
	struct Angles angles;
	angles.x_angle = 0.0f;
	angles.y_angle = 0.0f;


	// I2C
	uint8_t data[2];
	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDRESS, 0x1C, 1, data, 2, 2000);
	angles.x_angle = ((int16_t)(data[0] | (data[1] << 8))) / 16.0f;

	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDRESS, 0x1E, 1, data, 2, 2000);
	angles.y_angle = ((int16_t)(data[0] | (data[1] << 8))) / 16.0f;

	x_vals[4] = x_vals[3];
	x_vals[3] = x_vals[2];
	x_vals[2] = x_vals[1];
	x_vals[1] = x_vals[0];
	x_vals[0] = angles.x_angle;

	y_vals[4] = y_vals[3];
	y_vals[3] = y_vals[2];
	y_vals[2] = y_vals[1];
	y_vals[1] = y_vals[0];
	y_vals[0] = angles.y_angle;

	float x_values[5];
	copy_array(x_vals, x_values);
	float y_values[5];
	copy_array(y_vals, y_values);

	order_array(x_values);
	order_array(y_values);

	angles.x_angle = y_values[2];
	angles.y_angle = -x_values[2];


	return angles;
}

