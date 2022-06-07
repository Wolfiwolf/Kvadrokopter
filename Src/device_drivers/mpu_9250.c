#include "device_drivers/mpu_9250.h"

#include <math.h>

extern I2C_HandleTypeDef hi2c2;

#define MPU_ADDRESS 0xD0

static float prev_x_angle;
static float prev_y_angle;
static uint32_t prev_time;

void mpu9250_init() {
	prev_x_angle = 0.0f;
	prev_y_angle = 0.0f;
	prev_time = 0;

	uint8_t data = 0x0;
	HAL_I2C_Mem_Read(&hi2c2, MPU_ADDRESS, 0x75, 1, &data, 1, 1000);

	if (data != 0x70) {
		HAL_I2C_Mem_Read(&hi2c2, MPU_ADDRESS, 0x75, 1, &data, 1, 1000);
		if (data != 0x70) {
			HAL_I2C_Mem_Read(&hi2c2, MPU_ADDRESS, 0x75, 1, &data, 1, 1000);
			if (data != 0x70) {
				return;
			}
		}
	}

	// WRITE SO THE DEVICE WAKES UP
	data = 0x0;
	HAL_I2C_Mem_Write(&hi2c2, MPU_ADDRESS, 0x6B, 1, &data, 1, 1000);

	// DIVIDING SAMPLERATE TO 0
	data = 0x0;
	HAL_I2C_Mem_Write(&hi2c2, MPU_ADDRESS, 0x6B, 1, &data, 1, 1000);

	// ACCELOMETER CONFIGURATION
	data = 0x0;
	HAL_I2C_Mem_Write(&hi2c2, MPU_ADDRESS, 0x1C, 1, &data, 1, 1000);

	// ACCELOMETER CONFIGURATION 2
	data = 0x00;
	HAL_I2C_Mem_Write(&hi2c2, MPU_ADDRESS, 0x1D, 1, &data, 1, 1000);
	/*
	 */

	// GYRO
	data = 0x00;
	HAL_I2C_Mem_Write(&hi2c2, MPU_ADDRESS, 0x1A, 1, &data, 1, 1000);
	// GYRO CONFIGURATION
	data = 0x18;
	HAL_I2C_Mem_Write(&hi2c2, MPU_ADDRESS, 0x1B, 1, &data, 1, 1000);

}

static const float radian_to_degrees = 57.295779513;

struct Angles mpu9250_get_axis_data() {
	struct Angles angles;
	angles.x_angle = 0.0f;
	angles.y_angle = 0.0f;

	uint8_t data[6];
	HAL_I2C_Mem_Read(&hi2c2, MPU_ADDRESS, 0x3B, 1, data, 6, 6000);

	int16_t x = (int16_t) ((data[0] << 8) + data[1]);
	int16_t y = (int16_t) ((data[2] << 8) + data[3]);
	int16_t z = (int16_t) ((data[4] << 8) + data[5]);

	float x_acc = (x / 16384.0f) + 0.158;
	float y_acc = (y / 16384.0f) - 0.428f;
	float z_acc = z / 16384.0f;

	z_acc = -z_acc;

	float accelerometer_angle_x = asinf(
			x_acc / (sqrtf(powf(z_acc, 2) + powf(x_acc, 2))))
			* radian_to_degrees;
	float accelerometer_angle_y = asinf(
			y_acc / (sqrtf(powf(z_acc, 2) + powf(y_acc, 2))))
			* radian_to_degrees;

	HAL_I2C_Mem_Read(&hi2c2, MPU_ADDRESS, 0x43, 1, data, 6, 6000);

	x = (int16_t) ((data[0] << 8) + data[1]);
	y = (int16_t) ((data[2] << 8) + data[3]);
	z = (int16_t) ((data[4] << 8) + data[5]);

	float x_gy = (x / 16.4f) - 5.4;
	float y_gy = (y / 16.4f) - 5.25;

	float delta_time = (HAL_GetTick() - prev_time) / 1000.0f;
	float gyro_angle_x = -x_gy * delta_time + prev_y_angle;
	float gyro_angle_y = y_gy * delta_time + prev_x_angle;
	prev_time = HAL_GetTick();

	angles.x_angle = 0.98f * gyro_angle_y + 0.02f * accelerometer_angle_x;
	angles.y_angle = 0.98f * gyro_angle_x + 0.02f * accelerometer_angle_y;

	prev_x_angle = angles.x_angle;
	prev_y_angle = angles.y_angle;

	angles.x_angle -= 2.0f;
	angles.y_angle -= 1.0f;

	// FOR TESTING
	angles.x_angle = x_gy;
	angles.y_angle = y_acc * 100;
	// END TESTING

	return angles;
}
