#include "device_drivers/mpu_9250.h"

#include <math.h>

extern I2C_HandleTypeDef hi2c1;

#define MPU_ADDRESS 0xD0

#define COMPLEMENTARY_ALPHA 0.9995f
// #define COMPLEMENTARY_ALPHA 0.0f
// #define COMPLEMENTARY_ALPHA 1.0f

static const float radian_to_degrees = 180.0 / 3.14159265;
static float prev_x_angle;
static float prev_y_angle;
static float prev_z_angle;
static uint32_t prev_time;

static float x_angle_accelometer_offset;
static float y_angle_accelometer_offset;
static float z_angle_accelometer_offset;

static float x_gyro_offset;
static float y_gyro_offset;
static float z_gyro_offset;

void mpu9250_init() {
	prev_x_angle = 0.0f;
	prev_y_angle = 0.0f;
	prev_z_angle = 0.0f;
	prev_time = HAL_GetTick();

	uint8_t data = 0x0;
	HAL_I2C_Mem_Read(&hi2c1, MPU_ADDRESS, 0x75, 1, &data, 1, 1000);

	if (data != 0x70) {
		HAL_I2C_Mem_Read(&hi2c1, MPU_ADDRESS, 0x75, 1, &data, 1, 1000);
		if (data != 0x70) {
			HAL_I2C_Mem_Read(&hi2c1, MPU_ADDRESS, 0x75, 1, &data, 1, 1000);
			if (data != 0x70) {
				return;
			}
		}
	}

	// WRITE SO THE DEVICE WAKES UP
	data = 0x0;
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDRESS, 0x6B, 1, &data, 1, 1000);

	// DIVIDING SAMPLERATE TO 0
	data = 0x0;
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDRESS, 0x6B, 1, &data, 1, 1000);

	// ACCELOMETER CONFIGURATION
	data = 0x0;
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDRESS, 0x1C, 1, &data, 1, 1000);

	// ACCELOMETER CONFIGURATION 2
	data = 0x02;
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDRESS, 0x1D, 1, &data, 1, 1000);
	/*
	 */

	// GYRO
	data = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDRESS, 0x1A, 1, &data, 1, 1000);
	// GYRO CONFIGURATION
	data = 0x18;
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDRESS, 0x1B, 1, &data, 1, 1000);

}

static float Kg_x = 0.0;
static float Ee_x = 0.5;
static float prev_Ee_x = 0.5;
static float Em_x = 0.1;
static float prev_val_x = 0.0;

static float Kg_y = 0.0;
static float Ee_y = 0.5;
static float prev_Ee_y = 0.5;
static float Em_y = 0.1;
static float prev_val_y = 0.0;

struct Angles mpu9250_get_axis_data() {
	struct Angles angles;
	angles.x_angle = 0.0f;
	angles.y_angle = 0.0f;

	uint8_t data[6];
	HAL_I2C_Mem_Read(&hi2c1, MPU_ADDRESS, 0x3B, 1, data, 6, 6000);

	int16_t x = (int16_t) ((data[0] << 8) + data[1]);
	int16_t y = (int16_t) ((data[2] << 8) + data[3]);
	int16_t z = (int16_t) ((data[4] << 8) + data[5]);

	float x_acc = (x / 16384.0f) - x_angle_accelometer_offset;
	float y_acc = (y / 16384.0f) - y_angle_accelometer_offset;
	float z_acc = (-z / 16384.0f) - z_angle_accelometer_offset;

	float accelerometer_angle_x = asinf(
			x_acc / (sqrtf(powf(z_acc, 2) + powf(x_acc, 2))))
			* radian_to_degrees;
	float accelerometer_angle_y = asinf(
			y_acc / (sqrtf(powf(z_acc, 2) + powf(y_acc, 2))))
			* radian_to_degrees;

	HAL_I2C_Mem_Read(&hi2c1, MPU_ADDRESS, 0x43, 1, data, 6, 6000);

	x = (int16_t) ((data[0] << 8) + data[1]);
	y = (int16_t) ((data[2] << 8) + data[3]);
	z = (int16_t) ((data[4] << 8) + data[5]);

	float x_gy = -(x / 16.4f) - x_gyro_offset;
	float y_gy = (y / 16.4f) - y_gyro_offset;
	float z_gy = (z / 16.4f) - z_gyro_offset;

	uint32_t current_time = HAL_GetTick();
	float delta_time = (current_time - prev_time) / 1000.0f;
	prev_time = current_time;
	float gyro_angle_x = x_gy * delta_time + prev_y_angle;
	float gyro_angle_y = y_gy * delta_time + prev_x_angle;
	float gyro_angle_z = z_gy * delta_time + prev_z_angle;

	float z_change = sin(z_gy * delta_time * (3.1415f / 180.0f));
	gyro_angle_x += gyro_angle_y * z_change;
	gyro_angle_y -= gyro_angle_x * z_change;

	angles.x_angle = COMPLEMENTARY_ALPHA * gyro_angle_y
			+ (1.0f - COMPLEMENTARY_ALPHA) * accelerometer_angle_x;
	angles.y_angle = COMPLEMENTARY_ALPHA * gyro_angle_x
			+ (1.0f - COMPLEMENTARY_ALPHA) * accelerometer_angle_y;
	angles.z_angle = gyro_angle_z;


	Kg_x = Ee_x / (Ee_x + Em_x);
	angles.x_angle = prev_val_x + Kg_x * (angles.x_angle - prev_val_x);
	double temp_prev_Ee_x = prev_Ee_x;
	prev_Ee_x = Ee_x;
	Ee_x = (1 - Kg_x) * temp_prev_Ee_x;
	prev_val_x = angles.x_angle;

	Kg_y = Ee_y / (Ee_y + Em_y);
	angles.y_angle = prev_val_y + Kg_y * (angles.y_angle - prev_val_y);
	double temp_prev_Ee_y = prev_Ee_y;
	prev_Ee_y = Ee_y;
	Ee_y = (1 - Kg_y) * temp_prev_Ee_y;
	prev_val_y = angles.y_angle;

	prev_x_angle = angles.x_angle;
	prev_y_angle = angles.y_angle;
	prev_z_angle = angles.z_angle;

	return angles;
}

static void calibrate_accelometer() {
	const float num_of_samples = 1000.0f;
	float x_sum = 0.0f, y_sum = 0.0f, z_sum = 0.0f;
	for (int i = 0; i < num_of_samples; ++i) {
		uint8_t data[6];
		HAL_I2C_Mem_Read(&hi2c1, MPU_ADDRESS, 0x3B, 1, data, 6, 6000);

		int16_t x = (int16_t) ((data[0] << 8) + data[1]);
		int16_t y = (int16_t) ((data[2] << 8) + data[3]);
		int16_t z = (int16_t) ((data[4] << 8) + data[5]);

		float x_acc = (x / 16384.0f);
		float y_acc = (y / 16384.0f);
		float z_acc = (-z / 16384.0f);

		x_sum += x_acc;
		y_sum += y_acc;
		z_sum += 1.0f - z_acc;
	}
	x_angle_accelometer_offset = x_sum / num_of_samples;
	y_angle_accelometer_offset = y_sum / num_of_samples;
	z_angle_accelometer_offset = z_sum / num_of_samples;
}

static void calibrate_gyro() {
	const float num_of_samples = 3000.0f;
	float x_sum, y_sum, z_sum;
	for (int i = 0; i < num_of_samples; ++i) {
		uint8_t data[6];

		HAL_I2C_Mem_Read(&hi2c1, MPU_ADDRESS, 0x43, 1, data, 6, 6000);

		float x = (int16_t) ((data[0] << 8) + data[1]);
		float y = (int16_t) ((data[2] << 8) + data[3]);
		float z = (int16_t) ((data[4] << 8) + data[5]);

		float x_gy = -(x / 16.4f);
		float y_gy = (y / 16.4f);
		float z_gy = (z / 16.4f);

		x_sum += x_gy;
		y_sum += y_gy;
		z_sum += z_gy;
	}
	x_gyro_offset = x_sum / num_of_samples;
	y_gyro_offset = y_sum / num_of_samples;
	z_gyro_offset = z_sum / num_of_samples;

}

void mpu9250_calibrate() {
	calibrate_accelometer();
	calibrate_gyro();
}
