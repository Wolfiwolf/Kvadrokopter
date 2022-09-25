#include "device_drivers/mpu_6050.h"

#include <math.h>
#include "attitude/quaternion_operations.h"

extern I2C_HandleTypeDef hi2c1;

#define MPU_6050_ADDRESS 0x68<<1

#define COMPLEMENTARY_ALPHA 0.9996f
// #define COMPLEMENTARY_ALPHA 0.0f
// #define COMPLEMENTARY_ALPHA 1.0f

#define DEGREES_TO_RADIANS (M_PI / 180.0)
#define RADIANS_TO_DEGREES (180.0 / M_PI)

static uint32_t prev_time;

static float x_angle_offset;
static float y_angle_offset;

static float x_gyro_offset;
static float y_gyro_offset;
static float z_gyro_offset;

static struct Matrix euler_angles;
static struct Matrix quaternion;
static struct Matrix quaternion_rate;
static struct Matrix gyro_rate;

void mpu6050_init() {
	uint8_t data = 0;
	HAL_I2C_Mem_Read(&hi2c1, MPU_6050_ADDRESS, 0x75, 1, &data, 1, 1000);

	data = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, MPU_6050_ADDRESS, 0x1A, 1, &data, 1, 1000);

	data = 0x08;
	HAL_I2C_Mem_Write(&hi2c1, MPU_6050_ADDRESS, 0x1B, 1, &data, 1, 1000);

	data = 0x08;
	HAL_I2C_Mem_Write(&hi2c1, MPU_6050_ADDRESS, 0x1C, 1, &data, 1, 1000);

	data = 0x08;
	HAL_I2C_Mem_Write(&hi2c1, MPU_6050_ADDRESS, 0x6B, 1, &data, 1, 1000);

	data = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU_6050_ADDRESS, 0x6C, 1, &data, 1, 1000);

	Matrix_create(&euler_angles, 3, 1);
	Matrix_create(&quaternion, 4, 1);
	Matrix_set(&quaternion, 0, 0, 1.0);
	Matrix_create(&quaternion_rate, 4, 1);
	Matrix_create(&gyro_rate, 3, 1);

}

static float Kg_x = 0.0f;
static float Ee_x = 0.5f;
static float prev_Ee_x = 0.5f;
static float Em_x = 1.5f;
static float prev_val_x = 0.0f;

static float Kg_y = 0.0f;
static float Ee_y = 0.5f;
static float prev_Ee_y = 0.5f;
static float Em_y = 1.5f;
static float prev_val_y = 0.0f;

static float x_vals[5] = { 0, 0, 0, 0, 0 };
static float y_vals[5] = { 0, 0, 0, 0, 0 };

static void order_array(float *arr) {
	for (int i = 0; i < 3; ++i) {
		for (int p = 0; p < 2; ++p) {
			if (arr[i + 1] > arr[i]) {
				float temp = arr[i + 1];
				arr[i + 1] = arr[i];
				arr[i] = temp;
			}
		}
	}
}

static void copy_array(float *arr_src, float *arr_dest) {
	for (int i = 0; i < 3; ++i) {
		arr_dest[i] = arr_src[i];
	}
}

struct Angles mpu6050_get_axis_data_quaternion() {
	uint8_t data[6];

	// GYRO
	HAL_I2C_Mem_Read(&hi2c1, MPU_6050_ADDRESS, 0x43, 1, data, 6, 6000);

	float x = (int16_t) ((data[0] << 8) + data[1]);
	float y = (int16_t) ((data[2] << 8) + data[3]);
	float z = (int16_t) ((data[4] << 8) + data[5]);

	uint32_t current_time = HAL_GetTick();
	float delta_time = (current_time - prev_time) / 1000.0f;
	prev_time = current_time;

	Matrix_set(&gyro_rate, 0, 0,
			((-x / 131.0f) - x_gyro_offset) * DEGREES_TO_RADIANS * delta_time);
	Matrix_set(&gyro_rate, 1, 0,
			((-y / 131.0f) - y_gyro_offset) * DEGREES_TO_RADIANS * delta_time);
	Matrix_set(&gyro_rate, 2, 0,
			((z / 131.0f) - z_gyro_offset) * DEGREES_TO_RADIANS * delta_time);

	Quaternion_gyro_data_to_quaternion_rate(&quaternion, &gyro_rate,
			&quaternion_rate);
	Matrix_add(&quaternion, &quaternion_rate);

	Quaternion_quaternion_to_euler_angles(&quaternion, &euler_angles);

	// ACCELOMETER
	HAL_I2C_Mem_Read(&hi2c1, MPU_6050_ADDRESS, 0x3B, 1, data, 6, 6000);

	int16_t x_acc = (int16_t) ((data[0] << 8) + data[1]);
	int16_t y_acc = (int16_t) ((data[2] << 8) + data[3]);
	int16_t z_acc = (int16_t) ((data[4] << 8) + data[5]);

	float x_real = -(x_acc / 4096.0f);
	float y_real = (y_acc / 4096.0f);
	float z_real = (-z_acc / 4096.0f);

	float accelerometer_angle_x = asinf(
			x_real / (sqrtf(powf(z_real, 2) + powf(x_real, 2))))
			* RADIANS_TO_DEGREES + 2.1f;
	float accelerometer_angle_y = asinf(
			y_real / (sqrtf(powf(z_real, 2) + powf(y_real, 2))))
			* RADIANS_TO_DEGREES + 1.1f;

	// KALMAN FILTER
	Kg_x = Ee_x / (Ee_x + Em_x);
	accelerometer_angle_x = prev_val_x
			+ Kg_x * (accelerometer_angle_x - prev_val_x);
	double temp_prev_Ee_x = prev_Ee_x;
	prev_Ee_x = Ee_x;
	Ee_x = (1 - Kg_x) * temp_prev_Ee_x;
	prev_val_x = accelerometer_angle_x;

	Kg_y = Ee_y / (Ee_y + Em_y);
	accelerometer_angle_y = prev_val_y
			+ Kg_y * (accelerometer_angle_y - prev_val_y);
	double temp_prev_Ee_y = prev_Ee_y;
	prev_Ee_y = Ee_y;
	Ee_y = (1 - Kg_y) * temp_prev_Ee_y;
	prev_val_y = accelerometer_angle_y;

	// MEDIAN FILTER
	/*
	x_vals[4] = x_vals[3];
	x_vals[3] = x_vals[2];
	*/
	x_vals[2] = x_vals[1];
	x_vals[1] = x_vals[0];
	x_vals[0] = accelerometer_angle_x;

	/*
	y_vals[4] = y_vals[3];
	y_vals[3] = y_vals[2];
	*/
	y_vals[2] = y_vals[1];
	y_vals[1] = y_vals[0];
	y_vals[0] = accelerometer_angle_y;

	float x_values[3];
	copy_array(x_vals, x_values);
	float y_values[3];
	copy_array(y_vals, y_values);

	order_array(x_values);
	order_array(y_values);

	accelerometer_angle_x = x_values[1];
	accelerometer_angle_y = y_values[1];

	// COMPLEMENTARY FILTER
	Matrix_set(&euler_angles, 0, 0,
			COMPLEMENTARY_ALPHA
					* Matrix_get(&euler_angles, 0,
							0) + (1.0f - COMPLEMENTARY_ALPHA) * accelerometer_angle_y * DEGREES_TO_RADIANS);
	Matrix_set(&euler_angles, 1, 0,
			COMPLEMENTARY_ALPHA
					* Matrix_get(&euler_angles, 1,
							0) + (1.0f - COMPLEMENTARY_ALPHA) * accelerometer_angle_x * DEGREES_TO_RADIANS);

	Quaternion_euler_angles_to_quaternion(&euler_angles, &quaternion);

	struct Angles angles;

	angles.x_angle = (Matrix_get(&euler_angles, 1, 0) * RADIANS_TO_DEGREES)
			- x_angle_offset;
	angles.y_angle = (Matrix_get(&euler_angles, 0, 0) * RADIANS_TO_DEGREES)
			- y_angle_offset;
	angles.z_angle = Matrix_get(&euler_angles, 2, 0) * RADIANS_TO_DEGREES;
	return angles;
}

static void mpu6050_calibrate_accelometer() {
	float x_sum = 0.0f;
	float y_sum = 0.0f;
	int num_of_samples = 1000;
	uint8_t data[6];
	for (int i = 0; i < num_of_samples; i++) {
		HAL_I2C_Mem_Read(&hi2c1, MPU_6050_ADDRESS, 0x3B, 1, data, 6, 1000);

		int16_t x_acc = (int16_t) ((data[0] << 8) + data[1]);
		int16_t y_acc = (int16_t) ((data[2] << 8) + data[3]);
		int16_t z_acc = (int16_t) ((data[4] << 8) + data[5]);

		float x_real = -(x_acc / 4096.0f);
		float y_real = (y_acc / 4096.0f);
		float z_real = (-z_acc / 4096.0f);

		float accelerometer_angle_x = asinf(
				x_real / (sqrtf(powf(z_real, 2) + powf(x_real, 2))))
				* RADIANS_TO_DEGREES + 2.1f;
		float accelerometer_angle_y = asinf(
				y_real / (sqrtf(powf(z_real, 2) + powf(y_real, 2))))
				* RADIANS_TO_DEGREES + 1.1f;

		x_sum += accelerometer_angle_x;
		y_sum += accelerometer_angle_y;
	}

	x_angle_offset = x_sum / ((float) num_of_samples);
	y_angle_offset = y_sum / ((float) num_of_samples);
}

static void mpu6050_calibrate_gyro() {
	float x_sum = 0.0f;
	float y_sum = 0.0f;
	float z_sum = 0.0f;
	int num_of_samples = 1000;
	uint8_t data[6];
	for (int i = 0; i < num_of_samples; i++) {
		HAL_I2C_Mem_Read(&hi2c1, MPU_6050_ADDRESS, 0x43, 1, data, 6, 1000);

		float x = (int16_t) ((data[0] << 8) + data[1]);
		float y = (int16_t) ((data[2] << 8) + data[3]);
		float z = (int16_t) ((data[4] << 8) + data[5]);

		float x_real = (-x / 131.0f);
		float y_real = (-y / 131.0f);
		float z_real = (z / 131.0f);

		x_sum += x_real;
		y_sum += y_real;
		z_sum += z_real;
	}

	x_gyro_offset = x_sum / ((float) num_of_samples);
	y_gyro_offset = y_sum / ((float) num_of_samples);
	z_gyro_offset = z_sum / ((float) num_of_samples);
}

void mpu6050_calibrate() {
	mpu6050_calibrate_accelometer();
	mpu6050_calibrate_gyro();
	prev_time = HAL_GetTick();
}
