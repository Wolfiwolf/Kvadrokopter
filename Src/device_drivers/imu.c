#include "device_drivers/imu.h"
#include <math.h>
#include "attitude/quaternion_operations.h"
#include "attitude/kalman_filter.h"

extern I2C_HandleTypeDef hi2c1;

#define IMU_ACCELOMETER_ADDRESS 0x1F<<1
#define IMU_GYRO_ADDRESS 		0x21<<1

#define DEGREES_TO_RADIANS (M_PI / 180.0)
#define RADIANS_TO_DEGREES (180.0 / M_PI)

#define COMPLEMENTARY_ALPHA 0.9996f
// #define COMPLEMENTARY_ALPHA 0.0f
// #define COMPLEMENTARY_ALPHA 1.0f

static const float radian_to_degrees = 180.0 / 3.14159265;
static float prev_x_angle;
static float prev_y_angle;
static float prev_z_angle;
static uint32_t prev_time;

static float x_angle_accelometer_offset;
static float y_angle_accelometer_offset;

static float x_accelometer_offset;
static float y_accelometer_offset;
static float z_accelometer_offset;

static float x_gyro_offset;
static float y_gyro_offset;
static float z_gyro_offset;

static struct Matrix euler_angles;
static struct Matrix quaternion;
static struct Matrix quaternion_rate;
static struct Matrix gyro_rate;

static void accelometer_init() {
	HAL_Delay(1000);
	uint8_t data[1];

	data[0] = 0x00;
	HAL_I2C_Mem_Read(&hi2c1, IMU_ACCELOMETER_ADDRESS, 0x0D, 1, data, 1, 1000);
	if (data[0] != 0xC7) {
		return;
	}

	// CONTROL 2 REG
	// data[0] = 0x00;
	data[0] = 0x09;
	HAL_I2C_Mem_Write(&hi2c1, IMU_ACCELOMETER_ADDRESS, 0x2B, 1, data, 1, 1000);

	// XYZ DATA | HIGH PASS DISABLED, RANGE IS 2G
	// data[0] = 0x00;
	data[0] = 0x02;
	HAL_I2C_Mem_Write(&hi2c1, IMU_ACCELOMETER_ADDRESS, 0x0E, 1, data, 1, 1000);

	// CONTROL 1 REG
	// data[0] = 0x05; // LOW NOISE
	data[0] = 0x01; // NORMAL MODE
	HAL_I2C_Mem_Write(&hi2c1, IMU_ACCELOMETER_ADDRESS, 0x2A, 1, data, 1, 1000);
}

static void gyro_init() {
	uint8_t data[1];

	data[0] = 0x00;
	HAL_I2C_Mem_Read(&hi2c1, IMU_GYRO_ADDRESS, 0x0C, 1, data, 1, 1000);
	if (data[0] != 0xD7) {
		return;
	}

	HAL_Delay(500);

	// CONTROL 0 REG
	// data[0] = 0x43;
	data[0] = 0x12;
	HAL_I2C_Mem_Write(&hi2c1, IMU_GYRO_ADDRESS, 0x0D, 1, data, 1, 1000);

	// CONTROL 1 REG
	// data[0] = 0x03;
	data[0] = 0x06;
	HAL_I2C_Mem_Write(&hi2c1, IMU_GYRO_ADDRESS, 0x13, 1, data, 1, 1000);
}

void IMU_init() {
	accelometer_init();
	gyro_init();

	prev_x_angle = 0.0f;
	prev_y_angle = 0.0f;
	prev_z_angle = 0.0f;

	Matrix_create(&euler_angles, 3, 1);
	Matrix_create(&quaternion, 4, 1);
	Matrix_set(&quaternion, 0, 0, 1.0);
	Matrix_create(&quaternion_rate, 4, 1);
	Matrix_create(&gyro_rate, 3, 1);

	KalmanFilter_start(0.0f, 0.0f, 0.5f, 20000.0f, 110);
}

static float Kg_x = 0.0f;
static float Ee_x = 0.5f;
static float prev_Ee_x = 0.5f;
static float Em_x = 3.0f;
static float prev_val_x = 0.0f;

static float Kg_y = 0.0f;
static float Ee_y = 0.5f;
static float prev_Ee_y = 0.5f;
static float Em_y = 3.0f;
static float prev_val_y = 0.0f;

static float Kg_z = 0.0f;
static float Ee_z = 0.5f;
static float prev_Ee_z = 0.5f;
static float Em_z = 3.0f;
static float prev_val_z = 0.0f;

static float x_vals[5] = { 0, 0, 0, 0, 0 };
static float y_vals[5] = { 0, 0, 0, 0, 0 };

static void order_array(float *arr) {
	for (int i = 0; i < 5; ++i) {
		for (int p = 0; p < 4; ++p) {
			if (arr[i + 1] > arr[i]) {
				float temp = arr[i + 1];
				arr[i + 1] = arr[i];
				arr[i] = temp;
			}
		}
	}
}

static void copy_array(float *arr_src, float *arr_dest) {
	for (int i = 0; i < 5; ++i) {
		arr_dest[i] = arr_src[i];
	}
}

void IMU_get_axis_data(struct Angles *output) {
	uint8_t data[6];

	// GYRO
	HAL_I2C_Mem_Read(&hi2c1, IMU_GYRO_ADDRESS, 0x01, 1, data, 6, 1000);

	int16_t x = (data[0] << 8) | data[1];
	int16_t y = (data[2] << 8) | data[3];
	int16_t z = (data[4] << 8) | data[5];

	float p = (x * 0.015625f - x_gyro_offset) * DEGREES_TO_RADIANS;
	float q = (y * 0.015625f - y_gyro_offset) * DEGREES_TO_RADIANS;
	float r = (z * 0.015625f - z_gyro_offset) * DEGREES_TO_RADIANS;

	uint32_t current_time = HAL_GetTick();
	float delta_time = (current_time - prev_time) / 1000.0f;
	prev_time = current_time;

	Matrix_set(&gyro_rate, 0, 0, p * delta_time);
	Matrix_set(&gyro_rate, 1, 0, q * delta_time);
	Matrix_set(&gyro_rate, 2, 0, r * delta_time);

	Quaternion_gyro_data_to_quaternion_rate(&quaternion, &gyro_rate,
			&quaternion_rate);
	Matrix_add(&quaternion, &quaternion_rate);

	Quaternion_quaternion_to_euler_angles(&quaternion, &euler_angles);

	// ACCELOMETER
	HAL_I2C_Mem_Read(&hi2c1, IMU_ACCELOMETER_ADDRESS, 0x01, 1, data, 6, 1000);

	int16_t ax_int = (data[0] << 8) | data[1];
	int16_t ay_int = (data[2] << 8) | data[3];
	int16_t az_int = (data[4] << 8) | data[5];

	float ax = -ax_int * 0.000976f - x_accelometer_offset;
	float ay = ay_int * 0.000976f - y_accelometer_offset;
	float az = az_int * 0.000976f - z_accelometer_offset;

	// KALMAN FILTER
	/*
	Kg_x = Ee_x / (Ee_x + Em_x);
	ax = prev_val_x + Kg_x * (ax - prev_val_x);
	double temp_prev_Ee_x = prev_Ee_x;
	prev_Ee_x = Ee_x;
	Ee_x = (1 - Kg_x) * temp_prev_Ee_x;
	prev_val_x = ax;

	Kg_y = Ee_y / (Ee_y + Em_y);
	ay = prev_val_y + Kg_y * (ay - prev_val_y);
	double temp_prev_Ee_y = prev_Ee_y;
	prev_Ee_y = Ee_y;
	Ee_y = (1 - Kg_y) * temp_prev_Ee_y;
	prev_val_y = ay;

	Kg_z = Ee_z / (Ee_z + Em_z);
	az = prev_val_z + Kg_z * (az - prev_val_z);
	double temp_prev_Ee_z = prev_Ee_z;
	prev_Ee_z = Ee_z;
	Ee_z = (1 - Kg_z) * temp_prev_Ee_z;
	prev_val_z = az;
	 */

	float accelerometer_angle_x = (asinf(
			ax / (sqrtf(powf(az, 2) + powf(ax, 2)))) * radian_to_degrees
			- x_angle_accelometer_offset) * 1.75f;
	float accelerometer_angle_y = (asinf(
			ay / (sqrtf(powf(az, 2) + powf(ay, 2)))) * radian_to_degrees
			- y_angle_accelometer_offset) * 1.75f;

	// MEDIAN FILTER
	/*
	 x_vals[4] = x_vals[3];
	 x_vals[3] = x_vals[2];
	 x_vals[2] = x_vals[1];
	 x_vals[1] = x_vals[0];
	 x_vals[0] = accelerometer_angle_x;

	 y_vals[4] = y_vals[3];
	 y_vals[3] = y_vals[2];
	 y_vals[2] = y_vals[1];
	 y_vals[1] = y_vals[0];
	 y_vals[0] = accelerometer_angle_y;

	 float x_values[5];
	 copy_array(x_vals, x_values);
	 float y_values[5];
	 copy_array(y_vals, y_values);

	 order_array(x_values);
	 order_array(y_values);

	 accelerometer_angle_x = x_values[2];
	 accelerometer_angle_y = y_values[2];
	 */

	// COMPLEMENTARY FILTER SENSOR FUSION
	/*
	Matrix_set(&euler_angles, 0, 0,
			COMPLEMENTARY_ALPHA
					* Matrix_get(&euler_angles, 0,
							0) + (1.0f - COMPLEMENTARY_ALPHA) * accelerometer_angle_y * DEGREES_TO_RADIANS);
	Matrix_set(&euler_angles, 1, 0,
			COMPLEMENTARY_ALPHA
					* Matrix_get(&euler_angles, 1,
							0) + (1.0f - COMPLEMENTARY_ALPHA) * accelerometer_angle_x * DEGREES_TO_RADIANS);
	 */

	// KALMAN FILTER SENSOR FUSION
	 KalmanFilter_update_prediction(Matrix_get(&euler_angles, 0, 0),
	 Matrix_get(&euler_angles, 1, 0));
	 KalmanFilter_calculate_kalman_gain(delta_time);
	 KalmanFilter_update_measurment(accelerometer_angle_y * DEGREES_TO_RADIANS,
	 accelerometer_angle_x * DEGREES_TO_RADIANS);
	 KalmanFilter_calculate_new_state();
	 KalmanFilter_update_covariance_matrix();
	 Matrix_set(&euler_angles, 0, 0, KalmanFilter_get_current_phi());
	 Matrix_set(&euler_angles, 1, 0, KalmanFilter_get_current_theta());

	 // KalmanFilter_update_standard_diviation();

	/*
	 */

	Quaternion_euler_angles_to_quaternion(&euler_angles, &quaternion);

	output->x_angle = -Matrix_get(&euler_angles, 0, 0) * RADIANS_TO_DEGREES;
	output->y_angle = -Matrix_get(&euler_angles, 1, 0) * RADIANS_TO_DEGREES;
	output->z_angle = -Matrix_get(&euler_angles, 2, 0) * RADIANS_TO_DEGREES;
}

static void calibrate_accelometer() {
	const float num_of_samples = 1000.0f;
	float x_sum = 0.0f, y_sum = 0.0f;
	float x_sum2 = 0.0f, y_sum2 = 0.0f, z_sum2 = 0.0f;
	uint8_t data[6];
	for (int i = 0; i < num_of_samples; ++i) {
		HAL_I2C_Mem_Read(&hi2c1, IMU_ACCELOMETER_ADDRESS, 0x01, 1, data, 6,
				1000);

		int16_t ax_int = (data[0] << 8) | data[1];
		int16_t ay_int = (data[2] << 8) | data[3];
		int16_t az_int = (data[4] << 8) | data[5];

		float ax = -ax_int * 0.000976f;
		float ay = ay_int * 0.000976f;
		float az = az_int * 0.000976f;

		x_sum2 += ax;
		y_sum2 += ay;
		z_sum2 += 1.0f - az;
	}

	x_accelometer_offset = x_sum2 / num_of_samples;
	y_accelometer_offset = y_sum2 / num_of_samples;
	z_accelometer_offset = z_sum2 / num_of_samples;
	/*
	 x_accelometer_offset = 0.0f;
	 y_accelometer_offset = 0.0f;
	 z_accelometer_offset = 0.0f;
	 */

	for (int i = 0; i < num_of_samples; ++i) {
		HAL_I2C_Mem_Read(&hi2c1, IMU_ACCELOMETER_ADDRESS, 0x01, 1, data, 6,
				1000);

		int16_t ax_int = (data[0] << 8) | data[1];
		int16_t ay_int = (data[2] << 8) | data[3];
		int16_t az_int = (data[4] << 8) | data[5];

		float ax = -ax_int * 0.000976f - x_accelometer_offset;
		float ay = ay_int * 0.000976f - y_accelometer_offset;
		float az = az_int * 0.000976f - z_accelometer_offset;

		float accelerometer_angle_x = asinf(
				ax / (sqrtf(powf(az, 2) + powf(ax, 2)))) * radian_to_degrees;
		float accelerometer_angle_y = asinf(
				ay / (sqrtf(powf(az, 2) + powf(ay, 2)))) * radian_to_degrees;

		x_sum += accelerometer_angle_x;
		y_sum += accelerometer_angle_y;
	}

	x_angle_accelometer_offset = x_sum / num_of_samples;
	y_angle_accelometer_offset = y_sum / num_of_samples;

}

static void calibrate_gyro() {
	const int num_of_samples = 5000;
	float x_sum = 0.0f, y_sum = 0.0f, z_sum = 0.0f;
	uint8_t data[6];
	for (int i = 0; i < num_of_samples; ++i) {
		HAL_I2C_Mem_Read(&hi2c1, IMU_GYRO_ADDRESS, 0x01, 1, data, 6, 1000);

		int16_t x = (data[0] << 8) | data[1];
		int16_t y = (data[2] << 8) | data[3];
		int16_t z = (data[4] << 8) | data[5];

		float x_real = x * 0.015625f;
		float y_real = y * 0.015625f;
		float z_real = z * 0.015625f;

		x_sum += x_real;
		y_sum += y_real;
		z_sum += z_real;
	}

	const float num_of_samples_real = (float) num_of_samples;
	x_gyro_offset = x_sum / num_of_samples_real;
	y_gyro_offset = y_sum / num_of_samples_real;
	z_gyro_offset = z_sum / num_of_samples_real;
}

void IMU_calibrate() {
	calibrate_accelometer();
	calibrate_gyro();
	prev_time = HAL_GetTick();
}
