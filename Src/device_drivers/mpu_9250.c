#include "device_drivers/mpu_9250.h"

#include <math.h>
#include "attitude/quaternion_operations.h"
#include "attitude/kalman_filter.h"

extern I2C_HandleTypeDef hi2c1;

#define MPU_ADDRESS 0xD0

#define COMPLEMENTARY_ALPHA 0.9996f
// #define COMPLEMENTARY_ALPHA 0.0f
// #define COMPLEMENTARY_ALPHA 1.0f

#define DEGREES_TO_RADIANS (M_PI / 180.0)
#define RADIANS_TO_DEGREES (180.0 / M_PI)

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
	// data = 0x0;
	data = 0x18;
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDRESS, 0x1C, 1, &data, 1, 1000);

	// ACCELOMETER CONFIGURATION 2
	data = 0x04;
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDRESS, 0x1D, 1, &data, 1, 1000);

	// GYRO
	data = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDRESS, 0x1A, 1, &data, 1, 1000);
	// GYRO CONFIGURATION
	data = 0x10;
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDRESS, 0x1B, 1, &data, 1, 1000);

	Matrix_create(&euler_angles, 3, 1);
	Matrix_create(&quaternion, 4, 1);
	Matrix_set(&quaternion, 0, 0, 1.0);
	Matrix_create(&quaternion_rate, 4, 1);
	Matrix_create(&gyro_rate, 3, 1);

	KalmanFilter_start(0.0f, 0.0f, 0.5f, 10000.0f, 110);
}

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

void mpu9250_get_axis_data_quaternion(struct Angles *output) {
	uint8_t data[6];

	// GYRO
	HAL_I2C_Mem_Read(&hi2c1, MPU_ADDRESS, 0x43, 1, data, 6, 6000);

	float x = (int16_t) ((data[0] << 8) + data[1]);
	float y = (int16_t) ((data[2] << 8) + data[3]);
	float z = (int16_t) ((data[4] << 8) + data[5]);

	uint32_t current_time = HAL_GetTick();
	float delta_time = (current_time - prev_time) / 1000.0f;
	prev_time = current_time;

	float p = ((x / 32.8f) - x_gyro_offset) * DEGREES_TO_RADIANS;
	float q = ((y / 32.8f) - y_gyro_offset) * DEGREES_TO_RADIANS;
	float r = ((z / 32.8f) - z_gyro_offset) * DEGREES_TO_RADIANS;

	Matrix_set(&gyro_rate, 0, 0, p * delta_time);
	Matrix_set(&gyro_rate, 1, 0, q * delta_time);
	Matrix_set(&gyro_rate, 2, 0, r * delta_time);

	Quaternion_gyro_data_to_quaternion_rate(&quaternion, &gyro_rate,
			&quaternion_rate);
	Matrix_add(&quaternion, &quaternion_rate);

	Quaternion_quaternion_to_euler_angles(&quaternion, &euler_angles);

	// ACCELOMETER
	HAL_I2C_Mem_Read(&hi2c1, MPU_ADDRESS, 0x3B, 1, data, 6, 6000);

	int16_t x_acc = (int16_t) ((data[0] << 8) + data[1]);
	int16_t y_acc = (int16_t) ((data[2] << 8) + data[3]);
	int16_t z_acc = (int16_t) ((data[4] << 8) + data[5]);

	float x_real = (x_acc / 2048.0f) - x_accelometer_offset;
	float y_real = -(y_acc / 2048.0f) - y_accelometer_offset;
	float z_real = -(z_acc / 2048.0f) + z_accelometer_offset;

	float accelerometer_vec = sqrtf(
			(x_real * x_real) + (y_real * y_real) + (z_real * z_real));
	float accelerometer_angle_x = asin(x_real / accelerometer_vec)
			* RADIANS_TO_DEGREES - x_accelometer_offset;
	float accelerometer_angle_y = asin(y_real / accelerometer_vec)
			* RADIANS_TO_DEGREES - y_accelometer_offset;

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

	// KALMAN FILTER FOR ACCELOMETER
	/*
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
	/*

	 */

	Quaternion_euler_angles_to_quaternion(&euler_angles, &quaternion);

	output->x_angle = -Matrix_get(&euler_angles, 1, 0) * RADIANS_TO_DEGREES;
	output->y_angle = -Matrix_get(&euler_angles, 0, 0) * RADIANS_TO_DEGREES;
	output->z_angle = Matrix_get(&euler_angles, 2, 0) * RADIANS_TO_DEGREES;
}

static void calibrate_accelometer() {
	const float num_of_samples = 1000.0f;
	float x_sum = 0.0f, y_sum = 0.0f;
	float x_sum2 = 0.0f, y_sum2 = 0.0f, z_sum2 = 0.0f;
	for (int i = 0; i < num_of_samples; ++i) {
		uint8_t data[6];
		HAL_I2C_Mem_Read(&hi2c1, MPU_ADDRESS, 0x3B, 1, data, 6, 6000);

		int16_t x_acc = (int16_t) ((data[0] << 8) + data[1]);
		int16_t y_acc = (int16_t) ((data[2] << 8) + data[3]);
		int16_t z_acc = (int16_t) ((data[4] << 8) + data[5]);

		float x_real = (x_acc / 2048.0f);
		float y_real = -(y_acc / 2048.0f);
		float z_real = -(z_acc / 2048.0f);

		x_sum2 += x_real;
		y_sum2 += y_real;
		z_sum2 += 1.0f - z_real;
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
		uint8_t data[6];
		HAL_I2C_Mem_Read(&hi2c1, MPU_ADDRESS, 0x3B, 1, data, 6, 6000);

		int16_t x_acc = (int16_t) ((data[0] << 8) + data[1]);
		int16_t y_acc = (int16_t) ((data[2] << 8) + data[3]);
		int16_t z_acc = (int16_t) ((data[4] << 8) + data[5]);

		float x_real = (x_acc / 2048.0f) - x_accelometer_offset;
		float y_real = -(y_acc / 2048.0f) - y_accelometer_offset;
		float z_real = -(z_acc / 2048.0f) + z_accelometer_offset;

		/*
		 float accelerometer_angle_x =
		 -acosf(
		 (x_real * x_real)
		 / (sqrtf(x_real * x_real + z_real * z_real)
		 * x_real)) * RADIANS_TO_DEGREES;
		 float accelerometer_angle_y =
		 -acosf(
		 (y_real * y_real)
		 / (sqrtf(y_real * y_real + z_real * z_real)
		 * y_real)) * RADIANS_TO_DEGREES;
		 */
		float accelerometer_vec = sqrtf(
				(x_real * x_real) + (y_real * y_real) + (z_real * z_real));
		float accelerometer_angle_x = asin(
				x_real / accelerometer_vec) * RADIANS_TO_DEGREES;
		float accelerometer_angle_y = asin(
				y_real / accelerometer_vec) * RADIANS_TO_DEGREES;

		/*
		 float accelerometer_angle_x = asinf(
		 x_real / (sqrtf(powf(z_real, 2) + powf(x_real, 2))))
		 * radian_to_degrees;
		 float accelerometer_angle_y = asinf(
		 y_real / (sqrtf(powf(z_real, 2) + powf(y_real, 2))))
		 * radian_to_degrees;
		 */
		x_sum += accelerometer_angle_x;
		y_sum += accelerometer_angle_y;
	}

	x_angle_accelometer_offset = x_sum / num_of_samples;
	y_angle_accelometer_offset = y_sum / num_of_samples;

}
/*
 */

/*
 static void calibrate_accelometer() {
 const float num_of_samples = 1000.0f;
 float x_sum = 0.0f, y_sum = 0.0f, z_sum = 0.0f;
 for (int i = 0; i < num_of_samples; ++i) {
 uint8_t data[6];
 HAL_I2C_Mem_Read(&hi2c1, MPU_ADDRESS, 0x3B, 1, data, 6, 6000);

 int16_t x_acc = (int16_t) ((data[0] << 8) + data[1]);
 int16_t y_acc = (int16_t) ((data[2] << 8) + data[3]);
 int16_t z_acc = (int16_t) ((data[4] << 8) + data[5]);

 float x_real = (x_acc / 4096.0f);
 float y_real = -(y_acc / 4096.0f);
 float z_real = (-z_acc / 4096.0f);

 x_sum += x_real;
 y_sum += y_real;
 z_sum += 1.0f + z_real;
 }

 x_accelometer_offset = x_sum / num_of_samples;
 y_accelometer_offset = y_sum / num_of_samples;
 z_accelometer_offset = z_sum / num_of_samples;
 }
 */

static void calibrate_gyro() {
	const int num_of_samples = 1000;
	float x_sum = 0.0f, y_sum = 0.0f, z_sum = 0.0f;
	for (int i = 0; i < num_of_samples; ++i) {
		uint8_t data[6];

		HAL_I2C_Mem_Read(&hi2c1, MPU_ADDRESS, 0x43, 1, data, 6, 6000);

		float x = (int16_t) ((data[0] << 8) + data[1]);
		float y = (int16_t) ((data[2] << 8) + data[3]);
		float z = (int16_t) ((data[4] << 8) + data[5]);

		float x_gy = (x / 32.8f);
		float y_gy = (y / 32.8f);
		float z_gy = (z / 32.8f);

		x_sum += x_gy;
		y_sum += y_gy;
		z_sum += z_gy;
	}
	x_gyro_offset = x_sum / (float) num_of_samples;
	y_gyro_offset = y_sum / (float) num_of_samples;
	z_gyro_offset = z_sum / (float) num_of_samples;

}

void mpu9250_calibrate() {
	calibrate_accelometer();
	calibrate_gyro();
}

