#include "device_drivers/imu.h"
#include <math.h>

extern I2C_HandleTypeDef hi2c1;

#define IMU_ACCELOMETER_ADDRESS 0x1F<<1
#define IMU_GYRO_ADDRESS 		0x21<<1

static const float radian_to_degrees = 57.29577;

static float accelometer_x_offset = 0.0f;
static float accelometer_y_offset = 0.0f;
static float accelometer_z_offset = 0.0f;
static float gyro_x_offset = 0.0f;
static float gyro_y_offset = 0.0f;
static float gyro_z_offset = 0.0f;
static struct Angles prev_angles;
static uint32_t prev_gyro_time = 0;
static const float cf_alpha = 0.9996f;  // MIX
// static const float cf_alpha = 0.0f; // TRUST ACCELOMETER
// static const float cf_alpha = 1.0f; // TRUST GYRO

static void accelometer_init() {
	uint8_t data[1];

	data[0] = 0x00;
	HAL_I2C_Mem_Read(&hi2c1, IMU_ACCELOMETER_ADDRESS, 0x0D, 1, data, 1, 1000);
	if (data[0] != 0xC7) {
		return;
	}

	// CONTROL 2 REG
	data[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, IMU_ACCELOMETER_ADDRESS, 0x2B, 1, data, 1, 1000);

	// XYZ DATA | HIGH PASS DISABLED, RANGE IS 2G
	data[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, IMU_ACCELOMETER_ADDRESS, 0x0E, 1, data, 1, 1000);

	// CONTROL 1 REG
	data[0] = 0x05; // LOW NOISE
	// data[0] = 0x01; // NORMAL MODE
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
	data[0] = 0x41; 	// 64HZ
	HAL_I2C_Mem_Write(&hi2c1, IMU_GYRO_ADDRESS, 0x0D, 1, data, 1, 1000);

	// CONTROL 1 REG
	data[0] = 0x03;
	HAL_I2C_Mem_Write(&hi2c1, IMU_GYRO_ADDRESS, 0x13, 1, data, 1, 1000);

}

void IMU_Init() {
	prev_angles.x_angle = 0.0f;
	prev_angles.y_angle = 0.0f;
	prev_angles.z_angle = 0.0f;

	accelometer_init();
	gyro_init();

	prev_gyro_time = 0;
}

static void (*gyro_getters[2])(struct Angles*);
static uint8_t gyro_getter_index = 0;
static void get_accelometer_angles(struct Angles *angles);
static void get_gyro_angles_first_time(struct Angles *angles) {
	get_accelometer_angles(angles);

	prev_gyro_time = HAL_GetTick();
	gyro_getter_index = 1;
}

static void get_gyro_angles(struct Angles *angles) {
	uint8_t data[6] = { 0x00 };
	HAL_I2C_Mem_Read(&hi2c1, IMU_GYRO_ADDRESS, 0x01, 1, data, 6, 1000);

	int16_t x = (data[0] << 8) | data[1];
	int16_t y = (data[2] << 8) | data[3];
	int16_t z = (data[4] << 8) | data[5];

	float x_real = x * 0.03125f - gyro_x_offset;
	float y_real = -y * 0.03125f - gyro_y_offset;
	float z_real = z * 0.03125f - gyro_z_offset;

	uint32_t current_time = HAL_GetTick();
	float delta_time = (current_time - prev_gyro_time) / 1000.0f;
	prev_gyro_time = current_time;

	angles->x_angle = prev_angles.x_angle + delta_time * x_real;
	angles->y_angle = prev_angles.y_angle + delta_time * y_real;
	angles->z_angle = prev_angles.z_angle + delta_time * z_real;

	float z_change = sin(z_real * delta_time * (3.14159f / 180.0f));
	angles->x_angle -= angles->y_angle * z_change;
	angles->y_angle += angles->x_angle * z_change;
}

static float Kg_x = 0.0;
static float Ee_x = 0.5;
static float prev_Ee_x = 0.5;
static float Em_x = 0.3;
static float prev_val_x = 0.0;

static float Kg_y = 0.0;
static float Ee_y = 0.5;
static float prev_Ee_y = 0.5;
static float Em_y = 0.3;
static float prev_val_y = 0.0;

static float Kg_z = 0.0;
static float Ee_z = 0.5;
static float prev_Ee_z = 0.5;
static float Em_z = 0.3;
static float prev_val_z = 0.0;

static void get_accelometer_angles(struct Angles *angles) {
	uint8_t data[6] = { 0x00 };
	HAL_I2C_Mem_Read(&hi2c1, IMU_ACCELOMETER_ADDRESS, 0x01, 1, data, 6, 1000);

	int16_t y = (data[0] << 8) | data[1];
	int16_t x = (data[2] << 8) | data[3];
	int16_t z = (data[4] << 8) | data[5];

	float x_real = x / 16720.0 - accelometer_x_offset;
	float y_real = y / 16720.0 - accelometer_y_offset;
	float z_real = z / 16720.0 - accelometer_z_offset;

	Kg_x = Ee_x / (Ee_x + Em_x);
	x_real = prev_val_x
			+ Kg_x * (x_real - prev_val_x);
	double temp_prev_Ee_x = prev_Ee_x;
	prev_Ee_x = Ee_x;
	Ee_x = (1 - Kg_x) * temp_prev_Ee_x;
	prev_val_x = x_real;

	Kg_y = Ee_y / (Ee_y + Em_y);
	y_real = prev_val_y
			+ Kg_y * (y_real - prev_val_y);
	double temp_prev_Ee_y = prev_Ee_y;
	prev_Ee_y = Ee_y;
	Ee_y = (1 - Kg_y) * temp_prev_Ee_y;
	prev_val_y = y_real;

	Kg_z = Ee_z / (Ee_z + Em_z);
	z_real = prev_val_z
			+ Kg_z * (z_real - prev_val_z);
	double temp_prev_Ee_z = prev_Ee_z;
	prev_Ee_z = Ee_z;
	Ee_z = (1 - Kg_z) * temp_prev_Ee_z;
	prev_val_z = z_real;

	float accelerometer_angle_x = asinf(
			x_real / (sqrtf(powf(z_real, 2) + powf(x_real, 2))))
			* radian_to_degrees;
	float accelerometer_angle_y = asinf(
			y_real / (sqrtf(powf(z_real, 2) + powf(y_real, 2))))
			* radian_to_degrees;

	angles->x_angle = accelerometer_angle_x;
	angles->y_angle = accelerometer_angle_y;
	angles->z_angle = 0.0f;
}

struct Angles IMU_Get_angles() {
	struct Angles accelometer_angles;
	get_accelometer_angles(&accelometer_angles);
	struct Angles gyro_angles;
	gyro_getters[gyro_getter_index](&gyro_angles);

	struct Angles angles;

	angles.x_angle = cf_alpha * gyro_angles.x_angle
			+ (1.0f - cf_alpha) * accelometer_angles.x_angle;
	angles.y_angle = cf_alpha * gyro_angles.y_angle
			+ (1.0f - cf_alpha) * accelometer_angles.y_angle;
	angles.z_angle = cf_alpha * gyro_angles.z_angle
			+ (1.0f - cf_alpha) * accelometer_angles.z_angle;

	prev_angles.x_angle = angles.x_angle;
	prev_angles.y_angle = angles.y_angle;
	prev_angles.z_angle = angles.z_angle;

	return angles;
}

static void calibrate_accelometer() {
	const int num_of_samples = 1000;
	float x_sum = 0.0f, y_sum = 0.0f, z_sum = 0.0f;
	for (int i = 0; i < num_of_samples; ++i) {
		uint8_t data[6] = { 0x00 };
		HAL_I2C_Mem_Read(&hi2c1, IMU_ACCELOMETER_ADDRESS, 0x01, 1, data, 6,
				1000);

		int16_t y = (data[0] << 8) | data[1];
		int16_t x = (data[2] << 8) | data[3];
		int16_t z = (data[4] << 8) | data[5];

		float x_real = x / 16700.0f;
		float y_real = y / 16700.0;
		float z_real = z / 16700.0;

		x_sum += x_real;
		y_sum += y_real;
		z_sum += z_real - 1.0f;
	}

	const float num_of_samples_real = (float) num_of_samples;
	accelometer_x_offset = x_sum / num_of_samples_real;
	accelometer_y_offset = y_sum / num_of_samples_real;
	accelometer_z_offset = z_sum / num_of_samples_real;
}

static void calibrate_gyro() {
	const int num_of_samples = 2000;
	float x_sum = 0.0f, y_sum = 0.0f, z_sum = 0.0f;
	for (int i = 0; i < num_of_samples; ++i) {
		uint8_t data[6] = { 0x00 };
		HAL_I2C_Mem_Read(&hi2c1, IMU_GYRO_ADDRESS, 0x01, 1, data, 6, 1000);

		int16_t x = (data[0] << 8) | data[1];
		int16_t y = (data[2] << 8) | data[3];
		int16_t z = (data[4] << 8) | data[5];

		float x_real = x * 0.03125f;
		float y_real = -y * 0.03125;
		float z_real = z * 0.03125;

		x_sum += x_real;
		y_sum += y_real;
		z_sum += z_real;
	}

	const float num_of_samples_real = (float) num_of_samples;
	gyro_x_offset = x_sum / num_of_samples_real;
	gyro_y_offset = y_sum / num_of_samples_real;
	gyro_z_offset = z_sum / num_of_samples_real;
}

void IMU_Calibrate() {
	calibrate_accelometer();
	calibrate_gyro();

	gyro_getters[0] = get_gyro_angles;
	gyro_getters[1] = get_gyro_angles_first_time;
}
