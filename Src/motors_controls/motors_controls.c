#include <math.h>
#include <stdint.h>

#include "motors_controls/motors_controls.h"

#define MOTORS_DEGREES_TO_RADIANS (M_PI / 180.0)

#define PID_Kp 1.2f
#define PID_Ki 0.1f
#define PID_Kd 0.2f

#define Z_PID_Kp 6.0f
#define Z_PID_Ki 0.2f
#define Z_PID_Kd 0.2f

struct TiltPIDData {
	float prev_angle;
	float prev_error;
	float prev_I;
	float prev_D;
};

static struct TiltPIDData pid_data_x;
static struct TiltPIDData pid_data_y;
static struct TiltPIDData pid_data_z;

static uint32_t prev_time;

extern float x_tilt_set_point;
extern float y_tilt_set_point;

int PID_x_angle(float angle, float delta_time) {
	float error = x_tilt_set_point - angle;

	float P = error;
	float I = pid_data_x.prev_I + error * delta_time;
	float D = (error - pid_data_x.prev_error) / delta_time;

	pid_data_x.prev_I = I;
	pid_data_x.prev_error = error;
	pid_data_x.prev_angle = angle;

	int out = PID_Kp * P + PID_Ki * I + PID_Kd * D;

	return out;
}

int PID_y_angle(float angle, float delta_time) {
	float error = y_tilt_set_point - angle;

	float P = error;
	float I = pid_data_y.prev_I + error * delta_time;
	float D = (error - pid_data_y.prev_error) / delta_time;

	pid_data_y.prev_I = I;
	pid_data_y.prev_error = error;
	pid_data_y.prev_angle = angle;

	int out = PID_Kp * P + PID_Ki * I + PID_Kd * D;

	return out;
}

int PID_z_angle(float angle, float delta_time) {
	float error = - angle;

	float P = error;
	float I = pid_data_z.prev_I + error * delta_time;
	float D = (error - pid_data_z.prev_error) / delta_time;

	pid_data_z.prev_I = I;
	pid_data_z.prev_error = error;
	pid_data_z.prev_angle = angle;

	int out = Z_PID_Kp * P + Z_PID_Ki * I + Z_PID_Kd * D;

	return out;
}

static uint32_t prev_PID_time;

struct MotorSpeeds get_motor_speeds(int current_power, float x_angle,
		float y_angle, float z_angle) {
	struct MotorSpeeds speeds;

	// sin takes in radians
	const float radian_to_degrees = 57.295779513;

	float ddx = 0.0f;
	float ddy = 0.0f;
	if (x_angle != 90.0f && y_angle != 90.0f) {
		ddx = tanf(fabsf(x_angle * (float)MOTORS_DEGREES_TO_RADIANS)) * current_power;
		ddy = tanf(fabsf(y_angle * (float)MOTORS_DEGREES_TO_RADIANS)) * current_power;
	}

	float P = sqrtf(powf(ddx, 2) + powf(ddy, 2) + powf(current_power, 2));

	uint32_t current_time = HAL_GetTick();
	float delta_time = (current_time - prev_PID_time) / 1000.0f;
	prev_PID_time = current_time;
	int x_tilt = PID_x_angle(x_angle, delta_time);
	int y_tilt = PID_y_angle(y_angle, delta_time);
	int z_tilt = PID_z_angle(z_angle, delta_time);

	speeds.m1 = P + y_tilt - x_tilt + z_tilt;
	speeds.m2 = P + y_tilt + x_tilt - z_tilt;
	speeds.m3 = P - y_tilt - x_tilt - z_tilt;
	speeds.m4 = P - y_tilt + x_tilt + z_tilt;

	return speeds;
}

uint8_t set_FL_motor_speed(TIM_HandleTypeDef *htim2, int speed) {
	htim2->Instance->CCR1 = 500 + speed;
	return 0;
}

uint8_t set_FR_motor_speed(TIM_HandleTypeDef *htim2, int speed) {
	htim2->Instance->CCR2 = 500 + speed;
	return 0;
}

uint8_t set_BL_motor_speed(TIM_HandleTypeDef *htim2, int speed) {
	htim2->Instance->CCR3 = 500 + speed;
	return 0;
}

uint8_t set_BR_motor_speed(TIM_HandleTypeDef *htim2, int speed) {
	htim2->Instance->CCR4 = 500 + speed;
	return 0;
}

static uint32_t prev_time;
struct MotorSpeeds correct_motors_for_tilt(TIM_HandleTypeDef *htim2,
		int current_power, float x_tilt_angle, float y_tilt_angle,
		float z_tilt_angle, struct MotorSpeeds *prev_speeds) {

	uint32_t current = HAL_GetTick();
	if (current - prev_time > 5) {
		prev_time = current;
		struct MotorSpeeds speeds = get_motor_speeds(current_power,
				x_tilt_angle, y_tilt_angle, z_tilt_angle);

		speeds.m1 = speeds.m1 < 0 ? 0 : speeds.m1;
		speeds.m2 = speeds.m2 < 0 ? 0 : speeds.m2;
		speeds.m3 = speeds.m3 < 0 ? 0 : speeds.m3;
		speeds.m4 = speeds.m4 < 0 ? 0 : speeds.m4;

		set_FL_motor_speed(htim2, speeds.m1);
		set_FR_motor_speed(htim2, speeds.m2);
		set_BL_motor_speed(htim2, speeds.m3);
		set_BR_motor_speed(htim2, speeds.m4);

		return speeds;
	} else {
		return *prev_speeds;
	}

}

void motor_controls_init() {
	prev_time = 0;

	pid_data_x.prev_angle = 0.0f;
	pid_data_x.prev_D = 0.0f;
	pid_data_x.prev_I = 0.0f;
	pid_data_x.prev_error = 0.0f;

	pid_data_y.prev_angle = 0.0f;
	pid_data_y.prev_D = 0.0f;
	pid_data_y.prev_I = 0.0f;
	pid_data_y.prev_error = 0.0f;

	pid_data_z.prev_angle = 0.0f;
	pid_data_z.prev_D = 0.0f;
	pid_data_z.prev_I = 0.0f;
	pid_data_z.prev_error = 0.0f;
}

