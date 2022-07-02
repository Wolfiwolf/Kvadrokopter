#include <math.h>
#include <stdint.h>

#include "motors_controls/motors_controls.h"

// Vse na 0, potem dvigni Kd da bo postal nemiren (kot da se trese), potem znizaj da se neha trest, pol pa se znizaj za 25%
// Dviguj Kp po 0.2 dokler zacne overshootat pol znizaj za 50%
// Pocasi zvisuj Ki dokler se bea pajtla, pol znizaj za 50%

#define PID_Kp 2.0f
#define PID_Ki 0.0f
#define PID_Kd 0.02f
// #define PID_Kd 0.00f

#define TAU 0.000002f
#define T 0.001f

#define Z_PID_Kp 0.0f
#define Z_PID_Ki 0.0f
#define Z_PID_Kd 0.0f

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

/*
 int PID_x_angle(float angle) {
 float error = -angle;

 float P = error;
 float I = pid_data_x.prev_I + error;
 float D = error - pid_data_x.prev_error;

 pid_data_x.prev_I = I;
 pid_data_x.prev_error = error;

 int out = PID_Kp * P + PID_Ki * I + PID_Kd * D;

 return out;
 }
 */

int PID_x_angle(float angle, float delta_time) {
	float error = -angle;

	float P = PID_Kp * error;
	float I = pid_data_x.prev_I
			+ 0.5f * PID_Ki * T * (error + pid_data_x.prev_error);
	float D = -(2.0f * PID_Kd * (angle - pid_data_x.prev_angle)
			+ (2.0f * TAU - T) * pid_data_x.prev_D) / (2.0f * TAU + T);

	pid_data_x.prev_D = D;

	pid_data_x.prev_I = I;
	pid_data_x.prev_error = error;
	pid_data_x.prev_angle = angle;

	int out = P + I + D;

	return out;
}

int PID_y_angle(float angle, float delta_time) {
	float error = -angle;

	float P = PID_Kp * error;
	float I = pid_data_y.prev_I
			+ 0.5f * PID_Ki * T * (error + pid_data_y.prev_error);
	float D = -(2.0f * PID_Kd * (angle - pid_data_y.prev_angle)
			+ (2.0f * TAU - T) * pid_data_y.prev_D) / (2.0f * TAU + T);

	pid_data_y.prev_D = D;

	pid_data_y.prev_I = I;
	pid_data_y.prev_error = error;
	pid_data_y.prev_angle = angle;

	int out = P + I + D;

	return out;
}

/*
 int PID_y_angle(float angle) {
 float error = -angle;

 float P = error;
 float I = pid_data_y.prev_I + error;
 float D = error - pid_data_y.prev_error;

 pid_data_y.prev_I = I;
 pid_data_y.prev_error = error;

 int out = PID_Kp * P + PID_Ki * I + PID_Kd * D;

 return out;
 }
 */

int PID_z_angle(float angle) {
	float error = -angle;

	float P = error;
	float I = pid_data_z.prev_I + error;
	float D = error - pid_data_z.prev_error;

	pid_data_z.prev_I = I;
	pid_data_z.prev_error = error;

	int out = Z_PID_Kp * P + Z_PID_Ki * I + Z_PID_Kd * D;

	return out;
}

static uint32_t prev_PID_time;

struct MotorSpeeds get_motor_speeds(int current_power, float x_angle,
		float y_angle, float z_angle) {
	struct MotorSpeeds speeds;

	// sin takes in radians
	const float radian_to_degrees = 57.295779513;

	float P = current_power;

	uint32_t current_time = HAL_GetTick();
	float delta_time = (current_time - prev_PID_time) / 1000.0f;
	prev_PID_time = current_time;
	int x_tilt = PID_x_angle(x_angle, delta_time);
	int y_tilt = PID_y_angle(y_angle, delta_time);
	int z_tilt = PID_z_angle(z_angle);

	speeds.m1 = P + y_tilt + x_tilt + z_tilt;
	speeds.m2 = P + y_tilt - x_tilt - z_tilt;
	speeds.m3 = P - y_tilt + x_tilt - z_tilt;
	speeds.m4 = P - y_tilt - x_tilt + z_tilt;

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

struct MotorSpeeds correct_motors_for_tilt(TIM_HandleTypeDef *htim2,
		int current_power, float x_tilt_angle, float y_tilt_angle,
		float z_tilt_angle, struct MotorSpeeds *prev_speeds) {

	// uint32_t current_time = HAL_GetTick();
	// if (current_time - prev_time > 5) {
	struct MotorSpeeds speeds = get_motor_speeds(current_power, x_tilt_angle,
			y_tilt_angle, z_tilt_angle);

	speeds.m1 = speeds.m1 < 0 ? 0 : speeds.m1;
	speeds.m2 = speeds.m2 < 0 ? 0 : speeds.m2;
	speeds.m3 = speeds.m3 < 0 ? 0 : speeds.m3;
	speeds.m4 = speeds.m4 < 0 ? 0 : speeds.m4;

	if (current_power == 0) {
		speeds.m1 = 0;
		speeds.m2 = 0;
		speeds.m3 = 0;
		speeds.m4 = 0;
	}

	set_FL_motor_speed(htim2, speeds.m1);
	set_FR_motor_speed(htim2, speeds.m2);
	set_BL_motor_speed(htim2, speeds.m3);
	set_BR_motor_speed(htim2, speeds.m4);

	//	prev_time = current_time;

	return speeds;
	// } else {
//		return *prev_speeds;
	//}

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

