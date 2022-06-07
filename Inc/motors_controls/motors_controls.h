#ifndef MOTORS_CONTROLS_H_FILE
#define MOTORS_CONTROLS_H_FILE

#include "stm32f3xx_hal.h"

struct MotorSpeeds {
	int m1, m2, m3, m4;
};

void turn_for_x_degrees(float *x, float *y, float amount_of_degrees);
struct MotorSpeeds get_motor_speeds(int current_power,
		float x_angle, float y_angle);
struct MotorSpeeds  correct_motors_for_tilt(TIM_HandleTypeDef *htim2, int current_power,
		float x_tilt_angle, float y_tilt_angle, struct MotorSpeeds *prev_speeds);

uint8_t set_FL_motor_speed(TIM_HandleTypeDef *htim2, int speed);
uint8_t set_FR_motor_speed(TIM_HandleTypeDef *htim2, int speed);
uint8_t set_BL_motor_speed(TIM_HandleTypeDef *htim2, int speed);
uint8_t set_BR_motor_speed(TIM_HandleTypeDef *htim2, int speed);

void motor_controls_init();

#endif
