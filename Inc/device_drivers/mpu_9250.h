#ifndef MPU_9250_H_FILE
#define MPU_9250_H_FILE

#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"

struct Angles {
	float x_angle, y_angle;
};

void mpu9250_init();
struct Angles mpu9250_get_axis_data();

#endif
