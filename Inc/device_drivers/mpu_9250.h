#ifndef MPU_9250_H_FILE
#define MPU_9250_H_FILE

#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"

#include "common.h"

void mpu9250_init();
struct Angles mpu9250_get_axis_data();
void mpu9250_get_axis_data_quaternion(struct Angles *output);
void mpu9250_calibrate();

#endif
