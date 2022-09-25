#ifndef MPU_6050_H_FILE
#define MPU_6050_H_FILE

#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"

#include "common.h"

void mpu6050_init();
struct Angles mpu6050_get_axis_data_quaternion();
void mpu6050_calibrate();

#endif
