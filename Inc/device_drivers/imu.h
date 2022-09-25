#ifndef IMU_H_FILE
#define IMU_H_FILE

#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "common.h"

void IMU_init();
void IMU_get_axis_data(struct Angles *output);
void IMU_calibrate();

#endif

