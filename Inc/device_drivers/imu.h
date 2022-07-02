#ifndef IMU_H_FILE
#define IMU_H_FILE

#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "common.h"

void IMU_Init();
struct Angles IMU_Get_angles();
void IMU_Calibrate();

#endif

