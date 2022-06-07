#ifndef BNO055_H_FILE
#define BNO055_H_FILE

#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "common.h"

void BNO055_Init();
struct Angles BNO055_Get_angles();

#endif

