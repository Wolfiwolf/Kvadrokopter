#ifndef DRONE_CONTROLLER_H_FILE
#define DRONE_CONTROLLER_H_FILE

#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "communications/drone_commands.h"

void drone_controller_None();
void drone_controller_Pan(enum PanDirection dir, enum DroneMovementSpeed speed);
void drone_controller_Lift(enum LiftDirection dir, enum DroneMovementSpeed speed);
void drone_controller_Rotate(enum RotateDirection dir, enum DroneMovementSpeed speed);

#endif
