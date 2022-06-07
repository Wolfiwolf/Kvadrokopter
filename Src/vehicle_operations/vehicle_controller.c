#include "vehicle_operations/vehicle_controller.h"

void drone_controller_None() {
}

void drone_controller_Pan(enum PanDirection dir, enum DroneMovementSpeed speed) {

}

void drone_controller_Lift(enum LiftDirection dir, enum DroneMovementSpeed speed) {
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 1);
}

void drone_controller_Rotate(enum RotateDirection dir, enum DroneMovementSpeed speed) {

}

