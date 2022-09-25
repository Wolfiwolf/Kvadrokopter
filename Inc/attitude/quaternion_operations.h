#ifndef QUATERNION_OPERATIONS_H
#define QUATERNION_OPERATIONS_H

#include "attitude/matrix_math.h"

void Quaternion_euler_angles_to_quaternion(struct Matrix *euler_angles, struct Matrix* res);

void Quaternion_quaternion_to_euler_angles(struct Matrix *quaternion, struct Matrix* res);

void Quaternion_gyro_data_to_quaternion_rate(struct Matrix *quaternion_state, struct Matrix *gyro_data, struct Matrix* res);


#endif
