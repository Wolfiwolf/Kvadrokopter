#include "attitude/quaternion_operations.h"

#include <math.h>

void Quaternion_euler_angles_to_quaternion(struct Matrix *euler_angles, struct Matrix *res)
{
    float phi = Matrix_get(euler_angles, 0, 0);
    float theta = Matrix_get(euler_angles, 1, 0);
    float psi = Matrix_get(euler_angles, 2, 0);

    struct Matrix DCM;
    Matrix_create(&DCM, 3, 3);
    Matrix_set(&DCM, 0, 0, cos(theta) * cos(psi));
    Matrix_set(&DCM, 0, 1, cos(theta) * sin(psi));
    Matrix_set(&DCM, 0, 2, -sin(theta));

    Matrix_set(&DCM, 1, 0, sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi));
    Matrix_set(&DCM, 1, 1, cos(phi) * cos(psi) + sin(phi) * sin(theta) * sin(psi));
    Matrix_set(&DCM, 1, 2, sin(phi) * cos(theta));

    Matrix_set(&DCM, 2, 0, cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi));
    Matrix_set(&DCM, 2, 1, cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi));
    Matrix_set(&DCM, 2, 2, cos(phi) * cos(theta));

    struct Matrix q_lambda;
    Matrix_create(&q_lambda, 4, 1);

    float q_lambda_s = 0.25f * (1 + Matrix_get(&DCM, 0, 0) + Matrix_get(&DCM, 1, 1) + Matrix_get(&DCM, 2, 2));
    float q_lambda_x = 0.25f * (1 + Matrix_get(&DCM, 0, 0) - Matrix_get(&DCM, 1, 1) - Matrix_get(&DCM, 2, 2));
    float q_lambda_y = 0.25f * (1 - Matrix_get(&DCM, 0, 0) + Matrix_get(&DCM, 1, 1) - Matrix_get(&DCM, 2, 2));
    float q_lambda_z = 0.25f * (1 - Matrix_get(&DCM, 0, 0) - Matrix_get(&DCM, 1, 1) + Matrix_get(&DCM, 2, 2));

    Matrix_set(&q_lambda, 0, 0, sqrt(fabsf(q_lambda_s)));
    Matrix_set(&q_lambda, 1, 0, sqrt(fabsf(q_lambda_x)));
    Matrix_set(&q_lambda, 2, 0, sqrt(fabsf(q_lambda_y)));
    Matrix_set(&q_lambda, 3, 0, sqrt(fabsf(q_lambda_z)));

    float q_lambda_max = Matrix_max(&q_lambda);

    if (q_lambda_max == Matrix_get(&q_lambda, 0, 0))
    {
        Matrix_set(res, 0, 0, q_lambda_max);
        Matrix_set(res, 1, 0, (Matrix_get(&DCM, 1, 2) - Matrix_get(&DCM, 2, 1)) / (4 * q_lambda_max));
        Matrix_set(res, 2, 0, (Matrix_get(&DCM, 2, 0) - Matrix_get(&DCM, 0, 2)) / (4 * q_lambda_max));
        Matrix_set(res, 3, 0, (Matrix_get(&DCM, 0, 1) - Matrix_get(&DCM, 1, 0)) / (4 * q_lambda_max));
    }
    else if (q_lambda_max == Matrix_get(&q_lambda, 1, 0))
    {
        Matrix_set(res, 0, 0, (Matrix_get(&DCM, 1, 2) - Matrix_get(&DCM, 2, 1)) / (4 * q_lambda_max));
        Matrix_set(res, 1, 0, q_lambda_max);
        Matrix_set(res, 2, 0, (Matrix_get(&DCM, 0, 1) - Matrix_get(&DCM, 1, 0)) / (4 * q_lambda_max));
        Matrix_set(res, 3, 0, (Matrix_get(&DCM, 2, 0) - Matrix_get(&DCM, 0, 2)) / (4 * q_lambda_max));
    }
    else if (q_lambda_max == Matrix_get(&q_lambda, 2, 0))
    {
        Matrix_set(res, 0, 0, (Matrix_get(&DCM, 2, 0) - Matrix_get(&DCM, 0, 2)) / (4 * q_lambda_max));
        Matrix_set(res, 1, 0, (Matrix_get(&DCM, 0, 1) + Matrix_get(&DCM, 1, 0)) / (4 * q_lambda_max));
        Matrix_set(res, 2, 0, q_lambda_max);
        Matrix_set(res, 3, 0, (Matrix_get(&DCM, 1, 2) + Matrix_get(&DCM, 2, 1)) / (4 * q_lambda_max));
    }
    else if (q_lambda_max == Matrix_get(&q_lambda, 3, 0))
    {
        Matrix_set(res, 0, 0, (Matrix_get(&DCM, 0, 1) - Matrix_get(&DCM, 1, 0)) / (4 * q_lambda_max));
        Matrix_set(res, 1, 0, (Matrix_get(&DCM, 2, 0) + Matrix_get(&DCM, 0, 2)) / (4 * q_lambda_max));
        Matrix_set(res, 2, 0, (Matrix_get(&DCM, 1, 2) + Matrix_get(&DCM, 2, 1)) / (4 * q_lambda_max));
        Matrix_set(res, 3, 0, q_lambda_max);
    }
}

void Quaternion_quaternion_to_euler_angles(struct Matrix *quaternion, struct Matrix *res)
{
    struct Matrix DCM;
    Matrix_create(&DCM, 3, 3);

    float qs = Matrix_get(quaternion, 0, 0);
    float qx = Matrix_get(quaternion, 1, 0);
    float qy = Matrix_get(quaternion, 2, 0);
    float qz = Matrix_get(quaternion, 3, 0);

    float qs2 = qs * qs;
    float qx2 = qx * qx;
    float qy2 = qy * qy;
    float qz2 = qz * qz;

    Matrix_set(&DCM, 1, 2, 2.0f * (qy * qz + qx * qs));
    Matrix_set(&DCM, 2, 2, qs2 - qx2 - qy2 + qz2);
    Matrix_set(&DCM, 0, 2, 2.0f * (qx * qz - qy * qs));
    Matrix_set(&DCM, 0, 1, 2.0f * (qx * qy + qz * qs));
    Matrix_set(&DCM, 0, 0, qs2 + qx2 - qy2 - qz2);


    Matrix_set(res, 0, 0, atan2(Matrix_get(&DCM, 1, 2), Matrix_get(&DCM, 2, 2)));
    Matrix_set(res, 1, 0, -asin(Matrix_get(&DCM, 0, 2)));
    Matrix_set(res, 2, 0, atan2(Matrix_get(&DCM, 0, 1), Matrix_get(&DCM, 0, 0)));
}

void Quaternion_gyro_data_to_quaternion_rate(struct Matrix *quaternion_state, struct Matrix *gyro_data, struct Matrix *res)
{
    struct Matrix omega;
    Matrix_create(&omega, 4, 4);

    Matrix_set(&omega, 0, 0, 0.0);
    Matrix_set(&omega, 0, 1, -Matrix_get(gyro_data, 0, 0));
    Matrix_set(&omega, 0, 2, -Matrix_get(gyro_data, 1, 0));
    Matrix_set(&omega, 0, 3, -Matrix_get(gyro_data, 2, 0));

    Matrix_set(&omega, 1, 0, Matrix_get(gyro_data, 0, 0));
    Matrix_set(&omega, 1, 1, 0.0);
    Matrix_set(&omega, 1, 2, Matrix_get(gyro_data, 2, 0));
    Matrix_set(&omega, 1, 3, -Matrix_get(gyro_data, 1, 0));

    Matrix_set(&omega, 2, 0, Matrix_get(gyro_data, 1, 0));
    Matrix_set(&omega, 2, 1, -Matrix_get(gyro_data, 2, 0));
    Matrix_set(&omega, 2, 2, 0.0);
    Matrix_set(&omega, 2, 3, Matrix_get(gyro_data, 0, 0));

    Matrix_set(&omega, 3, 0, Matrix_get(gyro_data, 2, 0));
    Matrix_set(&omega, 3, 1, Matrix_get(gyro_data, 1, 0));
    Matrix_set(&omega, 3, 2, -Matrix_get(gyro_data, 0, 0));
    Matrix_set(&omega, 3, 3, 0.0);

    Matrix_multiply(&omega, quaternion_state, res);

    Matrix_multiply_scalar(res, 0.5f);
}
