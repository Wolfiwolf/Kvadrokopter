#include "attitude/kalman_filter.h"
#include <stdint.h>
#include "attitude/matrix_math.h"
#include <math.h>
#include <stdio.h>

static struct Matrix X;
static struct Matrix Y;
static struct Matrix P;
static struct Matrix K;
static struct Matrix R;
static struct Matrix Q;

static float _standard_diviation_multipier = 1.0f;

static int _number_of_cycles_without_updating_standard_diviation;
static int _number_of_cycles_without_updating_standard_diviation_counter = 0;

void KalmanFilter_start(float inital_phi, float inital_theta, float inital_standard_diviation,
                        float standard_diviation_multiplier, int number_of_cycles_without_updating_standard_diviation)
{
    _standard_diviation_multipier = standard_diviation_multiplier;
    _number_of_cycles_without_updating_standard_diviation = number_of_cycles_without_updating_standard_diviation;

    Matrix_create(&X, 2, 1);
    Matrix_set(&X, 0, 0, inital_phi);
    Matrix_set(&X, 1, 0, inital_theta);

    Matrix_create(&P, 2, 2);
    Matrix_set(&P, 0, 0, 0.01f);
    Matrix_set(&P, 1, 1, 0.01f);

    Matrix_create(&R, 2, 2);
    Matrix_set(&R, 0, 0, inital_standard_diviation * inital_standard_diviation * _standard_diviation_multipier);
    Matrix_set(&R, 1, 1, inital_standard_diviation * inital_standard_diviation * _standard_diviation_multipier);

    Matrix_create(&Q, 2, 2);
    Matrix_set(&Q, 0, 0, 0.005f);
    Matrix_set(&Q, 1, 1, 0.005f);

    Matrix_create(&Y, 2, 1);
    Matrix_create(&K, 2, 2);
}

void KalmanFilter_update_prediction(float phi_gyro, float theta_gyro)
{
    Matrix_set(&X, 0, 0, phi_gyro);
    Matrix_set(&X, 1, 0, theta_gyro);
}

void KalmanFilter_calculate_kalman_gain(float delta_t)
{
    struct Matrix A;
    struct Matrix At;
    Matrix_create(&A, 2, 2);
    Matrix_create(&At, 2, 2);

    Matrix_set(&A, 0, 0, 1.0f);
    Matrix_set(&A, 0, 1, delta_t);
    Matrix_set(&A, 1, 1, 1.0f);

    Matrix_set(&At, 0, 0, 1.0f);
    Matrix_set(&At, 1, 0, delta_t);
    Matrix_set(&At, 1, 1, 1.0f);

    struct Matrix T6;
    Matrix_create(&T6, 2, 2);
    struct Matrix T7;
    Matrix_create(&T7, 2, 2);

    // P = A * P * At + Q

    // T6 = A * P
    Matrix_multiply(&A, &P, &T6);

    // T7 = T6 * At = (A * P) * At
    Matrix_multiply(&T6, &At, &T7);

    // T7 = T7 + Q = A * P * At + Q
    Matrix_add(&T7, &Q);

    Matrix_copy(&T7, &P);

    Matrix_set(&P, 0, 1, 0.0f);
    Matrix_set(&P, 1, 0, 0.0f);

    struct Matrix T;
    Matrix_create(&T, 2, 2);
    // T = P
    Matrix_copy(&P, &T);

    // T = P + R

    Matrix_add(&T, &R);

    struct Matrix T2;
    Matrix_create(&T2, 2, 2);
    // T2 = P
    Matrix_copy(&P, &T2);

    // T2 = P / T = P / (P + R)
    Matrix_diagonal_divide(&T2, &T);

    // K = P / (P + R)
    Matrix_copy(&T2, &K);
}

void KalmanFilter_update_measurment(float phi, float theta)
{
    Matrix_set(&Y, 0, 0, phi);
    Matrix_set(&Y, 1, 0, theta);
}

void KalmanFilter_calculate_new_state()
{
    struct Matrix T;
    Matrix_create(&T, 2, 1);

    // T = Y
    Matrix_copy(&Y, &T);

    // T = Y - X
    Matrix_subtract(&T, &X);

    struct Matrix T2;
    Matrix_create(&T2, 2, 2);

    // T2 = K * (Y - X)
    struct Matrix T9;
    Matrix_create(&T9, 2, 2);
    Matrix_copy(&K, &T9);
    Matrix_set(&T9, 1, 1, Matrix_get(&T9, 0, 0));

    Matrix_multiply(&K, &T, &T2);

    // X = X + T2 = X + K * (Y - X)
    Matrix_add(&X, &T2);
}

void KalmanFilter_update_covariance_matrix()
{
    struct Matrix I;
    Matrix_create(&I, 2, 2);
    Matrix_set(&I, 0, 0, 1.0f);
    Matrix_set(&I, 1, 1, 1.0f);

    // I = I - K
    Matrix_subtract(&I, &K);

    struct Matrix T;
    Matrix_create(&T, 2, 2);

    // T = (I - K) * P
    Matrix_multiply(&I, &P, &T);

    // P = T
    Matrix_copy(&T, &P);
}

float KalmanFilter_get_current_phi()
{
    return Matrix_get(&X, 0, 0);
}

float KalmanFilter_get_current_theta()
{
    return Matrix_get(&X, 1, 0);
}

#define RECALCULATION_WINDOW_SIZE 100
static float _updating_window_phi[RECALCULATION_WINDOW_SIZE];
static float _updating_window_theta[RECALCULATION_WINDOW_SIZE];

void KalmanFilter_update_standard_diviation()
{
    for (uint8_t i = RECALCULATION_WINDOW_SIZE - 1; i > 0; --i)
    {
        _updating_window_phi[i] = _updating_window_phi[i - 1];
        _updating_window_theta[i] = _updating_window_theta[i - 1];
    }
    _updating_window_phi[0] = KalmanFilter_get_current_phi();
    _updating_window_theta[0] = KalmanFilter_get_current_theta();

    if (_number_of_cycles_without_updating_standard_diviation_counter > _number_of_cycles_without_updating_standard_diviation) {
    	return;
    }


    if (_number_of_cycles_without_updating_standard_diviation_counter < _number_of_cycles_without_updating_standard_diviation)
    {
        float sum_phi = 0.0f;
        float sum_theta = 0.0f;
        for (uint8_t i = 0; i < RECALCULATION_WINDOW_SIZE; i++)
        {
            sum_phi += _updating_window_phi[i];
            sum_theta += _updating_window_theta[i];
        }
        float average_phi = sum_phi / (float)RECALCULATION_WINDOW_SIZE;
        float average_theta = sum_theta / (float)RECALCULATION_WINDOW_SIZE;

        sum_phi = 0.0f;
        sum_theta = 0.0f;
        for (uint8_t i = 0; i < RECALCULATION_WINDOW_SIZE; i++)
        {
            sum_phi += fabsf(average_phi - _updating_window_phi[i]);
            sum_theta += fabsf(average_theta - _updating_window_theta[i]);
        }

        float standard_diviation_phi = sum_phi / (float)RECALCULATION_WINDOW_SIZE;
        float standard_diviation_theta = sum_theta / (float)RECALCULATION_WINDOW_SIZE;

        Matrix_set(&R, 0, 0, standard_diviation_phi * standard_diviation_phi * _standard_diviation_multipier);
        Matrix_set(&R, 1, 1, standard_diviation_theta * standard_diviation_theta * _standard_diviation_multipier);
    }
}
