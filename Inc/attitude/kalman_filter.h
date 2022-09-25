#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

// STARTING KALMAN FILTER FROM THE BEGGINING
void KalmanFilter_start(float inital_phi, float inital_theta, float inital_standard_diviation, float standard_diviation_multiplier, int number_of_cycles_without_updating_standard_diviation);

// 1
void KalmanFilter_update_prediction(float x_dot, float y_dot);

// 2
void KalmanFilter_calculate_kalman_gain(float delta_t);

// 3
void KalmanFilter_update_measurment(float phi, float theta);

// 4 - After calling this function you can get the output value of the filter
void KalmanFilter_calculate_new_state();

// 5
void KalmanFilter_update_covariance_matrix();

// Get output value of phi
float KalmanFilter_get_current_phi();

// Get output value of theta
float KalmanFilter_get_current_theta();

void KalmanFilter_update_standard_diviation();

#endif
