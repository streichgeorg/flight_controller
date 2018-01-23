#include "pid.hpp"

void PID::update(float dt, float target, float current) {
    float err = target - current; 
    err_sum += dt * err;

    float p_term = kp * err;
    float d_term = kd * (err - last_err) / dt;
    float i_term = ki * err_sum;

    last_err = err;

    value = p_term + i_term + d_term;
}

void PID::reset() {
    err_sum = 0.0;
    last_err = 0.0;
}