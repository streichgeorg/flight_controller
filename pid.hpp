#pragma once

struct PID {
    float kp, ki, kd;
    float err_sum;
    float last_err;

    float value;

    PID(float kp, float ki, float kd) :
        kp(kp), ki(ki), kd(kd){};

    void update(float dt, float target, float current);
    void reset();
};