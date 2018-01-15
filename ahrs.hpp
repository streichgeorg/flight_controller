#pragma once

#include "math.hpp"

struct AHRS {
    float pitch_vel_rad_s = 0.0;
    float roll_vel_rad_s = 0.0;
    float yaw_vel_rad_s = 0.0;

    quat rotation = quat(1, 0, 0, 0);

    float pitch_rad = 0.0;
    float roll_rad = 0.0;

    void update_euler_angles();
};