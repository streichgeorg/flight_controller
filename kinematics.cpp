#include "kinematics.hpp"

#include "state.hpp"
#include "ahrs.hpp"
#include "math.hpp"

#include <Arduino.h>

void init_kinematics() {
}

int32_t last_update_us;
void update_kinematics() {

    float ax_ms2, ay_ms2, az_ms2;
    float gx_rad_s, gy_rad_s, gz_rad_s;

    // TODO: Get sensor values

    float dt_s = (micros() - last_update_us) / 10e6;
    last_update_us = micros();

    vec3 w_rad = vec3(gx_rad_s , gy_rad_s, gz_rad_s).rmul(dt_s);

    // TODO: Consider accel values

    float w_norm = w_rad.length();
    vec3 axis = w_rad.rmul(1 / w_norm);

    quat q = quat::from_axis_angle(w_norm, axis);

    q.mul(ahrs.rotation, ahrs.rotation);
    ahrs.rotation.normalize();

    ahrs.update_euler_angles();
}