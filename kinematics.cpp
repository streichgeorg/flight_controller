#include "kinematics.hpp"

#include "config.hpp"
#include "common.hpp"
#include "state.hpp"
#include "ahrs.hpp"
#include "math.hpp"

#include <Arduino.h>

void init_kinematics() {
}

int32_t last_update_us;
void update_kinematics() {
    float dt_s = (micros() - last_update_us) / 10e6;
    last_update_us = micros();

    vec3 w_rad = raw_gyro_rad_s.rmul(dt_s);

    float accel_norm = raw_accel.length();

    if (accel_norm > 0.99 * ACCEL_1G && accel_norm < 1.01 * ACCEL_1G) {
        vec3 est_gravity = ahrs.rotation.rmul(vec3(0, 0, 1));
        vec3 accel_gravity = raw_accel.rmul(1.0 / accel_norm);

        vec3 error = est_gravity.rcross(accel_gravity.rsub(est_gravity));
        error.mul(ACCEL_WEIGHT);
        w_rad.mul(GYRO_WEIGHT);

        w_rad.add(error);
    }

    float w_norm = w_rad.length();
    if (w_norm < 0.000001) {
        return;
    }

    vec3 axis = w_rad.rmul(1 / w_norm);
    quat q = quat::from_axis_angle(w_norm, axis);

    q.mul(ahrs.rotation, ahrs.rotation);
    ahrs.rotation.normalize();

    ahrs.update_euler_angles();
}