#include "kinematics.hpp"

#include "config.hpp"
#include "common.hpp"
#include "sensor.hpp"
#include "math.hpp"

#include <Arduino.h>

quat rotation(1.0, 0.0, 0.0, 0.0);
float est_pitch_rad = 0, est_roll_rad = 0;

void init_kinematics() {
}

void debug_gravity_stuff(vec3 est_gravity, vec3 accel_gravity, vec3 error) {
    debug_vec("est_gravity is %f %f %f", est_gravity);
    debug_vec("accel_gravity is %f %f %f", accel_gravity);
    debug_vec("error is %f %f %f", error);
}

void debug_rotation(quat rotation) {
    debug_num("q0 is %f", rotation.q0);
    debug_num("q1 is %f", rotation.q1);
    debug_num("q2 is %f", rotation.q2);
    debug_num("q3 is %f", rotation.q3);
}

int32_t last_kinematics_us;
void update_kinematics() {
    float dt_s = (micros() - last_kinematics_us) / 1e6;

    last_kinematics_us = micros();

    vec3 w_rad_s = raw_gyro_rad_s;

    vec3 accel = raw_accel.rmul(1.0 / ACCEL_1G);
    float accel_norm = accel.length();

    if (abs(accel_norm - 1.0) < 0.15) {
        vec3 est_gravity = vec3(
            rotation.q1 * rotation.q3 - rotation.q0 * rotation.q2,
            rotation.q0 * rotation.q1 + rotation.q2 * rotation.q3,
            rotation.q0 * rotation.q0 + rotation.q3 * rotation.q3 - 0.5
        ).rmul(2);

        vec3 accel_gravity = accel.rmul(1.0 / accel_norm);

        vec3 error_rad = est_gravity.rcross(accel_gravity);
        error_rad.mul(ACCEL_WEIGHT);

        TASK(debug_gravity_stuff(est_gravity, accel_gravity, error_rad), 0.5);

        w_rad_s.add(error_rad);
        w_rad_s.mul(1.0 / (1.0 + ACCEL_WEIGHT));
    }

    vec3 w_rad = w_rad_s.rmul(dt_s);

    float w_norm = w_rad.length();
    if (w_norm < 0.0000001) {
        return;
    }

    vec3 axis = w_rad.rmul(1 / w_norm);
    quat q = quat::from_axis_angle(w_norm, w_rad.rmul(1.0 / w_norm));

    q.mul(rotation, rotation);
    rotation.normalize();

    est_pitch_rad = rotation.get_pitch_rad();
    est_roll_rad = rotation.get_roll_rad();
}