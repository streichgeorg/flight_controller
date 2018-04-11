#include "kinematics.hpp"

#include "config.hpp"
#include "common.hpp"
#include "sensor.hpp"
#include "math.hpp"

#include <Arduino.h>

quat rotation;
float est_pitch_rad = 0, est_roll_rad = 0;

void init_kinematics() {
    vec3 down = vec3(0.0, 0.0, 1.0);
    vec3 accel = (raw_accel / ACCEL_1G).normalized();

    float angle = asin(accel.dot(down));
    rotation = quat::from_axis_angle(angle, accel.cross(down));
}

bool first_update = true;
int32_t last_kinematics_us;
vec3 error_sum(0.0, 0.0, 0.0);
void update_kinematics() {
    float dt_s = (micros() - last_kinematics_us) / 1e6;

    last_kinematics_us = micros();

    if (first_update) {
        first_update = false;
        return;
    }

    vec3 w_rad_s = raw_gyro_rad_s;

    vec3 accel = raw_accel / ACCEL_1G;
    float accel_norm = accel.length();

    if (abs(accel_norm - 1.0) < MAX_ACCEL_ACCELERATION) {
        vec3 est_gravity = vec3(
            rotation.q1 * rotation.q3 - rotation.q0 * rotation.q2,
            rotation.q0 * rotation.q1 + rotation.q2 * rotation.q3,
            rotation.q0 * rotation.q0 + rotation.q3 * rotation.q3 - 0.5
        ) * 2;

        vec3 accel_gravity = accel / accel_norm;

        vec3 error_rad = accel_gravity.cross(est_gravity);
        error_sum += error_rad;

        w_rad_s += (error_rad * DRIFT_CORRECTION_P) + (error_sum * DRIFT_CORRECTION_I);
    }

    if (w_rad_s.length() < 0.0000001) {
        return;
    }

    float factor = 0.5 * dt_s;

    rotation = quat(
        rotation.q0 + factor * (-rotation.q1 * w_rad_s.x - rotation.q2 * w_rad_s.y - rotation.q3 * w_rad_s.z),
        rotation.q1 + factor * (rotation.q0 * w_rad_s.x + rotation.q2 * w_rad_s.z - rotation.q3 * w_rad_s.y),
        rotation.q2 + factor * (rotation.q0 * w_rad_s.y - rotation.q1 * w_rad_s.z + rotation.q3 * w_rad_s.x),
        rotation.q3 + factor * (rotation.q0 * w_rad_s.z + rotation.q1 * w_rad_s.y - rotation.q2 * w_rad_s.x)
    );

    est_pitch_rad = rotation.get_pitch_rad();
    est_roll_rad = rotation.get_roll_rad();
}