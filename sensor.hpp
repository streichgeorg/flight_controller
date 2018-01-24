#pragma once

#include <Arduino.h>

bool init_sensors();
void update_imu_values();
void calibrate_gyro();

struct vec3;
extern vec3 raw_gyro_rad_s;
extern vec3 raw_accel;

extern bool imu_calibrated;