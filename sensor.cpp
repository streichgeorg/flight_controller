#include "sensor.hpp"

#include "config.hpp"
#include "common.hpp"
#include "state.hpp"
#include "math.hpp"

#include <I2Cdev.h>
#include <MPU6050.h>
#include <Wire.h>

MPU6050 mpu;

bool init_sensors() {
    Wire.begin();

    mpu.initialize();
    if (!mpu.testConnection()) {
        return false;
    }

    #ifdef DEBUG
        int start_calibration_us = micros();
    #endif

    int pitch_sum = 0, roll_sum = 0, yaw_sum = 0;
    for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
        int16_t gx, gy, gz; 
        mpu.getRotation(&gx, &gy, &gz);

        pitch_sum += gx;
        roll_sum += gy;
        yaw_sum += gz;
    }

    gyro_offset_pitch = pitch_sum / GYRO_CALIBRATION_SAMPLES;
    gyro_offset_roll = roll_sum / GYRO_CALIBRATION_SAMPLES;
    gyro_offset_yaw = yaw_sum / GYRO_CALIBRATION_SAMPLES;

    #ifdef DEBUG
        char debug_str[30];
        sprintf(debug_str, "Calibration of gyro took %d us", micros() - start_calibration_us);
        debug(debug_str);
    #endif

    return true;
}

int16_t gx, gy, gz;
int16_t ax, ay, az;

void update_imu_values() {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    raw_gyro_rad_s = vec3(gx, gy, gz).rmul((1.0 / GYRO_RANGE_DEG) * DEG_TO_RAD);
    raw_accel = vec3(ax, ay, az);
}