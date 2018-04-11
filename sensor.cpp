#include "sensor.hpp"

#include "config.hpp"
#include "common.hpp"
#include "state.hpp"
#include "math.hpp"

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>

MPU6050 mpu;

vec3 raw_gyro_rad_s;
vec3 raw_accel;

bool imu_calibrated = false;

#define NUM_SAMPLES 4000
#define NUM_PRE_SAMPLES 100 
#define NUM_CHECK_SAMPLES 200 
#define MAX_GYRO_OFFSET 3
#define NUM_ATTEMPTS 10

void get_average_gyro(int num_samples, int16_t &out_gx, int16_t &out_gy, int16_t &out_gz) {
    int sum_gx = 0, sum_gy = 0, sum_gz = 0;

    int16_t gx, gy, gz;
    for (int i = 0; i < num_samples; i++) {
        mpu.getRotation(&gx, &gy, &gz);

        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;

        delay(2);
    }

    out_gx = sum_gx / num_samples;
    out_gy = sum_gy / num_samples;
    out_gz = sum_gz / num_samples;
}

bool calibrate_gyro() {
    #ifdef MANUAL_GYRO_CALIBRATION
        mpu.setXGyroOffset(GYRO_OFFSET_X);
        mpu.setYGyroOffset(GYRO_OFFSET_Y);
        mpu.setZGyroOffset(GYRO_OFFSET_Z);

        return true;
    #else
        debug("Calibrating gyro");
        for (int i = 0; i < NUM_ATTEMPTS; i++) {
            int16_t gx, gy, gz;

            // Reset offsets
            mpu.setXGyroOffset(0);
            mpu.setYGyroOffset(0);
            mpu.setZGyroOffset(0);

            // Take some samples before calibrating
            get_average_gyro(NUM_PRE_SAMPLES, gx, gy, gz);

            // Get calibration samples
            get_average_gyro(NUM_SAMPLES, gx, gy, gz);
            mpu.setXGyroOffset(-gx / 4);
            mpu.setYGyroOffset(-gy / 4);
            mpu.setZGyroOffset(-gz / 4);

            debug("Offsets are:");
            debug_num("offset gx: %f", gx / 4);
            debug_num("offset gy: %f", gy / 4);
            debug_num("offset gz: %f", gz / 4);

            get_average_gyro(NUM_PRE_SAMPLES, gx, gy, gz);
            debug("Read values:");
            debug_num("gx: %f", gx);
            debug_num("gy: %f", gy);
            debug_num("gz: %f", gz);

            // Check calibration 
            get_average_gyro(NUM_CHECK_SAMPLES, gx, gy, gz);
            if (abs(gx) <= MAX_GYRO_OFFSET && abs(gy) <= MAX_GYRO_OFFSET && abs(gz) <= MAX_GYRO_OFFSET) {
                debug("Succesfully calibrated gyro");
                return true;
            } else if (i < NUM_ATTEMPTS - 1) {
                debug("Failed to calibrate gyro, trying again.");
            } else {
                debug("Failed to calibrate gyro, giving up.");
            }
        }

        return false;
    #endif
}

bool init_sensors() {
    Wire.begin();
    Wire.setClock(400000);

    mpu.initialize();
    if (!mpu.testConnection()) {
        return false;
    }

    // TODO: Experiment with this
    mpu.setDLPFMode(MPU6050_DLPF_BW_20);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

    mpu.setXAccelOffset(ACCEL_OFFSET_X);
    mpu.setYAccelOffset(ACCEL_OFFSET_Y);
    mpu.setZAccelOffset(ACCEL_OFFSET_Z);

    delay(2000);

    return calibrate_gyro();
}

void update_imu_values() {
    int16_t gx, gy, gz;
    int16_t ax, ay, az;

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    raw_gyro_rad_s = vec3(gx, gy, gz) * GYRO_SENS_DEG * DEG_TO_RAD;
    raw_accel = vec3(ax, ay, az);
}