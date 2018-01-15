#include "sensor.hpp"

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

    // TODO: Calibrating

    return true;
}

void read_imu_values(int16_t *gx, int16_t *gy, int16_t *gz, int16_t *ax, int16_t *ay, int16_t *az) {
    mpu.getMotion6(ax, ay, az, gx, gy, gz);
}