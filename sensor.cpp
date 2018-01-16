#include "sensor.hpp"

#include "config.hpp"
#include "common.hpp"
#include "state.hpp"
#include "math.hpp"

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>

MPU6050 mpu;

bool init_sensors() {
    Wire.begin();
    Wire.setClock(400000);

    mpu.initialize();
    if (!mpu.testConnection()) {
        return false;
    }

    mpu.setXAccelOffset(ACCEL_OFFSET_X);
    mpu.setYAccelOffset(ACCEL_OFFSET_Y);
    mpu.setZAccelOffset(ACCEL_OFFSET_Z);

    mpu.setXGyroOffset(GYRO_OFFSET_X);
    mpu.setYGyroOffset(GYRO_OFFSET_Y);
    mpu.setZGyroOffset(GYRO_OFFSET_Z);


    return true;
}

int16_t gx, gy, gz;
int16_t ax, ay, az;

void update_imu_values() {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    raw_gyro_rad_s = vec3(gx, gy, gz).rmul((1.0 / GYRO_RANGE_DEG) * DEG_TO_RAD);
    raw_accel = vec3(ax, ay, az);
}