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

bool init_sensors() {
    Wire.begin();
    Wire.setClock(400000);

    mpu.initialize();
    if (!mpu.testConnection()) {
        return false;
    }

    // TODO: Experiment with this
    mpu.setDLPFMode(MPU6050_DLPF_BW_98);

    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

    mpu.setXAccelOffset(ACCEL_OFFSET_X);
    mpu.setYAccelOffset(ACCEL_OFFSET_Y);
    mpu.setZAccelOffset(ACCEL_OFFSET_Z);

    mpu.setXGyroOffset(GYRO_OFFSET_X);
    mpu.setYGyroOffset(GYRO_OFFSET_Y);
    mpu.setZGyroOffset(GYRO_OFFSET_Z);

    return true;
}

void update_imu_values() {
    int16_t gx, gy, gz;
    int16_t ax, ay, az;

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    raw_gyro_rad_s = vec3(gx, gy, gz).rmul(GYRO_SENS_DEG * DEG_TO_RAD);
    raw_accel = vec3(ax, ay, az);
}