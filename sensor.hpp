#pragma once

#include <Arduino.h>

bool init_sensors();
void read_imu_values(int16_t *gx, int16_t *gy, int16_t *gz, int16_t *ax, int16_t *ay, int16_t *az);