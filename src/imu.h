#pragma once

#include <stdint.h>

struct imu_data_t {
  float accel_x, accel_y, accel_z;  // m/s^2
  float gyro_x, gyro_y, gyro_z;     // deg/s
  uint32_t timestamp_us;
};

void imu_init();
imu_data_t imu_read();
