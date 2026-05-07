#pragma once

#include "imu.h"

struct attitude_t {
  float roll;   // deg, positive = right side down
  float pitch;  // deg, positive = nose up
  float yaw;    // deg, integrated from gyro only
};

attitude_t complementary_filter(const imu_data_t& imu);
