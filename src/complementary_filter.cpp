#include "complementary_filter.h"

#include <Arduino.h>
#include <math.h>

#define ALPHA 0.98f  // trust gyro 98%, accel 2%

static attitude_t current_attitude = {0, 0, 0};
static uint32_t last_time_us = 0;

attitude_t complementary_filter(const imu_data_t& imu) {
  uint32_t now = micros();
  float dt = (now - last_time_us) / 1000000.0f;
  last_time_us = now;

  if (dt > 0.1f) dt = 0.01f;  // first call / overflow guard

  float accel_roll  = atan2f(imu.accel_y, imu.accel_z) * RAD_TO_DEG;
  float accel_pitch = atan2f(-imu.accel_x,
                             sqrtf(imu.accel_y * imu.accel_y +
                                   imu.accel_z * imu.accel_z)) * RAD_TO_DEG;

  current_attitude.roll  = ALPHA * (current_attitude.roll  + imu.gyro_x * dt)
                           + (1 - ALPHA) * accel_roll;
  current_attitude.pitch = ALPHA * (current_attitude.pitch + imu.gyro_y * dt)
                           + (1 - ALPHA) * accel_pitch;
  current_attitude.yaw  += imu.gyro_z * dt;

  return current_attitude;
}
