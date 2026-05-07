#pragma once

#include "complementary_filter.h"

struct rc_setpoints_t {
  float roll;      // deg
  float pitch;     // deg
  float yaw_rate;  // deg/s
  float throttle;  // 0..1
};

struct motor_commands_t {
  float m1, m2, m3, m4;  // 0..1
};

void pid_init();
motor_commands_t pid_update(const attitude_t& attitude, const rc_setpoints_t& sp);
