#include "pid.h"

#include <Arduino.h>

struct pid_state_t {
  float kp, ki, kd;
  float prev_error;
  float integral;
  float integral_limit;
};

static pid_state_t roll_pid  = {1.4f, 0.04f, 0.12f, 0.0f, 0.0f, 50.0f};
static pid_state_t pitch_pid = {1.4f, 0.04f, 0.12f, 0.0f, 0.0f, 50.0f};
static pid_state_t yaw_pid   = {2.0f, 0.02f, 0.05f, 0.0f, 0.0f, 30.0f};

void pid_init() {
  roll_pid.prev_error  = 0; roll_pid.integral  = 0;
  pitch_pid.prev_error = 0; pitch_pid.integral = 0;
  yaw_pid.prev_error   = 0; yaw_pid.integral   = 0;
}

static float pid_compute(pid_state_t& pid, float setpoint, float measured, float dt) {
  float error = setpoint - measured;

  pid.integral += error * dt;
  pid.integral = constrain(pid.integral, -pid.integral_limit, pid.integral_limit);

  float derivative = (error - pid.prev_error) / dt;
  derivative = 0.7f * derivative + 0.3f * (pid.prev_error / dt);  // LP filter

  pid.prev_error = error;

  return pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
}

motor_commands_t pid_update(const attitude_t& attitude, const rc_setpoints_t& sp) {
  float dt = 0.001f;  // 1ms loop

  float roll_out  = pid_compute(roll_pid,  sp.roll,     attitude.roll,  dt);
  float pitch_out = pid_compute(pitch_pid, sp.pitch,    attitude.pitch, dt);
  float yaw_out   = pid_compute(yaw_pid,   sp.yaw_rate, 0.0f,           dt);
  float throttle  = sp.throttle;

  // X-config motor mixing
  motor_commands_t cmd;
  cmd.m1 = throttle + roll_out - pitch_out + yaw_out;  // front-left
  cmd.m2 = throttle - roll_out - pitch_out - yaw_out;  // front-right
  cmd.m3 = throttle + roll_out + pitch_out - yaw_out;  // rear-left
  cmd.m4 = throttle - roll_out + pitch_out + yaw_out;  // rear-right

  cmd.m1 = constrain(cmd.m1, 0.0f, 1.0f);
  cmd.m2 = constrain(cmd.m2, 0.0f, 1.0f);
  cmd.m3 = constrain(cmd.m3, 0.0f, 1.0f);
  cmd.m4 = constrain(cmd.m4, 0.0f, 1.0f);

  return cmd;
}
