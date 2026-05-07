#include "motors.h"
#include "config.h"

#include <Arduino.h>

#define PWM_FREQ       400
#define PWM_RESOLUTION 12   // 12-bit -> 0..4095
#define ESC_MIN_US     1000
#define ESC_MAX_US     2000

static const int MOTOR_PINS[4] = {M1_PIN, M2_PIN, M3_PIN, M4_PIN};

static void motor_write_us(int motor, int pulse_us) {
  float period_us = 1000000.0f / PWM_FREQ;
  int duty = (int)((pulse_us / period_us) * (1 << PWM_RESOLUTION));
  ledcWrite(motor, duty);
}

void motors_init() {
  for (int i = 0; i < 4; i++) {
    ledcSetup(i, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_PINS[i], i);
    motor_write_us(i, ESC_MIN_US);  // arm
  }
  delay(2000);  // ESC arming delay
}

void motor_write(const motor_commands_t& cmd) {
  float cmds[4] = {cmd.m1, cmd.m2, cmd.m3, cmd.m4};
  for (int i = 0; i < 4; i++) {
    int pulse = (int)(ESC_MIN_US + cmds[i] * (ESC_MAX_US - ESC_MIN_US));
    motor_write_us(i, pulse);
  }
}
