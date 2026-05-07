#pragma once

// I2C
#define SDA_PIN 21
#define SCL_PIN 22

// Motor PWM outputs
#define M1_PIN 13   // front-left
#define M2_PIN 12   // front-right
#define M3_PIN 14   // rear-left
#define M4_PIN 27   // rear-right

// Loop timing GPIO toggle (Saleae capture)
#define DEBUG_PIN 2

// Setpoint scaling — RC packet values are -1.0..1.0
#define ROLL_PITCH_RANGE_DEG  30.0f
#define YAW_RATE_RANGE_DPS    180.0f
