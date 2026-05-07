#include "imu.h"
#include "config.h"

#include <Arduino.h>
#include <Wire.h>

#define MPU6050_ADDR 0x68
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG  0x1B
#define PWR_MGMT_1   0x6B

static void wire_write(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static void wire_read(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)addr, (int)len);
  for (size_t i = 0; i < len && Wire.available(); i++) {
    buf[i] = Wire.read();
  }
}

void imu_init() {
  Wire.begin(SDA_PIN, SCL_PIN, 400000);  // 400kHz I2C

  wire_write(MPU6050_ADDR, PWR_MGMT_1, 0x00);    // wake
  wire_write(MPU6050_ADDR, ACCEL_CONFIG, 0x08);  // +/-4g
  wire_write(MPU6050_ADDR, GYRO_CONFIG, 0x08);   // +/-500 deg/s

  delay(100);
}

imu_data_t imu_read() {
  uint8_t buf[14];
  wire_read(MPU6050_ADDR, 0x3B, buf, 14);

  imu_data_t data;

  int16_t ax_raw = (buf[0]  << 8) | buf[1];
  int16_t ay_raw = (buf[2]  << 8) | buf[3];
  int16_t az_raw = (buf[4]  << 8) | buf[5];
  int16_t gx_raw = (buf[8]  << 8) | buf[9];
  int16_t gy_raw = (buf[10] << 8) | buf[11];
  int16_t gz_raw = (buf[12] << 8) | buf[13];

  // +/-4g: 8192 LSB/g
  data.accel_x = ax_raw / 8192.0f * 9.81f;
  data.accel_y = ay_raw / 8192.0f * 9.81f;
  data.accel_z = az_raw / 8192.0f * 9.81f;

  // +/-500 deg/s: 65.5 LSB/(deg/s)
  data.gyro_x = gx_raw / 65.5f;
  data.gyro_y = gy_raw / 65.5f;
  data.gyro_z = gz_raw / 65.5f;

  data.timestamp_us = micros();
  return data;
}
