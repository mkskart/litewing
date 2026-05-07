#pragma once

#include <stdint.h>
#include <stddef.h>

struct rc_packet_t {
  float throttle;     // 0..1
  float roll;         // -1..1 -> deg via ROLL_PITCH_RANGE_DEG
  float pitch;        // -1..1
  float yaw_rate;     // -1..1 -> deg/s via YAW_RATE_RANGE_DPS
  uint8_t armed;      // 0/1
  uint16_t checksum;  // XOR over preceding bytes
} __attribute__((packed));

extern volatile rc_packet_t latest_rc;
extern volatile bool rc_updated;

void espnow_init();
uint16_t compute_checksum(const uint8_t* data, size_t len);
