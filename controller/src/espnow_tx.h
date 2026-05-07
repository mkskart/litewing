#pragma once

#include <stdint.h>
#include <stddef.h>

struct rc_packet_t {
  float throttle;
  float roll;
  float pitch;
  float yaw_rate;
  uint8_t armed;
  uint16_t checksum;
} __attribute__((packed));

// MAC address of the drone ESP32 — replace with your drone's actual MAC.
extern uint8_t DRONE_MAC[6];

void espnow_tx_init();
void espnow_tx_send(const rc_packet_t& pkt_in);
uint16_t compute_checksum(const uint8_t* data, size_t len);
