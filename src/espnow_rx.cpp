#include "espnow_rx.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <string.h>

volatile rc_packet_t latest_rc = {};
volatile bool rc_updated = false;

uint16_t compute_checksum(const uint8_t* data, size_t len) {
  uint16_t csum = 0;
  for (size_t i = 0; i < len; i++) csum ^= data[i];
  return csum;
}

static void espnow_rx_callback(const uint8_t* mac, const uint8_t* data, int len) {
  (void)mac;
  if (len != sizeof(rc_packet_t)) return;

  rc_packet_t pkt;
  memcpy(&pkt, data, sizeof(rc_packet_t));

  uint16_t computed = compute_checksum((uint8_t*)&pkt, sizeof(rc_packet_t) - 2);
  if (computed != pkt.checksum) return;  // drop corrupt packet

  memcpy((void*)&latest_rc, &pkt, sizeof(rc_packet_t));
  rc_updated = true;
}

void espnow_init() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) return;
  esp_now_register_recv_cb(espnow_rx_callback);
}
