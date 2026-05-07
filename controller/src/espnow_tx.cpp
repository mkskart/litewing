#include "espnow_tx.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <string.h>

uint8_t DRONE_MAC[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

uint16_t compute_checksum(const uint8_t* data, size_t len) {
  uint16_t csum = 0;
  for (size_t i = 0; i < len; i++) csum ^= data[i];
  return csum;
}

void espnow_tx_init() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) return;

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, DRONE_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (!esp_now_is_peer_exist(DRONE_MAC)) {
    esp_now_add_peer(&peer);
  }
}

void espnow_tx_send(const rc_packet_t& pkt_in) {
  rc_packet_t pkt = pkt_in;
  pkt.checksum = compute_checksum((const uint8_t*)&pkt,
                                  sizeof(rc_packet_t) - 2);
  esp_now_send(DRONE_MAC, (const uint8_t*)&pkt, sizeof(rc_packet_t));
}
