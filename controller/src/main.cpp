#include "espnow_tx.h"

#include <Arduino.h>

// Stick / trim ADC pins — adjust to your hardware.
#define PIN_THROTTLE  34
#define PIN_ROLL      35
#define PIN_PITCH     32
#define PIN_YAW       33
#define PIN_ARM_SW    25

static float read_axis_normalized(int pin) {
  // ESP32 ADC: 0..4095 -> -1..1
  int raw = analogRead(pin);
  return (raw - 2048) / 2048.0f;
}

static float read_throttle_normalized(int pin) {
  int raw = analogRead(pin);
  return raw / 4095.0f;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_ARM_SW, INPUT_PULLUP);
  espnow_tx_init();
}

void loop() {
  rc_packet_t pkt = {};
  pkt.throttle = read_throttle_normalized(PIN_THROTTLE);
  pkt.roll     = read_axis_normalized(PIN_ROLL);
  pkt.pitch    = read_axis_normalized(PIN_PITCH);
  pkt.yaw_rate = read_axis_normalized(PIN_YAW);
  pkt.armed    = digitalRead(PIN_ARM_SW) == LOW ? 1 : 0;

  espnow_tx_send(pkt);
  delay(20);  // ~50 Hz
}
