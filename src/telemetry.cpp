#include "telemetry.h"
#include "espnow_rx.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void telemetry_task(void* params) {
  (void)params;
  const TickType_t period = pdMS_TO_TICKS(100);  // 10 Hz
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    Serial.printf("rc armed=%u thr=%.2f r=%.2f p=%.2f y=%.2f\n",
                  latest_rc.armed,
                  latest_rc.throttle,
                  latest_rc.roll,
                  latest_rc.pitch,
                  latest_rc.yaw_rate);
    vTaskDelayUntil(&lastWakeTime, period);
  }
}
