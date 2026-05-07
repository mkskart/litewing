#include "config.h"
#include "imu.h"
#include "complementary_filter.h"
#include "pid.h"
#include "motors.h"
#include "espnow_rx.h"
#include "telemetry.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static void flight_control_task(void* params) {
  (void)params;
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1);  // 1ms / 1kHz

  while (true) {
    digitalWrite(DEBUG_PIN, HIGH);

    imu_data_t imu = imu_read();
    attitude_t attitude = complementary_filter(imu);

    rc_setpoints_t sp;
    if (latest_rc.armed) {
      sp.throttle = latest_rc.throttle;
      sp.roll     = latest_rc.roll     * ROLL_PITCH_RANGE_DEG;
      sp.pitch    = latest_rc.pitch    * ROLL_PITCH_RANGE_DEG;
      sp.yaw_rate = latest_rc.yaw_rate * YAW_RATE_RANGE_DPS;
    } else {
      sp = {0, 0, 0, 0};
    }

    motor_commands_t commands = pid_update(attitude, sp);

    if (latest_rc.armed) {
      motor_write(commands);
    } else {
      motor_commands_t off = {0, 0, 0, 0};
      motor_write(off);
    }

    digitalWrite(DEBUG_PIN, LOW);

    vTaskDelayUntil(&lastWakeTime, period);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(DEBUG_PIN, OUTPUT);

  imu_init();
  pid_init();
  motors_init();
  espnow_init();

  xTaskCreatePinnedToCore(
    flight_control_task,
    "FlightControl",
    4096,
    NULL,
    configMAX_PRIORITIES - 1,
    NULL,
    1  // core 1
  );

  xTaskCreatePinnedToCore(
    telemetry_task,
    "Telemetry",
    2048,
    NULL,
    1,
    NULL,
    0  // core 0
  );
}

void loop() {
  // Work happens in pinned tasks.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
