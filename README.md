# LiteWing

A from-scratch quadcopter flight controller for the ESP32, plus a matching
handheld radio transmitter. The drone runs a 1 kHz stabilization loop under
FreeRTOS; the controller streams stick positions over ESP-NOW. Two small
PlatformIO projects, no external flight stack.

## How it works

```
  Transmitter (ESP32)                      Drone (ESP32)
  ┌───────────────────┐    ESP-NOW    ┌────────────────────────┐
  │ 4 analog sticks   │  rc_packet_t  │ espnow_rx              │
  │ + arm switch      │ ────────────▶ │   │                    │
  │ espnow_tx @ 50 Hz │   2.4 GHz     │   ▼                    │
  └───────────────────┘               │ flight_control_task    │
                                      │   @ 1 kHz, core 1:     │
                                      │   IMU → filter → PID   │
                                      │        → motors        │
                                      │ telemetry_task, core 0 │
                                      └────────────────────────┘
```

The flight controller reads the IMU, fuses gyro and accelerometer with a
complementary filter into a roll/pitch/yaw attitude estimate, runs a PID
controller against the RC setpoints, and writes the four motor PWM outputs —
every millisecond. Control and telemetry are split across the ESP32's two
cores so logging never stalls the loop. If the craft is disarmed (or the radio
link drops), motors are forced to zero.

## Layout

```
litewing/
├── platformio.ini              # drone firmware (env:esp32dev)
├── src/
│   ├── main.cpp                # FreeRTOS tasks: flight control + telemetry
│   ├── imu.{h,cpp}             # IMU sampling over I2C
│   ├── complementary_filter.*  # gyro/accel fusion -> attitude
│   ├── pid.{h,cpp}             # attitude PID -> motor mix
│   ├── motors.{h,cpp}          # PWM ESC output
│   ├── espnow_rx.{h,cpp}       # receives rc_packet_t from the transmitter
│   ├── telemetry.{h,cpp}       # telemetry task (core 0)
│   └── config.h                # pin map + setpoint scaling
└── controller/                 # separate PlatformIO project
    ├── platformio.ini
    └── src/
        ├── main.cpp            # reads sticks, sends rc_packet_t @ 50 Hz
        └── espnow_tx.{h,cpp}   # ESP-NOW transmit
```

## Hardware

Both boards are `esp32dev` targets. Drone pin map (`src/config.h`):

| Function | Pin |
|----------|-----|
| I2C SDA / SCL (IMU) | 21 / 22 |
| Motor 1 PWM (front-left)  | 13 |
| Motor 2 PWM (front-right) | 12 |
| Motor 3 PWM (rear-left)   | 14 |
| Motor 4 PWM (rear-right)  | 27 |
| Loop-timing debug toggle  | 2  |

Transmitter inputs (`controller/src/main.cpp`): throttle/roll/pitch/yaw on
ADC pins 34/35/32/33, arm switch on pin 25 (active-low).

Setpoint scaling: RC stick values are normalized to `-1.0..1.0` and mapped to
±30° roll/pitch and ±180°/s yaw rate.

## Build and flash

Each project is built separately with [PlatformIO](https://platformio.org/).

```bash
# Drone firmware (from repo root)
pio run -t upload

# Transmitter firmware
cd controller
pio run -t upload
```

Serial monitor runs at 115200 baud (`pio device monitor`).

## Notes

- The `DEBUG_PIN` toggles high/low around the control loop body so loop timing
  can be verified on a logic analyzer (e.g. Saleae) — it should show a clean
  1 kHz square wave.
- The flight loop is pinned to core 1 at the highest FreeRTOS priority;
  telemetry runs on core 0 so it cannot jitter the control loop.
- Arm safety: motors only spin when the transmitter's arm switch is engaged;
  otherwise all four outputs are held at zero.
