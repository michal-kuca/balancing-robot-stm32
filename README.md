# Self-Balancing Two-Wheeled Robot

An inverted-pendulum robot that balances on two wheels using an IMU, a complementary filter, and a PID controller — built from scratch on an STM32F103 with a 3D-printed chassis.

<!-- Drag & drop robot_clip_small.mp4 here in the GitHub web editor to embed the demo clip -->

**Full demo video:** https://youtube.com/shorts/e7V2TYy4I70

## Overview

A two-wheeled robot is an inverted pendulum: inherently unstable, with an unstable pole at √(g/l) ≈ 8 s⁻¹ for this chassis geometry — left alone, any tilt error grows e-fold roughly every 0.12 s. The firmware closes the loop fast enough to catch it: it reads the MPU-6050 over I²C every 5 ms, fuses the accelerometer and gyroscope into a tilt-angle estimate with a complementary filter, runs a PID controller on the angle error, and drives two DC motors through a DRV8833 H-bridge using slow-decay PWM.

The robot balances indefinitely and recovers from small pushes.

## Hardware

| Component | Part | Role |
|---|---|---|
| MCU board | STM32F103 (Nucleo-F103RB pinout) | Runs the control loop |
| IMU | MPU-6050 | 3-axis accelerometer + 3-axis gyroscope, I²C |
| Motor driver | DRV8833 | Dual H-bridge, slow-decay PWM drive |
| Motors | 2× geared DC motors with encoders | Drive wheels (encoders wired, odometry planned) |
| Power | 2S 18650 pack (7.4 V nominal) | Motor supply |
| Chassis | 3D printed | Two-level frame, battery on top for a higher center of mass |

Wiring diagram: [`docs/wiring_diagram.png`](docs/wiring_diagram.png) (Fritzing source: [`docs/wiring.fzz`](docs/wiring.fzz))

## Control loop

Every 5 ms (200 Hz — comfortably above the ~100 Hz minimum the pendulum dynamics demand):

1. **Read** raw accelerometer and gyroscope data from the MPU-6050 over I²C.
2. **Fuse** into a tilt angle with a complementary filter:
   `angle = 0.98 · (angle + gyro·dt) + 0.02 · accel_angle`
   — the gyro provides fast, drift-prone dynamics; the accelerometer provides slow, noisy but drift-free correction.
3. **Filter** the gyro rate with a software low-pass (`gyro_f = 0.7·gyro_f + 0.3·gyro`) before it feeds the D-term, breaking a motor-vibration → gyro-noise → D-term feedback loop (see the debug log, entry 13).
4. **PID** on the angle error, with the integral state clamped for anti-windup and the output clamped to the valid PWM range.
5. **Drive** the motors in slow-decay mode: one bridge input held high, PWM = `period − duty` on the other. Slow decay was chosen after fast decay left a dead zone of ~60 % duty; it recovered nearly the full torque range (minimum effective duty dropped from 60 to 10 out of 99) and gives braking at standstill for free.
6. **Safety cutoff** past ±45° of tilt — beyond recovery, motors stop.

## Repository structure

```
├── Core/                  # Application code (main.c: control loop, filter, PID, motor drive)
├── Drivers/               # STM32 HAL and CMSIS (vendor code)
├── cmake/                 # Toolchain files
├── docs/
│   ├── DEBUGGING.md       # Full debug log — every fault, root cause, and fix
│   ├── wiring_diagram.png # Fritzing wiring diagram
│   └── wiring.fzz         # Fritzing source
├── CMakeLists.txt
└── robot_blink.ioc        # STM32CubeMX configuration
```

## Building and flashing

Requires the ARM GCC toolchain and STM32CubeCLT (build tested with CubeCLT 1.22.0).

```powershell
cmake --preset Debug
cmake --build build/Debug; if ($?) {
  STM32_Programmer_CLI.exe -c port=SWD freq=480 mode=UR reset=HWrst `
    -w build\Debug\robot_blink.elf -v -rst
}
```

The `if ($?)` gate matters: it prevents flashing a stale `.elf` when the build fails. Telemetry (angle, gyro, PID terms) prints over the ST-Link virtual COM port; a version marker in the print string confirms each flash actually took.

## What went wrong (and got fixed)

This project did not work on the first try, and the record of that is deliberately part of the repo: **[docs/DEBUGGING.md](docs/DEBUGGING.md)** logs 20 distinct faults from destroyed hardware to a one-line PWM bug that made one motor unable to reverse — each with symptom, root cause, fix, and the lesson extracted. Highlights:

- A loose battery wire touched the MCU and killed an entire board — leading to strict live-wire discipline and a battery master disconnect.
- Switching the H-bridge from fast to slow decay inverted the meaning of every PWM register write, spawning three separate bugs before every write site was audited.
- The D-term amplified motor vibration back into the motors through the gyro — a self-sustaining oscillation diagnosed from serial logs showing a stable angle alongside an alternating ±130 °/s gyro reading.

## Planned improvements

- Start the encoder timers (TIM3/TIM4 already configured) and add velocity feedback to stop slow drift-walking.
- Untethered operation from the battery via the board's VIN input.
- Tune with the MPU-6050's hardware DLPF instead of (or alongside) the software gyro filter.

## License

MIT
