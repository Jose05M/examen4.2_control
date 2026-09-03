# ESP32 Motor Driver Firmware (micro-ROS)

Firmware for the ESP32 that drives the two real PuzzleBot DC gearmotors and
closes the loop with ROS 2 via **micro-ROS**. It is the hardware counterpart
to the simulation in [`../puzzlebot_control`](../puzzlebot_control) — it lets
the same `/cmd_vel`-driven pipeline run on the physical robot instead of the
Gazebo/kinematic simulator.

## What it does

- Runs a micro-ROS node (`motor_control`) that:
  - Subscribes to `joint_states` (`sensor_msgs/JointState`) for per-wheel
    velocity setpoints (rad/s), normalizes them against the motor's max
    speed, and drives each wheel's H-bridge (direction pins + PWM duty).
  - Reads each wheel's quadrature encoder via interrupt, computes filtered
    RPM (EMA filter), converts to rad/s, and publishes it on `motor_speeds`
    (`sensor_msgs/JointState`) so the ROS 2 side can close the loop with real
    odometry.
  - Implements the micro-ROS agent connection state machine
    (`WAITING_AGENT → AGENT_AVAILABLE → AGENT_CONNECTED → AGENT_DISCONNECTED`)
    so it reconnects automatically if the agent drops.

## Hardware pinout (as wired in this firmware)

| Function | Left motor | Right motor |
|---|---|---|
| PWM | GPIO 27 | GPIO 4 |
| Direction (IN1/IN2) | GPIO 25 / 26 | GPIO 33 / 32 |
| Encoder A/B | GPIO 14 / 13 | GPIO 16 / 17 |

Encoder resolution: 495 pulses/rev, geared motor max ≈134 RPM (≈14 rad/s).

## Flashing

1. Open `esp32_motor_driver.ino` in the Arduino IDE (or `arduino-cli`) with
   the ESP32 board package and the `micro_ros_arduino` library installed.
2. Select your ESP32 board/port and upload.
3. On the ROS 2 machine, run the micro-ROS agent over serial:
   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
   ```
4. Once connected, the ESP32 exposes `joint_states` (subscriber) and
   `motor_speeds` (publisher) — the same topics `puzzlebot_control`'s
   `base_controller.py` expects for closed-loop feedback on real hardware.
