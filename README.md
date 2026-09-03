# PuzzleBot Nonholonomic Control — Examen 4.2

Practical exam project for the robot control module: five trajectory-tracking
controllers for a differential-drive robot (PuzzleBot), a physics simulator
with terrain disturbances, a live web dashboard with 12 real-time charts, and
the ESP32/micro-ROS firmware to run the same pipeline on the real robot.

## Repository layout

```
examen4.2_control/
├── puzzlebot_control/     ROS 2 (ament_python) package — simulator, 5 controllers,
│                           terrain perturbations, live dashboard, teleop, benchmark.
│                           See puzzlebot_control/README.md for full usage details.
├── firmware/               ESP32 + micro-ROS firmware that drives the real
│                           PuzzleBot's motors and reports encoder feedback.
├── docs/dashboard_screenshots/  Example runs captured from the live dashboard.
├── LICENSE                 Apache-2.0
└── README.md                This file
```

## What it does

`puzzlebot_control` implements a common `BaseController` (odometry, goal /
multi-waypoint trajectory tracking, Lyapunov and sliding-surface publishing)
and five interchangeable controllers built on top of it:

| Controller | Idea | Sliding surface |
|---|---|---|
| **PID** | Classic proportional-integral-derivative on distance/heading error | — |
| **SMC** | Sliding Mode Control | s_v = e_d, s_w = θ_e |
| **ISMC** | Integral Sliding Mode Control (no reaching phase) | σ = e + α∫e |
| **CTC** | Computed Torque Control (feedback-linearized dynamics) | — |
| **Port-Hamiltonian** | Energy-shaping IDA-PBC (Ferguson, Donaire, Renton & Middleton, 2018 / Gimenez, Rosales & Carelli, 2015) | — |

A kinematic/dynamic simulator (`puzzlebot_sim`) integrates the nonholonomic
constraint under a configurable terrain-perturbation generator, and a
zero-dependency live dashboard (`dashboard`, plain HTTP + canvas, no
external JS) plots Lyapunov convergence, phase portraits, sliding surfaces,
control effort and disturbances in real time — see example captures in
[`docs/dashboard_screenshots/`](docs/dashboard_screenshots/). All five
controllers can be swapped at runtime from the keyboard teleop or the
dashboard, and a Lyapunov benchmark script runs all of them back-to-back for
comparison.

`firmware/` is the hardware counterpart: ESP32 code that receives wheel
velocity setpoints and reports encoder feedback over micro-ROS, so the exact
same `/cmd_vel` pipeline can drive the physical PuzzleBot instead of the
simulator.

## Quick start (simulation, no real robot required)

```bash
# 1. Install prerequisites (see puzzlebot_control/README.md for the full list)
sudo apt install -y ros-${ROS_DISTRO}-tf-transformations
pip3 install tf-transformations numpy matplotlib

# 2. Build
mkdir -p ~/pb_ws/src && cp -r puzzlebot_control ~/pb_ws/src/
cd ~/pb_ws && colcon build --packages-select puzzlebot_control
source install/setup.bash

# 3. Run (3 terminals)
ros2 launch puzzlebot_control sim.launch.py       # or gazebo.launch.py for full Gazebo/RViz
ros2 run puzzlebot_control teleop_keyboard         # drive it, switch controllers with 1-5
xdg-open http://localhost:8080                     # live dashboard
```

Full options (Gazebo vs. lightweight simulator, custom trajectories, keyboard
shortcuts, dashboard chart reference, standalone no-ROS benchmark) are
documented in [`puzzlebot_control/README.md`](puzzlebot_control/README.md).

## Running on the real robot

See [`firmware/README.md`](firmware/README.md) for flashing the ESP32,
pinout, and starting the micro-ROS agent — once connected it exposes the
same `joint_states` / `motor_speeds` topics the ROS 2 side expects, so no
controller code changes are needed to move from simulation to hardware.

## Team 5: RJ CREW

- José Eduardo Sánchez Martínez
- Josue Ureña Valencia
- Cesar Arellano
- Rafael Gamiz
