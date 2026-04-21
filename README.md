# MicroMouse — Autonomous Maze-Solving Robot

An autonomous robot that navigates and solves an unknown maze from start to center
using real-time wall sensing, Flood-Fill BFS pathfinding, and PD motor control.

Built for the IEEE-style MicroMouse competition at Birzeit University.
Course: ENCS4380 — Interfacing Techniques | Academic Year 2025–2026
Supervised by: Dr. Wasel Ghanem
Team: Aya Fares, Nagham Massis, Alzahra Nassif, Marah Hamarsheh

---

## How It Works

The robot starts at corner (0,0), senses walls in three directions using LiDAR,
updates its internal maze map, runs Flood-Fill to compute distances to the center,
then moves to the best adjacent cell. This cycle repeats until it reaches the goal.

---

## Hardware

| Component | Role |
|-----------|------|
| ESP32-WROOM-32 | Main microcontroller — dual-core 240MHz, WiFi, dual I2C |
| VL53L0X x3 (Front/Left/Right) | Time-of-Flight LiDAR wall detection |
| MPU6050 IMU | Gyroscope for straight-line correction and 90° turns |
| DC Gear Motors x2 + Encoders | Drive system with tick-based cell counting |
| L298N Motor Driver | PWM H-bridge control |
| Li-ion 9V + DC-DC Boost | Power system |

---

## Software Architecture

### Flood-Fill BFS
- 8x8 grid with Manhattan distance initialization to four center goal cells
- `floodFill()` propagates consistent distance values after each wall update
- `selectDirection()` picks the neighbor with the lowest flood value,
  breaking ties by exploration potential (unknown score)

### PD Motor Control
- Gyro PD loop: corrects heading drift every 5ms using yaw integration
- Wall PD loop: reads side LiDAR every 30ms and centers between walls
- Danger zone override: if either side reads below 20mm, a hard push correction fires
- `gradualStop()`: 8-step deceleration ramp with gyro still active during slowdown

### Turn System
- 90° turns driven by encoder tick count (target: 85 ticks per wheel)
- `straightenYaw()` corrects residual heading error before each turn
- `fullCenter()` laterally centers the robot between walls before turning
- Balanced tick control during turn: if one wheel leads by 5+ ticks, it slows to 70% PWM

### Initialization Sequence
1. Motor + encoder setup
2. I2C buses + XSHUT address cycling for 3 LiDARs (0x30/0x31/0x32)
3. MPU6050 init + 800-sample gyro bias calibration
4. Boundary walls pre-loaded into maze grid
5. Flood-Fill initialized + first decision made before motors start

---

## Simulation

Algorithm tested using the [MMS MicroMouse Simulator](https://github.com/mackorone/mms)
before deploying to hardware.

To run in MMS:
1. Clone this repo
2. Open MMS and point it to `micromouse_8x8_v4.ino` (adapted for simulator I/O)

---

## Key Results

- Flood-Fill navigated all test mazes without getting stuck
- PD controller maintained straight-line accuracy within ±2mm
- Gyroscope achieved consistent 90° turns within ±1°
- WiFi dashboard reduced PID tuning time from hours to minutes

---

## Lessons Learned

- Always add voltage protection before connecting the MCU (we burned one ESP32
  to back-EMF spikes from the motor driver)
- Unit test every component before system integration
- Sensor fusion (gyro + walls) significantly outperforms single-sensor appro
