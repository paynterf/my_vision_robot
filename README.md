# my_vision_robot

Vision-enhanced two-wheel robot with Teensy 4.1 + Raspberry Pi 5 + OAK-D Lite.

## Project Goals
- Real-time object detection and avoidance using OAK-D Lite
- AprilTag-based charging station homing
- Reliable Pi 5 ↔ Teensy 4.1 communication

## Repository Structure

- `firmware/` → Teensy 4.1 code (Visual Studio + Visual Micro)
- `software/` → Raspberry Pi 5 Python code
- `docs/`     → Schematics, notes, calibration data
- `tools/`    → Build and utility scripts

## Getting Started

### Teensy Firmware
1. Open `my_vision_robot.sln` in Visual Studio
2. Select `teensy_main` project
3. Set board to **Teensy 4.1**
4. Verify & Upload

### Pi 5 Software
(TBD - to be filled after setup)

## Hardware
- Main Controller: Teensy 4.1
- Vision: OAK-D Lite
- IMU: MPU6050
- Encoders: Hall-effect on both wheels
- Host: Raspberry Pi 5