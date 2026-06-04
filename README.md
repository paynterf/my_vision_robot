# my_vision_robot

Vision-enhanced two-wheel robot project using Teensy 4.1 + Raspberry Pi 5 + OAK-D Lite.

## Overview

This project implements real-time object detection/avoidance and eventual autonomous navigation/charging using vision.

## Repository Structure

- `firmware/teensy_main/` — Main Teensy 4.1 firmware
- `software/` — Raspberry Pi 5 code (Python)
- `docs/` — Schematics, notes, calibration data

## OTA Updates

This project uses the shared **[Teensy_OTA_Base](https://github.com/paynterf/Teensy_OTA_Base)** repository for Over-The-Air updates.

Symlinks are used to connect:
- `board.txt`
- `TeensyOTA1.ttl`

Run `mklink_ota.py` from `Teensy_OTA_Base\tools\` whenever you create a new Teensy project that needs OTA support.

## Getting Started

### Teensy Firmware
1. Open `my_vision_robot.sln` in Visual Studio + Visual Micro
2. Select **Teensy 4.1** as the board
3. Build and upload

### Pi 5 Software
(TBD - Vision + Navigation code coming soon)

## Related Repositories

- **[Teensy_OTA_Base](https://github.com/paynterf/Teensy_OTA_Base)** — Shared OTA infrastructure and tools