#!/usr/bin/env python3
"""
Wifi_OTA.py - Automatic OTA uploader for Teensy 4.1
Watches for latest.hex and triggers OTA via Serial1

Standardized path (Aug 2026):
  /home/pi/my_vision_robot/firmware/latest.hex
"""

import serial
import time
import os
import sys
from datetime import datetime
import signal

# Configuration
HEX_FILE_PATH = "/home/pi/my_vision_robot/firmware/latest.hex"
POLL_INTERVAL = 1.0          # seconds
MIN_HEX_SIZE = 50000         # bytes
MAX_AGE_SECONDS = 300        # 5 minutes
SERIAL_PORT = "/dev/ttyAMA0"
BAUD = 115200


def log(msg):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{timestamp}] {msg}")


def is_fresh_hex_file(path):
    if not os.path.exists(path):
        return False
    try:
        size = os.path.getsize(path)
        if size < MIN_HEX_SIZE:
            log(f"File too small ({size} bytes)")
            return False

        mtime = os.path.getmtime(path)
        age = time.time() - mtime
        if age > MAX_AGE_SECONDS:
            return False

        return True
    except Exception as e:
        log(f"Error checking file: {e}")
        return False


def ota_upload(hex_file_path):
    if not os.path.exists(hex_file_path):
        log(f"Error: Hex file not found: {hex_file_path}")
        return False

    log(f"\n=== OTA Upload Started at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ===")
    log(f"File: {hex_file_path}\n")

    try:
        ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUD, timeout=3)
    except serial.SerialException as e:
        log(f"Cannot open {SERIAL_PORT}: {e}")
        log("Is clearest_direction_node (or another process) holding the port?")
        log("Stop the vision node first, then the watcher will retry on the next cycle.")
        return False

    try:
        log("Sending 'U' trigger...")
        ser.write(b'U')
        time.sleep(1.0)

        log("Waiting for Teensy prompt...")
        start = time.time()
        while time.time() - start < 10:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                log(f"Teensy: {line}")
                if "reading hex lines" in line.lower():
                    log("✓ Got prompt")
                    break
            time.sleep(0.2)
        else:
            log("✗ Timed out waiting for prompt")
            return False

        # Send file
        log("Sending .hex file...")
        line_count = 0
        with open(hex_file_path, 'r') as f:
            for line_count, line in enumerate(f, 1):
                ser.write(line.encode('utf-8'))
                if line_count % 200 == 0:
                    time.sleep(0.005)

        log(f"Sent {line_count} lines. Sending EOF...")
        ser.write(b':00000001FF\r\n')
        ser.flush()
        time.sleep(1.0)

        # Line count confirmation
        log("Waiting for line count prompt...")
        start = time.time()
        while time.time() - start < 15:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                log(f"Teensy: {line}")
                if "enter" in line.lower() and "flash" in line.lower():
                    parts = line.split()
                    if len(parts) > 1 and parts[1].isdigit():
                        num = parts[1]
                        log(f"Sending line count: {num}")
                        ser.write((num + '\r\n').encode('utf-8'))
                        ser.flush()
                        break
            time.sleep(0.3)

        log("\n✅ OTA UPDATE SUCCESSFUL!")
        log("   Flash process started.")
        log("   Waiting for Teensy reboot...\n")

        time.sleep(12)
        log("Upload process finished.\n")
        return True

    except Exception as e:
        log(f"Error during OTA: {e}")
        return False
    finally:
        ser.close()


def main():
    log("Wifi_OTA watcher started")
    log(f"Watching: {HEX_FILE_PATH}")
    last_mtime = 0.0

    if os.path.exists(HEX_FILE_PATH):
        last_mtime = os.path.getmtime(HEX_FILE_PATH)
        log(f"Initial latest.hex found (size: {os.path.getsize(HEX_FILE_PATH)} bytes)")

    while True:
        try:
            if is_fresh_hex_file(HEX_FILE_PATH):
                current_mtime = os.path.getmtime(HEX_FILE_PATH)
                if current_mtime > last_mtime + 0.5:   # small tolerance
                    log("=== New latest.hex detected! Starting OTA ===")
                    last_mtime = current_mtime
                    ota_upload(HEX_FILE_PATH)
        except Exception as e:
            log(f"Watcher error: {e}")

        time.sleep(POLL_INTERVAL)


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, lambda s, f: sys.exit(0))
    main()