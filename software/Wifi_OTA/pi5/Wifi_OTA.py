#!/usr/bin/env python3
"""
Wifi_OTA_Demo.py - Final version for ongoing vision-enhanced robot project
Last updated: 2026-06-24
"""

import serial
import time
import sys
import os
from datetime import datetime

def ota_upload(hex_file_path):
    if not os.path.exists(hex_file_path):
        print(f"Error: Hex file not found: {hex_file_path}")
        return False

    print(f"\n=== OTA Upload Started at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ===")
    print(f"File: {hex_file_path}\n")

    ser = serial.Serial(port='/dev/ttyAMA0', baudrate=115200, timeout=3)

    try:
        print("Sending 'U' trigger...")
        ser.write(b'U')
        time.sleep(1.0)

        print("Waiting for Teensy prompt...")
        start = time.time()
        while time.time() - start < 10:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                print(f"Teensy: {line}")
                if "reading hex lines" in line.lower():
                    print("✓ Got prompt")
                    break
            time.sleep(0.2)
        else:
            print("✗ Timed out waiting for prompt")
            return False

        # Send file
        print("Sending .hex file...")
        line_count = 0
        with open(hex_file_path, 'r') as f:
            for line_count, line in enumerate(f, 1):
                ser.write(line.encode('utf-8'))
                if line_count % 200 == 0:      # Fixed: was line_num
                    time.sleep(0.005)

        print(f"Sent {line_count} lines. Sending EOF...")
        ser.write(b':00000001FF\r\n')
        ser.flush()
        time.sleep(1.0)

        # Line count confirmation
        print("Waiting for line count prompt...")
        start = time.time()
        while time.time() - start < 15:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                print(f"Teensy: {line}")
                if "enter" in line.lower() and "flash" in line.lower():
                    parts = line.split()
                    if len(parts) > 1 and parts[1].isdigit():
                        num = parts[1]
                        print(f"Sending line count: {num}")
                        ser.write((num + '\r\n').encode('utf-8'))
                        ser.flush()
                        break
            time.sleep(0.3)

        print("\n✅ OTA UPDATE SUCCESSFUL!")
        print("   Flash process started.")
        print("   Waiting for Teensy reboot and Serial1 re-initialization...\n")
        
        time.sleep(12)                    # Important for reliable reboot
        print("Upload process finished. You can now check Serial1 output.\n")
        return True

    except Exception as e:
        print(f"Error during OTA: {e}")
        return False
    finally:
        ser.close()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 Wifi_OTA_Demo.py <full_path_to_.hex>")
        sys.exit(1)
    
    ota_upload(sys.argv[1])