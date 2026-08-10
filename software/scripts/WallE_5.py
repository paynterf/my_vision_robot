#!/usr/bin/env python3
"""
WallE_5.py – Supervisory program for the vision-enhanced 4-wheel robot

Features:
  - Does NOT automatically enter Teensy command mode
  - Starts camera pipeline (background)
  - Opens serial link to Teensy and relays telemetry
  - Background WiFi OTA watcher for latest.hex
  - Simple interactive command interface

Location (recommended):
  /home/pi/my_vision_robot/software/scripts/WallE_5.py
"""

import serial
import threading
import time
import subprocess
import sys
import os
from pathlib import Path
from datetime import datetime

# ----------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------
SERIAL_PORT     = "/dev/ttyAMA0"
BAUD            = 115200
CAMERA_SCRIPT   = Path.home() / "my_vision_robot/software/scripts/restart_camera.sh"

# OTA configuration
HEX_FILE_PATH   = Path.home() / "my_vision_robot/firmware/latest.hex"
OTA_POLL_INTERVAL = 1.0          # seconds
MIN_HEX_SIZE    = 50000          # bytes
MAX_AGE_SECONDS = 300            # 5 minutes

# ----------------------------------------------------------------------
# Globals
# ----------------------------------------------------------------------
ser = None
telemetry_running = True

# ----------------------------------------------------------------------
# Utility
# ----------------------------------------------------------------------
def timestamp() -> str:
    return datetime.now().strftime("%H:%M:%S")

def log(msg: str):
    print(f"[{timestamp()}] {msg}")

# ----------------------------------------------------------------------
# Serial helpers
# ----------------------------------------------------------------------
def open_serial(max_attempts: int = 10) -> bool:
    """Try to open the serial port. Returns True on success."""
    global ser
    for attempt in range(1, max_attempts + 1):
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.1)
            time.sleep(0.4)
            ser.reset_input_buffer()
            log(f"Serial opened on {SERIAL_PORT}")
            return True
        except Exception as e:
            log(f"Serial open attempt {attempt}/{max_attempts} failed: {e}")
            time.sleep(2)
    return False

def send_raw(cmd: str):
    """Send a raw string + newline to the Teensy."""
    if ser and ser.is_open:
        ser.write((cmd.strip() + "\n").encode())
        log(f"TX → {cmd.strip()}")

def send_turn(direction: str, degrees: float, rate: float = None):
    """
    Helper for navigation code.
    direction : 'L' or 'R'
    degrees   : positive float
    rate      : optional deg/sec (Teensy uses default if omitted)
    """
    direction = direction.upper()
    if direction not in ("L", "R"):
        print("direction must be 'L' or 'R'")
        return

    if rate is None:
        cmd = f"{direction}{degrees:.2f}"
    else:
        cmd = f"{direction}{degrees:.2f},{rate:.1f}"
    send_raw(cmd)

# ----------------------------------------------------------------------
# Telemetry relay
# ----------------------------------------------------------------------
def telemetry_loop():
    global telemetry_running
    while True:
        if telemetry_running and ser and ser.is_open and ser.in_waiting:
            try:
                line = ser.readline().decode(errors="ignore").strip()
                if line:
                    print(f"[Teensy] {line}")
            except Exception as e:
                print(f"[telemetry] {e}")
        time.sleep(0.02)

# ----------------------------------------------------------------------
# Camera
# ----------------------------------------------------------------------
def start_camera():
    log("Starting camera pipeline...")
    try:
        subprocess.Popen(
            ["bash", str(CAMERA_SCRIPT)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True
        )
        time.sleep(2)
        log("Camera start requested (running in background)")
    except Exception as e:
        log(f"Camera start failed: {e}")

# ----------------------------------------------------------------------
# OTA support
# ----------------------------------------------------------------------
def is_fresh_hex_file(path: Path) -> bool:
    if not path.exists():
        return False
    try:
        size = path.stat().st_size
        if size < MIN_HEX_SIZE:
            return False
        age = time.time() - path.stat().st_mtime
        if age > MAX_AGE_SECONDS:
            return False
        return True
    except Exception:
        return False

def perform_ota_upload(hex_path: Path) -> bool:
    """Perform the OTA transfer. Caller must have already closed the main serial port."""
    log("=== OTA Upload Started ===")
    log(f"File: {hex_path}")

    try:
        ota_ser = serial.Serial(SERIAL_PORT, BAUD, timeout=3)
    except Exception as e:
        log(f"Cannot open serial for OTA: {e}")
        return False

    try:
        log("Sending 'U' trigger...")
        ota_ser.write(b'U')
        time.sleep(1.0)

        log("Waiting for Teensy prompt...")
        start = time.time()
        got_prompt = False
        while time.time() - start < 10:
            if ota_ser.in_waiting:
                line = ota_ser.readline().decode('utf-8', errors='replace').strip()
                print(f"[Teensy] {line}")
                if "reading hex lines" in line.lower():
                    log("✓ Got prompt")
                    got_prompt = True
                    break
            time.sleep(0.2)

        if not got_prompt:
            log("✗ Timed out waiting for prompt")
            return False

        # Send the hex file
        log("Sending .hex file...")
        line_count = 0
        with open(hex_path, 'r') as f:
            for line_count, line in enumerate(f, 1):
                ota_ser.write(line.encode('utf-8'))
                if line_count % 200 == 0:
                    time.sleep(0.005)

        log(f"Sent {line_count} lines. Sending EOF...")
        ota_ser.write(b':00000001FF\r\n')
        ota_ser.flush()
        time.sleep(1.0)

        # Wait for line-count confirmation
        log("Waiting for line count prompt...")
        start = time.time()
        while time.time() - start < 15:
            if ota_ser.in_waiting:
                line = ota_ser.readline().decode('utf-8', errors='replace').strip()
                print(f"[Teensy] {line}")
                if "enter" in line.lower() and "flash" in line.lower():
                    parts = line.split()
                    if len(parts) > 1 and parts[1].isdigit():
                        num = parts[1]
                        log(f"Sending line count: {num}")
                        ota_ser.write((num + '\r\n').encode('utf-8'))
                        ota_ser.flush()
                        break
            time.sleep(0.3)

        log("✅ OTA UPDATE SUCCESSFUL!")
        log("Waiting for Teensy reboot...")
        time.sleep(12)
        return True

    except Exception as e:
        log(f"OTA error: {e}")
        return False
    finally:
        ota_ser.close()

def ota_watcher_loop():
    """Background thread that watches for a new latest.hex and triggers OTA."""
    global ser, telemetry_running

    last_mtime = 0.0
    if HEX_FILE_PATH.exists():
        last_mtime = HEX_FILE_PATH.stat().st_mtime
        log(f"OTA watcher: initial latest.hex found "
            f"(size {HEX_FILE_PATH.stat().st_size} bytes)")

    log(f"OTA watcher started – monitoring {HEX_FILE_PATH}")

    while True:
        try:
            if is_fresh_hex_file(HEX_FILE_PATH):
                current_mtime = HEX_FILE_PATH.stat().st_mtime
                if current_mtime > last_mtime + 0.5:
                    log("=== New latest.hex detected! ===")
                    last_mtime = current_mtime

                    # Pause telemetry and release the serial port
                    telemetry_running = False
                    time.sleep(0.3)
                    if ser and ser.is_open:
                        ser.close()
                        log("Serial closed for OTA")

                    # Perform the upload
                    success = perform_ota_upload(HEX_FILE_PATH)

                    # Re-open serial and restart telemetry
                    time.sleep(1.0)
                    if open_serial(max_attempts=6):
                        telemetry_running = True
                        log("Serial re-opened and telemetry restarted")
                    else:
                        log("WARNING: Could not re-open serial after OTA")

                    if success:
                        log("OTA cycle complete – Teensy should be running new firmware\n")
        except Exception as e:
            log(f"OTA watcher error: {e}")

        time.sleep(OTA_POLL_INTERVAL)

# ----------------------------------------------------------------------
# Interactive help
# ----------------------------------------------------------------------
def print_help():
    print("""
Available commands:
  C / cmd      Enter Teensy command mode
  5 / stop     Stop motors
  Lxx.xx       Turn left  xx.xx degrees   (once in command mode)
  Rxx.xx       Turn right xx.xx degrees
  nav          (placeholder) start navigation
  help         Show this help
  quit / exit  Exit WallE_5
""")

# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------
def main():
    global telemetry_running

    print("\n========================================")
    print("       WallE_5 Supervisor Starting")
    print("========================================\n")

    # 1. Serial
    if not open_serial():
        log("Could not open serial port. Exiting.")
        sys.exit(1)

    # 2. Camera
    start_camera()

    # 3. Telemetry thread
    t = threading.Thread(target=telemetry_loop, daemon=True)
    t.start()
    log("Telemetry relay started")

    # 4. OTA watcher thread
    ota_thread = threading.Thread(target=ota_watcher_loop, daemon=True)
    ota_thread.start()

    log("Ready – robot is idle (NOT in command mode)")
    print("Type 'help' for commands, or 'C' when you want to enable motion.\n")

    # Give any lingering camera messages time to finish, then show a clean prompt
    time.sleep(2.5)
    print("> ", end="", flush=True)

    # 5. Interactive loop
    while True:
        try:
            user = input().strip()          # no prompt string – we control it ourselves
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not user:
            print("> ", end="", flush=True)
            continue

        low = user.lower()

        if low in ("quit", "exit", "q"):
            break
        elif low in ("help", "h", "?"):
            print_help()
        elif low in ("c", "cmd"):
            send_raw("C")
            log("Sent 'C' – Teensy should now be in command mode")
        elif low in ("5", "stop"):
            send_raw("5")
        elif low == "nav":
            log("'nav' not yet implemented")
        elif user[0].upper() in ("L", "R"):
            send_raw(user.upper())
        else:
            send_raw(user)

        print("> ", end="", flush=True)

    # Cleanup
    log("Shutting down WallE_5...")
    telemetry_running = False
    time.sleep(0.3)
    if ser and ser.is_open:
        ser.close()
    log("Done.")

if __name__ == "__main__":
    main()